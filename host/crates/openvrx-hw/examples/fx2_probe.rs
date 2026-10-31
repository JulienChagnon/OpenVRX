// SPDX-License-Identifier: GPL-3.0-or-later
//! Lists FX2 boards and C5 radio heads, uploads the OpenVRX firmware to any
//! blank FX2, then runs a 5 second throughput test on every OpenVRX FX2.
//!
//! Usage: `cargo run -p openvrx-hw --example fx2_probe [-- path/to/openvrx-fx2.ihx] [--pattern]`

use openvrx_hw::c5::find_c5_ports;
use openvrx_hw::fx2::{
    self, firmware, list_fx2, run_bench, BenchConfig, BenchMode, Fx2Kind, Fx2Selector, NusbPort, StreamPort,
};
use openvrx_hw::power::{SelectiveSuspend, TimerResolution};
use std::path::PathBuf;
use std::time::Duration;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let pattern = args.iter().any(|a| a == "--pattern");
    let fw_path = args.iter().find(|a| !a.starts_with("--")).map(PathBuf::from);

    if let Ok(s) = SelectiveSuspend::read() {
        if let Some(h) = s.hint() {
            println!("{h}\n");
        }
    }
    let _timer = TimerResolution::request(1).ok();

    match find_c5_ports() {
        Ok(p) if p.is_empty() => println!("C5: no Espressif USB serial ports"),
        Ok(p) => {
            for c in p {
                println!(
                    "C5: {} (pid {:04x}, serial {})",
                    c.port_name,
                    c.pid,
                    c.serial_number.as_deref().unwrap_or("-")
                );
            }
        }
        Err(e) => println!("C5: cannot list serial ports: {e}"),
    }

    let devices = match list_fx2() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("cannot list USB devices: {e}");
            std::process::exit(1);
        }
    };
    if devices.is_empty() {
        println!("FX2: no devices");
        return;
    }
    for d in &devices {
        println!("FX2: {}", d.describe());
        if let Some(h) = d.driver_hint() {
            println!("     {h}");
        }
    }

    // Upload firmware to blank boards.
    if devices.iter().any(|d| d.kind == Fx2Kind::Bootloader) {
        match firmware::resolve(fw_path.as_deref()) {
            Ok(img) => {
                println!("uploading {} bytes of firmware", img.len());
                match fx2::upload_all(&img) {
                    Ok(results) => {
                        for (target, r) in results {
                            match r {
                                Ok(o) => println!(
                                    "  {} -> {} (firmware {}, board {})",
                                    target.describe(),
                                    o.device.describe(),
                                    o.info.fw_version(),
                                    o.info.serial()
                                ),
                                Err(e) => println!("  {}: upload failed: {e}", target.describe()),
                            }
                        }
                    }
                    Err(e) => println!("upload failed: {e}"),
                }
            }
            Err(e) => println!("cannot upload: {e}"),
        }
    }

    // Throughput test on every OpenVRX board at once.
    let boards: Vec<_> = match list_fx2() {
        Ok(d) => d.into_iter().filter(|d| d.kind == Fx2Kind::Openvrx).collect(),
        Err(e) => {
            eprintln!("cannot list USB devices: {e}");
            std::process::exit(1);
        }
    };
    if boards.is_empty() {
        println!("no OpenVRX FX2 to benchmark");
        return;
    }
    let ports: Vec<(String, Box<dyn StreamPort>)> = boards
        .iter()
        .map(|b| {
            let sel = match &b.serial {
                Some(s) => Fx2Selector::Serial(s.clone()),
                None => Fx2Selector::Location {
                    bus_id: b.location.bus_id.clone(),
                    port_chain: b.location.port_chain.clone(),
                },
            };
            (b.describe(), Box::new(NusbPort::new(sel)) as Box<dyn StreamPort>)
        })
        .collect();
    let cfg = BenchConfig {
        mode: if pattern {
            BenchMode::TestPattern
        } else {
            BenchMode::Internal48
        },
        duration: Duration::from_secs(5),
        ..Default::default()
    };
    println!(
        "running a {} s {:?} throughput test on {} board(s)",
        cfg.duration.as_secs(),
        cfg.mode,
        ports.len()
    );
    match run_bench(ports, &cfg) {
        Ok(results) => {
            for r in results {
                println!("{}", r.summary());
            }
        }
        Err(e) => println!("bench failed: {e}"),
    }
}
