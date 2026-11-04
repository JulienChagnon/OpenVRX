// SPDX-License-Identifier: GPL-3.0-or-later
//! Slave FIFO bring-up diagnostic: starts the first OpenVRX FX2 in several
//! clock, width, strobe-polarity and commit modes, reads the bulk stream for
//! a moment and prints what arrived next to the firmware status block.
//!
//! What the rows tell apart:
//! - data with SLWR active low, internal clock: FIFO and USB path good.
//! - data only with SLWR active high: RDY1 is not seen low (wire not on
//!   RDY1, poor contact, or the tie went to another pin).
//! - watch rows commit packets but AUTOIN rows deliver nothing: the FIFO
//!   fills but AUTOIN never commits (firmware configuration).
//! - nothing with the external clock but data with the internal one: no
//!   IFCLK from the C5 (clock wire, C5 not running, wrong pin).
//!
//! Usage: `cargo run --release -p openvrx-hw --example fx2_diag [-- --live]`
//! (`--live` prints the control ties and PB lanes every 0.5 s for 60 s,
//! for re-seating wires while watching; `--find-ifclk` drives 48 MHz out
//! of IFCLK so a jumper from a header pin to PD1 shows whether it is IFCLK).

use openvrx_hw::fx2::{list_fx2, Fx2Kind, Fx2Requests, InEndpoint, StartFlags};
use std::time::{Duration, Instant};

const SET_OPTION: u8 = 0xB7;
const OPT_SLWR_HIGH: u16 = 2;
const GET_PINS: u8 = 0xBD;
const OPT_IFCLK_OUT: u16 = 5;

fn level(or: u8, and: u8, bit: u8) -> &'static str {
    match ((or >> bit) & 1, (and >> bit) & 1) {
        (1, 1) => "high",
        (0, 0) => "low",
        _ => "TOGGLING",
    }
}

/// Prints the 0xBD readback: control ties with their expected level, and
/// the data lanes (noise from the C5 should make them toggle).
fn print_pins(p: &[u8]) {
    let (a_or, a_and, b_or, b_and, d_or, d_and, r_or, r_and) = (p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
    println!("control ties (pin: seen, wanted):");
    for (name, or, and, bit, want) in [
        ("RDY1 / SLWR", r_or, r_and, 1, "low"),
        ("RDY0 / SLRD", r_or, r_and, 0, "high"),
        ("PA4 / FIFOADR0", a_or, a_and, 4, "low"),
        ("PA5 / FIFOADR1", a_or, a_and, 5, "low"),
        ("PA2 / SLOE", a_or, a_and, 2, "high"),
        ("PA6 / PKTEND", a_or, a_and, 6, "high"),
    ] {
        let seen = level(or, and, bit);
        let mark = if seen == want { "ok" } else { "<-- CHECK THIS WIRE" };
        println!("  {name:15} {seen:8} want {want:5} {mark}");
    }
    let lanes = |name: &str, or: u8, and: u8| {
        let v: Vec<String> = (0..8).rev().map(|b| format!("{}{}:{}", name, b, level(or, and, b))).collect();
        println!("  {}", v.join(" "));
    };
    let pa: Vec<String> = (0..8).rev().map(|b| format!("PA{}:{}", b, level(a_or, a_and, b))).collect();
    println!("  port A: {}", pa.join(" "));
    println!("data pins (lanes in use should toggle with C5 noise):");
    lanes("PB", b_or, b_and);
    lanes("PD", d_or, d_and);
    println!(
        "  IFCONFIG {:02x} EP2FIFOCFG {:02x} EP24FIFOFLGS {:02x} EP2FIFOBC {} EP2CS {:02x} FIFOPINPOLAR {:02x} OEA {:02x}",
        p[8],
        p[9],
        p[10],
        ((p[11] as u16) << 8) | p[12] as u16,
        p[13],
        p[14],
        p[15]
    );
}

fn main() {
    // A replugged FX2 is blank (firmware lives in RAM): load it first.
    if list_fx2().is_ok_and(|d| d.iter().any(|d| d.kind == Fx2Kind::Bootloader)) {
        match openvrx_hw::fx2::firmware::embedded() {
            Ok(fw) => match openvrx_hw::fx2::upload_all(&fw) {
                Ok(r) => println!("loaded OpenVRX firmware on {} board(s)", r.len()),
                Err(e) => println!("firmware upload failed: {e}"),
            },
            Err(e) => println!("no embedded firmware: {e}"),
        }
    }
    let found = match list_fx2() {
        Ok(d) => d.into_iter().find(|d| d.kind == Fx2Kind::Openvrx),
        Err(e) => {
            eprintln!("cannot list USB devices: {e}");
            std::process::exit(1);
        }
    };
    let Some(found) = found else {
        println!("no OpenVRX FX2 (run fx2_probe first to load the firmware)");
        return;
    };
    let dev = match found.open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("open: {e}");
            std::process::exit(1);
        }
    };
    let ctl = dev.control();
    println!("{}", found.describe());

    let _ = ctl.stop();
    match ctl.vendor_in(GET_PINS, 0, 0, 16) {
        Ok(p) if p.len() >= 16 => print_pins(&p),
        Ok(_) | Err(_) => println!("pin readback not supported (replug the FX2 so fx2_probe loads the new firmware)"),
    }

    if std::env::args().any(|a| a == "--find-ifclk") {
        // The FX2 drives 48 MHz out of its IFCLK pin; a jumper from the
        // header pin under test to PD1 shows `~` on PD1 when it is IFCLK.
        // Nothing else may drive that pin meanwhile (C5 clock wire off D10).
        if let Err(e) = ctl.vendor_out(SET_OPTION, OPT_IFCLK_OUT, 1, &[]) {
            println!("IFCLK output not supported ({e}); replug the FX2 so the new firmware loads");
            return;
        }
        // The output clock comes from the CPU's own oscillator, so samples
        // alias to one level; instead the output is switched on and off and
        // PD1 must follow it (floating high when off, a fixed level or
        // toggling when on).
        println!("switching the IFCLK output on and off; jumper the header pin under test to PD1:");
        let read_pd1 = || ctl.vendor_in(GET_PINS, 0, 0, 16).ok().map(|p| level(p[4], p[5], 1));
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(90) {
            let _ = ctl.vendor_out(SET_OPTION, OPT_IFCLK_OUT, 1, &[]);
            std::thread::sleep(Duration::from_millis(150));
            let on = read_pd1().unwrap_or("?");
            let _ = ctl.vendor_out(SET_OPTION, OPT_IFCLK_OUT, 0, &[]);
            std::thread::sleep(Duration::from_millis(150));
            let off = read_pd1().unwrap_or("?");
            let verdict = if on != off {
                "<-- PD1 follows the IFCLK output: this pin IS IFCLK"
            } else {
                "(no change: not IFCLK, or no jumper yet)"
            };
            println!("{:5.1}s  PD1 with output on {on:8} off {off:8} {verdict}", t0.elapsed().as_secs_f64());
            std::thread::sleep(Duration::from_millis(200));
        }
        let _ = ctl.vendor_out(SET_OPTION, OPT_IFCLK_OUT, 0, &[]);
        return;
    }

    if std::env::args().any(|a| a == "--live") {
        // Live pin view for re-seating wires: one line every 0.5 s for 60 s.
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(60) {
            if let Ok(p) = ctl.vendor_in(GET_PINS, 0, 0, 16) {
                let (a_or, a_and, b_or, b_and, r_or, r_and) = (p[0], p[1], p[2], p[3], p[6], p[7]);
                let short = |or: u8, and: u8, bit: u8| match level(or, and, bit) {
                    "high" => "H",
                    "low" => "L",
                    _ => "~",
                };
                let pb: String = (0..8).rev().map(|b| short(b_or, b_and, b)).collect();
                println!(
                    "{:5.1}s  RDY1 {} (L)  PA4 {} (L)  PA5 {} (L)  RDY0 {} (H)  PA2 {} (H)  PA6 {} (H)  PB7..0 {}  (~ = toggling)",
                    t0.elapsed().as_secs_f64(),
                    short(r_or, r_and, 1),
                    short(a_or, a_and, 4),
                    short(a_or, a_and, 5),
                    short(r_or, r_and, 0),
                    short(a_or, a_and, 2),
                    short(a_or, a_and, 6),
                    pb
                );
            }
            std::thread::sleep(Duration::from_millis(500));
        }
        return;
    }

    let int48 = StartFlags::INTERNAL_CLK | StartFlags::INTERNAL_48;
    let cases: [(&str, StartFlags, bool, u16); 8] = [
        ("internal 48 MHz, 16-bit, SLWR low", int48 | StartFlags::WIDTH16, false, 0),
        ("internal 48 MHz, 8-bit, SLWR low", int48, false, 0),
        ("internal 48 MHz, 8-bit, SLWR HIGH", int48, true, 0),
        ("internal 48 MHz, 8-bit, SLWR low, watch 1/2", int48 | StartFlags::WATCH, false, 2),
        ("internal 48 MHz, 8-bit, SLWR HIGH, watch 1/2", int48 | StartFlags::WATCH, true, 2),
        ("C5 clock, 8-bit, SLWR low", StartFlags::empty(), false, 0),
        ("C5 clock inverted, 8-bit, SLWR low", StartFlags::IFCLK_INV, false, 0),
        ("C5 clock, 8-bit, SLWR low, watch 1/2", StartFlags::WATCH, false, 2),
    ];
    let c5_first = std::env::args().any(|a| a == "--c5");
    let mut cases = cases.to_vec();
    if c5_first {
        // Only the C5-clock cases, the plain one first.
        cases.retain(|c| !c.1.contains(StartFlags::INTERNAL_CLK));
    }
    for (name, flags, slwr_high, watch) in cases {
        let _ = ctl.stop();
        if let Err(e) = ctl.vendor_out(SET_OPTION, OPT_SLWR_HIGH, slwr_high as u16, &[]) {
            println!("{name}: SET_OPTION failed: {e}");
            continue;
        }
        let _ = ctl.reset_counters();
        let mut ep = match dev.stream_endpoint() {
            Ok(e) => e,
            Err(e) => {
                println!("{name}: endpoint: {e}");
                continue;
            }
        };
        let _ = ep.clear_halt();
        if let Err(e) = ctl.start(flags, watch) {
            println!("{name}: START failed: {e}");
            continue;
        }
        for _ in 0..8 {
            ep.submit(vec![0u8; 64 * 1024]);
        }
        let t0 = Instant::now();
        let mut bytes = 0usize;
        let mut sample: Vec<u8> = Vec::new();
        while t0.elapsed() < Duration::from_millis(700) {
            if let Some(c) = ep.wait_complete(Duration::from_millis(100)) {
                if c.status.is_ok() {
                    let n = c.data.len();
                    bytes += n;
                    if sample.is_empty() && n >= 16 {
                        sample = c.data[..16].to_vec();
                    }
                }
                ep.submit(vec![0u8; 64 * 1024]);
            }
        }
        let secs = t0.elapsed().as_secs_f64();
        // FIFO registers while running: byte count moving means IFCLK edges
        // are writing; NPAK (EP2CS bits 6:4) counts packets handed to USB.
        let mut fifo = Vec::new();
        for _ in 0..3 {
            if let Ok(p) = ctl.vendor_in(GET_PINS, 0, 0, 16) {
                fifo.push(format!(
                    "IFC {:02x} FLG {:02x} BC {:4} CS {:02x}",
                    p[8],
                    p[10],
                    ((p[11] as u16) << 8) | p[12] as u16,
                    p[13]
                ));
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        let st = ctl.get_status();
        let _ = ctl.stop();
        ep.cancel_all();
        while ep.pending() > 0 && ep.wait_complete(Duration::from_millis(200)).is_some() {}
        let rate = bytes as f64 / secs / 1e6;
        match st {
            Ok(s) => println!(
                "{name:46} {rate:7.2} MB/s  committed {:6}  skipped {:6}  fifo_full {:6}  flags {:?}  first bytes {:02x?}",
                s.packets_committed, s.packets_skipped, s.fifo_full, s.flags, sample
            ),
            Err(e) => println!("{name:46} {rate:7.2} MB/s  status error: {e}"),
        }
        println!("      running: {}", fifo.join(" | "));
    }
    let _ = ctl.vendor_out(SET_OPTION, OPT_SLWR_HIGH, 0, &[]);
}
