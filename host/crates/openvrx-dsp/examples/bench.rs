// SPDX-License-Identifier: GPL-3.0-or-later
//! Throughput of the receiver chain on one thread.
//!
//! ```text
//! cargo run --release -p openvrx-dsp --features testsig --example bench
//! ```
//!
//! Generates a synthetic NTSC capture (C/N 15 dB) per sample rate, then
//! times `RxDsp::push_bytes` over it in 16 KB transfers and prints the
//! real-time factor (seconds of signal per second of CPU).

use openvrx_core::{LanePreset, SampleFormat};
use openvrx_dsp::testsig::{SigGen, SigSpec};
use openvrx_dsp::{DemodKind, RxDsp, RxDspConfig};
use std::time::Instant;

/// A configuration change for one benchmark case.
type Tweak<'a> = &'a dyn Fn(&mut RxDspConfig);

fn bench(preset: LanePreset, divider: u32, demod: DemodKind, tweak: &dyn Fn(&mut RxDspConfig), label: &str) {
    let fmt = SampleFormat::from_preset(preset, divider);
    let fs = fmt.sample_rate_hz();
    let spec = SigSpec {
        cn_db: Some(15.0),
        sample_format: fmt.clone(),
        ..SigSpec::default()
    };
    let seconds = 0.25;
    let mut g = SigGen::new(spec);
    let data = g.next((fs * seconds) as usize).bytes;
    let mut cfg = RxDspConfig {
        sample_format: fmt,
        demod,
        ..RxDspConfig::default()
    };
    tweak(&mut cfg);
    let mut dsp = RxDsp::new(0, cfg);
    let mut lines = Vec::new();
    // Warm-up pass (lock), then timed passes.
    for c in data.chunks(16384) {
        dsp.push_bytes(c, 0, &mut lines);
    }
    let reps = 4;
    lines.clear();
    let t0 = Instant::now();
    for _ in 0..reps {
        for c in data.chunks(16384) {
            dsp.push_bytes(c, 0, &mut lines);
            lines.clear();
        }
    }
    let cpu = t0.elapsed().as_secs_f64();
    let st = dsp.stats();
    let parts: Vec<String> = openvrx_dsp::STAGES
        .iter()
        .zip(&st.stage_ns)
        .map(|(n, v)| format!("{n} {v:.1}"))
        .collect();
    println!("    stages ns/sample: {}", parts.join(", "));
    let rt = seconds * reps as f64 / cpu;
    let ns = cpu / (fs * seconds * reps as f64) * 1e9;
    let ghz = clock_ghz();
    let cycles = ns * ghz;
    // Projection to a 4.7 GHz P-core (i7-13700H single-thread turbo).
    let proj = 1e9 / (cycles / 4.7) / fs;
    println!(
        "{:>6.2} MS/s {:<8} {:<6} {:<22} {:5.2}x real time here ({:5.1} ns, {:4.0} cycles/sample at {:.2} GHz), {:5.1}x projected at 4.7 GHz",
        fs / 1e6,
        preset.name(),
        demod.name(),
        label,
        rt,
        ns,
        cycles,
        ghz,
        proj
    );
}

/// Effective clock of this thread right now: a dependent chain of FP
/// multiplies (4 cycles latency each on Intel P- and E-cores alike).
fn clock_ghz() -> f64 {
    let mut best = 0.0f64;
    for _ in 0..3 {
        let t = Instant::now();
        let mut x = std::hint::black_box(1.0f64);
        let y = std::hint::black_box(1.000_000_001f64);
        for _ in 0..50_000_000u64 {
            x *= y;
        }
        std::hint::black_box(x);
        best = best.max(4.0 * 0.05 / t.elapsed().as_secs_f64());
    }
    best
}

fn main() {
    let none: &dyn Fn(&mut RxDspConfig) = &|_| {};
    if std::env::args().any(|a| a == "quick") {
        bench(LanePreset::I7Q7, 18, RxDspConfig::default().demod, none, "default");
        return;
    }
    println!("Full default chain:");
    for (p, d) in [
        (LanePreset::I7Q7, 18),
        (LanePreset::I8Q8, 18),
        (LanePreset::I8Q8, 12),
        (LanePreset::I4Q4Ultra, 9),
        (LanePreset::I8Q8, 9),
        (LanePreset::I4Q4Top, 6),
    ] {
        bench(p, d, RxDspConfig::default().demod, none, "default");
    }
    println!("Demodulators at 13.33 MS/s 7x7:");
    for d in DemodKind::ALL {
        bench(LanePreset::I7Q7, 18, d, none, "default");
    }
    println!("Stage costs at 13.33 MS/s 7x7 (polar), by removal:");
    let cases: [(&str, Tweak); 6] = [
        ("no pipeline thread", &|c| c.pipeline_thread = false),
        ("no channel filter", &|c| {
            c.channel_filter.enabled = false;
        }),
        ("no click repair", &|c| c.clicks.enabled = false),
        ("no video lpf", &|c| c.video_lpf.enabled = false),
        ("no dc/iq", &|c| {
            c.dc.enabled = false;
            c.iq_correction.enabled = false;
        }),
        ("spectrum on", &|c| c.spectrum.enabled = true),
    ];
    for (name, f) in cases {
        bench(LanePreset::I7Q7, 18, DemodKind::Polar, f, name);
    }
}
