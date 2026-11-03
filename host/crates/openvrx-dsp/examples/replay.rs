// SPDX-License-Identifier: GPL-3.0-or-later
//! Times the receiver chain on a real recording.
//!
//! ```text
//! cargo run --release -p openvrx-dsp --example replay -- NAME.openvrx.json [demod] [--loops N] [--gaps] [--chunk BYTES] [--dump FILE] [--serial]
//! ```
//!
//! Reads the sidecar for the sample format and pushes receiver 0's raw
//! bytes through `RxDsp::push_bytes` in `--chunk` byte transfers (32 KB by
//! default, the USB block size), `--loops` times over (default 3). With
//! `--gaps` the recorded USB gaps (from the events file) are declared with
//! `RxDsp::mark_gap` as the live app does; otherwise the chunks are pushed
//! back to back. Prints the time per sample, the stage breakdown and the
//! receiver statistics. `--dump FILE` writes every line (frame line,
//! flags, 0H position, then the samples as f32 and the damage bytes) for
//! regression comparisons; `--loops` is then 1. `--serial` runs the back
//! half of the chain on the calling thread (no pipeline thread).

use openvrx_core::SampleFormat;
use openvrx_dsp::{DemodKind, RxDsp, RxDspConfig, STAGES};
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let side = args
        .get(1)
        .expect("usage: replay NAME.openvrx.json [polar|lut|ls|pll|ekf] [--loops N] [--gaps] [--chunk BYTES] [--dump FILE] [--serial]");
    let flag = |name: &str| args.iter().position(|a| a == name);
    let value = |name: &str, default: usize| {
        flag(name)
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse().ok())
            .unwrap_or(default)
    };
    let dump_path = flag("--dump").and_then(|i| args.get(i + 1)).cloned();
    let loops = if dump_path.is_some() { 1 } else { value("--loops", 3).max(1) };
    let mut dump: Vec<u8> = Vec::new();
    let mut keep = |lines: &[openvrx_core::LineRecord]| {
        if dump_path.is_none() {
            return;
        }
        for l in lines {
            dump.extend_from_slice(&(l.samples.len() as u32).to_le_bytes());
            dump.extend_from_slice(&(l.frame_line as u32).to_le_bytes());
            dump.extend_from_slice(&l.quality.flags.0.to_le_bytes());
            dump.extend_from_slice(&l.src_position.to_le_bytes());
            for v in &l.samples {
                dump.extend_from_slice(&v.to_le_bytes());
            }
            dump.extend_from_slice(&l.damage);
        }
    };
    let chunk = value("--chunk", 32768).max(1);
    let with_gaps = flag("--gaps").is_some();
    let text = std::fs::read_to_string(side).expect("read sidecar");
    let v: serde_json::Value = serde_json::from_str(&text).expect("sidecar json");
    let rx = &v["receivers"][0];
    let fmt: SampleFormat = serde_json::from_value(rx["sample_format"].clone()).expect("sample_format");
    let dir = std::path::Path::new(side).parent().unwrap_or(std::path::Path::new("."));
    let bin = dir.join(rx["file"].as_str().expect("file"));
    let data = std::fs::read(&bin).expect("read bin");
    let demod = match args.get(2).map(|s| s.as_str()) {
        Some("lut") => DemodKind::LutAngle,
        Some("ls") => DemodKind::LeastSquares,
        Some("pll") => DemodKind::Pll,
        Some("ekf") => DemodKind::Ekf,
        _ => RxDspConfig::default().demod,
    };
    // Recorded gaps: (byte offset in the file, lost bytes).
    let mut gaps: Vec<(usize, u64)> = Vec::new();
    if with_gaps {
        let ev = side.replace(".openvrx.json", ".events.jsonl");
        if let Ok(t) = std::fs::read_to_string(&ev) {
            for line in t.lines() {
                let Ok(e) = serde_json::from_str::<serde_json::Value>(line) else { continue };
                if e["kind"] == "usb_gap" && e["rx"] == 0 {
                    if let (Some(b), Some(l)) = (e["byte"].as_u64(), e["lost_bytes"].as_u64()) {
                        gaps.push((b as usize, l));
                    }
                }
            }
        }
        gaps.sort();
    }
    let wb = fmt.lanes.word_bytes as u64;
    let fs = fmt.sample_rate_hz();
    let cfg = RxDspConfig {
        sample_format: fmt.clone(),
        demod,
        pipeline_thread: flag("--serial").is_none(),
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    let mut lines = Vec::new();
    let mut n_lines = 0usize;
    let mut gap_ns = 0u64;
    let t0 = Instant::now();
    for _ in 0..loops {
        let mut pos = 0usize;
        let mut g = 0usize;
        while pos < data.len() {
            // Never let a transfer straddle a recorded gap.
            let next_gap = gaps.get(g).map(|x| x.0).unwrap_or(usize::MAX);
            let end = (pos + chunk).min(data.len()).min(if next_gap > pos { next_gap } else { usize::MAX });
            dsp.push_bytes(&data[pos..end], 0, &mut lines);
            keep(&lines);
            n_lines += lines.len();
            lines.clear();
            pos = end;
            while g < gaps.len() && gaps[g].0 <= pos {
                if gaps[g].0 == pos {
                    let tg = Instant::now();
                    dsp.mark_gap(gaps[g].1 / wb);
                    gap_ns += tg.elapsed().as_nanos() as u64;
                }
                g += 1;
            }
        }
        if with_gaps {
            // The loop seam is a discontinuity too.
            let tg = Instant::now();
            dsp.mark_gap(fs as u64);
            gap_ns += tg.elapsed().as_nanos() as u64;
        }
    }
    let el = t0.elapsed().as_secs_f64();
    if let Some(p) = &dump_path {
        std::fs::write(p, &dump).expect("write dump");
    }
    let samples = loops * data.len() / wb as usize;
    let ns = el / samples as f64 * 1e9;
    let st = dsp.stats();
    println!(
        "{} samples ({} loops) at {:.2} MS/s ({:?}, {}{}): {:.2} ns/sample, {:.1} MS/s sustained, {:.2}x real time, {} lines",
        samples,
        loops,
        fs / 1e6,
        fmt.preset,
        demod.name(),
        if with_gaps { ", gaps marked" } else { "" },
        ns,
        1e3 / ns,
        (samples as f64 / fs) / el,
        n_lines
    );
    let parts: Vec<String> = STAGES.iter().zip(&st.stage_ns).map(|(n, v)| format!("{n} {v:.2}")).collect();
    println!("stages ns/sample: {}", parts.join(", "));
    if with_gaps {
        println!("mark_gap (coasting through the gaps): {:.2} ns per pushed sample", gap_ns as f64 / samples as f64);
    }
    println!(
        "lock {:?} std {:?} line {:.1} Hz cn {:.1} dB clip {:.1} % dc ({:.1}, {:.1}) carrier {:.0} Hz relocks {} coasted {}",
        st.lock,
        st.standard,
        st.line_rate_hz,
        st.cn_db,
        st.clip_pct,
        st.i_dc,
        st.q_dc,
        st.carrier_offset_hz,
        st.relocks,
        st.coasted_lines
    );
}
