// SPDX-License-Identifier: GPL-3.0-or-later
//! Offline tools for OpenVRX recordings.
//!
//! ```text
//! openvrx-cli inspect REC [OUT_DIR]           lane activity, I/Q histograms, a sync-free FM strip image
//! openvrx-cli decode  REC [OUT_DIR] [FRAMES]  frames through the live DSP and video chain (raw preset)
//! ```
//! REC is any file of a recording (`NAME.openvrx.json`, `NAME.rx0.bin`) or its base path.

use openvrx_core::LineRecord;
use openvrx_dsp::{RxDsp, RxDspConfig};
use openvrx_format::Recording;
use openvrx_video::{VideoConfig, VideoPipeline};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Line rate the strip is cut at (NTSC; PAL rows drift slowly sideways).
const LINE_HZ: f64 = 15_734.264;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (cmd, rec) = match (args.first(), args.get(1)) {
        (Some(c), Some(r)) => (c.as_str(), r.as_str()),
        _ => {
            eprintln!("usage: openvrx-cli inspect|decode REC [OUT_DIR] [FRAMES]");
            std::process::exit(2);
        }
    };
    let out = args.get(2).map(PathBuf::from).unwrap_or_else(|| PathBuf::from("openvrx-out"));
    let result = match cmd {
        "inspect" => inspect(rec, &out),
        "decode" => decode(rec, &out, args.get(3).and_then(|s| s.parse().ok()).unwrap_or(8)),
        _ => Err(format!("unknown command {cmd}")),
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn read_start(rec: &Recording, max_bytes: u64) -> Result<Vec<u8>, String> {
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let n = r.len_bytes().min(max_bytes) as usize;
    let mut buf = vec![0u8; n];
    let got = r.read(&mut buf).map_err(|e| e.to_string())?;
    buf.truncate(got);
    Ok(buf)
}

fn words(data: &[u8], word_bytes: usize) -> impl Iterator<Item = u32> + '_ {
    data.chunks_exact(word_bytes).map(move |c| match word_bytes {
        1 => c[0] as u32,
        2 => u16::from_le_bytes([c[0], c[1]]) as u32,
        _ => u32::from_le_bytes([c[0], c[1], c[2], c[3]]),
    })
}

fn inspect(path: &str, out: &Path) -> Result<(), String> {
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let lanes = info.sample_format.lanes.clone();
    let wb = lanes.word_bytes as usize;
    let fs = info.sample_rate_hz;
    println!(
        "{}: {:?} {:.2} MS/s, {:.2} s, channel {} MHz, gain {}",
        rec.name(),
        info.sample_format.preset,
        fs / 1e6,
        rec.duration_s(),
        info.channel_mhz,
        info.gain
    );
    let data = read_start(&rec, (fs * wb as f64 * 2.0) as u64)?;
    let n = (data.len() / wb).max(1);

    // Lane activity: share of samples with each word bit set.
    let bits = wb * 8;
    let mut ones = vec![0u64; bits];
    for w in words(&data, wb) {
        for (b, o) in ones.iter_mut().enumerate() {
            *o += ((w >> b) & 1) as u64;
        }
    }
    println!("word bit duty (0 % or 100 % = stuck lane):");
    for b in (0..bits).rev() {
        print!(" b{b}:{:.0}%", 100.0 * ones[b] as f64 / n as f64);
    }
    println!();

    // Histograms of the decoded components.
    let (mut hi, mut hq) = (BTreeMap::new(), BTreeMap::new());
    let mut mag = 0f64;
    for w in words(&data, wb) {
        let z = lanes.decode(w);
        *hi.entry(z.re as i32).or_insert(0u64) += 1;
        *hq.entry(z.im as i32).or_insert(0u64) += 1;
        mag += (z.re as f64).hypot(z.im as f64);
    }
    let show = |name: &str, h: &BTreeMap<i32, u64>| {
        let s: Vec<String> = h
            .iter()
            .map(|(v, c)| format!("{v}:{:.1}%", 100.0 * *c as f64 / n as f64))
            .collect();
        println!("{name} values: {}", s.join(" "));
    };
    show("I", &hi);
    show("Q", &hq);
    println!("mean |z| {:.1} (ADC units, full scale 512)", mag / n as f64);

    // Sync-free FM strip: instantaneous frequency, one row per line period.
    let mut f = Vec::with_capacity(n);
    let mut prev = lanes.decode(words(&data, wb).next().unwrap_or(0));
    for w in words(&data, wb) {
        let z = lanes.decode(w);
        let d = z * prev.conj();
        f.push(d.im.atan2(d.re) as f64 * fs / (2.0 * std::f64::consts::PI));
        prev = z;
    }
    let k = 3usize;
    let sm: Vec<f64> = (0..f.len())
        .map(|i| {
            let a = i.saturating_sub(k);
            let b = (i + k + 1).min(f.len());
            f[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect();
    let spl = fs / LINE_HZ;
    let width = spl.round() as usize;
    let rows = ((sm.len() as f64 / spl) as usize).min(1200);
    let (lo, hi_f) = (-6e6, 8e6);
    let mut img = image::GrayImage::new(width as u32, rows as u32);
    for r in 0..rows {
        let start = (r as f64 * spl) as usize;
        for x in 0..width {
            let v = sm.get(start + x).copied().unwrap_or(0.0);
            let g = ((v - lo) / (hi_f - lo)).clamp(0.0, 1.0) * 255.0;
            img.put_pixel(x as u32, r as u32, image::Luma([g as u8]));
        }
    }
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let p = out.join(format!("{}-fmstrip.png", rec.name()));
    img.save(&p).map_err(|e| e.to_string())?;
    let mean_f = f.iter().sum::<f64>() / f.len().max(1) as f64;
    println!(
        "mean instantaneous frequency {:.0} kHz; FM strip {width} x {rows} (dark = low frequency) -> {}",
        mean_f / 1e3,
        p.display()
    );
    Ok(())
}

fn decode(path: &str, out: &Path, frames: usize) -> Result<(), String> {
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let mut pipe: Option<(openvrx_core::VideoStandard, VideoPipeline)> = None;
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut buf = vec![0u8; 1 << 18];
    let mut saved = 0;
    let mut total_lines = 0u64;
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    while saved < frames {
        let n = r.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        lines.clear();
        dsp.push_bytes(&buf[..n], 0, &mut lines);
        for line in lines.drain(..) {
            total_lines += 1;
            let std = line.standard;
            if pipe.as_ref().is_none_or(|(s, _)| *s != std) {
                pipe = Some((std, VideoPipeline::new(&VideoConfig::raw(std))));
            }
            let Some((_, p)) = pipe.as_mut() else { continue };
            for f in p.push_line(line).frames_completed {
                if saved >= frames {
                    break;
                }
                let img = image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8())
                    .ok_or("frame size")?;
                let path = out.join(format!("{}-frame{saved:02}.png", rec.name()));
                img.save(&path).map_err(|e| e.to_string())?;
                println!("saved {}", path.display());
                saved += 1;
            }
        }
    }
    println!("{total_lines} lines from the DSP; stats: {:?}", dsp.stats());
    Ok(())
}
