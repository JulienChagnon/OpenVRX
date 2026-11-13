// SPDX-License-Identifier: GPL-3.0-or-later
//! Offline tools for OpenVRX recordings.
//!
//! ```text
//! openvrx-cli inspect REC [OUT_DIR]           lane activity, I/Q histograms, a sync-free FM strip image
//! openvrx-cli decode  REC [OUT_DIR] [FRAMES] [--steady-osd] [--damage DIR] [--side DIR [--side-only]]
//!                                             frames through the live DSP and video chain (raw preset;
//!                                             --steady-osd: line-doubled fields as in the app's default;
//!                                             --damage: each frame's damage mask as a grey PNG of the
//!                                             same name in DIR, for the denoise models;
//!                                             --side: what the receiver knew about each pixel and row,
//!                                             in DIR: the I/Q envelope as a grey PNG per frame (same
//!                                             name and geometry as the picture), rows.f32 (per frame
//!                                             row: C/N, noise, sync score, timing error, clicks, line
//!                                             flags) and side.json describing both; with --side-only
//!                                             no pictures or masks are written, only DIR)
//! openvrx-cli decode  REC OUT_DIR FRAMES --live [--preset NAME] [--no-steady-osd]
//!                                             the app's live video path instead: preset (default
//!                                             low_latency), streaming, every line through the combiner
//!                                             with host times from the byte position; saves displayed
//!                                             fields (FRAMES 0: none) and prints lost lines once a second
//!                                             and the combiner/assembler/repair counters (DSP-5)
//! openvrx-cli record  OUT_BASE --lanes P --clk D [--seconds S] [--mhz M] [--gain G]
//!                                             FX2-only capture, no window (bench scripts)
//! openvrx-cli score   REC [--ref REC] [--frames N] [--skip K] [--demod polar|lut|ls|pll|ekf]
//!                     [--preset raw|low_latency|balanced|quality] [--ref-demod D]
//!                                             one JSON line: lock, C/N, clipping, frames, PSNR vs ref,
//!                                             DSP load and stage_ns (defaults: polar, raw; the
//!                                             reference goes through the same chain)
//! openvrx-cli timeline REC [--window S] [--demod polar|lut|ls|pll|ekf]
//!                                             the whole recording in S-second rows (default 0.5):
//!                                             lock, C/N, clipping, relocks, damaged share, USB gaps
//! openvrx-cli render  REC OUT.mp4 [--filter FILE | --no-filter] [--start S] [--seconds N]
//!                     [--crf Q] [--x264-preset P]
//!                                             post-flight video: the raw decode through the learned
//!                                             denoise filter (default: the built-in 4x4top filter,
//!                                             tools/denoise), one frame per field, x264 via ffmpeg
//!                                             (on PATH); defaults crf 16, preset medium
//! ```
//! REC is any file of a recording (`NAME.openvrx.json`, `NAME.rx0.bin`) or its base path.

mod denoise;

use denoise::{DenoiseFilter, Planes};
use openvrx_core::{Frame, LanePreset, LineRecord};
use openvrx_dsp::{RxDsp, RxDspConfig, RxStats};
use openvrx_format::{ReceiverInfo, Recording, RecordingSource, RecordingWriter, Sidecar, WriterOptions};
use openvrx_hw::c5::{C5Settings, PortSelector};
use openvrx_hw::fx2::{self, list_fx2, Fx2Kind, Fx2Selector};
use openvrx_hw::receiver::{Receiver, ReceiverConfig};
use openvrx_video::{DeinterlaceMethod, VideoConfig, VideoPipeline};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Line rate the strip is cut at (NTSC; PAL rows drift slowly sideways).
const LINE_HZ: f64 = 15_734.264;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (cmd, rec) = match (args.first(), args.get(1)) {
        (Some(c), Some(r)) => (c.as_str(), r.as_str()),
        _ => {
            eprintln!("usage: openvrx-cli inspect|decode|record|score|timeline|render ... (see the source header)");
            std::process::exit(2);
        }
    };
    let out = args.get(2).map(PathBuf::from).unwrap_or_else(|| PathBuf::from("openvrx-out"));
    let result = match cmd {
        "inspect" => inspect(rec, &out),
        "decode" if args.iter().any(|a| a == "--live") => decode_live(
            rec,
            &out,
            args.get(3).and_then(|s| s.parse().ok()).unwrap_or(8),
            opt(&args, "--preset").unwrap_or("low_latency"),
            !args.iter().any(|a| a == "--no-steady-osd"),
        ),
        "decode" => decode(
            rec,
            &out,
            args.get(3).and_then(|s| s.parse().ok()).unwrap_or(8),
            args.iter().any(|a| a == "--steady-osd"),
            opt(&args, "--damage").map(PathBuf::from),
            opt(&args, "--side").map(PathBuf::from),
            args.iter().any(|a| a == "--side-only"),
        ),
        "record" => record(rec, &args[2..]),
        "score" => score(rec, &args[2..]),
        "timeline" => timeline(rec, &args[2..]),
        "render" => render(rec, &args[2..]),
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

/// Runs a recording through the live DSP and the raw video chain, handing
/// each completed frame to `on_frame` until it returns false or the data
/// ends. Returns the DSP line count and its final stats.
///
/// `preset` names the video cleanup (`VideoConfig::preset`); `raw` is the
/// historical scoring chain. Other presets keep their colour, repair,
/// deinterlace and denoise stages but still bypass the combiner (offline its
/// waits run on host time) and always return finished frames.
fn walk_frames(
    rec: &Recording,
    steady_osd: bool,
    demod: openvrx_dsp::DemodKind,
    preset: &str,
    mut on_frame: impl FnMut(&Frame) -> Result<bool, String>,
) -> Result<(u64, RxStats), String> {
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        demod,
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let mut pipe: Option<(openvrx_core::VideoStandard, VideoPipeline)> = None;
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut buf = vec![0u8; 1 << 18];
    let mut total_lines = 0u64;
    'read: loop {
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
                let mut cfg = VideoConfig::preset(preset, std).ok_or(format!("unknown preset {preset}"))?;
                cfg.combiner.mode = openvrx_video::CombineMode::Blend;
                cfg.emit_frames = true;
                if steady_osd {
                    cfg.deinterlace = DeinterlaceMethod::LineDouble;
                }
                pipe = Some((std, VideoPipeline::new(&cfg)));
            }
            let Some((_, p)) = pipe.as_mut() else { continue };
            // One receiver: bypass the combiner, whose waits run on host time.
            for f in p.push_combined_line(&line).frames_completed {
                if !on_frame(&f)? {
                    break 'read;
                }
            }
        }
    }
    Ok((total_lines, dsp.stats()))
}

/// Envelope range of the `decode --side` plane: dB relative to the mean
/// |z| of the line, mapped linearly onto grey 0..255 (after the IRE trip
/// through the video chain, see `walk_frames_side`).
const SIDE_ENV_DB: (f32, f32) = (-18.0, 6.0);
/// IRE levels the envelope range is mapped to before the video chain, away
/// from both ends so nothing clips on the way to 0..1 luma.
const SIDE_ENV_IRE: (f32, f32) = (20.0, 80.0);
/// Row features of `decode --side`, in file order (f32 each, per frame row).
const SIDE_ROWS: [&str; 6] = ["cn_db", "noise_ire", "sync_score", "timing_err", "clicks", "flags"];

/// Linear interpolation of `buf` at position `p` (None outside it).
fn lerp_at(buf: &[f32], p: f64) -> Option<f32> {
    if p < 0.0 {
        return None;
    }
    let i = p as usize;
    if i + 1 >= buf.len() {
        return None;
    }
    let f = (p - i as f64) as f32;
    Some(buf[i] + f * (buf[i + 1] - buf[i]))
}

/// `walk_frames` (polar demod, raw preset) that also hands over an envelope
/// picture of each frame. The DSP's video tap gives |z| aligned with the
/// video the sync stage saw; each line's envelope is resampled onto the
/// line's own grid (`src_position`, `src_line_len`), written as dB against
/// the line's mean into a copy of the line record, and sent through a second
/// video pipeline of the same configuration. That pipeline crops and
/// deinterlaces exactly like the picture's, so pixel (x, y) of the envelope
/// frame is the envelope of pixel (x, y) of the picture, with no geometry
/// duplicated here. Returns the lines, the DSP stats, the mean correlation
/// between the resampled tap video and the line samples (a check of the
/// alignment: near 1) and the share of envelope samples the tap did not hold.
fn walk_frames_side(
    rec: &Recording,
    steady_osd: bool,
    mut on_frame: impl FnMut(&Frame, &Frame) -> Result<bool, String>,
) -> Result<(u64, RxStats, f64, f64), String> {
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    dsp.set_demod_tap(Some(openvrx_dsp::TapPoint::Video));
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let mut pipe: Option<(openvrx_core::VideoStandard, VideoPipeline, VideoPipeline)> = None;
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut buf = vec![0u8; 1 << 18];
    let mut total_lines = 0u64;
    // Rolling copy of the tap: mag[i] and vid[i] belong to input sample first + i.
    let (mut first, mut mag, mut vid) = (0u64, Vec::<f32>::new(), Vec::<f32>::new());
    const KEEP: usize = 1 << 22;
    let (mut corr_sum, mut corr_n) = (0.0f64, 0u64);
    let (mut missing, mut wanted) = (0u64, 0u64);
    let (lo_db, hi_db) = SIDE_ENV_DB;
    let (lo_ire, hi_ire) = SIDE_ENV_IRE;
    'read: loop {
        let n = r.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        lines.clear();
        dsp.push_bytes(&buf[..n], 0, &mut lines);
        if let Some(ch) = dsp.demod_tap() {
            if ch.first_sample != first + mag.len() as u64 {
                first = ch.first_sample;
                mag.clear();
                vid.clear();
            }
            mag.extend_from_slice(&ch.mag);
            vid.extend_from_slice(&ch.freq_hz);
        }
        for line in lines.drain(..) {
            total_lines += 1;
            let std = line.standard;
            if pipe.as_ref().is_none_or(|(s, _, _)| *s != std) {
                let mut cfg = VideoConfig::preset("raw", std).ok_or("unknown preset raw")?;
                cfg.combiner.mode = openvrx_video::CombineMode::Blend;
                cfg.emit_frames = true;
                if steady_osd {
                    cfg.deinterlace = DeinterlaceMethod::LineDouble;
                }
                pipe = Some((std, VideoPipeline::new(&cfg), VideoPipeline::new(&cfg)));
            }
            let Some((_, p, pe)) = pipe.as_mut() else { continue };
            let spl = line.samples.len();
            let step = line.src_line_len / spl as f64;
            // Video sample j of the tap represents input time j - 0.5 (as src_position says).
            let p0 = line.src_position + 0.5 - first as f64;
            let mut env = line.clone();
            let e: Vec<Option<f32>> = (0..spl).map(|j| lerp_at(&mag, p0 + j as f64 * step)).collect();
            let have = e.iter().flatten().count();
            wanted += spl as u64;
            missing += (spl - have) as u64;
            let mean = e.iter().flatten().map(|&v| v as f64).sum::<f64>() / have.max(1) as f64;
            for (s, v) in env.samples.iter_mut().zip(&e) {
                let db = match v {
                    Some(v) if mean > 0.0 => 20.0 * ((*v as f64 / mean).max(1e-6)).log10() as f32,
                    Some(_) => lo_db,
                    None => 0.0,
                };
                *s = lo_ire + (db.clamp(lo_db, hi_db) - lo_db) / (hi_db - lo_db) * (hi_ire - lo_ire);
            }
            // Alignment check on every 16th line: the tap's video, resampled the same way, against
            // the line the DSP made from it (correlation: the level and gain do not matter).
            if total_lines % 16 == 0 && have == spl {
                let (mut sx, mut sy, mut sxx, mut syy, mut sxy) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
                for (j, &y) in line.samples.iter().enumerate() {
                    let x = lerp_at(&vid, p0 + j as f64 * step).unwrap_or(0.0) as f64;
                    let y = y as f64;
                    sx += x;
                    sy += y;
                    sxx += x * x;
                    syy += y * y;
                    sxy += x * y;
                }
                let nn = spl as f64;
                let den = ((sxx - sx * sx / nn) * (syy - sy * sy / nn)).sqrt();
                if den > 0.0 {
                    corr_sum += (sxy - sx * sy / nn) / den;
                    corr_n += 1;
                }
            }
            // One receiver: bypass the combiner, whose waits run on host time.
            let done = p.push_combined_line(&line).frames_completed;
            let done_env = pe.push_combined_line(&env).frames_completed;
            if done.len() != done_env.len() {
                return Err("the envelope pipeline fell out of step with the picture's".into());
            }
            for (f, fe) in done.iter().zip(&done_env) {
                if !on_frame(f, fe)? {
                    break 'read;
                }
            }
        }
        if mag.len() > 2 * KEEP {
            let cut = mag.len() - KEEP;
            mag.drain(..cut);
            vid.drain(..cut);
            first += cut as u64;
        }
    }
    let corr = if corr_n > 0 { corr_sum / corr_n as f64 } else { f64::NAN };
    Ok((total_lines, dsp.stats(), corr, missing as f64 / wanted.max(1) as f64))
}

/// `decode --side DIR`: pictures (and masks) as `decode` writes them, plus the
/// side data in DIR (see the file header). `out` None (`--side-only`): the
/// side data alone, for recordings whose pictures are already decoded.
fn decode_side(
    rec: &Recording,
    out: Option<&Path>,
    frames: usize,
    steady_osd: bool,
    damage: Option<PathBuf>,
    side: &Path,
) -> Result<(), String> {
    use std::io::Write;
    std::fs::create_dir_all(side).map_err(|e| e.to_string())?;
    let mut rows = std::io::BufWriter::new(std::fs::File::create(side.join("rows.f32")).map_err(|e| e.to_string())?);
    let mut saved = 0;
    let mut height = 0;
    let (lo_ire, hi_ire) = SIDE_ENV_IRE;
    let (total_lines, stats, corr, missing) = walk_frames_side(rec, steady_osd, |f, fe| {
        if saved >= frames {
            return Ok(false);
        }
        if (fe.width, fe.height) != (f.width, f.height) {
            return Err("envelope frame size differs from the picture's".into());
        }
        let name = format!("{}-frame{saved:02}.png", rec.name());
        if let Some(out) = out {
            let img = image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8()).ok_or("frame size")?;
            img.save(out.join(&name)).map_err(|e| e.to_string())?;
            if let Some(d) = &damage {
                let m = image::GrayImage::from_raw(f.width as u32, f.height as u32, f.damage.clone()).ok_or("damage size")?;
                m.save(d.join(&name)).map_err(|e| e.to_string())?;
            }
        }
        // Luma 0..1 is IRE / 100: back to 0..255 over the envelope's IRE span.
        let env: Vec<u8> = fe
            .y
            .iter()
            .map(|&y| ((y * 100.0 - lo_ire) / (hi_ire - lo_ire) * 255.0).round().clamp(0.0, 255.0) as u8)
            .collect();
        let e = image::GrayImage::from_raw(f.width as u32, f.height as u32, env).ok_or("envelope size")?;
        e.save(side.join(&name)).map_err(|e| e.to_string())?;
        height = f.height;
        for q in &f.row_quality {
            for v in [q.cn_db, q.noise_ire, q.sync_score, q.timing_err, q.clicks as f32, q.flags.0 as f32] {
                rows.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
            }
        }
        saved += 1;
        Ok(saved < frames)
    })?;
    rows.flush().map_err(|e| e.to_string())?;
    let (lo_db, hi_db) = SIDE_ENV_DB;
    let meta = format!(
        "{{\"format\":\"openvrx-side\",\"version\":1,\"frames\":{saved},\"height\":{height},\
         \"rows_file\":\"rows.f32\",\"rows_dtype\":\"<f4\",\"row_features\":{:?},\
         \"flags\":{{\"FLYWHEEL\":1,\"CLIPPED\":2,\"NO_CARRIER\":4,\"GAIN_STEP\":8,\"SAMPLES_LOST\":16,\
         \"REPAIRED\":32,\"VBI\":64,\"BURST_OK\":128,\"BLENDED\":256,\"RELOCK\":512}},\
         \"envelope\":\"grey PNG per frame: 20*log10(|z| / mean |z| of the line), {lo_db} dB = 0 .. {hi_db} dB = 255\",\
         \"envelope_db\":[{lo_db},{hi_db}],\"alignment_corr\":{corr:.4},\"envelope_missing\":{missing:.6}}}\n",
        SIDE_ROWS
    );
    std::fs::write(side.join("side.json"), meta).map_err(|e| e.to_string())?;
    println!("{saved} frames with side data in {}; alignment correlation {corr:.4}, envelope missing {:.4} %", side.display(), 100.0 * missing);
    println!("{total_lines} lines from the DSP; stats: {stats:?}");
    Ok(())
}

fn decode(
    path: &str,
    out: &Path,
    frames: usize,
    steady_osd: bool,
    damage: Option<PathBuf>,
    side: Option<PathBuf>,
    side_only: bool,
) -> Result<(), String> {
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    if let (Some(s), true) = (&side, side_only) {
        return decode_side(&rec, None, frames, steady_osd, None, s);
    }
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    if let Some(d) = &damage {
        std::fs::create_dir_all(d).map_err(|e| e.to_string())?;
    }
    if let Some(s) = &side {
        return decode_side(&rec, Some(out), frames, steady_osd, damage, s);
    }
    let mut saved = 0;
    let (total_lines, stats) = walk_frames(&rec, steady_osd, openvrx_dsp::DemodKind::Polar, "raw", |f| {
        if saved >= frames {
            return Ok(false);
        }
        let img = image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8()).ok_or("frame size")?;
        let name = format!("{}-frame{saved:02}.png", rec.name());
        let path = out.join(&name);
        img.save(&path).map_err(|e| e.to_string())?;
        if let Some(d) = &damage {
            // The frame's damage byte per pixel (openvrx_core::quality::damage:
            // 0 clean, 64 noisy, 96 repaired click, 160 click, 224 no carrier,
            // 255 lost), same name as the picture, in its own folder so
            // tools that read every PNG of OUT_DIR still see only pictures.
            let m = image::GrayImage::from_raw(f.width as u32, f.height as u32, f.damage.clone()).ok_or("damage size")?;
            m.save(d.join(&name)).map_err(|e| e.to_string())?;
        }
        println!("saved {} (field {} parity {:?})", path.display(), f.meta.field_seq, f.meta.parity);
        saved += 1;
        Ok(saved < frames)
    })?;
    println!("{total_lines} lines from the DSP; stats: {stats:?}");
    Ok(())
}

/// Post-flight video: every frame of the raw decode (as `decode` saves
/// them, which is what the filter was trained on) through the denoise
/// filter and into ffmpeg/x264. The first and last frames of the range
/// lack temporal context and are written unfiltered.
fn render(path: &str, args: &[String]) -> Result<(), String> {
    let out = args.first().filter(|a| !a.starts_with("--")).ok_or("render: REC OUT.mp4")?;
    let filter = if args.iter().any(|a| a == "--no-filter") {
        None
    } else {
        let text = match opt(args, "--filter") {
            Some(f) => std::fs::read_to_string(f).map_err(|e| format!("{f}: {e}"))?,
            None => denoise::DEFAULT_FILTER.to_string(),
        };
        Some(DenoiseFilter::from_json(&text)?)
    };
    let start: f64 = opt_num(args, "--start", 0.0)?;
    let seconds: f64 = opt_num(args, "--seconds", f64::INFINITY)?;
    let crf: u32 = opt_num(args, "--crf", 16)?;
    let x264_preset = opt(args, "--x264-preset").unwrap_or("medium");
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    // The filter's window: `before` frames are written unfiltered first,
    // then each new frame completes a window whose frame `before` is output.
    let (before, window) = filter.as_ref().map_or((0, 1), |f| (f.before(), f.before() + f.after() + 1));

    let mut ffmpeg: Option<(std::process::Child, std::process::ChildStdin)> = None;
    let mut hist: std::collections::VecDeque<Planes> = std::collections::VecDeque::new();
    let mut index = 0usize;
    let written = std::cell::Cell::new(0usize);
    let (mut first, mut last) = (usize::MAX, usize::MAX);
    let t0 = Instant::now();
    let write = |ff: &mut Option<(std::process::Child, std::process::ChildStdin)>, p: &Planes| -> Result<(), String> {
        use std::io::Write;
        let (_, stdin) = ff.as_mut().ok_or("ffmpeg not started")?;
        stdin.write_all(&p.to_rgb8()).map_err(|e| format!("ffmpeg: {e}"))?;
        written.set(written.get() + 1);
        Ok(())
    };
    let (_, stats) = walk_frames(&rec, false, openvrx_dsp::DemodKind::Polar, "raw", |f| {
        let hz = f.meta.standard.map_or(60_000.0 / 1001.0, |s| s.params().field_hz);
        if first == usize::MAX {
            first = (start * hz).round() as usize;
            last = if seconds.is_finite() { first + (seconds * hz).round() as usize } else { usize::MAX };
        }
        index += 1;
        if index - 1 < first {
            return Ok(true);
        }
        if index - 1 >= last {
            return Ok(false);
        }
        if ffmpeg.is_none() {
            let mut child = std::process::Command::new("ffmpeg")
                .args(["-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s"])
                .arg(format!("{}x{}", f.width, f.height))
                .args(["-r", &format!("{hz:.6}"), "-i", "-", "-c:v", "libx264", "-preset", x264_preset, "-crf"])
                .arg(crf.to_string())
                .args(["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
                .arg(out)
                .stdin(std::process::Stdio::piped())
                .spawn()
                .map_err(|e| format!("cannot start ffmpeg (is it on PATH?): {e}"))?;
            let stdin = child.stdin.take().ok_or("ffmpeg stdin")?;
            ffmpeg = Some((child, stdin));
        }
        hist.push_back(Planes::from_rgba8(f.width, f.height, &f.to_rgba8()));
        match &filter {
            None => write(&mut ffmpeg, &hist.pop_front().expect("frame"))?,
            Some(flt) => {
                if hist.len() <= before {
                    write(&mut ffmpeg, &hist[hist.len() - 1])?;
                } else if hist.len() == window {
                    let window: Vec<&Planes> = hist.iter().collect();
                    write(&mut ffmpeg, &flt.apply(&window))?;
                    hist.pop_front();
                }
            }
        }
        let n = written.get();
        if n > 0 && n % 600 == 0 {
            eprintln!("{n} frames ({:.0} s of video) in {:.0} s", n as f64 / hz, t0.elapsed().as_secs_f64());
        }
        Ok(true)
    })?;
    // The last `after` frames of the range have no later context (none for a
    // causal filter: every frame went out as it came in).
    if filter.is_some() {
        for p in hist.iter().skip(before) {
            write(&mut ffmpeg, p)?;
        }
    }
    let Some((mut child, stdin)) = ffmpeg else {
        return Err("no frames decoded in that range".into());
    };
    drop(stdin);
    let status = child.wait().map_err(|e| e.to_string())?;
    if !status.success() {
        return Err(format!("ffmpeg failed: {status}"));
    }
    println!(
        "{} frames -> {out} ({}, {:.0} s); dsp lock {:?}, C/N {:.1} dB",
        written.get(),
        if filter.is_some() { "denoise filter" } else { "no filter" },
        t0.elapsed().as_secs_f64(),
        stats.lock,
        stats.cn_db
    );
    Ok(())
}

/// The app's live video path on a recording (DSP-5): the named preset with
/// streaming on, every line through the combiner (`push_line`, not the
/// bypass) with host times rebuilt from the byte position at the sample
/// rate, 32 KB per block as the FX2 delivers them, and the gaps the
/// recording's events name declared as the app does. A field is saved
/// whenever the display would show a completed one (`current_frame()`
/// after the bottom row updated). Prints the pipeline counters at the end.
fn decode_live(path: &str, out: &Path, frames: usize, preset: &str, steady_osd: bool) -> Result<(), String> {
    const BLOCK: usize = 32 * 1024;
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        ..RxDspConfig::default()
    };
    let word = info.sample_format.lanes.word_bytes.max(1) as u64;
    let ns_per_byte = 1e9 / (info.sample_rate_hz * word as f64);
    let mut dsp = RxDsp::new(0, cfg);
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let mut pipe: Option<(openvrx_core::VideoStandard, VideoPipeline)> = None;
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut buf = vec![0u8; BLOCK];
    let (mut byte, mut saved, mut fields) = (0u64, 0usize, 0u64);
    'read: loop {
        let n = r.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        byte += n as u64;
        // The FX2 stamps a block when it completes: the time of its last byte.
        let host_ns = (byte as f64 * ns_per_byte) as u64 + 1_000_000_000;
        lines.clear();
        dsp.push_bytes(&buf[..n], host_ns, &mut lines);
        for line in lines.drain(..) {
            let std = line.standard;
            if pipe.as_ref().is_none_or(|(s, _)| *s != std) {
                let mut vc = VideoConfig::preset(preset, std).ok_or(format!("unknown preset {preset}"))?;
                vc.combiner.mode = openvrx_video::CombineMode::Blend;
                if steady_osd {
                    vc.deinterlace = DeinterlaceMethod::LineDouble;
                }
                vc.streaming = true;
                vc.emit_frames = false;
                pipe = Some((std, VideoPipeline::new(&vc)));
            }
            let Some((_, p)) = pipe.as_mut() else { continue };
            let o = p.push_line(line);
            let Some(rows) = o.updated_rows else { continue };
            let f = p.current_frame();
            if rows.end + 1 < f.height {
                continue;
            }
            fields += 1;
            // Once a second of signal: lines the display lost per field
            // (assembler missing + late, combiner late), as the app's
            // health check sees them.
            if fields % 60 == 0 {
                let s = p.stats();
                let lost = s.assembler.missing_lines + s.assembler.late_lines + s.combiner.late_lines;
                println!(
                    "health t={:6.2}s lost_total={lost} missing={} asm_late={} comb_late={}",
                    byte as f64 * ns_per_byte * 1e-9,
                    s.assembler.missing_lines,
                    s.assembler.late_lines,
                    s.combiner.late_lines
                );
            }
            if saved < frames {
                let img =
                    image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8()).ok_or("frame size")?;
                let path = out.join(format!("{}-live{saved:04}.png", rec.name()));
                img.save(&path).map_err(|e| e.to_string())?;
                saved += 1;
            } else if frames > 0 {
                break 'read;
            }
        }
    }
    let st = pipe.as_ref().map(|(_, p)| p.stats());
    println!("{fields} displayed fields, {saved} saved; dsp {:?}", dsp.stats().lock);
    if let Some(s) = st {
        println!("combiner {:?}", s.combiner);
        println!("assembler {:?}", s.assembler);
        println!("repair {:?}", s.repair);
    }
    Ok(())
}

/// `--demod` (or another option name) as a demodulator, `default` if absent.
fn parse_demod(args: &[String], name: &str, default: openvrx_dsp::DemodKind) -> Result<openvrx_dsp::DemodKind, String> {
    let Some(v) = opt(args, name) else { return Ok(default) };
    openvrx_dsp::DemodKind::ALL
        .into_iter()
        .find(|d| d.name() == v)
        .ok_or(format!("{name} {v}: polar, lut, ls, pll or ekf"))
}

/// Value of `--name VALUE` in `args`.
fn opt<'a>(args: &'a [String], name: &str) -> Option<&'a str> {
    args.iter().position(|a| a == name).and_then(|k| args.get(k + 1)).map(|s| s.as_str())
}

fn opt_num<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> Result<T, String> {
    match opt(args, name) {
        Some(v) => v.parse().map_err(|_| format!("bad {name} {v}")),
        None => Ok(default),
    }
}

/// FX2-only capture without the app: the C5 streams its saved settings,
/// `--lanes`/`--clk` must match them (as for `openvrx --lanes --clk`).
/// `--mhz` and `--gain` only label the recording.
fn record(base: &str, args: &[String]) -> Result<(), String> {
    let lanes: LanePreset = opt(args, "--lanes")
        .ok_or("record needs --lanes PRESET")?
        .parse()
        .map_err(|e| format!("{e}"))?;
    let divider: u32 = opt_num(args, "--clk", 9)?;
    let seconds: f64 = opt_num(args, "--seconds", 5.0)?;
    let mhz: u32 = opt_num(args, "--mhz", 0)?;
    let gain: u32 = opt_num(args, "--gain", 0)?;

    if list_fx2().map_err(|e| e.to_string())?.iter().any(|f| f.kind == Fx2Kind::Bootloader) {
        let fw = fx2::firmware::embedded().map_err(|e| e.to_string())?;
        for (dev, r) in fx2::upload_all(&fw).map_err(|e| e.to_string())? {
            r.map_err(|e| format!("FX2 firmware upload on {}: {e}", dev.describe()))?;
        }
        // Renumeration after the upload.
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(5)
            && !list_fx2().map_err(|e| e.to_string())?.iter().any(|f| f.kind == Fx2Kind::Openvrx)
        {
            std::thread::sleep(Duration::from_millis(200));
        }
    }
    let settings = C5Settings {
        lanes,
        divider,
        mhz,
        gain,
        ..C5Settings::default()
    };
    let cfg = ReceiverConfig::new(0, PortSelector::Name(String::new()), Fx2Selector::Only, settings.clone());
    let receiver = Receiver::start_fx2_only(cfg).map_err(|e| format!("starting the FX2: {e}"))?;

    let mut info = ReceiverInfo::new(0, settings.sample_format());
    info.channel_mhz = mhz;
    info.gain = gain;
    info.board = settings.board.name().to_string();
    info.start_host_ns = openvrx_core::clock::now_ns();
    let sidecar = Sidecar::new(RecordingSource::Hardware, vec![info]);
    let mut writer = RecordingWriter::create(base, sidecar, WriterOptions::default()).map_err(|e| e.to_string())?;
    let want = (seconds * settings.sample_format().sample_rate_hz()) as u64
        * settings.sample_format().lanes.word_bytes as u64;
    let t0 = Instant::now();
    let mut bytes = 0u64;
    let mut blocks = 0u64;
    while bytes < want {
        match receiver.blocks().recv_timeout(Duration::from_millis(500)) {
            Ok(b) => {
                bytes += writer.write_block(&b).map_err(|e| e.to_string())?;
                blocks += 1;
            }
            Err(_) if t0.elapsed() > Duration::from_secs(3) && bytes == 0 => {
                return Err("no data from the FX2 in 3 s (C5 off, wrong wiring or IFCLK)".into());
            }
            Err(_) => {}
        }
        if t0.elapsed().as_secs_f64() > seconds * 3.0 + 5.0 {
            eprintln!("warning: stream slower than expected, stopping early");
            break;
        }
    }
    drop(receiver);
    let path = writer.paths().sidecar();
    writer.close().map_err(|e| e.to_string())?;
    println!(
        "recorded {} ({:.1} MB in {blocks} blocks, {:.1} s wall)",
        path.display(),
        bytes as f64 / 1e6,
        t0.elapsed().as_secs_f64()
    );
    Ok(())
}

/// Luma planes, seconds read, lines and DSP statistics of a recording.
type Lumas = (Vec<Vec<f32>>, f64, u64, RxStats);

/// Luma of the frames of a recording, after skipping `skip` frames.
fn lumas(
    rec: &Recording,
    skip: usize,
    frames: usize,
    demod: openvrx_dsp::DemodKind,
    preset: &str,
) -> Result<Lumas, String> {
    let mut out = Vec::new();
    let mut seen = 0usize;
    let mut damage = 0u64;
    let mut pixels = 0u64;
    let (lines, stats) = walk_frames(rec, false, demod, preset, |f| {
        seen += 1;
        if seen > skip {
            damage += f.damage.iter().filter(|&&d| d != 0).count() as u64;
            pixels += f.damage.len() as u64;
            out.push(f.y.clone());
        }
        Ok(out.len() < frames)
    })?;
    let damaged = if pixels > 0 { damage as f64 / pixels as f64 } else { 1.0 };
    Ok((out, damaged, lines, stats))
}

/// One JSON line per recording for bench sweeps. With `--ref`, PSNR of
/// each frame's luma against the reference recording's mean luma (a static
/// scene recorded at a strong, clean level), so no known test picture is
/// needed.
///
/// `--demod` picks the FM demodulator and `--preset` the video cleanup
/// (default polar and raw, the historical numbers). The reference goes
/// through the same chain, so PSNR measures the damage noise does to that
/// chain's output; `--ref-demod` decodes the reference with another
/// demodulator instead (to expose a demodulator's systematic distortion).
fn score(path: &str, args: &[String]) -> Result<(), String> {
    let frames: usize = opt_num(args, "--frames", 30)?;
    let skip: usize = opt_num(args, "--skip", 10)?;
    let demod = parse_demod(args, "--demod", openvrx_dsp::DemodKind::Polar)?;
    let ref_demod = parse_demod(args, "--ref-demod", demod)?;
    let preset = opt(args, "--preset").unwrap_or("raw");
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let (ys, damaged, lines, stats) = lumas(&rec, skip, frames, demod, preset)?;

    let mut psnr = Vec::new();
    if let Some(r) = opt(args, "--ref") {
        let reference = Recording::open(r).map_err(|e| e.to_string())?;
        let (ref_ys, _, _, _) = lumas(&reference, skip, frames, ref_demod, preset)?;
        if let Some(first) = ref_ys.first() {
            let size = first.len();
            let mut mean = vec![0f32; size];
            for y in ref_ys.iter().filter(|y| y.len() == size) {
                for (m, v) in mean.iter_mut().zip(y) {
                    *m += v / ref_ys.len() as f32;
                }
            }
            for y in ys.iter().filter(|y| y.len() == mean.len()) {
                let mse: f64 =
                    y.iter().zip(&mean).map(|(a, b)| ((a - b) as f64).powi(2)).sum::<f64>() / y.len() as f64;
                psnr.push(10.0 * (1.0 / mse.max(1e-10)).log10());
            }
        }
    }
    psnr.sort_by(|a, b| a.total_cmp(b));
    let median = psnr.get(psnr.len() / 2).copied();
    let duration = rec.duration_s();
    let fmt_opt = |v: Option<f64>| v.map_or("null".to_string(), |v| format!("{v:.2}"));
    // Per-stage DSP cost (ns per input sample) by stage name.
    let stage_ns: Vec<String> = openvrx_dsp::STAGES
        .iter()
        .zip(&stats.stage_ns)
        .map(|(name, ns)| format!("\"{name}\":{ns:.3}"))
        .collect();
    println!(
        "{{\"rec\":\"{}\",\"seconds\":{duration:.2},\"lines\":{lines},\"line_rate_hz\":{:.1},\
         \"frames\":{},\"damaged\":{damaged:.4},\"lock\":\"{:?}\",\"cn_db\":{:.2},\"clip_pct\":{:.2},\
         \"power_db\":{:.2},\"psnr_median\":{},\"psnr_min\":{},\
         \"demod\":\"{}\",\"ref_demod\":\"{}\",\"preset\":\"{preset}\",\"load\":{:.4},\"stage_ns\":{{{}}}}}",
        rec.name(),
        stats.line_rate_hz,
        ys.len(),
        stats.lock,
        stats.cn_db,
        stats.clip_pct,
        stats.power_db,
        fmt_opt(median),
        fmt_opt(psnr.first().copied()),
        stats.demod.name(),
        ref_demod.name(),
        stats.load,
        stage_ns.join(",")
    );
    Ok(())
}

/// Health of a whole recording over time, one row per `--window` seconds
/// (default 0.5): where a flight lost its picture and why. Time is the
/// position in the raw file (samples recorded), so a USB gap does not shift
/// later rows; `gap_ms` is the radio time lost in gaps that began in the
/// window. The other columns come from the same DSP and video chain as
/// `score`: lock and C/N and clipping as they stand at the window's end,
/// relocks during it, and the damaged share of the frames finished in it.
fn timeline(path: &str, args: &[String]) -> Result<(), String> {
    let window: f64 = opt_num(args, "--window", 0.5)?;
    if !(window > 0.0) {
        return Err("--window must be above 0".into());
    }
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let byte_rate = info.byte_rate();
    let win_bytes = ((byte_rate * window) as usize).max(1);

    let mut gap_ms: BTreeMap<u64, f64> = BTreeMap::new();
    for e in &rec.events().events {
        if let (Some(byte), openvrx_format::EventKind::UsbGap { lost_bytes, .. }) = (e.byte, &e.kind) {
            *gap_ms.entry(byte / win_bytes as u64).or_default() += *lost_bytes as f64 / byte_rate * 1e3;
        }
    }

    let demod = parse_demod(args, "--demod", openvrx_dsp::DemodKind::Polar)?;
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        demod,
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let mut pipe: Option<(openvrx_core::VideoStandard, VideoPipeline)> = None;
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut buf = vec![0u8; win_bytes.min(1 << 18)];
    let mut relocks_before = 0u64;
    println!("    t_s  lock       cn_db  clip%  relocks  damaged%  frames  gap_ms");
    for w in 0u64.. {
        let (mut damaged, mut pixels, mut frames, mut left, mut eof) = (0u64, 0u64, 0u32, win_bytes, false);
        while left > 0 {
            let take = left.min(buf.len());
            let n = r.read(&mut buf[..take]).map_err(|e| e.to_string())?;
            if n == 0 {
                eof = true;
                break;
            }
            left -= n;
            lines.clear();
            dsp.push_bytes(&buf[..n], 0, &mut lines);
            for line in lines.drain(..) {
                let std = line.standard;
                if pipe.as_ref().is_none_or(|(s, _)| *s != std) {
                    pipe = Some((std, VideoPipeline::new(&VideoConfig::raw(std))));
                }
                let Some((_, p)) = pipe.as_mut() else { continue };
                // One receiver: bypass the combiner, as in walk_frames.
                for f in p.push_combined_line(&line).frames_completed {
                    damaged += f.damage.iter().filter(|&&d| d != 0).count() as u64;
                    pixels += f.damage.len() as u64;
                    frames += 1;
                }
            }
        }
        if left == win_bytes {
            break; // nothing read: end of the recording
        }
        let s = dsp.stats();
        let dmg = if pixels > 0 { format!("{:8.1}", 100.0 * damaged as f64 / pixels as f64) } else { "       -".into() };
        let gap = gap_ms.get(&w).map_or("      -".to_string(), |ms| format!("{ms:7.0}"));
        println!(
            "{:7.2}  {:<9}  {:5.1}  {:5.1}  {:7}  {dmg}  {frames:6}  {gap}",
            w as f64 * window,
            format!("{:?}", s.lock),
            s.cn_db,
            s.clip_pct,
            s.relocks - relocks_before
        );
        relocks_before = s.relocks;
        if eof {
            break;
        }
    }
    Ok(())
}
