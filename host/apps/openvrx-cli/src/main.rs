// SPDX-License-Identifier: GPL-3.0-or-later
//! Offline tools for OpenVRX recordings.
//!
//! ```text
//! openvrx-cli inspect REC [OUT_DIR]           lane activity, I/Q histograms, a sync-free FM strip image
//! openvrx-cli decode  REC [OUT_DIR] [FRAMES] [--steady-osd]
//!                                             frames through the live DSP and video chain (raw preset;
//!                                             --steady-osd: line-doubled fields as in the app's default)
//! openvrx-cli decode  REC OUT_DIR FRAMES --live [--preset NAME] [--no-steady-osd]
//!                                             the app's live video path instead: preset (default
//!                                             low_latency), streaming, every line through the combiner
//!                                             with host times from the byte position; saves displayed
//!                                             fields (FRAMES 0: none) and prints lost lines once a second
//!                                             and the combiner/assembler/repair counters (DSP-5)
//! openvrx-cli decode  ... --filter-plugin LIB [--plugin-config TEXT] [--plugin-deadline-ms MS]
//!                     [--plugin-json PATH]
//!                                             either decode with every field through a filter plugin
//!                                             (openvrx-filter ABI, as the app's plugin preset) before it
//!                                             is saved; the pipeline and geometry the plugin's info names
//!                                             replace --preset and the steady OSD choice. Prints ms per
//!                                             field (p50/p99/max/mean) and the fields over the live
//!                                             deadline (default 6 ms) on stderr; --plugin-json writes
//!                                             them as JSON. --live with FRAMES 0 times the whole
//!                                             recording without saving pictures
//! openvrx-cli phasecal --lanes P --clk D [--c5 PORT] [--seconds S]
//!                                             sets the C5 sample clock edge to the cleanest of its
//!                                             three positions, measured through the FX2 (S seconds
//!                                             per position, default 0.25); needs the C5 console
//! openvrx-cli scan --lanes P --clk D [--c5 PORT] [--bands RABEFL] [--settle MS] [--dwell MS]
//!                  [--json PATH] [--stay]
//!                                             band scan on the live receiver: retunes the C5 over the
//!                                             bands (default all six; settle 30 ms, dwell 60 ms), ranks
//!                                             the channels by sync, C/N and level and names the video
//!                                             transmitters; then back to the channel it was on (--stay:
//!                                             to the best transmitter). Needs the C5 console
//! openvrx-cli record  OUT_BASE --lanes P --clk D [--seconds S] [--mhz M] [--gain G] [--json PATH]
//!                                             FX2-only capture, no window (bench scripts). OUT_BASE
//!                                             `-` drains: streams and counts, writes nothing. Prints
//!                                             the byte rate against nominal, FIFO-full polls, host
//!                                             drops and transfer errors; --json PATH writes them too
//! openvrx-cli score   REC [--ref REC] [--frames N] [--skip K] [--demod polar|lut|ls|pll|ekf]
//!                     [--preset raw|low_latency|balanced|quality] [--ref-demod D]
//!                                             one JSON line: lock, C/N, clipping, frames, PSNR vs ref,
//!                                             DSP load and stage_ns (defaults: polar, raw; the
//!                                             reference goes through the same chain)
//! openvrx-cli timeline REC [--window S] [--demod polar|lut|ls|pll|ekf]
//!                                             the whole recording in S-second rows (default 0.5):
//!                                             lock, C/N, clipping, relocks, damaged share, USB gaps
//! openvrx-cli lines REC OUT.csv [--demod polar|lut|ls|pll|ekf]
//!                                             one CSV row per line from the DSP: input sample
//!                                             position, field and line, quality (C/N, noise on the
//!                                             porch and tip, clicks, power, weight, flags) and the
//!                                             damaged and severe shares of its mask
//! ```
//! REC is any file of a recording (`NAME.openvrx.json`, `NAME.rx0.bin`) or its base path.

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

mod filter;
mod scan;

/// Line rate the strip is cut at (NTSC; PAL rows drift slowly sideways).
const LINE_HZ: f64 = 15_734.264;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    // Commands without a recording argument.
    let live: Option<fn(&[String]) -> Result<(), String>> = match args.first().map(String::as_str) {
        Some("phasecal") => Some(phasecal),
        Some("scan") => Some(scan::scan),
        _ => None,
    };
    if let Some(run) = live {
        if let Err(e) = run(&args[1..]) {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
        return;
    }
    let (cmd, rec) = match (args.first(), args.get(1)) {
        (Some(c), Some(r)) => (c.as_str(), r.as_str()),
        _ => {
            eprintln!(
                "usage: openvrx-cli inspect|decode|record|score|timeline|lines|phasecal|scan ... (see the source header)"
            );
            std::process::exit(2);
        }
    };
    let out = args.get(2).map(PathBuf::from).unwrap_or_else(|| PathBuf::from("openvrx-out"));
    let result = match cmd {
        "inspect" => inspect(rec, &out),
        "decode" => filter::CliFilter::from_args(&args).and_then(|mut f| {
            let frames = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(8);
            let r = if args.iter().any(|a| a == "--live") {
                decode_live(
                    rec,
                    &out,
                    frames,
                    opt(&args, "--preset").unwrap_or("low_latency"),
                    !args.iter().any(|a| a == "--no-steady-osd"),
                    f.as_mut(),
                )
            } else {
                decode(rec, &out, frames, args.iter().any(|a| a == "--steady-osd"), f.as_mut())
            };
            match &f {
                Some(f) => r.and(f.report(&args)),
                None => r,
            }
        }),
        "record" => record(rec, &args[2..]),
        "score" => score(rec, &args[2..]),
        "timeline" => timeline(rec, &args[2..]),
        "lines" => lines_csv(rec, &args[2..]),
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

/// `decode`: the raw chain's frames as PNG; with a filter plugin, its
/// pipeline and geometry, and each frame as the plugin made it.
fn decode(
    path: &str,
    out: &Path,
    frames: usize,
    steady_osd: bool,
    mut filter: Option<&mut filter::CliFilter>,
) -> Result<(), String> {
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let (preset, steady_osd) = match filter.as_deref() {
        Some(fl) => (fl.info().pipeline.clone(), fl.info().geometry.steady(steady_osd)),
        None => ("raw".to_string(), steady_osd),
    };
    let mut saved = 0;
    let (total_lines, stats) = walk_frames(&rec, steady_osd, openvrx_dsp::DemodKind::Polar, &preset, |f| {
        if saved >= frames {
            return Ok(false);
        }
        let f = match filter.as_deref_mut() {
            Some(fl) => fl.apply(f, steady_osd)?,
            None => f,
        };
        let img = image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8()).ok_or("frame size")?;
        let name = format!("{}-frame{saved:02}.png", rec.name());
        let path = out.join(&name);
        img.save(&path).map_err(|e| e.to_string())?;
        println!("saved {} (field {} parity {:?})", path.display(), f.meta.field_seq, f.meta.parity);
        saved += 1;
        Ok(saved < frames)
    })?;
    println!("{total_lines} lines from the DSP; stats: {stats:?}");
    Ok(())
}

/// The app's live video path on a recording (DSP-5): the named preset with
/// streaming on, every line through the combiner (`push_line`, not the
/// bypass) with host times rebuilt from the byte position at the sample
/// rate, 32 KB per block as the FX2 delivers them, and the gaps the
/// recording's events name declared as the app does. A field is saved
/// whenever the display would show a completed one (`current_frame()`
/// after the bottom row updated). Prints the pipeline counters at the end.
/// With a filter plugin each displayed field goes through it (its pipeline
/// and geometry replace `preset` and `steady_osd`), saved or not.
fn decode_live(
    path: &str,
    out: &Path,
    frames: usize,
    preset: &str,
    steady_osd: bool,
    mut filter: Option<&mut filter::CliFilter>,
) -> Result<(), String> {
    const BLOCK: usize = 32 * 1024;
    let (preset, steady_osd) = match filter.as_deref() {
        Some(fl) => {
            if preset != fl.info().pipeline && preset != "low_latency" {
                eprintln!("note: the plugin asks for the {} pipeline, not {preset}", fl.info().pipeline);
            }
            (fl.info().pipeline.clone(), fl.info().geometry.steady(steady_osd))
        }
        None => (preset.to_string(), steady_osd),
    };
    let preset = preset.as_str();
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
            if frames > 0 && saved >= frames {
                break 'read;
            }
            let f = match filter.as_deref_mut() {
                Some(fl) => fl.apply(f, steady_osd)?,
                None => f,
            };
            if saved < frames {
                let img =
                    image::RgbaImage::from_raw(f.width as u32, f.height as u32, f.to_rgba8()).ok_or("frame size")?;
                let path = out.join(format!("{}-live{saved:04}.png", rec.name()));
                img.save(&path).map_err(|e| e.to_string())?;
                saved += 1;
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

    ensure_fx2_firmware()?;
    let settings = C5Settings {
        lanes,
        divider,
        mhz,
        gain,
        ..C5Settings::default()
    };
    let mut cfg = ReceiverConfig::new(0, PortSelector::Name(String::new()), Fx2Selector::Only, settings.clone());
    // This loop writes the file itself, so the block channel is the only
    // buffer against a slow write: the default 64 blocks (78 ms at 26.7 MB/s)
    // are shorter than the periodic flush stalls of a long recording to a USB
    // drive. 512 blocks of 32 KB = 16 MB, about 600 ms.
    cfg.block_capacity = 512;
    let receiver = Receiver::start_fx2_only(cfg).map_err(|e| format!("starting the FX2: {e}"))?;

    let mut info = ReceiverInfo::new(0, settings.sample_format());
    info.channel_mhz = mhz;
    info.gain = gain;
    info.board = settings.board.name().to_string();
    info.start_host_ns = openvrx_core::clock::now_ns();
    let sidecar = Sidecar::new(RecordingSource::Hardware, vec![info]);
    // `-` as OUT_BASE: a drain. The same capture path, but the blocks are
    // only counted, so losses come from USB and the FX2 FIFO, never the disk.
    let drain = base == "-";
    let mut writer = if drain {
        None
    } else {
        Some(RecordingWriter::create(base, sidecar, WriterOptions::default()).map_err(|e| e.to_string())?)
    };
    let want = (seconds * settings.sample_format().sample_rate_hz()) as u64
        * settings.sample_format().lanes.word_bytes as u64;
    let t0 = Instant::now();
    let mut bytes = 0u64;
    let mut blocks = 0u64;
    let mut fifo_polls = 0u64;
    // Byte rate from the first block to the last: (time, bytes) at the first block.
    let mut first: Option<(Instant, u64)> = None;
    let mut last_at = t0;
    // Reports from before the first block (stream start-up) say nothing about this file.
    if let Some(s) = receiver.stream() {
        let _ = s.events().try_iter().count();
    }
    while bytes < want {
        match receiver.blocks().recv_timeout(Duration::from_millis(500)) {
            Ok(b) => {
                bytes += match writer.as_mut() {
                    Some(w) => w.write_block(&b).map_err(|e| e.to_string())?,
                    None => b.data.len() as u64,
                };
                blocks += 1;
                last_at = Instant::now();
                first.get_or_insert((last_at, bytes));
                fifo_polls += log_fifo_full(&receiver, writer.as_mut())?;
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
    // The last status blocks of the run.
    std::thread::sleep(Duration::from_millis(50));
    fifo_polls += log_fifo_full(&receiver, writer.as_mut())?;
    let stats = receiver.stream().map(|s| s.stats());
    drop(receiver);
    let wall = t0.elapsed().as_secs_f64();
    let target = if drain {
        "drain (nothing written)".to_string()
    } else {
        let w = writer.take().expect("writer when not draining");
        let path = w.paths().sidecar();
        w.close().map_err(|e| e.to_string())?;
        path.display().to_string()
    };
    println!(
        "recorded {target} ({:.1} MB in {blocks} blocks, {wall:.1} s wall)",
        bytes as f64 / 1e6
    );
    // Measured against the nominal rate. The C5 crystal alone moves this by
    // about +-150 ppm, so only shortfalls well above 0.02 % mean lost data.
    let nominal = settings.sample_format().byte_rate();
    let measured = match first {
        Some((t1, b1)) if last_at > t1 => Some((bytes - b1) as f64 / (last_at - t1).as_secs_f64()),
        _ => None,
    };
    let short_pct = measured.map(|m| (1.0 - m / nominal) * 100.0);
    let s = stats.unwrap_or_default();
    println!(
        "stream: {} MB/s (nominal {:.3}, short {}), fifo_full {fifo_polls} polls ({:.2} ms), \
         host drops {} blocks, transfer errors {}, restarts {}",
        measured.map_or("?".into(), |m| format!("{:.3}", m / 1e6)),
        nominal / 1e6,
        short_pct.map_or("?".into(), |p| format!("{p:.3} %")),
        fifo_polls as f64 * FIFO_POLL_US / 1e3,
        s.dropped_blocks,
        s.transfer_errors,
        s.restarts
    );
    if let Some(json) = opt(args, "--json") {
        let summary = serde_json::json!({
            "out": if drain { serde_json::Value::Null } else { target.clone().into() },
            "drain": drain,
            "lanes": lanes.to_string(),
            "clk": divider,
            "seconds": seconds,
            "bytes": bytes,
            "blocks": blocks,
            "wall_s": wall,
            "nominal_bps": nominal,
            "measured_bps": measured,
            "short_pct": short_pct,
            "fifo_full_polls": fifo_polls,
            "fifo_lost_ms": fifo_polls as f64 * FIFO_POLL_US / 1e3,
            "host_dropped_blocks": s.dropped_blocks,
            "host_dropped_bytes": s.dropped_bytes,
            "transfer_errors": s.transfer_errors,
            "stalls": s.stalls,
            "restarts": s.restarts,
            "disconnects": s.disconnects,
        });
        std::fs::write(json, format!("{summary}\n")).map_err(|e| format!("{json}: {e}"))?;
    }
    if fifo_polls > 0 {
        eprintln!(
            "warning: FX2 FIFO full for {fifo_polls} polls (about {:.2} ms of samples lost at the pins)",
            fifo_polls as f64 * FIFO_POLL_US / 1e3
        );
        if cfg!(target_os = "linux") && openvrx_hw::power::DmaLatencyGuard::request(1).is_err() {
            eprintln!(
                "         /dev/cpu_dma_latency is not writable, so idle C-states were allowed: \
                 install the udev rule from tools/bench/setup-fedora.sh"
            );
        }
    }
    Ok(())
}

/// One poll of the FX2's AUTOIN loop (firmware/fx2/src/hotloops.asm: 10
/// cycles of 83.3 ns); a poll that finds the FIFO full stands for about this
/// much of the C5's samples dropped at the pins.
const FIFO_POLL_US: f64 = 0.833;

/// Loads the embedded firmware into every FX2 still in its boot loader and
/// waits for the renumeration. A board already running OpenVRX firmware is
/// left alone: loading over a running image resets only its CPU, and a
/// board in that state can stop streaming until it is power-cycled.
fn ensure_fx2_firmware() -> Result<(), String> {
    if !list_fx2().map_err(|e| e.to_string())?.iter().any(|f| f.kind == Fx2Kind::Bootloader) {
        return Ok(());
    }
    let fw = fx2::firmware::embedded().map_err(|e| e.to_string())?;
    for (dev, r) in fx2::upload_all(&fw).map_err(|e| e.to_string())? {
        r.map_err(|e| format!("FX2 firmware upload on {}: {e}", dev.describe()))?;
    }
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs(5)
        && !list_fx2().map_err(|e| e.to_string())?.iter().any(|f| f.kind == Fx2Kind::Openvrx)
    {
        std::thread::sleep(Duration::from_millis(200));
    }
    Ok(())
}

/// Share of samples (both components) that jump away from what their
/// neighbours predict by more than a few steps of the lowest carried bit.
/// The captured signal is oversampled (analog filter well under half the
/// sample rate), so a sample is close to `(4 (x[n-1] + x[n+1]) - (x[n-2] +
/// x[n+2])) / 6`; a lane caught mid-change at the FX2 sampling edge flips
/// its bit and leaves a jump of that bit's weight. Words are little-endian
/// as received; NaN when the capture is too short.
fn jump_share(bytes: &[u8], fmt: &openvrx_core::LaneFormat) -> f64 {
    let wb = fmt.word_bytes as usize;
    let n = bytes.len() / wb;
    if n < 1000 {
        return f64::NAN;
    }
    let mut re = Vec::with_capacity(n);
    let mut im = Vec::with_capacity(n);
    for c in bytes.chunks_exact(wb) {
        let w = if wb == 1 { c[0] as u32 } else { c[0] as u32 | (c[1] as u32) << 8 };
        let z = fmt.decode(w);
        re.push(z.re);
        im.push(z.im);
    }
    // Step of the lowest carried bit: word 0 decodes to half a step.
    let step = 2.0 * fmt.decode(0).re.abs();
    let half_codes = (fmt.component_full_scale() / step).max(1.0);
    // 6 steps on 7 bits (just under the 8-step weight of the fourth bit from
    // the top), at least 1.5 steps on fewer bits.
    let thr = step * (0.75 * half_codes / 8.0).max(1.5);
    let mut bad = 0usize;
    let mut total = 0usize;
    for x in [&re, &im] {
        for k in 2..x.len() - 2 {
            let pred = (4.0 * (x[k - 1] + x[k + 1]) - (x[k - 2] + x[k + 2])) / 6.0;
            bad += ((x[k] - pred).abs() > thr) as usize;
            total += 1;
        }
    }
    bad as f64 / total as f64
}

/// Sets the C5's sample clock edge to the cleanest of its three positions
/// against the MODEM_DIAG bus, measured through the FX2 at the working rate.
/// One stream: measure, `slip`, measure, `slip`, measure, then slip to the
/// best. Uses the C5 console (`--c5 PORT`, default the first Espressif port)
/// and pushes no other settings; `--lanes`/`--clk` must match the C5.
fn phasecal(args: &[String]) -> Result<(), String> {
    use openvrx_hw::c5::{find_c5_ports, C5Client, ClientConfig, PortConnector};
    let lanes: LanePreset = opt(args, "--lanes")
        .ok_or("phasecal needs --lanes PRESET")?
        .parse()
        .map_err(|e| format!("{e}"))?;
    let divider: u32 = opt_num(args, "--clk", 9)?;
    let seconds: f64 = opt_num(args, "--seconds", 0.25)?;
    let port = match opt(args, "--c5") {
        Some(p) => p.to_string(),
        None => find_c5_ports()
            .map_err(|e| e.to_string())?
            .into_iter()
            .next()
            .map(|p| p.port_name)
            .ok_or("no C5 on USB (phasecal needs its console)")?,
    };
    let c5 = C5Client::start(PortConnector::new(PortSelector::Name(port.clone())), None, ClientConfig::default())
        .map_err(|e| e.to_string())?;
    if !c5.wait_connected(Duration::from_secs(3)) {
        return Err(format!("C5 on {port} does not answer"));
    }
    let st = c5.status().map_err(|e| format!("C5 status: {e}"))?;
    if st.lanes != lanes.name() || st.clk != divider || !st.clk_on {
        return Err(format!(
            "the C5 streams {} clk {} (clock {}), not {} clk {divider}",
            st.lanes,
            st.clk,
            if st.clk_on { "on" } else { "off" },
            lanes.name()
        ));
    }

    ensure_fx2_firmware()?;
    let settings = C5Settings {
        lanes,
        divider,
        ..C5Settings::default()
    };
    let fmt = settings.sample_format().lanes.clone();
    let want = (seconds * settings.sample_format().sample_rate_hz()) as usize * fmt.word_bytes as usize;
    let cfg = ReceiverConfig::new(0, PortSelector::Name(String::new()), Fx2Selector::Only, settings.clone());
    let receiver = Receiver::start_fx2_only(cfg).map_err(|e| format!("starting the FX2: {e}"))?;

    // Bytes after the last slip: drop what is queued, then 50 ms more (the
    // slip itself is a ~25 us clock gap), then collect `want` bytes.
    let capture = |receiver: &Receiver| -> Result<Vec<u8>, String> {
        while receiver.blocks().try_recv().is_ok() {}
        let t0 = Instant::now();
        let mut buf = Vec::with_capacity(want);
        while buf.len() < want {
            match receiver.blocks().recv_timeout(Duration::from_millis(500)) {
                Ok(b) if t0.elapsed() >= Duration::from_millis(50) => buf.extend_from_slice(&b.data),
                Ok(_) => {}
                Err(_) if t0.elapsed() > Duration::from_secs(3) => {
                    return Err(if buf.is_empty() {
                        "no data from the FX2 in 3 s (replug the FX2 if it stays so)".into()
                    } else {
                        "the FX2 stream stopped during the measurement".into()
                    });
                }
                Err(_) => {}
            }
        }
        Ok(buf)
    };
    let mut shares = [f64::NAN; 3];
    let mut total = 0u32;
    for (k, share) in shares.iter_mut().enumerate() {
        *share = jump_share(&capture(&receiver)?, &fmt);
        if k < 2 {
            total = c5.slip(1).map_err(|e| format!("slip: {e}"))?;
        }
    }
    drop(receiver);

    // Positions relative to now (the third one): +1 slip is the first, +2 the second.
    let best = (0..3)
        .filter(|&k| shares[k].is_finite())
        .min_by(|&a, &b| shares[a].total_cmp(&shares[b]))
        .ok_or("no position could be measured")?;
    let slips = (best as u32 + 1) % 3;
    if slips > 0 {
        total = c5.slip(slips).map_err(|e| format!("slip: {e}"))?;
    }
    let pct = |v: f64| format!("{:.2} %", 100.0 * v);
    println!(
        "phasecal: bit-jump share per position {} / {} / {}: chose {} (C5 slip total {total}, rel {})",
        pct(shares[0]),
        pct(shares[1]),
        pct(shares[2]),
        best + 1,
        total % 3
    );
    if shares[best] > 0.02 {
        eprintln!(
            "warning: no clean position ({} at best). Check the clock wire and the grounds; with no \
             signal the score is less sharp.",
            pct(shares[best])
        );
    }
    Ok(())
}

/// Logs the FIFO-full reports the FX2 stream has queued as `fifo_full`
/// events: a recording otherwise has no trace of samples lost at
/// the pins, since no stream position jumps. Returns the polls seen (also
/// counted with no writer, for a drain).
fn log_fifo_full(receiver: &Receiver, mut writer: Option<&mut RecordingWriter>) -> Result<u64, String> {
    let Some(stream) = receiver.stream() else { return Ok(0) };
    let mut polls = 0u64;
    for ev in stream.events().try_iter() {
        if let fx2::StreamEvent::FifoFull {
            new_events,
            byte,
            host_ns,
            ..
        } = ev
        {
            if let Some(w) = writer.as_deref_mut() {
                let e = openvrx_format::Event::new(
                    Some(receiver.rx()),
                    host_ns,
                    Some(byte),
                    openvrx_format::EventKind::FifoFull { polls: new_events as u64 },
                );
                w.log_stream_event(&e).map_err(|e| e.to_string())?;
            }
            polls += new_events as u64;
        }
    }
    Ok(polls)
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
    let loss = data_loss(&rec);
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
         \"usb_gaps\":{},\"usb_gap_ms\":{:.1},\"fifo_full_polls\":{},\"fifo_lost_ms\":{:.2},\
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
        loss.gaps,
        loss.gap_ms,
        loss.fifo_polls,
        loss.fifo_polls as f64 * FIFO_POLL_US / 1e3,
        stats.demod.name(),
        ref_demod.name(),
        stats.load,
        stage_ns.join(",")
    );
    Ok(())
}

/// Samples a recording is missing: `usb_gap` events (host drops, the file
/// skips them) and `fifo_full` polls (dropped at the FX2 pins).
struct DataLoss {
    gaps: usize,
    gap_ms: f64,
    fifo_polls: u64,
}

fn data_loss(rec: &Recording) -> DataLoss {
    let rate = rec.receiver(0).map(|i| i.byte_rate()).unwrap_or(0.0);
    let mut d = DataLoss { gaps: 0, gap_ms: 0.0, fifo_polls: 0 };
    for e in &rec.events().events {
        match &e.kind {
            openvrx_format::EventKind::UsbGap { lost_bytes, .. } => {
                d.gaps += 1;
                if rate > 0.0 {
                    d.gap_ms += *lost_bytes as f64 / rate * 1e3;
                }
            }
            openvrx_format::EventKind::FifoFull { polls } => d.fifo_polls += polls,
            _ => {}
        }
    }
    d
}

/// Health of a whole recording over time, one row per `--window` seconds
/// (default 0.5): where a flight lost its picture and why. Time is the
/// position in the raw file (samples recorded), so a USB gap does not shift
/// later rows; `gap_ms` is the radio time lost in gaps that began in the
/// window. The other columns come from the same DSP and video chain as
/// `score`: lock and C/N and clipping as they stand at the window's end,
/// relocks during it, and the damaged share of the frames finished in it.
/// Per-line quality as CSV, to line up the DSP's view of each line with events in the raw
/// samples (gain writes, clipping, fades) by input sample position.
fn lines_csv(path: &str, args: &[String]) -> Result<(), String> {
    use std::io::Write;
    let out = args.first().filter(|a| !a.starts_with("--")).ok_or("lines needs an output CSV path")?;
    let rec = Recording::open(path).map_err(|e| e.to_string())?;
    let info = rec.receiver(0).map_err(|e| e.to_string())?.clone();
    let cfg = RxDspConfig {
        sample_format: info.sample_format.clone(),
        standard: None,
        demod: parse_demod(args, "--demod", openvrx_dsp::DemodKind::Polar)?,
        ..RxDspConfig::default()
    };
    let mut dsp = RxDsp::new(0, cfg);
    let mut r = rec.raw_reader(0).map_err(|e| e.to_string())?;
    let mut w = std::io::BufWriter::new(std::fs::File::create(out).map_err(|e| e.to_string())?);
    let io = |e: std::io::Error| e.to_string();
    writeln!(w, "src_pos,field_seq,field_line,frame_line,flags,cn_db,noise_ire,clicks,power_db,weight,sync_score,damaged,severe")
        .map_err(io)?;
    let mut buf = vec![0u8; 1 << 18];
    let mut lines: Vec<LineRecord> = Vec::new();
    let mut count = 0u64;
    loop {
        let n = r.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        lines.clear();
        dsp.push_bytes(&buf[..n], 0, &mut lines);
        for l in &lines {
            let len = l.damage.len().max(1) as f32;
            let damaged = l.damage.iter().filter(|&&d| d != 0).count() as f32 / len;
            let severe = l.damage.iter().filter(|&&d| d >= openvrx_core::damage::SEVERE).count() as f32 / len;
            let q = &l.quality;
            writeln!(
                w,
                "{:.1},{},{},{},{},{:.2},{:.3},{},{:.2},{:.4},{:.3},{:.4},{:.4}",
                l.src_position,
                l.field_seq,
                l.field_line,
                l.frame_line,
                q.flags.0,
                q.cn_db,
                q.noise_ire,
                q.clicks,
                q.power_db,
                q.weight,
                q.sync_score,
                damaged,
                severe
            )
            .map_err(io)?;
            count += 1;
        }
    }
    w.flush().map_err(io)?;
    println!("{count} lines written to {out}");
    Ok(())
}

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
    // Estimated ms of samples dropped at the FX2 pins (fifo_full events).
    let mut fifo_ms: BTreeMap<u64, f64> = BTreeMap::new();
    for e in &rec.events().events {
        if let (Some(byte), openvrx_format::EventKind::UsbGap { lost_bytes, .. }) = (e.byte, &e.kind) {
            *gap_ms.entry(byte / win_bytes as u64).or_default() += *lost_bytes as f64 / byte_rate * 1e3;
        }
        if let (Some(byte), openvrx_format::EventKind::FifoFull { polls }) = (e.byte, &e.kind) {
            *fifo_ms.entry(byte / win_bytes as u64).or_default() += *polls as f64 * FIFO_POLL_US / 1e3;
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
    println!("    t_s  lock       cn_db  clip%  relocks  damaged%  frames  gap_ms  fifo_ms");
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
        let fifo = fifo_ms.get(&w).map_or("      -".to_string(), |ms| format!("{ms:7.2}"));
        println!(
            "{:7.2}  {:<9}  {:5.1}  {:5.1}  {:7}  {dmg}  {frames:6}  {gap}  {fifo}",
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

#[cfg(test)]
mod tests {
    use super::*;

    /// An oversampled FM carrier in 7x7 words, with one lane optionally
    /// replaced by noise on a share of the samples (a lane caught mid-change).
    fn carrier_words(bad_bit: Option<u32>, bad_share: f64) -> Vec<u8> {
        let fmt = LanePreset::I7Q7.format();
        let mut rng = 0x1234_5678_u64;
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut out = Vec::new();
        let mut phase = 0.0f64;
        for n in 0..200_000 {
            // 0.3 MHz offset plus a 1 MHz deviation tone at 16 MS/s.
            phase += std::f64::consts::TAU * (0.3e6 + 1.0e6 * (n as f64 * 0.02).sin()) / 16e6;
            let a = 100.0; // 10-bit ADC units: about 12.5 steps of 8
            let mut w = fmt.encode((a * phase.cos()) as i32, (a * phase.sin()) as i32);
            if let Some(b) = bad_bit {
                if next() < bad_share {
                    w ^= 1 << b;
                }
            }
            out.extend_from_slice(&(w as u16).to_le_bytes());
        }
        out
    }

    #[test]
    fn jump_share_separates_clean_and_bad_lanes() {
        let fmt = LanePreset::I7Q7.format();
        let clean = jump_share(&carrier_words(None, 0.0), &fmt);
        // Word bit 12 is I6 (weight 8 steps): flipped on 20 % of samples.
        let bad = jump_share(&carrier_words(Some(12), 0.2), &fmt);
        assert!(clean < 0.002, "clean {clean}");
        assert!(bad > 0.05, "bad {bad}");
        assert!(jump_share(&[0u8; 100], &fmt).is_nan());
    }
}
