// SPDX-License-Identifier: GPL-3.0-or-later
//! Per-field processing time of the video pipeline (run with
//! `cargo run --release --example video_bench`).
//!
//! For each standard: the presets as the live app runs them (streaming,
//! `push_line` per line, per-field mean, median and worst time), streaming
//! without returning frame copies (`emit_frames` off), field mode (worker
//! threads) and the stage split of single-line streaming, then each colour
//! separation and each deinterlacer on its own.
//!
//! Other processes and the CPU's power state change the numbers a lot:
//! run it several times and compare minimum and median.

use openvrx_core::{Field, LineRecord, VideoStandard};
use openvrx_video::color::{ColorConfig, ColorDecoder, NtscSeparation, PalSeparation};
use openvrx_video::deinterlace::{create, DeinterlaceMethod};
use openvrx_video::encoder::{CompositeEncoder, EncoderConfig};
use openvrx_video::pipeline::{VideoConfig, VideoPipeline};
use openvrx_video::testgen::{add_noise, field_info, image_to_field, Scene};
use openvrx_video::util::Rng;
use std::sync::Arc;
use std::time::Instant;

const FIELDS: u64 = 24;
const WARM: u64 = 4;

fn records(std: VideoStandard) -> Vec<Vec<LineRecord>> {
    let p = std.params();
    let enc = CompositeEncoder::new(std, EncoderConfig::default());
    let scene = Scene::new(3, (1.0, 0.25), (3.0, 1.0));
    let mut rng = Rng::new(1);
    let field_ns = 1e9 / p.field_hz;
    (0..FIELDS)
        .map(|seq| {
            let pic = scene.render(p.active_width(), p.frame_height(), seq as f64);
            let (par, fi) = field_info(seq);
            let f = image_to_field(&pic, std, par, seq);
            let mut recs = enc.encode_field(&f, fi, (seq as f64 * field_ns) as u64 + 1);
            for r in &mut recs {
                add_noise(r, 2.0, &mut rng);
            }
            recs
        })
        .collect()
}

fn to_field(std: VideoStandard, recs: &[LineRecord]) -> Field {
    let mut f = Field::new(std, recs[0].field_seq, recs[0].parity);
    for r in recs {
        f.put_line(r);
    }
    f
}

fn ms(ns: f64) -> f64 {
    ns * 1e-6
}

fn main() {
    println!("threads: {}", rayon::current_num_threads());
    for std in VideoStandard::ALL {
        let recs = records(std);
        let fields: Vec<Field> = recs.iter().map(|r| to_field(std, r)).collect();
        println!("\n=== {std:?} (ms per field, mean of {} fields) ===", FIELDS - WARM);
        println!("presets          live (preset as configured, streaming)   no-emit   field-mode   stream/1 line: color/repair/deint/denoise");
        println!("                   mean   median      max                      median    mean");
        for name in ["raw", "low_latency", "balanced", "quality"] {
            let cfg = VideoConfig::preset(name, std).expect("preset");
            let mut p = VideoPipeline::new(&cfg);
            let mut t_field = 0.0;
            for (i, f) in fields.iter().enumerate() {
                let t = Instant::now();
                let _ = p.push_field(f.clone());
                if i as u64 >= WARM {
                    t_field += t.elapsed().as_nanos() as f64;
                }
            }
            // Per-field times of `push_line` over every line of a field.
            let stream = |cfg: &VideoConfig| -> (Vec<f64>, [f64; 4]) {
                let mut p = VideoPipeline::new(cfg);
                let mut times = Vec::new();
                let mut split = [0.0f64; 4];
                for (i, rs) in recs.iter().enumerate() {
                    let rs = rs.clone();
                    let t = Instant::now();
                    for r in rs {
                        let _ = p.push_line(r);
                    }
                    if i as u64 >= WARM {
                        times.push(t.elapsed().as_nanos() as f64);
                        let s = p.stats().last;
                        split[0] += s.color_ns as f64;
                        split[1] += s.repair_ns as f64;
                        split[2] += s.deinterlace_ns as f64;
                        split[3] += s.denoise_ns as f64;
                    }
                }
                times.sort_by(f64::total_cmp);
                (times, split)
            };
            let (live, _) = stream(&cfg);
            let (no_emit, _) = stream(&VideoConfig {
                emit_frames: false,
                ..cfg.clone()
            });
            let (_, split) = stream(&VideoConfig {
                stream_batch_lines: 1,
                ..cfg.clone()
            });
            let n = (FIELDS - WARM) as f64;
            let mean = live.iter().sum::<f64>() / n;
            println!(
                "  {:<14} {:>6.2} {:>8.2} {:>8.2}                      {:>8.2} {:>8.2}      ({:.2}/{:.2}/{:.2}/{:.2})",
                name,
                ms(mean),
                ms(live[live.len() / 2]),
                ms(live[live.len() - 1]),
                ms(no_emit[no_emit.len() / 2]),
                ms(t_field / n),
                ms(split[0] / n),
                ms(split[1] / n),
                ms(split[2] / n),
                ms(split[3] / n)
            );
        }

        println!("colour decode only     threads   1 thread");
        let seps: Vec<(String, ColorConfig)> = match std {
            VideoStandard::Ntsc => [NtscSeparation::Notch, NtscSeparation::Comb2D, NtscSeparation::Comb3D]
                .iter()
                .map(|&s| (format!("{s:?}"), ColorConfig { ntsc: s, ..ColorConfig::default() }))
                .collect(),
            VideoStandard::Pal => [
                PalSeparation::Notch,
                PalSeparation::DelayLine,
                PalSeparation::Comb2D,
                PalSeparation::Comb3D,
                PalSeparation::Transform2D,
                PalSeparation::Transform3D,
            ]
            .iter()
            .map(|&s| (format!("{s:?}"), ColorConfig { pal: s, ..ColorConfig::default() }))
            .collect(),
        };
        let mut yuv = Vec::new();
        for (name, cfg) in &seps {
            let mut times = [0.0f64; 2];
            for (k, par) in [true, false].into_iter().enumerate() {
                let mut d = ColorDecoder::new(std, cfg);
                for (i, f) in fields.iter().enumerate() {
                    let f = Arc::new(f.clone());
                    let mut out = openvrx_core::YuvField::new(std, f.seq, f.parity);
                    let t = Instant::now();
                    d.advance(&f, f.lines, true, &mut out, par);
                    d.finish_field(f.clone(), &mut out, par);
                    if i as u64 >= WARM {
                        times[k] += t.elapsed().as_nanos() as f64;
                    }
                    if k == 0 && name.as_str() == "Comb2D" {
                        yuv.push(Arc::new(out));
                    }
                }
            }
            let n = (FIELDS - WARM) as f64;
            println!("  {:<18} {:>9.2} {:>9.2}", name, ms(times[0] / n), ms(times[1] / n));
        }

        println!("deinterlace only (threads), delay");
        for m in DeinterlaceMethod::ALL {
            let mut d = create(m, std);
            let mut t_acc = 0.0;
            for (i, f) in yuv.iter().enumerate() {
                let t = Instant::now();
                let _ = d.push_field(f.clone());
                if i as u64 >= WARM {
                    t_acc += t.elapsed().as_nanos() as f64;
                }
            }
            println!("  {:<18} {:>9.2}   {}", m.name(), ms(t_acc / (FIELDS - WARM) as f64), m.latency_fields());
        }
    }
}
