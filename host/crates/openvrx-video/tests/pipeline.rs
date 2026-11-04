// SPDX-License-Identifier: GPL-3.0-or-later
//! Pipeline: streaming (line by line) output is bit-identical to field
//! mode, rows update as lines arrive, presets decode a picture correctly,
//! configs round-trip through serde.

mod common;

use common::*;
use openvrx_core::{Field, Frame, LineRecord, VideoStandard};
use openvrx_video::metrics::{psnr, Region};
use openvrx_video::pipeline::{VideoConfig, VideoPipeline};
use openvrx_video::testgen::{add_clicks, add_noise, Scene};
use openvrx_video::util::Rng;

/// Noisy lines with clicks and a few lost lines, from a moving scene.
fn lines(std: VideoStandard, fields: u64) -> (Vec<LineRecord>, Vec<openvrx_video::testgen::YuvImage>) {
    let p = std.params();
    let enc = encoder(std);
    let scene = Scene::new(4, (1.0, 0.25), (3.0, 1.0));
    let mut rng = Rng::new(17);
    let mut out = Vec::new();
    let mut pics = Vec::new();
    for seq in 0..fields {
        let pic = scene.render(p.active_width(), p.frame_height(), seq as f64);
        for mut l in encode(&enc, &pic, seq) {
            if rng.uniform() < 0.01 {
                continue; // lost line
            }
            add_noise(&mut l, 1.5, &mut rng);
            if rng.uniform() < 0.1 {
                add_clicks(&mut l, 2, &mut rng);
            }
            out.push(l);
        }
        pics.push(pic);
    }
    (out, pics)
}

fn same(a: &Frame, b: &Frame) -> bool {
    a.y == b.y && a.u == b.u && a.v == b.v && a.damage == b.damage && a.row_source == b.row_source
}

#[test]
fn streaming_equals_field_mode() {
    for std in VideoStandard::ALL {
        let (recs, _) = lines(std, 8);
        let single_line = VideoConfig {
            stream_batch_lines: 1,
            ..VideoConfig::balanced(std)
        };
        for (name, cfg) in [
            ("balanced, every line", single_line),
            ("low_latency", VideoConfig::low_latency(std)),
            ("balanced", VideoConfig::balanced(std)),
            ("quality", VideoConfig::quality(std)),
            ("raw", VideoConfig::raw(std)),
        ] {
            // Streaming, line by line.
            let mut sp = VideoPipeline::new(&cfg);
            let mut streamed = Vec::new();
            let mut updates = 0usize;
            for r in &recs {
                let o = sp.push_line(r.clone());
                if let Some(rows) = o.updated_rows {
                    assert!(rows.end <= sp.current_frame().height);
                    updates += 1;
                }
                streamed.extend(o.frames_completed);
            }
            streamed.extend(sp.flush());
            // Field mode.
            let mut fields: Vec<Field> = Vec::new();
            let mut cur: Option<Field> = None;
            for r in &recs {
                let new = cur.as_ref().map(|f| (f.seq, f.parity) != (r.field_seq, r.parity)).unwrap_or(true);
                if new {
                    if let Some(f) = cur.take() {
                        fields.push(f);
                    }
                    cur = Some(Field::new(std, r.field_seq, r.parity));
                }
                cur.as_mut().unwrap().put_line(r);
            }
            fields.extend(cur);
            let mut fp = VideoPipeline::new(&cfg);
            let mut offline = Vec::new();
            for f in fields {
                offline.extend(fp.push_field(f));
            }
            offline.extend(fp.flush());
            assert_eq!(streamed.len(), offline.len(), "{std:?} {name}");
            for (a, b) in streamed.iter().zip(&offline) {
                assert_eq!(a.meta.field_seq, b.meta.field_seq);
                assert!(same(a, b), "{std:?} {name}: field {} differs", a.meta.field_seq);
            }
            // Rows were updated many times per field (once per batch of
            // lines; the quality preset uses larger batches).
            let per_field = (200 / cfg.stream_batch_lines.max(1)).min(15);
            assert!(updates > per_field * streamed.len(), "{std:?} {name}: {updates} updates");
        }
    }
}

#[test]
fn rows_follow_arriving_lines() {
    let std = VideoStandard::Pal;
    let p = std.params();
    let (recs, _) = lines(std, 3);
    let cfg = VideoConfig {
        stream_batch_lines: 1,
        ..VideoConfig::balanced(std)
    };
    let mut sp = VideoPipeline::new(&cfg);
    let mut max_lag = 0isize;
    for r in &recs {
        let fl = r.field_line as isize;
        let o = sp.push_line(r.clone());
        if r.field_seq < 2 {
            continue;
        }
        if let Some(rows) = o.updated_rows {
            if !o.frames_completed.is_empty() {
                continue;
            }
            // Output row of the newest received line vs. newest updated row.
            let first = p.first_active_field_line(r.parity) as isize;
            let newest_row = 2 * (fl - first) + p.field_row_offset(r.parity) as isize;
            max_lag = max_lag.max(newest_row - rows.end as isize);
        }
    }
    // Comb (2 lines) + repair (2) + deinterlace (2) + denoise (1-2):
    // under 20 frame rows, about 0.6 ms behind the newest line.
    assert!(max_lag < 20, "row lag {max_lag}");
}

#[test]
fn presets_decode_the_picture() {
    for std in VideoStandard::ALL {
        let (recs, pics) = lines(std, 10);
        for cfg in [VideoConfig::low_latency(std), VideoConfig::balanced(std), VideoConfig::quality(std)] {
            let mut sp = VideoPipeline::new(&cfg);
            let mut frames = Vec::new();
            for r in &recs {
                frames.extend(sp.push_line(r.clone()).frames_completed);
            }
            let fr = frames.iter().find(|f| f.meta.field_seq == 8).expect("frame 8");
            let truth = &pics[8];
            let reg = Region::inset(fr.width, fr.height, 24, 12);
            let py = psnr(&fr.y, &truth.y, fr.width, reg, 1.0);
            let pu = psnr(&fr.u, &truth.u, fr.width, reg, 1.0);
            println!("{std:?} {}: Y {py:.2} dB, U {pu:.2} dB, timings {:?}", fr.meta.pipeline, sp.stats().last);
            assert!(py > 28.0, "{std:?} {} luma {py}", fr.meta.pipeline);
            assert!(pu > 30.0, "{std:?} {} chroma {pu}", fr.meta.pipeline);
        }
    }
}

#[test]
fn config_serde_roundtrip() {
    for name in ["low_latency", "balanced", "quality", "raw"] {
        let c = VideoConfig::preset(name, VideoStandard::Ntsc).unwrap();
        let s = serde_json::to_string(&c).unwrap();
        let back: VideoConfig = serde_json::from_str(&s).unwrap();
        assert_eq!(c, back);
    }
    let partial: VideoConfig = serde_json::from_str(r#"{"standard":"ntsc","deinterlace":"bob"}"#).unwrap();
    assert_eq!(partial.deinterlace, openvrx_video::DeinterlaceMethod::Bob);
}
