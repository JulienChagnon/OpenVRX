// SPDX-License-Identifier: GPL-3.0-or-later
//! Deinterlacer quality: progressive sequences are rendered, split into
//! fields, deinterlaced and compared with the progressive truth at each
//! field time.

use openvrx_core::{Frame, VideoStandard};
use openvrx_video::deinterlace::{create, DeinterlaceMethod};
use openvrx_video::metrics::{psnr, Region};
use openvrx_video::testgen::{field_info, image_to_field, Scene, YuvImage};
use std::collections::HashMap;
use std::sync::Arc;

const FIELDS: u64 = 16;
const SKIP: u64 = 6;

fn sequences(std: VideoStandard) -> Vec<(&'static str, Vec<YuvImage>)> {
    let p = std.params();
    let (w, h) = (p.active_width(), p.frame_height());
    let mk = |pan: (f64, f64), obj: (f64, f64)| -> Vec<YuvImage> {
        let s = Scene::new(11, pan, obj);
        (0..FIELDS).map(|t| s.render(w, h, t as f64)).collect()
    };
    vec![
        ("static", mk((0.0, 0.0), (0.0, 0.0))),
        ("object", mk((0.0, 0.0), (3.0, 1.0))),
        ("hpan", mk((2.0, 0.0), (0.0, 0.0))),
        ("pan", mk((1.5, 0.5), (3.0, 1.0))),
        ("tilt", mk((0.0, 1.25), (0.0, 0.0))),
    ]
}

/// Mean luma PSNR of a method over fields SKIP..FIELDS-1.
fn run(std: VideoStandard, method: DeinterlaceMethod, pics: &[YuvImage]) -> f64 {
    let mut d = create(method, std);
    let mut frames: HashMap<u64, Frame> = HashMap::new();
    for (t, pic) in pics.iter().enumerate() {
        let seq = t as u64;
        let (par, _) = field_info(seq);
        let f = Arc::new(image_to_field(pic, std, par, seq));
        if let Some(fr) = d.push_field(f) {
            frames.insert(fr.meta.field_seq, fr);
        }
    }
    if let Some(fr) = d.flush() {
        frames.insert(fr.meta.field_seq, fr);
    }
    let mut acc = 0.0;
    let mut n = 0.0;
    for seq in SKIP..FIELDS - 1 {
        let fr = &frames[&seq];
        let truth = &pics[seq as usize];
        let r = Region::inset(fr.width, fr.height, 16, 8);
        acc += psnr(&fr.y, &truth.y, fr.width, r, 1.0);
        n += 1.0;
    }
    acc / n
}

#[test]
fn psnr_table() {
    for std in VideoStandard::ALL {
        let seqs = sequences(std);
        println!("\n{std:?} luma PSNR (dB), mean over fields {SKIP}..{}", FIELDS - 2);
        print!("  {:<16} {:>6}", "method", "delay");
        for (name, _) in &seqs {
            print!(" {name:>8}");
        }
        println!();
        let mut table: HashMap<(DeinterlaceMethod, &str), f64> = HashMap::new();
        for m in DeinterlaceMethod::ALL {
            print!("  {:<16} {:>6}", m.name(), m.latency_fields());
            for (name, pics) in &seqs {
                let v = run(std, m, pics);
                table.insert((m, name), v);
                print!(" {v:>8.2}");
            }
            println!();
        }
        use DeinterlaceMethod::*;
        let g = |m, s| table[&(m, s)];
        // Static pictures: everything that weaves is (near) exact.
        for m in [Weave, MotionAdaptive, YadifPast, EdgeDirectedMA, Yadif, Bwdif, MotionCompensated] {
            assert!(g(m, "static") > 45.0, "{std:?} {m:?} static {}", g(m, "static"));
        }
        // The live default beats bob and weave on motion and never falls
        // far below the best past-only method.
        for s in ["object", "pan", "tilt"] {
            let edi = g(EdgeDirectedMA, s);
            assert!(edi > g(Weave, s) + 3.0, "{std:?} {s}: edi {edi} weave {}", g(Weave, s));
            assert!(edi > g(Bob, s) - 0.6, "{std:?} {s}: edi {edi} bob {}", g(Bob, s));
            assert!(edi > g(LinearBlend, s), "{std:?} {s}: edi {edi} blend {}", g(LinearBlend, s));
        }
        assert!(g(EdgeDirectedMA, "static") > g(Bob, "static") + 10.0);
        // Motion compensation pays off on pans.
        assert!(
            g(MotionCompensated, "pan") > g(EdgeDirectedMA, "pan"),
            "{std:?} mc {} edi {}",
            g(MotionCompensated, "pan"),
            g(EdgeDirectedMA, "pan")
        );
    }
}

/// Bob keeps every field at its true height: shifting its output by one
/// frame row in either direction makes it worse, for both parities.
#[test]
fn bob_has_no_vertical_jitter() {
    for std in VideoStandard::ALL {
        let p = std.params();
        let s = Scene::new(3, (0.0, 0.0), (0.0, 0.0));
        let pic = s.render(p.active_width(), p.frame_height(), 0.0);
        let mut d = create(DeinterlaceMethod::Bob, std);
        for seq in 0..2u64 {
            let (par, _) = field_info(seq);
            let fr = d.push_field(Arc::new(image_to_field(&pic, std, par, seq))).unwrap();
            let w = fr.width;
            let h = fr.height;
            let shifted = |dy: isize| -> f64 {
                let mut acc = 0.0f64;
                let mut n = 0.0;
                for y in 8..h - 8 {
                    let ys = (y as isize + dy) as usize;
                    for x in 16..w - 16 {
                        let e = (fr.y[y * w + x] - pic.y[ys * w + x]) as f64;
                        acc += e * e;
                        n += 1.0;
                    }
                }
                acc / n
            };
            let (m0, mu, md) = (shifted(0), shifted(-1), shifted(1));
            assert!(m0 < 0.5 * mu && m0 < 0.5 * md, "{std:?} {par:?}: {m0} {mu} {md}");
            // Own rows are passed through exactly.
            let off = p.field_row_offset(par);
            for y in (off..h).step_by(2) {
                assert_eq!(&fr.y[y * w..(y + 1) * w], &pic.y[y * w..(y + 1) * w]);
            }
        }
    }
}

/// Static fine detail must not flicker with the live default: two
/// consecutive outputs of a static picture are identical.
#[test]
fn edi_static_detail_does_not_flicker() {
    let std = VideoStandard::Pal;
    let p = std.params();
    let s = Scene::new(5, (0.0, 0.0), (0.0, 0.0));
    let pic = s.render(p.active_width(), p.frame_height(), 0.0);
    let mut d = create(DeinterlaceMethod::EdgeDirectedMA, std);
    let mut outs = Vec::new();
    for seq in 0..6u64 {
        let (par, _) = field_info(seq);
        outs.push(d.push_field(Arc::new(image_to_field(&pic, std, par, seq))).unwrap());
    }
    let a = &outs[4];
    let b = &outs[5];
    let maxd = a.y.iter().zip(&b.y).map(|(x, y)| (x - y).abs()).fold(0.0f32, f32::max);
    assert!(maxd < 1e-6, "static flicker {maxd}");
}

/// Shallow moving edges: the edge-directed interpolation beats the
/// vertical-only filters where it matters (jaggies on diagonals).
#[test]
fn edi_beats_bob_on_moving_diagonal_edges() {
    for std in VideoStandard::ALL {
        let p = std.params();
        let (w, h) = (p.active_width(), p.frame_height());
        let pics: Vec<YuvImage> = (0..FIELDS)
            .map(|t| {
                let mut img = YuvImage::new(w, h);
                for y in 0..h {
                    for x in 0..w {
                        // Edges at about 14 degrees from horizontal, moving
                        // right 3 samples per field, smooth (band-limited)
                        // profile.
                        let u = (x as f64 - 3.0 * t as f64) * 0.25 - y as f64;
                        let ph = u.rem_euclid(48.0) - 24.0;
                        let v = 0.5 + 0.4 * (ph.abs() - 12.0).clamp(-1.2, 1.2) / 1.2;
                        img.y[y * w + x] = v as f32;
                    }
                }
                img
            })
            .collect();
        let edi = run(std, DeinterlaceMethod::EdgeDirectedMA, &pics);
        let bob = run(std, DeinterlaceMethod::Bob, &pics);
        println!("{std:?} moving diagonal edges: edi-ma {edi:.2} dB, bob {bob:.2} dB");
        assert!(edi > bob + 1.0, "{std:?} edi {edi} bob {bob}");
    }
}

#[test]
fn latency_reported() {
    for m in DeinterlaceMethod::ALL {
        let d = create(m, VideoStandard::Ntsc);
        assert_eq!(d.latency_fields(), if m.is_bidirectional() { 1 } else { 0 });
    }
}
