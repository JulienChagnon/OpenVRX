// SPDX-License-Identifier: GPL-3.0-or-later
//! Colour decoding accuracy: colour bars per separation method, cross
//! colour on a luma zone plate, cross luma on a chroma sweep, colour
//! killer, ACC, sub-carrier phase offsets and noise.

mod common;

use common::*;
use openvrx_core::{VideoStandard, YuvField};
use openvrx_video::color::{ColorConfig, ColorDecoder, NtscSeparation, PalSeparation};
use openvrx_video::encoder::{CompositeEncoder, EncoderConfig};
use openvrx_video::metrics::{chroma_power, color_error, mse, Region};
use openvrx_video::testgen::{add_noise, chroma_sweep, color_bars, zone_plate, YuvImage};
use openvrx_video::util::Rng;
use std::sync::Arc;

#[derive(Copy, Clone, Debug)]
enum M {
    N(NtscSeparation),
    P(PalSeparation),
}

fn methods(std: VideoStandard) -> Vec<M> {
    match std {
        VideoStandard::Ntsc => vec![M::N(NtscSeparation::Notch), M::N(NtscSeparation::Comb2D), M::N(NtscSeparation::Comb3D)],
        VideoStandard::Pal => vec![
            M::P(PalSeparation::Notch),
            M::P(PalSeparation::DelayLine),
            M::P(PalSeparation::Comb2D),
            M::P(PalSeparation::Comb3D),
            M::P(PalSeparation::Transform2D),
            M::P(PalSeparation::Transform3D),
        ],
    }
}

fn cfg_for(m: M) -> ColorConfig {
    let mut c = ColorConfig::default();
    match m {
        M::N(s) => c.ntsc = s,
        M::P(s) => c.pal = s,
    }
    c
}

/// Decodes `n` fields of a static picture (with optional noise) and
/// returns the last decoded field with its truth.
fn decode_static(std: VideoStandard, enc: &CompositeEncoder, cfg: &ColorConfig, pic: &YuvImage, n: u64, noise: f32) -> (YuvField, YuvField) {
    let mut dec = ColorDecoder::new(std, cfg);
    let mut rng = Rng::new(7);
    let mut last = None;
    for seq in 0..n {
        let mut recs = encode(enc, pic, seq);
        if noise > 0.0 {
            for r in &mut recs {
                add_noise(r, noise, &mut rng);
            }
        }
        let out = dec.decode_field(Arc::new(to_field(std, &recs)));
        last = Some((out, truth_field(pic, std, seq)));
    }
    last.unwrap()
}

/// Bar centres: middle 50 % of each bar, lines away from the top/bottom.
fn bar_errors(out: &YuvField, truth: &YuvField) -> (f64, f64, f64) {
    let w = out.width;
    let mut worst_uv: f64 = 0.0;
    let mut worst_y: f64 = 0.0;
    let mut mean_uv = 0.0;
    for bar in 0..8 {
        let x0 = bar * w / 8 + w / 32;
        let x1 = (bar + 1) * w / 8 - w / 32;
        let r = Region { x0, x1, y0: 8, y1: out.lines - 8 };
        let e = color_error(&out.y, &out.u, &out.v, &truth.y, &truth.u, &truth.v, w, r);
        worst_uv = worst_uv.max(e.mean_uv);
        worst_y = worst_y.max(e.mean_y);
        mean_uv += e.mean_uv / 8.0;
    }
    (worst_y, worst_uv, mean_uv)
}

#[test]
fn colour_bars_all_methods() {
    println!("\ncolour bars 75 %: worst bar mean |dY|, worst bar mean |dUV|");
    for std in VideoStandard::ALL {
        let p = std.params();
        let enc = encoder(std);
        let bars = color_bars(p.active_width(), p.frame_height(), 0.75);
        for m in methods(std) {
            let (out, truth) = decode_static(std, &enc, &cfg_for(m), &bars, 10, 0.0);
            let (ey, euv, _) = bar_errors(&out, &truth);
            println!("  {:?} {:<28} dY {:.4}  dUV {:.4}", std, format!("{m:?}"), ey, euv);
            assert!(ey < 0.006, "{std:?} {m:?} luma error {ey}");
            assert!(euv < 0.008, "{std:?} {m:?} chroma error {euv}");
        }
    }
}

#[test]
fn cross_colour_and_cross_luma() {
    println!("
cross colour = chroma RMS on luma-only zone plates (A: low vertical");
    println!("frequency, B: circular), cross luma = luma RMS error on a chroma sweep");
    for std in VideoStandard::ALL {
        let p = std.params();
        let enc = CompositeEncoder::new(
            std,
            EncoderConfig {
                luma_bandwidth_hz: if std == VideoStandard::Ntsc { 4.2e6 } else { 5.0e6 },
                ..EncoderConfig::default()
            },
        );
        let (w, h) = (p.active_width(), p.frame_height());
        let zpa = zone_plate(w, h, 0.32, 0.03, 0.35);
        let zpb = zone_plate(w, h, 0.32, 0.1, 0.35);
        let cs = chroma_sweep(w, h, 1.0);
        let mut results = Vec::new();
        for m in methods(std) {
            let cfg = cfg_for(m);
            let cc = |pic: &YuvImage| {
                let (o, _) = decode_static(std, &enc, &cfg, pic, 10, 0.0);
                let reg = Region::inset(o.width, o.lines, 24, 6);
                chroma_power(&o.u, &o.v, o.width, reg).sqrt()
            };
            let (cca, ccb) = (cc(&zpa), cc(&zpb));
            let (co, ct) = decode_static(std, &enc, &cfg, &cs, 10, 0.0);
            let reg = Region::inset(co.width, co.lines, 24, 6);
            let cl = mse(&co.y, &ct.y, co.width, reg).sqrt();
            println!("  {:?} {:<28} cross-colour A {:.4} B {:.4}  cross-luma {:.4}", std, format!("{m:?}"), cca, ccb, cl);
            results.push((m, cca, ccb, cl));
        }
        let notch = results[0];
        for &(m, cca, ccb, cl) in &results[1..] {
            let three_d = matches!(m, M::N(NtscSeparation::Comb3D) | M::P(PalSeparation::Comb3D) | M::P(PalSeparation::Transform3D));
            let transform = matches!(m, M::P(PalSeparation::Transform2D) | M::P(PalSeparation::Transform3D));
            match m {
                M::P(PalSeparation::DelayLine) => {
                    // PAL-D shares the 1D separation; line averaging only
                    // reduces cross colour, it cannot remove it.
                    assert!(cca <= notch.1 && ccb <= notch.2, "{m:?} {cca} {ccb} vs notch");
                }
                _ => {
                    assert!(cca < 0.5 * notch.1, "{m:?} cross colour A {cca} vs notch {}", notch.1);
                    assert!(cl < 0.5 * notch.3 || cl < 0.002, "{m:?} cross luma {cl} vs notch {}", notch.3);
                }
            }
            if three_d || transform {
                assert!(ccb < 0.5 * notch.2, "{m:?} cross colour B {ccb} vs notch {}", notch.2);
            }
            if three_d {
                assert!(cca < 0.05 * notch.1 && ccb < 0.1 * notch.2, "{m:?} static 3D {cca} {ccb}");
            }
        }
    }
}

#[test]
fn monochrome_camera_gives_full_band_luma() {
    for std in VideoStandard::ALL {
        let p = std.params();
        let enc = CompositeEncoder::new(
            std,
            EncoderConfig {
                color: false,
                ..EncoderConfig::default()
            },
        );
        let zp = zone_plate(p.active_width(), p.frame_height(), 0.3, 0.1, 0.35);
        let (out, truth) = decode_static(std, &enc, &ColorConfig::default(), &zp, 8, 0.0);
        let reg = Region::inset(out.width, out.lines, 24, 6);
        let cc = chroma_power(&out.u, &out.v, out.width, reg).sqrt();
        let ly = mse(&out.y, &truth.y, out.width, reg).sqrt();
        assert!(cc < 1e-3, "{std:?} killer leaves chroma {cc}");
        assert!(ly < 0.01, "{std:?} luma error with killer {ly}");
    }
}

#[test]
fn acc_and_phase_offset() {
    // Burst and chroma attenuated by 6 dB plus an arbitrary subcarrier
    // phase: ACC restores the saturation, the burst PLL the hue.
    for std in VideoStandard::ALL {
        let p = std.params();
        let enc = CompositeEncoder::new(
            std,
            EncoderConfig {
                subcarrier_phase_deg: 137.0,
                ..EncoderConfig::default()
            },
        );
        let bars = color_bars(p.active_width(), p.frame_height(), 0.75);
        let mut dec = ColorDecoder::new(std, &ColorConfig::default());
        let mut last = None;
        let bp = p.burst_range();
        let a = p.active_range();
        for seq in 0..10u64 {
            let mut recs = encode(&enc, &bars, seq);
            for r in &mut recs {
                // Halve the chroma: subtract half the high-passed signal in
                // the burst and active ranges (decoded later at half
                // amplitude without ACC).
                let lp: Vec<f32> = r.samples.clone();
                let mut smooth = vec![0.0f32; lp.len()];
                let k = openvrx_video::util::design_lowpass(0.12, 31);
                openvrx_video::util::fir_range(&lp, &k, 0..lp.len(), &mut smooth);
                for x in bp.start..bp.end {
                    r.samples[x] = smooth[x] + 0.5 * (lp[x] - smooth[x]);
                }
                for x in a.clone() {
                    r.samples[x] = smooth[x] + 0.5 * (lp[x] - smooth[x]);
                }
            }
            let out = dec.decode_field(Arc::new(to_field(std, &recs)));
            last = Some((out, truth_field(&bars, std, seq)));
        }
        let (out, truth) = last.unwrap();
        let (_, euv, _) = bar_errors(&out, &truth);
        assert!(euv < 0.02, "{std:?} ACC/phase chroma error {euv}");
    }
}

#[test]
fn noisy_bars_stay_coloured_and_accurate() {
    // 2.5 IRE RMS noise (about 32 dB unweighted SNR).
    for std in VideoStandard::ALL {
        let p = std.params();
        let enc = encoder(std);
        let bars = color_bars(p.active_width(), p.frame_height(), 0.75);
        let cfg = ColorConfig::default();
        let (out, truth) = decode_static(std, &enc, &cfg, &bars, 10, 2.5);
        let (_, _, mean_uv) = bar_errors(&out, &truth);
        assert!(mean_uv < 0.03, "{std:?} noisy chroma error {mean_uv}");
    }
}
