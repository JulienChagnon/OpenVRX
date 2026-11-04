// SPDX-License-Identifier: GPL-3.0-or-later
//! Mask-driven restoration of synthetic sequences against ground truth.

mod common;

use common::{add_line_bursts, add_lost_frames, add_sparkles, quality, truth_sequence, Rng};
use openvrx_core::{damage, FieldParity, VideoStandard, YuvField};
use openvrx_restore::{restore_fields, restore_frames, RestoreConfig};
use std::time::Instant;

const W: usize = 320;
const H: usize = 240;
const N: usize = 24;

#[test]
fn mask_driven_rebuild_improves_psnr_and_keeps_clean_pixels() {
    let truth = truth_sequence(W, H, N);
    let mut rng = Rng(0x5eed);
    let mut bursts = truth.clone();
    add_line_bursts(&mut bursts, &mut rng);
    add_sparkles(&mut bursts, 40, &mut rng);
    let mut lost = bursts.clone();
    add_lost_frames(&mut lost, &[7, 15, 16], &mut rng);

    // (case, damaged input, preset, min restored PSNR, min restored SSIM)
    let cases = [
        ("bursts + sparkles", &bursts, 38.0, 0.985),
        ("bursts + sparkles + 3 lost frames", &lost, 30.0, 0.97),
    ];
    println!("| damage | preset | damaged PSNR/SSIM | restored PSNR/SSIM | fps |");
    for (name, input, min_psnr, min_ssim) in cases {
        let (dp, ds) = quality(input, &truth);
        for preset in ["fast", "balanced", "max"] {
            let cfg = RestoreConfig::preset(preset).unwrap();
            let t0 = Instant::now();
            let out = restore_frames(input, &cfg).unwrap();
            let fps = N as f64 / t0.elapsed().as_secs_f64();
            assert_eq!(out.len(), N);
            let (rp, rs) = quality(&out, &truth);
            println!("| {name} | {preset} | {dp:.1} dB / {ds:.3} | {rp:.1} dB / {rs:.3} | {fps:.0} |");
            assert!(rp >= min_psnr && rp > dp + 10.0, "{name}/{preset}: PSNR {rp:.2} (damaged {dp:.2})");
            assert!(rs >= min_ssim, "{name}/{preset}: SSIM {rs:.4}");
            // Unflagged pixels are bit-exact (denoise is off by default).
            for (o, i) in out.iter().zip(input.iter()) {
                for k in 0..o.y.len() {
                    if i.damage[k] == damage::NONE {
                        assert_eq!(o.y[k], i.y[k]);
                        assert_eq!(o.u[k], i.u[k]);
                    }
                }
                assert!(o.meta.pipeline.ends_with("doctor"));
            }
        }
    }
}

#[test]
fn fields_use_same_parity_neighbours() {
    // Weave-free field sequence: alternate parities, each field the even or
    // odd rows of the truth frame at field rate.
    let truth = truth_sequence(W, H, 16);
    let mut rng = Rng(77);
    let mut damaged = truth.clone();
    add_line_bursts(&mut damaged, &mut rng);
    add_sparkles(&mut damaged, 20, &mut rng);
    let to_field = |f: &openvrx_core::Frame, k: usize| {
        let parity = if k.is_multiple_of(2) { FieldParity::First } else { FieldParity::Second };
        let mut fl = YuvField::new(VideoStandard::Ntsc, k as u64, parity);
        fl.width = W;
        fl.lines = H / 2;
        let rows = |p: &[f32]| -> Vec<f32> {
            (0..H / 2).flat_map(|r| p[(2 * r + k % 2) * W..][..W].iter().copied()).collect()
        };
        fl.y = rows(&f.y);
        fl.u = rows(&f.u);
        fl.v = rows(&f.v);
        fl.damage = (0..H / 2).flat_map(|r| f.damage[(2 * r + k % 2) * W..][..W].iter().copied()).collect();
        fl.quality.truncate(H / 2);
        fl.source.truncate(H / 2);
        fl
    };
    let fields_in: Vec<YuvField> = damaged.iter().enumerate().map(|(k, f)| to_field(f, k)).collect();
    let fields_truth: Vec<YuvField> = truth.iter().enumerate().map(|(k, f)| to_field(f, k)).collect();
    let out = restore_fields(&fields_in, &RestoreConfig::balanced()).unwrap();
    let flat = |v: &[YuvField]| v.iter().flat_map(|f| f.y.iter().copied()).collect::<Vec<f32>>();
    let before = openvrx_restore::picture::psnr(&flat(&fields_in), &flat(&fields_truth));
    let after = openvrx_restore::picture::psnr(&flat(&out), &flat(&fields_truth));
    println!("fields: {before:.1} dB -> {after:.1} dB");
    assert!(after > before + 12.0 && after > 34.0, "{before} -> {after}");
}
