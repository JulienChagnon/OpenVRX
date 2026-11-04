// SPDX-License-Identifier: GPL-3.0-or-later
//! Repair safety rules and concealment quality on decoded fields.

use openvrx_core::quality::damage;
use openvrx_core::{VideoStandard, YuvField};
use openvrx_video::metrics::{mse, Region};
use openvrx_video::repair::{RepairConfig, Repairer};
use openvrx_video::testgen::{field_info, image_to_field, Scene};
use openvrx_video::util::Rng;

const STD: VideoStandard = VideoStandard::Pal;

fn field(scene: &Scene, seq: u64, t: f64) -> YuvField {
    let p = STD.params();
    let pic = scene.render(p.active_width(), p.frame_height(), t);
    image_to_field(&pic, STD, field_info(seq).0, seq)
}

/// Corrupts samples (+-0.5 luma) and flags them.
fn corrupt(f: &mut YuvField, k: usize, len: usize) {
    for i in k..k + len {
        f.y[i] += if i % 2 == 0 { 0.5 } else { -0.5 };
        f.u[i] = 0.3;
        f.damage[i] = damage::CLICK;
    }
}

#[test]
fn only_flagged_samples_change_and_clicks_are_concealed() {
    let scene = Scene::new(8, (0.0, 0.0), (0.0, 0.0));
    let mut rep = Repairer::new(STD, &RepairConfig::default());
    let mut rng = Rng::new(2);
    for seq in 0..6u64 {
        let truth = field(&scene, seq, 0.0);
        let mut f = truth.clone();
        for _ in 0..200 {
            let k = rng.below((f.y.len() - 8) as u64) as usize;
            corrupt(&mut f, k, 1 + rng.below(4) as usize);
        }
        let out = rep.repair_field(&f);
        for i in 0..f.y.len() {
            if f.damage[i] < damage::CLICK {
                assert_eq!((out.y[i], out.u[i], out.v[i]), (f.y[i], f.u[i], f.v[i]), "unflagged sample {i} changed");
            } else {
                assert!(out.damage[i] <= damage::REPAIRED);
            }
        }
        let r = Region::full(f.width, f.lines);
        let before = mse(&f.y, &truth.y, f.width, r);
        let after = mse(&out.y, &truth.y, f.width, r);
        let gain = 10.0 * (before / after).log10();
        println!("field {seq}: concealment gain {gain:.1} dB");
        if seq >= 2 {
            // Static scene: the previous same-parity field fills exactly.
            assert!(gain > 25.0, "field {seq}: concealment gain {gain:.1} dB");
        } else {
            assert!(gain > 10.0, "field {seq}: spatial concealment gain {gain:.1} dB");
        }
    }
}

#[test]
fn past_fill_stops_after_three_fields() {
    let scene = Scene::new(8, (0.0, 0.0), (0.0, 0.0));
    let cfg = RepairConfig::default();
    let mut rep = Repairer::new(STD, &cfg);
    let w = STD.params().active_width();
    // Field lines 40..45 lost in fields 2..=9, clean again from field 10.
    let mut filled = Vec::new();
    for seq in 0..12u64 {
        let truth = field(&scene, seq, 0.0);
        let mut f = truth.clone();
        let damaged = (2..=9).contains(&seq);
        if damaged {
            corrupt(&mut f, 40 * w, 5 * w);
        }
        let out = rep.repair_field(&f);
        if damaged {
            let k = 42 * w + w / 2;
            filled.push(out.y[k] != f.y[k]);
        }
    }
    let mut run = 0;
    let mut max_run = 0;
    for &fl in &filled {
        run = if fl { run + 1 } else { 0 };
        max_run = max_run.max(run);
    }
    assert!(max_run <= cfg.max_past_fields as usize, "filled {filled:?}");
    assert!(filled.iter().filter(|&&f| f).count() >= 3, "filled {filled:?}");
    assert!(!filled[filled.len() - 1], "still filling at the end: {filled:?}");
    // After clean fields the counter is back at zero.
    assert_eq!(rep.block_count(42, w / 2), 0);
}

#[test]
fn moving_dropout_uses_spatial_not_stale_past() {
    let scene = Scene::new(8, (6.0, 0.0), (0.0, 0.0));
    let mut rep = Repairer::new(STD, &RepairConfig::default());
    let w = STD.params().active_width();
    let mut last = (0.0, 0.0);
    for seq in 0..6u64 {
        let truth = field(&scene, seq, seq as f64);
        let mut f = truth.clone();
        corrupt(&mut f, 100 * w, w);
        let out = rep.repair_field(&f);
        let r = Region { x0: 16, x1: w - 16, y0: 100, y1: 101 };
        last = (mse(&out.y, &truth.y, w, r), mse(&f.y, &truth.y, w, r));
    }
    assert!(rep.last_field_stats().spatial_samples > 0);
    assert!(10.0 * (last.1 / last.0).log10() > 15.0, "{last:?}");
}
