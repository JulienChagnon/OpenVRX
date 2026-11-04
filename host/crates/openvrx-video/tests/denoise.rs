// SPDX-License-Identifier: GPL-3.0-or-later
//! Temporal denoise: gain on static and moving noisy sequences without
//! smearing motion, damage never blended, scene cuts reset.

use openvrx_core::quality::damage;
use openvrx_core::{Frame, VideoStandard};
use openvrx_video::denoise::{DenoiseConfig, Denoiser, TemporalMode};
use openvrx_video::metrics::{psnr, Region};
use openvrx_video::testgen::{image_to_frame, Scene};
use openvrx_video::util::Rng;

const STD: VideoStandard = VideoStandard::Pal;

fn noisy(f: &Frame, sigma: f32, rng: &mut Rng) -> Frame {
    let mut o = f.clone();
    for v in o.y.iter_mut() {
        *v += sigma * rng.gauss() as f32;
    }
    for v in o.u.iter_mut().chain(o.v.iter_mut()) {
        *v += 0.7 * sigma * rng.gauss() as f32;
    }
    for q in o.row_quality.iter_mut() {
        q.noise_ire = sigma * 100.0;
    }
    o
}

/// Mean luma PSNR gain (dB) over frames 6.. for the whole frame and for
/// the moving disc.
fn gain(mode: TemporalMode, pan: (f64, f64), object: (f64, f64)) -> (f64, f64) {
    let p = STD.params();
    let scene = Scene::new(12, pan, object);
    let cfg = DenoiseConfig {
        temporal: mode,
        ..DenoiseConfig::default()
    };
    let mut d = Denoiser::new(STD, &cfg);
    let mut rng = Rng::new(4);
    let (mut g, mut gm, mut n) = (0.0, 0.0, 0.0);
    for t in 0..14 {
        let truth = image_to_frame(&scene.render(p.active_width(), p.frame_height(), t as f64));
        let inp = noisy(&truth, 0.03, &mut rng);
        let out = d.process(&inp);
        if t >= 6 {
            let r = Region::inset(truth.width, truth.height, 24, 12);
            g += psnr(&out.y, &truth.y, truth.width, r, 1.0) - psnr(&inp.y, &truth.y, truth.width, r, 1.0);
            let cx = (truth.width as f64 * 0.3 + object.0 * t as f64) as usize;
            let cy = (truth.height as f64 * 0.5 + object.1 * t as f64) as usize;
            let rr = Region {
                x0: cx - 30,
                x1: cx + 30,
                y0: cy - 30,
                y1: cy + 30,
            };
            gm += psnr(&out.y, &truth.y, truth.width, rr, 1.0) - psnr(&inp.y, &truth.y, truth.width, rr, 1.0);
            n += 1.0;
        }
    }
    (g / n, gm / n)
}

#[test]
fn gains_without_smearing() {
    println!("\nluma PSNR gain (dB) at 3 % noise: whole frame / moving disc");
    let cases = [
        ("static", (0.0, 0.0), (0.0, 0.0)),
        ("object", (0.0, 0.0), (4.0, 1.0)),
        ("pan", (2.0, 0.5), (4.0, 1.0)),
    ];
    for (name, pan, obj) in cases {
        let r = gain(TemporalMode::Recursive, pan, obj);
        let m = gain(TemporalMode::MotionCompensated, pan, obj);
        println!("  {name:<7} recursive {:.2} / {:.2}   mc {:.2} / {:.2}", r.0, r.1, m.0, m.1);
        match name {
            "static" => assert!(r.0 > 4.0 && m.0 > 4.0, "{r:?} {m:?}"),
            "object" => {
                assert!(r.0 > 3.0, "{r:?}");
                // The moving object is not smeared: never worse than input.
                assert!(r.1 > -0.3 && m.1 > -0.3, "{r:?} {m:?}");
            }
            _ => {
                assert!(r.0 > -0.3, "{r:?}");
                assert!(m.0 > 1.5, "mc on pan {m:?}");
            }
        }
    }
}

#[test]
fn damage_never_blended_and_scene_cut_resets() {
    let p = STD.params();
    let a = Scene::new(1, (0.0, 0.0), (0.0, 0.0));
    let b = Scene::new(2, (0.0, 0.0), (0.0, 0.0));
    let mut d = Denoiser::new(STD, &DenoiseConfig::default());
    let mut rng = Rng::new(9);
    let fa = image_to_frame(&a.render(p.active_width(), p.frame_height(), 0.0));
    let fb = image_to_frame(&b.render(p.active_width(), p.frame_height(), 0.0));
    for _ in 0..5 {
        d.process(&noisy(&fa, 0.02, &mut rng));
    }
    let mut inp = noisy(&fa, 0.02, &mut rng);
    for k in 1000..1100 {
        inp.damage[k] = damage::SEVERE;
    }
    let out = d.process(&inp);
    for k in 1000..1100 {
        assert_eq!(out.y[k], inp.y[k]);
    }
    let cut = noisy(&fb, 0.02, &mut rng);
    let out = d.process(&cut);
    let r = Region::inset(fb.width, fb.height, 0, 40);
    assert!(d.stats().scene_cut);
    let ghost = psnr(&out.y, &cut.y, fb.width, r, 1.0);
    assert!(ghost > 60.0, "scene cut ghosting: {ghost}");
}
