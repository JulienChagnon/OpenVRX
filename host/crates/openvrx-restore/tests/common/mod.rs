// SPDX-License-Identifier: GPL-3.0-or-later
//! Synthetic ground truth and analog-FPV-like damage for the tests.

#![allow(dead_code)]

use openvrx_core::{damage, Frame};
use openvrx_restore::picture::{psnr, ssim};

/// SplitMix64.
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }
    pub fn range(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next_u64() % (hi - lo) as u64) as usize
    }
}

const GLYPHS: [[u8; 7]; 5] = [
    [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
    [0b01111, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b01111],
    [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
    [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
    [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
];

/// Scene colour at a continuous position and time: a panning textured
/// background, a rotating spoked disc and moving "OpenVRX" text.
fn scene(x: f32, y: f32, t: f32, w: f32, h: f32) -> (f32, f32, f32) {
    let u = x + 2.3 * t;
    let v = y + 0.7 * t;
    let mut ly = 0.45
        + 0.18 * (0.11 * u).sin() * (0.07 * v).cos()
        + 0.08 * (0.031 * u + 0.043 * v).sin()
        + 0.06 * (4.0 * (0.25 * u).sin() * (0.25 * v).sin()).tanh();
    let mut lu = 0.08 * (0.02 * u).sin();
    let mut lv = 0.06 * (0.017 * v).cos();
    let (cx, cy, r) = (0.65 * w, 0.55 * h, 0.22 * h);
    let (dx, dy) = (x - cx, y - cy);
    let d = (dx * dx + dy * dy).sqrt();
    if d < r {
        let a = dy.atan2(dx) + 0.06 * t;
        ly = 0.3 + 0.35 * (0.5 + 0.5 * (6.0 * a).cos()) + 0.15 * (0.5 * d).sin();
        lu = -0.1;
        lv = 0.12;
    }
    let scale = 4.0;
    let tx = x - (10.0 + 3.0 * t);
    let ty = y - 0.12 * h;
    if tx >= 0.0 && ty >= 0.0 {
        let gx = (tx / scale) as usize;
        let gy = (ty / scale) as usize;
        let ch = gx / 6;
        let col = gx % 6;
        if ch < 5 && col < 5 && gy < 7 && GLYPHS[ch][gy] & (1 << (4 - col)) != 0 {
            ly = 0.95;
            lu = 0.0;
            lv = 0.0;
        }
    }
    (ly, lu, lv)
}

/// Ground-truth progressive sequence.
pub fn truth_sequence(w: usize, h: usize, n: usize) -> Vec<Frame> {
    (0..n)
        .map(|t| {
            let mut f = Frame::new(w, h);
            for yy in 0..h {
                for xx in 0..w {
                    let (mut a, mut b, mut c) = (0.0, 0.0, 0.0);
                    for (ox, oy) in [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)] {
                        let s = scene(xx as f32 + ox, yy as f32 + oy, t as f32, w as f32, h as f32);
                        a += s.0;
                        b += s.1;
                        c += s.2;
                    }
                    let i = yy * w + xx;
                    f.y[i] = a / 4.0;
                    f.u[i] = b / 4.0;
                    f.v[i] = c / 4.0;
                }
            }
            f.meta.seq = t as u64;
            f
        })
        .collect()
}

/// Bursts of noise lines (partial or full width) on about half the frames,
/// marked `SEVERE` in the mask.
pub fn add_line_bursts(frames: &mut [Frame], rng: &mut Rng) {
    for f in frames.iter_mut() {
        if rng.f32() < 0.5 {
            continue;
        }
        let (w, h) = (f.width, f.height);
        for _ in 0..rng.range(1, 3) {
            let r0 = rng.range(0, h - 12);
            let len = rng.range(2, 11);
            let (x0, x1) = if rng.f32() < 0.5 {
                (0, w)
            } else {
                let a = rng.range(0, w / 2);
                (a, (a + rng.range(w / 3, w / 2 + 1)).min(w))
            };
            for r in r0..r0 + len {
                let mut prev = rng.f32();
                for x in x0..x1 {
                    let n = rng.f32();
                    let i = r * w + x;
                    f.y[i] = 0.5 * (n + prev);
                    prev = n;
                    f.u[i] = 0.4 * (rng.f32() - 0.5);
                    f.v[i] = 0.4 * (rng.f32() - 0.5);
                    f.damage[i] = damage::SEVERE;
                }
            }
        }
    }
}

/// FM click sparkles: short one-line white or black streaks, marked `CLICK`.
pub fn add_sparkles(frames: &mut [Frame], per_frame: usize, rng: &mut Rng) {
    for f in frames.iter_mut() {
        let (w, h) = (f.width, f.height);
        for _ in 0..per_frame {
            let r = rng.range(0, h);
            let x0 = rng.range(0, w - 8);
            let val = if rng.f32() < 0.6 { 1.0 } else { 0.0 };
            for x in x0..x0 + rng.range(2, 7) {
                let i = r * w + x;
                f.y[i] = val;
                f.damage[i] = damage::CLICK;
            }
        }
    }
}

/// Replaces whole frames with static, marked `LOST`.
pub fn add_lost_frames(frames: &mut [Frame], which: &[usize], rng: &mut Rng) {
    for &k in which {
        let f = &mut frames[k];
        for i in 0..f.y.len() {
            f.y[i] = rng.f32();
            f.u[i] = 0.3 * (rng.f32() - 0.5);
            f.v[i] = 0.3 * (rng.f32() - 0.5);
            f.damage[i] = damage::LOST;
        }
    }
}

/// Luma PSNR over the whole sequence (global MSE) and mean SSIM.
pub fn quality(a: &[Frame], b: &[Frame]) -> (f64, f64) {
    assert_eq!(a.len(), b.len());
    let ya: Vec<f32> = a.iter().flat_map(|f| f.y.iter().copied()).collect();
    let yb: Vec<f32> = b.iter().flat_map(|f| f.y.iter().copied()).collect();
    let s = a
        .iter()
        .zip(b)
        .map(|(p, q)| ssim(&p.y, &q.y, p.width, p.height))
        .sum::<f64>()
        / a.len() as f64;
    (psnr(&ya, &yb), s)
}
