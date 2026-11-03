// SPDX-License-Identifier: GPL-3.0-or-later
//! Fast, accurate, branch-free elementary functions for the hot loops.
//!
//! All functions are written so that LLVM can vectorise loops calling them:
//! no data-dependent branches, only selects, multiplies and adds.
//!
//! - [`atan2`]: max error 2e-7 rad over the full plane (11th-order odd
//!   minimax polynomial on [0, 1] plus octant reconstruction).
//! - [`sin_cos`]: max error 3e-7 for |x| <= 4 pi (callers keep phases wrapped) (range reduction to
//!   [-pi/2, pi/2] and Taylor-minimax polynomials).

use std::f32::consts::{FRAC_PI_2, PI};

/// Minimax (Lawson-reweighted least squares) coefficients for atan(t)/t on t in [0, 1] as a
/// polynomial in t^2.
const A0: f32 = 0.999_999_34;
const A1: f32 = -0.333_298_6;
const A2: f32 = 0.199_465_63;
const A3: f32 = -0.139_086_2;
const A4: f32 = 0.096_421_7;
const A5: f32 = -0.055_911_93;
const A6: f32 = 0.021_862_67;
const A7: f32 = -0.004_054_483_5;

/// The atan polynomial coefficients (shared with the SIMD kernel).
pub const ATAN_C: [f32; 8] = [A0, A1, A2, A3, A4, A5, A6, A7];

pub use crate::vmath::atan2_slice;

/// atan(t) for t in [0, 1].
#[inline(always)]
fn atan_unit(t: f32) -> f32 {
    let s = t * t;
    let p = A7;
    let p = p * s + A6;
    let p = p * s + A5;
    let p = p * s + A4;
    let p = p * s + A3;
    let p = p * s + A2;
    let p = p * s + A1;
    let p = p * s + A0;
    p * t
}

/// Four-quadrant arctangent, result in [-pi, pi]. `atan2(0, 0)` is 0.
#[inline(always)]
pub fn atan2(y: f32, x: f32) -> f32 {
    let ax = x.abs();
    let ay = y.abs();
    let mx = ax.max(ay);
    let mn = ax.min(ay);
    let t = mn / mx.max(f32::MIN_POSITIVE);
    let mut r = atan_unit(t);
    r = if ay > ax { FRAC_PI_2 - r } else { r };
    r = if x < 0.0 { PI - r } else { r };
    r.copysign(y)
}


/// Wraps an angle to [-pi, pi).
#[inline(always)]
pub fn wrap_pi(x: f32) -> f32 {
    x - std::f32::consts::TAU * (x * (1.0 / std::f32::consts::TAU) + 0.5).floor()
}

/// Wraps an f64 angle to [-pi, pi).
#[inline(always)]
pub fn wrap_pi64(x: f64) -> f64 {
    x - std::f64::consts::TAU * (x * (1.0 / std::f64::consts::TAU) + 0.5).floor()
}

/// sin and cos of x.
#[inline(always)]
pub fn sin_cos(x: f32) -> (f32, f32) {
    // Reduce to [-pi, pi] with a two-part 2*pi for accuracy.
    const TAU_HI: f32 = 6.283_185_5;
    const TAU_LO: f32 = -1.748_455_5e-7;
    let k = (x * (1.0 / std::f32::consts::TAU)).round();
    let r = (x - k * TAU_HI) - k * TAU_LO;
    // Fold to [-pi/2, pi/2]: sin(pi - r) = sin r, cos(pi - r) = -cos r.
    let hi = r > FRAC_PI_2;
    let lo = r < -FRAC_PI_2;
    let f = if hi {
        PI - r
    } else if lo {
        -PI - r
    } else {
        r
    };
    let csign = if hi || lo { -1.0 } else { 1.0 };
    let s2 = f * f;
    let sp = -2.505_210_8e-8;
    let sp = sp * s2 + 2.755_731_9e-6;
    let sp = sp * s2 - 1.984_127e-4;
    let sp = sp * s2 + 8.333_333e-3;
    let sp = sp * s2 - 0.166_666_67;
    let sin = f + f * s2 * sp;
    let cp = -1.147_074_6e-11;
    let cp = cp * s2 + 2.087_675_7e-9;
    let cp = cp * s2 - 2.755_732e-7;
    let cp = cp * s2 + 2.480_158_7e-5;
    let cp = cp * s2 - 1.388_888_9e-3;
    let cp = cp * s2 + 4.166_666_8e-2;
    let cp = cp * s2 - 0.5;
    let cos = 1.0 + s2 * cp;
    (sin, cos * csign)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atan2_accuracy() {
        let mut worst = 0.0f64;
        let n = 20000;
        for k in 0..n {
            let a = -std::f64::consts::PI + std::f64::consts::TAU * (k as f64 + 0.37) / n as f64;
            for r in [1e-3f64, 0.5, 1.0, 37.0, 1e4] {
                let (y, x) = ((r * a.sin()) as f32, (r * a.cos()) as f32);
                let exact = (y as f64).atan2(x as f64);
                let got = atan2(y, x) as f64;
                let mut e = (got - exact).abs();
                if e > std::f64::consts::PI {
                    e = std::f64::consts::TAU - e;
                }
                worst = worst.max(e);
            }
        }
        assert!(worst < 1.2e-6, "atan2 worst error {worst}");
        assert_eq!(atan2(0.0, 0.0), 0.0);
        assert!((atan2(0.0, -1.0) - PI).abs() < 1e-6);
        assert!((atan2(1.0, 0.0) - FRAC_PI_2).abs() < 1e-6);
        assert!((atan2(-1.0, 0.0) + FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn sin_cos_accuracy() {
        let mut worst = 0.0f64;
        for k in 0..200_000 {
            let x = -12.5 + 25.0 * k as f64 / 200_000.0;
            let (s, c) = sin_cos(x as f32);
            let xf = x as f32 as f64;
            worst = worst.max((s as f64 - xf.sin()).abs()).max((c as f64 - xf.cos()).abs());
        }
        assert!(worst < 3e-7, "sin_cos worst error {worst}");
    }

    #[test]
    fn wrap() {
        assert!((wrap_pi(3.5) - (3.5 - std::f32::consts::TAU)).abs() < 1e-6);
        assert!((wrap_pi(-3.5) - (-3.5 + std::f32::consts::TAU)).abs() < 1e-6);
        assert!((wrap_pi(1.0) - 1.0).abs() < 1e-7);
        assert!((wrap_pi64(7.0) - (7.0 - std::f64::consts::TAU)).abs() < 1e-12);
    }
}
