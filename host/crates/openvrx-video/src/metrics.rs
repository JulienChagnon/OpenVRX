// SPDX-License-Identifier: GPL-3.0-or-later
//! Picture quality measures: PSNR, SSIM and colour error, with optional
//! regions so tests can exclude picture edges.

use openvrx_core::Frame;

/// Rectangle `[x0, x1) x [y0, y1)` in plane coordinates.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Region {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize,
}

impl Region {
    pub fn full(width: usize, height: usize) -> Region {
        Region {
            x0: 0,
            y0: 0,
            x1: width,
            y1: height,
        }
    }

    /// The full plane minus a border of `bx` columns and `by` rows.
    pub fn inset(width: usize, height: usize, bx: usize, by: usize) -> Region {
        Region {
            x0: bx,
            y0: by,
            x1: width - bx,
            y1: height - by,
        }
    }

    pub fn len(&self) -> usize {
        (self.x1 - self.x0) * (self.y1 - self.y0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Mean squared error between two planes over a region.
pub fn mse(a: &[f32], b: &[f32], width: usize, r: Region) -> f64 {
    let mut acc = 0.0f64;
    for y in r.y0..r.y1 {
        let ra = &a[y * width + r.x0..y * width + r.x1];
        let rb = &b[y * width + r.x0..y * width + r.x1];
        for (p, q) in ra.iter().zip(rb) {
            let d = (*p - *q) as f64;
            acc += d * d;
        }
    }
    acc / r.len().max(1) as f64
}

/// PSNR in dB for a plane with the given peak value (1.0 for Y).
pub fn psnr(a: &[f32], b: &[f32], width: usize, r: Region, peak: f64) -> f64 {
    let m = mse(a, b, width, r);
    if m <= 1e-20 {
        200.0
    } else {
        10.0 * (peak * peak / m).log10()
    }
}

/// Luma PSNR between two frames over a region.
pub fn psnr_y(a: &Frame, b: &Frame, r: Region) -> f64 {
    psnr(&a.y, &b.y, a.width, r, 1.0)
}

/// Combined Y/U/V PSNR (U and V weighted like Y, peak 1).
pub fn psnr_yuv(a: &Frame, b: &Frame, r: Region) -> f64 {
    let m = (mse(&a.y, &b.y, a.width, r) + mse(&a.u, &b.u, a.width, r) + mse(&a.v, &b.v, a.width, r)) / 3.0;
    if m <= 1e-20 {
        200.0
    } else {
        10.0 * (1.0 / m).log10()
    }
}

/// Mean structural similarity of two planes (8x8 windows, step 4,
/// constants for peak 1).
pub fn ssim(a: &[f32], b: &[f32], width: usize, r: Region) -> f64 {
    let c1 = (0.01f64).powi(2);
    let c2 = (0.03f64).powi(2);
    let mut total = 0.0;
    let mut count = 0usize;
    let mut y = r.y0;
    while y + 8 <= r.y1 {
        let mut x = r.x0;
        while x + 8 <= r.x1 {
            let (mut sa, mut sb, mut saa, mut sbb, mut sab) = (0.0f64, 0.0, 0.0, 0.0, 0.0);
            for yy in y..y + 8 {
                for xx in x..x + 8 {
                    let p = a[yy * width + xx] as f64;
                    let q = b[yy * width + xx] as f64;
                    sa += p;
                    sb += q;
                    saa += p * p;
                    sbb += q * q;
                    sab += p * q;
                }
            }
            let n = 64.0;
            let ma = sa / n;
            let mb = sb / n;
            let va = saa / n - ma * ma;
            let vb = sbb / n - mb * mb;
            let cov = sab / n - ma * mb;
            total += ((2.0 * ma * mb + c1) * (2.0 * cov + c2)) / ((ma * ma + mb * mb + c1) * (va + vb + c2));
            count += 1;
            x += 4;
        }
        y += 4;
    }
    total / count.max(1) as f64
}

/// Colour error statistics over a region: mean and max of the U/V vector
/// distance, and mean absolute Y error.
#[derive(Copy, Clone, Debug, Default)]
pub struct ColorError {
    pub mean_uv: f64,
    pub max_uv: f64,
    pub mean_y: f64,
    pub max_y: f64,
}

#[allow(clippy::too_many_arguments)]
pub fn color_error(
    y: &[f32],
    u: &[f32],
    v: &[f32],
    ry: &[f32],
    ru: &[f32],
    rv: &[f32],
    width: usize,
    r: Region,
) -> ColorError {
    let mut e = ColorError::default();
    for yy in r.y0..r.y1 {
        for xx in r.x0..r.x1 {
            let k = yy * width + xx;
            let duv = (((u[k] - ru[k]) as f64).powi(2) + ((v[k] - rv[k]) as f64).powi(2)).sqrt();
            let dy = ((y[k] - ry[k]) as f64).abs();
            e.mean_uv += duv;
            e.max_uv = e.max_uv.max(duv);
            e.mean_y += dy;
            e.max_y = e.max_y.max(dy);
        }
    }
    let n = r.len().max(1) as f64;
    e.mean_uv /= n;
    e.mean_y /= n;
    e
}

/// Mean chroma power (U^2 + V^2) over a region, for cross-colour tests on
/// luma-only pictures.
pub fn chroma_power(u: &[f32], v: &[f32], width: usize, r: Region) -> f64 {
    let mut acc = 0.0;
    for yy in r.y0..r.y1 {
        for xx in r.x0..r.x1 {
            let k = yy * width + xx;
            acc += (u[k] as f64).powi(2) + (v[k] as f64).powi(2);
        }
    }
    acc / r.len().max(1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn psnr_and_ssim_basics() {
        let a = vec![0.5f32; 64 * 64];
        let mut b = a.clone();
        let r = Region::full(64, 64);
        assert_eq!(psnr(&a, &b, 64, r, 1.0), 200.0);
        assert!((ssim(&a, &b, 64, r) - 1.0).abs() < 1e-9);
        for (i, v) in b.iter_mut().enumerate() {
            *v += if i % 2 == 0 { 0.01 } else { -0.01 };
        }
        assert!((psnr(&a, &b, 64, r, 1.0) - 40.0).abs() < 1e-3);
    }
}
