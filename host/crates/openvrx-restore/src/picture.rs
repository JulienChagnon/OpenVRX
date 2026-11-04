// SPDX-License-Identifier: GPL-3.0-or-later
//! Planar YUV picture with a per-pixel confidence map, plus the small image
//! helpers the restorer needs (sampling, blurs, noise estimate, metrics).

use rayon::prelude::*;

/// One picture (a progressive frame or a single field) in 4:4:4.
///
/// Y is 0..1, U/V on the BT.601 scale used by `openvrx_core::Frame`.
/// `conf` is 1.0 for a trusted pixel and 0.0 for a pixel that carries no
/// information; anything below 1.0 counts as flagged and gets rebuilt.
#[derive(Clone, Debug)]
pub struct Picture {
    pub width: usize,
    pub height: usize,
    pub y: Vec<f32>,
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    pub conf: Vec<f32>,
}

impl Picture {
    /// A black, fully trusted picture.
    pub fn new(width: usize, height: usize) -> Picture {
        let n = width * height;
        Picture {
            width,
            height,
            y: vec![0.0; n],
            u: vec![0.0; n],
            v: vec![0.0; n],
            conf: vec![1.0; n],
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.width * self.height
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Mean confidence over the picture.
    pub fn mean_conf(&self) -> f32 {
        if self.conf.is_empty() {
            return 0.0;
        }
        self.conf.iter().map(|&c| c as f64).sum::<f64>() as f32 / self.conf.len() as f32
    }
}

/// Bilinear sample with edge clamping.
#[inline]
pub fn sample_bilinear(p: &[f32], w: usize, h: usize, x: f32, y: f32) -> f32 {
    let xm = (w - 1) as f32;
    let ym = (h - 1) as f32;
    let x = x.clamp(0.0, xm);
    let y = y.clamp(0.0, ym);
    let x0 = x.floor();
    let y0 = y.floor();
    let fx = x - x0;
    let fy = y - y0;
    let x0 = x0 as usize;
    let y0 = y0 as usize;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let a = p[y0 * w + x0];
    let b = p[y0 * w + x1];
    let c = p[y1 * w + x0];
    let d = p[y1 * w + x1];
    let top = a + (b - a) * fx;
    let bot = c + (d - c) * fx;
    top + (bot - top) * fy
}

/// Catmull-Rom bicubic sample with edge clamping (keeps luma detail when
/// warping by fractional vectors).
#[inline]
pub fn sample_bicubic(p: &[f32], w: usize, h: usize, x: f32, y: f32) -> f32 {
    let x = x.clamp(0.0, (w - 1) as f32);
    let y = y.clamp(0.0, (h - 1) as f32);
    let xi = x.floor() as isize;
    let yi = y.floor() as isize;
    let fx = x - xi as f32;
    let fy = y - yi as f32;
    let wx = cubic_weights(fx);
    let wy = cubic_weights(fy);
    let wm = w as isize - 1;
    let hm = h as isize - 1;
    let mut acc = 0.0;
    for (j, &wyj) in wy.iter().enumerate() {
        let yy = (yi + j as isize - 1).clamp(0, hm) as usize;
        let row = &p[yy * w..yy * w + w];
        let mut r = 0.0;
        for (i, &wxi) in wx.iter().enumerate() {
            let xx = (xi + i as isize - 1).clamp(0, wm) as usize;
            r += row[xx] * wxi;
        }
        acc += r * wyj;
    }
    acc
}

#[inline]
fn cubic_weights(t: f32) -> [f32; 4] {
    let t2 = t * t;
    let t3 = t2 * t;
    [
        0.5 * (-t3 + 2.0 * t2 - t),
        0.5 * (3.0 * t3 - 5.0 * t2 + 2.0),
        0.5 * (-3.0 * t3 + 4.0 * t2 + t),
        0.5 * (t3 - t2),
    ]
}

/// Separable [1 2 1]/4 blur with edge clamping.
pub fn blur121(src: &[f32], w: usize, h: usize) -> Vec<f32> {
    let mut tmp = vec![0.0; w * h];
    tmp.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        let s = &src[y * w..y * w + w];
        for x in 0..w {
            let l = s[x.saturating_sub(1)];
            let r = s[(x + 1).min(w - 1)];
            row[x] = 0.25 * (l + r) + 0.5 * s[x];
        }
    });
    let mut out = vec![0.0; w * h];
    out.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        let a = &tmp[y.saturating_sub(1) * w..][..w];
        let b = &tmp[y * w..][..w];
        let c = &tmp[(y + 1).min(h - 1) * w..][..w];
        for x in 0..w {
            row[x] = 0.25 * (a[x] + c[x]) + 0.5 * b[x];
        }
    });
    out
}

/// Box mean over a (2r+1)^2 window with edge clamping.
pub fn box_mean(src: &[f32], w: usize, h: usize, r: usize) -> Vec<f32> {
    let k = (2 * r + 1) as f32;
    let mut tmp = vec![0.0; w * h];
    tmp.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        let s = &src[y * w..y * w + w];
        for (x, o) in row.iter_mut().enumerate() {
            let mut acc = 0.0;
            for d in 0..=2 * r {
                let xx = (x + d).saturating_sub(r).min(w - 1);
                acc += s[xx];
            }
            *o = acc / k;
        }
    });
    let mut out = vec![0.0; w * h];
    out.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        for (x, o) in row.iter_mut().enumerate() {
            let mut acc = 0.0;
            for d in 0..=2 * r {
                let yy = (y + d).saturating_sub(r).min(h - 1);
                acc += tmp[yy * w + x];
            }
            *o = acc / k;
        }
    });
    out
}

/// 2x2 average downsample (odd edges replicate). Returns (plane, w, h).
pub fn downsample2(src: &[f32], w: usize, h: usize) -> (Vec<f32>, usize, usize) {
    let nw = w.div_ceil(2);
    let nh = h.div_ceil(2);
    let mut out = vec![0.0; nw * nh];
    out.par_chunks_mut(nw).enumerate().for_each(|(y, row)| {
        let y0 = (2 * y).min(h - 1);
        let y1 = (2 * y + 1).min(h - 1);
        for (x, o) in row.iter_mut().enumerate() {
            let x0 = (2 * x).min(w - 1);
            let x1 = (2 * x + 1).min(w - 1);
            *o = 0.25 * (src[y0 * w + x0] + src[y0 * w + x1] + src[y1 * w + x0] + src[y1 * w + x1]);
        }
    });
    (out, nw, nh)
}

/// Robust per-picture noise sigma (luma) from trusted pixels.
///
/// Immerkaer's Laplacian-difference estimator evaluated on 8x8 blocks whose
/// pixels are all trusted; the 25th percentile of the block estimates is
/// returned so textured blocks do not inflate it.
pub fn estimate_noise(y: &[f32], conf: &[f32], w: usize, h: usize) -> f32 {
    const B: usize = 8;
    if w < B + 2 || h < B + 2 {
        return 0.0;
    }
    let bx = (w - 2) / B;
    let by = (h - 2) / B;
    let mut est: Vec<f32> = (0..by)
        .into_par_iter()
        .flat_map_iter(|j| {
            (0..bx).filter_map(move |i| {
                let mut acc = 0.0f32;
                for yy in 1 + j * B..1 + (j + 1) * B {
                    for xx in 1 + i * B..1 + (i + 1) * B {
                        if conf[yy * w + xx] < 1.0 {
                            return None;
                        }
                        let at = |dx: isize, dy: isize| {
                            y[(yy as isize + dy) as usize * w + (xx as isize + dx) as usize]
                        };
                        let l = at(-1, -1) - 2.0 * at(0, -1) + at(1, -1) - 2.0 * at(-1, 0)
                            + 4.0 * at(0, 0)
                            - 2.0 * at(1, 0)
                            + at(-1, 1)
                            - 2.0 * at(0, 1)
                            + at(1, 1);
                        acc += l.abs();
                    }
                }
                Some(acc * (std::f32::consts::PI / 2.0).sqrt() / (6.0 * (B * B) as f32))
            })
        })
        .collect();
    if est.is_empty() {
        return 0.0;
    }
    let k = est.len() / 4;
    let (_, v, _) = est.select_nth_unstable_by(k, |a, b| a.total_cmp(b));
    *v
}

/// Peak signal-to-noise ratio in dB for signals with peak 1.0.
pub fn psnr(a: &[f32], b: &[f32]) -> f64 {
    assert_eq!(a.len(), b.len());
    let mse = a
        .iter()
        .zip(b)
        .map(|(&x, &y)| {
            let d = (x.clamp(0.0, 1.0) - y.clamp(0.0, 1.0)) as f64;
            d * d
        })
        .sum::<f64>()
        / a.len().max(1) as f64;
    if mse <= 1e-12 {
        120.0
    } else {
        -10.0 * mse.log10()
    }
}

/// Mean SSIM (Wang et al. 2004) with an 8x8 sliding window stepped by 4,
/// constants for a peak of 1.0.
pub fn ssim(a: &[f32], b: &[f32], w: usize, h: usize) -> f64 {
    const N: usize = 8;
    const STEP: usize = 4;
    let c1 = (0.01f64).powi(2);
    let c2 = (0.03f64).powi(2);
    if w < N || h < N {
        return if psnr(a, b) >= 120.0 { 1.0 } else { 0.0 };
    }
    let ys: Vec<usize> = (0..=h - N).step_by(STEP).collect();
    let vals: Vec<f64> = ys
        .par_iter()
        .flat_map_iter(|&y0| {
            (0..=w - N).step_by(STEP).map(move |x0| {
                let (mut sa, mut sb, mut saa, mut sbb, mut sab) = (0.0, 0.0, 0.0, 0.0, 0.0);
                for y in y0..y0 + N {
                    for x in x0..x0 + N {
                        let p = a[y * w + x].clamp(0.0, 1.0) as f64;
                        let q = b[y * w + x].clamp(0.0, 1.0) as f64;
                        sa += p;
                        sb += q;
                        saa += p * p;
                        sbb += q * q;
                        sab += p * q;
                    }
                }
                let n = (N * N) as f64;
                let ma = sa / n;
                let mb = sb / n;
                let va = saa / n - ma * ma;
                let vb = sbb / n - mb * mb;
                let cov = sab / n - ma * mb;
                ((2.0 * ma * mb + c1) * (2.0 * cov + c2))
                    / ((ma * ma + mb * mb + c1) * (va + vb + c2))
            })
        })
        .collect();
    vals.iter().sum::<f64>() / vals.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bicubic_is_exact_on_grid_and_linear_ramps() {
        let w = 9;
        let h = 7;
        let p: Vec<f32> = (0..w * h).map(|k| (k % w) as f32 * 0.1 + (k / w) as f32 * 0.05).collect();
        assert!((sample_bicubic(&p, w, h, 3.0, 2.0) - p[2 * w + 3]).abs() < 1e-6);
        let v = sample_bicubic(&p, w, h, 3.25, 2.5);
        assert!((v - (0.325 + 0.125)).abs() < 1e-5);
        assert!((sample_bilinear(&p, w, h, 3.25, 2.5) - (0.325 + 0.125)).abs() < 1e-5);
    }

    #[test]
    fn noise_estimate_tracks_sigma() {
        let w = 160;
        let h = 120;
        let mut s = 12345u64;
        let mut gauss = || {
            let mut acc = 0.0;
            for _ in 0..12 {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                acc += (s >> 40) as f32 / (1u64 << 24) as f32;
            }
            acc - 6.0
        };
        let y: Vec<f32> = (0..w * h)
            .map(|k| 0.5 + 0.2 * ((k % w) as f32 / w as f32) + 0.02 * gauss())
            .collect();
        let c = vec![1.0; w * h];
        let e = estimate_noise(&y, &c, w, h);
        assert!((e - 0.02).abs() < 0.004, "sigma estimate {e}");
    }

    #[test]
    fn metrics_identity() {
        let a: Vec<f32> = (0..64 * 64).map(|k| (k % 13) as f32 / 13.0).collect();
        assert_eq!(psnr(&a, &a), 120.0);
        assert!((ssim(&a, &a, 64, 64) - 1.0).abs() < 1e-9);
    }
}
