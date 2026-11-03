// SPDX-License-Identifier: GPL-3.0-or-later
//! Vector kernels for the loops LLVM does not vectorise on its own
//! (floating-point reductions in strict order) and the discriminator.
//!
//! Every kernel is a plain loop over fixed 8-lane arrays inside
//! [`dispatch!`], so the AVX2 build runs it on 256-bit registers and the
//! baseline build on SSE2.

/// Lanes per accumulator.
const W: usize = 8;

#[inline(always)]
fn lanes(s: &[f32], k: usize) -> [f32; W] {
    s[k..k + W].try_into().expect("8 lanes")
}

#[inline(always)]
fn hsum(a: [f32; W]) -> f32 {
    ((a[0] + a[4]) + (a[2] + a[6])) + ((a[1] + a[5]) + (a[3] + a[7]))
}

dispatch! {
    /// out[k] = atan2(y[k], x[k]).
    pub fn atan2_slice(y: &[f32], x: &[f32], out: &mut [f32]) {
        let n = out.len().min(x.len()).min(y.len());
        let (y, x, out) = (&y[..n], &x[..n], &mut out[..n]);
        for k in 0..n {
            out[k] = crate::fastmath::atan2(y[k], x[k]);
        }
    }
}

dispatch! {
    /// Polar discriminator: out[k] = arg(z[k] conj z[k-1]), z[-1] = prev.
    /// A plain loop over the branch-free scalar atan2: LLVM vectorises it
    /// at the full vector width of the dispatched target.
    pub fn polar(re: &[f32], im: &[f32], prev: (f32, f32), out: &mut [f32]) {
        let n = re.len().min(im.len()).min(out.len());
        if n == 0 {
            return;
        }
        let (pr, pi) = prev;
        out[0] = crate::fastmath::atan2(im[0] * pr - re[0] * pi, re[0] * pr + im[0] * pi);
        let (xr, xi, yr, yi) = (&re[1..n], &im[1..n], &re[..n - 1], &im[..n - 1]);
        let o = &mut out[1..n];
        for k in 0..n - 1 {
            let a = xr[k] * yr[k] + xi[k] * yi[k];
            let b = xi[k] * yr[k] - xr[k] * yi[k];
            o[k] = crate::fastmath::atan2(b, a);
        }
    }
}

dispatch! {
    /// Sum and sum of squares (f32 lanes per 512 samples, f64 between).
    pub fn sums12(p: &[f32]) -> (f64, f64) {
        let (mut s1, mut s2) = (0.0f64, 0.0f64);
        for blk in p.chunks(512) {
            let full = blk.len() / W * W;
            let (mut a, mut b) = ([0.0f32; W], [0.0f32; W]);
            for k in (0..full).step_by(W) {
                let v = lanes(blk, k);
                for l in 0..W {
                    a[l] += v[l];
                    b[l] += v[l] * v[l];
                }
            }
            let (mut ta, mut tb) = (hsum(a) as f64, hsum(b) as f64);
            for &v in &blk[full..] {
                ta += v as f64;
                tb += v as f64 * v as f64;
            }
            s1 += ta;
            s2 += tb;
        }
        (s1, s2)
    }
}

/// Dot product of two equal-length slices.
#[inline(always)]
pub fn dot(a: &[f32], b: &[f32]) -> f32 {
    let n = a.len().min(b.len());
    let full = n / W * W;
    let mut acc = [0.0f32; W];
    for k in (0..full).step_by(W) {
        let (x, y) = (lanes(a, k), lanes(b, k));
        for l in 0..W {
            acc[l] += x[l] * y[l];
        }
    }
    let mut s = hsum(acc);
    for k in full..n {
        s += a[k] * b[k];
    }
    s
}

dispatch! {
    /// Clip flags and raw sums (i, q, i^2, q^2); returns the clip count.
    pub fn raw_pass(re: &[f32], im: &[f32], cl: f32, clip: &mut [u8]) -> ([f64; 4], u64) {
        let n = re.len().min(im.len()).min(clip.len());
        let mut s = [0.0f64; 4];
        let mut clips = 0u64;
        for start in (0..n).step_by(512) {
            let end = (start + 512).min(n);
            let full = start + (end - start) / W * W;
            let mut a = [[0.0f32; W]; 4];
            let mut cnt = [0u32; W];
            for k in (start..full).step_by(W) {
                let (x, y) = (lanes(re, k), lanes(im, k));
                let c: &mut [u8; W] = (&mut clip[k..k + W]).try_into().expect("8 lanes");
                for l in 0..W {
                    let f = (x[l].abs() >= cl) | (y[l].abs() >= cl);
                    c[l] = f as u8;
                    cnt[l] += f as u32;
                    a[0][l] += x[l];
                    a[1][l] += y[l];
                    a[2][l] += x[l] * x[l];
                    a[3][l] += y[l] * y[l];
                }
            }
            let mut tail = [0.0f64; 4];
            for k in full..end {
                let (x, y) = (re[k], im[k]);
                let f = (x.abs() >= cl) | (y.abs() >= cl);
                clip[k] = f as u8;
                clips += f as u64;
                tail[0] += x as f64;
                tail[1] += y as f64;
                tail[2] += (x * x) as f64;
                tail[3] += (y * y) as f64;
            }
            clips += cnt.iter().map(|&c| c as u64).sum::<u64>();
            for q in 0..4 {
                s[q] += hsum(a[q]) as f64 + tail[q];
            }
        }
        (s, clips)
    }
}

dispatch! {
    /// Subtracts DC, applies the I/Q correction, writes |z|^2; returns the
    /// moments (ii, qq, iq) of the DC-removed, uncorrected samples.
    pub fn apply_pass(
        re: &mut [f32],
        im: &mut [f32],
        env2: &mut [f32],
        dc: (f32, f32),
        corr: (f32, f32),
    ) -> [f64; 3] {
        let n = re.len().min(im.len()).min(env2.len());
        let (re, im, env2) = (&mut re[..n], &mut im[..n], &mut env2[..n]);
        let mut out = [0.0f64; 3];
        for start in (0..n).step_by(512) {
            let end = (start + 512).min(n);
            let full = start + (end - start) / W * W;
            let mut a = [[0.0f32; W]; 3];
            for k in (start..full).step_by(W) {
                let (xr, yr) = (lanes(re, k), lanes(im, k));
                let mut xo = [0.0f32; W];
                let mut yo = [0.0f32; W];
                let mut eo = [0.0f32; W];
                for l in 0..W {
                    let x = xr[l] - dc.0;
                    let y = yr[l] - dc.1;
                    a[0][l] += x * x;
                    a[1][l] += y * y;
                    a[2][l] += x * y;
                    let yc = corr.0 * x + corr.1 * y;
                    xo[l] = x;
                    yo[l] = yc;
                    eo[l] = x * x + yc * yc;
                }
                re[k..k + W].copy_from_slice(&xo);
                im[k..k + W].copy_from_slice(&yo);
                env2[k..k + W].copy_from_slice(&eo);
            }
            let mut tail = [0.0f64; 3];
            for k in full..end {
                let x = re[k] - dc.0;
                let y = im[k] - dc.1;
                tail[0] += (x * x) as f64;
                tail[1] += (y * y) as f64;
                tail[2] += (x * y) as f64;
                let yc = corr.0 * x + corr.1 * y;
                re[k] = x;
                im[k] = yc;
                env2[k] = x * x + yc * yc;
            }
            for q in 0..3 {
                out[q] += hsum(a[q]) as f64 + tail[q];
            }
        }
        out
    }
}

dispatch! {
    /// Circle-fit sums x, y, xx, yy, xy, z, zx, zy, zz (z = x^2 + y^2) of the
    /// samples relative to (ci, cq).
    pub fn circle_sums(re: &[f32], im: &[f32], ci: f32, cq: f32) -> [f64; 9] {
        let n = re.len().min(im.len());
        let mut s = [0.0f64; 9];
        for start in (0..n).step_by(64) {
            let end = (start + 64).min(n);
            let full = start + (end - start) / W * W;
            let mut a = [[0.0f32; W]; 9];
            for k in (start..full).step_by(W) {
                let (xr, yr) = (lanes(re, k), lanes(im, k));
                for l in 0..W {
                    let x = xr[l] - ci;
                    let y = yr[l] - cq;
                    let z = x * x + y * y;
                    a[0][l] += x;
                    a[1][l] += y;
                    a[2][l] += x * x;
                    a[3][l] += y * y;
                    a[4][l] += x * y;
                    a[5][l] += z;
                    a[6][l] += z * x;
                    a[7][l] += z * y;
                    a[8][l] += z * z;
                }
            }
            for q in 0..9 {
                s[q] += hsum(a[q]) as f64;
            }
            for k in full..end {
                let x = (re[k] - ci) as f64;
                let y = (im[k] - cq) as f64;
                let z = x * x + y * y;
                let t = [x, y, x * x, y * y, x * y, z, z * x, z * y, z * z];
                for q in 0..9 {
                    s[q] += t[q];
                }
            }
        }
        s
    }
}

dispatch! {
    /// Isolated samples far off the carrier ring with both neighbours on it.
    pub fn glitches(env2: &[f32], a: f32, far: f32, near: f32) -> u64 {
        const B: usize = 256;
        let n = env2.len();
        if n < 3 {
            return 0;
        }
        // Distance from the ring, once per sample, in blocks that overlap
        // by the two neighbours.
        let mut dev = [0.0f32; B + 2];
        let mut g = 0u64;
        let mut start = 0;
        while start + 2 < n {
            let end = (start + B + 2).min(n);
            let m = end - start;
            for (d, &e) in dev[..m].iter_mut().zip(&env2[start..end]) {
                *d = (e.sqrt() - a).abs();
            }
            let mut cnt = 0u32;
            for k in 1..m - 1 {
                cnt += ((dev[k] > far) & (dev[k - 1] < near) & (dev[k + 1] < near)) as u32;
            }
            g += cnt as u64;
            start += m - 2;
        }
        g
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kernels_match_scalar() {
        let n = 1003;
        let x: Vec<f32> = (0..n).map(|k| ((k * 37 % 101) as f32 - 50.0) * 0.7).collect();
        let y: Vec<f32> = (0..n).map(|k| ((k * 53 % 97) as f32 - 48.0) * 1.3).collect();
        let mut o = vec![0.0; n];
        atan2_slice(&y, &x, &mut o);
        for k in 0..n {
            let e = (o[k] as f64 - (y[k] as f64).atan2(x[k] as f64)).abs();
            let e = e.min((e - std::f64::consts::TAU).abs());
            assert!(e < 2e-6, "{k}: {} {} -> {}", y[k], x[k], o[k]);
        }
        let (s1, s2) = sums12(&x);
        let e1: f64 = x.iter().map(|&v| v as f64).sum();
        let e2: f64 = x.iter().map(|&v| (v as f64).powi(2)).sum();
        assert!((s1 - e1).abs() < 1e-2 && (s2 - e2).abs() / e2 < 1e-6);
        assert!((dot(&x, &y) as f64 - x.iter().zip(&y).map(|(a, b)| (*a * *b) as f64).sum::<f64>()).abs() < 1.0);
        let mut c = vec![0u8; n];
        let (s, cl) = raw_pass(&x, &y, 60.0, &mut c);
        let ecl = x.iter().zip(&y).filter(|(a, b)| a.abs() >= 60.0 || b.abs() >= 60.0).count();
        assert_eq!(cl as usize, ecl);
        assert_eq!(c.iter().map(|&v| v as usize).sum::<usize>(), ecl);
        assert!((s[0] - e1).abs() < 1e-2);
        let mut p = vec![0.0; n];
        polar(&x, &y, (1.0, 0.0), &mut p);
        for k in 1..n {
            if (x[k] == 0.0 && y[k] == 0.0) || (x[k - 1] == 0.0 && y[k - 1] == 0.0) {
                continue;
            }
            let a = (y[k] as f64).atan2(x[k] as f64) - (y[k - 1] as f64).atan2(x[k - 1] as f64);
            let a = (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            let e = (p[k] as f64 - a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            assert!(e.abs() < 1e-5, "{k}");
        }
    }
}
