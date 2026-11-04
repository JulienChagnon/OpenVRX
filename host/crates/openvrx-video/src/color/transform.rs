// SPDX-License-Identifier: GPL-3.0-or-later
//! Frequency-domain PAL luma/chroma separation ("Transform PAL" idea).
//!
//! In the 2D spectrum of a PAL field (horizontal cycles per sample,
//! vertical cycles per field line) U sits around (fsc, -1/4) and V around
//! (fsc, +1/4). Both are real signals modulated onto a carrier, so their
//! spectra are point-symmetric about their carrier, and with the PAL
//! offsets both symmetries reduce to one reflection:
//!
//! ```text
//! (fx, fy) <-> (2 fsc - fx, 1/2 - fy)        (mod 1 vertically)
//! ```
//!
//! Luma in the chroma band generally has no matching component at the
//! reflected frequency. The decoder therefore takes overlapping windowed
//! blocks, compares each chroma-band bin with its reflection and keeps only
//! the symmetric part as chroma; asymmetric energy is luma.
//!
//! The 3D variant adds time: the same block from the current field and the
//! three previous fields of the same parity (one frame apart, where U
//! advances by 3/4 cycle and V by 1/4). The reflection becomes
//! `ft -> 1/2 - ft`, which separates static luma from chroma at every
//! spatial frequency. Only past fields are used, so it adds no delay.
//!
//! Implementation: 16-line x 32-sample blocks, periodic Hann windows with
//! 50 % overlap in both directions (the windows sum to one, so the chroma
//! estimates are simply overlap-added). Only the chroma band columns are
//! transformed vertically and synthesised back.

use rustfft::num_complex::Complex32 as C32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::sync::Arc;

pub const BLOCK_W: usize = 32;
pub const BLOCK_H: usize = 16;
pub const HOP_X: usize = BLOCK_W / 2;
pub const HOP_Y: usize = BLOCK_H / 2;
/// Temporal depth of the 3D transform (same-parity fields).
pub const DEPTH: usize = 4;

/// Precomputed transforms and tables.
pub struct TransformPal {
    row_fft: Arc<dyn Fft<f32>>,
    col_fft: Arc<dyn Fft<f32>>,
    col_ifft: Arc<dyn Fft<f32>>,
    win_x: [f32; BLOCK_W],
    win_y: [f32; BLOCK_H],
    /// Horizontal bins processed (chroma band), centred on the carrier.
    band: Vec<usize>,
    /// Horizontal mirror bin index: 2 * carrier - kx.
    mirror_x: usize,
    /// Synthesis table: exp(j 2 pi k c / W) for band bins.
    synth: Vec<C32>,
    /// Bins below this similarity (min/max magnitude) count as luma.
    pub threshold: f32,
}

impl TransformPal {
    pub fn new(fsc_per_sample: f64, threshold: f32) -> TransformPal {
        let mut planner = FftPlanner::<f32>::new();
        let row_fft = planner.plan_fft_forward(BLOCK_W);
        let col_fft = planner.plan_fft_forward(BLOCK_H);
        let col_ifft = planner.plan_fft_inverse(BLOCK_H);
        let mut win_x = [0.0; BLOCK_W];
        for (i, w) in win_x.iter_mut().enumerate() {
            *w = 0.5 - 0.5 * (2.0 * PI * i as f32 / BLOCK_W as f32).cos();
        }
        let mut win_y = [0.0; BLOCK_H];
        for (i, w) in win_y.iter_mut().enumerate() {
            *w = 0.5 - 0.5 * (2.0 * PI * i as f32 / BLOCK_H as f32).cos();
        }
        let kc = (fsc_per_sample * BLOCK_W as f64).round() as usize;
        let band: Vec<usize> = (kc - 4..=kc + 4).collect();
        let mut synth = Vec::with_capacity(band.len() * BLOCK_W);
        for &k in &band {
            for c in 0..BLOCK_W {
                let a = 2.0 * PI * (k * c) as f32 / BLOCK_W as f32;
                synth.push(C32::new(a.cos(), a.sin()));
            }
        }
        TransformPal {
            row_fft,
            col_fft,
            col_ifft,
            win_x,
            win_y,
            band,
            mirror_x: 2 * kc,
            synth,
            threshold,
        }
    }

    pub fn band_len(&self) -> usize {
        self.band.len()
    }

    /// Forward transform of one windowed block. `get(r, c)` returns the
    /// composite sample of block row r, column c. Output: band bins,
    /// `[band index][ky]`.
    fn forward(&self, get: &dyn Fn(usize, usize) -> f32, scratch: &mut Scratch, out: &mut [C32]) {
        let rows = &mut scratch.rows;
        for r in 0..BLOCK_H {
            let wy = self.win_y[r];
            for c in 0..BLOCK_W {
                rows[r * BLOCK_W + c] = C32::new(get(r, c) * wy * self.win_x[c], 0.0);
            }
        }
        self.row_fft.process_with_scratch(rows, &mut scratch.fft);
        let nb = self.band.len();
        for (b, &k) in self.band.iter().enumerate() {
            for r in 0..BLOCK_H {
                out[b * BLOCK_H + r] = rows[r * BLOCK_W + k];
            }
        }
        self.col_fft.process_with_scratch(&mut out[..nb * BLOCK_H], &mut scratch.fft);
    }

    /// Band index of a horizontal bin (if inside the band).
    #[inline]
    fn band_index(&self, k: usize) -> usize {
        k - self.band[0]
    }

    /// Keep factor for a bin given its own and its mirror's magnitude.
    #[inline]
    fn keep(&self, m: f32, mm: f32) -> f32 {
        let hi = m.max(mm);
        if hi <= 1e-12 {
            return 0.0;
        }
        let lo = m.min(mm);
        if lo < self.threshold * hi {
            0.0
        } else {
            (mm / m.max(1e-12)).min(1.0)
        }
    }

    /// Inverse: band bins `[band][ky]` to the real block chroma estimate
    /// (16 x 32, row major), scaled for the analytic-signal synthesis.
    fn inverse(&self, bins: &mut [C32], scratch: &mut Scratch, out: &mut [f32; BLOCK_W * BLOCK_H]) {
        let nb = self.band.len();
        self.col_ifft.process_with_scratch(&mut bins[..nb * BLOCK_H], &mut scratch.fft);
        let norm = 2.0 / (BLOCK_W * BLOCK_H) as f32;
        for r in 0..BLOCK_H {
            for c in 0..BLOCK_W {
                let mut acc = 0.0f32;
                for b in 0..nb {
                    let z = bins[b * BLOCK_H + r];
                    let t = self.synth[b * BLOCK_W + c];
                    acc += z.re * t.re - z.im * t.im;
                }
                out[r * BLOCK_W + c] = acc * norm;
            }
        }
    }

    /// 2D separation of one block. Returns the chroma estimate (windowed,
    /// ready for overlap-add).
    pub fn block_2d(&self, get: &dyn Fn(usize, usize) -> f32, scratch: &mut Scratch, out: &mut [f32; BLOCK_W * BLOCK_H]) {
        let nb = self.band.len();
        let mut spec = std::mem::take(&mut scratch.spec);
        spec.resize(nb * BLOCK_H, C32::new(0.0, 0.0));
        self.forward(get, scratch, &mut spec);
        let mut kept = std::mem::take(&mut scratch.kept);
        kept.resize(nb * BLOCK_H, C32::new(0.0, 0.0));
        for b in 0..nb {
            let kx = self.band[b];
            let mb = self.band_index(self.mirror_x - kx);
            for ky in 0..BLOCK_H {
                let mky = (BLOCK_H / 2 + BLOCK_H - ky) % BLOCK_H;
                let z = spec[b * BLOCK_H + ky];
                let zm = spec[mb * BLOCK_H + mky];
                let f = self.keep(z.norm(), zm.norm());
                kept[b * BLOCK_H + ky] = z * f;
            }
        }
        self.inverse(&mut kept, scratch, out);
        scratch.spec = spec;
        scratch.kept = kept;
    }

    /// 3D separation. `get(t, r, c)` returns the sample of field t (0 =
    /// current, t = frames back). Writes the current field's chroma and
    /// returns a motion measure: the share of chroma-band energy at
    /// temporal frequency 1/2 cycle per frame, where neither static luma
    /// (0) nor static PAL chroma (+-1/4) can be.
    pub fn block_3d(&self, get: &dyn Fn(usize, usize, usize) -> f32, scratch: &mut Scratch, out: &mut [f32; BLOCK_W * BLOCK_H]) -> f32 {
        let nb = self.band.len();
        let n = nb * BLOCK_H;
        let mut spec = std::mem::take(&mut scratch.spec);
        spec.resize(DEPTH * n, C32::new(0.0, 0.0));
        for t in 0..DEPTH {
            let g = |r: usize, c: usize| get(t, r, c);
            self.forward(&g, scratch, &mut spec[t * n..(t + 1) * n]);
        }
        // Temporal DFT (4 points) in place per bin.
        let mut temporal = std::mem::take(&mut scratch.kept);
        temporal.resize(DEPTH * n, C32::new(0.0, 0.0));
        for i in 0..n {
            let x0 = spec[i];
            let x1 = spec[n + i];
            let x2 = spec[2 * n + i];
            let x3 = spec[3 * n + i];
            // X(k) = sum_t x_t e^{-j 2 pi k t / 4}
            temporal[i] = x0 + x1 + x2 + x3;
            temporal[n + i] = x0 + C32::new(x1.im, -x1.re) - x2 + C32::new(-x3.im, x3.re);
            temporal[2 * n + i] = x0 - x1 + x2 - x3;
            temporal[3 * n + i] = x0 + C32::new(-x1.im, x1.re) - x2 + C32::new(x3.im, -x3.re);
        }
        let mut e_total = 0.0f32;
        let mut e_half = 0.0f32;
        for kt in 0..DEPTH {
            let e: f32 = temporal[kt * n..(kt + 1) * n].iter().map(|z| z.norm_sqr()).sum();
            e_total += e;
            if kt == DEPTH / 2 {
                e_half = e;
            }
        }
        // Keep the symmetric part; synthesise time index 0 directly:
        // x_0 = (1/4) sum_k X(k).
        let out_bins = &mut spec[..n];
        out_bins.fill(C32::new(0.0, 0.0));
        for kt in 0..DEPTH {
            let mkt = (DEPTH / 2 + DEPTH - kt) % DEPTH;
            for b in 0..nb {
                let kx = self.band[b];
                let mb = self.band_index(self.mirror_x - kx);
                for ky in 0..BLOCK_H {
                    let mky = (BLOCK_H / 2 + BLOCK_H - ky) % BLOCK_H;
                    let z = temporal[kt * n + b * BLOCK_H + ky];
                    let zm = temporal[mkt * n + mb * BLOCK_H + mky];
                    let f = self.keep(z.norm(), zm.norm());
                    out_bins[b * BLOCK_H + ky] += z * (f * 0.25);
                }
            }
        }
        let mut bins = out_bins.to_vec();
        self.inverse(&mut bins, scratch, out);
        scratch.spec = spec;
        scratch.kept = temporal;
        if e_total > 1e-9 {
            e_half / e_total
        } else {
            0.0
        }
    }
}

/// Per-thread working buffers.
pub struct Scratch {
    rows: Vec<C32>,
    fft: Vec<C32>,
    spec: Vec<C32>,
    kept: Vec<C32>,
}

impl Scratch {
    pub fn new(tp: &TransformPal) -> Scratch {
        let fft_len = tp
            .row_fft
            .get_inplace_scratch_len()
            .max(tp.col_fft.get_inplace_scratch_len())
            .max(tp.col_ifft.get_inplace_scratch_len());
        Scratch {
            rows: vec![C32::new(0.0, 0.0); BLOCK_W * BLOCK_H],
            fft: vec![C32::new(0.0, 0.0); fft_len],
            spec: Vec::new(),
            kept: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pure PAL-like chroma pattern is kept, a pure luma pattern near
    /// fsc is rejected.
    #[test]
    fn separates_symmetric_from_asymmetric() {
        let f = 0.25f64;
        let tp = TransformPal::new(f, 0.4);
        let mut sc = Scratch::new(&tp);
        // Chroma: U constant modulated at (f, 0.7516 per line).
        let chroma = |r: usize, c: usize| -> f32 {
            let ph = 2.0 * std::f64::consts::PI * (f * c as f64 + 0.7516 * r as f64);
            (20.0 * ph.sin()) as f32
        };
        let mut out = [0.0f32; BLOCK_W * BLOCK_H];
        tp.block_2d(&chroma, &mut sc, &mut out);
        // Centre of the block: output = window * input.
        let mut err = 0.0f32;
        let mut pow = 0.0f32;
        for r in 6..10 {
            for c in 12..20 {
                let w = tp.win_y[r] * tp.win_x[c];
                err += (out[r * BLOCK_W + c] - w * chroma(r, c)).powi(2);
                pow += (w * chroma(r, c)).powi(2);
            }
        }
        assert!(err / pow < 0.01, "chroma kept: rel err {}", err / pow);
        // Luma: vertical stripes at 0.27 cycles/sample, constant vertically.
        let luma = |_r: usize, c: usize| -> f32 { (30.0 * (2.0 * std::f64::consts::PI * 0.27 * c as f64).cos()) as f32 };
        tp.block_2d(&luma, &mut sc, &mut out);
        let mut leak = 0.0f32;
        let mut lp = 0.0f32;
        for r in 6..10 {
            for c in 12..20 {
                let w = tp.win_y[r] * tp.win_x[c];
                leak += out[r * BLOCK_W + c].powi(2);
                lp += (w * luma(r, c)).powi(2);
            }
        }
        assert!(leak / lp < 0.02, "luma leak {}", leak / lp);
    }
}
