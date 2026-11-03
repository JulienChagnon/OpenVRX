// SPDX-License-Identifier: GPL-3.0-or-later
//! Pre-detection diversity combining: complex I/Q from two or more
//! receivers are aligned and summed BEFORE FM demodulation, so the C/N at
//! the discriminator (where analog FM breaks up) improves.
//!
//! Branch 0 is the timing reference; the combined stream is on its sample
//! clock and feeds an ordinary [`crate::RxDsp`] through `push_iq`. Each
//! other branch has its own crystal (tens of ppm) and LO, so per branch:
//!
//! - Acquisition: the polar-discriminator outputs of a long window are
//!   cross-correlated by FFT (insensitive to the unknown frequency offset)
//!   for the integer delay; the FFT of z_ref conj(z_k) at that delay gives
//!   the frequency offset; the complex correlation peak, interpolated over
//!   lags and measured on both halves of the window, gives the fractional
//!   delay and the sample-rate ratio.
//! - Tracking (per block): the branch is evaluated at its tracked
//!   fractional position by a Farrow resampler (polynomial-in-mu windowed
//!   sinc, value and derivative), derotated by a second-order phase loop,
//!   and steered by a delay-locked loop that uses the correlation gradient
//!   with the leave-one-out combination of the other branches (so a branch
//!   never tracks its own noise). Loop updates are scaled by the
//!   correlation reliability, so a faded branch coasts.
//! - Weights: maximal-ratio, |h_k| / sigma_k^2 on the phase-aligned branch,
//!   with |h_k|^2 and sigma_k^2 from the per-block power and the envelope
//!   C/N (noise power smoothed slowly, signal power per block, so deep
//!   Rayleigh fades are followed). Weights and loop corrections are ramped
//!   across each block so the combined phase stays continuous.
//!
//! Tests use the impairments reported for real receivers: LOs 150 kHz
//! apart, sample clocks 25 ppm apart, 6 dB gain difference, a fractional
//! raster delay, independent noise and independent Rayleigh fading.

use crate::cn::EnvAcc;
use crate::fastmath::{atan2, sin_cos, wrap_pi};
use crate::filters::{kaiser, sinc};
use openvrx_core::Complex32;
use rustfft::FftPlanner;
use serde::{Deserialize, Serialize};

/// Combiner settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PredetectConfig {
    /// Tracking block (samples of the reference).
    pub block: usize,
    /// Acquisition window (samples).
    pub acq_len: usize,
    /// Integer delay search range (+- samples) around the initial guess.
    pub search: usize,
    /// Phase loop gain per block (0..1).
    pub phase_gain: f32,
    /// Delay loop gain per block (0..1).
    pub delay_gain: f32,
    /// Minimum normalised correlation to accept an acquisition.
    pub min_corr: f32,
    /// A branch whose data lags the reference by more than this is left
    /// out (weight 0) instead of stalling the output.
    pub max_lag: usize,
}

impl Default for PredetectConfig {
    fn default() -> Self {
        PredetectConfig {
            block: 256,
            acq_len: 1 << 16,
            search: 8192,
            phase_gain: 0.25,
            delay_gain: 0.15,
            min_corr: 0.05,
            max_lag: 1 << 16,
        }
    }
}

/// Per-branch state for the OSD and tests.
#[derive(Copy, Clone, Debug, Default, Serialize, Deserialize)]
pub struct BranchStats {
    pub acquired: bool,
    /// Branch sample position of the current reference sample minus the
    /// reference index (samples).
    pub delay: f64,
    /// Sample-rate ratio minus one, ppm.
    pub rate_ppm: f64,
    /// Frequency of the branch relative to the reference (Hz).
    pub freq_hz: f64,
    pub weight: f32,
    /// Block envelope C/N, smoothed (dB).
    pub cn_db: f32,
    /// Normalised correlation with the other branches.
    pub corr: f32,
}

/// Farrow fractional-delay interpolator: windowed-sinc taps as degree-5
/// polynomials in mu.
#[derive(Clone, Debug)]
pub struct Farrow {
    /// coef[m][j]: tap j, power m.
    coef: Vec<[f32; 8]>,
}

impl Default for Farrow {
    fn default() -> Self {
        Self::new()
    }
}

impl Farrow {
    pub const TAPS: usize = 8;
    const DEG: usize = 5;

    pub fn new() -> Farrow {
        let half = Self::TAPS as f64 / 2.0;
        let kernel = |tau: f64| 2.0 * 0.46 * sinc(2.0 * 0.46 * tau) * kaiser(tau / (half + 0.5), 6.0);
        // Least-squares fit per tap over mu in [0, 1).
        let grid: Vec<f64> = (0..=200).map(|k| k as f64 / 200.0).collect();
        let mut coef = vec![[0.0f32; 8]; Self::DEG + 1];
        for j in 0..Self::TAPS {
            let rel = j as f64 - half + 1.0;
            // Normal equations.
            let d = Self::DEG + 1;
            let mut a = vec![vec![0.0f64; d]; d];
            let mut b = vec![0.0f64; d];
            for &mu in &grid {
                // Normalise each phase to unity DC gain.
                let s: f64 = (0..Self::TAPS).map(|i| kernel(mu - (i as f64 - half + 1.0))).sum();
                let y = kernel(mu - rel) / s;
                let mut pw = [1.0f64; 8];
                for m in 1..d {
                    pw[m] = pw[m - 1] * mu;
                }
                for r in 0..d {
                    b[r] += pw[r] * y;
                    for c in 0..d {
                        a[r][c] += pw[r] * pw[c];
                    }
                }
            }
            let x = solve(a, b);
            for m in 0..d {
                coef[m][j] = x[m] as f32;
            }
        }
        Farrow { coef }
    }

    /// Value and derivative (per sample) of `x` at `i + mu`, using
    /// x[i-3..=i+4].
    #[inline]
    pub fn eval(&self, re: &[f32], im: &[f32], i: usize, mu: f32) -> (Complex32, Complex32) {
        let r = &re[i - 3..i + 5];
        let q = &im[i - 3..i + 5];
        let mut vr = [0.0f32; 6];
        let mut vi = [0.0f32; 6];
        for m in 0..=Self::DEG {
            let c = &self.coef[m];
            let (mut a, mut b) = (0.0f32, 0.0f32);
            for j in 0..8 {
                a += c[j] * r[j];
                b += c[j] * q[j];
            }
            vr[m] = a;
            vi[m] = b;
        }
        let (mut yr, mut yi, mut dr, mut di) = (vr[5], vi[5], 5.0 * vr[5], 5.0 * vi[5]);
        for m in (0..5).rev() {
            yr = yr * mu + vr[m];
            yi = yi * mu + vi[m];
            if m > 0 {
                dr = dr * mu + m as f32 * vr[m];
                di = di * mu + m as f32 * vi[m];
            }
        }
        (Complex32::new(yr, yi), Complex32::new(dr, di))
    }
}

fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for c in 0..n {
        let p = (c..n).max_by(|&x, &y| a[x][c].abs().total_cmp(&a[y][c].abs())).unwrap();
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|k| a[r][k] * x[k]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    x
}

#[derive(Clone, Debug, Default)]
struct Branch {
    re: Vec<f32>,
    im: Vec<f32>,
    /// Absolute index of re[0] in this branch's stream.
    base: u64,
    /// Latest push: (host_ns, end index).
    last_push: Option<(u64, u64)>,
    acquired: bool,
    /// Branch position of reference sample `n_ref` is `pos + (n - n_ref) * rate`.
    pos: f64,
    rate: f64,
    /// Derotation phase (rad) at the block start and frequency (rad/sample).
    phase: f64,
    freq: f64,
    noise: f64,
    sig: f64,
    cn: f64,
    weight: f32,
    prev_weight: f32,
    corr: f32,
    next_acq: u64,
    lost_blocks: u32,
}

impl Branch {
    fn end(&self) -> u64 {
        self.base + self.re.len() as u64
    }
}

/// The combiner.
pub struct PredetectCombiner {
    cfg: PredetectConfig,
    fs: f64,
    br: Vec<Branch>,
    farrow: Farrow,
    /// Reference index of the next output sample.
    n_out: u64,
    started: bool,
    ys: Vec<Vec<Complex32>>,
    ds: Vec<Vec<Complex32>>,
}

impl PredetectCombiner {
    pub fn new(branches: usize, sample_rate_hz: f64, cfg: PredetectConfig) -> Self {
        let n = branches.max(1);
        let mut br = vec![Branch::default(); n];
        br[0].acquired = true;
        br[0].rate = 1.0;
        for b in br.iter_mut() {
            b.rate = 1.0;
            b.noise = f64::NAN;
        }
        PredetectCombiner {
            cfg,
            fs: sample_rate_hz,
            br,
            farrow: Farrow::new(),
            n_out: 0,
            started: false,
            ys: vec![Vec::new(); n],
            ds: vec![Vec::new(); n],
        }
    }

    /// Appends DC-corrected samples of branch `k`.
    pub fn push(&mut self, k: usize, iq: &[Complex32], host_ns: u64) {
        let b = &mut self.br[k];
        b.re.extend(iq.iter().map(|z| z.re));
        b.im.extend(iq.iter().map(|z| z.im));
        b.last_push = Some((host_ns, b.end()));
    }

    /// Declares lost samples on branch `k` (USB gap): that branch
    /// re-acquires.
    pub fn mark_gap(&mut self, k: usize, lost: u64) {
        let b = &mut self.br[k];
        b.base += b.re.len() as u64 + lost;
        b.re.clear();
        b.im.clear();
        if k == 0 {
            // The reference timeline jumps; every branch re-acquires.
            self.n_out = b.base;
            for o in self.br.iter_mut().skip(1) {
                o.acquired = false;
            }
        } else {
            b.acquired = false;
        }
    }

    pub fn branch_stats(&self) -> Vec<BranchStats> {
        self.br
            .iter()
            .enumerate()
            .map(|(k, b)| BranchStats {
                acquired: b.acquired,
                delay: if k == 0 { 0.0 } else { b.pos - self.n_out as f64 },
                rate_ppm: (b.rate - 1.0) * 1e6,
                freq_hz: b.freq * self.fs / std::f64::consts::TAU,
                weight: b.weight,
                cn_db: crate::cn::lin_to_db(b.cn),
                corr: b.corr,
            })
            .collect()
    }

    /// Reference index of the next output sample.
    pub fn next_index(&self) -> u64 {
        self.n_out
    }

    /// Produces every combined sample the buffered data allows.
    pub fn pull(&mut self, out: &mut Vec<Complex32>) {
        if !self.started {
            self.n_out = self.br[0].base;
            self.started = true;
        }
        for k in 1..self.br.len() {
            if !self.br[k].acquired {
                self.try_acquire(k);
            }
        }
        let bl = self.cfg.block.max(16);
        loop {
            let n0 = self.n_out;
            let r = &self.br[0];
            if n0 < r.base + 4 || n0 + bl as u64 + 5 > r.end() {
                break;
            }
            // Wait for acquired branches unless they lag too far.
            let mut wait = false;
            for b in self.br.iter().skip(1).filter(|b| b.acquired) {
                let need = b.pos + (bl as f64 + 1.0) * b.rate + 6.0;
                if (b.end() as f64) < need && r.end() - n0 < self.cfg.max_lag as u64 {
                    wait = true;
                }
            }
            if wait {
                break;
            }
            self.block(bl, out);
            self.n_out += bl as u64;
        }
        self.trim();
    }

    fn trim(&mut self) {
        let keep_ref = self.n_out.saturating_sub((self.cfg.acq_len + 2 * self.cfg.search + 64) as u64);
        let r = &mut self.br[0];
        if keep_ref > r.base + 65536 {
            let cut = ((keep_ref - r.base) as usize).min(r.re.len());
            r.re.drain(..cut);
            r.im.drain(..cut);
            r.base += cut as u64;
        }
        let limit = self.cfg.acq_len + 4 * self.cfg.search + 4 * self.cfg.max_lag;
        for b in self.br.iter_mut().skip(1) {
            let keep = if b.acquired {
                (b.pos as u64).saturating_sub(64)
            } else {
                b.end().saturating_sub(limit as u64)
            };
            if keep > b.base + 65536 {
                let cut = ((keep - b.base) as usize).min(b.re.len());
                b.re.drain(..cut);
                b.im.drain(..cut);
                b.base += cut as u64;
            }
        }
    }

    fn try_acquire(&mut self, k: usize) {
        let l = self.cfg.acq_len.next_power_of_two();
        let s = self.cfg.search;
        let r_end = self.br[0].end();
        let b = &self.br[k];
        if r_end < self.br[0].base + l as u64 || b.re.len() < l + 2 * s || b.end() < b.next_acq {
            return;
        }
        // Initial guess: map the reference window end to the branch through
        // the latest push times (both streams live), else by stream ends.
        let r_win_end = r_end;
        let r_win_start = r_win_end - l as u64;
        let guess_end = match (self.br[0].last_push, b.last_push) {
            (Some((tr, er)), Some((tb, eb))) if tr > 0 && tb > 0 => {
                let dt = (tb as f64 - tr as f64) * 1e-9 * self.fs;
                eb as f64 - dt - (er as f64 - r_win_end as f64)
            }
            _ => b.end() as f64 - (self.br[0].end() as f64 - r_win_end as f64),
        };
        let guess_start = guess_end - l as f64;
        let x0 = (guess_start - s as f64).round() as i64;
        let xi0 = x0 - b.base as i64;
        if xi0 < 0 || xi0 as usize + l + 2 * s + 1 > b.re.len() {
            self.br[k].next_acq = b.end() + (l / 4) as u64;
            return;
        }
        let xi0 = xi0 as usize;
        let r = &self.br[0];
        let ri0 = (r_win_start - r.base) as usize;
        let fr: Vec<f32> = disc(&r.re[ri0..ri0 + l], &r.im[ri0..ri0 + l]);
        let fb: Vec<f32> = disc(&b.re[xi0..xi0 + l + 2 * s], &b.im[xi0..xi0 + l + 2 * s]);
        // FFT cross-correlation of the demodulated signals over lags 0..2s.
        let n = (l + 2 * s).next_power_of_two() * 2;
        let mut planner = FftPlanner::<f32>::new();
        let fwd = planner.plan_fft_forward(n);
        let inv = planner.plan_fft_inverse(n);
        let mr = fr.iter().sum::<f32>() / fr.len() as f32;
        let mb = fb.iter().sum::<f32>() / fb.len() as f32;
        let mut a: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new(if i < fr.len() { fr[i] - mr } else { 0.0 }, 0.0))
            .collect();
        let mut c: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new(if i < fb.len() { fb[i] - mb } else { 0.0 }, 0.0))
            .collect();
        fwd.process(&mut a);
        fwd.process(&mut c);
        for (x, y) in c.iter_mut().zip(&a) {
            *x *= y.conj();
        }
        inv.process(&mut c);
        // c[lag] = sum fb[i + lag] fr[i].
        let lag = (0..=2 * s).max_by(|&p, &q| c[p].re.total_cmp(&c[q].re)).unwrap();
        let pos0 = xi0 + lag; // branch local index matching r_win_start
        // Frequency offset from the FFT of z_r conj(z_b).
        let mut p: Vec<Complex32> = (0..l)
            .map(|i| {
                let zr = Complex32::new(r.re[ri0 + i], r.im[ri0 + i]);
                let zb = Complex32::new(b.re[pos0 + i], b.im[pos0 + i]);
                zr * zb.conj()
            })
            .collect();
        let fl = planner.plan_fft_forward(l);
        fl.process(&mut p);
        let pk = (0..l).max_by(|&x, &y| p[x].norm_sqr().total_cmp(&p[y].norm_sqr())).unwrap();
        let m = |i: usize| p[(i + l) % l].norm();
        let (y0, y1, y2) = (m(pk + l - 1), m(pk), m(pk + 1));
        let den = y0 - 2.0 * y1 + y2;
        let off = if den.abs() > 1e-12 { 0.5 * (y0 - y2) / den } else { 0.0 };
        let mut bin = pk as f64 + off as f64;
        if bin > l as f64 / 2.0 {
            bin -= l as f64;
        }
        // z_r conj(z_b) rotates at w_p = 2 pi bin / l = w_r - w_b; the
        // branch must be rotated by exp(-j (w_b - w_r) n) = exp(+j w_p n).
        let w_rel = -std::f64::consts::TAU * bin / l as f64;
        // Fractional delay and rate from complex correlation peaks of both
        // halves.
        let half = l / 2;
        let frac_peak = |from: usize| -> (f64, f32) {
            let corr = |d: i64| -> Complex32 {
                let mut acc = Complex32::new(0.0, 0.0);
                for i in from..from + half {
                    let bi = (pos0 as i64 + i as i64 + d) as usize;
                    let zr = Complex32::new(r.re[ri0 + i], r.im[ri0 + i]);
                    let zb = Complex32::new(b.re[bi], b.im[bi]);
                    let (sn, cs) = sin_cos(wrap_pi((w_rel * (i as f64 + d as f64)) as f32));
                    acc += zr * (zb * Complex32::new(cs, sn)).conj();
                }
                acc
            };
            let (c0, c1, c2) = (corr(-1).norm(), corr(0).norm(), corr(1).norm());
            let den = c0 - 2.0 * c1 + c2;
            let d = if den.abs() > 1e-12 { 0.5 * (c0 - c2) / den } else { 0.0 };
            let er: f32 = (from..from + half).map(|i| r.re[ri0 + i].powi(2) + r.im[ri0 + i].powi(2)).sum();
            let eb: f32 = (from..from + half)
                .map(|i| b.re[pos0 + i].powi(2) + b.im[pos0 + i].powi(2))
                .sum();
            (d.clamp(-1.0, 1.0) as f64, c1 / (er * eb).sqrt().max(1e-12))
        };
        let (d1, rho1) = frac_peak(0);
        let (d2, rho2) = frac_peak(half);
        let rho = 0.5 * (rho1 + rho2);
        if rho < self.cfg.min_corr {
            let e = self.br[k].end();
            self.br[k].next_acq = e + (l / 2) as u64;
            return;
        }
        let rate = 1.0 + ((d2 - d1) / half as f64).clamp(-2e-4, 2e-4);
        // Branch absolute position of reference index r_win_start, then
        // advanced to the next output sample.
        let pos_abs = b.base as f64 + pos0 as f64 + 0.5 * (d1 + d2) - 0.5 * (d2 - d1);
        let adv = self.n_out as f64 - r_win_start as f64;
        let bm = &mut self.br[k];
        bm.pos = pos_abs + adv * rate;
        bm.rate = rate;
        bm.freq = w_rel;
        bm.phase = 0.0;
        bm.acquired = true;
        bm.lost_blocks = 0;
        bm.noise = f64::NAN;
        bm.corr = rho;
        let ph = self.initial_phase(k);
        self.br[k].phase = ph;
    }

    /// Phase of the branch against the reference at the current output
    /// position, over one acquisition-length window.
    fn initial_phase(&self, k: usize) -> f64 {
        let b = &self.br[k];
        let r = &self.br[0];
        let mut acc = Complex32::new(0.0, 0.0);
        let len = 4096usize;
        for i in 0..len {
            let n = self.n_out + i as u64;
            if n < r.base || n >= r.end() {
                break;
            }
            let x = b.pos + i as f64 * b.rate;
            let xi = x.floor();
            let li = xi as i64 - b.base as i64;
            if li < 3 || li as usize + 5 >= b.re.len() {
                break;
            }
            let (y, _) = self.farrow.eval(&b.re, &b.im, li as usize, (x - xi) as f32);
            let (sn, cs) = sin_cos(wrap_pi((b.freq * i as f64) as f32));
            let y = y * Complex32::new(cs, sn);
            let ri = (n - r.base) as usize;
            acc += Complex32::new(r.re[ri], r.im[ri]) * y.conj();
        }
        // Rotation that aligns the branch with the reference.
        atan2(acc.im, acc.re) as f64
    }

    fn block(&mut self, bl: usize, out: &mut Vec<Complex32>) {
        let nb = self.br.len();
        let n0 = self.n_out;
        let r0 = (n0 - self.br[0].base) as usize;
        // Evaluate every branch on the reference grid.
        let mut active = vec![false; nb];
        for k in 0..nb {
            self.ys[k].resize(bl, Complex32::new(0.0, 0.0));
            self.ds[k].resize(bl, Complex32::new(0.0, 0.0));
        }
        {
            let r = &self.br[0];
            for i in 0..bl {
                self.ys[0][i] = Complex32::new(r.re[r0 + i], r.im[r0 + i]);
                // Derivative by central difference (only used for k > 0).
            }
            active[0] = true;
        }
        for k in 1..nb {
            let b = &self.br[k];
            if !b.acquired {
                continue;
            }
            let last = b.pos + (bl as f64) * b.rate;
            let li0 = b.pos.floor() as i64 - b.base as i64;
            let li1 = last.floor() as i64 - b.base as i64;
            if li0 < 3 || li1 as usize + 5 >= b.re.len() {
                continue;
            }
            active[k] = true;
            for i in 0..bl {
                let x = b.pos + i as f64 * b.rate;
                let xi = x.floor();
                let li = (xi as i64 - b.base as i64) as usize;
                let (y, d) = self.farrow.eval(&b.re, &b.im, li, (x - xi) as f32);
                let (sn, cs) = sin_cos(wrap_pi((b.phase + b.freq * i as f64) as f32));
                let rot = Complex32::new(cs, sn);
                self.ys[k][i] = y * rot;
                self.ds[k][i] = d * rot;
            }
        }
        // Per-branch power and C/N.
        for k in 0..nb {
            if !active[k] {
                self.br[k].weight = 0.0;
                continue;
            }
            let mut acc = EnvAcc::default();
            for y in &self.ys[k] {
                acc.add(y.norm_sqr());
            }
            let p = acc.mean();
            let c = acc.cn_linear();
            let n_est = p / (1.0 + c);
            let b = &mut self.br[k];
            b.noise = if b.noise.is_finite() { b.noise + 0.03 * (n_est - b.noise) } else { n_est };
            b.sig = (p - b.noise).max(0.0);
            b.cn = b.sig / b.noise.max(1e-12);
        }
        // Weights (MRC): sqrt(sig) / noise.
        for k in 0..nb {
            let b = &mut self.br[k];
            b.prev_weight = b.weight;
            b.weight = if active[k] && b.noise > 0.0 {
                (b.sig.sqrt() / b.noise) as f32
            } else {
                0.0
            };
        }
        let amp: f64 = self.br.iter().zip(&active).filter(|(_, a)| **a).map(|(b, _)| b.sig).sum::<f64>().sqrt();
        let wsum_new: f64 = self.br.iter().map(|b| b.weight as f64 * b.sig.sqrt()).sum();
        let wsum_old: f64 = self.br.iter().map(|b| b.prev_weight as f64 * b.sig.sqrt()).sum();
        // Combine with weights ramped from the previous block.
        let base = out.len();
        out.resize(base + bl, Complex32::new(0.0, 0.0));
        let scale_new = if wsum_new > 0.0 { amp / wsum_new } else { 0.0 };
        let scale_old = if wsum_old > 0.0 { amp / wsum_old } else { scale_new };
        for k in 0..nb {
            if !active[k] {
                continue;
            }
            let (w0, w1) = (
                self.br[k].prev_weight as f64 * scale_old,
                self.br[k].weight as f64 * scale_new,
            );
            let w0 = if self.br[k].prev_weight == 0.0 { w1 } else { w0 };
            for i in 0..bl {
                let t = (i as f64 + 0.5) / bl as f64;
                let w = (w0 + (w1 - w0) * t) as f32;
                out[base + i] += self.ys[k][i] * w;
            }
        }
        // Loop updates against the leave-one-out combination.
        let combined: Vec<Complex32> = out[base..base + bl].to_vec();
        let mut pending = Vec::new();
        for k in 1..nb {
            if !active[k] {
                continue;
            }
            let wk = self.br[k].weight as f64 * scale_new;
            let (mut c, mut cd, mut er, mut ey, mut ed) =
                (Complex32::new(0.0, 0.0), Complex32::new(0.0, 0.0), 0.0f32, 0.0f32, 0.0f32);
            for i in 0..bl {
                let rr = combined[i] - self.ys[k][i] * wk as f32;
                c += rr * self.ys[k][i].conj();
                cd += rr * self.ds[k][i].conj();
                er += rr.norm_sqr();
                ey += self.ys[k][i].norm_sqr();
                ed += self.ds[k][i].norm_sqr();
            }
            let rho = c.norm() / (er * ey).sqrt().max(1e-12);
            pending.push((k, c, cd, rho, ey, ed));
        }
        for (k, c, cd, rho, ey, ed) in pending {
            let b = &mut self.br[k];
            b.corr = rho;
            let rel = (rho / 0.3).min(1.0) as f64;
            // Phase: rotate the branch by arg(c).
            let e_ph = atan2(c.im, c.re) as f64;
            // Delay: correlation gradient over the curvature.
            let beta2 = (ed / ey.max(1e-12)) as f64;
            let g = (cd * c.conj()).re as f64;
            let e_d = if c.norm_sqr() > 0.0 && beta2 > 0.0 {
                (-g / (beta2 * c.norm_sqr() as f64)).clamp(-0.5, 0.5)
            } else {
                0.0
            };
            let gp = self.cfg.phase_gain as f64 * rel;
            let gd = self.cfg.delay_gain as f64 * rel;
            b.freq += 0.25 * gp * gp * e_ph / bl as f64;
            b.phase = wrap_pi64(b.phase + b.freq * bl as f64 + gp * e_ph);
            b.rate += 0.05 * gd * gd * e_d / bl as f64;
            b.rate = b.rate.clamp(1.0 - 3e-4, 1.0 + 3e-4);
            b.pos += b.rate * bl as f64 - gd * e_d;
            // Lost lock: decorrelated while the branch itself is strong.
            if rho < self.cfg.min_corr && b.cn > 2.0 {
                b.lost_blocks += 1;
                if b.lost_blocks > 200 {
                    b.acquired = false;
                    b.next_acq = 0;
                }
            } else {
                b.lost_blocks = 0;
            }
        }
        for k in 1..nb {
            if !active[k] && self.br[k].acquired {
                let b = &mut self.br[k];
                b.pos += b.rate * bl as f64;
                b.phase = wrap_pi64(b.phase + b.freq * bl as f64);
            }
        }
    }
}

fn wrap_pi64(x: f64) -> f64 {
    crate::fastmath::wrap_pi64(x)
}

/// Polar discriminator (rad/sample).
fn disc(re: &[f32], im: &[f32]) -> Vec<f32> {
    let mut out = Vec::with_capacity(re.len());
    out.push(0.0);
    for i in 1..re.len() {
        let (a, b) = (re[i] * re[i - 1] + im[i] * im[i - 1], im[i] * re[i - 1] - re[i] * im[i - 1]);
        out.push(atan2(b, a));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn farrow_interpolates_value_and_slope() {
        let f = Farrow::new();
        let w = 0.9f64;
        let re: Vec<f32> = (0..100).map(|k| (w * k as f64).cos() as f32).collect();
        let im: Vec<f32> = (0..100).map(|k| (w * k as f64).sin() as f32).collect();
        let mut worst = 0.0f32;
        let mut worst_d = 0.0f32;
        for k in 0..300 {
            let x = 20.0 + k as f64 * 0.173;
            let i = x.floor() as usize;
            let (y, d) = f.eval(&re, &im, i, (x - i as f64) as f32);
            let e = Complex32::new((w * x).cos() as f32, (w * x).sin() as f32);
            worst = worst.max((y - e).norm());
            let ed = e * Complex32::new(0.0, w as f32);
            worst_d = worst_d.max((d - ed).norm());
        }
        assert!(worst < 0.01, "farrow value error {worst}");
        assert!(worst_d < 0.05, "farrow slope error {worst_d}");
    }

    use crate::testsig::{Fading, SigGen, SigSpec};

    /// SNR of `y` against reference `s` after the best complex scaling.
    fn snr_db(y: &[Complex32], s: &[Complex32]) -> f64 {
        let (mut c, mut py, mut ps) = (Complex32::new(0.0, 0.0), 0.0f64, 0.0f64);
        for (a, b) in y.iter().zip(s) {
            c += a * b.conj();
            py += a.norm_sqr() as f64;
            ps += b.norm_sqr() as f64;
        }
        let rho = (c.norm_sqr() as f64 / (py * ps)).min(0.999_999);
        10.0 * (rho / (1.0 - rho)).log10()
    }

    struct Scenario {
        branches: Vec<SigGen>,
        reference: SigGen,
    }

    fn scenario(cn: f64, fading: bool) -> Scenario {
        let base = SigSpec {
            cn_db: Some(cn),
            rms_adc: 150.0,
            ..SigSpec::default()
        };
        // Branch 1: LO 150 kHz away, sample clock 25 ppm fast, 1.37 us
        // later in the raster, 6 dB less gain, its own noise and fading.
        let b0 = SigSpec {
            seed: 3,
            fading: fading.then_some(Fading { doppler_hz: 300.0, seed: 101 }),
            ..base.clone()
        };
        let b1 = SigSpec {
            seed: 4,
            blank_hz: base.blank_hz + 150e3,
            rx_ppm: 25.0,
            start_us: base.start_us + 1.37,
            rms_adc: 75.0,
            fading: fading.then_some(Fading { doppler_hz: 300.0, seed: 202 }),
            ..base.clone()
        };
        let reference = SigSpec {
            seed: 3,
            cn_db: None,
            fading: None,
            ..base
        };
        Scenario {
            branches: vec![SigGen::new(b0), SigGen::new(b1)],
            reference: SigGen::new(reference),
        }
    }

    /// Runs the combiner; returns (branch 0 input, combined, unfaded clean
    /// reference) aligned on branch 0's timeline after acquisition.
    fn combine(sc: &mut Scenario, total: usize) -> (Vec<Complex32>, Vec<Complex32>, Vec<Complex32>, Vec<BranchStats>) {
        let fs = sc.branches[0].sample_rate();
        let mut comb = PredetectCombiner::new(2, fs, PredetectConfig::default());
        let (mut b0, mut refs, mut out) = (Vec::new(), Vec::new(), Vec::new());
        let chunk = 8192;
        let mut done = 0;
        while done < total {
            for k in 0..2 {
                let c = sc.branches[k].next(chunk);
                let iq: Vec<Complex32> = c.re.iter().zip(&c.im).map(|(&a, &b)| Complex32::new(a, b)).collect();
                if k == 0 {
                    b0.extend_from_slice(&iq);
                }
                comb.push(k, &iq, 0);
            }
            let c = sc.reference.next(chunk);
            refs.extend(c.clean_re.iter().zip(&c.clean_im).map(|(&a, &b)| Complex32::new(a, b)));
            comb.pull(&mut out);
            done += chunk;
        }
        let first = comb.next_index() as usize - out.len();
        let n = out.len().min(b0.len() - first).min(refs.len() - first);
        out.truncate(n);
        let st = comb.branch_stats();
        (b0[first..first + n].to_vec(), out, refs[first..first + n].to_vec(), st)
    }

    #[test]
    fn mrc_gains_about_3_db_with_realistic_impairments() {
        let mut sc = scenario(6.0, false);
        let (b0, out, refs, st) = combine(&mut sc, 400_000);
        assert!(st[1].acquired, "branch 1 not acquired");
        assert!((st[1].freq_hz.abs() - 150e3).abs() < 2e3, "freq {}", st[1].freq_hz);
        assert!((st[1].rate_ppm - 25.0).abs() < 3.0, "rate {}", st[1].rate_ppm);
        // Skip the convergence of the loops.
        let skip = out.len() / 3;
        let single = snr_db(&b0[skip..], &refs[skip..]);
        let combined = snr_db(&out[skip..], &refs[skip..]);
        // Branch 1 has 6 dB less gain but the same C/N: MRC gains 3 dB.
        assert!(combined - single > 2.3, "single {single:.2} dB combined {combined:.2} dB");
    }

    #[test]
    fn mrc_reduces_outage_under_independent_rayleigh_fading() {
        let mut sc = scenario(10.0, true);
        let (b0, out, refs, st) = combine(&mut sc, 900_000);
        assert!(st[1].acquired);
        let skip = out.len() / 3;
        let blk = 512;
        let (mut out_single, mut out_comb, mut n) = (0, 0, 0);
        let mut k = skip;
        while k + blk <= out.len() {
            let s1 = snr_db(&b0[k..k + blk], &refs[k..k + blk]);
            let s2 = snr_db(&out[k..k + blk], &refs[k..k + blk]);
            out_single += (s1 < 3.0) as usize;
            out_comb += (s2 < 3.0) as usize;
            n += 1;
            k += blk;
        }
        let (p1, p2) = (out_single as f64 / n as f64, out_comb as f64 / n as f64);
        assert!(p1 > 0.05, "single-branch outage {p1}");
        assert!(p2 < 0.4 * p1, "outage single {p1:.3} combined {p2:.3}");
    }
}
