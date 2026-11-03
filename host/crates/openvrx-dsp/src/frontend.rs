// SPDX-License-Identifier: GPL-3.0-or-later
//! Front end: DC offset removal, blind I/Q imbalance correction, gain-step
//! detection, raw statistics for the gain loop, live glitch counter.
//!
//! DC: each sub-block (`DcConfig::block` samples) gets a centre estimate.
//! When the envelope is clearly constant (an FM carrier), the centre is the
//! Kasa least-squares circle fit, which does not depend on where the FM
//! phase dwells; otherwise it is the plain mean. The circle-centre idea
//! follows C5VRX (GPL-3.0, Twotoz and the C5VRX contributors, commit
//! 6bc9e4b, `main/dsp.h` `dsp_circle_dc`); here it runs in floating point
//! on 10-bit units relative to the running estimate, and is combined with
//! step detection and fast re-convergence.
//!
//! Gain steps: after `arm()` (a `GainChanged` event) the power and the
//! centre of every sub-block are compared with the running values. A step
//! is located to 16 samples by a power split search inside the sub-block;
//! samples before it keep the old DC, samples after get the new one, and
//! the tracker restarts as a cumulative mean (fast re-convergence) before
//! falling back to its normal time constant. Unannounced DC jumps are also
//! caught (two consecutive sub-blocks off by a large margin).
//!
//! I/Q imbalance: for a circular signal E[I^2] = E[Q^2] and E[IQ] = 0.
//! With I = a cos t, Q = a g sin(t + p): g = sqrt(E[Q^2]/E[I^2]),
//! sin p = E[IQ]/sqrt(E[I^2] E[Q^2]); the correction is
//! Q' = (Q/g - I sin p)/cos p.

use crate::cn::EnvAcc;
use crate::vmath::{apply_pass, circle_sums, glitches, raw_pass};
use crate::config::{DcConfig, GlitchConfig, IqConfig};

/// Raw statistics over a window.
#[derive(Copy, Clone, Debug, Default)]
pub struct RawStats {
    pub samples: u64,
    pub i_rms: f32,
    pub q_rms: f32,
    pub i_dc: f32,
    pub q_dc: f32,
    pub clip_pct: f32,
    pub power_db: f32,
    pub cn_db: f32,
    pub iq_gain_db: f32,
    pub iq_phase_deg: f32,
    /// Off-ring glitches per million samples (NaN when not measurable).
    pub glitch_ppm: f32,
}

#[derive(Copy, Clone, Debug, Default)]
struct WinAcc {
    n: u64,
    si: f64,
    sq: f64,
    sii: f64,
    sqq: f64,
    clips: u64,
    env: EnvAcc,
    glitch_n: u64,
    glitches: u64,
}

/// Front-end state.
#[derive(Clone, Debug)]
pub struct FrontEnd {
    fs: f64,
    has_mag: bool,
    clip_level: f32,
    dc_cfg: DcConfig,
    iq_cfg: IqConfig,
    glitch_cfg: GlitchConfig,
    dc_i: f64,
    dc_q: f64,
    dc_valid: bool,
    /// Sub-blocks averaged since a reset (cumulative mean phase), 0 = normal.
    fast_n: u32,
    armed_until: u64,
    pwr_avg: f64,
    pwr_valid: bool,
    /// Pending unannounced jump (centre seen off once).
    pending_jump: bool,
    /// Step split inside the current sub-block: (index, old DC i, old DC q).
    split: Option<(usize, f32, f32)>,
    // I/Q moments (smoothed) and correction.
    m_ii: f64,
    m_qq: f64,
    m_iq: f64,
    iq_valid: bool,
    corr_c: f32,
    corr_d: f32,
    /// Detected gain-step sample indices, drained by the caller.
    pub steps: Vec<u64>,
    win: WinAcc,
    win_len: u64,
    last: RawStats,
    /// Latest sub-block C/N (linear), for the demodulators.
    pub block_cn: f64,
}

impl FrontEnd {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        fs: f64,
        has_mag: bool,
        full_scale: f32,
        step: f32,
        dc: DcConfig,
        iq: IqConfig,
        glitch: GlitchConfig,
        stats_window_ms: f32,
    ) -> FrontEnd {
        FrontEnd {
            fs,
            has_mag,
            clip_level: full_scale - step,
            dc_cfg: dc,
            iq_cfg: iq,
            glitch_cfg: glitch,
            dc_i: 0.0,
            dc_q: 0.0,
            dc_valid: false,
            fast_n: 0,
            armed_until: 0,
            pwr_avg: 0.0,
            pwr_valid: false,
            pending_jump: false,
            split: None,
            m_ii: 0.0,
            m_qq: 0.0,
            m_iq: 0.0,
            iq_valid: false,
            corr_c: 0.0,
            corr_d: 1.0,
            steps: Vec::new(),
            win: WinAcc::default(),
            win_len: ((stats_window_ms as f64 * 1e-3 * fs) as u64).max(1024),
            last: RawStats {
                cn_db: f32::NAN,
                glitch_ppm: f32::NAN,
                ..RawStats::default()
            },
            block_cn: 0.0,
        }
    }

    /// Arms the gain-step detector until absolute sample `until`.
    pub fn arm(&mut self, until: u64) {
        self.armed_until = until;
    }

    /// Mean signal-plus-noise power after DC removal (ADC units^2).
    pub fn mean_power(&self) -> f64 {
        self.pwr_avg
    }

    pub fn dc(&self) -> (f32, f32) {
        (self.dc_i as f32, self.dc_q as f32)
    }

    pub fn iq_correction(&self) -> (f32, f32) {
        (self.corr_c, self.corr_d)
    }

    /// Statistics of the last completed window.
    pub fn stats(&self) -> RawStats {
        self.last
    }

    /// Processes samples in place. `env2` receives |z|^2 after correction,
    /// `clip` gets 1 where a raw component reached full scale.
    pub fn process(
        &mut self,
        re: &mut [f32],
        im: &mut [f32],
        env2: &mut [f32],
        clip: &mut [u8],
        start: u64,
    ) {
        let n = re.len();
        let b = self.dc_cfg.block.max(64);
        let mut pos = 0;
        while pos < n {
            let end = (pos + b).min(n);
            self.sub_block(
                &mut re[pos..end],
                &mut im[pos..end],
                &mut env2[pos..end],
                &mut clip[pos..end],
                start + pos as u64,
                end - pos,
                b,
            );
            pos = end;
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn sub_block(
        &mut self,
        re: &mut [f32],
        im: &mut [f32],
        env2: &mut [f32],
        clip: &mut [u8],
        start: u64,
        len: usize,
        nominal: usize,
    ) {
        // Raw statistics and clip flags.
        let ([si, sq, sii, sqq], mut clips) = raw_pass(re, im, self.clip_level, clip);
        if !self.has_mag {
            clip.fill(0);
            clips = 0;
        }
        self.win.n += len as u64;
        self.win.si += si;
        self.win.sq += sq;
        self.win.sii += sii;
        self.win.sqq += sqq;
        self.win.clips += clips;

        if self.has_mag && self.dc_cfg.enabled {
            self.track_dc(re, im, start, len, nominal);
        } else {
            self.dc_i = 0.0;
            self.dc_q = 0.0;
        }

        // Apply DC (already applied to any pre-step part by track_dc via
        // `split`) and the I/Q correction; collect moments.
        let (di, dq) = (self.dc_i as f32, self.dc_q as f32);
        let (cc, cd) = if self.has_mag && self.iq_cfg.enabled {
            (self.corr_c, self.corr_d)
        } else {
            (0.0, 1.0)
        };
        let split = self.split.take().unwrap_or((0, di, dq));
        let sp = split.0.min(len);
        let mut mom = [0.0f64; 3];
        for (lo, hi, oi, oq) in [(0, sp, split.1, split.2), (sp, len, di, dq)] {
            if hi > lo {
                let m = apply_pass(&mut re[lo..hi], &mut im[lo..hi], &mut env2[lo..hi], (oi, oq), (cc, cd));
                for j in 0..3 {
                    mom[j] += m[j];
                }
            }
        }
        if self.has_mag && self.iq_cfg.enabled && len >= 16 {
            let (a, b2, c) = (mom[0] / len as f64, mom[1] / len as f64, mom[2] / len as f64);
            let alpha = if self.iq_valid {
                (len as f64 / (self.iq_cfg.time_constant_ms as f64 * 1e-3 * self.fs)).min(1.0)
            } else {
                1.0
            };
            self.iq_valid = true;
            self.m_ii += alpha * (a - self.m_ii);
            self.m_qq += alpha * (b2 - self.m_qq);
            self.m_iq += alpha * (c - self.m_iq);
            if self.m_ii > 1e-6 && self.m_qq > 1e-6 {
                let g = (self.m_qq / self.m_ii).sqrt();
                let sp = (self.m_iq / (self.m_ii * self.m_qq).sqrt()).clamp(-0.5, 0.5);
                let cp = (1.0 - sp * sp).sqrt();
                self.corr_c = (-sp / cp) as f32;
                self.corr_d = (1.0 / (g * cp)) as f32;
            }
        }

        // Envelope statistics: power tracker, block C/N, glitches.
        let mut acc = EnvAcc::default();
        acc.add_slice(env2);
        let p = acc.mean();
        if !self.pwr_valid {
            self.pwr_avg = p;
            self.pwr_valid = true;
        } else {
            self.pwr_avg += 0.05 * (p - self.pwr_avg);
        }
        self.win.env.merge(&acc);
        let c = acc.cn_linear();
        self.block_cn = c;
        if self.glitch_cfg.enabled
            && self.has_mag
            && len >= 8
            && c >= 10f64.powf(self.glitch_cfg.min_cn_db as f64 / 10.0)
        {
            let a2 = p * c / (1.0 + c);
            let s2 = p / (1.0 + c);
            let a = a2.sqrt() as f32;
            let far = (0.5 * a).max(6.0 * s2.sqrt() as f32);
            let near = 0.3 * a;
            self.win.glitches += glitches(env2, a, far, near);
            self.win.glitch_n += (len - 2) as u64;
        }

        if self.win.n >= self.win_len {
            self.finish_window();
        }
    }

    fn finish_window(&mut self) {
        let w = self.win;
        let n = w.n.max(1) as f64;
        let mi = w.si / n;
        let mq = w.sq / n;
        let vi = (w.sii / n - mi * mi).max(0.0);
        let vq = (w.sqq / n - mq * mq).max(0.0);
        let (gain_db, phase_deg) = if self.iq_valid && self.m_ii > 0.0 && self.m_qq > 0.0 {
            let g = (self.m_qq / self.m_ii).sqrt();
            let sp = (self.m_iq / (self.m_ii * self.m_qq).sqrt()).clamp(-1.0, 1.0);
            (20.0 * g.log10(), sp.asin().to_degrees())
        } else {
            (0.0, 0.0)
        };
        self.last = RawStats {
            samples: w.n,
            i_rms: vi.sqrt() as f32,
            q_rms: vq.sqrt() as f32,
            i_dc: if self.has_mag { self.dc_i as f32 } else { 0.0 },
            q_dc: if self.has_mag { self.dc_q as f32 } else { 0.0 },
            clip_pct: (100.0 * w.clips as f64 / n) as f32,
            power_db: (10.0 * w.env.mean().max(1e-12).log10()) as f32,
            cn_db: if self.has_mag { w.env.cn_db() } else { f32::NAN },
            iq_gain_db: gain_db as f32,
            iq_phase_deg: phase_deg as f32,
            glitch_ppm: if w.glitch_n > 0 {
                (1e6 * w.glitches as f64 / w.glitch_n as f64) as f32
            } else {
                f32::NAN
            },
        };
        self.win = WinAcc::default();
    }

    /// Updates the DC estimate for one sub-block. May record a split point
    /// (samples before it use the previous DC).
    fn track_dc(&mut self, re: &[f32], im: &[f32], start: u64, len: usize, nominal: usize) {
        let est = |a: usize, b: usize, ci: f64, cq: f64| -> Option<(f64, f64, f64)> {
            circle_or_mean(&re[a..b], &im[a..b], ci, cq)
        };
        let Some((ei, eq, p)) = est(0, len, self.dc_i, self.dc_q) else {
            return;
        };
        let alpha_n = (len as f64 / (self.dc_cfg.time_constant_ms as f64 * 1e-3 * self.fs)).min(1.0);
        if !self.dc_valid {
            self.dc_i = ei;
            self.dc_q = eq;
            self.dc_valid = true;
            self.fast_n = 1;
            return;
        }
        let armed = start < self.armed_until;
        let shift = ((ei - self.dc_i).powi(2) + (eq - self.dc_q).powi(2)).sqrt();
        let sigma_c = (p / len as f64).sqrt();
        let ratio_db = if self.pwr_avg > 0.0 && p > 0.0 {
            10.0 * (p / self.pwr_avg).log10()
        } else {
            0.0
        };
        let partial = len < nominal / 2;
        let step = !partial
            && if armed {
                ratio_db.abs() > self.dc_cfg.step_db as f64 || shift > (2.0f64).max(4.0 * sigma_c)
            } else {
                let off = shift > (6.0f64).max(6.0 * sigma_c);
                let confirmed = off && self.pending_jump;
                self.pending_jump = off;
                confirmed
            };
        if step {
            // Locate the change inside the sub-block by power split.
            let at = if ratio_db.abs() > self.dc_cfg.step_db as f64 * 0.5 {
                power_split(re, im, self.dc_i, self.dc_q)
            } else {
                0
            };
            let (ni, nq) = if len - at >= 64 {
                est(at, len, ei, eq).map(|(a, b, _)| (a, b)).unwrap_or((ei, eq))
            } else {
                (ei, eq)
            };
            if at > 0 {
                self.split = Some((at, self.dc_i as f32, self.dc_q as f32));
            }
            self.dc_i = ni;
            self.dc_q = nq;
            self.fast_n = 1;
            self.pending_jump = false;
            self.pwr_avg = p;
            self.armed_until = 0;
            self.steps.push(start + at as u64);
            return;
        }
        if self.fast_n > 0 {
            let a = 1.0 / (self.fast_n as f64 + 1.0);
            self.dc_i += a * (ei - self.dc_i);
            self.dc_q += a * (eq - self.dc_q);
            self.fast_n += 1;
            if a < alpha_n {
                self.fast_n = 0;
            }
        } else {
            self.dc_i += alpha_n * (ei - self.dc_i);
            self.dc_q += alpha_n * (eq - self.dc_q);
        }
    }
}





/// Centre estimate of a block: circle fit when the envelope is constant
/// enough, else the mean. Returns (centre_i, centre_q, mean power about the
/// centre). Sums are taken relative to (ci, cq) for numerical accuracy.
pub fn circle_or_mean(re: &[f32], im: &[f32], ci: f64, cq: f64) -> Option<(f64, f64, f64)> {
    let n = re.len();
    if n < 32 {
        return None;
    }
    let s = circle_sums(re, im, ci as f32, cq as f32);
    let nf = n as f64;
    let (mx, my) = (s[0] / nf, s[1] / nf);
    let sxx = s[2] / nf - mx * mx;
    let syy = s[3] / nf - my * my;
    let sxy = s[4] / nf - mx * my;
    let mz = s[5] / nf;
    let szx = s[6] / nf - mz * mx;
    let szy = s[7] / nf - mz * my;
    let det = sxx * syy - sxy * sxy;
    let mean_power = sxx + syy;
    if det > 1e-9 * (sxx + syy).powi(2).max(1e-12) {
        let a = 0.5 * (szx * syy - szy * sxy) / det;
        let b = 0.5 * (szy * sxx - szx * sxy) / det;
        // Envelope constancy about the fitted centre.
        let m1 = mz - 2.0 * a * mx - 2.0 * b * my + a * a + b * b;
        let ezz = s[8] / nf;
        let (ezx, ezy) = (s[6] / nf, s[7] / nf);
        let (exx, eyy, exy) = (s[2] / nf, s[3] / nf, s[4] / nf);
        let eu2 = ezz + 4.0 * a * a * exx + 4.0 * b * b * eyy - 4.0 * a * ezx - 4.0 * b * ezy
            + 8.0 * a * b * exy;
        let eu = mz - 2.0 * a * mx - 2.0 * b * my;
        let var = eu2 - eu * eu;
        let ratio = if var > 0.0 { m1 * m1 / var } else { 1e9 };
        // A constant envelope, and the centre no farther from the mean than
        // the radius (always true for points on a circle).
        if ratio >= 2.0 && (a - mx).powi(2) + (b - my).powi(2) <= 1.2 * m1.max(0.0) {
            let p = (m1).max(0.0);
            return Some((ci + a, cq + b, p));
        }
    }
    Some((ci + mx, cq + my, mean_power.max(0.0)))
}

/// Position (multiple of 16) inside the block where mean power changes
/// most, by maximising the two-segment likelihood contrast.
fn power_split(re: &[f32], im: &[f32], ci: f64, cq: f64) -> usize {
    let n = re.len();
    let chunks = n / 16;
    if chunks < 2 {
        return 0;
    }
    let (ci, cq) = (ci as f32, cq as f32);
    let e: Vec<f64> = (0..chunks)
        .map(|c| {
            let mut s = 0.0f32;
            for k in c * 16..c * 16 + 16 {
                let x = re[k] - ci;
                let y = im[k] - cq;
                s += x * x + y * y;
            }
            s as f64
        })
        .collect();
    let total: f64 = e.iter().sum();
    let mut best = (0.0f64, 0usize);
    let mut left = 0.0;
    for s in 1..chunks {
        left += e[s - 1];
        let (nl, nr) = (s as f64, (chunks - s) as f64);
        let ml = left / nl;
        let mr = (total - left) / nr;
        // Log-likelihood gain for exponential-ish power segments.
        let all = total / chunks as f64;
        let g = chunks as f64 * all.max(1e-12).ln() - nl * ml.max(1e-12).ln() - nr * mr.max(1e-12).ln();
        if g > best.0 {
            best = (g, s);
        }
    }
    best.1 * 16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rng(seed: u64) -> impl FnMut() -> f64 {
        let mut s = seed;
        move || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }
    }

    fn fe(fs: f64) -> FrontEnd {
        FrontEnd::new(
            fs,
            true,
            512.0,
            2.0,
            DcConfig::default(),
            IqConfig::default(),
            GlitchConfig::default(),
            10.0,
        )
    }

    /// FM-like carrier with noise, DC and imbalance.
    fn carrier(n: usize, dc: (f64, f64), seed: u64, amp: f64) -> (Vec<f32>, Vec<f32>) {
        let mut u = rng(seed);
        let mut ph = 0.0f64;
        let (mut re, mut im) = (Vec::new(), Vec::new());
        for k in 0..n {
            ph += 0.9 * (k as f64 * 0.0007).sin() - 0.45;
            let r = (-2.0 * u().ln()).sqrt() * 3.0;
            let t = std::f64::consts::TAU * u();
            re.push((amp * ph.cos() + dc.0 + r * t.cos()) as f32);
            im.push((amp * ph.sin() + dc.1 + r * t.sin()) as f32);
        }
        (re, im)
    }

    #[test]
    fn removes_dc_and_recovers_after_gain_step() {
        let fs = 13.333e6;
        let mut f = fe(fs);
        let n = 200_000;
        let (mut re, mut im) = carrier(n, (37.0, -5.0), 1, 60.0);
        // Gain step at sample 120_037: amplitude x1.25, DC jumps to 52.
        let step = 120_037;
        let (r2, i2) = carrier(n, (52.0, -2.0), 2, 75.0);
        re[step..].copy_from_slice(&r2[step..]);
        im[step..].copy_from_slice(&i2[step..]);
        let mut env = vec![0.0; n];
        let mut clip = vec![0u8; n];
        let mut pos = 0;
        while pos < n {
            let e = (pos + 4096).min(n);
            if pos <= 118_000 && e > 118_000 {
                f.arm(118_000 + (0.06 * fs) as u64);
            }
            f.process(&mut re[pos..e], &mut im[pos..e], &mut env[pos..e], &mut clip[pos..e], pos as u64);
            pos = e;
        }
        assert_eq!(f.steps.len(), 1, "steps {:?}", f.steps);
        assert!((f.steps[0] as i64 - step as i64).abs() <= 16, "step at {}", f.steps[0]);
        let (di, dq) = f.dc();
        assert!((di - 52.0).abs() < 0.5 && (dq + 2.0).abs() < 0.5, "dc {di} {dq}");
        // Residual mean right after the step (first 2000 samples) is small.
        let m: f32 = re[step + 600..step + 2600].iter().sum::<f32>() / 2000.0;
        assert!(m.abs() < 3.0, "post-step residual {m}");
        // Before the step the DC was right too.
        let m0: f32 = re[100_000..110_000].iter().sum::<f32>() / 10_000.0;
        assert!(m0.abs() < 1.0, "pre-step residual {m0}");
    }

    #[test]
    fn corrects_iq_imbalance() {
        let fs = 13.333e6;
        let mut f = fe(fs);
        let n = 600_000;
        let (g, p) = (1.15f64, 0.12f64);
        let mut u = rng(9);
        let mut ph = 0.0f64;
        let (mut re, mut im) = (Vec::new(), Vec::new());
        for k in 0..n {
            ph += 0.7 * (k as f64 * 0.0003).sin() + 0.2;
            let r = (-2.0 * u().ln()).sqrt();
            let t = std::f64::consts::TAU * u();
            re.push((80.0 * ph.cos() + r * t.cos()) as f32);
            im.push((80.0 * g * (ph + p).sin() + r * t.sin()) as f32);
        }
        let mut env = vec![0.0; n];
        let mut clip = vec![0u8; n];
        for s in (0..n).step_by(8192) {
            let e = (s + 8192).min(n);
            f.process(&mut re[s..e], &mut im[s..e], &mut env[s..e], &mut clip[s..e], s as u64);
        }
        // After convergence the output is circular.
        let tail = n - 100_000..n;
        let (mut a, mut b, mut c) = (0.0f64, 0.0f64, 0.0f64);
        for k in tail {
            a += (re[k] as f64).powi(2);
            b += (im[k] as f64).powi(2);
            c += re[k] as f64 * im[k] as f64;
        }
        let gain_err = (b / a).sqrt();
        let corr = c / (a * b).sqrt();
        assert!((gain_err - 1.0).abs() < 0.01, "gain {gain_err}");
        assert!(corr.abs() < 0.01, "corr {corr}");
        let st = f.stats();
        assert!((st.iq_gain_db - 20.0 * 1.15f32.log10()).abs() < 0.1, "{}", st.iq_gain_db);
        assert!((st.iq_phase_deg - 0.12f32.to_degrees()).abs() < 0.3, "{}", st.iq_phase_deg);
    }

    #[test]
    fn circle_fit_beats_mean_on_partial_arc() {
        // A carrier dwelling on part of the circle biases the mean.
        let mut re = Vec::new();
        let mut im = Vec::new();
        for k in 0..2000 {
            let a = 0.3 + 1.5 * (k as f64 / 2000.0);
            re.push((10.0 + 50.0 * a.cos()) as f32);
            im.push((-4.0 + 50.0 * a.sin()) as f32);
        }
        let (ci, cq, _) = circle_or_mean(&re, &im, 0.0, 0.0).unwrap();
        assert!((ci - 10.0).abs() < 0.01 && (cq + 4.0).abs() < 0.01, "{ci} {cq}");
    }

    #[test]
    fn glitch_counter_sees_off_ring_samples() {
        let fs = 13.333e6;
        let mut f = fe(fs);
        let n = 400_000;
        let (mut re, mut im) = carrier(n, (0.0, 0.0), 4, 100.0);
        for k in (1000..n).step_by(997) {
            re[k] = -re[k] * 0.1 + 7.0;
            im[k] *= 0.05;
        }
        let mut env = vec![0.0; n];
        let mut clip = vec![0u8; n];
        f.process(&mut re, &mut im, &mut env, &mut clip, 0);
        let ppm = f.stats().glitch_ppm;
        assert!((ppm - 1e6 / 997.0).abs() < 150.0, "glitch ppm {ppm}");
    }
}
