// SPDX-License-Identifier: GPL-3.0-or-later
//! FM click and phase-wrap repair on the discriminator output.
//!
//! Three mechanisms, in order, on each sample that stands out from its
//! neighbours (spike `e = d[n] - (d[n-1] + d[n+1])/2`):
//!
//! 1. Wrap: a run of samples that jumps by more than pi and back within a
//!    few samples crossed +-pi (deviation past the Nyquist of the phase
//!    difference, as at 13.33 MS/s near the sync tip). Moving the run by
//!    2 pi restores it exactly.
//! 2. Slip: a short span (2h+1 samples) whose excess over a straight
//!    baseline through the flanks integrates to about +-2 pi is an FM
//!    click (the phase went once around the origin during a fade). The
//!    span is replaced by the baseline.
//! 3. Impulse: a spike near +-pi inside an envelope dip (a half slip) is
//!    replaced by its neighbours.
//!
//! Repaired samples get `damage::REPAIRED`; samples in an envelope dip get
//! `damage::NOISY`. The repair runs with a fixed look-ahead, so its delay
//! is constant ([`ClickRepair::delay`]).

use crate::config::ClickConfig;
use openvrx_core::damage;
use std::f32::consts::{PI, TAU};

/// Samples per chunk of the vectorised candidate tests.
const CHUNK: i64 = 64;

/// True when some `i` in `i0..i1` has `|d[i] - d[i-1]| > pi` (or NaN).
fn any_jump(d: &[f32], i0: usize, i1: usize) -> bool {
    let (a, b) = (&d[i0 - 1..i1 - 1], &d[i0..i1]);
    a.iter().zip(b).fold(false, |acc, (&p, &x)| {
        let j = (x - p).abs();
        acc | (j > PI) | j.is_nan()
    })
}

/// True when some `i` in `i0..i1` has a spike `|d[i] - (d[i-1] + d[i+1])/2|`
/// of at least `thr` (or NaN).
fn any_spike(d: &[f32], i0: usize, i1: usize, thr: f32) -> bool {
    let (l, m, r) = (&d[i0 - 1..i1 - 1], &d[i0..i1], &d[i0 + 1..i1 + 1]);
    let mut hit = false;
    for k in 0..m.len() {
        let e = (m[k] - 0.5 * (l[k] + r[k])).abs();
        hit |= (e >= thr) | e.is_nan();
    }
    hit
}

#[derive(Clone, Debug)]
pub struct ClickRepair {
    cfg: ClickConfig,
    h: usize,
    /// Absolute index of `d[0]`.
    b0: i64,
    d: Vec<f32>,
    env: Vec<f32>,
    dmg: Vec<u8>,
    scan_next: i64,
    out_next: i64,
    /// Absolute indices (stream of this stage's input) of counted clicks.
    pub events: Vec<i64>,
    /// Wraps corrected without an envelope dip (over-deviation).
    pub wraps: u64,
}

impl ClickRepair {
    /// `tracking` widens the span (PLL/EKF spread slips over more samples).
    pub fn new(cfg: &ClickConfig, tracking: bool) -> ClickRepair {
        let h = cfg.half_span.max(1) * if tracking { 2 } else { 1 };
        Self::with_half(cfg, h)
    }

    fn with_half(cfg: &ClickConfig, h: usize) -> ClickRepair {
        let delay = 2 * h + 3;
        ClickRepair {
            cfg: cfg.clone(),
            h,
            b0: -(delay as i64),
            d: vec![0.0; delay],
            env: vec![0.0; delay],
            dmg: vec![damage::LOST; delay],
            scan_next: -(delay as i64),
            out_next: -(delay as i64),
            events: Vec::new(),
            wraps: 0,
        }
    }

    /// Constant delay in samples.
    pub fn delay(&self) -> usize {
        2 * self.h + 3
    }

    /// Restarts the stream (after a gap); counters are kept.
    pub fn reset(&mut self) {
        let mut fresh = Self::with_half(&self.cfg, self.h);
        fresh.events = std::mem::take(&mut self.events);
        fresh.wraps = self.wraps;
        *self = fresh;
    }

    /// Processes a block. `start` is the absolute index of `d_in[0]` in this
    /// stage's input stream; `a2` is the mean |z|^2 at the discriminator.
    /// Outputs exactly `d_in.len()` samples, delayed by `delay()`.
    pub fn process(
        &mut self,
        d_in: &[f32],
        env_in: &[f32],
        a2: f32,
        d_out: &mut Vec<f32>,
        dmg_out: &mut Vec<u8>,
    ) {
        let dip = self.cfg.dip_level * a2;
        self.d.extend_from_slice(d_in);
        self.env.extend_from_slice(env_in);
        let m0 = self.dmg.len();
        self.dmg.extend(env_in.iter().map(|&e| if e < dip { damage::NOISY } else { damage::NONE }));
        debug_assert_eq!(m0 + d_in.len(), self.dmg.len());
        let h = self.h as i64;
        let n_abs = self.b0 + self.d.len() as i64;
        if self.cfg.enabled {
            let from = self.scan_next.max(self.b0 + h + 3);
            let to = n_abs - (h + 3);
            // Wraps first, so a wrapped sample is never mistaken for the
            // middle of a slip by an earlier neighbour's check.
            // Both rules only act where a cheap vectorised test flags a
            // chunk; the test runs on the data as left by the chunks before.
            let mut c = from;
            while c < to {
                let e = (c + CHUNK).min(to);
                let (i0, i1) = ((c - self.b0) as usize, (e - self.b0) as usize);
                if any_jump(&self.d, i0, i1) {
                    for k in c..e {
                        self.unwrap_at(k, dip);
                    }
                }
                c = e;
            }
            let thr_min = self.cfg.spike_rad.min(self.cfg.dip_spike_rad);
            let mut c = from;
            while c < to {
                let e = (c + CHUNK).min(to);
                let (i0, i1) = ((c - self.b0) as usize, (e - self.b0) as usize);
                if !any_spike(&self.d, i0, i1, thr_min) {
                    c = e;
                    continue;
                }
                while c < e {
                    c = self.check(c, dip);
                }
            }
            self.scan_next = self.scan_next.max(to);
        }
        let final_end = n_abs - (2 * h + 3);
        let (a, b) = (
            (self.out_next - self.b0) as usize,
            (final_end - self.b0) as usize,
        );
        d_out.extend_from_slice(&self.d[a..b]);
        dmg_out.extend_from_slice(&self.dmg[a..b]);
        self.out_next = final_end;
        // Trim, keeping left context for the next scan.
        let keep_from = self.out_next.min(self.scan_next) - (h + 3);
        let cut = (keep_from - self.b0).max(0) as usize;
        if cut > 0 {
            self.d.drain(..cut);
            self.env.drain(..cut);
            self.dmg.drain(..cut);
            self.b0 += cut as i64;
        }
    }

    /// Wrap rule at absolute index `c`: a jump of more than pi from the
    /// previous sample that is undone by an opposite jump within a few
    /// samples marks a run that crossed +-pi; the run is moved by 2 pi.
    #[inline]
    fn unwrap_at(&mut self, c: i64, dip: f32) {
        let i = (c - self.b0) as usize;
        let d = &mut self.d;
        let jump = d[i] - d[i - 1];
        if jump.abs() <= PI {
            return;
        }
        let max_end = (i + self.h + 2).min(d.len() - 2);
        for j in i..=max_end {
            let back = d[j + 1] - d[j];
            if back.abs() > PI && back.signum() != jump.signum() {
                let shift = TAU.copysign(jump);
                let mut dipped = false;
                for k in i..=j {
                    d[k] -= shift;
                    self.dmg[k] = self.dmg[k].max(damage::REPAIRED);
                    dipped |= self.env[k] < dip;
                }
                if dipped {
                    self.events.push(c);
                } else {
                    self.wraps += 1;
                }
                return;
            }
        }
    }

    /// Examines candidate at absolute index `c`; returns the next index to
    /// examine.
    #[inline]
    fn check(&mut self, c: i64, dip: f32) -> i64 {
        let i = (c - self.b0) as usize;
        let d = &mut self.d;
        let e = d[i] - 0.5 * (d[i - 1] + d[i + 1]);
        let dipped = self.env[i] < dip;
        let thr = if dipped { self.cfg.dip_spike_rad } else { self.cfg.spike_rad };
        if e.abs() < thr {
            return c + 1;
        }
        let h = self.h;
        let (lo, hi) = (i - h, i + h);
        let lb = (d[lo - 3] + d[lo - 2] + d[lo - 1]) / 3.0;
        let rb = (d[hi + 1] + d[hi + 2] + d[hi + 3]) / 3.0;
        let (lp, rp) = ((lo - 2) as f32, (hi + 2) as f32);
        let slope = (rb - lb) / (rp - lp);
        let mut area = 0.0f32;
        let mut min_env = f32::INFINITY;
        for k in lo..=hi {
            area += d[k] - (lb + slope * (k as f32 - lp));
            min_env = min_env.min(self.env[k]);
        }
        if area.abs() >= self.cfg.area_frac * TAU {
            let (a, b) = (d[lo - 1], d[hi + 1]);
            let span = (hi - lo + 2) as f32;
            for k in lo..=hi {
                let t = (k - lo + 1) as f32 / span;
                d[k] = a + (b - a) * t;
                self.dmg[k] = self.dmg[k].max(damage::REPAIRED);
            }
            self.events.push(c);
            return c + h as i64 + 1;
        }
        if min_env < dip && e.abs() >= self.cfg.dip_impulse_rad {
            let (a, b) = (d[i - 2], d[i + 2]);
            for (j, k) in (i - 1..=i + 1).enumerate() {
                d[k] = a + (b - a) * (j + 1) as f32 / 4.0;
                self.dmg[k] = self.dmg[k].max(damage::REPAIRED);
            }
            self.events.push(c);
            return c + 2;
        }
        c + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(cr: &mut ClickRepair, d: &[f32], env: &[f32], block: usize) -> (Vec<f32>, Vec<u8>) {
        let (mut o, mut m) = (Vec::new(), Vec::new());
        for (dc, ec) in d.chunks(block).zip(env.chunks(block)) {
            cr.process(dc, ec, 1.0, &mut o, &mut m);
        }
        (o, m)
    }

    fn video(n: usize) -> Vec<f32> {
        (0..n)
            .map(|k| 0.9 * ((k as f32) * 0.07).sin() + 0.5 * ((k as f32) * 0.31).sin() - 0.2)
            .collect()
    }

    #[test]
    fn clean_signal_untouched_with_constant_delay() {
        let d = video(5000);
        let env = vec![1.0; 5000];
        let mut cr = ClickRepair::new(&ClickConfig::default(), false);
        let delay = cr.delay();
        let (o, m) = run(&mut cr, &d, &env, 333);
        assert_eq!(o.len(), d.len());
        for k in delay..5000 {
            assert_eq!(o[k], d[k - delay]);
            assert_eq!(m[k], damage::NONE);
        }
        assert!(cr.events.is_empty());
        // Hard steps (large video transitions) are not clicks either.
        let mut s = vec![-1.4f32; 3000];
        for (k, v) in s.iter_mut().enumerate() {
            if (k / 50) % 2 == 1 {
                *v = 1.8;
            }
        }
        let mut cr = ClickRepair::new(&ClickConfig::default(), false);
        let (o, _) = run(&mut cr, &s, &vec![1.0; 3000], 1000);
        assert!(cr.events.is_empty(), "false clicks {:?}", cr.events);
        assert_eq!(o[1000 + cr.delay()], s[1000]);
    }

    #[test]
    fn repairs_slips_wraps_and_impulses() {
        let n = 6000;
        let truth = video(n);
        let mut d = truth.clone();
        let mut env = vec![1.0f32; n];
        // Two-sample slip with dip.
        for (c, sign) in [(1000usize, 1.0f32), (2500, -1.0), (4000, 1.0)] {
            d[c] += sign * PI * 0.95;
            d[c + 1] += sign * PI * 1.05;
            env[c] = 0.01;
            env[c + 1] = 0.02;
        }
        // Wrap: a smooth excursion past +pi (over-deviation) wraps to -pi.
        let mut t2 = truth.clone();
        for k in 2990..3011 {
            let x = (k as f32 - 3000.0) / 10.0;
            t2[k] = truth[k] + (3.3 - truth[3000]) * 0.5 * (1.0 + (PI * x).cos());
            d[k] = crate::fastmath::wrap_pi(t2[k]);
        }
        // Impulse in a dip.
        d[5000] += 2.8;
        env[5000] = 0.05;
        let cfg = ClickConfig {
            dip_impulse_rad: 2.4,
            ..ClickConfig::default()
        };
        let mut cr = ClickRepair::new(&cfg, false);
        let delay = cr.delay();
        let (o, m) = run(&mut cr, &d, &env, 700);
        assert_eq!(cr.events.len(), 4, "events {:?}", cr.events);
        assert!(cr.wraps >= 1);
        for &c in &[1000usize, 2500, 4000] {
            let area: f32 = (c - 4..c + 6).map(|k| o[k + delay] - t2[k]).sum();
            assert!(m[c + delay - 1] <= damage::REPAIRED);
            assert!(area.abs() < 1.0, "residual area at {c}: {area}");
            assert_eq!(m[c + delay], damage::REPAIRED);
        }
        for k in 2990..3011 {
            assert!((o[k + delay] - t2[k]).abs() < 1e-5, "wrap at {k}");
        }
        assert!((o[5000 + delay] - truth[5000]).abs() < 0.3);
    }
}
