// SPDX-License-Identifier: GPL-3.0-or-later
//! Burst measurement and the burst-locked subcarrier reference.
//!
//! Each line's burst is correlated with a fixed line-locked oscillator
//! `exp(-j 2 pi f x)`; the result `z` has the burst's amplitude and phase
//! relative to that oscillator. A two-state Kalman filter (phase, per-line
//! phase advance error) runs line to line: the per-line advance is exactly
//! half a cycle for NTSC on the 4 fsc grid and 283.7516 cycles for PAL on
//! the 1135-sample grid, so the prediction carries the PAL grid slip. The
//! measurement noise follows the burst SNR, so clean bursts correct each
//! line's time-base residual fully while noisy bursts are smoothed.
//!
//! PAL: the swinging burst (+-45 degrees around -U) identifies the
//! V-switch. A leaky vote over many lines decides which lines carry +V, so
//! single noisy bursts cannot flip it.
//!
//! The tracker also runs automatic colour control (average burst
//! amplitude) and a graceful colour killer: saturation fades with burst
//! presence and burst phase noise, and luma/chroma separation is bypassed
//! entirely when no burst is present (monochrome cameras).

use crate::util::{smoothstep, wrap_pi};
use openvrx_core::standard::StandardParams;
use openvrx_core::VideoStandard;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

/// Burst correlation for one line.
#[derive(Copy, Clone, Debug, Default)]
pub struct BurstMeasure {
    /// Complex correlation (IRE), amplitude = burst half peak-to-peak.
    pub re: f64,
    pub im: f64,
    /// True when the burst window is undamaged.
    pub clean: bool,
}

impl BurstMeasure {
    #[inline]
    pub fn amp(&self) -> f64 {
        self.re.hypot(self.im)
    }
    #[inline]
    pub fn arg(&self) -> f64 {
        self.im.atan2(self.re)
    }
}

/// Measures the burst of one line. `osc_c`/`osc_s` are cos/sin of the
/// line-locked oscillator over the whole line.
pub fn measure_burst(
    samples: &[f32],
    damage: &[u8],
    range: std::ops::Range<usize>,
    osc_c: &[f32],
    osc_s: &[f32],
) -> BurstMeasure {
    let n = range.len() as f64;
    let mean = samples[range.clone()].iter().map(|&v| v as f64).sum::<f64>() / n;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    let mut clean = true;
    for x in range {
        let s = samples[x] as f64 - mean;
        re += s * osc_c[x] as f64;
        im -= s * osc_s[x] as f64;
        if damage[x] >= openvrx_core::damage::CLICK {
            clean = false;
        }
    }
    BurstMeasure {
        re: re * 2.0 / n,
        im: im * 2.0 / n,
        clean,
    }
}

/// Colour state of one line, produced by the tracker.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct LineColor {
    /// Subcarrier phase of the line relative to the oscillator (radians).
    pub phase: f32,
    /// PAL V-switch for the line (+1 or -1; NTSC always +1).
    pub vswitch: f32,
    /// Chroma gain: ACC times the colour-killer level.
    pub gain: f32,
    /// Luma/chroma separation level (0 = monochrome, full-band luma).
    pub sep: f32,
}

/// Tracker tuning.
#[derive(Copy, Clone, Debug)]
pub struct TrackerConfig {
    pub acc: bool,
    pub killer: bool,
    /// Expected line-to-line phase jitter left by the time-base corrector.
    pub line_jitter_deg: f64,
}

/// Burst PLL, ACC and colour killer state.
#[derive(Clone, Debug)]
pub struct BurstTracker {
    standard: VideoStandard,
    nominal_amp: f64,
    /// Nominal per-line phase advance (radians, wrapped).
    advance: f64,
    burst_len: usize,
    cfg: TrackerConfig,
    // Kalman state.
    phase: f64,
    freq: f64,
    p00: f64,
    p01: f64,
    p11: f64,
    locked: bool,
    big_errors: u32,
    // V-switch identification.
    ident: f64,
    prev_theta: Option<f64>,
    line_abs: u64,
    // ACC and killer.
    amp_avg: f64,
    amp_init: bool,
    err_var: f64,
    acc_gain: f64,
    color_level: f64,
    sep_level: f64,
}

impl BurstTracker {
    pub fn new(p: &StandardParams, burst_len: usize, cfg: TrackerConfig) -> BurstTracker {
        let cyc = p.fsc_per_sample() * p.samples_per_line as f64;
        BurstTracker {
            standard: p.standard,
            nominal_amp: p.burst_ire as f64,
            advance: wrap_pi(2.0 * PI * cyc.fract()),
            burst_len,
            cfg,
            phase: 0.0,
            freq: 0.0,
            p00: 1e3,
            p01: 0.0,
            p11: 1e-3,
            locked: false,
            big_errors: 0,
            ident: 0.0,
            prev_theta: None,
            line_abs: 0,
            amp_avg: 0.0,
            amp_init: false,
            err_var: (20f64.to_radians()).powi(2),
            acc_gain: 1.0,
            color_level: 0.0,
            sep_level: 0.0,
        }
    }

    /// Forgets the phase lock (keeps ACC and killer levels). Used after a
    /// discontinuity in the line sequence.
    pub fn unlock(&mut self) {
        self.locked = false;
        self.p00 = 1e3;
        self.p01 = 0.0;
        self.p11 = 1e-3;
        self.prev_theta = None;
        self.big_errors = 0;
    }

    /// Full reset.
    pub fn reset(&mut self) {
        let p = self.standard.params();
        *self = BurstTracker::new(p, self.burst_len, self.cfg);
    }

    pub fn burst_amplitude(&self) -> f64 {
        self.amp_avg
    }

    pub fn color_level(&self) -> f64 {
        self.color_level
    }

    pub fn phase_noise_deg(&self) -> f64 {
        self.err_var.sqrt().to_degrees()
    }

    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Processes one line. `burst` is None on lines without a burst
    /// (vertical interval); `noise_ire` is the line's RMS noise estimate.
    pub fn step(&mut self, burst: Option<BurstMeasure>, noise_ire: f32) -> LineColor {
        // Predict.
        if self.locked {
            self.phase = wrap_pi(self.phase + self.advance + self.freq);
            let q_phase = self.cfg.line_jitter_deg.to_radians().powi(2);
            let q_freq = 1e-6;
            let p00 = self.p00 + 2.0 * self.p01 + self.p11 + q_phase;
            let p01 = self.p01 + self.p11;
            let p11 = self.p11 + q_freq;
            self.p00 = p00.min(10.0);
            self.p01 = p01;
            self.p11 = p11.min(1e-2);
        }
        let alt = if self.line_abs & 1 == 0 { 1.0 } else { -1.0 };
        let mut vsw = 1.0;

        if let Some(b) = burst {
            let amp = b.amp();
            if b.clean {
                if !self.amp_init {
                    self.amp_avg = amp;
                    self.amp_init = true;
                } else {
                    self.amp_avg += (amp - self.amp_avg) / 16.0;
                }
            }
            let presence = self.amp_avg / self.nominal_amp;
            let usable = b.clean && amp > 0.2 * self.nominal_amp;
            let theta = b.arg();
            if self.standard == VideoStandard::Pal {
                let mut pair_vote = None;
                if let (true, Some(prev)) = (usable, self.prev_theta) {
                    let d = wrap_pi(theta - prev - self.advance);
                    // d = +90 deg: this line carries -V; -90: +V.
                    let vote = -d.sin();
                    self.ident = 0.97 * self.ident + vote * alt;
                    pair_vote = Some(vote);
                }
                vsw = match pair_vote {
                    // Until the vote has built up, trust the line pair.
                    Some(v) if self.ident.abs() < 1.5 => {
                        if v >= 0.0 {
                            1.0
                        } else {
                            -1.0
                        }
                    }
                    _ => {
                        if self.ident >= 0.0 {
                            alt
                        } else {
                            -alt
                        }
                    }
                };
            }
            self.prev_theta = if usable { Some(theta) } else { None };
            if usable {
                let meas = match self.standard {
                    VideoStandard::Ntsc => theta - FRAC_PI_2,
                    VideoStandard::Pal => theta - FRAC_PI_2 + vsw * FRAC_PI_4,
                };
                let sigma = if noise_ire.is_finite() && noise_ire > 0.0 {
                    noise_ire as f64
                } else {
                    0.5
                };
                let r = (sigma / (amp * (self.burst_len as f64 / 2.0).sqrt()))
                    .powi(2)
                    .max(0.25f64.to_radians().powi(2));
                if !self.locked {
                    self.phase = wrap_pi(meas);
                    self.freq = 0.0;
                    self.p00 = r;
                    self.p01 = 0.0;
                    self.p11 = 1e-4;
                    self.locked = true;
                } else {
                    let innov = wrap_pi(meas - self.phase);
                    if innov.abs() > 70f64.to_radians() {
                        self.big_errors += 1;
                    } else {
                        self.big_errors = 0;
                    }
                    if self.big_errors >= 4 {
                        self.phase = wrap_pi(meas);
                        self.freq = 0.0;
                        self.p00 = r;
                        self.p01 = 0.0;
                        self.big_errors = 0;
                    } else {
                        let s = self.p00 + r;
                        let k0 = self.p00 / s;
                        let k1 = self.p01 / s;
                        self.phase = wrap_pi(self.phase + k0 * innov);
                        self.freq = (self.freq + k1 * innov).clamp(-0.2, 0.2);
                        let p00 = (1.0 - k0) * self.p00;
                        let p01 = (1.0 - k0) * self.p01;
                        let p11 = self.p11 - k1 * self.p01;
                        self.p00 = p00;
                        self.p01 = p01;
                        self.p11 = p11.max(1e-8);
                    }
                    let e = innov.min(PI / 2.0);
                    self.err_var += (e * e - self.err_var) / 32.0;
                }
            } else {
                // A missing or damaged burst counts as a large phase error
                // for the killer only when bursts are generally absent.
                if presence < 0.2 {
                    self.err_var += ((PI / 2.0).powi(2) - self.err_var) / 32.0;
                }
            }
            if self.cfg.acc && presence > 0.25 {
                self.acc_gain = (self.nominal_amp / self.amp_avg.max(1e-6)).clamp(0.5, 3.0);
            }
            if self.cfg.killer {
                let rms = self.err_var.sqrt().to_degrees();
                let t_color = smoothstep(0.12, 0.25, presence as f32) as f64
                    * (1.0 - smoothstep(15.0, 35.0, rms as f32) as f64);
                let t_sep = smoothstep(0.08, 0.16, presence as f32) as f64;
                self.color_level += (t_color - self.color_level) / 32.0;
                self.sep_level += (t_sep - self.sep_level) / 32.0;
            } else {
                self.color_level = 1.0;
                self.sep_level = 1.0;
            }
        } else if self.standard == VideoStandard::Pal {
            vsw = if self.ident >= 0.0 { alt } else { -alt };
            self.prev_theta = None;
        }
        if !self.cfg.killer {
            self.color_level = 1.0;
            self.sep_level = 1.0;
        }
        self.line_abs += 1;
        LineColor {
            phase: self.phase as f32,
            vswitch: vsw as f32,
            gain: (self.acc_gain * self.color_level) as f32,
            sep: self.sep_level as f32,
        }
    }

    /// Advances over `n` lines that were not received.
    pub fn skip_lines(&mut self, n: usize) {
        for _ in 0..n {
            self.step(None, f32::NAN);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::standard::{NTSC, PAL};

    fn synth(p: &StandardParams, phase: f64, vsw: f64, amp: f64) -> BurstMeasure {
        // Correlation of an ideal burst: (V - jU) e^{j phase} with U = -A
        // (NTSC) or U = -A/sqrt2, V = vsw A/sqrt2 (PAL).
        let (u, v) = match p.standard {
            VideoStandard::Ntsc => (-amp, 0.0),
            VideoStandard::Pal => (-amp / 2f64.sqrt(), vsw * amp / 2f64.sqrt()),
        };
        let (c, s) = (phase.cos(), phase.sin());
        // (v - j u) * (c + j s)
        BurstMeasure {
            re: v * c + u * s,
            im: v * s - u * c,
            clean: true,
        }
    }

    #[test]
    fn tracks_phase_and_vswitch() {
        for p in [&NTSC, &PAL] {
            let cfg = TrackerConfig {
                acc: true,
                killer: true,
                line_jitter_deg: 3.0,
            };
            let mut t = BurstTracker::new(p, 32, cfg);
            let cyc = p.fsc_per_sample() * p.samples_per_line as f64;
            let mut last = LineColor::default();
            for n in 0..400u64 {
                let ph = 2.0 * PI * (cyc * n as f64).fract() + 0.3;
                let vsw = if n % 2 == 0 { 1.0 } else { -1.0 };
                let b = synth(p, ph, vsw, p.burst_ire as f64 * 0.8);
                last = t.step(Some(b), 0.5);
                if n > 100 {
                    let err = wrap_pi(last.phase as f64 - ph).abs();
                    assert!(err < 0.01, "{:?} line {n} err {err}", p.standard);
                    if p.standard == VideoStandard::Pal {
                        assert_eq!(last.vswitch as f64, vsw, "line {n}");
                    }
                }
            }
            // ACC compensates the 0.8 burst level.
            assert!((last.gain - 1.25).abs() < 0.02, "gain {}", last.gain);
        }
    }

    #[test]
    fn killer_without_burst() {
        let cfg = TrackerConfig {
            acc: true,
            killer: true,
            line_jitter_deg: 3.0,
        };
        let mut t = BurstTracker::new(&PAL, 32, cfg);
        let mut lc = LineColor::default();
        for n in 0..600u64 {
            let ph = n as f64 * 1.7;
            let b = BurstMeasure {
                re: 0.5 * ph.cos(),
                im: 0.5 * ph.sin(),
                clean: true,
            };
            lc = t.step(Some(b), 2.0);
        }
        assert!(lc.gain < 0.01 && lc.sep < 0.01, "{lc:?}");
    }
}
