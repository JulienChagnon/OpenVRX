// SPDX-License-Identifier: GPL-3.0-or-later
//! Fine digital frequency shift (NCO) and the complex channel filter.

use crate::config::ChannelFilterConfig;
use crate::filters::{design_lowpass, kaiser_beta, kaiser_len, SymFir};

const NCO_TABLE: usize = 1024;

/// Phase-continuous frequency shifter: multiplies by exp(-j 2 pi f n / fs).
#[derive(Clone, Debug)]
pub struct Nco {
    fs: f64,
    shift_hz: f64,
    /// Phase at the next sample (rad, f64 for long-term accuracy).
    phase: f64,
    tab_c: Vec<f32>,
    tab_s: Vec<f32>,
}

impl Nco {
    pub fn new(fs: f64) -> Nco {
        let mut n = Nco {
            fs,
            shift_hz: f64::NAN,
            phase: 0.0,
            tab_c: vec![1.0; NCO_TABLE],
            tab_s: vec![0.0; NCO_TABLE],
        };
        n.set_shift(0.0);
        n
    }

    pub fn shift_hz(&self) -> f64 {
        self.shift_hz
    }

    /// Sets the shift; takes effect at the next sample, phase-continuous.
    pub fn set_shift(&mut self, hz: f64) {
        if hz == self.shift_hz {
            return;
        }
        self.shift_hz = hz;
        let w = -std::f64::consts::TAU * hz / self.fs;
        // Exact values every 32 entries, a rotation recurrence between
        // (the AFC can retune every block; 2048 libm calls were 40 us).
        let (sw, cw) = w.sin_cos();
        for b in (0..NCO_TABLE).step_by(32) {
            let (mut s, mut c) = (w * b as f64).sin_cos();
            for k in b..(b + 32).min(NCO_TABLE) {
                self.tab_c[k] = c as f32;
                self.tab_s[k] = s as f32;
                let cn = c * cw - s * sw;
                s = s * cw + c * sw;
                c = cn;
            }
        }
    }

    pub fn process(&mut self, re: &mut [f32], im: &mut [f32]) {
        if self.shift_hz == 0.0 {
            return;
        }
        let w = -std::f64::consts::TAU * self.shift_hz / self.fs;
        let n = re.len();
        let mut pos = 0;
        while pos < n {
            let len = (n - pos).min(NCO_TABLE);
            let (s0, c0) = self.phase.sin_cos();
            rotate(
                &mut re[pos..pos + len],
                &mut im[pos..pos + len],
                &self.tab_c[..len],
                &self.tab_s[..len],
                (c0 as f32, s0 as f32),
            );
            self.phase = crate::fastmath::wrap_pi64(self.phase + w * len as f64);
            pos += len;
        }
    }
}

dispatch! {
    /// Multiplies by exp(j (phase0 + table angle)), the start phase given
    /// as (cos, sin).
    fn rotate(r: &mut [f32], i: &mut [f32], tc: &[f32], ts: &[f32], p0: (f32, f32)) {
        let n = r.len().min(i.len()).min(tc.len()).min(ts.len());
        let (r, i, tc, ts) = (&mut r[..n], &mut i[..n], &tc[..n], &ts[..n]);
        let (c0, s0) = p0;
        for k in 0..n {
            let rc = c0 * tc[k] - s0 * ts[k];
            let rs = c0 * ts[k] + s0 * tc[k];
            let (x, y) = (r[k], i[k]);
            r[k] = x * rc - y * rs;
            i[k] = x * rs + y * rc;
        }
    }
}

/// Complex FIR with real (symmetric) coefficients on SoA I/Q. The tap
/// count is fixed when the filter is built, so adaptive cutoff changes
/// never change the group delay.
#[derive(Clone, Debug)]
pub struct ChannelFilter {
    cfg: ChannelFilterConfig,
    fs: f64,
    active: bool,
    taps: usize,
    cutoff_hz: f64,
    fi: SymFir,
    fq: SymFir,
    /// Tap sets designed so far, by cutoff (f64 bits).
    designs: Vec<(u64, Vec<f32>)>,
}

impl ChannelFilter {
    pub fn new(cfg: &ChannelFilterConfig, fs: f64) -> ChannelFilter {
        let nyq = 0.47 * fs;
        let lowest = if cfg.adaptive {
            (cfg.min_cutoff_hz as f64).min(cfg.cutoff_hz as f64)
        } else {
            cfg.cutoff_hz as f64
        };
        let active = cfg.enabled && lowest < nyq;
        let taps = if active {
            kaiser_len(cfg.atten_db as f64, cfg.transition_hz as f64 / fs, cfg.max_taps)
        } else {
            1
        };
        let mut f = ChannelFilter {
            cfg: cfg.clone(),
            fs,
            active,
            taps,
            cutoff_hz: f64::NAN,
            fi: SymFir::identity(),
            fq: SymFir::identity(),
            designs: Vec::new(),
        };
        f.set_cutoff(cfg.cutoff_hz as f64);
        f
    }

    pub fn delay(&self) -> usize {
        (self.taps - 1) / 2
    }

    pub fn active(&self) -> bool {
        self.active
    }

    /// Current one-sided cutoff (Hz), or fs/2 when inactive.
    pub fn cutoff_hz(&self) -> f64 {
        if self.active {
            self.cutoff_hz.min(0.5 * self.fs)
        } else {
            0.5 * self.fs
        }
    }

    /// Noise bandwidth relative to the full sample-rate band.
    pub fn noise_fraction(&self) -> f64 {
        (2.0 * self.cutoff_hz() / self.fs).min(1.0)
    }

    fn set_cutoff(&mut self, hz: f64) {
        if !self.active || hz == self.cutoff_hz {
            return;
        }
        self.cutoff_hz = hz;
        // Designs are cached: the adaptive cutoff moves between a few
        // quantised values, and a Kaiser design costs microseconds.
        let key = hz.to_bits();
        let h = match self.designs.iter().find(|d| d.0 == key) {
            Some(d) => d.1.clone(),
            None => {
                // The design cutoff is the -6 dB point; place it so the
                // passband edge sits at `hz - transition/2`.
                let h = design_lowpass(
                    self.taps,
                    (hz / self.fs).min(0.5),
                    kaiser_beta(self.cfg.atten_db as f64),
                );
                if self.designs.len() >= 64 {
                    self.designs.remove(0);
                }
                self.designs.push((key, h.clone()));
                h
            }
        };
        self.fi.set_taps(h.clone());
        self.fq.set_taps(h);
    }

    /// Adapts the cutoff to the smoothed C/N (dB).
    pub fn adapt(&mut self, cn_db: f32) {
        if !self.active || !self.cfg.adaptive || !cn_db.is_finite() {
            return;
        }
        let (lo, hi) = (self.cfg.cn_lo_db, self.cfg.cn_hi_db.max(self.cfg.cn_lo_db + 0.1));
        let t = ((cn_db - lo) / (hi - lo)).clamp(0.0, 1.0) as f64;
        let full = self.cfg.cutoff_hz as f64;
        let min = (self.cfg.min_cutoff_hz as f64).min(full);
        let target = min + t * (full - min);
        // Quantise so the filter is not redesigned every block.
        let q = 0.25e6;
        let target = (target / q).round() * q;
        if (target - self.cutoff_hz).abs() >= q * 0.5 {
            self.set_cutoff(target);
        }
    }

    pub fn process(&mut self, re: &mut [f32], im: &mut [f32]) {
        if !self.active {
            return;
        }
        self.fi.process_in_place(re);
        self.fq.process_in_place(im);
    }

    pub fn reset(&mut self) {
        self.fi.reset();
        self.fq.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nco_shifts_tone_phase_continuously() {
        let fs = 13.333e6;
        let mut n = Nco::new(fs);
        n.set_shift(1.234e6);
        let f0 = 2.0e6;
        let len = 5000;
        let mut re: Vec<f32> = (0..len)
            .map(|k| (std::f64::consts::TAU * f0 * k as f64 / fs).cos() as f32)
            .collect();
        let mut im: Vec<f32> = (0..len)
            .map(|k| (std::f64::consts::TAU * f0 * k as f64 / fs).sin() as f32)
            .collect();
        // Odd block sizes exercise the phase carry.
        let mut p = 0;
        for b in [17usize, 1000, 2500, 1483] {
            n.process(&mut re[p..p + b], &mut im[p..p + b]);
            p += b;
        }
        for k in 0..len {
            let a = std::f64::consts::TAU * (f0 - 1.234e6) * k as f64 / fs;
            assert!((re[k] as f64 - a.cos()).abs() < 1e-4, "k={k}");
            assert!((im[k] as f64 - a.sin()).abs() < 1e-4, "k={k}");
        }
    }

    #[test]
    fn channel_filter_adapts_without_delay_change() {
        let cfg = ChannelFilterConfig::default();
        let fs = 26.667e6;
        let mut f = ChannelFilter::new(&cfg, fs);
        assert!(f.active());
        let d = f.delay();
        f.adapt(20.0);
        assert_eq!(f.cutoff_hz(), 8.0e6);
        f.adapt(0.0);
        assert_eq!(f.cutoff_hz(), 4.5e6);
        f.adapt(6.0);
        assert!((f.cutoff_hz() - 6.25e6).abs() < 1.0);
        assert_eq!(f.delay(), d);
        // At 13.33 MS/s the 8 MHz cutoff is above Nyquist but adaptation
        // keeps it active.
        let f13 = ChannelFilter::new(&cfg, 13.333e6);
        assert!(f13.active());
        let off = ChannelFilter::new(
            &ChannelFilterConfig {
                adaptive: false,
                ..cfg
            },
            13.333e6,
        );
        assert!(!off.active());
        assert_eq!(off.delay(), 0);
    }
}
