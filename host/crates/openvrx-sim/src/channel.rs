// SPDX-License-Identifier: GPL-3.0-or-later
//! Radio channel models: Rayleigh/Rician fading as a sum of sinusoids
//! (Zheng and Xiao's improved Clarke model, a pure function of time so
//! blocks can be computed in parallel), and slow antenna gain patterns.

use crate::config::{AntennaPattern, Fading, FadingKind};
use crate::dsp::{mix_seed, splitmix, unit_from};
use std::f64::consts::{PI, TAU};

/// Number of sinusoids per quadrature component.
const M: usize = 16;

/// A complex fading gain process with E|g|^2 = 1.
#[derive(Clone, Debug)]
pub struct FadingProcess {
    kind: FadingKind,
    wd: f64,
    /// Doppler factors (cos alpha_n, sin alpha_n) and phases for I and Q.
    terms: Vec<(f64, f64, f64, f64)>,
    los: f64,
    diffuse: f64,
    los_doppler: f64,
    los_phase: f64,
}

impl FadingProcess {
    pub fn new(cfg: &Fading, seed: u64) -> FadingProcess {
        let mut h = mix_seed(&[seed, 0xFAD]);
        let mut next = || {
            h = splitmix(h);
            unit_from(h)
        };
        let theta = TAU * next() - PI;
        let terms = (1..=M)
            .map(|n| {
                let a = (TAU * n as f64 - PI + theta) / (4.0 * M as f64);
                (a.cos(), a.sin(), TAU * next(), TAU * next())
            })
            .collect();
        let (los, diffuse) = match cfg.kind {
            FadingKind::None => (1.0, 0.0),
            FadingKind::Rayleigh => (0.0, 1.0),
            FadingKind::Rician => {
                let k = 10f64.powf(cfg.k_db / 10.0);
                ((k / (k + 1.0)).sqrt(), (1.0 / (k + 1.0)).sqrt())
            }
        };
        FadingProcess {
            kind: cfg.kind,
            wd: TAU * cfg.doppler_hz,
            terms,
            los,
            diffuse,
            los_doppler: (TAU * next()).cos() * TAU * cfg.doppler_hz,
            los_phase: TAU * next(),
        }
    }

    pub fn is_static(&self) -> bool {
        self.kind == FadingKind::None
    }

    /// Complex gain at time `t` seconds.
    pub fn gain(&self, t: f64) -> (f64, f64) {
        if self.kind == FadingKind::None {
            return (1.0, 0.0);
        }
        let mut xc = 0.0;
        let mut xs = 0.0;
        for &(ca, sa, pc, ps) in &self.terms {
            xc += (self.wd * t * ca + pc).cos();
            xs += (self.wd * t * sa + ps).cos();
        }
        // Each component has power M/2 before scaling: total 1 after.
        let s = (1.0 / M as f64).sqrt() * self.diffuse;
        let (mut re, mut im) = (xc * s, xs * s);
        if self.los > 0.0 {
            let ph = self.los_doppler * t + self.los_phase;
            re += self.los * ph.cos();
            im += self.los * ph.sin();
        }
        (re, im)
    }
}

/// Antenna gain in dB over time.
#[derive(Clone, Debug)]
pub struct Antenna {
    offset_db: f64,
    depth_db: f64,
    period_s: f64,
    phase: f64,
}

impl Antenna {
    pub fn new(cfg: &AntennaPattern, seed: u64) -> Antenna {
        let phase = match cfg.phase_deg {
            Some(d) => d.to_radians(),
            None => TAU * unit_from(mix_seed(&[seed, 0xA27])),
        };
        Antenna {
            offset_db: cfg.offset_db,
            depth_db: cfg.depth_db,
            period_s: cfg.period_s.max(1e-6),
            phase,
        }
    }

    pub fn gain_db(&self, t: f64) -> f64 {
        self.offset_db + 0.5 * self.depth_db * (TAU * t / self.period_s + self.phase).sin()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn envelope_samples(cfg: &Fading, seed: u64, n: usize, dt: f64) -> Vec<f64> {
        let f = FadingProcess::new(cfg, seed);
        (0..n)
            .map(|k| {
                let (r, i) = f.gain(k as f64 * dt);
                (r * r + i * i).sqrt()
            })
            .collect()
    }

    #[test]
    fn rayleigh_envelope_distribution() {
        let cfg = Fading {
            kind: FadingKind::Rayleigh,
            doppler_hz: 50.0,
            k_db: 0.0,
        };
        // Pool several seeds over many Doppler periods.
        let mut env = Vec::new();
        for seed in 0..8 {
            env.extend(envelope_samples(&cfg, seed, 40_000, 0.0013));
        }
        let n = env.len() as f64;
        let power: f64 = env.iter().map(|a| a * a).sum::<f64>() / n;
        assert!((power - 1.0).abs() < 0.05, "mean power {power}");
        // Rayleigh CDF with E[r^2] = 1: P(r < x) = 1 - exp(-x^2).
        for x in [0.1f64, 0.3, 0.5, 0.8, 1.0, 1.4, 2.0] {
            let emp = env.iter().filter(|&&a| a < x).count() as f64 / n;
            let th = 1.0 - (-x * x).exp();
            assert!((emp - th).abs() < 0.02, "x={x}: {emp} vs {th}");
        }
        // Deep fades exist: about 1 % below -20 dB.
        let deep = env.iter().filter(|&&a| a < 0.1).count() as f64 / n;
        assert!(deep > 0.004 && deep < 0.02, "{deep}");
    }

    #[test]
    fn doppler_sets_coherence_time() {
        // Autocorrelation of the in-phase part follows J0(2 pi fd tau):
        // first zero at tau = 0.383 / fd.
        let cfg = Fading {
            kind: FadingKind::Rayleigh,
            doppler_hz: 100.0,
            k_db: 0.0,
        };
        let mut acc0 = 0.0;
        let mut acc_z = 0.0;
        let tau = 0.383 / 100.0;
        for seed in 0..20 {
            let f = FadingProcess::new(&cfg, seed);
            for k in 0..4000 {
                let t = k as f64 * 0.0007;
                let a = f.gain(t);
                let b = f.gain(t + tau);
                acc0 += a.0 * a.0 + a.1 * a.1;
                acc_z += a.0 * b.0 + a.1 * b.1;
            }
        }
        assert!((acc_z / acc0).abs() < 0.12, "{}", acc_z / acc0);
    }

    #[test]
    fn rician_and_static() {
        let s = FadingProcess::new(&Fading::default(), 1);
        assert_eq!(s.gain(3.0), (1.0, 0.0));
        let cfg = Fading {
            kind: FadingKind::Rician,
            doppler_hz: 40.0,
            k_db: 10.0,
        };
        let env = envelope_samples(&cfg, 4, 50_000, 0.001);
        let power: f64 = env.iter().map(|a| a * a).sum::<f64>() / env.len() as f64;
        assert!((power - 1.0).abs() < 0.08, "{power}");
        // Strong line of sight: fades below -10 dB are rare.
        let deep = env.iter().filter(|&&a| a < 0.316).count() as f64 / env.len() as f64;
        assert!(deep < 0.01, "{deep}");
    }

    #[test]
    fn antenna_pattern() {
        let a = Antenna::new(
            &AntennaPattern {
                offset_db: -3.0,
                depth_db: 6.0,
                period_s: 2.0,
                phase_deg: Some(0.0),
            },
            0,
        );
        assert!((a.gain_db(0.0) + 3.0).abs() < 1e-12);
        assert!((a.gain_db(0.5) - 0.0).abs() < 1e-12);
        assert!((a.gain_db(1.5) + 6.0).abs() < 1e-12);
    }
}
