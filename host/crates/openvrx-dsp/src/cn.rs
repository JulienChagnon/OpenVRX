// SPDX-License-Identifier: GPL-3.0-or-later
//! Envelope carrier-to-noise estimate.
//!
//! Adapted from C5VRX (GPL-3.0, by Twotoz and the C5VRX contributors,
//! https://github.com/Twotoz/C5VRX, commit 6bc9e4b), `main/cn_estimate.h`
//! (`cn_estimate_env_x10`) and `main/dsp.h` (`dsp_envelope_ratio_x100`):
//! "with the window DC removed, |z|^2 of an FM carrier plus noise has
//! mean^2/var (c+1)^2/(2c+1); noise alone 1".
//!
//! Changes from C5VRX: C5VRX maps the ratio to dB through tables measured
//! on its 4-bit chain per analog filter class. OpenVRX carries 5..8-bit I/Q
//! with the DC removed upstream, so the model is inverted in closed form,
//! c = (r - 1) + sqrt(r (r - 1)). Accumulation is in f64 over arbitrary
//! windows (a video line, a stats window). Like the C5VRX estimate, the
//! ratio flattens at high C/N (channel-filter AM on the FM carrier and
//! quantisation), so readings above about 25 dB are lower bounds.

/// Running sums of p = |z|^2.
#[derive(Copy, Clone, Debug, Default)]
pub struct EnvAcc {
    pub n: u64,
    pub s1: f64,
    pub s2: f64,
}

impl EnvAcc {
    #[inline]
    pub fn add(&mut self, p: f32) {
        self.n += 1;
        self.s1 += p as f64;
        self.s2 += p as f64 * p as f64;
    }

    /// Adds a slice of |z|^2 values.
    pub fn add_slice(&mut self, p: &[f32]) {
        let (s1, s2) = crate::vmath::sums12(p);
        self.n += p.len() as u64;
        self.s1 += s1;
        self.s2 += s2;
    }

    pub fn merge(&mut self, o: &EnvAcc) {
        self.n += o.n;
        self.s1 += o.s1;
        self.s2 += o.s2;
    }

    pub fn mean(&self) -> f64 {
        if self.n == 0 {
            0.0
        } else {
            self.s1 / self.n as f64
        }
    }

    /// mean^2 / var of |z|^2.
    pub fn ratio(&self) -> f64 {
        if self.n < 2 {
            return f64::NAN;
        }
        let m = self.s1 / self.n as f64;
        let v = (self.s2 / self.n as f64 - m * m).max(0.0);
        if v <= m * m * 1e-9 {
            1e9
        } else {
            m * m / v
        }
    }

    /// Linear C/N (see [`cn_from_ratio`]).
    pub fn cn_linear(&self) -> f64 {
        cn_from_ratio(self.ratio())
    }

    /// C/N in dB, clamped to [-10, 60].
    pub fn cn_db(&self) -> f32 {
        lin_to_db(self.cn_linear())
    }
}


/// Inverts r = (c+1)^2/(2c+1). Returns 0 for r <= 1 (noise-like).
#[inline]
pub fn cn_from_ratio(r: f64) -> f64 {
    if r.is_nan() || r <= 1.0 {
        return 0.0;
    }
    let d = r - 1.0;
    d + (r * d).sqrt()
}

/// The forward model, r(c).
#[inline]
pub fn ratio_from_cn(c: f64) -> f64 {
    (c + 1.0) * (c + 1.0) / (2.0 * c + 1.0)
}

/// dB with the clamp used for reporting.
#[inline]
pub fn lin_to_db(c: f64) -> f32 {
    if c <= 1e-1 {
        -10.0
    } else {
        (10.0 * c.log10()).clamp(-10.0, 60.0) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inversion_roundtrip() {
        for db in [-5.0f64, 0.0, 3.0, 10.0, 25.0, 40.0] {
            let c = 10f64.powf(db / 10.0);
            let r = ratio_from_cn(c);
            assert!((cn_from_ratio(r) - c).abs() / c < 1e-9, "{db}");
        }
        assert_eq!(cn_from_ratio(1.0), 0.0);
        assert_eq!(cn_from_ratio(0.5), 0.0);
    }

    #[test]
    fn measures_carrier_in_gaussian_noise() {
        // Deterministic LCG + Box-Muller.
        let mut s = 12345u64;
        let mut u = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        };
        for db in [0.0f64, 6.0, 12.0] {
            let c = 10f64.powf(db / 10.0);
            let sigma = (1.0 / c / 2.0).sqrt();
            let mut acc = EnvAcc::default();
            let mut ph = 0.0f64;
            for k in 0..400_000 {
                ph += 0.3 + 0.2 * (k as f64 * 0.001).sin();
                let r = (-2.0 * u().ln()).sqrt();
                let t = std::f64::consts::TAU * u();
                let (nr, ni) = (sigma * r * t.cos(), sigma * r * t.sin());
                let (zr, zi) = (ph.cos() + nr, ph.sin() + ni);
                acc.add((zr * zr + zi * zi) as f32);
            }
            let got = acc.cn_db() as f64;
            assert!((got - db).abs() < 0.3, "{db} dB read {got}");
        }
    }
}
