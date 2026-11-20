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

    /// Linear C/N, capped by what the lane quantiser can deliver at this mean power
    /// ([`quant_cap`]; `step2` is the squared step of the carried bits, 0 for no cap).
    /// Only below [`QUANT_CAP_MAX_POWER`], where the envelope reading stops being valid.
    pub fn cn_linear_q(&self, step2: f64) -> f64 {
        let c = self.cn_linear();
        let m = self.mean();
        if step2 > 0.0 && self.n > 0 && m < QUANT_CAP_MAX_POWER * step2 {
            c.min(quant_cap(m, step2))
        } else {
            c
        }
    }

    /// [`Self::cn_linear_q`] in dB.
    pub fn cn_db_q(&self, step2: f64) -> f32 {
        lin_to_db(self.cn_linear_q(step2))
    }

    /// The carrier sits at the quantiser floor ([`at_floor`]).
    pub fn at_floor(&self, step2: f64) -> bool {
        step2 > 0.0 && self.n > 0 && at_floor(self.mean(), step2)
    }
}

/// Best linear C/N a mid-rise quantiser with step `sqrt(step2)` can deliver to the FM
/// demodulator at mean power `mean` (|z|^2 after DC removal, same units as `step2`):
/// `6 (mean / step^2 - 1/2)`.
///
/// The lane decode puts every code at its cell centre, so the smallest |z|^2 is step^2 / 2:
/// the four cells around zero, one per quadrant. Only the power above that floor
/// carries phase. Well above the floor the quantiser error across the carrier is step^2 / 12
/// per axis, worth C/N = A^2 / (step^2 / 6) as FM noise; near the floor the carrier collapses
/// onto the four inner points and its phase onto quadrants. A noise-free 4x4top carrier,
/// Monte Carlo against the phase error of the quantised samples: mean 2.6 / 3.6 / 8.5 / 21 / 39
/// step^2 gives 10.7 / 13.8 / 16.9 / 20.4 / 24.1 dB, this formula 11.1 / 12.7 / 16.8 / 20.9 /
/// 23.7, and the formula goes to zero at the floor, where any noise makes the picture worse
/// than the formula says.
///
/// Why a cap is needed: the envelope estimator assumes Gaussian noise on a continuous carrier.
/// Below about one step of radius the quantised envelope sits on one or a few radii and looks
/// constant, so the estimator reads a clean carrier whatever the truth (same Monte Carlo: radius
/// 0.3 step with noise 0.2 step reads 30 dB against -0.7 dB effective). On 4-bit lanes a deep
/// fade with the gain at its top row is exactly that case, and without the cap the damage mask,
/// the line quality and the diversity weights take the lost carrier for a good one. With more
/// carried bits the receiver noise spans several steps and the cap never binds.
///
/// The cap is applied only below [`QUANT_CAP_MAX_POWER`] step^2: above it the quantiser error
/// behaves as additive noise that the envelope estimator already sees, and readings of strong
/// carriers stay exactly what they were.
#[inline]
pub fn quant_cap(mean: f64, step2: f64) -> f64 {
    (6.0 * (mean / step2 - 0.5)).max(0.0)
}

/// Mean power (in step^2) below which [`EnvAcc::cn_linear_q`] applies [`quant_cap`]: a carrier
/// radius of about 2.8 steps. In the Monte Carlo the envelope reading is within about 2 dB of the
/// effective C/N from radius 1.5 steps up, so the cap only has to cover the region below.
pub const QUANT_CAP_MAX_POWER: f64 = 8.0;

/// The carrier is at the quantiser floor: even a noise-free carrier at this mean power would
/// give the demodulator under 0 dB ([`quant_cap`] below 1, mean power within step^2 / 6 of
/// the floor). The picture there is the discriminator's output for quadrant-quantised phase:
/// flat mid grey with black and white streaks.
#[inline]
pub fn at_floor(mean: f64, step2: f64) -> bool {
    quant_cap(mean, step2) < 1.0
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

    /// 4x4top-style mid-rise quantiser (step 1, 16 codes, cell centres).
    fn q4(x: f64) -> f64 {
        x.floor().clamp(-8.0, 7.0) + 0.5
    }

    /// Envelope statistics of a quantised FM carrier of radius `r` steps with complex
    /// Gaussian noise of `sigma` steps per axis.
    fn quantised_carrier(r: f64, sigma: f64, seed: u64) -> EnvAcc {
        let mut s = seed;
        let mut u = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        };
        let mut acc = EnvAcc::default();
        let mut ph = 0.0f64;
        for k in 0..200_000 {
            ph += 0.25 + 0.2 * (k as f64 * 0.001).sin();
            let a = (-2.0 * u().ln()).sqrt() * sigma;
            let t = std::f64::consts::TAU * u();
            let (zr, zi) = (q4(r * ph.cos() + a * t.cos()), q4(r * ph.sin() + a * t.sin()));
            acc.add((zr * zr + zi * zi) as f32);
        }
        acc
    }

    #[test]
    fn quantiser_floor_is_not_a_clean_carrier() {
        // Radius under one step: the four inner cells, a constant quantised envelope. The
        // plain estimator reads a strong carrier; the capped one and the floor test do not.
        let acc = quantised_carrier(0.3, 0.2, 7);
        assert!(acc.cn_db() > 15.0, "plain reading {}", acc.cn_db());
        assert!(acc.cn_db_q(1.0) < 0.0, "capped {}", acc.cn_db_q(1.0));
        assert!(acc.at_floor(1.0));
        // Same carrier read without a step (angle lanes, unknown format): unchanged.
        assert_eq!(acc.cn_db_q(0.0), acc.cn_db());
    }

    #[test]
    fn cap_leaves_a_carrier_inside_the_lanes_alone() {
        // At the 4-bit gain target (4.4 steps rms per axis, radius 6.2), noisy or not, the
        // plain reading is valid and the cap must not move it.
        for sigma in [0.05, 0.41] {
            let acc = quantised_carrier(6.2, sigma, 3);
            assert!(!acc.at_floor(1.0));
            assert_eq!(acc.cn_db_q(1.0), acc.cn_db(), "sigma {sigma}");
        }
        // Bench breakup steps sit near radius 1 step (mean about 1.5 step^2): not the floor.
        let acc = quantised_carrier(1.0, 0.41, 5);
        assert!(!acc.at_floor(1.0), "mean {}", acc.mean());
        // The formula itself.
        assert_eq!(quant_cap(0.5, 1.0), 0.0);
        assert!((quant_cap(39.0, 1.0) - 231.0).abs() < 1e-9);
        assert!(at_floor(0.6, 1.0) && !at_floor(0.7, 1.0));
    }
}
