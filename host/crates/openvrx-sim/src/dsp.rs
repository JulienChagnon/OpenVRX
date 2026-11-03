// SPDX-License-Identifier: GPL-3.0-or-later
//! Small DSP building blocks for the simulator: Butterworth design by the
//! bilinear transform, biquad cascades, windowed-sinc kernels, fast
//! sine/cosine in turns and seed mixing.

use std::f64::consts::PI;

/// One second-order section, normalised (a0 = 1). A first-order section
/// has `b2 = a2 = 0`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Section {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
}

impl Section {
    /// Frequency response at normalised angular frequency `w` (rad/sample).
    pub fn response(&self, w: f64) -> (f64, f64) {
        // H = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
        let (c1, s1) = (w.cos(), -w.sin());
        let (c2, s2) = ((2.0 * w).cos(), -(2.0 * w).sin());
        let nr = self.b0 + self.b1 * c1 + self.b2 * c2;
        let ni = self.b1 * s1 + self.b2 * s2;
        let dr = 1.0 + self.a1 * c1 + self.a2 * c2;
        let di = self.a1 * s1 + self.a2 * s2;
        let d = dr * dr + di * di;
        ((nr * dr + ni * di) / d, (ni * dr - nr * di) / d)
    }
}

/// A cascade of sections (a digital IIR filter design).
#[derive(Clone, Debug, PartialEq)]
pub struct Cascade {
    pub sections: Vec<Section>,
}

impl Cascade {
    /// Butterworth low-pass of order `n` with -3 dB at `fc` (fraction of
    /// the sample rate, 0 < fc < 0.5), by the bilinear transform with
    /// prewarping.
    pub fn butterworth_lowpass(n: usize, fc: f64) -> Cascade {
        assert!(n >= 1 && fc > 0.0 && fc < 0.5);
        let k = 2.0;
        let wc = 2.0 * (PI * fc).tan();
        let mut sections = Vec::new();
        for i in 0..n / 2 {
            // Analog poles of the pair: wc * exp(j pi (2i + n + 1) / 2n).
            let theta = PI * (2 * i + n + 1) as f64 / (2 * n) as f64;
            let sigma = wc * theta.cos();
            let d0 = k * k - 2.0 * sigma * k + wc * wc;
            let d1 = -2.0 * k * k + 2.0 * wc * wc;
            let d2 = k * k + 2.0 * sigma * k + wc * wc;
            let g = wc * wc / d0;
            sections.push(Section {
                b0: g,
                b1: 2.0 * g,
                b2: g,
                a1: d1 / d0,
                a2: d2 / d0,
            });
        }
        if n % 2 == 1 {
            let d0 = k + wc;
            sections.push(Section {
                b0: wc / d0,
                b1: wc / d0,
                b2: 0.0,
                a1: (wc - k) / d0,
                a2: 0.0,
            });
        }
        Cascade { sections }
    }

    /// First-order shelf `g (1 + s/wz) / (1 + s/wp)` (frequencies as a
    /// fraction of the sample rate), bilinear with prewarping at the zero
    /// and pole.
    pub fn shelf(fz: f64, fp: f64, g: f64) -> Cascade {
        let k = 2.0;
        let wz = 2.0 * (PI * fz).tan();
        let wp = 2.0 * (PI * fp).tan();
        // (1 + s/wz)/(1 + s/wp), s = k (1 - z^-1)/(1 + z^-1)
        let n0 = 1.0 + k / wz;
        let n1 = 1.0 - k / wz;
        let d0 = 1.0 + k / wp;
        let d1 = 1.0 - k / wp;
        Cascade {
            sections: vec![Section {
                b0: g * n0 / d0,
                b1: g * n1 / d0,
                b2: 0.0,
                a1: d1 / d0,
                a2: 0.0,
            }],
        }
    }

    pub fn response(&self, w: f64) -> (f64, f64) {
        self.sections.iter().fold((1.0, 0.0), |(r, i), s| {
            let (a, b) = s.response(w);
            (r * a - i * b, r * b + i * a)
        })
    }

    /// |H| at `f` (fraction of the sample rate).
    pub fn magnitude(&self, f: f64) -> f64 {
        let (r, i) = self.response(2.0 * PI * f);
        (r * r + i * i).sqrt()
    }

    /// Group delay in samples at `f` (fraction of the sample rate).
    pub fn group_delay(&self, f: f64) -> f64 {
        let h = 1e-6;
        let ph = |f: f64| {
            let (r, i) = self.response(2.0 * PI * f);
            i.atan2(r)
        };
        let mut d = ph(f + h) - ph(f - h);
        while d > PI {
            d -= 2.0 * PI;
        }
        while d < -PI {
            d += 2.0 * PI;
        }
        -d / (2.0 * PI * 2.0 * h)
    }

    /// Impulse response energy sum(h^2): the noise power gain for white
    /// input.
    pub fn noise_gain(&self) -> f64 {
        let mut st = IirState::new(self);
        let mut sum = 0.0;
        for n in 0..65536 {
            let y = st.step(if n == 0 { 1.0 } else { 0.0 }) as f64;
            sum += y * y;
            if n > 1024 && y.abs() < 1e-12 {
                break;
            }
        }
        sum
    }

    /// Samples after which the impulse response stays below `eps` of its
    /// peak (for warm-up lengths).
    pub fn settle_samples(&self, eps: f64) -> usize {
        let mut st = IirState::new(self);
        let mut peak = 0.0f64;
        let mut last = 0;
        for n in 0..1_000_000 {
            let y = (st.step(if n == 0 { 1.0 } else { 0.0 }) as f64).abs();
            peak = peak.max(y);
            if y > eps * peak {
                last = n;
            }
            if n > last + 4096 {
                break;
            }
        }
        last + 1
    }
}

/// Direct form II transposed state for a cascade, f32 arithmetic.
#[derive(Clone, Debug)]
pub struct IirState {
    coef: Vec<[f32; 5]>,
    z: Vec<[f32; 2]>,
}

impl IirState {
    pub fn new(c: &Cascade) -> IirState {
        IirState {
            coef: c
                .sections
                .iter()
                .map(|s| [s.b0 as f32, s.b1 as f32, s.b2 as f32, s.a1 as f32, s.a2 as f32])
                .collect(),
            z: vec![[0.0; 2]; c.sections.len()],
        }
    }

    pub fn reset(&mut self) {
        self.z.fill([0.0; 2]);
    }

    #[inline(always)]
    pub fn step(&mut self, mut x: f32) -> f32 {
        for (c, z) in self.coef.iter().zip(self.z.iter_mut()) {
            let y = c[0] * x + z[0];
            z[0] = c[1] * x - c[3] * y + z[1];
            z[1] = c[2] * x - c[4] * y;
            x = y;
        }
        x
    }
}

/// The same cascade applied to I and Q with independent state, unrolled
/// for up to three sections (fifth order).
#[derive(Clone, Debug)]
pub struct ComplexIir {
    coef: [[f32; 5]; 3],
    zr: [[f32; 2]; 3],
    zi: [[f32; 2]; 3],
    n: usize,
}

impl ComplexIir {
    pub fn new(c: &Cascade) -> ComplexIir {
        assert!(c.sections.len() <= 3, "at most 3 sections");
        let mut coef = [[1.0, 0.0, 0.0, 0.0, 0.0]; 3];
        for (k, s) in c.sections.iter().enumerate() {
            coef[k] = [s.b0 as f32, s.b1 as f32, s.b2 as f32, s.a1 as f32, s.a2 as f32];
        }
        ComplexIir {
            coef,
            zr: [[0.0; 2]; 3],
            zi: [[0.0; 2]; 3],
            n: c.sections.len(),
        }
    }

    #[inline(always)]
    pub fn step(&mut self, mut xr: f32, mut xi: f32) -> (f32, f32) {
        for k in 0..self.n {
            let c = &self.coef[k];
            let zr = &mut self.zr[k];
            let yr = c[0] * xr + zr[0];
            zr[0] = c[1] * xr - c[3] * yr + zr[1];
            zr[1] = c[2] * xr - c[4] * yr;
            let zi = &mut self.zi[k];
            let yi = c[0] * xi + zi[0];
            zi[0] = c[1] * xi - c[3] * yi + zi[1];
            zi[1] = c[2] * xi - c[4] * yi;
            xr = yr;
            xi = yi;
        }
        (xr, xi)
    }
}

/// Zeroth-order modified Bessel function (Kaiser window).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let q = x * x / 4.0;
    for k in 1..64 {
        term *= q / (k * k) as f64;
        sum += term;
        if term < 1e-17 * sum {
            break;
        }
    }
    sum
}

/// Kaiser window value at `x` in [-1, 1].
pub fn kaiser(x: f64, beta: f64) -> f64 {
    if x.abs() > 1.0 {
        0.0
    } else {
        bessel_i0(beta * (1.0 - x * x).sqrt()) / bessel_i0(beta)
    }
}

/// Symmetric low-pass FIR (odd length), cutoff `fc` as a fraction of the
/// sample rate, Kaiser window, unity DC gain.
pub fn lowpass_fir(taps: usize, fc: f64, beta: f64) -> Vec<f32> {
    assert!(taps % 2 == 1);
    let m = (taps / 2) as f64;
    let h: Vec<f64> = (0..taps)
        .map(|k| {
            let t = k as f64 - m;
            let s = if t == 0.0 {
                2.0 * fc
            } else {
                (2.0 * PI * fc * t).sin() / (PI * t)
            };
            s * kaiser(t / (m + 1.0), beta)
        })
        .collect();
    let sum: f64 = h.iter().sum();
    h.iter().map(|v| (v / sum) as f32).collect()
}

/// Polyphase fractional interpolator: `phases` sub-sample positions, each
/// `taps` long (even), windowed sinc with cutoff `fc` (fraction of the
/// input rate), every phase normalised to unity DC gain. Phase `p` holds
/// the weights of input samples `floor(x) - taps/2 + 1 ..= floor(x) +
/// taps/2` for `x = floor(x) + p / phases`.
#[derive(Clone, Debug)]
pub struct Polyphase {
    pub taps: usize,
    pub phases: usize,
    pub table: Vec<f32>,
}

impl Polyphase {
    pub fn new(taps: usize, phases: usize, fc: f64, beta: f64) -> Polyphase {
        assert!(taps & 1 == 0 && phases > 0);
        let half = (taps / 2) as f64;
        let mut table = Vec::with_capacity(taps * (phases + 1));
        for p in 0..=phases {
            let frac = p as f64 / phases as f64;
            let row: Vec<f64> = (0..taps)
                .map(|j| {
                    // input sample floor(x) - half + 1 + j is at distance t
                    let t = (j as f64 - half + 1.0) - frac;
                    let s = if t.abs() < 1e-12 {
                        2.0 * fc
                    } else {
                        (2.0 * PI * fc * t).sin() / (PI * t)
                    };
                    s * kaiser(t / half, beta)
                })
                .collect();
            let sum: f64 = row.iter().sum();
            table.extend(row.iter().map(|v| (v / sum) as f32));
        }
        Polyphase {
            taps,
            phases,
            table,
        }
    }

    /// Weights for fractional position `frac` in [0, 1) (nearest phase).
    #[inline(always)]
    pub fn weights(&self, frac: f64) -> &[f32] {
        let p = ((frac * self.phases as f64) + 0.5) as usize;
        let p = p.min(self.phases);
        &self.table[p * self.taps..(p + 1) * self.taps]
    }
}

/// Floor without a libm call (the baseline x86-64 target has no SSE4.1
/// rounding instruction). Valid for |x| < 2^62.
#[inline(always)]
pub fn ffloor(x: f64) -> f64 {
    let t = x as i64 as f64;
    if t > x {
        t - 1.0
    } else {
        t
    }
}

/// Floor to i32 without a libm call, for |x| < 2^31.
#[inline(always)]
pub fn ifloor32(x: f32) -> i32 {
    let t = x as i32;
    if (t as f32) > x {
        t - 1
    } else {
        t
    }
}

/// sin and cos of `2 pi x` for any `x` (turns), absolute error below 1e-6.
#[inline(always)]
pub fn sincos_turns(x: f64) -> (f32, f32) {
    let r = (x - ffloor(x + 0.5)) as f32; // [-0.5, 0.5]
    sincos_turns_reduced(r)
}

/// sin and cos of `2 pi r` for `r` in [-0.5, 0.5].
#[inline(always)]
pub fn sincos_turns_reduced(r: f32) -> (f32, f32) {
    // Fold to [-0.25, 0.25]: sin(pi - a) = sin a, cos(pi - a) = -cos a.
    let (r, flip) = if r > 0.25 {
        (0.5 - r, -1.0f32)
    } else if r < -0.25 {
        (-0.5 - r, -1.0f32)
    } else {
        (r, 1.0f32)
    };
    let t = r * std::f32::consts::TAU;
    let t2 = t * t;
    let s = t * (1.0
        + t2 * (-1.0 / 6.0
            + t2 * (1.0 / 120.0
                + t2 * (-1.0 / 5040.0 + t2 * (1.0 / 362_880.0 + t2 * (-1.0 / 39_916_800.0))))));
    let c = 1.0
        + t2 * (-0.5
            + t2 * (1.0 / 24.0
                + t2 * (-1.0 / 720.0
                    + t2 * (1.0 / 40320.0 + t2 * (-1.0 / 3_628_800.0 + t2 * (1.0 / 479_001_600.0))))));
    (s, c * flip)
}

/// An approximately Gaussian value (zero mean, unit variance) from one
/// 64-bit random word: the scaled sum of its eight bytes (Irwin-Hall of
/// order 8, tails cut at 4.9 sigma). Used for the receiver's input noise
/// at 80 MS/s: the channel filter sums many input samples, which brings
/// the excess kurtosis of its output to about -0.04 (Gaussian: 0).
#[inline(always)]
pub fn gauss8(r: u64) -> f32 {
    // Byte sum: mean 8 x 127.5 = 1020, variance 8 x (256^2 - 1) / 12.
    const S: f32 = 1.0 / 209.022_33; // 1 / sqrt(8 * 65535 / 12)
    let x = (r & 0x00ff_00ff_00ff_00ff) + ((r >> 8) & 0x00ff_00ff_00ff_00ff);
    let x = (x & 0x0000_ffff_0000_ffff) + ((x >> 16) & 0x0000_ffff_0000_ffff);
    let sum = (x & 0xffff_ffff) + (x >> 32);
    (sum as f32 - 1020.0) * S
}

/// Seed mixer (SplitMix64 finaliser over a combination of words).
pub fn mix_seed(parts: &[u64]) -> u64 {
    let mut h = 0x9E37_79B9_7F4A_7C15u64;
    for &p in parts {
        h ^= p.wrapping_add(0x9E37_79B9_7F4A_7C15).wrapping_add(h << 6).wrapping_add(h >> 2);
        h = splitmix(h);
    }
    h
}

#[inline]
pub fn splitmix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Uniform in [0, 1) from a hash.
#[inline]
pub fn unit_from(h: u64) -> f64 {
    (h >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn butterworth_is_3db_at_cutoff_and_flat() {
        for n in 1..=6 {
            let c = Cascade::butterworth_lowpass(n, 0.125);
            assert!((c.magnitude(0.0) - 1.0).abs() < 1e-9);
            let db = 20.0 * c.magnitude(0.125).log10();
            assert!((db + 3.0103).abs() < 0.01, "n={n} {db}");
            // Monotone in the stopband, steep for n = 5.
            if n == 5 {
                let at2 = 20.0 * c.magnitude(0.25).log10();
                assert!(at2 < -28.0, "{at2}");
            }
        }
    }

    #[test]
    fn noise_gain_matches_bandwidth() {
        // 5th order Butterworth: ENBW = fc * (pi/2n)/sin(pi/2n), two-sided
        // for a real filter: sum h^2 = 2 * ENBW / fs.
        let c = Cascade::butterworth_lowpass(5, 0.125);
        let g = c.noise_gain();
        let enbw = 0.125 * (PI / 10.0) / (PI / 10.0).sin();
        assert!((g - 2.0 * enbw).abs() / g < 0.03, "{g} vs {}", 2.0 * enbw);
        assert!(c.settle_samples(1e-9) < 200);
    }

    #[test]
    fn shelf_gains() {
        let s = Cascade::shelf(0.62 / 80.0, 3.1 / 80.0, 1.0);
        assert!((s.magnitude(0.0) - 1.0).abs() < 1e-9);
        let hf = 20.0 * s.magnitude(0.3).log10();
        assert!(hf > 12.0 && hf < 14.5, "{hf}");
    }

    #[test]
    fn sincos_accuracy() {
        let mut worst = 0.0f64;
        for k in -20000..20000 {
            let x = k as f64 * 0.000_731 + 0.0001;
            let (s, c) = sincos_turns(x);
            let a = 2.0 * PI * x;
            worst = worst.max((s as f64 - a.sin()).abs()).max((c as f64 - a.cos()).abs());
        }
        assert!(worst < 2e-6, "{worst}");
    }

    #[test]
    fn polyphase_interpolates_sine() {
        let pp = Polyphase::new(16, 512, 0.45, 7.0);
        let f = 0.2;
        let x: Vec<f32> = (0..200).map(|n| (2.0 * PI * f * n as f64).sin() as f32).collect();
        let mut worst = 0.0f32;
        for k in 0..500 {
            let pos = 40.0 + k as f64 * 0.237;
            let i = pos.floor() as usize;
            let w = pp.weights(pos - pos.floor());
            let v: f32 = w.iter().enumerate().map(|(j, w)| w * x[i + j + 1 - 8]).sum();
            worst = worst.max((v - (2.0 * PI * f * pos).sin() as f32).abs());
        }
        assert!(worst < 5e-3, "{worst}");
    }

    #[test]
    fn gauss8_moments() {
        let mut h = 7u64;
        let n = 400_000;
        let (mut m1, mut m2, mut m4) = (0.0f64, 0.0f64, 0.0f64);
        for _ in 0..n {
            h = splitmix(h);
            let x = gauss8(h) as f64;
            m1 += x;
            m2 += x * x;
            m4 += x * x * x * x;
        }
        let (m1, m2, m4) = (m1 / n as f64, m2 / n as f64, m4 / n as f64);
        assert!(m1.abs() < 0.01 && (m2 - 1.0).abs() < 0.01, "{m1} {m2}");
        // Irwin-Hall order 8 kurtosis is 2.85; after the channel filter
        // it is within 1.5 % of 3.
        assert!((m4 / (m2 * m2) - 2.85).abs() < 0.04);
        let c = Cascade::butterworth_lowpass(5, 0.125);
        let mut st = IirState::new(&c);
        let (mut f2, mut f4) = (0.0f64, 0.0f64);
        for k in 0..n {
            h = splitmix(h);
            let y = st.step(gauss8(h)) as f64;
            if k > 100 {
                f2 += y * y;
                f4 += y * y * y * y;
            }
        }
        let kurt = f4 * (n - 101) as f64 / (f2 * f2);
        assert!((kurt - 3.0).abs() < 0.045, "{kurt}");
    }

    #[test]
    fn fast_floors() {
        for x in [-3.5f64, -3.0, -0.2, 0.0, 0.7, 5.0, 1e12 + 0.5, -1e12 - 0.5] {
            assert_eq!(ffloor(x), x.floor(), "{x}");
        }
        for x in [-3.5f32, -3.0, -0.2, 0.0, 0.7, 5.0, 511.9, -512.0] {
            assert_eq!(ifloor32(x), x.floor() as i32, "{x}");
        }
    }

    #[test]
    fn fir_dc_gain() {
        let h = lowpass_fir(21, 0.1, 6.0);
        assert!((h.iter().sum::<f32>() - 1.0).abs() < 1e-6);
        assert_eq!(h[0], h[20]);
    }

    #[test]
    fn seeds_differ() {
        assert_ne!(mix_seed(&[1, 2]), mix_seed(&[2, 1]));
        assert_ne!(mix_seed(&[0]), mix_seed(&[1]));
        let u = unit_from(splitmix(5));
        assert!((0.0..1.0).contains(&u));
    }
}
