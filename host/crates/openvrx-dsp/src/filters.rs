// SPDX-License-Identifier: GPL-3.0-or-later
//! Filter design and streaming filters.
//!
//! - Kaiser-windowed sinc low-pass FIR design, sized from attenuation and
//!   transition width.
//! - [`SymFir`]: streaming real FIR with linear phase (odd length, integer
//!   group delay), written taps-outer so the inner loop vectorises.
//! - [`Biquad`] cascades, Butterworth low-pass design (bilinear transform
//!   with pre-warping) and a first-order shelf (de-emphasis).
//! - [`InterpTable`]: polyphase windowed-sinc interpolation kernel for
//!   fractional-delay evaluation and resampling.

use std::f64::consts::PI;

/// Zeroth-order modified Bessel function of the first kind (series).
pub fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let q = x * x / 4.0;
    for k in 1..200 {
        term *= q / (k as f64 * k as f64);
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

/// Kaiser beta for a stopband attenuation in dB (Kaiser's formula).
pub fn kaiser_beta(atten_db: f64) -> f64 {
    if atten_db > 50.0 {
        0.1102 * (atten_db - 8.7)
    } else if atten_db >= 21.0 {
        0.5842 * (atten_db - 21.0).powf(0.4) + 0.07886 * (atten_db - 21.0)
    } else {
        0.0
    }
}

/// Odd FIR length for an attenuation and a transition width given as a
/// fraction of the sample rate (Kaiser's estimate), clamped to `max_taps`.
pub fn kaiser_len(atten_db: f64, transition: f64, max_taps: usize) -> usize {
    let tw = (2.0 * PI * transition).max(1e-6);
    let n = ((atten_db - 7.95) / (2.285 * tw)).ceil().max(3.0) as usize + 1;
    let n = n.min(max_taps.max(3));
    n | 1
}

/// Kaiser window value at position `x` in [-1, 1].
#[inline]
pub fn kaiser(x: f64, beta: f64) -> f64 {
    if x.abs() > 1.0 {
        0.0
    } else {
        bessel_i0(beta * (1.0 - x * x).sqrt()) / bessel_i0(beta)
    }
}

/// Normalised sinc: sin(pi x)/(pi x).
#[inline]
pub fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        (PI * x).sin() / (PI * x)
    }
}

/// Linear-phase low-pass FIR, `n` taps (odd), cutoff in cycles per sample
/// (0..0.5, the -6 dB point), Kaiser window. DC gain exactly 1.
pub fn design_lowpass(n: usize, cutoff: f64, beta: f64) -> Vec<f32> {
    let n = n.max(1) | 1;
    let m = (n - 1) as f64 / 2.0;
    let fc = cutoff.clamp(1e-4, 0.5);
    let mut h: Vec<f64> = (0..n)
        .map(|k| {
            let x = k as f64 - m;
            2.0 * fc * sinc(2.0 * fc * x) * if m > 0.0 { kaiser(x / (m + 0.5), beta) } else { 1.0 }
        })
        .collect();
    let s: f64 = h.iter().sum();
    for v in &mut h {
        *v /= s;
    }
    h.into_iter().map(|v| v as f32).collect()
}

/// Magnitude response of a real FIR at `f` cycles per sample.
pub fn fir_response(h: &[f32], f: f64) -> f64 {
    let (mut re, mut im) = (0.0, 0.0);
    for (k, &c) in h.iter().enumerate() {
        let a = -2.0 * PI * f * k as f64;
        re += c as f64 * a.cos();
        im += c as f64 * a.sin();
    }
    (re * re + im * im).sqrt()
}

/// Streaming linear-phase FIR (symmetric taps, odd length). The output
/// sample `k` of a call corresponds to input sample `k - delay()` of the
/// stream, so the group delay is the integer `(len - 1) / 2`.
#[derive(Clone, Debug)]
pub struct SymFir {
    taps: Vec<f32>,
    /// Last `len - 1` inputs, followed by scratch for the current block.
    buf: Vec<f32>,
}

impl SymFir {
    pub fn new(taps: Vec<f32>) -> SymFir {
        assert!(taps.len() % 2 == 1, "SymFir needs an odd length");
        let n = taps.len();
        SymFir {
            taps,
            buf: vec![0.0; n - 1],
        }
    }

    /// Identity filter.
    pub fn identity() -> SymFir {
        SymFir::new(vec![1.0])
    }

    pub fn len(&self) -> usize {
        self.taps.len()
    }

    pub fn is_empty(&self) -> bool {
        self.taps.is_empty()
    }

    pub fn delay(&self) -> usize {
        (self.taps.len() - 1) / 2
    }

    pub fn taps(&self) -> &[f32] {
        &self.taps
    }

    /// Replaces the coefficients, keeping the history when the length is
    /// unchanged (no transient beyond the coefficient change).
    pub fn set_taps(&mut self, taps: Vec<f32>) {
        assert!(taps.len() % 2 == 1);
        if taps.len() != self.taps.len() {
            self.buf = vec![0.0; taps.len() - 1];
        }
        self.taps = taps;
    }

    /// Clears the history.
    pub fn reset(&mut self) {
        let n = self.taps.len() - 1;
        self.buf.clear();
        self.buf.resize(n, 0.0);
    }

    /// Filters `x` into `y` (same length).
    pub fn process(&mut self, x: &[f32], y: &mut [f32]) {
        let n = x.len();
        assert_eq!(y.len(), n);
        let l = self.taps.len();
        if l == 1 {
            let g = self.taps[0];
            for (o, &v) in y.iter_mut().zip(x) {
                *o = v * g;
            }
            return;
        }
        let hist = l - 1;
        self.buf.truncate(hist);
        self.buf.extend_from_slice(x);
        sym_kernel(&self.taps, &self.buf[..hist + n], y);
        self.keep_history(hist);
    }

    /// Filters `x` in place.
    pub fn process_in_place(&mut self, x: &mut [f32]) {
        let n = x.len();
        let l = self.taps.len();
        if l == 1 {
            let g = self.taps[0];
            if g != 1.0 {
                for v in x.iter_mut() {
                    *v *= g;
                }
            }
            return;
        }
        let hist = l - 1;
        self.buf.truncate(hist);
        self.buf.extend_from_slice(x);
        sym_kernel(&self.taps, &self.buf[..hist + n], x);
        self.keep_history(hist);
    }

    /// Keeps the last `hist` inputs at the front of the buffer.
    fn keep_history(&mut self, hist: usize) {
        let total = self.buf.len();
        self.buf.copy_within(total - hist..total, 0);
        self.buf.truncate(hist);
    }
}

dispatch! {
    /// y[k] = sum_j h[j] b[k + j] for symmetric odd-length h. Outputs are
    /// computed 32 at a time in registers (taps inner), so each tap pair
    /// costs two loads and one multiply-add per 8 outputs and nothing is
    /// stored until the block is done.
    fn sym_kernel(taps: &[f32], b: &[f32], y: &mut [f32]) {
        const W: usize = 8;
        const R: usize = 4;
        let l = taps.len();
        let half = l / 2;
        let n = y.len();
        let hc = taps[half];
        let full = n / (W * R) * (W * R);
        let mut k = 0;
        while k < full {
            let mut acc = [[0.0f32; W]; R];
            let c: &[f32; W * R] = b[k + half..k + half + W * R].try_into().expect("block");
            for (r, a) in acc.iter_mut().enumerate() {
                for q in 0..W {
                    a[q] = hc * c[r * W + q];
                }
            }
            for j in 0..half {
                let h = taps[j];
                let lo: &[f32; W * R] = b[k + j..k + j + W * R].try_into().expect("block");
                let hi: &[f32; W * R] = b[k + l - 1 - j..k + l - 1 - j + W * R].try_into().expect("block");
                for (r, a) in acc.iter_mut().enumerate() {
                    for q in 0..W {
                        a[q] += h * (lo[r * W + q] + hi[r * W + q]);
                    }
                }
            }
            let o: &mut [f32; W * R] = (&mut y[k..k + W * R]).try_into().expect("block");
            for (r, a) in acc.iter().enumerate() {
                o[r * W..r * W + W].copy_from_slice(a);
            }
            k += W * R;
        }
        for k in full..n {
            let mut a = hc * b[k + half];
            for j in 0..half {
                a += taps[j] * (b[k + j] + b[k + l - 1 - j]);
            }
            y[k] = a;
        }
    }
}

/// One second-order section, direct form II transposed, f64 state.
#[derive(Copy, Clone, Debug, Default)]
pub struct Biquad {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
    s1: f64,
    s2: f64,
}

impl Biquad {
    pub fn new(b0: f64, b1: f64, b2: f64, a1: f64, a2: f64) -> Biquad {
        Biquad {
            b0,
            b1,
            b2,
            a1,
            a2,
            s1: 0.0,
            s2: 0.0,
        }
    }

    #[inline]
    pub fn run(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.s1;
        self.s1 = self.b1 * x - self.a1 * y + self.s2;
        self.s2 = self.b2 * x - self.a2 * y;
        y
    }

    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    /// Complex response at `f` cycles per sample.
    pub fn response(&self, f: f64) -> (f64, f64) {
        let w = 2.0 * PI * f;
        let (c1, s1, c2, s2) = (w.cos(), -w.sin(), (2.0 * w).cos(), -(2.0 * w).sin());
        let nr = self.b0 + self.b1 * c1 + self.b2 * c2;
        let ni = self.b1 * s1 + self.b2 * s2;
        let dr = 1.0 + self.a1 * c1 + self.a2 * c2;
        let di = self.a1 * s1 + self.a2 * s2;
        let d = dr * dr + di * di;
        ((nr * dr + ni * di) / d, (ni * dr - nr * di) / d)
    }
}

/// Cascade of biquads.
#[derive(Clone, Debug, Default)]
pub struct Sos {
    pub sections: Vec<Biquad>,
}

impl Sos {
    #[inline]
    pub fn run(&mut self, mut x: f64) -> f64 {
        for s in &mut self.sections {
            x = s.run(x);
        }
        x
    }

    pub fn reset(&mut self) {
        for s in &mut self.sections {
            s.reset();
        }
    }

    /// Magnitude and phase-derivative group delay (samples) at `f`.
    pub fn response(&self, f: f64) -> (f64, f64) {
        let (mut re, mut im) = (1.0, 0.0);
        for s in &self.sections {
            let (a, b) = s.response(f);
            let r = re * a - im * b;
            im = re * b + im * a;
            re = r;
        }
        (re, im)
    }

    /// Group delay in samples at `f` cycles per sample (numerical).
    pub fn group_delay(&self, f: f64) -> f64 {
        let df = 1e-5;
        let ph = |f: f64| {
            let (r, i) = self.response(f);
            i.atan2(r)
        };
        let mut d = ph(f + df) - ph((f - df).max(0.0));
        while d > PI {
            d -= 2.0 * PI;
        }
        while d < -PI {
            d += 2.0 * PI;
        }
        let span = (f + df) - (f - df).max(0.0);
        -d / (2.0 * PI * span)
    }
}

/// Butterworth low-pass of order `n`, -3 dB at `cutoff` cycles per sample,
/// as second-order sections (an odd order gets one first-order section,
/// stored as a biquad with b2 = a2 = 0). DC gain 1.
pub fn butterworth_lowpass(order: usize, cutoff: f64) -> Sos {
    let order = order.max(1);
    // Pre-warped analog cutoff for a bilinear transform with T = 1 (fs = 1).
    let wa = 2.0 * (PI * cutoff.clamp(1e-6, 0.499)).tan();
    let k = 2.0; // 2/T
    let mut sections = Vec::new();
    for i in 0..order / 2 {
        // Pole pair at angle theta from the negative real axis.
        let theta = PI * (2 * i + 1) as f64 / (2 * order) as f64;
        let re = -wa * theta.sin();
        let mag2 = wa * wa;
        // H(s) = mag2 / (s^2 - 2 re s + mag2); bilinear s = k (z-1)/(z+1).
        let a0 = k * k - 2.0 * re * k + mag2;
        let a1 = 2.0 * (mag2 - k * k);
        let a2 = k * k + 2.0 * re * k + mag2;
        let g = mag2 / a0;
        sections.push(Biquad::new(g, 2.0 * g, g, a1 / a0, a2 / a0));
    }
    if order % 2 == 1 {
        // H(s) = wa / (s + wa).
        let a0 = k + wa;
        let a1 = wa - k;
        let g = wa / a0;
        sections.push(Biquad::new(g, g, 0.0, a1 / a0, 0.0));
    }
    Sos { sections }
}

/// First-order shelf H(s) = (1 + s/wz) / (1 + s/wp), unity DC gain, as a
/// biquad (bilinear transform, zero and pole pre-warped). `pole_hz` below
/// `zero_hz` gives de-emphasis (high frequencies attenuated by
/// `pole_hz / zero_hz`).
pub fn first_order_shelf(pole_hz: f64, zero_hz: f64, fs: f64) -> Biquad {
    let warp = |f: f64| 2.0 * fs * (PI * (f / fs).clamp(1e-9, 0.499)).tan();
    let wp = warp(pole_hz);
    let wz = warp(zero_hz);
    let k = 2.0 * fs;
    // (1 + s/wz) / (1 + s/wp) with s = k (z - 1)/(z + 1):
    // numerator: (1 + k/wz) + (1 - k/wz) z^-1, denominator likewise.
    let n0 = 1.0 + k / wz;
    let n1 = 1.0 - k / wz;
    let d0 = 1.0 + k / wp;
    let d1 = 1.0 - k / wp;
    Biquad::new(n0 / d0, n1 / d0, 0.0, d1 / d0, 0.0)
}

/// Polyphase windowed-sinc interpolation table.
///
/// `eval(x, i, frac)` returns the band-limited value of `x` at position
/// `i + frac` (0 <= frac < 1) using `taps` samples `x[i - taps/2 + 1 ..=
/// i + taps/2]`. Each phase is normalised to unity DC gain.
#[derive(Clone, Debug)]
pub struct InterpTable {
    taps: usize,
    phases: usize,
    /// (phases + 1) rows of `taps` coefficients.
    table: Vec<f32>,
    /// Derivative kernel (d/dfrac), same layout.
    dtable: Vec<f32>,
}

impl InterpTable {
    /// `taps` even (>= 4), `phases` >= 2, `cutoff` in cycles per input
    /// sample (0.5 = Nyquist), Kaiser `beta`.
    pub fn new(taps: usize, phases: usize, cutoff: f64, beta: f64) -> InterpTable {
        let taps = (taps.max(4) + 1) & !1;
        let phases = phases.max(2);
        let half = taps as f64 / 2.0;
        let fc = cutoff.clamp(0.05, 0.5);
        // The window reaches half a sample past the outer taps so its slope
        // stays finite at every tap (the derivative table needs that).
        let kernel = |tau: f64| 2.0 * fc * sinc(2.0 * fc * tau) * kaiser(tau / (half + 0.5), beta);
        let mut table = vec![0.0f32; (phases + 1) * taps];
        let mut dtable = vec![0.0f32; (phases + 1) * taps];
        for p in 0..=phases {
            let frac = p as f64 / phases as f64;
            let mut row = vec![0.0f64; taps];
            let mut drow = vec![0.0f64; taps];
            for (j, (r, d)) in row.iter_mut().zip(drow.iter_mut()).enumerate() {
                // Sample index relative to i: j - taps/2 + 1; distance tau = frac - rel.
                let rel = j as f64 - half + 1.0;
                let tau = frac - rel;
                *r = kernel(tau);
                let h = 1e-4;
                *d = (kernel(tau + h) - kernel(tau - h)) / (2.0 * h);
            }
            let s: f64 = row.iter().sum();
            let ds: f64 = drow.iter().sum();
            for j in 0..taps {
                table[p * taps + j] = (row[j] / s) as f32;
                // d/dfrac of the normalised kernel row[j] / s (quotient rule);
                // its taps sum to exactly zero, so constants have no slope.
                dtable[p * taps + j] = (drow[j] / s - row[j] * ds / (s * s)) as f32;
            }
        }
        InterpTable {
            taps,
            phases,
            table,
            dtable,
        }
    }

    #[inline]
    pub fn taps(&self) -> usize {
        self.taps
    }

    /// Samples needed before and after position `i`.
    #[inline]
    pub fn reach(&self) -> (usize, usize) {
        (self.taps / 2 - 1, self.taps / 2)
    }

    #[inline]
    fn row(&self, frac: f64) -> usize {
        let p = (frac * self.phases as f64).round() as usize;
        p.min(self.phases) * self.taps
    }

    /// Coefficients for a fractional position.
    #[inline]
    pub fn coeffs(&self, frac: f64) -> &[f32] {
        let r = self.row(frac);
        &self.table[r..r + self.taps]
    }

    /// Value at `i + frac`; caller guarantees the reach is inside `x`.
    #[inline]
    pub fn eval(&self, x: &[f32], i: usize, frac: f64) -> f32 {
        let c = self.coeffs(frac);
        let s = &x[i + 1 - self.taps / 2..i + 1 + self.taps / 2];
        dot(c, s)
    }

    /// Derivative (per sample) at `i + frac`.
    #[inline]
    pub fn eval_deriv(&self, x: &[f32], i: usize, frac: f64) -> f32 {
        let r = self.row(frac);
        let c = &self.dtable[r..r + self.taps];
        let s = &x[i + 1 - self.taps / 2..i + 1 + self.taps / 2];
        dot(c, s)
    }

    /// Value at an absolute fractional position `t` (in `x` indices),
    /// clamping to the valid range.
    #[inline]
    pub fn eval_at(&self, x: &[f32], t: f64) -> f32 {
        let (lo, hi) = self.reach();
        if x.len() < lo + hi + 1 {
            return x.first().copied().unwrap_or(0.0);
        }
        let t = t.clamp(lo as f64, (x.len() - 1 - hi) as f64);
        let i = t.floor() as usize;
        self.eval(x, i, t - i as f64)
    }
}

impl InterpTable {
    /// Resamples `x` at positions `t0 + k * step` (local indices) into
    /// `out`; positions without full reach get `fill`. Returns how many
    /// samples needed the fill.
    pub fn resample(&self, x: &[f32], t0: f64, step: f64, out: &mut [f32], fill: f32) -> usize {
        resample_kernel(&self.table, self.taps, self.phases, x, t0, step, out, fill)
    }
}

/// Fixed-point scale of the resampler positions (32 fractional bits).
const FX: f64 = 4_294_967_296.0;

dispatch! {
    /// Positions advance in 32.32 fixed point: f64 to integer conversions
    /// per output cost several times the 16-tap dot product itself.
    #[allow(clippy::too_many_arguments)]
    fn resample_kernel(
        table: &[f32],
        taps: usize,
        phases: usize,
        x: &[f32],
        t0: f64,
        step: f64,
        out: &mut [f32],
        fill: f32,
    ) -> usize {
        let half = taps / 2;
        let mut missing = 0;
        let ph = phases as u64;
        let lo = (half - 1) as i64;
        let hi = x.len() as i64 - half as i64 - 1;
        let mut pos = (t0 * FX).round() as i64;
        let st = (step * FX).round() as i64;
        for o in out.iter_mut() {
            let i = pos >> 32;
            let frac = (pos & 0xffff_ffff) as u64;
            pos += st;
            if i < lo || i > hi {
                *o = fill;
                missing += 1;
                continue;
            }
            let iu = i as usize;
            let p = ((frac * ph + (1 << 31)) >> 32) as usize;
            if taps == 16 {
                let c: &[f32; 16] = table[p * 16..p * 16 + 16].try_into().expect("16 taps");
                let s: &[f32; 16] = x[iu - 7..iu + 9].try_into().expect("16 taps");
                *o = dot16(c, s);
            } else {
                let c = &table[p * taps..p * taps + taps];
                let s = &x[iu + 1 - half..iu + 1 + half];
                *o = dot(c, s);
            }
        }
        missing
    }
}

/// 16-tap dot product written so it maps onto two 8-lane FMAs.
#[inline(always)]
fn dot16(c: &[f32; 16], s: &[f32; 16]) -> f32 {
    let mut a = [0.0f32; 8];
    for j in 0..8 {
        a[j] = c[j] * s[j] + c[j + 8] * s[j + 8];
    }
    let b = [a[0] + a[4], a[1] + a[5], a[2] + a[6], a[3] + a[7]];
    (b[0] + b[2]) + (b[1] + b[3])
}

pub use crate::vmath::dot;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_meets_spec() {
        let fs = 13.333e6;
        let n = kaiser_len(55.0, 1.2e6 / fs, 255);
        let h = design_lowpass(n, 5.0e6 / fs + 0.6e6 / fs, kaiser_beta(55.0));
        assert!((fir_response(&h, 0.0) - 1.0).abs() < 1e-6);
        assert!(fir_response(&h, 4.4e6 / fs) > 0.98);
        assert!(fir_response(&h, 6.3e6 / fs) < 0.01);
    }

    #[test]
    fn symfir_matches_direct_convolution() {
        let h = design_lowpass(15, 0.2, 6.0);
        let x: Vec<f32> = (0..1000).map(|k| ((k * 7919) % 101) as f32 - 50.0).collect();
        let mut f = SymFir::new(h.clone());
        let mut y = vec![0.0; x.len()];
        // Process in irregular blocks.
        let mut pos = 0;
        for b in [1usize, 7, 100, 3, 500, 389] {
            f.process(&x[pos..pos + b], &mut y[pos..pos + b]);
            pos += b;
        }
        for k in 0..x.len() {
            let mut s = 0.0f64;
            for (j, &c) in h.iter().enumerate() {
                if k >= j {
                    s += c as f64 * x[k - j] as f64;
                }
            }
            assert!((s as f32 - y[k]).abs() < 1e-3, "k={k}");
        }
        assert_eq!(f.delay(), 7);
    }

    #[test]
    fn butterworth_response() {
        let sos = butterworth_lowpass(5, 10e6 / 80e6);
        let mag = |f: f64| {
            let (r, i) = sos.response(f);
            (r * r + i * i).sqrt()
        };
        assert!((mag(0.0) - 1.0).abs() < 1e-9);
        assert!((mag(10e6 / 80e6) - 0.5f64.sqrt()).abs() < 1e-6);
        assert!(mag(20e6 / 80e6) < 0.05);
        // Impulse response sums to the DC gain.
        let mut s = butterworth_lowpass(5, 0.125);
        let mut acc = 0.0;
        for k in 0..2000 {
            acc += s.run(if k == 0 { 1.0 } else { 0.0 });
        }
        assert!((acc - 1.0).abs() < 1e-9);
        // Even order.
        let s4 = butterworth_lowpass(4, 0.1);
        let (r, i) = s4.response(0.1);
        assert!(((r * r + i * i).sqrt() - 0.5f64.sqrt()).abs() < 1e-6);
    }

    #[test]
    fn shelf_gains() {
        let fs = 13.333e6;
        let b = first_order_shelf(0.5e6, 2.0e6, fs);
        let mag = |f: f64| {
            let (r, i) = b.response(f / fs);
            (r * r + i * i).sqrt()
        };
        assert!((mag(0.0) - 1.0).abs() < 1e-9);
        assert!((mag(6.0e6) - 0.25).abs() < 0.03);
    }

    #[test]
    fn interp_reconstructs_bandlimited() {
        let t = InterpTable::new(16, 512, 0.45, 7.0);
        let f = 0.17;
        let x: Vec<f32> = (0..200).map(|k| (2.0 * PI * f * k as f64 + 0.3).sin() as f32).collect();
        let mut worst = 0.0f64;
        let mut worst_d = 0.0f64;
        for k in 0..400 {
            let pos = 20.0 + k as f64 * 0.37;
            let i = pos.floor() as usize;
            let v = t.eval(&x, i, pos - i as f64) as f64;
            let exact = (2.0 * PI * f * pos + 0.3).sin();
            worst = worst.max((v - exact).abs());
            let d = t.eval_deriv(&x, i, pos - i as f64) as f64;
            let exact_d = 2.0 * PI * f * (2.0 * PI * f * pos + 0.3).cos();
            worst_d = worst_d.max((d - exact_d).abs());
        }
        assert!(worst < 2e-3, "interp error {worst}");
        assert!(worst_d < 1e-2, "deriv error {worst_d}");
    }
}
