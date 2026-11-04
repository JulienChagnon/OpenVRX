// SPDX-License-Identifier: GPL-3.0-or-later
//! Small signal-processing helpers shared by the video modules: FIR design
//! and application, fractional delay, a deterministic random generator and
//! a few scalar helpers.

use std::f64::consts::PI;

/// Blackman-windowed sinc lowpass with unity DC gain. `cutoff` is in cycles
/// per sample (0..0.5), `taps` must be odd.
pub fn design_lowpass(cutoff: f64, taps: usize) -> Vec<f32> {
    assert!(taps % 2 == 1, "FIR length must be odd");
    let h = (taps / 2) as f64;
    let mut out: Vec<f64> = (0..taps)
        .map(|i| {
            let n = i as f64 - h;
            let sinc = if n == 0.0 {
                2.0 * cutoff
            } else {
                (2.0 * PI * cutoff * n).sin() / (PI * n)
            };
            let w = 0.42 + 0.5 * (PI * n / (h + 1.0)).cos() + 0.08 * (2.0 * PI * n / (h + 1.0)).cos();
            sinc * w
        })
        .collect();
    let sum: f64 = out.iter().sum();
    for v in &mut out {
        *v /= sum;
    }
    out.into_iter().map(|v| v as f32).collect()
}

/// Bandpass around `center` (cycles/sample) with half bandwidth
/// `half_bw`, unity gain at the centre frequency.
pub fn design_bandpass(center: f64, half_bw: f64, taps: usize) -> Vec<f32> {
    let lp = design_lowpass(half_bw, taps);
    let h = (taps / 2) as f64;
    let mut out: Vec<f64> = lp
        .iter()
        .enumerate()
        .map(|(i, &v)| 2.0 * v as f64 * (2.0 * PI * center * (i as f64 - h)).cos())
        .collect();
    // Normalise the response at the centre frequency to exactly 1.
    let (mut re, mut im) = (0.0, 0.0);
    for (i, v) in out.iter().enumerate() {
        let ph = 2.0 * PI * center * (i as f64 - h);
        re += v * ph.cos();
        im += v * ph.sin();
    }
    let g = (re * re + im * im).sqrt();
    for v in &mut out {
        *v /= g;
    }
    out.into_iter().map(|v| v as f32).collect()
}

/// Magnitude response of a symmetric FIR at `freq` cycles/sample.
pub fn fir_response(taps: &[f32], freq: f64) -> f64 {
    let h = (taps.len() / 2) as f64;
    let (mut re, mut im) = (0.0, 0.0);
    for (i, &v) in taps.iter().enumerate() {
        let ph = 2.0 * PI * freq * (i as f64 - h);
        re += v as f64 * ph.cos();
        im += v as f64 * ph.sin();
    }
    (re * re + im * im).sqrt()
}

/// Applies an odd-length FIR to `input`, writing `out[i]` for output
/// indices `range` (out is indexed relative to `range.start`). Input
/// indices outside `input` are clamped to the edge sample.
///
/// The interior is computed tap by tap (`out += tap * shifted input`),
/// which vectorises; only the edges use clamped indexing.
pub fn fir_range(input: &[f32], taps: &[f32], range: std::ops::Range<usize>, out: &mut [f32]) {
    let n = input.len();
    let h = taps.len() / 2;
    debug_assert_eq!(out.len(), range.len());
    // Interior output indices: x - h >= 0 and x + h < n.
    let lo = range.start.max(h).min(range.end);
    let hi = range.end.min(n.saturating_sub(h)).max(lo);
    let edge = |x: usize| -> f32 {
        let mut acc = 0.0f32;
        for (k, t) in taps.iter().enumerate() {
            let idx = (x as isize + k as isize - h as isize).clamp(0, n as isize - 1) as usize;
            acc += input[idx] * t;
        }
        acc
    };
    for x in range.start..lo {
        out[x - range.start] = edge(x);
    }
    for x in hi..range.end {
        out[x - range.start] = edge(x);
    }
    if hi > lo {
        fir_interior(input, taps, lo, hi, &mut out[lo - range.start..hi - range.start]);
    }
}

/// FIR over fully interior outputs `lo..hi` (input index x - h .. x + h
/// valid), eight outputs at a time with the accumulators in registers.
/// Uses AVX2/FMA when the CPU has it (detected once at run time).
fn fir_interior(input: &[f32], taps: &[f32], lo: usize, hi: usize, out: &mut [f32]) {
    #[cfg(target_arch = "x86_64")]
    {
        if has_avx2() {
            // SAFETY: the CPU supports AVX2 and FMA (checked above).
            unsafe { fir_interior_avx2(input, taps, lo, hi, out) };
            return;
        }
    }
    fir_interior_generic(input, taps, lo, hi, out);
}

#[cfg(target_arch = "x86_64")]
#[inline]
pub(crate) fn has_avx2() -> bool {
    use std::sync::atomic::{AtomicU8, Ordering};
    static STATE: AtomicU8 = AtomicU8::new(0);
    match STATE.load(Ordering::Relaxed) {
        1 => true,
        2 => false,
        _ => {
            let yes = is_x86_feature_detected!("avx2") && is_x86_feature_detected!("fma");
            STATE.store(if yes { 1 } else { 2 }, Ordering::Relaxed);
            yes
        }
    }
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
unsafe fn fir_interior_avx2(input: &[f32], taps: &[f32], lo: usize, hi: usize, out: &mut [f32]) {
    use std::arch::x86_64::*;
    let h = taps.len() / 2;
    let n = hi - lo;
    let base = &input[lo - h..hi + h];
    let nt = taps.len();
    let mut i = 0;
    // SAFETY: every load reads base[i + k .. i + k + 32] (or + 8) with
    // i + 32 <= n (or i + 8 <= n) and k < nt, inside base (len n + nt - 1);
    // stores write out[i .. i + 32] (or + 8) inside out (len n).
    unsafe {
        let bp = base.as_ptr();
        let op = out.as_mut_ptr();
        while i + 32 <= n {
            let mut a0 = _mm256_setzero_ps();
            let mut a1 = _mm256_setzero_ps();
            let mut a2 = _mm256_setzero_ps();
            let mut a3 = _mm256_setzero_ps();
            for (k, &tv) in taps.iter().enumerate().take(nt) {
                let t = _mm256_set1_ps(tv);
                let p = bp.add(i + k);
                a0 = _mm256_fmadd_ps(t, _mm256_loadu_ps(p), a0);
                a1 = _mm256_fmadd_ps(t, _mm256_loadu_ps(p.add(8)), a1);
                a2 = _mm256_fmadd_ps(t, _mm256_loadu_ps(p.add(16)), a2);
                a3 = _mm256_fmadd_ps(t, _mm256_loadu_ps(p.add(24)), a3);
            }
            _mm256_storeu_ps(op.add(i), a0);
            _mm256_storeu_ps(op.add(i + 8), a1);
            _mm256_storeu_ps(op.add(i + 16), a2);
            _mm256_storeu_ps(op.add(i + 24), a3);
            i += 32;
        }
        while i + 8 <= n {
            let mut a0 = _mm256_setzero_ps();
            for (k, &tv) in taps.iter().enumerate().take(nt) {
                a0 = _mm256_fmadd_ps(_mm256_set1_ps(tv), _mm256_loadu_ps(bp.add(i + k)), a0);
            }
            _mm256_storeu_ps(op.add(i), a0);
            i += 8;
        }
    }
    while i < n {
        let mut acc = 0.0f32;
        for (k, &t) in taps.iter().enumerate() {
            acc += t * base[i + k];
        }
        out[i] = acc;
        i += 1;
    }
}

#[inline(always)]
fn fir_interior_generic(input: &[f32], taps: &[f32], lo: usize, hi: usize, out: &mut [f32]) {
    let h = taps.len() / 2;
    let n = hi - lo;
    let base = &input[lo - h..hi + h];
    let mut i = 0;
    while i + 16 <= n {
        let mut a0 = [0.0f32; 8];
        let mut a1 = [0.0f32; 8];
        for (k, &t) in taps.iter().enumerate() {
            let s0 = &base[i + k..i + k + 8];
            let s1 = &base[i + k + 8..i + k + 16];
            for j in 0..8 {
                a0[j] += t * s0[j];
                a1[j] += t * s1[j];
            }
        }
        out[i..i + 8].copy_from_slice(&a0);
        out[i + 8..i + 16].copy_from_slice(&a1);
        i += 16;
    }
    while i < n {
        let mut acc = 0.0f32;
        for (k, &t) in taps.iter().enumerate() {
            acc += t * base[i + k];
        }
        out[i] = acc;
        i += 1;
    }
}

/// Lanczos-3 fractional delay: `out[i] = input(i - delay)` with clamped
/// edges. Positive delay moves content to the right (later).
pub fn fractional_delay(input: &[f32], delay: f64, out: &mut [f32]) {
    let n = input.len() as isize;
    let whole = delay.floor();
    let frac = delay - whole;
    let whole = whole as isize;
    if frac.abs() < 1e-6 {
        for (i, o) in out.iter_mut().enumerate() {
            let idx = (i as isize - whole).clamp(0, n - 1) as usize;
            *o = input[idx];
        }
        return;
    }
    // Taps for sample positions i - whole - k, k = -2..=3, evaluated at
    // offset frac.
    let mut taps = [0.0f32; 6];
    let mut sum = 0.0;
    for (j, t) in taps.iter_mut().enumerate() {
        let k = j as f64 - 2.0; // source offset relative to i - whole
        let x = frac - k; // distance from the wanted position
        let v = lanczos(x, 3.0);
        *t = v as f32;
        sum += v;
    }
    for t in &mut taps {
        *t /= sum as f32;
    }
    for (i, o) in out.iter_mut().enumerate() {
        let base = i as isize - whole;
        let mut acc = 0.0f32;
        for (j, t) in taps.iter().enumerate() {
            let idx = (base - (j as isize - 2)).clamp(0, n - 1) as usize;
            acc += input[idx] * t;
        }
        *o = acc;
    }
}

#[inline]
fn lanczos(x: f64, a: f64) -> f64 {
    if x.abs() < 1e-9 {
        1.0
    } else if x.abs() >= a {
        0.0
    } else {
        let px = PI * x;
        a * px.sin() * (px / a).sin() / (px * px)
    }
}

/// Catmull-Rom style cubic interpolation of four equally spaced samples at
/// fractional position `t` between `p1` and `p2`.
#[inline]
pub fn cubic(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
    let b = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
    let c = -0.5 * p0 + 0.5 * p2;
    ((a * t + b) * t + c) * t + p1
}

/// Samples a row at fractional position with linear interpolation and
/// clamped edges.
#[inline]
pub fn sample_linear(row: &[f32], x: f32) -> f32 {
    let n = row.len();
    let xf = x.floor();
    let t = x - xf;
    let i0 = (xf as isize).clamp(0, n as isize - 1) as usize;
    let i1 = (xf as isize + 1).clamp(0, n as isize - 1) as usize;
    row[i0] + (row[i1] - row[i0]) * t
}

#[inline]
pub fn median3(a: f32, b: f32, c: f32) -> f32 {
    a.max(b).min(a.min(b).max(c))
}

#[inline]
pub fn smoothstep(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Wraps an angle to (-pi, pi].
#[inline]
pub fn wrap_pi(a: f64) -> f64 {
    let mut x = (a + PI).rem_euclid(2.0 * PI) - PI;
    if x <= -PI {
        x += 2.0 * PI;
    }
    x
}

/// Horizontal max filter of a byte mask with radius `r`, into `out`.
pub fn dilate_u8(input: &[u8], r: usize, out: &mut [u8]) {
    let n = input.len();
    if r == 0 {
        out.copy_from_slice(input);
        return;
    }
    out.fill(0);
    // Spread each nonzero run; damage is sparse so this is cheap.
    let mut i = 0;
    while i < n {
        let v = input[i];
        if v == 0 {
            i += 1;
            continue;
        }
        let lo = i.saturating_sub(r);
        let hi = (i + r + 1).min(n);
        for o in &mut out[lo..hi] {
            if *o < v {
                *o = v;
            }
        }
        i += 1;
    }
}

/// Deterministic xorshift64* generator with Gaussian samples (Box-Muller).
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng {
            state: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            spare: None,
        }
    }

    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in [0, 1).
    #[inline]
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal.
    pub fn gauss(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        loop {
            let u = 2.0 * self.uniform() - 1.0;
            let v = 2.0 * self.uniform() - 1.0;
            let s = u * u + v * v;
            if s > 0.0 && s < 1.0 {
                let m = (-2.0 * s.ln() / s).sqrt();
                self.spare = Some(v * m);
                return u * m;
            }
        }
    }

    /// Uniform integer in [0, n).
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.next_u64() % n
        }
    }
}

/// Zipped mutable rows (Y, U, V, damage) as produced by
/// `par_chunks_mut(..).zip(..)` chains.
pub type RowsMut<'a> = (((&'a mut [f32], &'a mut [f32]), &'a mut [f32]), &'a mut [u8]);
/// Zipped mutable rows (Y, U, V).
pub type Rows3Mut<'a> = ((&'a mut [f32], &'a mut [f32]), &'a mut [f32]);

/// A per-thread recycled `f32` buffer. Hot per-line code takes its
/// temporaries from here instead of allocating; the buffer returns to the
/// calling thread's pool when dropped.
pub struct Scratch {
    v: Vec<f32>,
}

thread_local! {
    static POOL: std::cell::RefCell<Vec<Vec<f32>>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// A scratch buffer of `n` values, all set to `fill`.
pub fn scratch(n: usize, fill: f32) -> Scratch {
    let mut v = POOL.with(|p| p.borrow_mut().pop()).unwrap_or_default();
    v.clear();
    v.resize(n, fill);
    Scratch { v }
}

/// A scratch buffer of `n` values with unspecified contents, for buffers
/// the caller overwrites completely (skips the fill).
pub fn scratch_any(n: usize) -> Scratch {
    let mut v = POOL.with(|p| p.borrow_mut().pop()).unwrap_or_default();
    if v.len() >= n {
        v.truncate(n);
    } else {
        v.resize(n, 0.0);
    }
    Scratch { v }
}

impl std::ops::Deref for Scratch {
    type Target = [f32];
    fn deref(&self) -> &[f32] {
        &self.v
    }
}

impl std::ops::DerefMut for Scratch {
    fn deref_mut(&mut self) -> &mut [f32] {
        &mut self.v
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let v = std::mem::take(&mut self.v);
        POOL.with(|p| {
            let mut p = p.borrow_mut();
            if p.len() < 64 {
                p.push(v);
            }
        });
    }
}

/// 5-tap moving average (edges average the available taps).
#[inline(always)]
pub fn box5_into(v: &[f32], out: &mut [f32]) {
    let n = v.len();
    if n < 5 {
        for (j, o) in out.iter_mut().enumerate().take(n) {
            let lo = j.saturating_sub(2);
            let hi = (j + 3).min(n);
            *o = v[lo..hi].iter().sum::<f32>() / (hi - lo) as f32;
        }
        return;
    }
    // Equal-length shifted slices: no bounds checks, vectorises.
    let m = n - 4;
    let (a, b, c, d, e) = (&v[0..m], &v[1..m + 1], &v[2..m + 2], &v[3..m + 3], &v[4..m + 4]);
    for (j, o) in out[2..m + 2].iter_mut().enumerate() {
        *o = (a[j] + b[j] + c[j] + d[j] + e[j]) * 0.2;
    }
    out[0] = (v[0] + v[1] + v[2]) / 3.0;
    out[1] = (v[0] + v[1] + v[2] + v[3]) * 0.25;
    out[n - 2] = (v[n - 4] + v[n - 3] + v[n - 2] + v[n - 1]) * 0.25;
    out[n - 1] = (v[n - 3] + v[n - 2] + v[n - 1]) / 3.0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_and_bandpass_responses() {
        let lp = design_lowpass(0.1, 31);
        assert!((fir_response(&lp, 0.0) - 1.0).abs() < 1e-6);
        assert!(fir_response(&lp, 0.25) < 1e-3);
        let bp = design_bandpass(0.25, 0.08, 31);
        assert!((fir_response(&bp, 0.25) - 1.0).abs() < 1e-6);
        assert!(fir_response(&bp, 0.0) < 1e-3);
    }

    #[test]
    fn fractional_delay_moves_sine() {
        let f = 0.05;
        let src: Vec<f32> = (0..200).map(|i| (2.0 * PI * f * i as f64).sin() as f32).collect();
        let mut out = vec![0.0; 200];
        fractional_delay(&src, 0.37, &mut out);
        for (i, &o) in out.iter().enumerate().take(180).skip(20) {
            let want = (2.0 * PI * f * (i as f64 - 0.37)).sin() as f32;
            assert!((o - want).abs() < 5e-3, "{i}: {o} vs {want}");
        }
    }

    #[test]
    fn wrap_and_median() {
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(-0.5) + 0.5).abs() < 1e-12);
        assert_eq!(median3(3.0, 1.0, 2.0), 2.0);
        assert_eq!(median3(1.0, 3.0, 2.0), 2.0);
    }

    #[test]
    fn dilate() {
        let m = [0u8, 0, 0, 200, 0, 0, 0, 0];
        let mut o = [0u8; 8];
        dilate_u8(&m, 2, &mut o);
        assert_eq!(o, [0, 200, 200, 200, 200, 200, 0, 0]);
    }
}
