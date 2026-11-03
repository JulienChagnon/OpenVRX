// SPDX-License-Identifier: GPL-3.0-or-later
//! Averaged power spectrum of the input I/Q for the spectrum/waterfall view.
//!
//! Hann-windowed FFT frames are averaged; a published spectrum is
//! `fft_size` values in dB (ADC units^2 per bin, window power removed),
//! ordered from -fs/2 to +fs/2 (DC in the middle). Frames between
//! publications are skipped so the cost follows `rate_hz`, not the sample
//! rate.

use crate::config::SpectrumConfig;
use openvrx_core::Complex32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

pub struct SpectrumTap {
    cfg: SpectrumConfig,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    frame: Vec<Complex32>,
    scratch: Vec<Complex32>,
    acc: Vec<f32>,
    frames: usize,
    /// Samples to skip before the next frame.
    skip: u64,
    gap: u64,
    ready: Option<Vec<f32>>,
}

impl std::fmt::Debug for SpectrumTap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SpectrumTap").field("cfg", &self.cfg).finish()
    }
}

impl SpectrumTap {
    pub fn new(cfg: &SpectrumConfig, fs: f64) -> SpectrumTap {
        let n = cfg.fft_size.max(16);
        let fft = FftPlanner::new().plan_fft_forward(n);
        let window: Vec<f32> = (0..n)
            .map(|k| (0.5 - 0.5 * (std::f64::consts::TAU * k as f64 / n as f64).cos()) as f32)
            .collect();
        let wpow: f32 = window.iter().map(|w| w * w).sum();
        let window = window.into_iter().map(|w| w / wpow.sqrt()).collect();
        let per = (fs / cfg.rate_hz.max(0.1) as f64) as u64;
        let used = (n * cfg.averages.max(1)) as u64;
        let scratch_len = fft.get_inplace_scratch_len();
        SpectrumTap {
            cfg: cfg.clone(),
            fft,
            window,
            frame: Vec::with_capacity(n),
            scratch: vec![Complex32::new(0.0, 0.0); scratch_len],
            acc: vec![0.0; n],
            frames: 0,
            skip: 0,
            gap: per.saturating_sub(used) / cfg.averages.max(1) as u64,
            ready: None,
        }
    }

    pub fn push(&mut self, re: &[f32], im: &[f32]) {
        let n = self.cfg.fft_size.max(16);
        let mut k = 0;
        while k < re.len() {
            if self.skip > 0 {
                let s = (self.skip as usize).min(re.len() - k);
                self.skip -= s as u64;
                k += s;
                continue;
            }
            let take = (n - self.frame.len()).min(re.len() - k);
            self.frame
                .extend((k..k + take).map(|j| Complex32::new(re[j], im[j])));
            k += take;
            if self.frame.len() == n {
                for (z, &w) in self.frame.iter_mut().zip(&self.window) {
                    *z *= w;
                }
                self.fft.process_with_scratch(&mut self.frame, &mut self.scratch);
                for (a, z) in self.acc.iter_mut().zip(&self.frame) {
                    *a += z.norm_sqr();
                }
                self.frame.clear();
                self.frames += 1;
                self.skip = self.gap;
                if self.frames >= self.cfg.averages.max(1) {
                    let inv = 1.0 / self.frames as f32;
                    let mut out = vec![0.0f32; n];
                    for (j, o) in out.iter_mut().enumerate() {
                        let src = (j + n / 2 + n % 2) % n;
                        *o = 10.0 * (self.acc[src] * inv).max(1e-12).log10();
                    }
                    self.ready = Some(out);
                    self.acc.fill(0.0);
                    self.frames = 0;
                }
            }
        }
    }

    pub fn take(&mut self) -> Option<Vec<f32>> {
        self.ready.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tone_lands_in_right_bin() {
        let fs = 13.333e6;
        let cfg = SpectrumConfig {
            enabled: true,
            fft_size: 256,
            averages: 4,
            rate_hz: 1000.0,
        };
        let mut t = SpectrumTap::new(&cfg, fs);
        let f = 2.0e6;
        let n = 20000;
        let re: Vec<f32> = (0..n).map(|k| (50.0 * (std::f64::consts::TAU * f * k as f64 / fs).cos()) as f32).collect();
        let im: Vec<f32> = (0..n).map(|k| (50.0 * (std::f64::consts::TAU * f * k as f64 / fs).sin()) as f32).collect();
        t.push(&re, &im);
        let s = t.take().expect("spectrum");
        let peak = s.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        let expect = 128 + (f / fs * 256.0).round() as usize;
        assert!((peak as i64 - expect as i64).abs() <= 1, "peak {peak} expect {expect}");
        // Power 2500 per sample with a unit-power Hann window: peak at
        // P + 10 log10(N) - 1.76 dB, less up to 1.4 dB scalloping.
        let expect_db = 10.0 * 2500f32.log10() + 10.0 * 256f32.log10() - 1.76;
        assert!(s[peak] <= expect_db + 0.2 && s[peak] > expect_db - 1.6, "{}", s[peak]);
        assert!(t.take().is_none());
    }
}
