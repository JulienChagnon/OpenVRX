// SPDX-License-Identifier: GPL-3.0-or-later
//! Per-field processing times.

use std::collections::VecDeque;

/// Times in ms: the last `cap` for the percentiles, all of them for the
/// count, mean and maximum.
#[derive(Clone, Debug)]
pub struct Timing {
    window: VecDeque<f32>,
    cap: usize,
    count: u64,
    sum_ms: f64,
    max_ms: f32,
}

/// A summary of [`Timing`].
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TimingSummary {
    pub count: u64,
    pub p50_ms: f32,
    pub p99_ms: f32,
    pub max_ms: f32,
    pub mean_ms: f32,
}

impl Timing {
    /// Percentiles over the last `cap` samples (`usize::MAX`: all).
    pub fn new(cap: usize) -> Timing {
        Timing {
            window: VecDeque::new(),
            cap: cap.max(1),
            count: 0,
            sum_ms: 0.0,
            max_ms: 0.0,
        }
    }

    pub fn push(&mut self, ms: f32) {
        if self.window.len() == self.cap {
            self.window.pop_front();
        }
        self.window.push_back(ms);
        self.count += 1;
        self.sum_ms += ms as f64;
        self.max_ms = self.max_ms.max(ms);
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    /// Samples in the window above `ms`.
    pub fn over(&self, ms: f32) -> usize {
        self.window.iter().filter(|&&x| x > ms).count()
    }

    pub fn summary(&self) -> TimingSummary {
        if self.count == 0 {
            return TimingSummary::default();
        }
        let mut s: Vec<f32> = self.window.iter().copied().collect();
        s.sort_by(f32::total_cmp);
        TimingSummary {
            count: self.count,
            p50_ms: percentile(&s, 0.50),
            p99_ms: percentile(&s, 0.99),
            max_ms: self.max_ms,
            mean_ms: (self.sum_ms / self.count as f64) as f32,
        }
    }
}

/// Nearest-rank percentile of sorted samples.
fn percentile(sorted: &[f32], q: f64) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let rank = (q * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_over_the_window() {
        let mut t = Timing::new(100);
        assert_eq!(t.summary(), TimingSummary::default());
        for k in 1..=100 {
            t.push(k as f32);
        }
        let s = t.summary();
        assert_eq!((s.count, s.p50_ms, s.p99_ms, s.max_ms), (100, 50.0, 99.0, 100.0));
        assert!((s.mean_ms - 50.5).abs() < 1e-4);
        assert_eq!(t.over(90.0), 10);
        // The window keeps the newest 100; count, mean and max keep all.
        for _ in 0..100 {
            t.push(1.0);
        }
        let s = t.summary();
        assert_eq!((s.count, s.p50_ms, s.p99_ms, s.max_ms), (200, 1.0, 1.0, 100.0));
    }
}
