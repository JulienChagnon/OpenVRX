// SPDX-License-Identifier: GPL-3.0-or-later
//! Rolling latency statistics without per-sample allocation.

/// Summary over the current window, milliseconds.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Summary {
    pub count: usize,
    pub min_ms: f64,
    pub avg_ms: f64,
    pub p99_ms: f64,
    pub max_ms: f64,
}

/// Fixed-size ring of the most recent samples (nanoseconds).
#[derive(Clone, Debug)]
pub struct LatencyStats {
    ring: Vec<u64>,
    head: usize,
    len: usize,
    total: u64,
    scratch: Vec<u64>,
}

impl LatencyStats {
    pub fn new(window: usize) -> LatencyStats {
        let window = window.max(1);
        LatencyStats {
            ring: vec![0; window],
            head: 0,
            len: 0,
            total: 0,
            scratch: Vec::with_capacity(window),
        }
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.total = 0;
    }

    pub fn push(&mut self, ns: u64) {
        self.ring[self.head] = ns;
        self.head = (self.head + 1) % self.ring.len();
        self.len = (self.len + 1).min(self.ring.len());
        self.total += 1;
    }

    #[cfg(test)]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Samples pushed since creation or the last clear.
    #[cfg(test)]
    pub fn total(&self) -> u64 {
        self.total
    }

    /// Min / avg / p99 / max over the window (nearest-rank p99).
    pub fn summary(&mut self) -> Summary {
        if self.len == 0 {
            return Summary::default();
        }
        self.scratch.clear();
        self.scratch.extend_from_slice(&self.ring[..self.len]);
        let n = self.scratch.len();
        let sum: u128 = self.scratch.iter().map(|&x| x as u128).sum();
        let min = *self.scratch.iter().min().unwrap_or(&0);
        let max = *self.scratch.iter().max().unwrap_or(&0);
        let rank = ((0.99 * n as f64).ceil() as usize).clamp(1, n) - 1;
        let (_, p99, _) = self.scratch.select_nth_unstable(rank);
        Summary {
            count: n,
            min_ms: min as f64 * 1e-6,
            avg_ms: sum as f64 / n as f64 * 1e-6,
            p99_ms: *p99 as f64 * 1e-6,
            max_ms: max as f64 * 1e-6,
        }
    }
}

/// Events per second over a sliding one-second window.
#[derive(Clone, Debug, Default)]
pub struct RateMeter {
    window_start_ns: u64,
    count: u32,
    rate: f32,
}

impl RateMeter {
    pub fn tick(&mut self, now_ns: u64) {
        if self.window_start_ns == 0 {
            self.window_start_ns = now_ns;
        }
        self.count += 1;
        let dt = now_ns.saturating_sub(self.window_start_ns);
        if dt >= 1_000_000_000 {
            self.rate = self.count as f32 * 1e9 / dt as f32;
            self.count = 0;
            self.window_start_ns = now_ns;
        }
    }

    pub fn rate(&self) -> f32 {
        self.rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty() {
        let mut s = LatencyStats::new(8);
        assert!(s.is_empty());
        assert_eq!(s.summary(), Summary::default());
    }

    #[test]
    fn basic_summary() {
        let mut s = LatencyStats::new(1000);
        for k in 1..=100u64 {
            s.push(k * 1_000_000);
        }
        let m = s.summary();
        assert_eq!(m.count, 100);
        assert!((m.min_ms - 1.0).abs() < 1e-9);
        assert!((m.max_ms - 100.0).abs() < 1e-9);
        assert!((m.avg_ms - 50.5).abs() < 1e-9);
        assert!((m.p99_ms - 99.0).abs() < 1e-9);
    }

    #[test]
    fn window_rolls() {
        let mut s = LatencyStats::new(4);
        for v in [100, 1, 2, 3, 4] {
            s.push(v * 1_000_000);
        }
        let m = s.summary();
        assert_eq!(m.count, 4);
        assert!((m.max_ms - 4.0).abs() < 1e-9);
        assert_eq!(s.total(), 5);
        s.clear();
        assert!(s.is_empty());
    }

    #[test]
    fn p99_single() {
        let mut s = LatencyStats::new(4);
        s.push(7_000_000);
        assert!((s.summary().p99_ms - 7.0).abs() < 1e-9);
    }

    #[test]
    fn rate_meter() {
        let mut r = RateMeter::default();
        for k in 0..=60u64 {
            r.tick(1 + k * 1_000_000_000 / 60);
        }
        assert!((r.rate() - 60.0).abs() < 1.5, "{}", r.rate());
    }
}
