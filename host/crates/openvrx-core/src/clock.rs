//! One monotonic clock for the whole process.
//!
//! USB completions, DSP output, display presents and control commands are
//! all stamped with [`now_ns`], so latency can be measured stage to stage.
//! `std::time::Instant` uses QueryPerformanceCounter on Windows.

use std::sync::OnceLock;
use std::time::Instant;

fn epoch() -> Instant {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    *EPOCH.get_or_init(Instant::now)
}

/// Nanoseconds since the first call in this process.
#[inline]
pub fn now_ns() -> u64 {
    epoch().elapsed().as_nanos() as u64
}

/// Converts a duration in nanoseconds to milliseconds as f64.
#[inline]
pub fn ns_to_ms(ns: u64) -> f64 {
    ns as f64 * 1e-6
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn monotonic() {
        let a = now_ns();
        let b = now_ns();
        assert!(b >= a);
    }
}
