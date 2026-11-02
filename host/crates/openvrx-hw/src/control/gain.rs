// SPDX-License-Identifier: GPL-3.0-or-later
//! Receive gain loop (DESIGN 7.3).
//!
//! Aims the raw I/Q RMS at a band that depends on the lane format, slowly:
//! the level is smoothed in dB, must stay outside the band for a hold time
//! before a change is planned, changes are spaced by a minimum interval,
//! and a planned change is only released when the caller says the receiver
//! is in vertical blanking (a fast loop made strong transmitters flash).
//! Clipping skips the hold. Every change is logged and kept in a history
//! with its host time so the recorder can write a `gain` event.

use openvrx_core::lanes::LaneEncoding;
use openvrx_core::LaneFormat;
use serde::{Deserialize, Serialize};

/// Gain index to dB table for one board (from the bench rig B3), or the
/// linear fallback.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GainTable {
    /// dB for every index, non-decreasing.
    pub db: Vec<f32>,
}

impl GainTable {
    /// `n` indices, `db_per_step` apart.
    pub fn linear(n: usize, db_per_step: f32) -> GainTable {
        GainTable {
            db: (0..n).map(|k| k as f32 * db_per_step).collect(),
        }
    }

    pub fn max_index(&self) -> u32 {
        self.db.len().saturating_sub(1) as u32
    }

    /// dB at an index (clamped).
    pub fn db_at(&self, index: u32) -> f32 {
        let k = (index as usize).min(self.db.len().saturating_sub(1));
        self.db.get(k).copied().unwrap_or(0.0)
    }

    /// dB at a fractional index (linear interpolation).
    pub fn db_interp(&self, index: f32) -> f32 {
        if self.db.is_empty() {
            return 0.0;
        }
        let x = index.clamp(0.0, (self.db.len() - 1) as f32);
        let k = x.floor() as usize;
        let t = x - k as f32;
        let a = self.db[k];
        let b = self.db[(k + 1).min(self.db.len() - 1)];
        a + (b - a) * t
    }

    /// Index whose gain is closest to `db_at(from) + delta_db`.
    pub fn index_for_change(&self, from: u32, delta_db: f32) -> u32 {
        let want = self.db_at(from) + delta_db;
        (0..self.db.len() as u32)
            .min_by(|&a, &b| {
                (self.db_at(a) - want)
                    .abs()
                    .partial_cmp(&(self.db_at(b) - want).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(from)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GainConfig {
    /// Target per-component RMS band as fractions of the lane format's
    /// component full scale.
    pub target_lo: f32,
    pub target_hi: f32,
    /// Smoothing time constant of the level (s).
    pub tau_s: f32,
    /// Time outside the band before a change is planned (s).
    pub hold_s: f32,
    /// Minimum time between changes (s).
    pub min_interval_s: f32,
    /// Largest change in one step (indices).
    pub max_step: u32,
    pub min_gain: u32,
    pub max_gain: u32,
    /// Fraction of clipped samples that triggers an immediate step down.
    pub clip_fraction: f32,
}

impl GainConfig {
    /// Band for a lane format; None for angle-only formats (no magnitude).
    ///
    /// Contiguous I/Q lanes: 0.31..0.57 of full scale (RMS 40-72 on the
    /// 7-bit lanes' +-127 scale). Folded windows keep more headroom so
    /// little folds; the coarse 4-bit top slice sits a little higher to
    /// use its few levels.
    pub fn for_lanes(f: &LaneFormat) -> Option<GainConfig> {
        let (lo, hi) = match &f.encoding {
            LaneEncoding::Angle { .. } => return None,
            LaneEncoding::Iq { i, .. } => {
                let contiguous = i.adc_bits.windows(2).all(|w| w[0] == w[1] + 1);
                if !contiguous {
                    (0.22, 0.42)
                } else if i.width() <= 4 {
                    (0.35, 0.60)
                } else {
                    (0.31, 0.57)
                }
            }
        };
        Some(GainConfig {
            target_lo: lo,
            target_hi: hi,
            tau_s: 0.5,
            hold_s: 0.3,
            min_interval_s: 0.6,
            max_step: 4,
            min_gain: 0,
            max_gain: 77,
            clip_fraction: 0.01,
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum GainReason {
    TooLow,
    TooHigh,
    Clipping,
}

/// One applied gain change.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GainChange {
    pub from: u32,
    pub to: u32,
    pub reason: GainReason,
    /// Smoothed RMS as a fraction of full scale when planned.
    pub level: f32,
    pub host_ns: u64,
}

/// The gain controller. Feed it measurements; poll it at every vertical
/// blanking interval (or any time, with `in_vblank` false).
#[derive(Clone, Debug)]
pub struct GainLoop {
    cfg: GainConfig,
    table: GainTable,
    full_scale: f32,
    gain: u32,
    ema_db: Option<f32>,
    last_ns: u64,
    out_since: Option<(u64, i8)>,
    last_change_ns: Option<u64>,
    pending: Option<(u32, GainReason, f32)>,
    history: Vec<GainChange>,
}

fn db(x: f32) -> f32 {
    20.0 * x.max(1e-6).log10()
}

impl GainLoop {
    pub fn new(cfg: GainConfig, lanes: &LaneFormat, table: GainTable, initial_gain: u32) -> GainLoop {
        GainLoop {
            full_scale: lanes.component_full_scale(),
            cfg,
            table,
            gain: initial_gain,
            ema_db: None,
            last_ns: 0,
            out_since: None,
            last_change_ns: None,
            pending: None,
            history: Vec::new(),
        }
    }

    pub fn gain(&self) -> u32 {
        self.gain
    }

    /// Smoothed level as a fraction of full scale.
    pub fn level(&self) -> Option<f32> {
        self.ema_db.map(|d| 10f32.powf(d / 20.0))
    }

    /// The planned change, if any (waiting for vertical blanking).
    pub fn pending(&self) -> Option<u32> {
        self.pending.map(|p| p.0)
    }

    /// Applied changes, oldest first (bounded to the last 1000).
    pub fn history(&self) -> &[GainChange] {
        &self.history
    }

    /// The gain was changed elsewhere (user, scan): restart smoothing.
    pub fn set_gain(&mut self, g: u32) {
        self.gain = g;
        self.ema_db = None;
        self.out_since = None;
        self.pending = None;
    }

    /// Feeds one measurement: per-component RMS in ADC units (DC removed)
    /// and the fraction of samples at full scale.
    pub fn measure(&mut self, now_ns: u64, rms: f32, clipped_fraction: f32) {
        let x = db(rms / self.full_scale);
        let dt = if self.last_ns == 0 {
            0.0
        } else {
            (now_ns.saturating_sub(self.last_ns)) as f32 * 1e-9
        };
        self.last_ns = now_ns;
        let ema = match self.ema_db {
            None => x,
            Some(e) => {
                let a = 1.0 - (-dt / self.cfg.tau_s.max(1e-3)).exp();
                e + a * (x - e)
            }
        };
        self.ema_db = Some(ema);
        let lo = db(self.cfg.target_lo);
        let hi = db(self.cfg.target_hi);
        let centre = 0.5 * (lo + hi);
        let level = 10f32.powf(ema / 20.0);
        if clipped_fraction > self.cfg.clip_fraction {
            let delta = (centre - ema).min(-6.0);
            self.plan(delta, GainReason::Clipping, level);
            return;
        }
        let dir: i8 = if ema > hi {
            1
        } else if ema < lo {
            -1
        } else {
            0
        };
        if dir == 0 {
            self.out_since = None;
            // Back inside the band: drop a plan that was not applied yet.
            if self.pending.is_some_and(|p| p.1 != GainReason::Clipping) {
                self.pending = None;
            }
            return;
        }
        match self.out_since {
            Some((_, d)) if d == dir => {}
            _ => self.out_since = Some((now_ns, dir)),
        }
        let since = self.out_since.expect("set").0;
        if (now_ns - since) as f32 * 1e-9 >= self.cfg.hold_s {
            let reason = if dir > 0 {
                GainReason::TooHigh
            } else {
                GainReason::TooLow
            };
            self.plan(centre - ema, reason, level);
        }
    }

    fn plan(&mut self, delta_db: f32, reason: GainReason, level: f32) {
        let mut to = self.table.index_for_change(self.gain, delta_db);
        let max = self.cfg.max_gain.min(self.table.max_index());
        to = to.clamp(self.cfg.min_gain, max);
        let step = self.cfg.max_step.max(1);
        to = to.clamp(self.gain.saturating_sub(step), self.gain + step);
        // Always move at least one index in the wanted direction.
        if to == self.gain {
            to = if delta_db > 0.0 {
                (self.gain + 1).min(max)
            } else {
                self.gain.saturating_sub(1).max(self.cfg.min_gain)
            };
        }
        self.pending = (to != self.gain).then_some((to, reason, level));
    }

    /// Releases a planned change when in vertical blanking and the minimum
    /// interval has passed. The caller sends `gain <to>` to the C5.
    pub fn poll(&mut self, now_ns: u64, in_vblank: bool) -> Option<GainChange> {
        let (to, reason, level) = self.pending?;
        if !in_vblank {
            return None;
        }
        let min = match reason {
            GainReason::Clipping => self.cfg.min_interval_s * 0.5,
            _ => self.cfg.min_interval_s,
        };
        if let Some(t) = self.last_change_ns {
            if (now_ns.saturating_sub(t)) as f32 * 1e-9 < min {
                return None;
            }
        }
        let from = self.gain;
        let expected_db = self.table.db_at(to) - self.table.db_at(from);
        self.gain = to;
        self.pending = None;
        self.out_since = None;
        self.last_change_ns = Some(now_ns);
        if let Some(e) = self.ema_db.as_mut() {
            *e += expected_db;
        }
        let change = GainChange {
            from,
            to,
            reason,
            level,
            host_ns: now_ns,
        };
        log::info!("gain {from} -> {to} ({reason:?}, level {:.2} of full scale)", level);
        if self.history.len() >= 1000 {
            self.history.remove(0);
        }
        self.history.push(change.clone());
        Some(change)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::LanePreset;

    const MS: u64 = 1_000_000;

    /// Plant: RMS = path loss * 10^(gain dB / 20), slowly varying input.
    fn simulate(input_db: impl Fn(f64) -> f32, seconds: f64, start_gain: u32) -> (GainLoop, Vec<f32>) {
        let f = LanePreset::I7Q7.format();
        let cfg = GainConfig::for_lanes(&f).unwrap();
        let table = GainTable::linear(78, 0.5);
        let mut g = GainLoop::new(cfg, &f, table.clone(), start_gain);
        let mut levels = Vec::new();
        let field_ns = 16_683_000u64;
        let mut t = MS;
        while (t as f64) < seconds * 1e9 {
            let rms_db = input_db(t as f64 * 1e-9) + table.db_at(g.gain());
            let rms = 10f32.powf(rms_db / 20.0);
            let clipped = if rms > 0.9 * f.component_full_scale() {
                0.05
            } else {
                0.0
            };
            g.measure(t, rms, clipped);
            g.poll(t + 1000, true);
            levels.push(rms / f.component_full_scale());
            t += field_ns;
        }
        (g, levels)
    }

    #[test]
    fn converges_into_band_from_low() {
        // At gain 20 the level is 0.03 of full scale.
        let (g, levels) = simulate(|_| db(0.03 * 512.0) - 10.0, 20.0, 20);
        let last = *levels.last().unwrap();
        assert!((0.31..=0.57).contains(&last), "level {last}, gain {}", g.gain());
        // Slow: never more than 4 indices per change, changes spaced.
        for w in g.history().windows(2) {
            assert!(w[1].host_ns - w[0].host_ns >= 600 * MS);
        }
        assert!(g.history().iter().all(|c| c.to.abs_diff(c.from) <= 4));
    }

    #[test]
    fn clipping_steps_down_fast_and_band_is_quiet() {
        let (g, levels) = simulate(|_| db(512.0) - 30.0, 10.0, 77);
        assert!(g.history()[0].reason == GainReason::Clipping);
        let last = *levels.last().unwrap();
        assert!((0.31..=0.57).contains(&last), "{last}");
        // Once inside the band with a constant input, nothing changes.
        let n = g.history().len();
        let (g2, _) = simulate(|_| db(0.44 * 512.0) - 32.0, 10.0, 64);
        assert!(g2.history().is_empty(), "{:?}", g2.history());
        assert!(n >= 1);
    }

    #[test]
    fn hysteresis_ignores_short_excursions() {
        // A 150 ms burst far above the band does not move the gain.
        let (g, _) = simulate(
            |t| {
                if (2.0..2.15).contains(&t) {
                    db(0.8 * 512.0) - 32.0
                } else {
                    db(0.44 * 512.0) - 32.0
                }
            },
            5.0,
            64,
        );
        assert!(g.history().is_empty(), "{:?}", g.history());
    }

    #[test]
    fn only_in_vertical_blanking() {
        let f = LanePreset::I8Q8.format();
        let mut g = GainLoop::new(GainConfig::for_lanes(&f).unwrap(), &f, GainTable::linear(78, 1.0), 30);
        for k in 0..60u64 {
            g.measure(k * 16 * MS + 1, 10.0, 0.0);
        }
        assert!(g.pending().is_some());
        assert!(g.poll(2000 * MS, false).is_none());
        let c = g.poll(2000 * MS, true).unwrap();
        assert!(c.to > c.from && c.reason == GainReason::TooLow);
        assert!(GainConfig::for_lanes(&LanePreset::Angle8.format()).is_none());
    }

    #[test]
    fn table_lookup() {
        let t = GainTable {
            db: vec![0.0, 1.0, 3.0, 6.0, 10.0],
        };
        assert_eq!(t.index_for_change(1, 4.5), 3);
        assert_eq!(t.index_for_change(4, -100.0), 0);
        assert!((t.db_interp(2.5) - 4.5).abs() < 1e-6);
    }
}
