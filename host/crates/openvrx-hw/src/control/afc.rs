// SPDX-License-Identifier: GPL-3.0-or-later
//! Automatic frequency control: when to retune the C5.
//!
//! The DSP removes small carrier offsets digitally. This controller only
//! decides when the residual offset is large enough to retune the radio
//! (two C5 crystals differ by tens of ppm, about 100 kHz at 5.8 GHz, and
//! transmitters drift with temperature). It acts on a smoothed offset, only
//! while the carrier has good C/N and sync is locked, waits for the offset
//! to persist, applies at vertical blanking, then lets the new tuning
//! settle and disarms until the offset has come back under a release
//! threshold (hysteresis).
//!
//! Sign: `offset_hz` is where the carrier sits relative to the frequency
//! it should be at (positive: carrier above), so the C5 is retuned up by
//! that amount.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AfcConfig {
    /// Retune when the smoothed offset exceeds this.
    pub threshold_hz: f64,
    /// Re-arm once the offset is back under this.
    pub release_hz: f64,
    /// Ignore measurements below this C/N.
    pub min_cn_db: f32,
    /// Offset must persist this long.
    pub hold_s: f64,
    /// Ignore measurements this long after a retune.
    pub settle_s: f64,
    pub tau_s: f64,
    /// Largest kHz offset passed with `tune`; beyond it the MHz moves.
    pub max_offset_khz: i32,
    /// Corrections are rounded to this many kHz.
    pub step_khz: i32,
}

impl Default for AfcConfig {
    fn default() -> Self {
        AfcConfig {
            threshold_hz: 400e3,
            release_hz: 150e3,
            min_cn_db: 12.0,
            hold_s: 1.0,
            settle_s: 2.0,
            tau_s: 0.3,
            max_offset_khz: 2000,
            step_khz: 10,
        }
    }
}

/// A retune decision: send `tune <mhz> <offset_khz>`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Retune {
    pub mhz: u32,
    pub offset_khz: i32,
    pub correction_hz: f64,
    pub host_ns: u64,
}

#[derive(Clone, Debug)]
pub struct Afc {
    cfg: AfcConfig,
    mhz: u32,
    offset_khz: i32,
    ema: Option<f64>,
    last_ns: u64,
    over_since: Option<u64>,
    settle_until: u64,
    armed: bool,
    disarmed_at: u64,
    pending: Option<f64>,
}

impl Afc {
    pub fn new(cfg: AfcConfig, mhz: u32, offset_khz: i32) -> Afc {
        Afc {
            cfg,
            mhz,
            offset_khz,
            ema: None,
            last_ns: 0,
            over_since: None,
            settle_until: 0,
            armed: true,
            disarmed_at: 0,
            pending: None,
        }
    }

    /// The tuning was changed elsewhere (channel change, scan).
    pub fn set_tuning(&mut self, mhz: u32, offset_khz: i32, now_ns: u64) {
        self.mhz = mhz;
        self.offset_khz = offset_khz;
        self.ema = None;
        self.over_since = None;
        self.pending = None;
        self.armed = true;
        self.settle_until = now_ns + (self.cfg.settle_s * 1e9) as u64;
    }

    pub fn tuning(&self) -> (u32, i32) {
        (self.mhz, self.offset_khz)
    }

    pub fn smoothed_offset_hz(&self) -> Option<f64> {
        self.ema
    }

    /// Feeds one measurement (per field or per few lines).
    pub fn measure(&mut self, now_ns: u64, offset_hz: f64, cn_db: f32, locked: bool) {
        if now_ns < self.settle_until
            || !locked
            || cn_db.is_nan()
            || cn_db < self.cfg.min_cn_db
            || !offset_hz.is_finite()
        {
            self.over_since = None;
            if now_ns < self.settle_until {
                self.ema = None;
                self.last_ns = 0;
            }
            return;
        }
        let dt = if self.last_ns == 0 {
            0.0
        } else {
            (now_ns - self.last_ns) as f64 * 1e-9
        };
        self.last_ns = now_ns;
        let e = match self.ema {
            None => offset_hz,
            Some(e) => e + (1.0 - (-dt / self.cfg.tau_s.max(1e-3)).exp()) * (offset_hz - e),
        };
        self.ema = Some(e);
        if !self.armed {
            let stale = (now_ns - self.disarmed_at) as f64 * 1e-9 > 3.0 * self.cfg.settle_s;
            if e.abs() < self.cfg.release_hz || stale {
                self.armed = true;
            } else {
                return;
            }
        }
        if e.abs() > self.cfg.threshold_hz {
            let since = *self.over_since.get_or_insert(now_ns);
            if (now_ns - since) as f64 * 1e-9 >= self.cfg.hold_s {
                self.pending = Some(e);
            }
        } else {
            self.over_since = None;
            self.pending = None;
        }
    }

    /// Releases a planned retune at vertical blanking.
    pub fn poll(&mut self, now_ns: u64, in_vblank: bool) -> Option<Retune> {
        let corr = self.pending?;
        if !in_vblank {
            return None;
        }
        let step = self.cfg.step_khz.max(1) as f64;
        let corr_khz = ((corr / 1e3) / step).round() as i64 * step as i64;
        let total_khz = self.mhz as i64 * 1000 + self.offset_khz as i64 + corr_khz;
        let mut mhz = self.mhz as i64;
        let mut off = total_khz - mhz * 1000;
        if off.abs() > self.cfg.max_offset_khz as i64 {
            mhz = (total_khz as f64 / 1000.0).round() as i64;
            off = total_khz - mhz * 1000;
        }
        let r = Retune {
            mhz: mhz as u32,
            offset_khz: off as i32,
            correction_hz: corr,
            host_ns: now_ns,
        };
        log::info!(
            "AFC retune by {:+.0} kHz to {} MHz {:+} kHz",
            corr / 1e3,
            r.mhz,
            r.offset_khz
        );
        self.mhz = r.mhz;
        self.offset_khz = r.offset_khz;
        self.pending = None;
        self.ema = None;
        self.last_ns = 0;
        self.over_since = None;
        self.armed = false;
        self.disarmed_at = now_ns;
        self.settle_until = now_ns + (self.cfg.settle_s * 1e9) as u64;
        Some(r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIELD: u64 = 16_683_000;

    /// Transmitter at `tx_khz` absolute; the measured offset is tx minus
    /// the tuned frequency, with noise.
    fn run(afc: &mut Afc, tx_khz: f64, seconds: f64, cn: f32) -> Vec<Retune> {
        let mut out = Vec::new();
        let mut t = 1u64;
        let mut k = 0u64;
        while (t as f64) < seconds * 1e9 {
            let (m, o) = afc.tuning();
            let tuned = m as f64 * 1000.0 + o as f64;
            let noise = ((k * 7919) % 101) as f64 - 50.0;
            afc.measure(t, (tx_khz - tuned) * 1e3 + noise * 1e3, cn, true);
            if let Some(r) = afc.poll(t, k.is_multiple_of(5)) {
                out.push(r);
            }
            t += FIELD;
            k += 1;
        }
        out
    }

    #[test]
    fn retunes_once_onto_the_carrier() {
        let mut afc = Afc::new(AfcConfig::default(), 5732, 0);
        let r = run(&mut afc, 5732.0 * 1000.0 + 620.0, 10.0, 20.0);
        assert_eq!(r.len(), 1, "{r:?}");
        let (m, o) = afc.tuning();
        assert_eq!(m, 5732);
        assert!((o - 620).abs() <= 30, "{o}");
    }

    #[test]
    fn small_offsets_and_poor_cn_are_left_to_the_dsp() {
        let mut afc = Afc::new(AfcConfig::default(), 5732, 0);
        assert!(run(&mut afc, 5732.0 * 1000.0 + 250.0, 10.0, 20.0).is_empty());
        let mut afc = Afc::new(AfcConfig::default(), 5732, 0);
        assert!(run(&mut afc, 5732.0 * 1000.0 + 900.0, 10.0, 6.0).is_empty());
    }

    #[test]
    fn large_correction_moves_the_mhz() {
        let mut afc = Afc::new(
            AfcConfig {
                max_offset_khz: 500,
                ..Default::default()
            },
            5732,
            0,
        );
        let r = run(&mut afc, 5733.0 * 1000.0 + 900.0, 10.0, 20.0);
        assert!(!r.is_empty());
        let (m, o) = afc.tuning();
        assert!(m as i64 * 1000 + o as i64 >= 5_733_850, "{m} {o}");
        assert!(o.abs() <= 500);
    }
}
