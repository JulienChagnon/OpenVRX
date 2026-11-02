// SPDX-License-Identifier: GPL-3.0-or-later
//! Watch-mode promotion and demotion for three or more receivers
//! (DESIGN 7.4).
//!
//! At most `max_full` receivers stream at full rate; the others stream in
//! watch mode (one packet in N) and the DSP still estimates their quality
//! from the bursts. A watcher whose smoothed quality beats the weakest full
//! receiver by a margin for about 100 ms is promoted and that full receiver
//! demoted, at the start of the next field, then nothing moves for a
//! cooldown. A full receiver that stops delivering is replaced at once.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PromoteConfig {
    pub max_full: usize,
    /// Quality margin (dB of C/N) a watcher needs.
    pub margin_db: f32,
    /// How long the margin must hold.
    pub hold_ms: u64,
    /// No switch within this long of the previous one.
    pub cooldown_ms: u64,
    /// Smoothing of full receivers' and watchers' quality.
    pub tau_full_ms: f32,
    pub tau_watch_ms: f32,
    /// A receiver without a measurement for this long counts as absent.
    pub stale_ms: u64,
}

impl Default for PromoteConfig {
    fn default() -> Self {
        PromoteConfig {
            max_full: 2,
            margin_db: 3.0,
            hold_ms: 100,
            cooldown_ms: 500,
            tau_full_ms: 20.0,
            tau_watch_ms: 40.0,
            stale_ms: 250,
        }
    }
}

/// A role change to apply now (at a field start).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Switch {
    pub promote: usize,
    /// None when a full slot was free.
    pub demote: Option<usize>,
    pub host_ns: u64,
}

#[derive(Clone, Debug)]
struct RxState {
    full: bool,
    ema: Option<f32>,
    last_ns: u64,
    present: bool,
}

#[derive(Clone, Debug)]
pub struct WatchManager {
    cfg: PromoteConfig,
    rx: Vec<RxState>,
    candidate: Option<(usize, Option<usize>, u64)>,
    pending: Option<(usize, Option<usize>)>,
    last_switch_ns: Option<u64>,
}

impl WatchManager {
    /// `n` receivers; `initial_full` stream at full rate.
    pub fn new(cfg: PromoteConfig, n: usize, initial_full: &[usize]) -> WatchManager {
        WatchManager {
            rx: (0..n)
                .map(|k| RxState {
                    full: initial_full.contains(&k),
                    ema: None,
                    last_ns: 0,
                    present: true,
                })
                .collect(),
            cfg,
            candidate: None,
            pending: None,
            last_switch_ns: None,
        }
    }

    /// True per receiver when it streams at full rate.
    pub fn roles(&self) -> Vec<bool> {
        self.rx.iter().map(|r| r.full).collect()
    }

    pub fn quality(&self, rx: usize) -> Option<f32> {
        self.rx.get(rx).and_then(|r| r.ema)
    }

    /// A receiver appeared or vanished (USB, C5 reset).
    pub fn set_present(&mut self, rx: usize, present: bool, now_ns: u64) {
        if let Some(r) = self.rx.get_mut(rx) {
            r.present = present;
            if !present {
                r.ema = None;
            }
        }
        self.evaluate(now_ns);
    }

    /// Feeds a quality measurement (C/N in dB, or any dB-like score).
    pub fn update(&mut self, rx: usize, now_ns: u64, quality_db: f32) {
        let tau = if self.rx.get(rx).is_some_and(|r| r.full) {
            self.cfg.tau_full_ms
        } else {
            self.cfg.tau_watch_ms
        };
        if let Some(r) = self.rx.get_mut(rx) {
            let dt = if r.last_ns == 0 {
                0.0
            } else {
                (now_ns.saturating_sub(r.last_ns)) as f32 * 1e-6
            };
            r.ema = Some(match r.ema {
                None => quality_db,
                Some(e) => e + (1.0 - (-dt / tau.max(1e-3)).exp()) * (quality_db - e),
            });
            r.last_ns = now_ns;
        }
        self.evaluate(now_ns);
    }

    fn effective(&self, k: usize, now_ns: u64) -> Option<f32> {
        let r = &self.rx[k];
        if !r.present || now_ns.saturating_sub(r.last_ns) > self.cfg.stale_ms * 1_000_000 {
            return None;
        }
        r.ema
    }

    fn evaluate(&mut self, now_ns: u64) {
        let best_watcher = (0..self.rx.len())
            .filter(|&k| !self.rx[k].full)
            .filter_map(|k| self.effective(k, now_ns).map(|q| (k, q)))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let Some((w, wq)) = best_watcher else {
            self.candidate = None;
            return;
        };
        let full: Vec<usize> = (0..self.rx.len()).filter(|&k| self.rx[k].full).collect();
        // A free slot or a dead full receiver: switch at the next field.
        if full.len() < self.cfg.max_full {
            self.pending = Some((w, None));
            return;
        }
        if let Some(&dead) = full.iter().find(|&&k| self.effective(k, now_ns).is_none()) {
            self.pending = Some((w, Some(dead)));
            return;
        }
        let (weak, fq) = full
            .iter()
            .map(|&k| (k, self.effective(k, now_ns).unwrap_or(f32::NEG_INFINITY)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .expect("max_full > 0");
        let cooling = self
            .last_switch_ns
            .is_some_and(|t| now_ns.saturating_sub(t) < self.cfg.cooldown_ms * 1_000_000);
        if wq > fq + self.cfg.margin_db && !cooling {
            match self.candidate {
                Some((cw, cd, since)) if cw == w && cd == Some(weak) => {
                    if now_ns - since >= self.cfg.hold_ms * 1_000_000 {
                        self.pending = Some((w, Some(weak)));
                    }
                }
                _ => self.candidate = Some((w, Some(weak), now_ns)),
            }
        } else {
            self.candidate = None;
            // A pending margin switch no longer justified is dropped.
            if let Some((_, Some(d))) = self.pending {
                if self.effective(d, now_ns).is_some() {
                    self.pending = None;
                }
            }
        }
    }

    /// Call at the start of every field; returns the switch to apply now
    /// (send `SET_WATCH` to the two FX2s).
    pub fn on_field_start(&mut self, now_ns: u64) -> Option<Switch> {
        let (p, d) = self.pending.take()?;
        self.rx[p].full = true;
        if let Some(d) = d {
            self.rx[d].full = false;
        }
        self.candidate = None;
        self.last_switch_ns = Some(now_ns);
        log::info!(
            "receiver {p} promoted to full stream{}",
            d.map(|d| format!(", receiver {d} to watch")).unwrap_or_default()
        );
        Some(Switch {
            promote: p,
            demote: d,
            host_ns: now_ns,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    #[test]
    fn promotes_after_hold_at_field_start() {
        let mut m = WatchManager::new(PromoteConfig::default(), 3, &[0, 1]);
        let mut t = MS;
        let mut switches = Vec::new();
        for k in 0..60 {
            // Receiver 2 (watching) becomes much better after 200 ms.
            let q2 = if k >= 10 { 25.0 } else { 10.0 };
            m.update(0, t, 18.0);
            m.update(1, t, 12.0);
            m.update(2, t, q2);
            if let Some(s) = m.on_field_start(t) {
                switches.push((k, s));
            }
            t += 20 * MS;
        }
        assert_eq!(switches.len(), 1, "{switches:?}");
        let (k, s) = &switches[0];
        assert_eq!((s.promote, s.demote), (2, Some(1)));
        // Hold of ~100 ms after the change at k = 10 (20 ms steps).
        assert!((14..=18).contains(k), "switched at step {k}");
        assert_eq!(m.roles(), vec![true, false, true]);
    }

    #[test]
    fn small_margin_or_short_spike_does_nothing() {
        let mut m = WatchManager::new(PromoteConfig::default(), 3, &[0, 1]);
        let mut t = MS;
        for k in 0..100 {
            m.update(0, t, 15.0);
            m.update(1, t, 14.0);
            let q = if k == 50 { 30.0 } else { 16.0 };
            m.update(2, t, q);
            assert!(m.on_field_start(t).is_none(), "step {k}");
            t += 20 * MS;
        }
    }

    #[test]
    fn dead_full_receiver_is_replaced_at_once() {
        let mut m = WatchManager::new(PromoteConfig::default(), 3, &[0, 1]);
        m.update(0, MS, 20.0);
        m.update(1, MS, 20.0);
        m.update(2, MS, 5.0);
        m.set_present(0, false, 2 * MS);
        let s = m.on_field_start(3 * MS).unwrap();
        assert_eq!((s.promote, s.demote), (2, Some(0)));
        // Cooldown: no immediate swap back even with a big margin.
        m.set_present(0, true, 4 * MS);
        m.update(0, 5 * MS, 40.0);
        m.update(1, 5 * MS, 20.0);
        m.update(2, 5 * MS, 5.0);
        assert!(m.on_field_start(6 * MS).is_none());
    }
}
