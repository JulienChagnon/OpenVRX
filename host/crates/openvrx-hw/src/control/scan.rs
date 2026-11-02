// SPDX-License-Identifier: GPL-3.0-or-later
//! Channel table and band scanner (DESIGN 7.3).
//!
//! The band/channel table matches C5VRX's `main/rf.c` (by Twotoz and the
//! C5VRX contributors, GPL-3.0): bands R, A, B, E, F, L, eight channels
//! each. The scanner tunes each channel, lets it settle, collects metrics
//! through [`ScanIo`] for a dwell time, ranks the channels and, with
//! several receivers, splits the band so they scan in parallel.

use crate::error::Result;
use serde::{Deserialize, Serialize};
use std::time::Duration;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Band {
    /// Raceband.
    R,
    /// Boscam A.
    A,
    /// Boscam B.
    B,
    /// Boscam E.
    E,
    /// Fatshark F.
    F,
    /// Lowband L.
    L,
}

impl Band {
    pub const ALL: [Band; 6] = [Band::R, Band::A, Band::B, Band::E, Band::F, Band::L];

    pub fn letter(self) -> char {
        match self {
            Band::R => 'R',
            Band::A => 'A',
            Band::B => 'B',
            Band::E => 'E',
            Band::F => 'F',
            Band::L => 'L',
        }
    }

    pub fn long_name(self) -> &'static str {
        match self {
            Band::R => "RACEBAND R",
            Band::A => "BOSCAM A",
            Band::B => "BOSCAM B",
            Band::E => "BOSCAM E",
            Band::F => "FATSHARK F",
            Band::L => "LOWBAND L",
        }
    }

    /// Centre frequencies of channels 1..=8 in MHz.
    pub fn freqs(self) -> [u16; 8] {
        match self {
            Band::R => [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917],
            Band::A => [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725],
            Band::B => [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],
            Band::E => [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945],
            Band::F => [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],
            Band::L => [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621],
        }
    }
}

/// One channel.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Channel {
    pub band: Band,
    /// 1..=8.
    pub number: u8,
    pub mhz: u16,
}

impl Channel {
    pub fn name(&self) -> String {
        format!("{}{}", self.band.letter(), self.number)
    }
}

/// All 48 channels, band by band.
pub fn all_channels() -> Vec<Channel> {
    channels_in(&Band::ALL)
}

pub fn channels_in(bands: &[Band]) -> Vec<Channel> {
    bands
        .iter()
        .flat_map(|&b| {
            b.freqs().into_iter().enumerate().map(move |(k, mhz)| Channel {
                band: b,
                number: k as u8 + 1,
                mhz,
            })
        })
        .collect()
}

/// Looks up "R3", "f8", ...
pub fn channel(name: &str) -> Option<Channel> {
    let mut c = name.trim().chars();
    let letter = c.next()?.to_ascii_uppercase();
    let n: u8 = c.as_str().parse().ok()?;
    let band = Band::ALL.into_iter().find(|b| b.letter() == letter)?;
    (1..=8).contains(&n).then(|| Channel {
        band,
        number: n,
        mhz: band.freqs()[n as usize - 1],
    })
}

/// Channels at a frequency (R7 and F8 share 5880 MHz).
pub fn channels_at(mhz: u16) -> Vec<Channel> {
    all_channels().into_iter().filter(|c| c.mhz == mhz).collect()
}

/// Metrics collected for one channel during its dwell.
#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ChannelMetrics {
    /// Raw I/Q power, gain-normalised, dB.
    pub power_db: f32,
    /// Envelope C/N estimate, dB.
    pub cn_db: f32,
    /// 0..1 sync detection quality (fraction of lines with sync).
    pub sync_score: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScanConfig {
    /// Wait after tuning before measuring.
    pub settle: Duration,
    /// Measurement time per channel.
    pub dwell: Duration,
    /// Sync score above which a channel carries video.
    pub video_sync: f32,
    /// Channels closer than this are the same transmitter.
    pub same_signal_mhz: u16,
}

impl Default for ScanConfig {
    fn default() -> Self {
        ScanConfig {
            settle: Duration::from_millis(30),
            dwell: Duration::from_millis(60),
            video_sync: 0.5,
            same_signal_mhz: 6,
        }
    }
}

/// What the scanner needs: tune a receiver, wait, measure receivers.
pub trait ScanIo {
    fn tune(&mut self, rx: usize, mhz: u16) -> Result<()>;
    fn wait(&mut self, d: Duration);
    /// Collects metrics on every listed receiver for `dwell` (in parallel).
    fn measure(&mut self, rxs: &[usize], dwell: Duration) -> Result<Vec<ChannelMetrics>>;
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScanResult {
    /// Channels at this frequency (first is the requested one).
    pub channels: Vec<String>,
    pub mhz: u16,
    pub rx: usize,
    pub metrics: Option<ChannelMetrics>,
    pub video: bool,
    pub score: f32,
    /// A stronger result within `same_signal_mhz` (same transmitter).
    pub alias_of: Option<u16>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScanReport {
    /// Best first.
    pub ranked: Vec<ScanResult>,
}

impl ScanReport {
    /// Distinct transmitters with video, best first.
    pub fn transmitters(&self) -> Vec<&ScanResult> {
        self.ranked.iter().filter(|r| r.video && r.alias_of.is_none()).collect()
    }
}

/// Ranking score: video presence first, then C/N, sync quality, power.
pub fn score(m: &ChannelMetrics, video_sync: f32) -> f32 {
    let video = if m.sync_score >= video_sync { 100.0 } else { 0.0 };
    let cn = if m.cn_db.is_finite() { m.cn_db } else { -20.0 };
    video + 2.0 * cn + 20.0 * m.sync_score + 0.1 * m.power_db
}

/// Splits unique frequencies (sorted) into contiguous, balanced chunks,
/// one per receiver, so each receiver retunes in small steps.
pub fn split_band(channels: &[Channel], receivers: usize) -> Vec<Vec<u16>> {
    let mut f: Vec<u16> = channels.iter().map(|c| c.mhz).collect();
    f.sort_unstable();
    f.dedup();
    let r = receivers.max(1);
    let base = f.len() / r;
    let extra = f.len() % r;
    let mut out = Vec::with_capacity(r);
    let mut k = 0;
    for i in 0..r {
        let n = base + usize::from(i < extra);
        out.push(f[k..k + n].to_vec());
        k += n;
    }
    out
}

/// Scans `channels` with `receivers` receivers in lockstep and ranks them.
pub fn run_scan(channels: &[Channel], receivers: usize, cfg: &ScanConfig, io: &mut dyn ScanIo) -> ScanReport {
    let plan = split_band(channels, receivers);
    let steps = plan.iter().map(Vec::len).max().unwrap_or(0);
    let names = |mhz: u16| -> Vec<String> {
        let mut v: Vec<String> = channels.iter().filter(|c| c.mhz == mhz).map(|c| c.name()).collect();
        v.dedup();
        v
    };
    let mut results = Vec::new();
    for step in 0..steps {
        let mut active = Vec::new();
        for (rx, list) in plan.iter().enumerate() {
            if let Some(&mhz) = list.get(step) {
                match io.tune(rx, mhz) {
                    Ok(()) => active.push((rx, mhz)),
                    Err(e) => results.push(ScanResult {
                        channels: names(mhz),
                        mhz,
                        rx,
                        metrics: None,
                        video: false,
                        score: f32::NEG_INFINITY,
                        alias_of: None,
                        error: Some(e.to_string()),
                    }),
                }
            }
        }
        if active.is_empty() {
            continue;
        }
        io.wait(cfg.settle);
        let rxs: Vec<usize> = active.iter().map(|a| a.0).collect();
        match io.measure(&rxs, cfg.dwell) {
            Ok(ms) => {
                for ((rx, mhz), m) in active.into_iter().zip(ms) {
                    results.push(ScanResult {
                        channels: names(mhz),
                        mhz,
                        rx,
                        video: m.sync_score >= cfg.video_sync,
                        score: score(&m, cfg.video_sync),
                        metrics: Some(m),
                        alias_of: None,
                        error: None,
                    });
                }
            }
            Err(e) => {
                for (rx, mhz) in active {
                    results.push(ScanResult {
                        channels: names(mhz),
                        mhz,
                        rx,
                        metrics: None,
                        video: false,
                        score: f32::NEG_INFINITY,
                        alias_of: None,
                        error: Some(e.to_string()),
                    });
                }
            }
        }
    }
    results.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    // Mark weaker neighbours of a video channel as the same transmitter.
    for i in 0..results.len() {
        if !results[i].video || results[i].alias_of.is_some() {
            continue;
        }
        let f = results[i].mhz;
        for r in results.iter_mut().skip(i + 1) {
            if r.mhz.abs_diff(f) <= cfg.same_signal_mhz && r.alias_of.is_none() {
                r.alias_of = Some(f);
            }
        }
    }
    ScanReport { ranked: results }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_matches_c5vrx() {
        assert_eq!(all_channels().len(), 48);
        assert_eq!(channel("R1").unwrap().mhz, 5658);
        assert_eq!(channel("a1").unwrap().mhz, 5865);
        assert_eq!(channel("E8").unwrap().mhz, 5945);
        assert_eq!(channel("L8").unwrap().mhz, 5621);
        assert!(channel("R9").is_none() && channel("X1").is_none());
        let at = channels_at(5880);
        assert_eq!(at.iter().map(|c| c.name()).collect::<Vec<_>>(), vec!["R7", "F8"]);
        assert_eq!(Band::F.long_name(), "FATSHARK F");
    }

    #[test]
    fn split_is_balanced_and_contiguous() {
        let p = split_band(&all_channels(), 3);
        let total: usize = p.iter().map(Vec::len).sum();
        let unique = {
            let mut f: Vec<u16> = all_channels().iter().map(|c| c.mhz).collect();
            f.sort_unstable();
            f.dedup();
            f.len()
        };
        assert_eq!(total, unique);
        assert!(p.iter().all(|c| c.len() >= unique / 3));
        assert!(p[0].last().unwrap() < p[1].first().unwrap());
    }

    struct FakeAir {
        tuned: Vec<u16>,
        /// (MHz, C/N dB) of transmitters.
        tx: Vec<(u16, f32)>,
        waits: Duration,
    }

    impl ScanIo for FakeAir {
        fn tune(&mut self, rx: usize, mhz: u16) -> Result<()> {
            if self.tuned.len() <= rx {
                self.tuned.resize(rx + 1, 0);
            }
            self.tuned[rx] = mhz;
            Ok(())
        }
        fn wait(&mut self, d: Duration) {
            self.waits += d;
        }
        fn measure(&mut self, rxs: &[usize], dwell: Duration) -> Result<Vec<ChannelMetrics>> {
            self.waits += dwell;
            Ok(rxs
                .iter()
                .map(|&rx| {
                    let f = self.tuned[rx];
                    // C/N falls 6 dB per MHz of detuning.
                    let best = self
                        .tx
                        .iter()
                        .map(|&(m, cn)| cn - 6.0 * m.abs_diff(f) as f32)
                        .fold(-10.0f32, f32::max);
                    ChannelMetrics {
                        power_db: best,
                        cn_db: best,
                        sync_score: if best > 8.0 { 0.95 } else { 0.0 },
                    }
                })
                .collect())
        }
    }

    #[test]
    fn finds_and_ranks_transmitters_in_parallel() {
        let mut air = FakeAir {
            tuned: vec![],
            tx: vec![(5732, 25.0), (5800, 18.0)],
            waits: Duration::ZERO,
        };
        let cfg = ScanConfig::default();
        let report = run_scan(&channels_in(&[Band::R, Band::B, Band::F]), 2, &cfg, &mut air);
        let tx = report.transmitters();
        assert_eq!(tx.len(), 2, "{:?}", report.ranked.iter().take(5).collect::<Vec<_>>());
        assert_eq!(tx[0].mhz, 5732);
        assert!(tx[0].channels.contains(&"R3".to_string()));
        assert_eq!(tx[1].mhz, 5800);
        // B1 (5733) is the same transmitter as R3.
        let b1 = report.ranked.iter().find(|r| r.mhz == 5733).unwrap();
        assert_eq!(b1.alias_of, Some(5732));
        // Two receivers: about half the dwell time of one.
        let steps = split_band(&channels_in(&[Band::R, Band::B, Band::F]), 2)[0].len() as u32;
        assert_eq!(air.waits, (cfg.settle + cfg.dwell) * steps);
    }
}
