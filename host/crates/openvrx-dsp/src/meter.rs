// SPDX-License-Identifier: GPL-3.0-or-later
//! A short reading of one channel for a band scan (DSP-25).
//!
//! Each channel gets a fresh receiver DSP. [`ChannelMeter::warm`] feeds the
//! settling time after a retune (the radio's own gain loop, the DC and I/Q
//! correction and the sync search all start over) without counting it;
//! [`ChannelMeter::push`] feeds the dwell, and [`ChannelMeter::reading`]
//! sums it up:
//!
//! - C/N and power: the median over the front end's short statistics
//!   windows of the dwell. A burst from another transmitter (a 5 GHz Wi-Fi
//!   packet lasts up to a few ms) moves one or two windows, not the median.
//! - Sync score: the share of the lines the dwell holds on which the DSP
//!   measured a sync pulse (not the flywheel's guess). A channel that never
//!   locks scores 0, one locked for half the dwell about 0.5, so a video
//!   carrier ranks above a strong carrier without sync (Wi-Fi, a digital
//!   VTX) even when its C/N is lower. The lines' own sync scores (depth,
//!   noise and timing together) fall with the noise long before the pulses
//!   are lost, so they are only reported, as the sync quality.

use crate::{RxDsp, RxDspConfig};
use openvrx_core::lanes::SampleFormat;
use openvrx_core::{LineFlags, LineRecord, VideoStandard};
use serde::{Deserialize, Serialize};

/// Statistics window of the meter's DSP (the live default is 100 ms, longer
/// than a scan's dwell).
pub const METER_WINDOW_MS: f32 = 10.0;

/// What one dwell saw.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChannelReading {
    /// Raw I/Q power, dB relative to 1 ADC unit squared (median over the
    /// statistics windows). Not gain-normalised: the caller knows the gain.
    pub power_db: f32,
    /// Envelope C/N in the capture bandwidth, dB (median over the windows).
    pub cn_db: f32,
    /// Share of the dwell's lines with a measured sync pulse, 0..1.
    pub sync_score: f32,
    /// Mean line sync score (`LineQuality::sync_score`) over the dwell's lines, 0..1.
    pub sync_quality: f32,
    /// Lines the DSP put out during the dwell.
    pub lines: u64,
    /// Statistics windows behind the medians.
    pub windows: u32,
    /// Standard of the dwell's lines (None: no line).
    pub standard: Option<VideoStandard>,
    /// Input samples of the dwell.
    pub samples: u64,
}

pub struct ChannelMeter {
    dsp: RxDsp,
    fs: f64,
    out: Vec<LineRecord>,
    /// Samples processed when the dwell started (None: still warming up).
    dwell_from: Option<u64>,
    /// Samples processed when the first statistics window can be complete.
    first_window: u64,
    /// Last window recorded: (power, I RMS, Q RMS) bit patterns.
    last_window: Option<(u32, u32, u32)>,
    power: Vec<f32>,
    cn: Vec<f32>,
    sync_sum: f64,
    lines: u64,
    synced: u64,
    standard: Option<VideoStandard>,
}

impl ChannelMeter {
    pub fn new(sample_format: SampleFormat) -> ChannelMeter {
        let cfg = RxDspConfig {
            sample_format,
            standard: None,
            stats_window_ms: METER_WINDOW_MS,
            // One short-lived DSP per channel: no second thread to start and stop each time.
            pipeline_thread: false,
            ..RxDspConfig::default()
        };
        let dsp = RxDsp::new(0, cfg);
        let fs = dsp.stats().sample_rate_hz;
        ChannelMeter {
            dsp,
            fs,
            out: Vec::new(),
            dwell_from: None,
            first_window: ((METER_WINDOW_MS as f64 * 1e-3 * fs) as u64).max(1024),
            last_window: None,
            power: Vec::new(),
            cn: Vec::new(),
            sync_sum: 0.0,
            lines: 0,
            synced: 0,
            standard: None,
        }
    }

    /// Sample rate of the stream (Hz).
    pub fn sample_rate(&self) -> f64 {
        self.fs
    }

    /// Input samples fed so far (warm-up and dwell).
    pub fn samples(&self) -> u64 {
        self.dsp.stats().samples_processed
    }

    /// Settling time after a retune: processed, not counted.
    pub fn warm(&mut self, bytes: &[u8]) {
        self.feed(bytes);
        // Windows finished during the warm-up are not the dwell's.
        self.last_window = self.window_key();
    }

    /// Dwell data: processed and counted.
    pub fn push(&mut self, bytes: &[u8]) {
        if self.dwell_from.is_none() {
            self.dwell_from = Some(self.samples());
            self.last_window = self.window_key();
        }
        self.feed(bytes);
        for l in &self.out {
            self.sync_sum += l.quality.sync_score.clamp(0.0, 1.0) as f64;
            self.lines += 1;
            self.synced += u64::from(!l.quality.flags.contains(LineFlags::FLYWHEEL));
            self.standard.get_or_insert(l.standard);
        }
        // A new statistics window shows as new figures; record its C/N and power.
        let key = self.window_key();
        if key.is_some() && key != self.last_window {
            self.last_window = key;
            let s = self.dsp.stats();
            if s.power_db.is_finite() {
                self.power.push(s.power_db);
            }
            if s.cn_db.is_finite() {
                self.cn.push(s.cn_db);
            }
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        self.out.clear();
        self.dsp.push_bytes(bytes, 0, &mut self.out);
    }

    /// Identity of the last finished statistics window (None before the first).
    fn window_key(&self) -> Option<(u32, u32, u32)> {
        let s = self.dsp.stats();
        (s.samples_processed >= self.first_window).then(|| (s.power_db.to_bits(), s.i_rms.to_bits(), s.q_rms.to_bits()))
    }

    pub fn reading(&self) -> ChannelReading {
        let samples = self.dwell_from.map(|f| self.samples().saturating_sub(f)).unwrap_or(0);
        let line_hz = self.standard.unwrap_or(VideoStandard::Ntsc).params().line_hz;
        let expected = samples as f64 / self.fs * line_hz;
        let share = |x: f64| if expected >= 1.0 { (x / expected).min(1.0) as f32 } else { 0.0 };
        ChannelReading {
            power_db: median(&self.power),
            cn_db: median(&self.cn),
            sync_score: share(self.synced as f64),
            sync_quality: share(self.sync_sum),
            lines: self.lines,
            windows: self.power.len() as u32,
            standard: self.standard,
            samples,
        }
    }
}

/// Middle value (mean of the two middle ones for an even count); NaN when empty.
fn median(x: &[f32]) -> f32 {
    if x.is_empty() {
        return f32::NAN;
    }
    let mut s = x.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = s.len();
    0.5 * (s[(n - 1) / 2] + s[n / 2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testsig::{SigGen, SigSpec};

    /// `settle_ms` of warm-up and `dwell_ms` of dwell from `gen`, in 1 ms pushes.
    fn read(gen: &mut SigGen, settle_ms: f64, dwell_ms: f64, fmt: &SampleFormat) -> ChannelReading {
        let mut m = ChannelMeter::new(fmt.clone());
        let per_ms = (m.sample_rate() * 1e-3) as usize;
        for _ in 0..settle_ms as usize {
            m.warm(&gen.next(per_ms).bytes);
        }
        for _ in 0..dwell_ms as usize {
            m.push(&gen.next(per_ms).bytes);
        }
        m.reading()
    }

    fn spec(cn_db: f64, seed: u64) -> SigSpec {
        SigSpec {
            cn_db: Some(cn_db),
            seed,
            ..SigSpec::default()
        }
    }

    #[test]
    fn video_scores_and_noise_does_not() {
        let fmt = SigSpec::default().sample_format;
        let strong = read(&mut SigGen::new(spec(20.0, 1)), 30.0, 60.0, &fmt);
        assert!(strong.sync_score > 0.9 && strong.sync_quality > 0.5, "{strong:?}");
        assert!(strong.cn_db > 15.0, "{strong:?}");
        assert_eq!(strong.standard, Some(VideoStandard::Ntsc));
        assert!(strong.windows >= 4, "{strong:?}");
        // No transmitter: the carrier 40 dB under the noise.
        let empty = read(&mut SigGen::new(spec(-40.0, 2)), 30.0, 60.0, &fmt);
        assert!(empty.sync_score < 0.1, "{empty:?}");
        assert!(empty.cn_db < 3.0, "{empty:?}");
    }

    #[test]
    fn weak_video_still_beats_noise() {
        let fmt = SigSpec::default().sample_format;
        let weak = read(&mut SigGen::new(spec(8.0, 3)), 30.0, 60.0, &fmt);
        let empty = read(&mut SigGen::new(spec(-40.0, 4)), 30.0, 60.0, &fmt);
        if std::env::var("METER_TRACE").is_ok() {
            for cn in [20.0, 12.0, 8.0, 5.0, 3.0, 0.0, -40.0] {
                eprintln!("C/N {cn}: {:?}", read(&mut SigGen::new(spec(cn, 9)), 30.0, 60.0, &fmt));
            }
        }
        // From C/N 5 up the pulses are found on nearly every line (C/N 3: none within the dwell).
        assert!(weak.sync_score > 0.9, "{weak:?}");
        assert!(empty.sync_score < 0.1, "{empty:?}");
        assert!(weak.cn_db > empty.cn_db + 3.0, "{weak:?} {empty:?}");
    }

    #[test]
    fn a_burst_moves_the_mean_not_the_median() {
        // A packet that rails the ADC for 2 ms of a 60 ms dwell.
        let fmt = SigSpec::default().sample_format;
        let clean = read(&mut SigGen::new(spec(12.0, 5)), 30.0, 60.0, &fmt);
        let mut gen = SigGen::new(spec(12.0, 5));
        let mut burst = SigGen::new(SigSpec {
            rms_adc: 3000.0,
            ..spec(12.0, 6)
        });
        let mut m = ChannelMeter::new(fmt.clone());
        let per_ms = (m.sample_rate() * 1e-3) as usize;
        for _ in 0..30 {
            m.warm(&gen.next(per_ms).bytes);
        }
        for k in 0..60 {
            let c = gen.next(per_ms);
            if k == 21 || k == 22 {
                m.push(&burst.next(per_ms).bytes);
            } else {
                m.push(&c.bytes);
            }
        }
        let hit = m.reading();
        assert!((hit.cn_db - clean.cn_db).abs() < 1.5, "{hit:?} {clean:?}");
        assert!((hit.power_db - clean.power_db).abs() < 1.0, "{hit:?} {clean:?}");
    }

    #[test]
    fn warm_up_is_not_counted() {
        let fmt = SigSpec::default().sample_format;
        let mut gen = SigGen::new(spec(20.0, 7));
        let mut m = ChannelMeter::new(fmt);
        let per_ms = (m.sample_rate() * 1e-3) as usize;
        for _ in 0..30 {
            m.warm(&gen.next(per_ms).bytes);
        }
        let r = m.reading();
        assert_eq!((r.samples, r.lines, r.windows), (0, 0, 0));
        assert_eq!(r.sync_score, 0.0);
        assert!(r.cn_db.is_nan());
        m.push(&gen.next(per_ms * 20).bytes);
        let r = m.reading();
        assert_eq!(r.samples, (per_ms * 20) as u64);
        assert!(r.lines > 0 && r.windows >= 1, "{r:?}");
    }
}
