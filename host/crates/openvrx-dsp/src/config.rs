// SPDX-License-Identifier: GPL-3.0-or-later
//! Receiver DSP configuration. Every struct is serde-serialisable with
//! defaults for missing fields, so saved settings survive upgrades.

use openvrx_core::{LanePreset, SampleFormat, VideoStandard};
use serde::{Deserialize, Serialize};

/// FM demodulator selection. All produce instantaneous frequency in Hz.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DemodKind {
    /// Exact polar discriminator arg(z[n] conj z[n-1]).
    Polar,
    /// Angle look-up table on raw words and integer angle differences
    /// (byte input only, channel filter bypassed; falls back to `Polar`).
    LutAngle,
    /// Kay-weighted least-squares multi-sample frequency estimator.
    LeastSquares,
    /// Phase-locked loop (2nd or 3rd order) tracking demodulator.
    Pll,
    /// Extended Kalman filter tracker (phase, frequency, frequency slope).
    Ekf,
}

impl DemodKind {
    pub const ALL: [DemodKind; 5] = [
        DemodKind::Polar,
        DemodKind::LutAngle,
        DemodKind::LeastSquares,
        DemodKind::Pll,
        DemodKind::Ekf,
    ];

    pub fn name(self) -> &'static str {
        match self {
            DemodKind::Polar => "polar",
            DemodKind::LutAngle => "lut",
            DemodKind::LeastSquares => "ls",
            DemodKind::Pll => "pll",
            DemodKind::Ekf => "ekf",
        }
    }
}

/// DC offset removal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DcConfig {
    pub enabled: bool,
    /// Tracking time constant in normal operation.
    pub time_constant_ms: f32,
    /// Estimation sub-block in samples.
    pub block: usize,
    /// After `GainChanged`, how long to look for the step.
    pub armed_ms: f32,
    /// Power change that confirms a gain step while armed.
    pub step_db: f32,
}

impl Default for DcConfig {
    fn default() -> Self {
        DcConfig {
            enabled: true,
            time_constant_ms: 20.0,
            block: 512,
            armed_ms: 60.0,
            step_db: 0.35,
        }
    }
}

/// Blind I/Q gain and phase imbalance correction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct IqConfig {
    pub enabled: bool,
    pub time_constant_ms: f32,
}

impl Default for IqConfig {
    fn default() -> Self {
        IqConfig {
            enabled: true,
            time_constant_ms: 200.0,
        }
    }
}

/// Where the AFC puts the signal with the digital frequency shift.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "mode")]
pub enum AfcTarget {
    /// Centre between sync tip and (estimated) peak white at 0 Hz: the most
    /// margin against phase-step wrap on both sides.
    Centre,
    /// Blanking at this frequency after the shift.
    Porch { hz: f32 },
}

/// Carrier offset measurement and fine digital frequency shift.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AfcConfig {
    /// Track the carrier with the NCO. When false, `manual_shift_hz` is used.
    pub enabled: bool,
    pub target: AfcTarget,
    pub manual_shift_hz: f32,
    /// Changes smaller than this are not applied.
    pub deadband_hz: f32,
    /// Fraction of the error applied per update (once per field when
    /// locked, per block while searching).
    pub loop_gain: f32,
    /// Shift limit as a fraction of the sample rate.
    pub max_shift_frac: f32,
}

impl Default for AfcConfig {
    fn default() -> Self {
        AfcConfig {
            enabled: true,
            target: AfcTarget::Centre,
            manual_shift_hz: 0.0,
            deadband_hz: 20e3,
            loop_gain: 0.5,
            max_shift_frac: 0.3,
        }
    }
}

/// Complex channel FIR before the discriminator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ChannelFilterConfig {
    pub enabled: bool,
    /// One-sided -6 dB bandwidth around the shifted carrier. Values at or
    /// above 0.47 x sample rate disable the filter.
    pub cutoff_hz: f32,
    pub transition_hz: f32,
    pub atten_db: f32,
    /// Narrow the filter as C/N drops (threshold extension).
    pub adaptive: bool,
    pub min_cutoff_hz: f32,
    /// At or below this C/N the cutoff is `min_cutoff_hz`.
    pub cn_lo_db: f32,
    /// At or above this C/N the cutoff is `cutoff_hz`.
    pub cn_hi_db: f32,
    pub max_taps: usize,
}

impl Default for ChannelFilterConfig {
    fn default() -> Self {
        ChannelFilterConfig {
            enabled: true,
            cutoff_hz: 8.0e6,
            transition_hz: 2.0e6,
            atten_db: 40.0,
            adaptive: true,
            min_cutoff_hz: 4.5e6,
            cn_lo_db: 2.0,
            cn_hi_db: 10.0,
            max_taps: 63,
        }
    }
}

/// PLL demodulator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PllConfig {
    /// 2 or 3.
    pub order: u8,
    /// Loop natural frequency in Hz.
    pub natural_hz: f32,
    pub damping: f32,
}

impl Default for PllConfig {
    fn default() -> Self {
        PllConfig {
            order: 2,
            natural_hz: 6.0e6,
            damping: 0.8,
        }
    }
}

/// EKF tracker.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EkfConfig {
    /// Tracking bandwidth (Hz) the process noise is designed for, at
    /// `design_cn_db`. The effective bandwidth falls as C/N falls.
    pub bandwidth_hz: f32,
    pub design_cn_db: f32,
}

impl Default for EkfConfig {
    fn default() -> Self {
        EkfConfig {
            bandwidth_hz: 3.0e6,
            design_cn_db: 10.0,
        }
    }
}

/// Least-squares estimator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LsConfig {
    /// Phase differences combined per estimate (odd, >= 1).
    pub len: usize,
}

impl Default for LsConfig {
    fn default() -> Self {
        LsConfig { len: 3 }
    }
}

/// FM click and phase-wrap repair.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ClickConfig {
    pub enabled: bool,
    /// Spike (sample minus mean of neighbours, rad) that triggers a check.
    pub spike_rad: f32,
    /// Lower trigger used where the envelope dips.
    pub dip_spike_rad: f32,
    /// Envelope dip: |z|^2 below this fraction of the mean power.
    pub dip_level: f32,
    /// A span whose excess phase is at least this fraction of 2 pi is a slip.
    pub area_frac: f32,
    /// Spike near +-pi with a dip is repaired even without a full slip
    /// (values above pi disable this rule; the default does, because on
    /// synthetic captures it removed as much detail as noise).
    pub dip_impulse_rad: f32,
    /// Half width (samples) of the repaired span; tracking demodulators
    /// spread slips wider and use twice this.
    pub half_span: usize,
}

impl Default for ClickConfig {
    fn default() -> Self {
        ClickConfig {
            enabled: true,
            spike_rad: 1.8,
            dip_spike_rad: 1.2,
            dip_level: 0.2,
            area_frac: 0.85,
            dip_impulse_rad: 10.0,
            half_span: 1,
        }
    }
}

/// Optional de-emphasis (first-order shelf).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DeemphasisConfig {
    pub enabled: bool,
    pub pole_hz: f32,
    pub zero_hz: f32,
}

impl Default for DeemphasisConfig {
    fn default() -> Self {
        DeemphasisConfig {
            enabled: false,
            pole_hz: 0.4e6,
            zero_hz: 1.6e6,
        }
    }
}

/// Video low-pass (linear phase; its delay is compensated).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VideoLpfConfig {
    pub enabled: bool,
    /// -6 dB point.
    pub cutoff_hz: f32,
    pub transition_hz: f32,
    pub atten_db: f32,
    pub max_taps: usize,
}

impl Default for VideoLpfConfig {
    fn default() -> Self {
        VideoLpfConfig {
            enabled: true,
            cutoff_hz: 5.0e6,
            transition_hz: 1.3e6,
            atten_db: 50.0,
            max_taps: 127,
        }
    }
}

/// Sync separation and line flywheel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SyncConfig {
    /// Lines to coast on the flywheel before declaring loss of lock.
    pub coast_lines: u32,
    /// Consecutive consistent off-prediction pulses that force a relock.
    pub relock_pulses: u32,
    /// Flywheel timing process noise, input samples RMS per line.
    pub phase_noise: f32,
    /// Flywheel period process noise, input samples RMS per line.
    pub period_noise: f32,
    /// Level tracker process noise, IRE RMS per line (blanking).
    pub level_noise_ire: f32,
    /// Search window around the prediction (us).
    pub window_us: f32,
    /// Emit lines before the first vertical sync after this many fields.
    pub provisional_fields: u32,
    /// Segment length (us) for the per-segment envelope C/N grading of the
    /// damage mask (0 disables). Diversity repair that prefers the less
    /// damaged receiver per sample then selects per segment.
    pub segment_us: f32,
    /// Segments with an envelope C/N at or above this get no grading;
    /// lower C/N grades linearly up to `damage::NOISY` at 0 dB.
    pub segment_clean_db: f32,
}

impl Default for SyncConfig {
    fn default() -> Self {
        SyncConfig {
            coast_lines: 400,
            relock_pulses: 3,
            phase_noise: 0.004,
            period_noise: 0.0005,
            level_noise_ire: 0.15,
            window_us: 1.6,
            provisional_fields: 2,
            segment_us: 4.0,
            segment_clean_db: 15.0,
        }
    }
}

/// Line resampler onto the grid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ResamplerConfig {
    /// Taps per output sample (even, >= 8).
    pub taps: usize,
    pub phases: usize,
}

impl Default for ResamplerConfig {
    fn default() -> Self {
        ResamplerConfig {
            taps: 16,
            phases: 512,
        }
    }
}

/// Spectrum tap.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SpectrumConfig {
    pub enabled: bool,
    /// FFT size (power of two not required, rustfft handles any size).
    pub fft_size: usize,
    /// FFT frames averaged per published spectrum.
    pub averages: usize,
    /// Published spectra per second (input frames are skipped to match).
    pub rate_hz: f32,
}

impl Default for SpectrumConfig {
    fn default() -> Self {
        SpectrumConfig {
            enabled: false,
            fft_size: 1024,
            averages: 16,
            rate_hz: 30.0,
        }
    }
}

/// Live sampling-phase glitch counter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GlitchConfig {
    pub enabled: bool,
    /// The off-ring test is only meaningful on a strong carrier.
    pub min_cn_db: f32,
}

impl Default for GlitchConfig {
    fn default() -> Self {
        GlitchConfig {
            enabled: true,
            min_cn_db: 14.0,
        }
    }
}

/// Which signal the demodulator tap captures.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TapPoint {
    /// Discriminator output after click repair, before de-emphasis and LPF.
    Demod,
    /// After de-emphasis and the video low-pass (what sync sees).
    Video,
}

/// Complete per-receiver configuration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RxDspConfig {
    pub sample_format: SampleFormat,
    /// `None` detects NTSC/PAL automatically.
    pub standard: Option<VideoStandard>,
    pub demod: DemodKind,
    pub dc: DcConfig,
    pub iq_correction: IqConfig,
    pub afc: AfcConfig,
    pub channel_filter: ChannelFilterConfig,
    pub pll: PllConfig,
    pub ekf: EkfConfig,
    pub least_squares: LsConfig,
    pub clicks: ClickConfig,
    pub deemphasis: DeemphasisConfig,
    pub video_lpf: VideoLpfConfig,
    pub sync: SyncConfig,
    pub resampler: ResamplerConfig,
    pub spectrum: SpectrumConfig,
    pub glitch: GlitchConfig,
    /// Modulation polarity: true when sync is the highest frequency.
    pub invert: bool,
    /// Window for the receiver statistics.
    pub stats_window_ms: f32,
    /// Processing block in samples (latency/efficiency trade-off).
    pub block: usize,
    /// Runs the video filter and the sync/TBC stage on a second thread,
    /// overlapped with the front end of the next block. The output is
    /// identical either way (same block order, same AFC timing).
    pub pipeline_thread: bool,
}

impl Default for RxDspConfig {
    fn default() -> Self {
        RxDspConfig {
            sample_format: SampleFormat::from_preset(LanePreset::I7Q7, 18),
            standard: None,
            demod: DemodKind::Polar,
            dc: DcConfig::default(),
            iq_correction: IqConfig::default(),
            afc: AfcConfig::default(),
            channel_filter: ChannelFilterConfig::default(),
            pll: PllConfig::default(),
            ekf: EkfConfig::default(),
            least_squares: LsConfig::default(),
            clicks: ClickConfig::default(),
            deemphasis: DeemphasisConfig::default(),
            video_lpf: VideoLpfConfig::default(),
            sync: SyncConfig::default(),
            resampler: ResamplerConfig::default(),
            spectrum: SpectrumConfig::default(),
            glitch: GlitchConfig::default(),
            invert: false,
            stats_window_ms: 100.0,
            block: 2048,
            pipeline_thread: true,
        }
    }
}

impl RxDspConfig {
    pub fn sample_rate_hz(&self) -> f64 {
        self.sample_format.sample_rate_hz()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serde_roundtrip_and_defaults() {
        let c = RxDspConfig::default();
        let s = serde_json::to_string(&c).unwrap();
        let back: RxDspConfig = serde_json::from_str(&s).unwrap();
        assert_eq!(back, c);
        // Partial JSON keeps defaults for missing fields.
        let p: RxDspConfig = serde_json::from_str(r#"{"demod":"ekf","afc":{"deadband_hz":5000}}"#).unwrap();
        assert_eq!(p.demod, DemodKind::Ekf);
        assert_eq!(p.afc.deadband_hz, 5000.0);
        assert_eq!(p.afc.loop_gain, AfcConfig::default().loop_gain);
        assert!((p.sample_rate_hz() - 13.333e6).abs() < 1e3);
    }
}
