// SPDX-License-Identifier: GPL-3.0-or-later
//! Simulator configuration ([`SimConfig`]): serde, with defaults for every
//! field so a JSON file only needs what it changes, and named presets.

use crate::SimError;
use openvrx_core::{LanePreset, VideoStandard};
use serde::{Deserialize, Serialize};

/// Sync tip to peak white deviation at `deviation = 1.0` (C5VRX sim.py).
pub const DEV_PP_HZ: f64 = 6.7e6;

/// Everything the simulator needs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SimConfig {
    /// Master seed; every random draw derives from it.
    pub seed: u64,
    /// Length in seconds of receiver time (each receiver's own clock).
    pub seconds: f64,
    pub standard: VideoStandard,
    pub picture: PictureConfig,
    /// FPV-style OSD overlay (voltage, timer).
    pub osd: bool,
    pub cvbs: CvbsConfig,
    pub vtx: VtxConfig,
    pub channel: ChannelConfig,
    /// One entry per receiver (rx0, rx1, ...).
    pub receivers: Vec<RxConfig>,
    /// Scheduled commands and markers.
    pub events: Vec<ScheduledEvent>,
    /// Write the truth file (clean composite at rx0's samples).
    pub truth: bool,
    /// Free text stored in the sidecar's notes.
    pub notes: String,
}

impl Default for SimConfig {
    fn default() -> Self {
        SimConfig {
            seed: 1,
            seconds: 1.0,
            standard: VideoStandard::Ntsc,
            picture: PictureConfig::Bars,
            osd: false,
            cvbs: CvbsConfig::default(),
            vtx: VtxConfig::default(),
            channel: ChannelConfig::default(),
            receivers: vec![RxConfig::default()],
            events: Vec::new(),
            truth: true,
            notes: String::new(),
        }
    }
}

/// Picture source.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PictureConfig {
    /// SMPTE bars for NTSC, EBU 75 % bars for PAL.
    Bars,
    /// Luma ramp and staircase with a colour ramp band.
    Ramp,
    /// Multiburst frequency packets.
    Multiburst,
    /// Circular zone plate, slowly moving.
    ZonePlate,
    /// Moving scene: panning ground, rotating wheel, scrolling text,
    /// bouncing ball, vertical drift.
    Scene,
    /// Flat field at a luma level (0..1) with optional colour.
    Flat { y: f32, u: f32, v: f32 },
    /// Still image file (PNG/JPEG), letterboxed to 4:3.
    Image { path: String },
    /// Video or image sequence through ffmpeg (looped).
    Video { path: String },
}

impl std::str::FromStr for PictureConfig {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        if let Some(p) = s.strip_prefix("image:") {
            return Ok(PictureConfig::Image { path: p.to_string() });
        }
        if let Some(p) = s.strip_prefix("video:") {
            return Ok(PictureConfig::Video { path: p.to_string() });
        }
        if let Some(y) = s.strip_prefix("flat:") {
            let y: f32 = y.parse().map_err(|_| format!("bad flat level `{y}`"))?;
            return Ok(PictureConfig::Flat { y, u: 0.0, v: 0.0 });
        }
        match s {
            "bars" => Ok(PictureConfig::Bars),
            "ramp" => Ok(PictureConfig::Ramp),
            "multiburst" => Ok(PictureConfig::Multiburst),
            "zoneplate" | "zone_plate" => Ok(PictureConfig::ZonePlate),
            "scene" => Ok(PictureConfig::Scene),
            "black" => Ok(PictureConfig::Flat { y: 0.0, u: 0.0, v: 0.0 }),
            other => Err(format!(
                "unknown picture `{other}` (bars|ramp|multiburst|zoneplate|scene|black|flat:Y|image:PATH|video:PATH)"
            )),
        }
    }
}

/// Composite encoder settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CvbsConfig {
    /// 7.5 IRE setup (None: the standard's own, NTSC on, PAL off).
    pub setup: Option<bool>,
    /// Luma bandwidth (None: 4.2 MHz NTSC, 5.0 MHz PAL).
    pub luma_bw_hz: Option<f64>,
    /// U/V bandwidth before modulation.
    pub chroma_bw_hz: f64,
    /// Colour on (burst and chroma).
    pub colour: bool,
    /// Camera noise, RMS in IRE on luma (chroma gets half), white
    /// Gaussian per picture sample and field.
    pub camera_noise_ire: f32,
    /// Frame line (1-based) where the transmitter's stream starts for rx0
    /// (None: drawn from the seed).
    pub start_line: Option<u32>,
}

impl Default for CvbsConfig {
    fn default() -> Self {
        CvbsConfig {
            setup: None,
            luma_bw_hz: None,
            chroma_bw_hz: 1.3e6,
            colour: true,
            camera_noise_ire: 0.0,
            start_line: None,
        }
    }
}

/// Transmitter (analog VTX) model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VtxConfig {
    /// Deviation scale: sync tip to peak white is `deviation x 6.7 MHz`.
    pub deviation: f64,
    /// Frequency of the blanking level relative to the channel centre.
    pub blank_hz: f64,
    /// Carrier frequency error (VTX PLL).
    pub freq_error_hz: f64,
    /// Linear carrier drift (warm-up).
    pub drift_hz_per_s: f64,
    /// Slow sinusoidal carrier wander amplitude and period.
    pub wander_hz: f64,
    pub wander_period_s: f64,
    /// Video pre-emphasis (None: off).
    pub pre_emphasis: Option<PreEmphasis>,
    /// Modulator integral nonlinearity: bow of the voltage-to-frequency
    /// curve as a fraction of the full sync-to-white span (0 = linear).
    pub nonlinearity: f64,
    /// Third-order term of the nonlinearity.
    pub nonlinearity3: f64,
    /// Transmit channel in MHz.
    pub channel_mhz: u32,
    /// Camera video clock error in ppm (line rate offset).
    pub video_clock_ppm: f64,
}

impl Default for VtxConfig {
    fn default() -> Self {
        VtxConfig {
            deviation: 1.0,
            blank_hz: -1.0e6,
            freq_error_hz: 0.0,
            drift_hz_per_s: 0.0,
            wander_hz: 0.0,
            wander_period_s: 20.0,
            pre_emphasis: None,
            nonlinearity: 0.02,
            nonlinearity3: 0.0,
            channel_mhz: 5732,
            video_clock_ppm: 0.0,
        }
    }
}

/// First-order pre-emphasis shelf, unity gain at `crossover_hz`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PreEmphasis {
    pub zero_hz: f64,
    pub pole_hz: f64,
    pub crossover_hz: f64,
}

impl Default for PreEmphasis {
    fn default() -> Self {
        PreEmphasis {
            zero_hz: 0.62e6,
            pole_hz: 3.1e6,
            crossover_hz: 0.76e6,
        }
    }
}

/// Radio channel shared by all receivers (each receiver draws its own
/// realisation: fading, multipath phase).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ChannelConfig {
    /// Mean carrier-to-noise ratio in dB, per 20 MHz channel (noise in
    /// the C5's normal analog filter), before fading.
    pub cn_db: f64,
    /// Time-varying C/N curve added to `cn_db`.
    pub scenario: Scenario,
    pub fading: Fading,
    pub multipath: Option<Multipath>,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        ChannelConfig {
            cn_db: 20.0,
            scenario: Scenario::Constant,
            fading: Fading::default(),
            multipath: None,
        }
    }
}

/// C/N over time.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Scenario {
    Constant,
    /// Fly out to the range limit and back: C/N falls by `depth_db`
    /// (cosine shape) over `period_s` and returns.
    Flyout { depth_db: f64, period_s: f64 },
    /// Orbit: sinusoidal C/N swing of +-`depth_db`/2.
    Orbit { depth_db: f64, period_s: f64 },
    /// Piecewise-linear (time s, C/N offset dB) points.
    Curve { points: Vec<(f64, f64)> },
}

impl Scenario {
    /// C/N offset in dB at time t.
    pub fn offset_db(&self, t: f64) -> f64 {
        match self {
            Scenario::Constant => 0.0,
            Scenario::Flyout { depth_db, period_s } => {
                let ph = (t / period_s.max(1e-9)).fract();
                -depth_db * 0.5 * (1.0 - (std::f64::consts::TAU * ph).cos())
            }
            Scenario::Orbit { depth_db, period_s } => {
                0.5 * depth_db * (std::f64::consts::TAU * t / period_s.max(1e-9)).sin()
            }
            Scenario::Curve { points } => piecewise(points, t),
        }
    }
}

/// Piecewise-linear interpolation, clamped at the ends.
pub fn piecewise(points: &[(f64, f64)], t: f64) -> f64 {
    match points {
        [] => 0.0,
        [p] => p.1,
        _ => {
            if t <= points[0].0 {
                return points[0].1;
            }
            for w in points.windows(2) {
                if t <= w[1].0 {
                    let span = (w[1].0 - w[0].0).max(1e-12);
                    return w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / span;
                }
            }
            points[points.len() - 1].1
        }
    }
}

impl std::str::FromStr for Scenario {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "constant" | "none" => Ok(Scenario::Constant),
            "flyout" => Ok(Scenario::Flyout {
                depth_db: 20.0,
                period_s: 10.0,
            }),
            "orbit" => Ok(Scenario::Orbit {
                depth_db: 10.0,
                period_s: 6.0,
            }),
            other => Err(format!("unknown scenario `{other}` (constant|flyout|orbit)")),
        }
    }
}

/// Small-scale fading.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Fading {
    pub kind: FadingKind,
    /// Maximum Doppler frequency in Hz.
    pub doppler_hz: f64,
    /// Rician K factor in dB (Rician only).
    pub k_db: f64,
}

impl Default for Fading {
    fn default() -> Self {
        Fading {
            kind: FadingKind::None,
            doppler_hz: 30.0,
            k_db: 6.0,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FadingKind {
    None,
    Rayleigh,
    Rician,
}

impl std::str::FromStr for FadingKind {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "none" => Ok(FadingKind::None),
            "rayleigh" => Ok(FadingKind::Rayleigh),
            "rician" | "rice" => Ok(FadingKind::Rician),
            other => Err(format!("unknown fading `{other}` (none|rayleigh|rician)")),
        }
    }
}

/// Two-ray multipath: a delayed copy of the signal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Multipath {
    pub delay_ns: f64,
    /// Power of the second ray relative to the first, dB.
    pub rel_db: f64,
    /// Phase of the second ray (None: drawn per receiver).
    pub phase_deg: Option<f64>,
    /// The second ray fades independently (with the channel's fading).
    pub fading: bool,
}

impl Default for Multipath {
    fn default() -> Self {
        Multipath {
            delay_ns: 150.0,
            rel_db: -6.0,
            phase_deg: None,
            fading: true,
        }
    }
}

/// The C5's analog channel filter.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelFilter {
    /// About +-10 MHz (5th-order Butterworth).
    Normal,
    /// 802.11p setting, about +-6 MHz.
    P11,
}

impl ChannelFilter {
    /// -3 dB edge of the complex baseband filter (one side), Hz.
    pub fn cutoff_hz(self) -> f64 {
        match self {
            ChannelFilter::Normal => 10.0e6,
            ChannelFilter::P11 => 6.0e6,
        }
    }
}

/// Per-receiver antenna gain over time (polarisation, pattern).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AntennaPattern {
    /// Constant gain offset, dB.
    pub offset_db: f64,
    /// Peak-to-peak swing of a slow sinusoidal pattern, dB.
    pub depth_db: f64,
    pub period_s: f64,
    /// Pattern phase (None: drawn per receiver).
    pub phase_deg: Option<f64>,
}

impl Default for AntennaPattern {
    fn default() -> Self {
        AntennaPattern {
            offset_db: 0.0,
            depth_db: 0.0,
            period_s: 4.0,
            phase_deg: None,
        }
    }
}

/// Gain control.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Agc {
    /// Fixed gain index (changed only by scheduled events).
    Fixed { gain: u32 },
    /// Slow host gain loop: keeps the I/Q RMS (10-bit units, DC excluded)
    /// inside [low, high], aims at `target`, changes only in vertical
    /// blanking. `initial` None: chosen at start to hit the target.
    Loop {
        target_rms: f64,
        low_rms: f64,
        high_rms: f64,
        period_s: f64,
        initial: Option<u32>,
    },
}

