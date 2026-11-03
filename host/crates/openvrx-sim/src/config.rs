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

impl Default for Agc {
    fn default() -> Self {
        Agc::Loop {
            target_rms: 220.0,
            low_rms: 160.0,
            high_rms: 288.0,
            period_s: 0.25,
            initial: None,
        }
    }
}

/// Sampling-phase lottery at start-up.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Lottery {
    /// Mixed with this probability (3 in 10 by default).
    Random { p_mixed: f64 },
    /// Always a clean position.
    Clean,
    /// Always the position that latches while lanes change.
    Mixed,
}

impl Default for Lottery {
    fn default() -> Self {
        Lottery::Random { p_mixed: 0.3 }
    }
}

impl std::str::FromStr for Lottery {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "random" => Ok(Lottery::default()),
            "clean" => Ok(Lottery::Clean),
            "mixed" => Ok(Lottery::Mixed),
            other => Err(format!("unknown lottery `{other}` (random|clean|mixed)")),
        }
    }
}

/// Sample clock and lane timing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SamplingConfig {
    pub lottery: Lottery,
    /// RMS lane-to-lane skew in 240 MHz ticks (4.17 ns).
    pub lane_skew_ticks: f64,
    /// RMS clock jitter per edge in ticks.
    pub jitter_ticks: f64,
}

impl Default for SamplingConfig {
    fn default() -> Self {
        SamplingConfig {
            lottery: Lottery::default(),
            lane_skew_ticks: 0.12,
            jitter_ticks: 0.03,
        }
    }
}

/// USB loss: chunks of 512-byte packets dropped at random times.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct UsbLoss {
    /// Mean loss events per second (0 = none).
    pub rate_per_s: f64,
    /// Packets lost per event, drawn uniformly in 1..=max.
    pub max_packets: u32,
}

impl Default for UsbLoss {
    fn default() -> Self {
        UsbLoss {
            rate_per_s: 0.0,
            max_packets: 8,
        }
    }
}

/// One receiver (C5 + FX2).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RxConfig {
    pub preset: LanePreset,
    /// PARLIO divider of the 240 MHz PLL.
    pub divider: u32,
    pub filter: ChannelFilter,
    /// C/N offset of this receiver relative to the channel, dB.
    pub cn_offset_db: f64,
    pub antenna: AntennaPattern,
    /// Crystal error in ppm (None: drawn in +-15 ppm). Sets the sample
    /// clock against the transmitter and, unless `lo_offset_hz` is given,
    /// the LO error (ppm x channel frequency).
    pub crystal_ppm: Option<f64>,
    /// Crystal drift, ppm per second.
    pub ppm_drift_per_s: f64,
    /// LO error in Hz (None: from the crystal).
    pub lo_offset_hz: Option<f64>,
    /// Stream start delay against rx0's start, microseconds (None: drawn
    /// in 0..500 us; rx0 always 0).
    pub start_delay_us: Option<f64>,
    pub agc: Agc,
    /// Gain index step in dB (assumed C5 gain law).
    pub gain_step_db: f64,
    /// Gain index at which `noise_floor_rms` is specified.
    pub gain_ref: u32,
    pub gain_min: u32,
    pub gain_max: u32,
    /// Thermal noise after the normal channel filter, complex RMS in
    /// 10-bit ADC units at `gain_ref`.
    pub noise_floor_rms: f64,
    /// I DC offset table: (gain index, offset in 8-bit units), linear
    /// between points and extrapolated beyond.
    pub dc_i_table: Vec<(f64, f64)>,
    /// Q DC offset in 10-bit units.
    pub dc_q: f64,
    /// Scale of this receiver's DC table.
    pub dc_scale: f64,
    /// ADC bits that carry information (from the top); lower bits read 0.
    pub adc_live_bits: u32,
    pub sampling: SamplingConfig,
    pub usb_loss: UsbLoss,
    /// Command-to-effect latency used for event byte positions, ms.
    pub command_latency_ms: f64,
    pub board: String,
    pub antenna_name: String,
}

impl Default for RxConfig {
    fn default() -> Self {
        RxConfig {
            preset: LanePreset::I7Q7,
            divider: 18,
            filter: ChannelFilter::Normal,
            cn_offset_db: 0.0,
            antenna: AntennaPattern::default(),
            crystal_ppm: None,
            ppm_drift_per_s: 0.0,
            lo_offset_hz: None,
            start_delay_us: None,
            agc: Agc::default(),
            gain_step_db: 0.5,
            gain_ref: 64,
            gain_min: 40,
            gain_max: 80,
            noise_floor_rms: 12.0,
            dc_i_table: vec![(64.0, 20.0), (68.0, 32.0), (77.0, 85.0)],
            dc_q: 1.0,
            dc_scale: 1.0,
            adc_live_bits: 8,
            sampling: SamplingConfig::default(),
            usb_loss: UsbLoss::default(),
            command_latency_ms: 1.0,
            board: "sim".into(),
            antenna_name: "sim".into(),
        }
    }
}

/// A command or marker at a time (receiver time of the target receiver).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScheduledEvent {
    pub t_s: f64,
    /// Target receiver (None: all receivers; markers are session-wide).
    #[serde(default)]
    pub rx: Option<u8>,
    pub action: Action,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Action {
    Gain { index: u32 },
    Tune { mhz: u32, offset_khz: i32 },
    Clk { divider: u32 },
    Slip { count: u32 },
    Marker { text: String },
}

impl SimConfig {
    /// Named presets: `clean`, `edge`, `flyout`, `diversity`, `multipath`,
    /// `demo`, `glitch`.
    pub fn preset(name: &str) -> Result<SimConfig, SimError> {
        let mut c = SimConfig::default();
        match name {
            "clean" => {
                c.channel.cn_db = 25.0;
                for r in &mut c.receivers {
                    r.sampling.lottery = Lottery::Clean;
                }
            }
            "edge" => {
                c.channel.cn_db = 7.0;
                c.picture = PictureConfig::Scene;
            }
            "flyout" => {
                c.seconds = 10.0;
                c.channel.cn_db = 24.0;
                c.channel.scenario = Scenario::Flyout {
                    depth_db: 22.0,
                    period_s: 10.0,
                };
                c.channel.fading = Fading {
                    kind: FadingKind::Rician,
                    doppler_hz: 40.0,
                    k_db: 6.0,
                };
                c.picture = PictureConfig::Scene;
                c.osd = true;
            }
            "diversity" => {
                c.seconds = 2.0;
                c.channel.cn_db = 10.0;
                c.channel.fading = Fading {
                    kind: FadingKind::Rayleigh,
                    doppler_hz: 30.0,
                    k_db: 0.0,
                };
                c.receivers = vec![RxConfig::default(), RxConfig::default()];
                c.receivers[1].antenna.offset_db = -2.0;
                c.picture = PictureConfig::Scene;
            }
            "multipath" => {
                c.channel.cn_db = 18.0;
                c.channel.multipath = Some(Multipath::default());
                c.picture = PictureConfig::Multiburst;
            }
            "demo" => {
                c.seconds = 60.0;
                c.channel.cn_db = 16.0;
                c.channel.scenario = Scenario::Orbit {
                    depth_db: 10.0,
                    period_s: 8.0,
                };
                c.channel.fading = Fading {
                    kind: FadingKind::Rician,
                    doppler_hz: 20.0,
                    k_db: 8.0,
                };
                c.picture = PictureConfig::Scene;
                c.osd = true;
                for r in &mut c.receivers {
                    r.sampling.lottery = Lottery::Clean;
                }
            }
            "glitch" => {
                for r in &mut c.receivers {
                    r.sampling.lottery = Lottery::Mixed;
                }
                c.events.push(ScheduledEvent {
                    t_s: 0.5,
                    rx: None,
                    action: Action::Slip { count: 1 },
                });
            }
            other => {
                return Err(SimError::Config(format!(
                    "unknown preset `{other}` (clean|edge|flyout|diversity|multipath|demo|glitch)"
                )))
            }
        }
        Ok(c)
    }

    pub const PRESETS: [&'static str; 7] =
        ["clean", "edge", "flyout", "diversity", "multipath", "demo", "glitch"];

    /// Checks ranges.
    pub fn validate(&self) -> Result<(), SimError> {
        let bad = |m: String| Err(SimError::Config(m));
        if !self.seconds.is_finite() || self.seconds <= 0.0 {
            return bad(format!("seconds must be > 0 (got {})", self.seconds));
        }
        if self.receivers.is_empty() || self.receivers.len() > 8 {
            return bad("1 to 8 receivers".into());
        }
        if !(self.vtx.deviation > 0.0 && self.vtx.deviation < 4.0) {
            return bad(format!("deviation {} out of range", self.vtx.deviation));
        }
        if self.vtx.blank_hz.abs() > 10e6 {
            return bad("blank_hz beyond +-10 MHz".into());
        }
        if self.channel.fading.doppler_hz < 0.0 || self.channel.fading.doppler_hz > 5000.0 {
            return bad("doppler_hz out of 0..5000".into());
        }
        if let Some(m) = &self.channel.multipath {
            if !(0.0..=2000.0).contains(&m.delay_ns) {
                return bad("multipath delay must be 0..2000 ns".into());
            }
        }
        if let Some(pe) = &self.vtx.pre_emphasis {
            if !(pe.zero_hz > 0.0 && pe.pole_hz > pe.zero_hz && pe.pole_hz < 20e6) {
                return bad("pre-emphasis needs 0 < zero < pole < 20 MHz".into());
            }
        }
        for (k, r) in self.receivers.iter().enumerate() {
            if !(3..=255).contains(&r.divider) {
                return bad(format!("rx{k}: divider {} out of 3..255", r.divider));
            }
            if r.gain_min > r.gain_max {
                return bad(format!("rx{k}: gain_min > gain_max"));
            }
            if !(1..=10).contains(&r.adc_live_bits) {
                return bad(format!("rx{k}: adc_live_bits 1..10"));
            }
            if r.noise_floor_rms.is_nan() || r.noise_floor_rms <= 0.0 {
                return bad(format!("rx{k}: noise_floor_rms must be > 0"));
            }
            if let Some(p) = r.crystal_ppm {
                if p.abs() > 500.0 {
                    return bad(format!("rx{k}: crystal_ppm beyond 500"));
                }
            }
            if let Agc::Loop {
                low_rms, high_rms, target_rms, period_s, ..
            } = r.agc
            {
                if !(low_rms < target_rms && target_rms < high_rms && period_s > 0.0) {
                    return bad(format!("rx{k}: AGC needs low < target < high, period > 0"));
                }
            }
            if let Lottery::Random { p_mixed } = r.sampling.lottery {
                if !(0.0..=1.0).contains(&p_mixed) {
                    return bad(format!("rx{k}: p_mixed must be 0..1"));
                }
            }
        }
        for e in &self.events {
            if let Some(rx) = e.rx {
                if rx as usize >= self.receivers.len() {
                    return bad(format!("event for rx{rx}, which does not exist"));
                }
            }
            if let Action::Clk { divider } = e.action {
                if !(3..=255).contains(&divider) {
                    return bad(format!("clk divider {divider} out of 3..255"));
                }
            }
        }
        Ok(())
    }

    /// Sets the number of receivers, copying rx0's settings.
    pub fn set_receiver_count(&mut self, n: usize) {
        let proto = self.receivers.first().cloned().unwrap_or_default();
        self.receivers.resize(n.max(1), proto);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_validate_and_round_trip() {
        for p in SimConfig::PRESETS {
            let c = SimConfig::preset(p).unwrap();
            c.validate().unwrap();
            let j = serde_json::to_string(&c).unwrap();
            let back: SimConfig = serde_json::from_str(&j).unwrap();
            assert_eq!(back, c, "{p}");
        }
        assert!(SimConfig::preset("nope").is_err());
    }

    #[test]
    fn partial_json_uses_defaults() {
        let c: SimConfig = serde_json::from_str(
            r#"{"standard":"pal","picture":{"kind":"image","path":"x.png"},
                "receivers":[{"preset":"8x8","divider":12}]}"#,
        )
        .unwrap();
        assert_eq!(c.standard, VideoStandard::Pal);
        assert_eq!(c.receivers[0].divider, 12);
        assert_eq!(c.receivers[0].gain_ref, 64);
        assert!(serde_json::from_str::<SimConfig>(r#"{"sedd":1}"#).is_err());
    }

    #[test]
    fn scenarios() {
        let f = Scenario::Flyout {
            depth_db: 20.0,
            period_s: 10.0,
        };
        assert!(f.offset_db(0.0).abs() < 1e-9);
        assert!((f.offset_db(5.0) + 20.0).abs() < 1e-9);
        let c = Scenario::Curve {
            points: vec![(0.0, 0.0), (1.0, -10.0)],
        };
        assert_eq!(c.offset_db(0.5), -5.0);
        assert_eq!(c.offset_db(3.0), -10.0);
        assert!("flyout".parse::<Scenario>().is_ok());
        assert!(matches!("image:a.png".parse::<PictureConfig>(), Ok(PictureConfig::Image { .. })));
    }
}
