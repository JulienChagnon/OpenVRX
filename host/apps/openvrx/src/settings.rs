// SPDX-License-Identifier: GPL-3.0-or-later
//! Persistent application settings (`%APPDATA%\OpenVRX\settings.json`).
//!
//! Every struct uses `#[serde(default)]`, so files written by older or
//! newer versions load with missing fields filled from the defaults and
//! unknown fields ignored. A file that does not parse is moved aside to
//! `settings.json.bad` and the defaults are used.
//!
//! Saving happens on a background thread with a short debounce, so the
//! UI thread never touches the disk.

pub use openvrx_hw::c5::BoardKind;
use openvrx_core::{LanePreset, RxId, VideoStandard};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::thread::JoinHandle;
use std::time::Duration;

/// Current settings file version.
pub const SETTINGS_VERSION: u32 = 1;
/// Maximum number of receivers the settings model holds.
pub const MAX_RECEIVERS: usize = 8;
/// Number of view slots (quad view).
pub const VIEW_SLOTS: usize = 4;

// ---------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------

/// Swap-chain present mode.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PresentModeSetting {
    /// No vsync; tears, lowest latency. Needed for slice presentation.
    Immediate,
    /// Newest complete image replaces any queued one at vblank; no tearing.
    #[default]
    Mailbox,
    /// Classic vsync queue; highest latency, always supported.
    Fifo,
}

impl PresentModeSetting {
    pub const ALL: [PresentModeSetting; 3] = [
        PresentModeSetting::Immediate,
        PresentModeSetting::Mailbox,
        PresentModeSetting::Fifo,
    ];
    pub fn name(self) -> &'static str {
        match self {
            PresentModeSetting::Immediate => "Immediate",
            PresentModeSetting::Mailbox => "Mailbox",
            PresentModeSetting::Fifo => "Fifo (vsync)",
        }
    }
    pub fn next(self) -> Self {
        cycle(&Self::ALL, self)
    }
}

/// When new video rows cause a present.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PresentTrigger {
    /// After both fields of a frame.
    Frame,
    /// After every field (50 or 59.94 Hz).
    #[default]
    Field,
    /// Every `present_lines` new lines (slice presentation), and at the end
    /// of every field.
    Lines,
}

impl PresentTrigger {
    pub const ALL: [PresentTrigger; 3] =
        [PresentTrigger::Frame, PresentTrigger::Field, PresentTrigger::Lines];
    pub fn name(self) -> &'static str {
        match self {
            PresentTrigger::Frame => "every frame",
            PresentTrigger::Field => "every field",
            PresentTrigger::Lines => "every N lines",
        }
    }
    pub fn next(self) -> Self {
        cycle(&Self::ALL, self)
    }
}

/// How the 4:3 picture is fitted into its view.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AspectMode {
    /// Whole picture, correct shape, black bars.
    #[default]
    Fit,
    /// Fill the view, correct shape, edges cropped.
    Fill,
    /// Fill the view, picture distorted (16:9 stretch on a wide output).
    Stretch,
}

impl AspectMode {
    pub const ALL: [AspectMode; 3] = [AspectMode::Fit, AspectMode::Fill, AspectMode::Stretch];
    pub fn name(self) -> &'static str {
        match self {
            AspectMode::Fit => "Fit 4:3",
            AspectMode::Fill => "Fill (crop)",
            AspectMode::Stretch => "Stretch",
        }
    }
    pub fn next(self) -> Self {
        cycle(&Self::ALL, self)
    }
}

/// Physical shape of the output, when it differs from its pixel count
/// (non-square output pixels, as on the HD3 in HDMI mode).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputAspect {
    /// Square pixels: shape follows the pixel count.
    #[default]
    Auto,
    /// 16:9 screen whatever the mode.
    Wide16x9,
    /// HD3 HDMI mode: shown on an 800x480 area of the panel (5:3).
    Hd3Panel5x3,
    /// 4:3 screen whatever the mode.
    Standard4x3,
}

impl OutputAspect {
    pub const ALL: [OutputAspect; 4] = [
        OutputAspect::Auto,
        OutputAspect::Wide16x9,
        OutputAspect::Hd3Panel5x3,
        OutputAspect::Standard4x3,
    ];
    /// Physical width / height, or None for square pixels.
    pub fn ratio(self) -> Option<f32> {
        match self {
            OutputAspect::Auto => None,
            OutputAspect::Wide16x9 => Some(16.0 / 9.0),
            OutputAspect::Hd3Panel5x3 => Some(800.0 / 480.0),
            OutputAspect::Standard4x3 => Some(4.0 / 3.0),
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            OutputAspect::Auto => "Square pixels",
            OutputAspect::Wide16x9 => "16:9 screen",
            OutputAspect::Hd3Panel5x3 => "HD3 800x480 (5:3)",
            OutputAspect::Standard4x3 => "4:3 screen",
        }
    }
}

/// Picture scaling filter (luma; chroma always bilinear except Nearest).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScalingFilter {
    /// Diagnostics: shows the sample grid.
    Nearest,
    Bilinear,
    /// Catmull-Rom bicubic.
    #[default]
    Bicubic,
    Lanczos2,
}

impl ScalingFilter {
    pub const ALL: [ScalingFilter; 4] = [
        ScalingFilter::Nearest,
        ScalingFilter::Bilinear,
        ScalingFilter::Bicubic,
        ScalingFilter::Lanczos2,
    ];
    pub fn name(self) -> &'static str {
        match self {
            ScalingFilter::Nearest => "Nearest",
            ScalingFilter::Bilinear => "Bilinear",
            ScalingFilter::Bicubic => "Bicubic",
            ScalingFilter::Lanczos2 => "Lanczos-2",
        }
    }
    /// Shader code.
    pub fn code(self) -> u32 {
        match self {
            ScalingFilter::Nearest => 0,
            ScalingFilter::Bilinear => 1,
            ScalingFilter::Bicubic => 2,
            ScalingFilter::Lanczos2 => 3,
        }
    }
    pub fn next(self) -> Self {
        cycle(&Self::ALL, self)
    }
}

/// Split-screen layout.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViewLayout {
    #[default]
    Single,
    Dual,
    Quad,
}

impl ViewLayout {
    pub const ALL: [ViewLayout; 3] = [ViewLayout::Single, ViewLayout::Dual, ViewLayout::Quad];
    pub fn count(self) -> usize {
        match self {
            ViewLayout::Single => 1,
            ViewLayout::Dual => 2,
            ViewLayout::Quad => 4,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            ViewLayout::Single => "1 view",
            ViewLayout::Dual => "2 views",
            ViewLayout::Quad => "4 views",
        }
    }
    pub fn next(self) -> Self {
        cycle(&Self::ALL, self)
    }
}

/// OSD arrangement.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OsdLayout {
    /// Everything, laptop-screen sized.
    #[default]
    Full,
    /// Few large items inside a safe margin, for goggles.
    GoggleSafe,
}

/// Which GPU to render on.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GpuPreference {
    /// Integrated GPU: drives the laptop panel directly, no cross-adapter
    /// copy, ample for this workload.
    #[default]
    LowPower,
    /// Discrete GPU.
    HighPerformance,
}

/// Fullscreen flavour.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FullscreenMode {
    /// Window covering the monitor at its current mode.
    #[default]
    Borderless,
    /// Change the monitor mode (used for HD3 1280x720 when the desktop
    /// runs the HDMI output at another mode).
    Exclusive,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DisplaySettings {
    pub present_mode: PresentModeSetting,
    pub present_trigger: PresentTrigger,
    /// Lines per slice for [`PresentTrigger::Lines`].
    pub present_lines: u32,
    /// Upper bound on presents per second (0 = no limit).
    pub max_present_hz: u32,
    /// Swap-chain depth hint (1 = lowest latency).
    pub max_frame_latency: u32,
    pub aspect: AspectMode,
    pub output_aspect: OutputAspect,
    /// Cropped from each edge, percent of the picture size.
    pub overscan_percent: f32,
    pub scaling: ScalingFilter,
    /// Luma unsharp-mask amount, 0 = off.
    pub sharpen: f32,
    /// Display-only adjustments.
    pub brightness: f32,
    pub contrast: f32,
    pub saturation: f32,
    pub fullscreen: bool,
    pub fullscreen_mode: FullscreenMode,
    /// Monitor index in the system list (None = the window's monitor).
    pub monitor: Option<usize>,
    pub hd3_mode: bool,
    /// In HD3 mode, stretch the picture over the whole 16:9 output.
    pub hd3_stretch: bool,
    pub osd: bool,
    pub osd_layout: OsdLayout,
    pub ui_scale: f32,
    /// OSD text refresh rate when nothing else asks for a redraw.
    pub osd_refresh_hz: f32,
    /// Thin strip coloured by the receiver feeding each row.
    pub row_strip: bool,
    /// Tint samples the damage mask flags (diagnostics).
    pub damage_overlay: bool,
    pub layout: ViewLayout,
    pub gpu: GpuPreference,
}

impl Default for DisplaySettings {
    fn default() -> Self {
        DisplaySettings {
            present_mode: PresentModeSetting::Mailbox,
            present_trigger: PresentTrigger::Field,
            present_lines: 32,
            max_present_hz: 0,
            max_frame_latency: 1,
            aspect: AspectMode::Fit,
            output_aspect: OutputAspect::Auto,
            overscan_percent: 1.5,
            scaling: ScalingFilter::Bicubic,
            sharpen: 0.15,
            brightness: 0.0,
            contrast: 1.0,
            saturation: 1.0,
            fullscreen: false,
            fullscreen_mode: FullscreenMode::Borderless,
            monitor: None,
            hd3_mode: false,
            hd3_stretch: false,
            osd: true,
            osd_layout: OsdLayout::Full,
            ui_scale: 1.0,
            osd_refresh_hz: 15.0,
            row_strip: true,
            damage_overlay: false,
            layout: ViewLayout::Single,
            gpu: GpuPreference::LowPower,
        }
    }
}

/// Display parameters after applying the HD3 preset.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EffectiveDisplay {
    pub aspect: AspectMode,
    pub output_aspect: OutputAspect,
    pub osd_layout: OsdLayout,
    pub ui_scale: f32,
    pub fullscreen: bool,
}

impl DisplaySettings {
    /// The HD3 preset overrides a few display parameters without touching
    /// the user's normal values, so toggling it off restores them.
    pub fn effective(&self) -> EffectiveDisplay {
        if self.hd3_mode {
            EffectiveDisplay {
                aspect: if self.hd3_stretch {
                    AspectMode::Stretch
                } else if self.aspect == AspectMode::Stretch {
                    AspectMode::Fit
                } else {
                    self.aspect
                },
                output_aspect: OutputAspect::Hd3Panel5x3,
                osd_layout: OsdLayout::GoggleSafe,
                ui_scale: (self.ui_scale * 1.5).clamp(1.0, 3.0),
                fullscreen: true,
            }
        } else {
            EffectiveDisplay {
                aspect: self.aspect,
                output_aspect: self.output_aspect,
                osd_layout: self.osd_layout,
                ui_scale: self.ui_scale,
                fullscreen: self.fullscreen,
            }
        }
    }
}

// ---------------------------------------------------------------------
// Video pipeline
// ---------------------------------------------------------------------

/// Named pipeline presets (`openvrx_video::VideoConfig` presets).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VideoPreset {
    /// Nothing but demodulation, sync, blend and bob: the honest picture.
    Raw,
    /// 2D comb, repair, edge-directed deinterlace, no temporal filter.
    #[default]
    LowLatency,
    /// Low latency plus a light motion-gated recursive denoise.
    Balanced,
    /// 3D comb / transform, motion-compensated denoise (past only).
    Quality,
}

impl VideoPreset {
    pub const ALL: [VideoPreset; 4] = [
        VideoPreset::Raw,
        VideoPreset::LowLatency,
        VideoPreset::Balanced,
        VideoPreset::Quality,
    ];
    /// Name used by `openvrx_video::VideoConfig::preset`.
    pub fn name(self) -> &'static str {
        match self {
            VideoPreset::Raw => "raw",
            VideoPreset::LowLatency => "low_latency",
            VideoPreset::Balanced => "balanced",
            VideoPreset::Quality => "quality",
        }
    }
}

/// Video standard selection.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StandardChoice {
    /// Follow the standard the DSP detects.
    #[default]
    Auto,
    Ntsc,
    Pal,
}

impl StandardChoice {
    pub const ALL: [StandardChoice; 3] =
        [StandardChoice::Auto, StandardChoice::Ntsc, StandardChoice::Pal];
    pub fn fixed(self) -> Option<VideoStandard> {
        match self {
            StandardChoice::Auto => None,
            StandardChoice::Ntsc => Some(VideoStandard::Ntsc),
            StandardChoice::Pal => Some(VideoStandard::Pal),
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            StandardChoice::Auto => "Auto",
            StandardChoice::Ntsc => "NTSC",
            StandardChoice::Pal => "PAL",
        }
    }
}

/// Diversity combining stage.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiversityMode {
    /// D1: best line.
    Select,
    /// D2: weighted blend per line.
    #[default]
    Blend,
    /// D3: blend plus per-sample repair from the other receiver.
    SampleRepair,
}

impl DiversityMode {
    pub const ALL: [DiversityMode; 3] =
        [DiversityMode::Select, DiversityMode::Blend, DiversityMode::SampleRepair];
    pub fn name(self) -> &'static str {
        match self {
            DiversityMode::Select => "D1 select",
            DiversityMode::Blend => "D2 blend",
            DiversityMode::SampleRepair => "D3 per-sample",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VideoSettings {
    pub preset: VideoPreset,
    pub standard: StandardChoice,
    pub diversity: DiversityMode,
    /// Master cleanup switch (Space). Off = raw view whatever the preset.
    pub cleanup: bool,
    /// Steady OSD (I): every field on the same rows, as goggles show it.
    /// The flight controller OSD stops jumping a row per field; the camera
    /// picture loses its half-line placement.
    pub steady_osd: bool,
}

impl Default for VideoSettings {
    fn default() -> Self {
        VideoSettings {
            preset: VideoPreset::LowLatency,
            standard: StandardChoice::Auto,
            diversity: DiversityMode::Blend,
            cleanup: true,
            steady_osd: true,
        }
    }
}

impl VideoSettings {
    /// The preset actually running (Raw when cleanup is switched off).
    pub fn effective_preset(&self) -> VideoPreset {
        if self.cleanup {
            self.preset
        } else {
            VideoPreset::Raw
        }
    }

    /// True when picture cleanup is active (safety rule: the C/N bars
    /// must then stay visible).
    pub fn cleanup_active(&self) -> bool {
        self.effective_preset() != VideoPreset::Raw
    }
}

// ---------------------------------------------------------------------
// Receivers
// ---------------------------------------------------------------------

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum GainMode {
    /// Host gain loop (slow, hysteresis, changes in vertical blanking).
    #[default]
    Auto,
    /// Fixed gain index.
    Manual { index: u8 },
}

/// How the host reaches a receiver's C5.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    /// Only the FX2 is on USB. The C5 is powered on its own and runs the
    /// settings stored with its `save` command; `lanes` and `divider` here
    /// must match them. Channel and gain are whatever the C5 stored.
    #[default]
    Fx2Only,
    /// The C5 console is on USB too: the host sets channel, gain, lanes and
    /// clock, calibrates the sampling phase and runs the lane self-test.
    C5Console,
}

/// Gain index the C5 starts from (and stays at in manual mode until set).
pub const DEFAULT_GAIN: u8 = 64;

/// Usable PARLIO dividers: 240 MHz / divider.
pub const DIVIDERS: [u8; 4] = [6, 9, 12, 18];

/// Sample clock for a divider, Hz.
pub fn divider_hz(divider: u8) -> f64 {
    240e6 / divider.max(1) as f64
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ReceiverSettings {
    pub enabled: bool,
    pub name: String,
    pub freq_mhz: u32,
    pub gain: GainMode,
    pub board: BoardKind,
    pub lanes: LanePreset,
    pub divider: u8,
    pub control: ControlMode,
    /// FX2 only: the channel the C5 saved, when given with `--mhz` (the app
    /// cannot read it). Not stored: the C5 can be retuned between runs.
    #[serde(skip)]
    pub fx2_only_mhz: Option<u32>,
    /// C5 console port (`COM7`); None pairs automatically.
    pub c5_port: Option<String>,
    /// FX2 serial (8 hex digits); None pairs automatically.
    pub fx2_serial: Option<String>,
}

impl Default for ReceiverSettings {
    fn default() -> Self {
        ReceiverSettings {
            enabled: true,
            name: String::new(),
            freq_mhz: 5658,
            gain: GainMode::Auto,
            board: BoardKind::Xiao,
            lanes: LanePreset::I7Q7,
            divider: 18,
            control: ControlMode::default(),
            fx2_only_mhz: None,
            c5_port: None,
            fx2_serial: None,
        }
    }
}

// ---------------------------------------------------------------------
// Recording, views
// ---------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RecordingSettings {
    pub folder: PathBuf,
}

impl Default for RecordingSettings {
    fn default() -> Self {
        RecordingSettings {
            folder: default_recording_folder(),
        }
    }
}

fn default_recording_folder() -> PathBuf {
    match std::env::var_os("USERPROFILE").or_else(|| std::env::var_os("HOME")) {
        Some(home) => PathBuf::from(home).join("Videos").join("OpenVRX"),
        None => PathBuf::from("recordings"),
    }
}

/// Built-in test patterns.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TestPattern {
    /// Scrolling bars, zone plate and a bouncing box together.
    #[default]
    Mix,
    /// Animated (scrolling) 75 % colour bars with a grey ramp.
    Bars,
    /// Moving circular zone plate.
    ZonePlate,
    /// Bouncing boxes on grey.
    MovingBox,
    /// Fast vertical bar: shows tearing and slice presentation.
    Sweep,
    /// The bench ESP32's NTSC test patterns (tools/bench/att_ctl), drawn
    /// exactly as it sends them: the reference for side-by-side comparison.
    BenchCard,
    BenchBars,
    BenchRamp,
    BenchCheck,
    BenchGrey,
    BenchBlack,
    BenchWhite,
    /// The colour card and colour bars of the resistor-ladder output
    /// (tools/bench/att_ctl/ladder.cpp, the default since 2026-11-04); its
    /// other patterns match the DAC ones above.
    BenchColourCard,
    BenchColourBars,
}

impl TestPattern {
    #[cfg(test)]
    pub const ALL: [TestPattern; 14] = [
        TestPattern::Mix,
        TestPattern::Bars,
        TestPattern::ZonePlate,
        TestPattern::MovingBox,
        TestPattern::Sweep,
        TestPattern::BenchCard,
        TestPattern::BenchBars,
        TestPattern::BenchRamp,
        TestPattern::BenchCheck,
        TestPattern::BenchGrey,
        TestPattern::BenchBlack,
        TestPattern::BenchWhite,
        TestPattern::BenchColourCard,
        TestPattern::BenchColourBars,
    ];
    pub fn name(self) -> &'static str {
        match self {
            TestPattern::Mix => "mix",
            TestPattern::Bars => "bars",
            TestPattern::ZonePlate => "zone plate",
            TestPattern::MovingBox => "moving box",
            TestPattern::Sweep => "sweep",
            TestPattern::BenchCard => "bench card",
            TestPattern::BenchBars => "bench bars",
            TestPattern::BenchRamp => "bench ramp",
            TestPattern::BenchCheck => "bench check",
            TestPattern::BenchGrey => "bench grey",
            TestPattern::BenchBlack => "bench black",
            TestPattern::BenchWhite => "bench white",
            TestPattern::BenchColourCard => "bench colour card",
            TestPattern::BenchColourBars => "bench colour bars",
        }
    }
    /// A bench pattern by its att_ctl name as the ESP32 output `out` draws
    /// it (`ladder`, the default, or `dac`, the old monochrome P25 output).
    pub fn bench_on(name: &str, out: &str) -> Option<TestPattern> {
        let p = TestPattern::bench(name)?;
        Some(match (p, out) {
            (TestPattern::BenchCard, "ladder") => TestPattern::BenchColourCard,
            (TestPattern::BenchBars, "ladder") => TestPattern::BenchColourBars,
            _ => p,
        })
    }
    /// A bench ESP32 pattern by its att_ctl name (card, bars, ramp, check,
    /// grey, black, white).
    pub fn bench(name: &str) -> Option<TestPattern> {
        Some(match name {
            "card" => TestPattern::BenchCard,
            "bars" => TestPattern::BenchBars,
            "ramp" => TestPattern::BenchRamp,
            "check" => TestPattern::BenchCheck,
            "grey" => TestPattern::BenchGrey,
            "black" => TestPattern::BenchBlack,
            "white" => TestPattern::BenchWhite,
            _ => return None,
        })
    }
}

/// What feeds a view.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ViewSource {
    /// Built-in generator with simulated field-by-field row updates.
    Test {
        pattern: TestPattern,
        standard: VideoStandard,
    },
    /// Combined output of these receivers (live backend).
    Receivers { rx: Vec<RxId> },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ViewSettings {
    pub source: ViewSource,
}

impl Default for ViewSettings {
    fn default() -> Self {
        ViewSettings {
            source: ViewSource::Test {
                pattern: TestPattern::Mix,
                standard: VideoStandard::Ntsc,
            },
        }
    }
}

fn default_views() -> Vec<ViewSettings> {
    let t = |pattern, standard| ViewSettings {
        source: ViewSource::Test { pattern, standard },
    };
    vec![
        t(TestPattern::Mix, VideoStandard::Ntsc),
        t(TestPattern::Bars, VideoStandard::Pal),
        t(TestPattern::ZonePlate, VideoStandard::Ntsc),
        t(TestPattern::MovingBox, VideoStandard::Pal),
    ]
}

fn default_receivers() -> Vec<ReceiverSettings> {
    (0..2)
        .map(|k| ReceiverSettings {
            name: format!("rx{k}"),
            ..ReceiverSettings::default()
        })
        .collect()
}

// ---------------------------------------------------------------------
// Root
// ---------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppSettings {
    pub version: u32,
    pub display: DisplaySettings,
    pub video: VideoSettings,
    pub receivers: Vec<ReceiverSettings>,
    pub recording: RecordingSettings,
    pub views: Vec<ViewSettings>,
}

impl Default for AppSettings {
    fn default() -> Self {
        AppSettings {
            version: SETTINGS_VERSION,
            display: DisplaySettings::default(),
            video: VideoSettings::default(),
            receivers: default_receivers(),
            recording: RecordingSettings::default(),
            views: default_views(),
        }
    }
}

impl AppSettings {
    /// Clamps every value into its valid range and fills missing slots.
    pub fn sanitize(&mut self) {
        self.version = SETTINGS_VERSION;
        let d = &mut self.display;
        d.present_lines = d.present_lines.clamp(4, 288);
        d.max_present_hz = d.max_present_hz.min(5000);
        d.max_frame_latency = d.max_frame_latency.clamp(1, 3);
        d.overscan_percent = finite_or(d.overscan_percent, 0.0).clamp(0.0, 15.0);
        d.sharpen = finite_or(d.sharpen, 0.0).clamp(0.0, 2.0);
        d.brightness = finite_or(d.brightness, 0.0).clamp(-0.5, 0.5);
        d.contrast = finite_or(d.contrast, 1.0).clamp(0.25, 3.0);
        d.saturation = finite_or(d.saturation, 1.0).clamp(0.0, 3.0);
        d.ui_scale = finite_or(d.ui_scale, 1.0).clamp(0.5, 3.0);
        d.osd_refresh_hz = finite_or(d.osd_refresh_hz, 15.0).clamp(1.0, 120.0);

        if self.receivers.is_empty() {
            self.receivers = default_receivers();
        }
        self.receivers.truncate(MAX_RECEIVERS);
        for (k, r) in self.receivers.iter_mut().enumerate() {
            r.freq_mhz = crate::channels::clamp_mhz(r.freq_mhz);
            if !DIVIDERS.contains(&r.divider) {
                r.divider = 18;
            }
            if r.name.trim().is_empty() {
                r.name = format!("rx{k}");
            }
        }
        let defaults = default_views();
        self.views.truncate(VIEW_SLOTS);
        while self.views.len() < VIEW_SLOTS {
            self.views.push(defaults[self.views.len()].clone());
        }
    }

    /// Parses settings JSON, sanitising the result.
    pub fn from_json(text: &str) -> Result<AppSettings, serde_json::Error> {
        let mut s: AppSettings = serde_json::from_str(text)?;
        s.sanitize();
        Ok(s)
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("settings always serialise")
    }
}

fn finite_or(x: f32, fallback: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        fallback
    }
}

fn cycle<T: Copy + PartialEq>(all: &[T], cur: T) -> T {
    let k = all.iter().position(|&x| x == cur).unwrap_or(0);
    all[(k + 1) % all.len()]
}

// ---------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------

/// `%APPDATA%\OpenVRX\settings.json` (or `$HOME/.config/openvrx` elsewhere).
pub fn default_settings_path() -> PathBuf {
    if let Some(appdata) = std::env::var_os("APPDATA") {
        return PathBuf::from(appdata).join("OpenVRX").join("settings.json");
    }
    if let Some(home) = std::env::var_os("HOME") {
        return PathBuf::from(home)
            .join(".config")
            .join("openvrx")
            .join("settings.json");
    }
    PathBuf::from("openvrx-settings.json")
}

/// Result of loading the settings file.
#[derive(Debug)]
pub struct Loaded {
    pub settings: AppSettings,
    /// A note for the user (file created, corrupt file moved aside...).
    pub note: Option<String>,
}

/// Loads settings; never fails (falls back to defaults).
pub fn load(path: &Path) -> Loaded {
    match std::fs::read_to_string(path) {
        Ok(text) => match AppSettings::from_json(&text) {
            Ok(settings) => Loaded {
                settings,
                note: None,
            },
            Err(e) => {
                let bad = path.with_extension("json.bad");
                let moved = std::fs::rename(path, &bad).is_ok();
                Loaded {
                    settings: AppSettings::default(),
                    note: Some(if moved {
                        format!(
                            "settings file was unreadable ({e}); moved to {} and using defaults",
                            bad.display()
                        )
                    } else {
                        format!("settings file was unreadable ({e}); using defaults")
                    }),
                }
            }
        },
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Loaded {
            settings: AppSettings::default(),
            note: None,
        },
        Err(e) => Loaded {
            settings: AppSettings::default(),
            note: Some(format!("cannot read {}: {e}; using defaults", path.display())),
        },
    }
}

/// Writes settings atomically (temporary file, then rename).
pub fn save(path: &Path, settings: &AppSettings) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        if !dir.as_os_str().is_empty() {
            std::fs::create_dir_all(dir)?;
        }
    }
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, settings.to_json())?;
    std::fs::rename(&tmp, path)
}

enum SaveMsg {
    Save(Box<AppSettings>),
    Flush(mpsc::Sender<()>),
}

/// Debounced background writer.
pub struct SettingsStore {
    path: PathBuf,
    tx: Option<mpsc::Sender<SaveMsg>>,
    thread: Option<JoinHandle<()>>,
    last: AppSettings,
}

/// Time after the last change before the file is written.
pub const SAVE_DEBOUNCE: Duration = Duration::from_millis(500);

impl SettingsStore {
    pub fn new(path: PathBuf, current: &AppSettings) -> SettingsStore {
        let (tx, rx) = mpsc::channel::<SaveMsg>();
        let p = path.clone();
        let thread = std::thread::Builder::new()
            .name("settings-save".into())
            .spawn(move || saver(p, rx))
            .ok();
        SettingsStore {
            path,
            tx: thread.as_ref().map(|_| tx),
            thread,
            last: current.clone(),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Queues a save when the settings differ from the last saved ones.
    pub fn update(&mut self, settings: &AppSettings) {
        if *settings == self.last {
            return;
        }
        self.last = settings.clone();
        match &self.tx {
            Some(tx) => {
                let _ = tx.send(SaveMsg::Save(Box::new(settings.clone())));
            }
            None => {
                if let Err(e) = save(&self.path, settings) {
                    log::warn!("saving settings: {e}");
                }
            }
        }
    }

    /// Writes any pending change now and waits for it.
    pub fn flush(&mut self) {
        if let Some(tx) = &self.tx {
            let (done_tx, done_rx) = mpsc::channel();
            if tx.send(SaveMsg::Flush(done_tx)).is_ok() {
                let _ = done_rx.recv_timeout(Duration::from_secs(5));
            }
        }
    }
}

impl Drop for SettingsStore {
    fn drop(&mut self) {
        self.flush();
        self.tx = None;
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn saver(path: PathBuf, rx: mpsc::Receiver<SaveMsg>) {
    let mut pending: Option<Box<AppSettings>> = None;
    let write = |s: &AppSettings| {
        if let Err(e) = save(&path, s) {
            log::warn!("saving settings to {}: {e}", path.display());
        }
    };
    loop {
        let msg = if pending.is_some() {
            match rx.recv_timeout(SAVE_DEBOUNCE) {
                Ok(m) => Some(m),
                Err(mpsc::RecvTimeoutError::Timeout) => None,
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        } else {
            match rx.recv() {
                Ok(m) => Some(m),
                Err(_) => break,
            }
        };
        match msg {
            Some(SaveMsg::Save(s)) => pending = Some(s),
            Some(SaveMsg::Flush(done)) => {
                if let Some(s) = pending.take() {
                    write(&s);
                }
                let _ = done.send(());
            }
            None => {
                if let Some(s) = pending.take() {
                    write(&s);
                }
            }
        }
    }
    if let Some(s) = pending.take() {
        write(&s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "openvrx-settings-test-{name}-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn defaults_are_sane() {
        let mut s = AppSettings::default();
        let before = s.clone();
        s.sanitize();
        assert_eq!(s, before, "defaults must already be sanitised");
        assert_eq!(s.views.len(), VIEW_SLOTS);
        assert_eq!(s.receivers.len(), 2);
        assert_eq!(s.display.max_frame_latency, 1);
        assert!(s.video.cleanup_active());
    }

    #[test]
    fn json_roundtrip() {
        let mut s = AppSettings::default();
        s.display.present_mode = PresentModeSetting::Immediate;
        s.display.present_trigger = PresentTrigger::Lines;
        s.receivers[1].gain = GainMode::Manual { index: 64 };
        s.receivers[1].lanes = LanePreset::I4Q4Ultra;
        s.views[2].source = ViewSource::Receivers { rx: vec![0, 1] };
        let text = s.to_json();
        assert!(text.contains("\"immediate\""));
        assert!(text.contains("\"4x4ultra\""));
        let back = AppSettings::from_json(&text).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn partial_and_unknown_fields() {
        let text = r#"{"display": {"present_mode": "fifo", "future_option": 3}, "extra": true}"#;
        let s = AppSettings::from_json(text).unwrap();
        assert_eq!(s.display.present_mode, PresentModeSetting::Fifo);
        assert_eq!(s.display.present_trigger, PresentTrigger::Field);
        assert_eq!(s.views.len(), VIEW_SLOTS);
        assert_eq!(s.receivers.len(), 2);
    }

    #[test]
    fn sanitize_clamps() {
        let mut s = AppSettings::default();
        s.display.overscan_percent = 80.0;
        s.display.contrast = f32::NAN;
        s.display.max_frame_latency = 0;
        s.receivers[0].freq_mhz = 9000;
        s.receivers[0].divider = 7;
        s.views.truncate(1);
        s.sanitize();
        assert_eq!(s.display.overscan_percent, 15.0);
        assert_eq!(s.display.contrast, 1.0);
        assert_eq!(s.display.max_frame_latency, 1);
        assert_eq!(s.receivers[0].freq_mhz, crate::channels::MAX_MHZ);
        assert_eq!(s.receivers[0].divider, 18);
        assert_eq!(s.views.len(), VIEW_SLOTS);
    }

    #[test]
    fn hd3_preset_overrides_without_overwriting() {
        let mut d = DisplaySettings {
            aspect: AspectMode::Fill,
            ..DisplaySettings::default()
        };
        assert_eq!(d.effective().aspect, AspectMode::Fill);
        d.hd3_mode = true;
        let e = d.effective();
        assert_eq!(e.output_aspect, OutputAspect::Hd3Panel5x3);
        assert_eq!(e.osd_layout, OsdLayout::GoggleSafe);
        assert!(e.fullscreen);
        assert_eq!(e.aspect, AspectMode::Fill);
        d.hd3_stretch = true;
        assert_eq!(d.effective().aspect, AspectMode::Stretch);
        d.hd3_mode = false;
        assert_eq!(d.effective().aspect, AspectMode::Fill);
    }

    #[test]
    fn cleanup_switch() {
        let mut v = VideoSettings {
            preset: VideoPreset::Quality,
            ..VideoSettings::default()
        };
        assert!(v.cleanup_active());
        assert_eq!(v.effective_preset(), VideoPreset::Quality);
        v.cleanup = false;
        assert!(!v.cleanup_active());
        assert_eq!(v.effective_preset(), VideoPreset::Raw);
        v.cleanup = true;
        v.preset = VideoPreset::Raw;
        assert!(!v.cleanup_active());
        for p in VideoPreset::ALL {
            let cfg = openvrx_video::VideoConfig::preset(p.name(), VideoStandard::Ntsc);
            assert!(cfg.is_some(), "{}", p.name());
        }
    }

    #[test]
    fn cycling() {
        assert_eq!(PresentModeSetting::Fifo.next(), PresentModeSetting::Immediate);
        assert_eq!(ViewLayout::Single.next(), ViewLayout::Dual);
        assert_eq!(ViewLayout::Quad.next(), ViewLayout::Single);
        assert_eq!(ScalingFilter::Lanczos2.next(), ScalingFilter::Nearest);
    }

    #[test]
    fn load_missing_corrupt_and_save() {
        let dir = temp_dir("io");
        let path = dir.join("sub").join("settings.json");
        let l = load(&path);
        assert!(l.note.is_none());
        assert_eq!(l.settings, AppSettings::default());

        let mut s = AppSettings::default();
        s.display.sharpen = 0.5;
        save(&path, &s).unwrap();
        assert_eq!(load(&path).settings, s);

        std::fs::write(&path, "{ not json").unwrap();
        let l = load(&path);
        assert!(l.note.unwrap().contains("unreadable"));
        assert!(path.with_extension("json.bad").exists());
        assert!(!path.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn store_debounces_and_flushes() {
        let dir = temp_dir("store");
        let path = dir.join("settings.json");
        let mut s = AppSettings::default();
        let mut store = SettingsStore::new(path.clone(), &s);
        store.update(&s);
        assert!(!path.exists(), "unchanged settings are not written");
        s.display.brightness = 0.1;
        store.update(&s);
        s.display.brightness = 0.2;
        store.update(&s);
        store.flush();
        assert_eq!(load(&path).settings.display.brightness, 0.2);
        s.display.brightness = 0.3;
        store.update(&s);
        drop(store);
        assert_eq!(load(&path).settings.display.brightness, 0.3);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn divider_rates() {
        assert!((divider_hz(18) - 13.333e6).abs() < 1e3);
        assert_eq!(divider_hz(6), 40e6);
    }
}
