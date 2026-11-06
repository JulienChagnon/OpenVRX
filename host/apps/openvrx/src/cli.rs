// SPDX-License-Identifier: GPL-3.0-or-later
//! Command line.

use crate::settings::{AppSettings, ControlMode, PresentModeSetting, ViewLayout, ViewSource, DIVIDERS, VIEW_SLOTS};
use clap::{Parser, ValueEnum};
use openvrx_core::{LanePreset, RxId};
use std::path::PathBuf;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum SourceKind {
    /// Built-in test patterns with simulated receivers.
    Test,
    /// FX2/C5 receivers.
    Live,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum PresentArg {
    Immediate,
    Mailbox,
    Fifo,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum ControlArg {
    /// Only the FX2 on USB; the C5 runs its saved settings.
    Fx2Only,
    /// C5 console on USB as well (host sets channel, gain, lanes, clock).
    C5,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum LayoutArg {
    #[value(name = "1")]
    One,
    #[value(name = "2")]
    Two,
    #[value(name = "4")]
    Four,
}

/// OpenVRX live receiver.
#[derive(Debug, Parser)]
#[command(name = "openvrx", version, about)]
pub struct Args {
    /// Start fullscreen.
    #[arg(long)]
    pub fullscreen: bool,
    /// HD3 goggle mode (fullscreen, 5:3 output geometry, goggle-safe OSD).
    #[arg(long)]
    pub hd3: bool,
    /// Picture source.
    #[arg(long, value_enum, default_value_t = SourceKind::Live)]
    pub source: SourceKind,
    /// Swap-chain present mode.
    #[arg(long, value_enum)]
    pub present: Option<PresentArg>,
    /// Number of views.
    #[arg(long, value_enum)]
    pub layout: Option<LayoutArg>,
    /// Settings file (default %APPDATA%\OpenVRX\settings.json).
    #[arg(long)]
    pub settings: Option<PathBuf>,
    /// Run this many seconds, print latency statistics and exit.
    #[arg(long, value_name = "SECONDS")]
    pub run_for: Option<f64>,
    /// Bench ESP32 pattern shown beside the live picture by key X: card,
    /// bars, ramp, check, grey, black, white.
    #[arg(long, value_name = "PATTERN", default_value = "card",
          value_parser = ["card", "bars", "ramp", "check", "grey", "black", "white"])]
    pub compare_pattern: String,
    /// The ESP32's video output the comparison draws: `ladder` (colour,
    /// att_ctl's default) or `dac` (the old monochrome P25 patterns).
    #[arg(long, value_name = "OUT", default_value = "ladder", value_parser = ["ladder", "dac"])]
    pub compare_out: String,
    /// Start in the side-by-side comparison (key X toggles it).
    #[arg(long)]
    pub compare: bool,
    /// Take bench commands on stdin (record FOLDER, stop, compare, quit)
    /// and print backend messages on stdout (tools/bench/bench.py --live).
    #[arg(long)]
    pub bench_control: bool,
    /// Render a test pattern offscreen through the display shader to this
    /// PNG and exit.
    #[arg(long, value_name = "PNG")]
    pub snapshot: Option<PathBuf>,
    /// How receivers are controlled (default from settings: FX2 only).
    #[arg(long, value_enum)]
    pub control: Option<ControlArg>,
    /// Lane preset of every receiver (8x8, 7x7, 6x6, 5x5, 4x4top, 4x4fine,
    /// 4x4ultra, angle8). In FX2-only mode it must match the C5's saved
    /// `lanes`.
    #[arg(long, value_parser = parse_lanes)]
    pub lanes: Option<LanePreset>,
    /// Sample clock divider of every receiver (6, 9, 12 or 18). In FX2-only
    /// mode it must match the C5's saved `clk`.
    #[arg(long, value_parser = parse_divider)]
    pub clk: Option<u8>,
    /// Channel frequency in MHz. With the C5 console it tunes the C5; in
    /// FX2-only mode it only labels the OSD and recordings with the channel
    /// the C5 saved (the app cannot read it).
    #[arg(long, value_name = "MHZ", value_parser = clap::value_parser!(u32).range(5000..=6100))]
    pub mhz: Option<u32>,
}

fn parse_lanes(s: &str) -> Result<LanePreset, String> {
    LanePreset::ALL
        .into_iter()
        .find(|p| p.name().eq_ignore_ascii_case(s))
        .ok_or_else(|| {
            let names: Vec<&str> = LanePreset::ALL.iter().map(|p| p.name()).collect();
            format!("unknown lane preset `{s}` (one of {})", names.join(", "))
        })
}

fn parse_divider(s: &str) -> Result<u8, String> {
    s.parse::<u8>()
        .ok()
        .filter(|d| DIVIDERS.contains(d))
        .ok_or_else(|| format!("divider must be one of {DIVIDERS:?}"))
}

impl Args {
    /// Applies the command line on top of the loaded settings.
    pub fn apply(&self, s: &mut AppSettings) {
        if self.fullscreen {
            s.display.fullscreen = true;
        }
        if self.hd3 {
            s.display.hd3_mode = true;
        }
        if let Some(p) = self.present {
            s.display.present_mode = match p {
                PresentArg::Immediate => PresentModeSetting::Immediate,
                PresentArg::Mailbox => PresentModeSetting::Mailbox,
                PresentArg::Fifo => PresentModeSetting::Fifo,
            };
        }
        if let Some(l) = self.layout {
            s.display.layout = match l {
                LayoutArg::One => ViewLayout::Single,
                LayoutArg::Two => ViewLayout::Dual,
                LayoutArg::Four => ViewLayout::Quad,
            };
        }
        for r in &mut s.receivers {
            if let Some(c) = self.control {
                r.control = match c {
                    ControlArg::Fx2Only => ControlMode::Fx2Only,
                    ControlArg::C5 => ControlMode::C5Console,
                };
            }
            if let Some(l) = self.lanes {
                r.lanes = l;
            }
            if let Some(d) = self.clk {
                r.divider = d;
            }
            if let Some(m) = self.mhz {
                r.freq_mhz = m;
                r.fx2_only_mhz = Some(m);
            }
        }
        match self.source {
            SourceKind::Live => {
                // View 1 combines every receiver; the others show one
                // receiver each (multi-quad viewing on several channels).
                let all: Vec<RxId> = (0..s.receivers.len() as RxId).collect();
                for v in 0..VIEW_SLOTS {
                    let rx = if v == 0 { all.clone() } else { vec![v as RxId] };
                    s.views[v].source = ViewSource::Receivers { rx };
                }
            }
            SourceKind::Test => {
                let defaults = AppSettings::default().views;
                for (v, view) in s.views.iter_mut().enumerate() {
                    if !matches!(view.source, ViewSource::Test { .. }) {
                        view.source = defaults[v].source.clone();
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::ReceiverSettings;

    #[test]
    fn parse_and_apply() {
        let a = Args::try_parse_from([
            "openvrx",
            "--hd3",
            "--present",
            "immediate",
            "--source",
            "live",
            "--layout",
            "2",
        ])
        .unwrap();
        let mut s = AppSettings::default();
        a.apply(&mut s);
        assert!(s.display.hd3_mode);
        assert_eq!(s.display.present_mode, PresentModeSetting::Immediate);
        assert_eq!(s.display.layout, ViewLayout::Dual);
        assert_eq!(s.views[0].source, ViewSource::Receivers { rx: vec![0, 1] });
        assert_eq!(s.views[2].source, ViewSource::Receivers { rx: vec![2] });

        let t = Args::try_parse_from(["openvrx", "--source", "test"]).unwrap();
        t.apply(&mut s);
        assert!(matches!(s.views[0].source, ViewSource::Test { .. }));
        assert!(Args::try_parse_from(["openvrx", "--present", "vsync"]).is_err());

        let h = Args::try_parse_from(["openvrx", "--control", "c5", "--lanes", "4x4TOP", "--clk", "9"]).unwrap();
        h.apply(&mut s);
        assert!(s.receivers.iter().all(|r| r.control == ControlMode::C5Console
            && r.lanes == LanePreset::I4Q4Top
            && r.divider == 9));
        assert!(Args::try_parse_from(["openvrx", "--clk", "7"]).is_err());
        assert!(Args::try_parse_from(["openvrx", "--lanes", "9x9"]).is_err());
        assert_eq!(ReceiverSettings::default().control, ControlMode::Fx2Only);
    }
}
