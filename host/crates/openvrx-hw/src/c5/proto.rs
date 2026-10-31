// SPDX-License-Identifier: GPL-3.0-or-later
//! C5 console line protocol (PROTOCOL.md section 1): line classification,
//! `OK`/`ERR` reply parsing, events, typed replies and the settings the
//! host owns and pushes on connect.

use crate::error::{HwError, Result};
use openvrx_core::LanePreset;
use serde::{Deserialize, Serialize};
use std::str::FromStr;

/// Longest command line the C5 accepts (without the newline).
pub const MAX_LINE: usize = 120;
/// Protocol version this host implements.
pub const PROTO_VERSION: u32 = 1;

/// `key=value` pairs in order; bare tokens have an empty value.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fields(pub Vec<(String, String)>);

impl Fields {
    fn parse(tokens: &[&str]) -> Fields {
        Fields(
            tokens
                .iter()
                .map(|t| match t.split_once('=') {
                    Some((k, v)) => (k.to_string(), v.to_string()),
                    None => (t.to_string(), String::new()),
                })
                .collect(),
        )
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    /// Parses a field, with an error naming the command and key.
    pub fn parse_as<T: FromStr>(&self, cmd: &str, key: &str) -> Result<T> {
        let v = self
            .get(key)
            .ok_or_else(|| HwError::Protocol(format!("`{cmd}` reply has no `{key}`")))?;
        v.parse()
            .map_err(|_| HwError::Protocol(format!("`{cmd}` reply: `{key}={v}` is not valid")))
    }

    pub fn flag(&self, cmd: &str, key: &str) -> Result<bool> {
        match self.get(key) {
            Some("1") | Some("on") | Some("true") => Ok(true),
            Some("0") | Some("off") | Some("false") => Ok(false),
            Some(v) => Err(HwError::Protocol(format!("`{cmd}` reply: `{key}={v}` is not a flag"))),
            None => Err(HwError::Protocol(format!("`{cmd}` reply has no `{key}`"))),
        }
    }
}

/// An `OK` reply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Reply {
    /// Echoed command word.
    pub command: String,
    pub fields: Fields,
}

/// `ERR` codes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErrCode {
    Syntax,
    Range,
    State,
    Unsupported,
    Hw,
    Other(String),
}

impl ErrCode {
    fn parse(s: &str) -> ErrCode {
        match s {
            "syntax" => ErrCode::Syntax,
            "range" => ErrCode::Range,
            "state" => ErrCode::State,
            "unsupported" => ErrCode::Unsupported,
            "hw" => ErrCode::Hw,
            o => ErrCode::Other(o.to_string()),
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            ErrCode::Syntax => "syntax",
            ErrCode::Range => "range",
            ErrCode::State => "state",
            ErrCode::Unsupported => "unsupported",
            ErrCode::Hw => "hw",
            ErrCode::Other(s) => s,
        }
    }
}

/// An `ERR` reply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ErrReply {
    pub command: String,
    pub code: ErrCode,
    pub text: String,
}

impl ErrReply {
    pub fn into_error(self) -> HwError {
        HwError::C5 {
            command: self.command,
            code: self.code.as_str().to_string(),
            text: self.text,
        }
    }
}

/// An `EVT` line.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Event {
    pub name: String,
    pub fields: Fields,
}

/// One line from the C5, classified.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Line {
    Ok(Reply),
    Err(ErrReply),
    Evt(Event),
    Log(String),
    /// Anything else (boot banner, panic output): kept for the session log.
    Other(String),
}

/// Classifies one received line (without its terminator).
pub fn parse_line(raw: &str) -> Line {
    let line = raw.trim_end_matches(['\r', '\n']);
    let mut it = line.splitn(2, ' ');
    let head = it.next().unwrap_or("");
    let rest = it.next().unwrap_or("");
    match head {
        "OK" => {
            let toks: Vec<&str> = rest.split(' ').filter(|t| !t.is_empty()).collect();
            match toks.split_first() {
                Some((cmd, tail)) => Line::Ok(Reply {
                    command: cmd.to_string(),
                    fields: Fields::parse(tail),
                }),
                None => Line::Other(line.to_string()),
            }
        }
        "ERR" => {
            let mut p = rest.splitn(3, ' ');
            let cmd = p.next().unwrap_or("");
            if cmd.is_empty() {
                return Line::Other(line.to_string());
            }
            Line::Err(ErrReply {
                command: cmd.to_string(),
                code: ErrCode::parse(p.next().unwrap_or("")),
                text: p.next().unwrap_or("").to_string(),
            })
        }
        "EVT" => {
            let toks: Vec<&str> = rest.split(' ').filter(|t| !t.is_empty()).collect();
            match toks.split_first() {
                Some((name, tail)) => Line::Evt(Event {
                    name: name.to_string(),
                    fields: Fields::parse(tail),
                }),
                None => Line::Other(line.to_string()),
            }
        }
        "LOG" => Line::Log(rest.to_string()),
        _ => Line::Other(line.to_string()),
    }
}

/// Command word of a command line (`clk off` -> `clk`).
pub fn command_word(line: &str) -> &str {
    line.split(' ').next().unwrap_or("")
}

/// Validates a command line before sending.
pub fn check_command(line: &str) -> Result<()> {
    if line.is_empty() {
        return Err(HwError::Invalid("empty command".into()));
    }
    if line.len() > MAX_LINE {
        return Err(HwError::Invalid(format!("command longer than {MAX_LINE} characters")));
    }
    if line.contains(['\n', '\r']) || !line.is_ascii() {
        return Err(HwError::Invalid("command must be one ASCII line".into()));
    }
    if line.contains("  ") || line.starts_with(' ') || line.ends_with(' ') {
        return Err(HwError::Invalid("tokens must be separated by single spaces".into()));
    }
    Ok(())
}

/// `ver` reply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerInfo {
    pub fw: String,
    pub proto: u32,
    pub board: String,
    pub idf: Option<String>,
    pub chip: Option<String>,
    pub rev: Option<String>,
}

impl VerInfo {
    pub fn from_reply(r: &Reply) -> Result<VerInfo> {
        Ok(VerInfo {
            fw: r.fields.parse_as("ver", "fw")?,
            proto: r.fields.parse_as("ver", "proto")?,
            board: r.fields.get("board").unwrap_or("").to_string(),
            idf: r.fields.get("idf").map(str::to_string),
            chip: r.fields.get("chip").map(str::to_string),
            rev: r.fields.get("rev").map(str::to_string),
        })
    }
}

/// `status` reply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct C5Status {
    pub mhz: u32,
    pub off_khz: i32,
    pub gain: u32,
    pub clk: u32,
    pub clk_on: bool,
    pub lanes: String,
    pub p11: bool,
    pub filter: String,
    pub selftest: String,
    pub up_ms: u64,
    pub slips: u32,
    pub phy: String,
}

impl C5Status {
    pub fn from_reply(r: &Reply) -> Result<C5Status> {
        let f = &r.fields;
        Ok(C5Status {
            mhz: f.parse_as("status", "mhz")?,
            off_khz: f.parse_as("status", "off_khz")?,
            gain: f.parse_as("status", "gain")?,
            clk: f.parse_as("status", "clk")?,
            clk_on: f.flag("status", "clk_on")?,
            lanes: f.get("lanes").unwrap_or("").to_string(),
            p11: f.flag("status", "p11")?,
            filter: f.get("filter").unwrap_or("").to_string(),
            selftest: f.get("selftest").unwrap_or("off").to_string(),
            up_ms: f.parse_as("status", "up_ms")?,
            slips: f.get("slips").and_then(|v| v.parse().ok()).unwrap_or(0),
            phy: f.get("phy").unwrap_or("").to_string(),
        })
    }

    /// The lane preset the C5 reports, if it is a known one.
    pub fn preset(&self) -> Option<LanePreset> {
        self.lanes.parse().ok()
    }
}

/// `pins` reply: GPIO numbers, MSB first per component.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinMap {
    pub clk: Option<u8>,
    pub i: Vec<u8>,
    pub q: Vec<u8>,
    /// Angle lanes (angle presets).
    pub a: Vec<u8>,
}

fn parse_gpio(t: &str) -> Option<u8> {
    let t = t.trim();
    let digits = t
        .strip_prefix("GPIO")
        .or_else(|| t.strip_prefix('G'))
        .or_else(|| t.strip_prefix('g'))
        .unwrap_or(t);
    digits.parse().ok()
}

impl PinMap {
    pub fn from_reply(r: &Reply) -> Result<PinMap> {
        let list = |k: &str| -> Result<Vec<u8>> {
            match r.fields.get(k) {
                None | Some("") => Ok(Vec::new()),
                Some(v) => v
                    .split(',')
                    .map(|t| parse_gpio(t).ok_or_else(|| HwError::Protocol(format!("`pins` {k}: bad pin `{t}`"))))
                    .collect(),
            }
        };
        Ok(PinMap {
            clk: r.fields.get("clk").and_then(parse_gpio),
            i: list("i")?,
            q: list("q")?,
            a: list("a").or_else(|_| list("angle"))?,
        })
    }
}

/// `gains` reply.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GainsInfo {
    pub max: u32,
    /// Optional dB estimate per index.
    pub table_db: Vec<f32>,
}

impl GainsInfo {
    pub fn from_reply(r: &Reply) -> Result<GainsInfo> {
        let table_db = match r.fields.get("table") {
            None | Some("") => Vec::new(),
            Some(v) => v.split(',').filter_map(|t| t.parse().ok()).collect(),
        };
        Ok(GainsInfo {
            max: r.fields.parse_as("gains", "max")?,
            table_db,
        })
    }
}

/// `selftest` reply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SelftestReply {
    pub mode: String,
    pub lanes: u32,
    pub phase_us: u32,
}

impl SelftestReply {
    pub fn from_reply(r: &Reply) -> Result<SelftestReply> {
        Ok(SelftestReply {
            mode: r.fields.get("mode").unwrap_or("").to_string(),
            lanes: r.fields.get("lanes").and_then(|v| v.parse().ok()).unwrap_or(0),
            phase_us: r.fields.get("phase_us").and_then(|v| v.parse().ok()).unwrap_or(1000),
        })
    }
}

/// Board pin maps known to the C5 firmware.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BoardKind {
    Xiao,
    Zero,
    Custom,
}

impl BoardKind {
    pub fn name(self) -> &'static str {
        match self {
            BoardKind::Xiao => "xiao",
            BoardKind::Zero => "zero",
            BoardKind::Custom => "custom",
        }
    }
}

impl FromStr for BoardKind {
    type Err = HwError;
    fn from_str(s: &str) -> Result<Self> {
        match s {
            "xiao" => Ok(BoardKind::Xiao),
            "zero" => Ok(BoardKind::Zero),
            "custom" => Ok(BoardKind::Custom),
            o => Err(HwError::Invalid(format!("unknown board `{o}` (xiao|zero|custom)"))),
        }
    }
}

/// Analog receive filter selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterCode {
    /// The chip's own calibration.
    Cal,
    Code(i32),
}

/// Self-test patterns.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelftestMode {
    Off,
    Walk,
    /// Fixed value; bit j drives lane j.
    Const(u32),
    Count,
}

impl SelftestMode {
    pub fn command(&self) -> String {
        match self {
            SelftestMode::Off => "selftest off".into(),
            SelftestMode::Walk => "selftest walk".into(),
            SelftestMode::Const(v) => format!("selftest const {v:X}"),
            SelftestMode::Count => "selftest count".into(),
        }
    }
}

/// Everything the host owns on a C5 and pushes after every connect or
/// reset.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct C5Settings {
    pub board: BoardKind,
    pub lanes: LanePreset,
    /// Working sample clock divider (240 MHz / divider).
    pub divider: u32,
    pub mhz: u32,
    pub offset_khz: i32,
    pub gain: u32,
    pub p11: bool,
    pub filter: FilterCode,
    pub filter_skirt: u32,
    /// Front-end bandwidth 20 or 40 MHz (None: firmware default).
    pub bw_mhz: Option<u32>,
    /// Pad drive (lanes, clock), 0..=3 (None: firmware default).
    pub drive: Option<(u8, u8)>,
}

impl Default for C5Settings {
    fn default() -> Self {
        C5Settings {
            board: BoardKind::Xiao,
            lanes: LanePreset::I7Q7,
            divider: 18,
            mhz: 5732,
            offset_khz: 0,
            gain: 64,
            p11: true,
            filter: FilterCode::Cal,
            filter_skirt: 0,
            bw_mhz: None,
            drive: None,
        }
    }
}

impl C5Settings {
    pub fn board_cmd(&self) -> String {
        format!("board {}", self.board.name())
    }
    pub fn lanes_cmd(&self) -> String {
        format!("lanes {}", self.lanes.name())
    }
    pub fn clk_cmd(&self) -> String {
        format!("clk {}", self.divider)
    }
    pub fn tune_cmd(&self) -> String {
        format!("tune {} {}", self.mhz, self.offset_khz)
    }
    pub fn gain_cmd(&self) -> String {
        format!("gain {}", self.gain)
    }
    pub fn p11_cmd(&self) -> String {
        format!("p11 {}", if self.p11 { "on" } else { "off" })
    }
    pub fn filter_cmd(&self) -> String {
        match self.filter {
            FilterCode::Cal => format!("filter cal {}", self.filter_skirt),
            FilterCode::Code(c) => format!("filter {c} {}", self.filter_skirt),
        }
    }

    /// The full state push, in protocol order (`board`, `lanes`, `clk`,
    /// `tune`, `gain`, `p11`, `filter`, then the optional `bw`, `drive`).
    pub fn push_commands(&self) -> Vec<String> {
        let mut v = vec![
            self.board_cmd(),
            self.lanes_cmd(),
            self.clk_cmd(),
            self.tune_cmd(),
            self.gain_cmd(),
            self.p11_cmd(),
            self.filter_cmd(),
        ];
        if let Some(bw) = self.bw_mhz {
            v.push(format!("bw {bw}"));
        }
        if let Some((l, c)) = self.drive {
            v.push(format!("drive {l} {c}"));
        }
        v
    }

    pub fn validate(&self) -> Result<()> {
        if !(6..=1023).contains(&self.divider) {
            return Err(HwError::Invalid(format!("divider {} not in 6..=1023", self.divider)));
        }
        if let Some(bw) = self.bw_mhz {
            if bw != 20 && bw != 40 {
                return Err(HwError::Invalid(format!("bandwidth {bw} MHz (20 or 40)")));
            }
        }
        if let Some((l, c)) = self.drive {
            if l > 3 || c > 3 {
                return Err(HwError::Invalid("drive strengths are 0..=3".into()));
            }
        }
        Ok(())
    }

    /// Sample format these settings produce.
    pub fn sample_format(&self) -> openvrx_core::SampleFormat {
        openvrx_core::SampleFormat::from_preset(self.lanes, self.divider)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_lines() {
        match parse_line("OK ver fw=0.1.0 proto=1 board=xiao idf=v6.0.2 chip=esp32c5 rev=1\r") {
            Line::Ok(r) => {
                assert_eq!(r.command, "ver");
                let v = VerInfo::from_reply(&r).unwrap();
                assert_eq!(v.fw, "0.1.0");
                assert_eq!(v.proto, 1);
                assert_eq!(v.board, "xiao");
                assert_eq!(v.chip.as_deref(), Some("esp32c5"));
            }
            o => panic!("{o:?}"),
        }
        assert_eq!(
            parse_line("ERR tune range 5000 MHz is outside 5645..5945"),
            Line::Err(ErrReply {
                command: "tune".into(),
                code: ErrCode::Range,
                text: "5000 MHz is outside 5645..5945".into()
            })
        );
        match parse_line("EVT boot fw=0.1.0 reason=poweron") {
            Line::Evt(e) => {
                assert_eq!(e.name, "boot");
                assert_eq!(e.fields.get("reason"), Some("poweron"));
            }
            o => panic!("{o:?}"),
        }
        assert_eq!(parse_line("LOG phy restored"), Line::Log("phy restored".into()));
        assert_eq!(
            parse_line("ESP-ROM:esp32c5-20240101"),
            Line::Other("ESP-ROM:esp32c5-20240101".into())
        );
        assert_eq!(parse_line("OK"), Line::Other("OK".into()));
        assert!(matches!(
            parse_line("ERR x weird"),
            Line::Err(ErrReply {
                code: ErrCode::Other(_),
                ..
            })
        ));
        assert!(matches!(parse_line("OK ping"), Line::Ok(Reply { .. })));
    }

    #[test]
    fn status_and_pins() {
        let Line::Ok(r) = parse_line(
            "OK status mhz=5732 off_khz=-120 gain=64 clk=18 clk_on=1 lanes=7x7 p11=1 filter=cal selftest=off up_ms=12345 slips=2 phy=ok",
        ) else {
            panic!()
        };
        let s = C5Status::from_reply(&r).unwrap();
        assert_eq!(s.off_khz, -120);
        assert!(s.clk_on && s.p11);
        assert_eq!(s.preset(), Some(LanePreset::I7Q7));
        assert_eq!(s.slips, 2);
        let Line::Ok(r) = parse_line("OK pins clk=G10 q=G1,G0,G25,G7,G23,G24,G11 i=G23,G12,G8,G9,G2,G3,G4") else {
            panic!()
        };
        let p = PinMap::from_reply(&r).unwrap();
        assert_eq!(p.clk, Some(10));
        assert_eq!(p.q, vec![1, 0, 25, 7, 23, 24, 11]);
        assert_eq!(p.i[0], 23);
        let Line::Ok(bad) = parse_line("OK status mhz=x") else {
            panic!()
        };
        assert!(C5Status::from_reply(&bad).is_err());
        let Line::Ok(g) = parse_line("OK gains max=77 table=0,0.5,1.0") else {
            panic!()
        };
        let g = GainsInfo::from_reply(&g).unwrap();
        assert_eq!(g.max, 77);
        assert_eq!(g.table_db, vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn command_checks() {
        assert!(check_command("tune 5732 0").is_ok());
        assert!(check_command("").is_err());
        assert!(check_command("tune  5732").is_err());
        assert!(check_command("tune 5732\nreboot").is_err());
        assert!(check_command(&"x".repeat(121)).is_err());
        assert_eq!(command_word("clk off"), "clk");
    }

    #[test]
    fn settings_push_order() {
        let s = C5Settings {
            offset_khz: -50,
            bw_mhz: Some(20),
            drive: Some((1, 2)),
            filter: FilterCode::Code(5),
            filter_skirt: 2,
            ..Default::default()
        };
        assert_eq!(
            s.push_commands(),
            vec![
                "board xiao",
                "lanes 7x7",
                "clk 18",
                "tune 5732 -50",
                "gain 64",
                "p11 on",
                "filter 5 2",
                "bw 20",
                "drive 1 2"
            ]
        );
        s.validate().unwrap();
        assert!(C5Settings {
            divider: 5,
            ..s.clone()
        }
        .validate()
        .is_err());
        assert!(C5Settings {
            bw_mhz: Some(30),
            ..s.clone()
        }
        .validate()
        .is_err());
        assert_eq!(SelftestMode::Const(0xA5).command(), "selftest const A5");
        assert!((s.sample_format().sample_rate_hz() - 13.333_333e6).abs() < 1.0);
    }
}
