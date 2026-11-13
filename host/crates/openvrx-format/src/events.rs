// SPDX-License-Identifier: GPL-3.0-or-later
//! Events (`NAME.events.jsonl`): one JSON object per line.
//!
//! Every event has `rx` (null for session-wide events), `host_ns`, `kind`
//! and `byte` (offset in that receiver's raw file when the event was
//! observed), plus kind-specific fields. Kinds this build does not know
//! are kept as [`EventKind::Other`] so tools can pass them through.

use crate::{io_err, Result};
use openvrx_core::RxId;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::io::BufRead;
use std::path::Path;

/// Phase probe verdict.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ProbeVerdict {
    Clean,
    Mixed,
}

/// What happened, with the kind-specific fields.
#[derive(Clone, Debug, PartialEq)]
pub enum EventKind {
    /// Gain index command.
    Gain { index: u32 },
    /// Tune command.
    Tune { mhz: u32, offset_khz: i32 },
    /// Sample clock divider command.
    Clk { divider: u32 },
    /// Clock slip command (240 MHz ticks).
    Slip { count: u32 },
    /// Sampling-phase probe result.
    Probe { score: f64, verdict: ProbeVerdict },
    /// Decoded FX2 status block (fields owned by the hardware layer).
    Fx2Status(Map<String, Value>),
    /// Data lost on the way (USB overflow, dropped transfers).
    UsbGap { lost_bytes: u64, reason: String },
    /// The FX2 FIFO was full at the pins since the last status block: samples
    /// the C5 sent were dropped there and are missing from the file without a
    /// position jump. `polls` counts the firmware loop's polls that found it
    /// full (0.83 us each, so about polls x 0.83 us of samples lost).
    FifoFull { polls: u64 },
    /// Every 64th bulk completion, for latency analysis.
    Transfer { bytes: u64 },
    /// User marker.
    Marker { text: String },
    /// Raw C5 console line.
    C5 { line: String },
    /// A kind this build does not know; all fields kept.
    Other { kind: String, fields: Map<String, Value> },
}

/// Typed form of the known kinds with flat fields (serde helper).
#[derive(Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Known {
    Gain { index: u32 },
    Tune { mhz: u32, offset_khz: i32 },
    Clk { divider: u32 },
    Slip { count: u32 },
    Probe { score: f64, verdict: ProbeVerdict },
    UsbGap { lost_bytes: u64, reason: String },
    FifoFull { polls: u64 },
    Transfer { bytes: u64 },
    Marker { text: String },
    C5 { line: String },
}

impl EventKind {
    /// The `kind` string.
    pub fn name(&self) -> &str {
        match self {
            EventKind::Gain { .. } => "gain",
            EventKind::Tune { .. } => "tune",
            EventKind::Clk { .. } => "clk",
            EventKind::Slip { .. } => "slip",
            EventKind::Probe { .. } => "probe",
            EventKind::Fx2Status(_) => "fx2_status",
            EventKind::UsbGap { .. } => "usb_gap",
            EventKind::FifoFull { .. } => "fifo_full",
            EventKind::Transfer { .. } => "transfer",
            EventKind::Marker { .. } => "marker",
            EventKind::C5 { .. } => "c5",
            EventKind::Other { kind, .. } => kind,
        }
    }

    fn to_known(&self) -> Option<Known> {
        Some(match self.clone() {
            EventKind::Gain { index } => Known::Gain { index },
            EventKind::Tune { mhz, offset_khz } => Known::Tune { mhz, offset_khz },
            EventKind::Clk { divider } => Known::Clk { divider },
            EventKind::Slip { count } => Known::Slip { count },
            EventKind::Probe { score, verdict } => Known::Probe { score, verdict },
            EventKind::UsbGap { lost_bytes, reason } => Known::UsbGap { lost_bytes, reason },
            EventKind::FifoFull { polls } => Known::FifoFull { polls },
            EventKind::Transfer { bytes } => Known::Transfer { bytes },
            EventKind::Marker { text } => Known::Marker { text },
            EventKind::C5 { line } => Known::C5 { line },
            EventKind::Fx2Status(_) | EventKind::Other { .. } => return None,
        })
    }

    fn from_known(k: Known) -> EventKind {
        match k {
            Known::Gain { index } => EventKind::Gain { index },
            Known::Tune { mhz, offset_khz } => EventKind::Tune { mhz, offset_khz },
            Known::Clk { divider } => EventKind::Clk { divider },
            Known::Slip { count } => EventKind::Slip { count },
            Known::Probe { score, verdict } => EventKind::Probe { score, verdict },
            Known::UsbGap { lost_bytes, reason } => EventKind::UsbGap { lost_bytes, reason },
            Known::FifoFull { polls } => EventKind::FifoFull { polls },
            Known::Transfer { bytes } => EventKind::Transfer { bytes },
            Known::Marker { text } => EventKind::Marker { text },
            Known::C5 { line } => EventKind::C5 { line },
        }
    }

    /// Kind-specific fields (without `kind`).
    fn fields(&self) -> Map<String, Value> {
        match self {
            EventKind::Fx2Status(m) => m.clone(),
            EventKind::Other { fields, .. } => fields.clone(),
            other => {
                let known = other.to_known().expect("known kind");
                match serde_json::to_value(known).expect("event serializes") {
                    Value::Object(mut m) => {
                        m.remove("kind");
                        m
                    }
                    _ => Map::new(),
                }
            }
        }
    }
}

/// One event.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// Receiver, or None for session-wide events.
    pub rx: Option<RxId>,
    /// Host time (`openvrx_core::clock::now_ns`).
    pub host_ns: u64,
    /// Offset in the receiver's raw file when the event was observed.
    pub byte: Option<u64>,
    pub kind: EventKind,
}

const RESERVED: [&str; 4] = ["rx", "host_ns", "byte", "kind"];

impl Event {
    pub fn new(rx: Option<RxId>, host_ns: u64, byte: Option<u64>, kind: EventKind) -> Event {
        Event {
            rx,
            host_ns,
            byte,
            kind,
        }
    }

    /// One JSON line (no trailing newline). Field order: rx, host_ns,
    /// kind, byte, then the kind's fields.
    pub fn to_json_line(&self) -> String {
        #[derive(Serialize)]
        struct Out<'a> {
            rx: Option<RxId>,
            host_ns: u64,
            kind: &'a str,
            byte: Option<u64>,
            #[serde(flatten)]
            fields: Map<String, Value>,
        }
        let mut fields = self.kind.fields();
        for k in RESERVED {
            fields.remove(k);
        }
        serde_json::to_string(&Out {
            rx: self.rx,
            host_ns: self.host_ns,
            kind: self.kind.name(),
            byte: self.byte,
            fields,
        })
        .expect("event serializes")
    }

    /// Parses one JSON line.
    pub fn from_json_line(line: &str) -> std::result::Result<Event, String> {
        let v: Value = serde_json::from_str(line).map_err(|e| e.to_string())?;
        Event::from_value(v)
    }

    pub fn from_value(v: Value) -> std::result::Result<Event, String> {
        let Value::Object(mut m) = v else {
            return Err("event is not a JSON object".into());
        };
        let rx = match m.remove("rx") {
            None | Some(Value::Null) => None,
            Some(Value::Number(n)) => Some(
                n.as_u64()
                    .and_then(|x| u8::try_from(x).ok())
                    .ok_or("rx out of range")?,
            ),
            Some(_) => return Err("rx is not a number".into()),
        };
        let host_ns = match m.remove("host_ns") {
            Some(Value::Number(n)) => n.as_u64().ok_or("host_ns is not an unsigned integer")?,
            None | Some(Value::Null) => 0,
            Some(_) => return Err("host_ns is not a number".into()),
        };
        let byte = match m.remove("byte") {
            None | Some(Value::Null) => None,
            Some(Value::Number(n)) => Some(n.as_u64().ok_or("byte is not an unsigned integer")?),
            Some(_) => return Err("byte is not a number".into()),
        };
        let kind_name = match m.get("kind") {
            Some(Value::String(s)) => s.clone(),
            _ => return Err("missing kind".into()),
        };
        let kind = match kind_name.as_str() {
            "fx2_status" => {
                m.remove("kind");
                EventKind::Fx2Status(m)
            }
            "gain" | "tune" | "clk" | "slip" | "probe" | "usb_gap" | "fifo_full" | "transfer" | "marker"
            | "c5" => {
                let known: Known = serde_json::from_value(Value::Object(m))
                    .map_err(|e| format!("{kind_name}: {e}"))?;
                EventKind::from_known(known)
            }
            _ => {
                m.remove("kind");
                EventKind::Other {
                    kind: kind_name,
                    fields: m,
                }
            }
        };
        Ok(Event {
            rx,
            host_ns,
            byte,
            kind,
        })
    }
}

impl Serialize for Event {
    fn serialize<S: serde::Serializer>(&self, s: S) -> std::result::Result<S::Ok, S::Error> {
        let v: Value = serde_json::from_str(&self.to_json_line()).map_err(serde::ser::Error::custom)?;
        v.serialize(s)
    }
}

impl<'de> Deserialize<'de> for Event {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Event, D::Error> {
        let v = Value::deserialize(d)?;
        Event::from_value(v).map_err(serde::de::Error::custom)
    }
}

/// Parsed events file.
#[derive(Clone, Debug, Default)]
pub struct EventLog {
    pub events: Vec<Event>,
    /// Lines that could not be parsed (a crash can leave a partial last
    /// line), with their 1-based line numbers and the reason.
    pub skipped: Vec<(usize, String)>,
}

impl EventLog {
    /// Reads an events file; a missing file is an empty log.
    pub fn read(path: &Path) -> Result<EventLog> {
        let f = match std::fs::File::open(path) {
            Ok(f) => f,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(EventLog::default()),
            Err(e) => return Err(io_err(path)(e)),
        };
        Self::parse(std::io::BufReader::new(f)).map_err(io_err(path))
    }

    pub fn parse(r: impl BufRead) -> std::io::Result<EventLog> {
        let mut log = EventLog::default();
        for (k, line) in r.split(b'\n').enumerate() {
            let line = line?;
            let text = String::from_utf8_lossy(&line);
            let text = text.trim();
            if text.is_empty() {
                continue;
            }
            match Event::from_json_line(text) {
                Ok(ev) => log.events.push(ev),
                Err(e) => log.skipped.push((k + 1, e)),
            }
        }
        Ok(log)
    }

    /// Events of one receiver, in file order.
    pub fn for_rx(&self, rx: RxId) -> impl Iterator<Item = &Event> {
        self.events.iter().filter(move |e| e.rx == Some(rx))
    }

    /// Count per kind.
    pub fn counts(&self) -> std::collections::BTreeMap<String, usize> {
        let mut m = std::collections::BTreeMap::new();
        for e in &self.events {
            *m.entry(e.kind.name().to_string()).or_insert(0) += 1;
        }
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_kinds() -> Vec<Event> {
        let mut status = Map::new();
        status.insert("fifo_full".into(), Value::from(3));
        status.insert("ok".into(), Value::from(true));
        let mut other = Map::new();
        other.insert("x".into(), Value::from(1.5));
        let kinds = vec![
            EventKind::Gain { index: 68 },
            EventKind::Tune {
                mhz: 5917,
                offset_khz: -250,
            },
            EventKind::Clk { divider: 18 },
            EventKind::Slip { count: 1 },
            EventKind::Probe {
                score: 31.5,
                verdict: ProbeVerdict::Mixed,
            },
            EventKind::Fx2Status(status),
            EventKind::UsbGap {
                lost_bytes: 1024,
                reason: "overflow".into(),
            },
            EventKind::FifoFull { polls: 115 },
            EventKind::Transfer { bytes: 16384 },
            EventKind::Marker { text: "lap 2".into() },
            EventKind::C5 {
                line: "OK gain 68".into(),
            },
            EventKind::Other {
                kind: "future_thing".into(),
                fields: other,
            },
        ];
        kinds
            .into_iter()
            .enumerate()
            .map(|(k, kind)| Event::new(if k % 3 == 0 { None } else { Some(1) }, 1000 + k as u64, Some(k as u64 * 2), kind))
            .collect()
    }

    #[test]
    fn round_trip_every_kind() {
        for ev in all_kinds() {
            let line = ev.to_json_line();
            assert!(!line.contains('\n'));
            let back = Event::from_json_line(&line).unwrap();
            assert_eq!(back, ev, "{line}");
            // serde path agrees with the line path
            let via_serde: Event = serde_json::from_str(&serde_json::to_string(&ev).unwrap()).unwrap();
            assert_eq!(via_serde, ev);
        }
    }

    #[test]
    fn line_shape_matches_doc() {
        let ev = Event::new(Some(0), 42, Some(4096), EventKind::Gain { index: 64 });
        assert_eq!(
            ev.to_json_line(),
            r#"{"rx":0,"host_ns":42,"kind":"gain","byte":4096,"index":64}"#
        );
        let s = Event::new(None, 1, None, EventKind::Marker { text: "m".into() });
        assert_eq!(
            s.to_json_line(),
            r#"{"rx":null,"host_ns":1,"kind":"marker","byte":null,"text":"m"}"#
        );
    }

    #[test]
    fn log_skips_partial_last_line() {
        let mut text = String::new();
        for ev in all_kinds() {
            text.push_str(&ev.to_json_line());
            text.push('\n');
        }
        text.push_str(r#"{"rx":0,"host_ns":5,"ki"#);
        let log = EventLog::parse(text.as_bytes()).unwrap();
        assert_eq!(log.events.len(), all_kinds().len());
        assert_eq!(log.skipped.len(), 1);
        assert_eq!(log.counts()["gain"], 1);
        assert_eq!(log.for_rx(1).count(), 8);
    }

    #[test]
    fn rejects_bad_fields() {
        assert!(Event::from_json_line(r#"{"rx":0,"host_ns":1,"kind":"gain","byte":0}"#).is_err());
        assert!(Event::from_json_line(r#"{"rx":300,"host_ns":1,"kind":"marker","text":""}"#).is_err());
        assert!(Event::from_json_line(r#"[1,2]"#).is_err());
    }
}
