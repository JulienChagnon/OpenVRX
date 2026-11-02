// SPDX-License-Identifier: GPL-3.0-or-later
//! The sidecar (`NAME.openvrx.json`): everything needed to decode.

use crate::{io_err, FormatError, Result, FORMAT_NAME, FORMAT_VERSION};
use openvrx_core::{RxId, SampleFormat, VideoStandard};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Where the data came from.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RecordingSource {
    Hardware,
    Sim,
}

/// The whole sidecar, field for field as in `docs/RECORDING_FORMAT.md`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Sidecar {
    pub format: String,
    pub version: u32,
    pub created_utc: String,
    pub software: String,
    pub source: RecordingSource,
    #[serde(default)]
    pub notes: String,
    #[serde(default)]
    pub closed_cleanly: bool,
    pub receivers: Vec<ReceiverInfo>,
    #[serde(default)]
    pub standard_hint: Option<VideoStandard>,
    #[serde(default)]
    pub sim: Option<SimInfo>,
}

/// One receiver's stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReceiverInfo {
    pub rx: RxId,
    /// Raw file name, relative to the sidecar's directory.
    pub file: String,
    /// Bytes in the raw file (valid when `closed_cleanly`).
    #[serde(default)]
    pub bytes: u64,
    pub sample_format: SampleFormat,
    pub sample_rate_hz: f64,
    #[serde(default)]
    pub channel_mhz: u32,
    #[serde(default)]
    pub offset_khz: i32,
    #[serde(default)]
    pub gain: u32,
    #[serde(default)]
    pub p11: bool,
    #[serde(default)]
    pub board: String,
    #[serde(default)]
    pub c5_fw: String,
    #[serde(default)]
    pub fx2_fw: String,
    #[serde(default)]
    pub fx2_serial: String,
    #[serde(default)]
    pub ifclk_inverted: bool,
    #[serde(default = "one")]
    pub watch_n: u32,
    #[serde(default)]
    pub start_host_ns: u64,
    #[serde(default)]
    pub antenna: String,
}

fn one() -> u32 {
    1
}

impl ReceiverInfo {
    /// A receiver entry with the given format and neutral defaults; `file`
    /// is filled in by the writer.
    pub fn new(rx: RxId, sample_format: SampleFormat) -> ReceiverInfo {
        let sample_rate_hz = sample_format.sample_rate_hz();
        ReceiverInfo {
            rx,
            file: String::new(),
            bytes: 0,
            sample_format,
            sample_rate_hz,
            channel_mhz: 0,
            offset_khz: 0,
            gain: 0,
            p11: false,
            board: String::new(),
            c5_fw: String::new(),
            fx2_fw: String::new(),
            fx2_serial: String::new(),
            ifclk_inverted: false,
            watch_n: 1,
            start_host_ns: 0,
            antenna: String::new(),
        }
    }

    /// Bytes per word on the wire.
    #[inline]
    pub fn word_bytes(&self) -> u64 {
        self.sample_format.lanes.word_bytes as u64
    }

    /// Nominal byte rate of the stream (watch mode keeps 1 packet in N).
    pub fn byte_rate(&self) -> f64 {
        self.sample_rate_hz * self.word_bytes() as f64 / self.watch_n.max(1) as f64
    }
}

/// Simulator settings plus the truth description (`sim` in the sidecar).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SimInfo {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub truth: Option<TruthInfo>,
    /// Generator settings (C/N, fading, deviation, standard, seeds,
    /// picture source...), free-form so the simulator owns their schema.
    #[serde(flatten)]
    pub settings: serde_json::Map<String, serde_json::Value>,
}

/// The truth file of a simulated recording.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TruthInfo {
    pub file: String,
    /// Receiver whose input samples the truth is aligned to.
    pub rx: RxId,
    /// Unit of the values ("ire").
    pub unit: String,
    /// Group delay of the simulated channel in input samples: decoder
    /// output at sample `n` compares with `truth[n - delay_samples]`.
    pub delay_samples: f64,
}

impl Sidecar {
    /// A fresh sidecar for a recording that is about to be written.
    pub fn new(source: RecordingSource, receivers: Vec<ReceiverInfo>) -> Sidecar {
        Sidecar {
            format: FORMAT_NAME.to_string(),
            version: FORMAT_VERSION,
            created_utc: crate::utc_now_string(),
            software: crate::software_string(),
            source,
            notes: String::new(),
            closed_cleanly: false,
            receivers,
            standard_hint: None,
            sim: None,
        }
    }

    /// Receiver entry by receiver index.
    pub fn receiver(&self, rx: RxId) -> Option<&ReceiverInfo> {
        self.receivers.iter().find(|r| r.rx == rx)
    }

    pub fn truth(&self) -> Option<&TruthInfo> {
        self.sim.as_ref().and_then(|s| s.truth.as_ref())
    }

    /// Checks format name, version and receiver consistency.
    pub fn validate(&self) -> Result<()> {
        if self.format != FORMAT_NAME {
            return Err(FormatError::NotRecording(format!(
                "format is `{}`, expected `{FORMAT_NAME}`",
                self.format
            )));
        }
        if self.version == 0 || self.version > FORMAT_VERSION {
            return Err(FormatError::Version(self.version));
        }
        let mut seen = std::collections::HashSet::new();
        for r in &self.receivers {
            if !seen.insert(r.rx) {
                return Err(FormatError::NotRecording(format!("receiver {} listed twice", r.rx)));
            }
            r.sample_format
                .lanes
                .validate()
                .map_err(|e| FormatError::NotRecording(format!("rx{}: {e}", r.rx)))?;
            if r.sample_format.divider == 0 || r.sample_rate_hz.is_nan() || r.sample_rate_hz <= 0.0 {
                return Err(FormatError::NotRecording(format!("rx{}: bad sample rate", r.rx)));
            }
        }
        Ok(())
    }

    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).expect("sidecar serializes")
    }

    /// Reads and validates a sidecar file.
    pub fn read(path: &Path) -> Result<Sidecar> {
        let text = std::fs::read_to_string(path).map_err(io_err(path))?;
        let s: Sidecar = serde_json::from_str(&text).map_err(|source| FormatError::Json {
            path: path.to_path_buf(),
            source,
        })?;
        s.validate()?;
        Ok(s)
    }

    /// Writes the sidecar atomically (temporary file, then rename over).
    pub fn write_atomic(&self, path: &Path) -> Result<()> {
        let mut tmp = path.as_os_str().to_owned();
        tmp.push(".tmp");
        let tmp = std::path::PathBuf::from(tmp);
        {
            use std::io::Write;
            let mut f = std::fs::File::create(&tmp).map_err(io_err(&tmp))?;
            f.write_all(self.to_json_pretty().as_bytes())
                .and_then(|_| f.write_all(b"\n"))
                .and_then(|_| f.sync_all())
                .map_err(io_err(&tmp))?;
        }
        std::fs::rename(&tmp, path).map_err(io_err(path))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::LanePreset;

    const DOC_EXAMPLE: &str = r#"{
  "format": "openvrx-recording",
  "version": 1,
  "created_utc": "2026-10-29T21:14:03Z",
  "software": "openvrx 0.1.0",
  "source": "hardware",
  "notes": "",
  "closed_cleanly": true,
  "receivers": [
    {
      "rx": 0,
      "file": "NAME.rx0.bin",
      "bytes": 1600000000,
      "sample_format": {
        "preset": "7x7",
        "lanes": { "word_bytes": 2, "encoding": { "kind": "iq",
          "i": { "word_msb": 15, "adc_bits": [9,8,7,6,5,4,3] },
          "q": { "word_msb": 7,  "adc_bits": [9,8,7,6,5,4,3] } } },
        "divider": 18
      },
      "sample_rate_hz": 13333333.333,
      "channel_mhz": 5732,
      "offset_khz": 0,
      "gain": 64,
      "p11": true,
      "board": "xiao",
      "c5_fw": "0.1.0",
      "fx2_fw": "0.1.0",
      "fx2_serial": "A1B2C3D4",
      "ifclk_inverted": false,
      "watch_n": 1,
      "start_host_ns": 123456789,
      "antenna": "omni RHCP"
    }
  ],
  "standard_hint": "ntsc",
  "sim": null
}"#;

    #[test]
    fn parses_doc_example() {
        let s: Sidecar = serde_json::from_str(DOC_EXAMPLE).unwrap();
        s.validate().unwrap();
        let r = &s.receivers[0];
        assert_eq!(r.sample_format, SampleFormat::from_preset(LanePreset::I7Q7, 18));
        assert_eq!(r.bytes, 1_600_000_000);
        assert_eq!(s.standard_hint, Some(VideoStandard::Ntsc));
        assert!(s.closed_cleanly);
        assert_eq!(r.antenna, "omni RHCP");
        // Round trip through our own serializer.
        let back: Sidecar = serde_json::from_str(&s.to_json_pretty()).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn sim_block_round_trip() {
        let mut s = Sidecar::new(
            RecordingSource::Sim,
            vec![ReceiverInfo::new(0, SampleFormat::from_preset(LanePreset::I8Q8, 18))],
        );
        let mut settings = serde_json::Map::new();
        settings.insert("cn_db".into(), serde_json::json!(8.0));
        s.sim = Some(SimInfo {
            truth: Some(TruthInfo {
                file: "x.truth.f32".into(),
                rx: 0,
                unit: "ire".into(),
                delay_samples: 3.25,
            }),
            settings,
        });
        let text = s.to_json_pretty();
        assert!(text.contains("\"truth\""));
        assert!(text.contains("\"cn_db\""));
        let back: Sidecar = serde_json::from_str(&text).unwrap();
        assert_eq!(back, s);
        assert_eq!(back.truth().unwrap().delay_samples, 3.25);
    }

    #[test]
    fn rejects_other_versions() {
        let mut s: Sidecar = serde_json::from_str(DOC_EXAMPLE).unwrap();
        s.version = 2;
        assert!(matches!(s.validate(), Err(FormatError::Version(2))));
        s.version = 1;
        s.format = "other".into();
        assert!(matches!(s.validate(), Err(FormatError::NotRecording(_))));
    }
}
