// SPDX-License-Identifier: GPL-3.0-or-later
//! Recording utilities: listing and summaries, and cutting a time range
//! (trim) or equal pieces (split) into new recordings.

use crate::events::EventKind;
use crate::reader::Recording;
use crate::sidecar::RecordingSource;
use crate::writer::{RecordingWriter, WriterOptions};
use crate::{io_err, FormatError, Result};
use openvrx_core::lanes::C5_PLL_HZ;
use openvrx_core::{RxId, VideoStandard};
use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

/// One receiver in a [`RecordingSummary`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReceiverSummary {
    pub rx: RxId,
    pub preset: Option<String>,
    pub divider: u32,
    pub sample_rate_hz: f64,
    pub word_bytes: u64,
    pub bytes: u64,
    pub samples: u64,
    pub duration_s: f64,
    /// Bytes per second of stream time actually stored.
    pub stored_byte_rate: f64,
    pub channel_mhz: u32,
    pub gain: u32,
    /// Bytes reported lost by `usb_gap` events.
    pub lost_bytes: u64,
}

/// What a recording holds.
#[derive(Clone, Debug, PartialEq)]
pub struct RecordingSummary {
    pub name: String,
    pub sidecar_path: PathBuf,
    pub created_utc: String,
    pub source: RecordingSource,
    pub closed_cleanly: bool,
    pub standard_hint: Option<VideoStandard>,
    pub notes: String,
    pub duration_s: f64,
    pub total_bytes: u64,
    pub receivers: Vec<ReceiverSummary>,
    pub event_counts: BTreeMap<String, usize>,
    pub skipped_event_lines: usize,
    pub has_truth: bool,
}

impl RecordingSummary {
    pub fn of(rec: &Recording) -> RecordingSummary {
        let s = rec.sidecar();
        let receivers = s
            .receivers
            .iter()
            .map(|r| {
                let bytes = rec.rx_bytes(r.rx).unwrap_or(0);
                let duration_s = rec.rx_duration_s(r.rx).unwrap_or(0.0);
                let lost_bytes = rec
                    .events()
                    .for_rx(r.rx)
                    .map(|e| match e.kind {
                        EventKind::UsbGap { lost_bytes, .. } => lost_bytes,
                        _ => 0,
                    })
                    .sum();
                ReceiverSummary {
                    rx: r.rx,
                    preset: r.sample_format.preset.map(|p| p.name().to_string()),
                    divider: r.sample_format.divider,
                    sample_rate_hz: r.sample_rate_hz,
                    word_bytes: r.word_bytes(),
                    bytes,
                    samples: bytes / r.word_bytes(),
                    duration_s,
                    stored_byte_rate: if duration_s > 0.0 {
                        bytes as f64 / duration_s
                    } else {
                        0.0
                    },
                    channel_mhz: r.channel_mhz,
                    gain: r.gain,
                    lost_bytes,
                }
            })
            .collect::<Vec<_>>();
        RecordingSummary {
            name: rec.name().to_string(),
            sidecar_path: rec.paths().sidecar(),
            created_utc: s.created_utc.clone(),
            source: s.source,
            closed_cleanly: s.closed_cleanly,
            standard_hint: s.standard_hint,
            notes: s.notes.clone(),
            duration_s: rec.duration_s(),
            total_bytes: receivers.iter().map(|r| r.bytes).sum(),
            receivers,
            event_counts: rec.events().counts(),
            skipped_event_lines: rec.events().skipped.len(),
            has_truth: s.truth().is_some(),
        }
    }
}

impl fmt::Display for RecordingSummary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "{}  {}  {:?}{}  {:.3} s  {:.1} MB{}{}",
            self.name,
            self.created_utc,
            self.source,
            if self.closed_cleanly { "" } else { " (not closed cleanly)" },
            self.duration_s,
            self.total_bytes as f64 / 1e6,
            self.standard_hint
                .map(|s| format!("  {}", s.name()))
                .unwrap_or_default(),
            if self.has_truth { "  truth" } else { "" },
        )?;
        for r in &self.receivers {
            writeln!(
                f,
                "  rx{}: {} /{} {:.3} MS/s, {} samples, {:.3} s, {:.2} MB/s stored, {} MHz, gain {}{}",
                r.rx,
                r.preset.as_deref().unwrap_or("custom"),
                r.divider,
                r.sample_rate_hz / 1e6,
                r.samples,
                r.duration_s,
                r.stored_byte_rate / 1e6,
                r.channel_mhz,
                r.gain,
                if r.lost_bytes > 0 {
                    format!(", {} bytes lost", r.lost_bytes)
                } else {
                    String::new()
                }
            )?;
        }
        if !self.event_counts.is_empty() {
            let list: Vec<String> = self
                .event_counts
                .iter()
                .map(|(k, n)| format!("{k} {n}"))
                .collect();
            writeln!(f, "  events: {}", list.join(", "))?;
        }
        if self.skipped_event_lines > 0 {
            writeln!(f, "  {} unreadable event lines", self.skipped_event_lines)?;
        }
        if !self.notes.is_empty() {
            writeln!(f, "  notes: {}", self.notes)?;
        }
        Ok(())
    }
}

/// Readable recordings (by name) and unreadable sidecars with the error.
pub type Listing = (Vec<RecordingSummary>, Vec<(PathBuf, FormatError)>);

/// Summaries of every readable recording in a directory, by name.
/// Unreadable sidecars are reported in the second list.
pub fn list_recordings(dir: impl AsRef<Path>) -> Result<Listing> {
    let dir = dir.as_ref();
    let mut ok = Vec::new();
    let mut bad = Vec::new();
    for entry in std::fs::read_dir(dir).map_err(io_err(dir))? {
        let entry = entry.map_err(io_err(dir))?;
        let p = entry.path();
        if !crate::paths::RecordingPaths::is_sidecar(&p) {
            continue;
        }
        match Recording::open(&p) {
            Ok(r) => ok.push(RecordingSummary::of(&r)),
            Err(e) => bad.push((p, e)),
        }
    }
    ok.sort_by(|a, b| a.name.cmp(&b.name));
    Ok((ok, bad))
}

/// Copies the stream-time range `[start_s, end_s)` (None: to the end) of
/// every receiver into a new recording at `out_base`. Byte ranges are cut
/// on whole words per receiver's own timeline; events inside the range are
/// kept with shifted offsets; the sidecar's gain, channel and divider are
/// set to the state in force at the cut; start host times move with the
/// cut; the truth of simulated recordings is cut to the same samples.
pub fn extract_range(
    src: &Recording,
    out_base: impl AsRef<Path>,
    start_s: f64,
    end_s: Option<f64>,
    opts: WriterOptions,
) -> Result<Recording> {
    if start_s.is_nan() || start_s < 0.0 || end_s.is_some_and(|e| e.is_nan() || e <= start_s) {
        return Err(FormatError::Invalid(format!(
            "bad range {start_s} .. {end_s:?}"
        )));
    }
    let mut sidecar = src.sidecar().clone();
    let mut ranges: Vec<(RxId, u64, u64)> = Vec::new();
    for r in &mut sidecar.receivers {
        let tl = src.timeline(r.rx)?;
        let len = src.rx_bytes(r.rx)?;
        let b0 = tl.byte_of_time(start_s).min(len);
        let b1 = end_s.map_or(len, |e| tl.byte_of_time(e).min(len)).max(b0);
        ranges.push((r.rx, b0, b1));
        r.start_host_ns += (tl.time_of_byte(b0) * 1e9) as u64;
        // State in force at the cut.
        for ev in src.events().for_rx(r.rx) {
            if ev.byte.is_none_or(|b| b >= b0) {
                continue;
            }
            match ev.kind {
                EventKind::Gain { index } => r.gain = index,
                EventKind::Tune { mhz, offset_khz } => {
                    r.channel_mhz = mhz;
                    r.offset_khz = offset_khz;
                }
                EventKind::Clk { divider } if divider > 0 => {
                    r.sample_format.divider = divider;
                    r.sample_rate_hz = C5_PLL_HZ / divider as f64;
                }
                _ => {}
            }
        }
    }
    let span = match end_s {
        Some(e) => format!("{start_s:.6} .. {e:.6} s"),
        None => format!("{start_s:.6} s .. end"),
    };
    let note = format!("cut from {} [{span}]", src.name());
    sidecar.notes = if sidecar.notes.is_empty() {
        note
    } else {
        format!("{}; {note}", sidecar.notes)
    };
    sidecar.created_utc = crate::utc_now_string();
    let mut w = RecordingWriter::create(out_base.as_ref(), sidecar, opts)?;

    let mut buf = vec![0u8; 4 << 20];
    for &(rx, b0, b1) in &ranges {
        let mut reader = src.raw_reader(rx)?;
        reader.seek_byte(b0)?;
        let mut left = b1 - b0;
        while left > 0 {
            let n = (buf.len() as u64).min(left) as usize;
            let got = reader.read(&mut buf[..n])?;
            if got == 0 {
                break;
            }
            w.write_raw(rx, &buf[..got])?;
            left -= got as u64;
        }
    }

    let ref_rx = ranges.first().map(|r| r.0);
    let (win0, win1) = match ref_rx {
        Some(rx) => {
            let info = src.receiver(rx)?;
            let tl = src.timeline(rx)?;
            let (_, b0, b1) = ranges[0];
            (
                info.start_host_ns + (tl.time_of_byte(b0) * 1e9) as u64,
                info.start_host_ns + (tl.time_of_byte(b1) * 1e9) as u64,
            )
        }
        None => (0, u64::MAX),
    };
    for ev in &src.events().events {
        let mut ev = ev.clone();
        match (ev.rx, ev.byte) {
            (Some(rx), Some(b)) => {
                let Some(&(_, b0, b1)) = ranges.iter().find(|r| r.0 == rx) else {
                    continue;
                };
                let inside = b >= b0 && (b < b1 || (b == b1 && b1 == src.rx_bytes(rx)?));
                if !inside {
                    continue;
                }
                ev.byte = Some(b - b0);
            }
            _ => {
                if ev.host_ns < win0 || ev.host_ns > win1 {
                    continue;
                }
            }
        }
        w.log_event(&ev)?;
    }

    if let Some(t) = src.truth_info() {
        if let Some(&(_, b0, b1)) = ranges.iter().find(|r| r.0 == t.rx) {
            let wb = src.receiver(t.rx)?.word_bytes();
            let mut truth = src.truth()?;
            let (mut s, e) = (b0 / wb, b1 / wb);
            while s < e {
                let n = ((e - s) as usize).min(1 << 20);
                let v = truth.read_range(s, n)?;
                if v.is_empty() {
                    break;
                }
                w.write_truth(&v)?;
                s += v.len() as u64;
            }
        }
    }
    let base = w.paths().base();
    w.close()?;
    Recording::open(base)
}

/// Cuts a recording into consecutive pieces of `piece_s` seconds named
/// `NAME_000`, `NAME_001`, ... in `out_dir`. Returns the new recordings.
pub fn split(src: &Recording, out_dir: impl AsRef<Path>, piece_s: f64, opts: WriterOptions) -> Result<Vec<Recording>> {
    if piece_s.is_nan() || piece_s <= 0.0 {
        return Err(FormatError::Invalid(format!("piece length {piece_s}")));
    }
    let total = src.duration_s();
    let mut out = Vec::new();
    let mut k = 0usize;
    let mut t = 0.0;
    while t < total - 1e-9 {
        let end = (t + piece_s).min(total);
        let base = out_dir.as_ref().join(format!("{}_{k:03}", src.name()));
        let last = end >= total - 1e-9;
        out.push(extract_range(src, base, t, if last { None } else { Some(end) }, opts.clone())?);
        k += 1;
        t = end;
    }
    Ok(out)
}
