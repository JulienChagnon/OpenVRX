// SPDX-License-Identifier: GPL-3.0-or-later
//! Reading recordings.

use crate::events::{EventKind, EventLog};
use crate::paths::RecordingPaths;
use crate::replay::{Replay, ReplayOptions};
use crate::sidecar::{ReceiverInfo, Sidecar, TruthInfo};
use crate::{io_err, FormatError, Result};
use openvrx_core::lanes::C5_PLL_HZ;
use openvrx_core::{Complex32, RawBlock, RxId};
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// Maps byte offsets of one raw file to stream time, accounting for
/// `usb_gap` events (time jumps by the lost bytes) and `clk` events (the
/// byte rate changes).
#[derive(Clone, Debug, PartialEq)]
pub struct Timeline {
    /// (file byte, stream time in s, byte rate) per segment, by byte.
    segments: Vec<(u64, f64, f64)>,
    word_bytes: u64,
}

impl Timeline {
    /// Builds the timeline of `info` from its receiver's events.
    pub fn build(info: &ReceiverInfo, events: &EventLog) -> Timeline {
        let word_bytes = info.word_bytes();
        let mut rate = info.byte_rate();
        let mut segments = vec![(0u64, 0.0f64, rate)];
        let mut evs: Vec<(u64, &EventKind)> = events
            .for_rx(info.rx)
            .filter_map(|e| e.byte.map(|b| (b, &e.kind)))
            .filter(|(_, k)| matches!(k, EventKind::UsbGap { .. } | EventKind::Clk { .. }))
            .collect();
        evs.sort_by_key(|(b, _)| *b);
        for (b, kind) in evs {
            let &(b0, t0, r0) = segments.last().expect("non-empty");
            let t = t0 + (b.saturating_sub(b0)) as f64 / r0;
            match kind {
                EventKind::UsbGap { lost_bytes, .. } => {
                    segments.push((b, t + *lost_bytes as f64 / r0, r0));
                }
                EventKind::Clk { divider } if *divider > 0 => {
                    rate = C5_PLL_HZ / *divider as f64 * word_bytes as f64
                        / info.watch_n.max(1) as f64;
                    segments.push((b, t, rate));
                }
                _ => {}
            }
        }
        Timeline {
            segments,
            word_bytes,
        }
    }

    /// A timeline with a constant byte rate.
    pub fn constant(byte_rate: f64, word_bytes: u64) -> Timeline {
        Timeline {
            segments: vec![(0, 0.0, byte_rate)],
            word_bytes: word_bytes.max(1),
        }
    }

    /// Stream time (s since the first byte) of a byte offset.
    pub fn time_of_byte(&self, byte: u64) -> f64 {
        let k = self.segments.partition_point(|s| s.0 <= byte).max(1) - 1;
        let (b0, t0, r) = self.segments[k];
        t0 + (byte - b0.min(byte)) as f64 / r
    }

    /// Word-aligned byte offset at or before stream time `t` (inside a
    /// gap: the byte where data resumes).
    pub fn byte_of_time(&self, t: f64) -> u64 {
        let t = t.max(0.0);
        let k = self.segments.partition_point(|s| s.1 <= t).max(1) - 1;
        let (b0, t0, r) = self.segments[k];
        let mut b = b0 + ((t - t0).max(0.0) * r).floor() as u64;
        if let Some(next) = self.segments.get(k + 1) {
            b = b.min(next.0);
        }
        b - b % self.word_bytes
    }

    /// Segments as (file byte, stream time, byte rate).
    pub fn segments(&self) -> &[(u64, f64, f64)] {
        &self.segments
    }
}

/// An open recording.
#[derive(Clone, Debug)]
pub struct Recording {
    paths: RecordingPaths,
    sidecar: Sidecar,
    events: EventLog,
    /// Usable bytes per receiver, in sidecar order.
    bytes: Vec<u64>,
    timelines: Vec<Timeline>,
}

impl Recording {
    /// Opens a recording from any of its files or its base path.
    pub fn open(path: impl AsRef<Path>) -> Result<Recording> {
        let paths = RecordingPaths::resolve(path.as_ref());
        let sidecar_path = paths.sidecar();
        if !sidecar_path.exists() {
            return Err(FormatError::NotRecording(format!(
                "{} not found",
                sidecar_path.display()
            )));
        }
        let sidecar = Sidecar::read(&sidecar_path)?;
        let events = EventLog::read(&paths.events())?;
        let mut bytes = Vec::with_capacity(sidecar.receivers.len());
        let mut timelines = Vec::with_capacity(sidecar.receivers.len());
        for r in &sidecar.receivers {
            let p = paths.file(&r.file);
            let size = match std::fs::metadata(&p) {
                Ok(m) => m.len(),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => 0,
                Err(e) => return Err(io_err(&p)(e)),
            };
            let wb = r.word_bytes();
            let usable = if sidecar.closed_cleanly {
                r.bytes.min(size)
            } else {
                size
            };
            bytes.push(usable - usable % wb);
            timelines.push(Timeline::build(r, &events));
        }
        Ok(Recording {
            paths,
            sidecar,
            events,
            bytes,
            timelines,
        })
    }

    pub fn paths(&self) -> &RecordingPaths {
        &self.paths
    }

    pub fn name(&self) -> &str {
        &self.paths.name
    }

    pub fn sidecar(&self) -> &Sidecar {
        &self.sidecar
    }

    pub fn receivers(&self) -> &[ReceiverInfo] {
        &self.sidecar.receivers
    }

    pub fn events(&self) -> &EventLog {
        &self.events
    }

    fn index(&self, rx: RxId) -> Result<usize> {
        self.sidecar
            .receivers
            .iter()
            .position(|r| r.rx == rx)
            .ok_or(FormatError::NoReceiver(rx))
    }

    pub fn receiver(&self, rx: RxId) -> Result<&ReceiverInfo> {
        Ok(&self.sidecar.receivers[self.index(rx)?])
    }

    /// Usable bytes of a receiver: the sidecar count when closed cleanly,
    /// otherwise the file size (whole words).
    pub fn rx_bytes(&self, rx: RxId) -> Result<u64> {
        Ok(self.bytes[self.index(rx)?])
    }

    /// Words (samples) of a receiver.
    pub fn rx_samples(&self, rx: RxId) -> Result<u64> {
        let r = self.receiver(rx)?;
        Ok(self.rx_bytes(rx)? / r.word_bytes())
    }

    pub fn timeline(&self, rx: RxId) -> Result<&Timeline> {
        Ok(&self.timelines[self.index(rx)?])
    }

    /// Stream duration of a receiver in seconds (including gaps).
    pub fn rx_duration_s(&self, rx: RxId) -> Result<f64> {
        let b = self.rx_bytes(rx)?;
        Ok(self.timeline(rx)?.time_of_byte(b))
    }

    /// Longest receiver duration.
    pub fn duration_s(&self) -> f64 {
        self.sidecar
            .receivers
            .iter()
            .filter_map(|r| self.rx_duration_s(r.rx).ok())
            .fold(0.0, f64::max)
    }

    /// Path of a receiver's raw file.
    pub fn rx_path(&self, rx: RxId) -> Result<PathBuf> {
        Ok(self.paths.file(&self.receiver(rx)?.file))
    }

    /// A seekable reader over a receiver's raw bytes.
    pub fn raw_reader(&self, rx: RxId) -> Result<RawReader> {
        let i = self.index(rx)?;
        let info = &self.sidecar.receivers[i];
        let path = self.paths.file(&info.file);
        let file = File::open(&path).map_err(io_err(&path))?;
        Ok(RawReader {
            path,
            file: BufReader::with_capacity(1 << 20, file),
            rx,
            len: self.bytes[i],
            pos: 0,
            word_bytes: info.word_bytes(),
            timeline: self.timelines[i].clone(),
            start_host_ns: info.start_host_ns,
            lut: None,
            lanes: info.sample_format.lanes.clone(),
            seq: 0,
        })
    }

    pub fn truth_info(&self) -> Option<&TruthInfo> {
        self.sidecar.truth()
    }

    /// The truth file of a simulated recording.
    pub fn truth(&self) -> Result<TruthReader> {
        let info = self.sidecar.truth().ok_or(FormatError::NoTruth)?.clone();
        let path = self.paths.file(&info.file);
        let file = File::open(&path).map_err(io_err(&path))?;
        let len = file.metadata().map_err(io_err(&path))?.len() / 4;
        Ok(TruthReader {
            path,
            file,
            len,
            info,
        })
    }

    /// A replay iterator over all (or selected) receivers.
    pub fn replay(&self, opts: ReplayOptions) -> Result<Replay> {
        Replay::new(self, opts)
    }
}

/// Seekable reader over one receiver's raw bytes.
pub struct RawReader {
    path: PathBuf,
    file: BufReader<File>,
    rx: RxId,
    len: u64,
    pos: u64,
    word_bytes: u64,
    timeline: Timeline,
    start_host_ns: u64,
    lanes: openvrx_core::LaneFormat,
    lut: Option<Vec<Complex32>>,
    seq: u64,
}

impl RawReader {
    pub fn rx(&self) -> RxId {
        self.rx
    }

    /// Usable length in bytes.
    pub fn len_bytes(&self) -> u64 {
        self.len
    }

    pub fn len_samples(&self) -> u64 {
        self.len / self.word_bytes
    }

    pub fn word_bytes(&self) -> u64 {
        self.word_bytes
    }

    pub fn timeline(&self) -> &Timeline {
        &self.timeline
    }

    pub fn position_bytes(&self) -> u64 {
        self.pos
    }

    pub fn position_samples(&self) -> u64 {
        self.pos / self.word_bytes
    }

    /// Stream time of the current position.
    pub fn position_time(&self) -> f64 {
        self.timeline.time_of_byte(self.pos)
    }

    pub fn remaining_bytes(&self) -> u64 {
        self.len - self.pos
    }

    /// Seeks to a byte offset (rounded down to a whole word, clamped to
    /// the end). Returns the new position.
    pub fn seek_byte(&mut self, byte: u64) -> Result<u64> {
        let b = byte.min(self.len);
        let b = b - b % self.word_bytes;
        self.file.seek(SeekFrom::Start(b)).map_err(io_err(&self.path))?;
        self.pos = b;
        Ok(b)
    }

    /// Seeks to a sample (word) index.
    pub fn seek_sample(&mut self, sample: u64) -> Result<u64> {
        Ok(self.seek_byte(sample.saturating_mul(self.word_bytes))? / self.word_bytes)
    }

    /// Seeks to a stream time in seconds (see [`Timeline`]).
    pub fn seek_time(&mut self, t: f64) -> Result<u64> {
        let b = self.timeline.byte_of_time(t);
        self.seek_byte(b)
    }

    /// Reads up to `buf.len()` bytes (fewer only at the end).
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let n = (buf.len() as u64).min(self.len - self.pos) as usize;
        self.file
            .read_exact(&mut buf[..n])
            .map_err(io_err(&self.path))?;
        self.pos += n as u64;
        Ok(n)
    }

    /// Reads the next block of up to `max_bytes` (rounded down to whole
    /// words, at least one word). `first_byte` is the file offset and
    /// `host_ns` the recorded host time of that byte
    /// (`start_host_ns + stream time`). None at the end.
    pub fn read_block(&mut self, max_bytes: usize) -> Result<Option<RawBlock>> {
        let wb = self.word_bytes as usize;
        let want = (max_bytes - max_bytes % wb).max(wb);
        let n = (want as u64).min(self.len - self.pos) as usize;
        if n == 0 {
            return Ok(None);
        }
        let first = self.pos;
        let mut data = vec![0u8; n];
        self.read(&mut data)?;
        let block = RawBlock {
            rx: self.rx,
            seq: self.seq,
            first_byte: first,
            host_ns: self.start_host_ns + (self.timeline.time_of_byte(first) * 1e9) as u64,
            data,
        };
        self.seq += 1;
        Ok(Some(block))
    }

    /// Iterator over the rest of the stream in blocks of `block_bytes`.
    pub fn blocks(self, block_bytes: usize) -> RawBlocks {
        RawBlocks {
            reader: self,
            block_bytes,
        }
    }

    /// Reads and decodes up to `max_samples` words into I/Q (10-bit ADC
    /// units) with the receiver's lane format.
    pub fn read_iq(&mut self, max_samples: usize) -> Result<Vec<Complex32>> {
        let wb = self.word_bytes as usize;
        let mut raw = vec![0u8; max_samples * wb];
        let n = self.read(&mut raw)?;
        let lut = self.lut.get_or_insert_with(|| self.lanes.build_lut());
        Ok(raw[..n]
            .chunks_exact(wb)
            .map(|w| {
                let word = if wb == 2 {
                    u16::from_le_bytes([w[0], w[1]]) as usize
                } else {
                    w[0] as usize
                };
                lut[word]
            })
            .collect())
    }

    /// Reads up to `max_samples` raw words (little-endian on the wire).
    pub fn read_words(&mut self, max_samples: usize) -> Result<Vec<u16>> {
        let wb = self.word_bytes as usize;
        let mut raw = vec![0u8; max_samples * wb];
        let n = self.read(&mut raw)?;
        Ok(raw[..n]
            .chunks_exact(wb)
            .map(|w| {
                if wb == 2 {
                    u16::from_le_bytes([w[0], w[1]])
                } else {
                    w[0] as u16
                }
            })
            .collect())
    }
}

/// Chunked iteration over a raw stream.
pub struct RawBlocks {
    reader: RawReader,
    block_bytes: usize,
}

impl Iterator for RawBlocks {
    type Item = Result<RawBlock>;
    fn next(&mut self) -> Option<Self::Item> {
        self.reader.read_block(self.block_bytes).transpose()
    }
}

/// Simulator truth: clean composite, one f32 (IRE) per input sample of
/// the truth receiver.
pub struct TruthReader {
    path: PathBuf,
    file: File,
    len: u64,
    info: TruthInfo,
}

impl TruthReader {
    pub fn info(&self) -> &TruthInfo {
        &self.info
    }

    /// Number of samples.
    pub fn len(&self) -> u64 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Reads samples starting at `start` into `out`; returns how many.
    pub fn read_at(&mut self, start: u64, out: &mut [f32]) -> Result<usize> {
        if start >= self.len {
            return Ok(0);
        }
        let n = (out.len() as u64).min(self.len - start) as usize;
        let mut raw = vec![0u8; 4 * n];
        self.file
            .seek(SeekFrom::Start(4 * start))
            .and_then(|_| self.file.read_exact(&mut raw))
            .map_err(io_err(&self.path))?;
        for (k, o) in out.iter_mut().enumerate() {
            let b = &raw[4 * k..4 * k + 4];
            *o = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        }
        Ok(n)
    }

    /// Reads `n` samples from `start` (fewer at the end).
    pub fn read_range(&mut self, start: u64, n: usize) -> Result<Vec<f32>> {
        let mut v = vec![0.0; n];
        let got = self.read_at(start, &mut v)?;
        v.truncate(got);
        Ok(v)
    }

    pub fn read_all(&mut self) -> Result<Vec<f32>> {
        self.read_range(0, self.len as usize)
    }
}
