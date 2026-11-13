// SPDX-License-Identifier: GPL-3.0-or-later
//! Writing recordings.
//!
//! The sidecar is written when the recording is created (with
//! `closed_cleanly = false`) and rewritten by [`RecordingWriter::close`]
//! with the final byte counts and `closed_cleanly = true`. Raw streams go
//! through large buffers (several MiB per receiver, so a 27-40 MB/s stream
//! costs a handful of write calls per second); events are flushed to the
//! OS after every line, so a crash loses at most the raw data still in the
//! buffers and never corrupts what is on disk. A reader of a recording that
//! was not closed takes byte counts from the file sizes.

use crate::events::Event;
use crate::paths::RecordingPaths;
use crate::sidecar::Sidecar;
use crate::{io_err, EventKind, FormatError, Result};
use openvrx_core::{RawBlock, RxId};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

/// Writer settings.
#[derive(Clone, Debug)]
pub struct WriterOptions {
    /// Buffer per raw stream.
    pub buffer_bytes: usize,
    /// Replace an existing recording of the same name instead of failing.
    pub overwrite: bool,
    /// `sync_data` the events file after every event (survives power loss,
    /// not just a process crash; slower).
    pub sync_events: bool,
}

impl Default for WriterOptions {
    fn default() -> Self {
        WriterOptions {
            buffer_bytes: 8 << 20,
            overwrite: false,
            sync_events: false,
        }
    }
}

struct RxStream {
    rx: RxId,
    path: PathBuf,
    file: BufWriter<File>,
    /// Bytes appended to the file.
    written: u64,
    /// Stream byte of file byte 0 (set by the first `write_block`).
    origin: Option<u64>,
    /// Next expected stream byte.
    next_stream: u64,
    /// (stream byte where data resumed, total bytes lost before it).
    gaps: Vec<(u64, u64)>,
    /// `written` when writeback was last started (see `start_writeback`).
    kicked: u64,
}

/// Bytes between writeback kicks of a raw file.
const WRITEBACK_EVERY: u64 = 8 << 20;

/// Starts writeback of the file's dirty pages without waiting for it.
/// Without this the kernel lets dirty data pile up to vm.dirty_ratio
/// (hundreds of MB) and then blocks the writer while it flushes, which on a
/// USB drive can outlast the stream's buffering even when the drive is far
/// faster than the recording. Kicking every 8 MiB keeps the dirty data
/// small, so nothing ever waits on a big flush.
#[cfg(target_os = "linux")]
fn start_writeback(f: &File) {
    use std::os::fd::AsRawFd;
    // SAFETY: fd is open for the call; offset 0 and length 0 mean the
    // whole file; SYNC_FILE_RANGE_WRITE only queues the writes.
    unsafe {
        libc::sync_file_range(f.as_raw_fd(), 0, 0, libc::SYNC_FILE_RANGE_WRITE);
    }
}

#[cfg(not(target_os = "linux"))]
fn start_writeback(_f: &File) {}

impl RxStream {
    /// Starts writeback every WRITEBACK_EVERY bytes (data still in the
    /// BufWriter waits for the next kick).
    fn maybe_kick(&mut self) {
        if self.written - self.kicked >= WRITEBACK_EVERY {
            self.kicked = self.written;
            start_writeback(self.file.get_ref());
        }
    }

    fn lost_before(&self, stream_byte: u64) -> u64 {
        match self.gaps.partition_point(|&(s, _)| s <= stream_byte) {
            0 => 0,
            k => self.gaps[k - 1].1,
        }
    }
}

/// Creates and fills one recording.
pub struct RecordingWriter {
    paths: RecordingPaths,
    sidecar: Sidecar,
    streams: Vec<RxStream>,
    events: BufWriter<File>,
    events_path: PathBuf,
    truth: Option<(PathBuf, BufWriter<File>)>,
    truth_samples: u64,
    opts: WriterOptions,
    closed: bool,
}

fn create_file(path: &Path, overwrite: bool) -> Result<File> {
    let mut o = OpenOptions::new();
    o.write(true);
    if overwrite {
        o.create(true).truncate(true);
    } else {
        o.create_new(true);
    }
    o.open(path).map_err(|e| {
        if e.kind() == std::io::ErrorKind::AlreadyExists {
            FormatError::Exists(path.to_path_buf())
        } else {
            io_err(path)(e)
        }
    })
}

/// Linux: marks a new, still empty raw file no-copy-on-write (what
/// `chattr +C` sets). On btrfs, the Fedora default, that also turns off
/// compression and checksums for the file. Without it, every writeback (each
/// 5 s by default) compressed and checksummed about 130 MB of a 27 MB/s
/// recording at once, and that burst of kernel CPU starved the USB stream into
/// dropping blocks, for the live picture too (2026-11-04 on the bench host:
/// 47 gaps in 40 s in a normal folder, 0 in a no-COW one). On other
/// filesystems the flag is refused or meaningless; errors are ignored, so this
/// never fails a recording. The flag only takes on an empty file.
#[cfg(all(target_os = "linux", target_pointer_width = "64"))]
fn prefer_nocow(f: &File) {
    use std::os::fd::AsRawFd;
    // _IOR('f', 1, long) and _IOW('f', 2, long); the kernel passes an int.
    const FS_IOC_GETFLAGS: u64 = 0x8008_6601;
    const FS_IOC_SETFLAGS: u64 = 0x4008_6602;
    const FS_NOCOW_FL: libc::c_int = 0x0080_0000;
    let fd = f.as_raw_fd();
    let mut flags: libc::c_int = 0;
    // SAFETY: fd is open for the call's duration and both ioctls read or
    // write one c_int through a valid pointer.
    unsafe {
        if libc::ioctl(fd, FS_IOC_GETFLAGS as _, &mut flags as *mut libc::c_int) == 0
            && flags & FS_NOCOW_FL == 0
        {
            flags |= FS_NOCOW_FL;
            let _ = libc::ioctl(fd, FS_IOC_SETFLAGS as _, &flags as *const libc::c_int);
        }
    }
}

#[cfg(not(all(target_os = "linux", target_pointer_width = "64")))]
fn prefer_nocow(_f: &File) {}

impl RecordingWriter {
    /// Creates `BASE.openvrx.json`, `BASE.rxK.bin` per receiver,
    /// `BASE.events.jsonl` and, when `sidecar.sim.truth` is set,
    /// `BASE.truth.f32`. File names in the sidecar are filled in; byte
    /// counts start at 0 and `closed_cleanly` at false.
    pub fn create(base: impl AsRef<Path>, mut sidecar: Sidecar, opts: WriterOptions) -> Result<Self> {
        let paths = RecordingPaths::from_base(base.as_ref());
        if paths.name.is_empty() {
            return Err(FormatError::Invalid("empty recording name".into()));
        }
        if sidecar.receivers.is_empty() {
            return Err(FormatError::Invalid("a recording needs at least one receiver".into()));
        }
        std::fs::create_dir_all(&paths.dir).map_err(io_err(&paths.dir))?;
        let sidecar_path = paths.sidecar();
        if !opts.overwrite && sidecar_path.exists() {
            return Err(FormatError::Exists(sidecar_path));
        }
        sidecar.closed_cleanly = false;
        for r in &mut sidecar.receivers {
            r.file = paths.rx_name(r.rx);
            r.bytes = 0;
        }
        if let Some(t) = sidecar.sim.as_mut().and_then(|s| s.truth.as_mut()) {
            t.file = paths.truth_name();
        }
        sidecar.validate()?;

        let mut streams = Vec::with_capacity(sidecar.receivers.len());
        for r in &sidecar.receivers {
            let path = paths.rx(r.rx);
            let f = create_file(&path, opts.overwrite)?;
            prefer_nocow(&f);
            streams.push(RxStream {
                rx: r.rx,
                path,
                file: BufWriter::with_capacity(opts.buffer_bytes.max(4096), f),
                written: 0,
                origin: None,
                next_stream: 0,
                gaps: Vec::new(),
                kicked: 0,
            });
        }
        let events_path = paths.events();
        let events = BufWriter::new(create_file(&events_path, opts.overwrite)?);
        let truth = match sidecar.truth() {
            Some(_) => {
                let p = paths.truth();
                let f = create_file(&p, opts.overwrite)?;
                Some((p, BufWriter::with_capacity(4 << 20, f)))
            }
            None => None,
        };
        sidecar.write_atomic(&sidecar_path)?;
        Ok(RecordingWriter {
            paths,
            sidecar,
            streams,
            events,
            events_path,
            truth,
            truth_samples: 0,
            opts,
            closed: false,
        })
    }

    pub fn paths(&self) -> &RecordingPaths {
        &self.paths
    }

    /// Sidecar as it will be written (byte counts updated at close).
    pub fn sidecar(&self) -> &Sidecar {
        &self.sidecar
    }

    /// Edits the sidecar (notes, gains...) before it is rewritten at close.
    pub fn sidecar_mut(&mut self) -> &mut Sidecar {
        &mut self.sidecar
    }

    fn stream(&mut self, rx: RxId) -> Result<&mut RxStream> {
        self.streams
            .iter_mut()
            .find(|s| s.rx == rx)
            .ok_or(FormatError::NoReceiver(rx))
    }

    /// Appends raw bytes to a receiver's file.
    pub fn write_raw(&mut self, rx: RxId, data: &[u8]) -> Result<()> {
        let s = self.stream(rx)?;
        s.file.write_all(data).map_err(io_err(&s.path))?;
        s.written += data.len() as u64;
        s.maybe_kick();
        if let Some(o) = s.origin {
            s.next_stream = s.next_stream.max(o) + data.len() as u64;
        }
        Ok(())
    }

    /// Appends a block, keeping stream positions: the first block fixes
    /// the stream origin; a block that starts after the expected position
    /// logs a `usb_gap` event (reason `stream_gap`) and the missing bytes
    /// are simply absent from the file; bytes already written are skipped.
    /// Returns the number of bytes appended.
    pub fn write_block(&mut self, block: &RawBlock) -> Result<u64> {
        let s = self.stream(block.rx)?;
        let len = block.data.len() as u64;
        let mut skip = 0u64;
        let mut gap = None;
        match s.origin {
            None => {
                s.origin = Some(block.first_byte);
                s.next_stream = block.first_byte;
            }
            Some(_) => {
                if block.first_byte > s.next_stream {
                    let lost = block.first_byte - s.next_stream;
                    let total = s.gaps.last().map_or(0, |g| g.1) + lost;
                    s.gaps.push((block.first_byte, total));
                    gap = Some((s.written, lost));
                    s.next_stream = block.first_byte;
                } else if block.first_byte < s.next_stream {
                    skip = (s.next_stream - block.first_byte).min(len);
                }
            }
        }
        let data = &block.data[skip as usize..];
        s.file.write_all(data).map_err(io_err(&s.path))?;
        s.written += data.len() as u64;
        s.maybe_kick();
        s.next_stream += data.len() as u64;
        if let Some((at, lost)) = gap {
            self.log_event(&Event::new(
                Some(block.rx),
                block.host_ns,
                Some(at),
                EventKind::UsbGap {
                    lost_bytes: lost,
                    reason: "stream_gap".into(),
                },
            ))?;
        }
        Ok(data.len() as u64)
    }

    /// File offset of a stream byte position (see [`write_block`]), or None
    /// when it lies before the recording started or no block has been
    /// written yet.
    ///
    /// [`write_block`]: RecordingWriter::write_block
    pub fn file_byte_for_stream(&self, rx: RxId, stream_byte: u64) -> Option<u64> {
        let s = self.streams.iter().find(|s| s.rx == rx)?;
        let origin = s.origin?;
        if stream_byte < origin {
            return None;
        }
        Some((stream_byte - origin).saturating_sub(s.lost_before(stream_byte)))
    }

    /// Appends one event line and flushes it to the OS. `byte` must be a
    /// file offset (see [`log_stream_event`] for stream positions).
    ///
    /// [`log_stream_event`]: RecordingWriter::log_stream_event
    pub fn log_event(&mut self, ev: &Event) -> Result<()> {
        let mut line = ev.to_json_line();
        line.push('\n');
        self.events
            .write_all(line.as_bytes())
            .and_then(|_| self.events.flush())
            .map_err(io_err(&self.events_path))?;
        if self.opts.sync_events {
            self.events
                .get_ref()
                .sync_data()
                .map_err(io_err(&self.events_path))?;
        }
        Ok(())
    }

    /// Logs an event whose `byte` is a stream position (as in
    /// `RawBlock::first_byte`); it is converted to a file offset. Events of
    /// a receiver from before the recording's first byte are dropped
    /// (returns false). Events logged before any data use offset 0.
    pub fn log_stream_event(&mut self, ev: &Event) -> Result<bool> {
        let mut ev = ev.clone();
        if let (Some(rx), Some(b)) = (ev.rx, ev.byte) {
            let s = self
                .streams
                .iter()
                .find(|s| s.rx == rx)
                .ok_or(FormatError::NoReceiver(rx))?;
            match s.origin {
                None => ev.byte = Some(0),
                Some(o) if b < o => return Ok(false),
                Some(_) => ev.byte = self.file_byte_for_stream(rx, b),
            }
        }
        self.log_event(&ev)?;
        Ok(true)
    }

    /// Appends truth samples (simulator recordings only).
    pub fn write_truth(&mut self, samples: &[f32]) -> Result<()> {
        let (path, w) = self.truth.as_mut().ok_or(FormatError::NoTruth)?;
        let mut buf = [0u8; 4096];
        for chunk in samples.chunks(1024) {
            for (k, v) in chunk.iter().enumerate() {
                buf[4 * k..4 * k + 4].copy_from_slice(&v.to_le_bytes());
            }
            w.write_all(&buf[..4 * chunk.len()]).map_err(io_err(path))?;
        }
        self.truth_samples += samples.len() as u64;
        Ok(())
    }

    /// Truth samples written so far.
    pub fn truth_samples(&self) -> u64 {
        self.truth_samples
    }

    /// Bytes appended to a receiver's file so far.
    pub fn bytes_written(&self, rx: RxId) -> u64 {
        self.streams.iter().find(|s| s.rx == rx).map_or(0, |s| s.written)
    }

    /// Next expected stream byte of a receiver (after `write_block`).
    pub fn next_stream_byte(&self, rx: RxId) -> Option<u64> {
        let s = self.streams.iter().find(|s| s.rx == rx)?;
        s.origin.map(|_| s.next_stream)
    }

    /// Flushes every buffer to the OS.
    pub fn flush(&mut self) -> Result<()> {
        for s in &mut self.streams {
            s.file.flush().map_err(io_err(&s.path))?;
        }
        self.events.flush().map_err(io_err(&self.events_path))?;
        if let Some((p, w)) = &mut self.truth {
            w.flush().map_err(io_err(p))?;
        }
        Ok(())
    }

    /// Flushes and rewrites the sidecar with the current byte counts while
    /// keeping `closed_cleanly = false` (a checkpoint during long runs).
    pub fn checkpoint(&mut self) -> Result<()> {
        self.flush()?;
        self.update_counts();
        self.sidecar.closed_cleanly = false;
        self.sidecar.write_atomic(&self.paths.sidecar())
    }

    fn update_counts(&mut self) {
        for s in &self.streams {
            if let Some(r) = self.sidecar.receivers.iter_mut().find(|r| r.rx == s.rx) {
                r.bytes = s.written;
            }
        }
    }

    /// Flushes, syncs and rewrites the sidecar with byte counts and
    /// `closed_cleanly = true`. Returns the final sidecar.
    pub fn close(mut self) -> Result<Sidecar> {
        self.flush()?;
        for s in &mut self.streams {
            s.file.get_ref().sync_all().map_err(io_err(&s.path))?;
        }
        self.events
            .get_ref()
            .sync_all()
            .map_err(io_err(&self.events_path))?;
        if let Some((p, w)) = &mut self.truth {
            w.get_ref().sync_all().map_err(io_err(p))?;
        }
        self.update_counts();
        self.sidecar.closed_cleanly = true;
        self.sidecar.write_atomic(&self.paths.sidecar())?;
        self.closed = true;
        Ok(self.sidecar.clone())
    }
}

impl Drop for RecordingWriter {
    /// Best effort when not closed: flush buffers and record the byte
    /// counts, leaving `closed_cleanly = false`.
    fn drop(&mut self) {
        if !self.closed {
            let _ = self.checkpoint();
        }
    }
}
