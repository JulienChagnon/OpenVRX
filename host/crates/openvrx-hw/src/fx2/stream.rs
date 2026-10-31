// SPDX-License-Identifier: GPL-3.0-or-later
//! The capture engine: one dedicated thread per FX2.
//!
//! The thread keeps 8 to 32 bulk IN transfers queued on EP2, turns every
//! completion into a [`RawBlock`] (stamped with `clock::now_ns()`, with
//! monotonically increasing `seq` and `first_byte`) and sends it over a
//! bounded channel. When the consumer falls behind, the oldest queued block
//! is dropped and counted: USB is never blocked. Transfer errors, stalls and
//! disconnects are reported as [`StreamEvent`]s and recovered by restarting
//! the stream (clear halt, `STOP`/`START`) or reconnecting the device.
//!
//! Byte accounting: `first_byte` counts every byte received from USB since
//! the stream was created, including blocks later dropped because the
//! consumer was slow, so a jump in `first_byte` (or `seq`) tells the
//! consumer exactly how much it missed on the host side. Data the device or
//! the bus lost (FIFO overflow, recovery after an error) never reached the
//! host; it is reported through `FifoFull` and `Gap` events with estimates.

use crate::error::{HwError, Result, XferError};
use crate::fx2::endpoint::InEndpoint;
use crate::fx2::protocol::{ControlIo, Fx2Requests, Fx2Status, StartFlags, PACKET_BYTES, STATUS_EP_BYTES};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use openvrx_core::clock::now_ns;
use openvrx_core::{RawBlock, RxId};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// An open connection to one FX2.
pub struct StreamLink {
    pub control: Arc<dyn ControlIo>,
    pub bulk: Box<dyn InEndpoint>,
    /// EP1 status endpoint; when absent the status is polled with
    /// `GET_STATUS` every 100 ms.
    pub status: Option<Box<dyn InEndpoint>>,
    pub description: String,
}

/// Something that can (re)open an FX2. Called again after a disconnect.
pub trait StreamPort: Send + 'static {
    fn connect(&mut self) -> Result<StreamLink>;
    fn describe(&self) -> String;
}

/// Capture settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StreamConfig {
    pub rx: RxId,
    /// Bulk transfers kept in flight (8..=32).
    pub transfers: usize,
    /// Bytes per transfer (16 KiB..=64 KiB, multiple of 512).
    pub transfer_bytes: usize,
    /// `START` flags (width, IFCLK polarity, test modes).
    pub flags: StartFlags,
    /// Watch ratio (1 = full stream).
    pub watch_n: u16,
    /// Expected byte rate, used to estimate lost bytes across gaps.
    pub nominal_byte_rate: Option<f64>,
    /// Report `NoData` after this long without a completion.
    pub no_data_ms: u64,
    /// Reconnect backoff bounds.
    pub reconnect_min_ms: u64,
    pub reconnect_max_ms: u64,
    /// Register the capture thread with MMCSS "Pro Audio".
    pub mmcss: bool,
    /// Emit a `Transfer` timing event every N completions (0 = never).
    pub transfer_event_every: u32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        StreamConfig {
            rx: 0,
            transfers: 16,
            transfer_bytes: 32 * 1024,
            flags: StartFlags::WIDTH16,
            watch_n: 1,
            nominal_byte_rate: None,
            no_data_ms: 1000,
            reconnect_min_ms: 200,
            reconnect_max_ms: 2000,
            mmcss: true,
            transfer_event_every: 64,
        }
    }
}

impl StreamConfig {
    pub fn validate(&self) -> Result<()> {
        if !(8..=32).contains(&self.transfers) {
            return Err(HwError::Invalid(format!("transfers {} not in 8..=32", self.transfers)));
        }
        if !(16 * 1024..=64 * 1024).contains(&self.transfer_bytes) || !self.transfer_bytes.is_multiple_of(PACKET_BYTES)
        {
            return Err(HwError::Invalid(format!(
                "transfer size {} must be 16..64 KiB and a multiple of 512",
                self.transfer_bytes
            )));
        }
        if self.watch_n == 0 {
            return Err(HwError::Invalid("watch ratio 0 (use 1 for the full stream)".into()));
        }
        Ok(())
    }

    /// Flags actually sent with `START` (adds `WATCH` when watch_n > 1).
    pub fn start_flags(&self) -> StartFlags {
        self.flags.with(StartFlags::WATCH, self.watch_n > 1)
    }
}

/// Producer side of a block channel: drops the oldest block when full.
#[derive(Clone)]
pub struct BlockSink {
    tx: Sender<RawBlock>,
    /// Second handle on the queue, used only to pop the oldest block.
    oldest: Receiver<RawBlock>,
}

/// Creates a bounded block channel whose producer drops the oldest block
/// instead of blocking. Keep the receiver for the consumer.
pub fn block_channel(capacity: usize) -> (BlockSink, Receiver<RawBlock>) {
    let (tx, rx) = bounded(capacity.max(1));
    (BlockSink { tx, oldest: rx.clone() }, rx)
}

/// What happened to a pushed block.
#[derive(Debug, PartialEq, Eq)]
pub enum Push {
    Queued,
    /// Queued after dropping the oldest block (returned for recycling).
    DroppedOldest(u64, usize),
    /// The queue stayed full; this block was dropped.
    DroppedNew,
}

impl BlockSink {
    /// Pushes without blocking. Returns what was dropped, if anything, and
    /// hands back the dropped buffer for reuse.
    pub fn push(&self, block: RawBlock) -> (Push, Option<Vec<u8>>) {
        match self.tx.try_send(block) {
            Ok(()) => (Push::Queued, None),
            Err(TrySendError::Disconnected(b)) => (Push::DroppedNew, Some(b.data)),
            Err(TrySendError::Full(b)) => {
                let old = self.oldest.try_recv().ok();
                match self.tx.try_send(b) {
                    Ok(()) => match old {
                        Some(o) => {
                            let info = Push::DroppedOldest(o.seq, o.data.len());
                            (info, Some(o.data))
                        }
                        None => (Push::Queued, None),
                    },
                    Err(TrySendError::Full(b)) | Err(TrySendError::Disconnected(b)) => {
                        (Push::DroppedNew, old.map(|o| o.data).or(Some(b.data)))
                    }
                }
            }
        }
    }
}

/// Running statistics of completion intervals (latency jitter).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct IntervalStats {
    pub count: u64,
    pub mean_us: f64,
    m2: f64,
    pub max_us: f64,
    pub min_us: f64,
}

impl IntervalStats {
    pub fn add(&mut self, us: f64) {
        self.count += 1;
        if self.count == 1 {
            self.min_us = us;
            self.max_us = us;
        } else {
            self.min_us = self.min_us.min(us);
            self.max_us = self.max_us.max(us);
        }
        let d = us - self.mean_us;
        self.mean_us += d / self.count as f64;
        self.m2 += d * (us - self.mean_us);
    }

    /// Standard deviation in microseconds.
    pub fn std_us(&self) -> f64 {
        if self.count < 2 {
            0.0
        } else {
            (self.m2 / (self.count - 1) as f64).sqrt()
        }
    }
}

/// Watch-mode bookkeeping: which committed 512-byte packets arrived.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WatchLedger {
    pub watch_n: u16,
    /// `first_byte` where this watch session began.
    pub start_byte: u64,
    /// Committed packets received by the host.
    pub packets_received: u64,
    /// Device counters at the session start (from the first status).
    base_committed: Option<u32>,
    /// Packets the device committed this session (latest status).
    pub packets_committed: u64,
    pub packets_skipped: u64,
    base_skipped: u32,
}

impl WatchLedger {
    pub fn new(watch_n: u16, start_byte: u64) -> WatchLedger {
        WatchLedger {
            watch_n,
            start_byte,
            ..Default::default()
        }
    }

    pub fn on_block(&mut self, bytes: usize) {
        self.packets_received += (bytes / PACKET_BYTES) as u64;
    }

    pub fn on_status(&mut self, s: &Fx2Status) {
        let base = *self.base_committed.get_or_insert_with(|| {
            self.base_skipped = s.packets_skipped;
            s.packets_committed.wrapping_sub(self.packets_received as u32)
        });
        self.packets_committed = s.packets_committed.wrapping_sub(base) as u64;
        self.packets_skipped = s.packets_skipped.wrapping_sub(self.base_skipped) as u64;
    }

    /// Committed index of the packet holding stream byte `byte`.
    pub fn packet_index(&self, byte: u64) -> Option<u64> {
        byte.checked_sub(self.start_byte).map(|d| d / PACKET_BYTES as u64)
    }

    /// Position of a committed packet in the full-rate packet sequence
    /// (packet k of the session is source packet k * N).
    pub fn source_packet(&self, index: u64) -> u64 {
        index * self.watch_n.max(1) as u64
    }

    /// Committed packets not (yet) received: in flight or lost.
    pub fn outstanding(&self) -> u64 {
        self.packets_committed.saturating_sub(self.packets_received)
    }
}

/// Capture statistics (a snapshot is returned by [`Fx2Stream::stats`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StreamStats {
    pub connected: bool,
    pub running: bool,
    /// Bytes received from USB.
    pub bytes: u64,
    /// Completed transfers.
    pub transfers: u64,
    /// Blocks and bytes dropped because the consumer was slow.
    pub dropped_blocks: u64,
    pub dropped_bytes: u64,
    pub transfer_errors: u64,
    pub stalls: u64,
    pub disconnects: u64,
    pub restarts: u64,
    /// Bytes estimated lost across recovery gaps.
    pub gap_bytes_est: u64,
    /// FIFO-full events reported by the firmware since the stream started.
    pub fifo_full_events: u64,
    /// Byte rate over the last ~second.
    pub rate_bps: f64,
    /// Completion interval statistics.
    pub interval: IntervalStats,
    pub last_status: Option<Fx2Status>,
    pub watch: Option<WatchLedger>,
    pub started_ns: u64,
    pub last_completion_ns: u64,
}

impl StreamStats {
    /// Fraction of received bytes the consumer lost (host side).
    pub fn host_loss_fraction(&self) -> f64 {
        if self.bytes == 0 {
            0.0
        } else {
            self.dropped_bytes as f64 / self.bytes as f64
        }
    }
}

/// Things a consumer (UI, recorder, log) should know about.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum StreamEvent {
    Connected {
        description: String,
        host_ns: u64,
    },
    Started {
        flags: StartFlags,
        watch_n: u16,
        byte: u64,
        host_ns: u64,
    },
    Status {
        status: Fx2Status,
        byte: u64,
        host_ns: u64,
    },
    /// The firmware saw its FIFO full (samples dropped on the device).
    FifoFull {
        new_events: u32,
        total: u64,
        byte: u64,
        host_ns: u64,
    },
    TransferError {
        error: XferError,
        byte: u64,
        host_ns: u64,
    },
    /// Data lost between `byte` and the next block (recording `usb_gap`).
    Gap {
        byte: u64,
        lost_bytes_est: u64,
        reason: String,
        host_ns: u64,
    },
    /// Blocks dropped because the consumer was slow (aggregated).
    HostDrop {
        blocks: u64,
        bytes: u64,
        byte: u64,
        host_ns: u64,
    },
    /// Timing sample of one bulk completion (recording `transfer`).
    Transfer {
        seq: u64,
        bytes: usize,
        byte: u64,
        host_ns: u64,
    },
    /// No completion for a while (IFCLK missing, C5 clock off).
    NoData {
        since_ns: u64,
        host_ns: u64,
    },
    DataResumed {
        host_ns: u64,
    },
    Reconnecting {
        attempt: u32,
        error: String,
        host_ns: u64,
    },
    WatchChanged {
        watch_n: u16,
        byte: u64,
        host_ns: u64,
    },
    Stopped {
        reason: String,
        byte: u64,
        host_ns: u64,
    },
}

enum Cmd {
    Stop,
    SetWatch(u16),
    ResetCounters,
}

/// Handle to a running capture thread. Dropping it stops the thread.
pub struct Fx2Stream {
    cmd: Sender<Cmd>,
    thread: Option<JoinHandle<()>>,
    stats: Arc<Mutex<StreamStats>>,
    events: Receiver<StreamEvent>,
    blocks: Option<Receiver<RawBlock>>,
    recycle: Sender<Vec<u8>>,
}

impl Fx2Stream {
    /// Starts capture with its own block channel of `cfg.transfers * 4`
    /// blocks (get it with [`Fx2Stream::blocks`]).
    pub fn start<P: StreamPort>(port: P, cfg: StreamConfig) -> Result<Fx2Stream> {
        let (sink, rx) = block_channel(cfg.transfers * 4);
        let mut s = Self::start_with_sink(port, cfg, sink)?;
        s.blocks = Some(rx);
        Ok(s)
    }

    /// Starts capture into an existing sink, so the consumer's channel
    /// survives stream restarts (used by `Receiver`).
    pub fn start_with_sink<P: StreamPort>(port: P, cfg: StreamConfig, sink: BlockSink) -> Result<Fx2Stream> {
        cfg.validate()?;
        let (cmd_tx, cmd_rx) = bounded(64);
        let (ev_tx, ev_rx) = bounded(4096);
        let (rec_tx, rec_rx) = bounded(cfg.transfers * 4);
        let stats = Arc::new(Mutex::new(StreamStats {
            started_ns: now_ns(),
            ..Default::default()
        }));
        let st = stats.clone();
        let thread = std::thread::Builder::new()
            .name(format!("openvrx-usb-rx{}", cfg.rx))
            .spawn(move || {
                let mut eng = Engine {
                    port,
                    cfg,
                    sink,
                    cmd: cmd_rx,
                    events: ev_tx,
                    stats: st,
                    pool: rec_rx,
                    seq: 0,
                    byte: 0,
                    pending_drop: (0, 0),
                    last_drop_event_ns: 0,
                    session_blocks: 0,
                };
                eng.run();
            })
            .map_err(HwError::Io)?;
        Ok(Fx2Stream {
            cmd: cmd_tx,
            thread: Some(thread),
            stats,
            events: ev_rx,
            blocks: None,
            recycle: rec_tx,
        })
    }

    /// The block receiver when the stream owns its channel.
    pub fn blocks(&self) -> Option<&Receiver<RawBlock>> {
        self.blocks.as_ref()
    }

    pub fn events(&self) -> &Receiver<StreamEvent> {
        &self.events
    }

    /// Hand a consumed block's buffer back to avoid allocations.
    pub fn recycle(&self, buf: Vec<u8>) {
        let _ = self.recycle.try_send(buf);
    }

    pub fn stats(&self) -> StreamStats {
        self.stats.lock().map(|s| s.clone()).unwrap_or_default()
    }

    /// Switches watch mode on the fly (vendor `SET_WATCH`).
    pub fn set_watch(&self, n: u16) {
        let _ = self.cmd.send(Cmd::SetWatch(n.max(1)));
    }

    pub fn reset_counters(&self) {
        let _ = self.cmd.send(Cmd::ResetCounters);
    }

    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(|t| t.is_finished())
    }

    /// Stops streaming (`STOP`, cancel transfers) and joins the thread.
    pub fn stop(mut self) -> StreamStats {
        self.shutdown();
        self.stats()
    }

    fn shutdown(&mut self) {
        let _ = self.cmd.send(Cmd::Stop);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for Fx2Stream {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Why the inner loop ended.
enum Exit {
    Stop,
    /// Restart on the same link (stall, fault).
    Restart(XferError),
    /// Link lost: reconnect.
    Reconnect(String),
}

struct Engine<P: StreamPort> {
    port: P,
    cfg: StreamConfig,
    sink: BlockSink,
    cmd: Receiver<Cmd>,
    events: Sender<StreamEvent>,
    stats: Arc<Mutex<StreamStats>>,
    pool: Receiver<Vec<u8>>,
    seq: u64,
    byte: u64,
    /// Aggregated host drops not yet reported: (blocks, bytes).
    pending_drop: (u64, u64),
    last_drop_event_ns: u64,
    /// Blocks delivered by the current START session.
    session_blocks: u64,
}

impl<P: StreamPort> Engine<P> {
    fn emit(&self, e: StreamEvent) {
        let _ = self.events.try_send(e);
    }

    fn stats(&self, f: impl FnOnce(&mut StreamStats)) {
        if let Ok(mut s) = self.stats.lock() {
            f(&mut s);
        }
    }

    /// Applies commands that arrive while no session runs; true on stop.
    fn idle_command(&mut self, c: Cmd) -> bool {
        match c {
            Cmd::Stop => true,
            Cmd::SetWatch(n) => {
                self.cfg.watch_n = n;
                false
            }
            Cmd::ResetCounters => false,
        }
    }

    fn stop_requested(&mut self) -> bool {
        let cmds: Vec<Cmd> = self.cmd.try_iter().collect();
        let mut stop = false;
        for c in cmds {
            stop |= self.idle_command(c);
        }
        stop
    }

    fn run(&mut self) {
        let _mmcss = if self.cfg.mmcss {
            crate::power::MmcssGuard::pro_audio().ok()
        } else {
            None
        };
        let mut attempt: u32 = 0;
        let mut backoff = self.cfg.reconnect_min_ms;
        let mut gap_from: Option<(u64, String)> = None;
        let mut consecutive_failures = 0u32;
        'outer: loop {
            if self.stop_requested() {
                break;
            }
            let mut link = match self.port.connect() {
                Ok(l) => {
                    attempt = 0;
                    backoff = self.cfg.reconnect_min_ms;
                    l
                }
                Err(e) => {
                    attempt += 1;
                    self.emit(StreamEvent::Reconnecting {
                        attempt,
                        error: e.to_string(),
                        host_ns: now_ns(),
                    });
                    if self.sleep_or_stop(Duration::from_millis(backoff)) {
                        break;
                    }
                    backoff = (backoff * 2).min(self.cfg.reconnect_max_ms);
                    continue;
                }
            };
            self.stats(|s| s.connected = true);
            self.emit(StreamEvent::Connected {
                description: link.description.clone(),
                host_ns: now_ns(),
            });
            loop {
                let exit = self.stream_once(&mut link, &mut gap_from);
                match exit {
                    Exit::Stop => break 'outer,
                    Exit::Restart(e) => {
                        consecutive_failures = if self.session_blocks > 0 {
                            1
                        } else {
                            consecutive_failures + 1
                        };
                        self.stats(|s| s.restarts += 1);
                        if gap_from.is_none() {
                            gap_from = Some((now_ns(), e.to_string()));
                        }
                        if e == XferError::Stall {
                            if let Err(err) = link.bulk.clear_halt() {
                                log::warn!("clear halt failed: {err}");
                            }
                        }
                        if consecutive_failures >= 3 {
                            // Three failed restarts in a row: reopen the device.
                            consecutive_failures = 0;
                            self.stats(|s| s.connected = false);
                            continue 'outer;
                        }
                        if self.sleep_or_stop(Duration::from_millis(20)) {
                            break 'outer;
                        }
                    }
                    Exit::Reconnect(reason) => {
                        self.stats(|s| {
                            s.connected = false;
                            s.running = false;
                            s.disconnects += 1;
                        });
                        if gap_from.is_none() {
                            gap_from = Some((now_ns(), reason));
                        }
                        if self.sleep_or_stop(Duration::from_millis(self.cfg.reconnect_min_ms)) {
                            break 'outer;
                        }
                        continue 'outer;
                    }
                }
            }
        }
        self.flush_drop_event(true);
        self.stats(|s| {
            s.running = false;
            s.connected = false;
        });
        self.emit(StreamEvent::Stopped {
            reason: "stopped by host".into(),
            byte: self.byte,
            host_ns: now_ns(),
        });
    }

    /// Sleeps up to `d`; true when a stop arrived.
    fn sleep_or_stop(&mut self, d: Duration) -> bool {
        let deadline = std::time::Instant::now() + d;
        loop {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                return false;
            }
            match self.cmd.recv_timeout(left) {
                Ok(c) => {
                    if self.idle_command(c) {
                        return true;
                    }
                }
                Err(_) => return false,
            }
        }
    }

    fn take_buffer(&self) -> Vec<u8> {
        let n = self.cfg.transfer_bytes;
        match self.pool.try_recv() {
            Ok(mut v) => {
                v.resize(n, 0);
                v
            }
            Err(_) => vec![0u8; n],
        }
    }

    /// Runs one START..STOP session on a link.
    fn stream_once(&mut self, link: &mut StreamLink, gap_from: &mut Option<(u64, String)>) -> Exit {
        self.session_blocks = 0;
        let flags = self.cfg.start_flags();
        let watch_n = self.cfg.watch_n;
        // Clean state, queue transfers, then start the FIFO.
        if let Err(e) = link.control.stop() {
            return self.control_failure(e);
        }
        for _ in 0..self.cfg.transfers {
            let b = self.take_buffer();
            link.bulk.submit(b);
        }
        if let Err(e) = link.control.start(flags, watch_n) {
            self.drain(link);
            return self.control_failure(e);
        }
        let started_ns = now_ns();
        self.emit(StreamEvent::Started {
            flags,
            watch_n,
            byte: self.byte,
            host_ns: started_ns,
        });
        let byte_at_start = self.byte;
        self.stats(|s| {
            s.running = true;
            s.watch = (watch_n > 1).then(|| WatchLedger::new(watch_n, byte_at_start));
        });

        // Status poller for this session.
        let poll_stop = Arc::new(AtomicBool::new(false));
        let poller = self.spawn_status_poller(link, poll_stop.clone());

        let mut last_ns = started_ns;
        let mut prev_completion_ns = 0u64;
        let mut no_data_reported = false;
        let mut window = (started_ns, self.byte);
        let exit = loop {
            // Commands.
            let mut stop = false;
            for c in self.cmd.try_iter() {
                match c {
                    Cmd::Stop => stop = true,
                    Cmd::SetWatch(n) => match link.control.set_watch(n) {
                        Ok(()) => {
                            self.cfg.watch_n = n;
                            let b = self.byte;
                            self.stats(|s| s.watch = (n > 1).then(|| WatchLedger::new(n, b)));
                            self.emit(StreamEvent::WatchChanged {
                                watch_n: n,
                                byte: b,
                                host_ns: now_ns(),
                            });
                        }
                        Err(e) => log::warn!("SET_WATCH {n} failed: {e}"),
                    },
                    Cmd::ResetCounters => {
                        if let Err(e) = link.control.reset_counters() {
                            log::warn!("RESET_COUNTERS failed: {e}");
                        }
                    }
                }
            }
            if stop {
                let _ = link.control.stop();
                break Exit::Stop;
            }
            let Some(c) = link.bulk.wait_complete(Duration::from_millis(100)) else {
                let now = now_ns();
                if !no_data_reported && now - last_ns > self.cfg.no_data_ms * 1_000_000 {
                    no_data_reported = true;
                    self.emit(StreamEvent::NoData {
                        since_ns: last_ns,
                        host_ns: now,
                    });
                }
                self.flush_drop_event(false);
                continue;
            };
            let host_ns = now_ns();
            match c.status {
                Ok(()) => {
                    if no_data_reported {
                        no_data_reported = false;
                        self.emit(StreamEvent::DataResumed { host_ns });
                    }
                    if let Some((since, reason)) = gap_from.take() {
                        let rate = self
                            .cfg
                            .nominal_byte_rate
                            .unwrap_or_else(|| self.stats.lock().map(|s| s.rate_bps).unwrap_or(0.0));
                        let est = ((host_ns.saturating_sub(since)) as f64 * 1e-9 * rate) as u64;
                        let est = est / PACKET_BYTES as u64 * PACKET_BYTES as u64;
                        self.stats(|s| s.gap_bytes_est += est);
                        self.emit(StreamEvent::Gap {
                            byte: self.byte,
                            lost_bytes_est: est,
                            reason,
                            host_ns,
                        });
                    }
                    let len = c.data.len();
                    let block = RawBlock {
                        rx: self.cfg.rx,
                        seq: self.seq,
                        first_byte: self.byte,
                        host_ns,
                        data: c.data,
                    };
                    if self.cfg.transfer_event_every > 0
                        && self.seq.is_multiple_of(self.cfg.transfer_event_every as u64)
                    {
                        self.emit(StreamEvent::Transfer {
                            seq: self.seq,
                            bytes: len,
                            byte: self.byte,
                            host_ns,
                        });
                    }
                    self.seq += 1;
                    self.byte += len as u64;
                    let (push, spare) = self.sink.push(block);
                    match push {
                        Push::Queued => {}
                        Push::DroppedOldest(_, n) => {
                            self.pending_drop.0 += 1;
                            self.pending_drop.1 += n as u64;
                        }
                        Push::DroppedNew => {
                            self.pending_drop.0 += 1;
                            self.pending_drop.1 += len as u64;
                        }
                    }
                    self.session_blocks += 1;
                    let interval = (prev_completion_ns != 0).then(|| (host_ns - prev_completion_ns) as f64 * 1e-3);
                    prev_completion_ns = host_ns;
                    let byte = self.byte;
                    let dropped = u64::from(push != Push::Queued);
                    let rate = if host_ns - window.0 >= 1_000_000_000 {
                        let r = (byte - window.1) as f64 / ((host_ns - window.0) as f64 * 1e-9);
                        window = (host_ns, byte);
                        Some(r)
                    } else {
                        None
                    };
                    let dropped_len = match push {
                        Push::Queued => 0,
                        Push::DroppedOldest(_, n) => n as u64,
                        Push::DroppedNew => len as u64,
                    };
                    self.stats(|s| {
                        s.bytes = byte;
                        s.transfers += 1;
                        s.dropped_blocks += dropped;
                        s.dropped_bytes += dropped_len;
                        s.last_completion_ns = host_ns;
                        if let Some(i) = interval {
                            s.interval.add(i);
                        }
                        if let Some(r) = rate {
                            s.rate_bps = r;
                        }
                        if let Some(w) = s.watch.as_mut() {
                            w.on_block(len);
                        }
                    });
                    self.flush_drop_event(false);
                    last_ns = host_ns;
                    let mut buf = spare.unwrap_or_else(|| self.take_buffer());
                    buf.resize(self.cfg.transfer_bytes, 0);
                    link.bulk.submit(buf);
                }
                Err(XferError::Disconnected) => {
                    self.stats(|s| s.transfer_errors += 1);
                    self.emit(StreamEvent::TransferError {
                        error: XferError::Disconnected,
                        byte: self.byte,
                        host_ns,
                    });
                    break Exit::Reconnect("device disconnected".into());
                }
                Err(e) => {
                    self.stats(|s| {
                        s.transfer_errors += 1;
                        if e == XferError::Stall {
                            s.stalls += 1;
                        }
                    });
                    self.emit(StreamEvent::TransferError {
                        error: e.clone(),
                        byte: self.byte,
                        host_ns,
                    });
                    let _ = link.control.stop();
                    break Exit::Restart(e);
                }
            }
        };
        poll_stop.store(true, Ordering::Relaxed);
        self.drain(link);
        if let Some(p) = poller {
            let _ = p.join();
        }
        self.stats(|s| s.running = false);
        exit
    }

    fn control_failure(&self, e: HwError) -> Exit {
        match e {
            HwError::Disconnected(m) => Exit::Reconnect(m),
            HwError::Transfer(XferError::Disconnected) => Exit::Reconnect("device disconnected".into()),
            other => Exit::Reconnect(format!("control request failed: {other}")),
        }
    }

    /// Cancels and collects every pending transfer (bounded wait).
    fn drain(&mut self, link: &mut StreamLink) {
        link.bulk.cancel_all();
        let t0 = now_ns();
        while link.bulk.pending() > 0 && now_ns() - t0 < 2_000_000_000 {
            // Data completing during cancellation arrives after STOP; it is
            // discarded and covered by the gap estimate.
            let _ = link.bulk.wait_complete(Duration::from_millis(200));
        }
    }

    fn flush_drop_event(&mut self, force: bool) {
        let now = now_ns();
        if self.pending_drop.0 > 0 && (force || now - self.last_drop_event_ns > 100_000_000) {
            self.emit(StreamEvent::HostDrop {
                blocks: self.pending_drop.0,
                bytes: self.pending_drop.1,
                byte: self.byte,
                host_ns: now,
            });
            self.pending_drop = (0, 0);
            self.last_drop_event_ns = now;
        }
    }

    fn spawn_status_poller(&self, link: &mut StreamLink, stop: Arc<AtomicBool>) -> Option<JoinHandle<()>> {
        let ep = link.status.take();
        let control = link.control.clone();
        let stats = self.stats.clone();
        let events = self.events.clone();
        let rx = self.cfg.rx;
        let poller = StatusPoller {
            stats,
            events,
            prev: None,
        };
        let r = std::thread::Builder::new()
            .name(format!("openvrx-fx2-status-rx{rx}"))
            .spawn(move || poller.run(ep, control, stop));
        match r {
            Ok(h) => Some(h),
            Err(e) => {
                log::warn!("cannot start the status poller: {e}");
                None
            }
        }
    }
}

struct StatusPoller {
    stats: Arc<Mutex<StreamStats>>,
    events: Sender<StreamEvent>,
    prev: Option<Fx2Status>,
}

impl StatusPoller {
    fn run(mut self, mut ep: Option<Box<dyn InEndpoint>>, control: Arc<dyn ControlIo>, stop: Arc<AtomicBool>) {
        if let Some(ep) = ep.as_mut() {
            for _ in 0..2 {
                ep.submit(vec![0u8; STATUS_EP_BYTES]);
            }
        }
        let mut last_emit = 0u64;
        while !stop.load(Ordering::Relaxed) {
            let block = match ep.as_mut() {
                Some(ep) => match ep.wait_complete(Duration::from_millis(50)) {
                    None => continue,
                    Some(c) => match c.status {
                        Ok(()) => {
                            let s = Fx2Status::decode(&c.data);
                            let mut buf = c.data;
                            buf.resize(STATUS_EP_BYTES, 0);
                            ep.submit(buf);
                            s.ok()
                        }
                        Err(XferError::Cancelled) | Err(XferError::Disconnected) => break,
                        Err(e) => {
                            log::debug!("status endpoint: {e}");
                            if ep.clear_halt().is_err() {
                                break;
                            }
                            let mut buf = c.data;
                            buf.resize(STATUS_EP_BYTES, 0);
                            ep.submit(buf);
                            None
                        }
                    },
                },
                None => {
                    std::thread::sleep(Duration::from_millis(100));
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    control.get_status().ok()
                }
            };
            if let Some(s) = block {
                let now = now_ns();
                // Publish every block to stats, events at most every 250 ms.
                let emit = now - last_emit > 250_000_000;
                if emit {
                    last_emit = now;
                }
                self.publish(s, now, emit);
            }
        }
        if let Some(mut ep) = ep {
            ep.cancel_all();
            let t0 = now_ns();
            while ep.pending() > 0 && now_ns() - t0 < 500_000_000 {
                let _ = ep.wait_complete(Duration::from_millis(50));
            }
        }
    }

    fn publish(&mut self, s: Fx2Status, now: u64, emit: bool) {
        let new_full = self.prev.map_or(0, |p| s.fifo_full_since(&p));
        // A firmware restart resets counters; ignore the wrap that implies.
        let new_full = if new_full > 1_000_000 { 0 } else { new_full };
        self.prev = Some(s);
        let mut byte = 0;
        let mut total = 0;
        if let Ok(mut st) = self.stats.lock() {
            st.last_status = Some(s);
            st.fifo_full_events += new_full as u64;
            byte = st.bytes;
            total = st.fifo_full_events;
            if let Some(w) = st.watch.as_mut() {
                w.on_status(&s);
            }
        }
        if new_full > 0 {
            let _ = self.events.try_send(StreamEvent::FifoFull {
                new_events: new_full,
                total,
                byte,
                host_ns: now,
            });
        }
        if emit {
            let _ = self.events.try_send(StreamEvent::Status {
                status: s,
                byte,
                host_ns: now,
            });
        }
    }
}

/// Captures about `bytes` from a port in one short session (no thread):
/// connect, queue transfers, `START`, collect completions, `STOP`. Each
/// returned buffer is one transfer, contiguous in itself. Used by
/// calibration, the lane self-test and receiver pairing.
pub fn capture_once(
    port: &mut dyn StreamPort,
    flags: StartFlags,
    bytes: usize,
    transfer_bytes: usize,
    timeout: Duration,
) -> Result<Vec<Vec<u8>>> {
    let tb = transfer_bytes.max(PACKET_BYTES) / PACKET_BYTES * PACKET_BYTES;
    let mut link = port.connect()?;
    link.control.stop()?;
    let want = bytes.div_ceil(tb).max(1);
    let queued = want.min(8);
    for _ in 0..queued {
        link.bulk.submit(vec![0u8; tb]);
    }
    let mut submitted = queued;
    link.control.start(flags, 1)?;
    let t0 = std::time::Instant::now();
    let mut out: Vec<Vec<u8>> = Vec::new();
    let mut got = 0usize;
    let mut failure = None;
    while got < bytes {
        if t0.elapsed() > timeout {
            failure = Some(HwError::Timeout(format!(
                "captured {got} of {bytes} bytes (is the sample clock running?)"
            )));
            break;
        }
        let Some(c) = link.bulk.wait_complete(Duration::from_millis(50)) else {
            continue;
        };
        match c.status {
            Ok(()) => {
                got += c.data.len();
                if !c.data.is_empty() {
                    out.push(c.data);
                }
                if submitted < want {
                    link.bulk.submit(vec![0u8; tb]);
                    submitted += 1;
                }
            }
            Err(e) => {
                failure = Some(HwError::Transfer(e));
                break;
            }
        }
    }
    let _ = link.control.stop();
    link.bulk.cancel_all();
    let t1 = std::time::Instant::now();
    while link.bulk.pending() > 0 && t1.elapsed() < Duration::from_secs(1) {
        let _ = link.bulk.wait_complete(Duration::from_millis(50));
    }
    match failure {
        Some(e) => Err(e),
        None => Ok(out),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fx2::endpoint::sim::SimFx2;
    use crate::fx2::protocol::req;

    fn cfg() -> StreamConfig {
        StreamConfig {
            transfers: 8,
            transfer_bytes: 16 * 1024,
            mmcss: false,
            nominal_byte_rate: Some(8e6),
            transfer_event_every: 4,
            ..Default::default()
        }
    }

    fn collect(rx: &Receiver<RawBlock>, n: usize) -> Vec<RawBlock> {
        let mut v = Vec::new();
        while v.len() < n {
            match rx.recv_timeout(Duration::from_secs(5)) {
                Ok(b) => v.push(b),
                Err(_) => panic!("only {} of {n} blocks arrived", v.len()),
            }
        }
        v
    }

    fn counter_ok(blocks: &[RawBlock]) -> bool {
        let mut prev: Option<u16> = None;
        for b in blocks {
            for w in b.data.as_chunks::<2>().0 {
                let v = u16::from_le_bytes([w[0], w[1]]);
                if let Some(p) = prev {
                    if v != p.wrapping_add(1) {
                        return false;
                    }
                }
                prev = Some(v);
            }
        }
        true
    }

    #[test]
    fn capture_once_collects_bytes() {
        let sim = SimFx2::new(20e6);
        let mut port = sim.port();
        let got = capture_once(&mut port, StartFlags::WIDTH16, 100_000, 16384, Duration::from_secs(2)).unwrap();
        let total: usize = got.iter().map(Vec::len).sum();
        assert!(total >= 100_000);
        assert!(counter_ok(
            &got.into_iter()
                .enumerate()
                .map(|(k, d)| RawBlock {
                    rx: 0,
                    seq: k as u64,
                    first_byte: 0,
                    host_ns: 0,
                    data: d
                })
                .collect::<Vec<_>>()
        ));
        assert!(!sim.with(|d| d.running));
        let dead = SimFx2::new(0.0);
        let mut port = dead.port();
        assert!(matches!(
            capture_once(&mut port, StartFlags::WIDTH16, 1000, 16384, Duration::from_millis(200)),
            Err(HwError::Timeout(_))
        ));
    }

    #[test]
    fn config_validation() {
        assert!(cfg().validate().is_ok());
        assert!(StreamConfig { transfers: 4, ..cfg() }.validate().is_err());
        assert!(StreamConfig {
            transfer_bytes: 1000,
            ..cfg()
        }
        .validate()
        .is_err());
        assert!(StreamConfig {
            transfer_bytes: 128 * 1024,
            ..cfg()
        }
        .validate()
        .is_err());
        assert!(StreamConfig { watch_n: 0, ..cfg() }.validate().is_err());
        let w = StreamConfig { watch_n: 8, ..cfg() };
        assert!(w.start_flags().contains(StartFlags::WATCH));
    }

    #[test]
    fn sink_drops_oldest() {
        let (sink, rx) = block_channel(2);
        let mk = |seq| RawBlock {
            rx: 0,
            seq,
            first_byte: seq * 10,
            host_ns: 0,
            data: vec![0; 10],
        };
        assert_eq!(sink.push(mk(0)).0, Push::Queued);
        assert_eq!(sink.push(mk(1)).0, Push::Queued);
        let (p, spare) = sink.push(mk(2));
        assert_eq!(p, Push::DroppedOldest(0, 10));
        assert_eq!(spare.unwrap().len(), 10);
        assert_eq!(rx.recv().unwrap().seq, 1);
        assert_eq!(rx.recv().unwrap().seq, 2);
    }

    #[test]
    fn interval_stats() {
        let mut s = IntervalStats::default();
        for x in [10.0, 12.0, 14.0] {
            s.add(x);
        }
        assert!((s.mean_us - 12.0).abs() < 1e-9);
        assert!((s.std_us() - 2.0).abs() < 1e-9);
        assert_eq!((s.min_us, s.max_us), (10.0, 14.0));
    }

    #[test]
    fn streams_contiguous_blocks() {
        let sim = SimFx2::new(16e6);
        let stream = Fx2Stream::start(sim.port(), cfg()).unwrap();
        let blocks = collect(stream.blocks().unwrap(), 40);
        for (k, b) in blocks.iter().enumerate() {
            assert_eq!(b.seq, k as u64);
            assert_eq!(b.first_byte, k as u64 * 16384);
            assert_eq!(b.data.len(), 16384);
        }
        assert!(blocks.windows(2).all(|w| w[1].host_ns >= w[0].host_ns));
        assert!(counter_ok(&blocks));
        let st = stream.stop();
        assert!(st.transfers >= 40);
        assert_eq!(st.dropped_blocks, 0);
        assert!(st.interval.count > 30);
        // START was sent with 16-bit width, then STOP at the end.
        let reqs = sim.with(|d| d.requests.clone());
        assert!(reqs.iter().any(|r| r.0 == req::START && r.1 & 1 == 1));
        assert_eq!(reqs.iter().filter(|r| r.0 == req::START).count(), 1);
        assert_eq!(reqs.last().unwrap().0, req::STOP);
    }

    #[test]
    fn slow_consumer_drops_and_counts() {
        let sim = SimFx2::new(64e6);
        let stream = Fx2Stream::start(sim.port(), cfg()).unwrap();
        std::thread::sleep(Duration::from_millis(400));
        let rx = stream.blocks().unwrap().clone();
        let got: Vec<RawBlock> = rx.try_iter().collect();
        let st = stream.stop();
        assert!(st.dropped_blocks > 0, "{st:?}");
        // The queued blocks are the newest ones and still in order.
        assert!(got.windows(2).all(|w| w[1].seq > w[0].seq));
        assert!(got.windows(2).all(|w| w[1].first_byte > w[0].first_byte));
        assert_eq!(st.bytes, st.transfers * 16384);
    }

    #[test]
    fn recovers_from_stall_and_disconnect() {
        let sim = SimFx2::new(16e6);
        sim.with(|d| {
            d.faults.push_back((10, XferError::Stall));
            d.faults.push_back((30, XferError::Disconnected));
        });
        let stream = Fx2Stream::start(
            sim.port(),
            StreamConfig {
                reconnect_min_ms: 20,
                ..cfg()
            },
        )
        .unwrap();
        sim.with(|d| d.connect_failures = 2);
        let blocks = collect(stream.blocks().unwrap(), 60);
        // seq and first_byte stay monotonic across recovery.
        assert!(blocks.windows(2).all(|w| w[1].seq == w[0].seq + 1));
        assert!(blocks.windows(2).all(|w| w[1].first_byte == w[0].first_byte + 16384));
        let events: Vec<StreamEvent> = stream.events().try_iter().collect();
        let st = stream.stop();
        assert_eq!(st.stalls, 1);
        assert_eq!(st.disconnects, 1);
        assert!(st.restarts >= 1);
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Gap { .. })));
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Reconnecting { .. })));
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Transfer { .. })));
        assert!(sim.with(|d| d.connects) >= 2);
    }

    #[test]
    fn watch_mode_switch_and_status() {
        let sim = SimFx2::new(8e6);
        let stream = Fx2Stream::start(sim.port(), cfg()).unwrap();
        let rx = stream.blocks().unwrap().clone();
        collect(&rx, 5);
        stream.set_watch(16);
        collect(&rx, 10);
        std::thread::sleep(Duration::from_millis(100));
        let st = stream.stats();
        let w = st.watch.clone().expect("watch ledger");
        assert_eq!(w.watch_n, 16);
        assert!(w.packets_received > 0);
        assert_eq!(w.source_packet(3), 48);
        assert_eq!(w.packet_index(w.start_byte + 1024), Some(2));
        assert!(st.last_status.is_some());
        assert_eq!(sim.with(|d| d.watch_n), 16);
        let events: Vec<StreamEvent> = stream.events().try_iter().collect();
        assert!(events
            .iter()
            .any(|e| matches!(e, StreamEvent::WatchChanged { watch_n: 16, .. })));
        drop(stream);
    }

    #[test]
    fn reports_no_data() {
        let sim = SimFx2::new(0.0);
        let stream = Fx2Stream::start(
            sim.port(),
            StreamConfig {
                no_data_ms: 100,
                ..cfg()
            },
        )
        .unwrap();
        std::thread::sleep(Duration::from_millis(400));
        let events: Vec<StreamEvent> = stream.events().try_iter().collect();
        assert!(
            events.iter().any(|e| matches!(e, StreamEvent::NoData { .. })),
            "{events:?}"
        );
        drop(stream);
    }

    #[test]
    fn watch_ledger_reconciles() {
        let mut w = WatchLedger::new(8, 1000);
        w.on_block(512 * 4);
        let s = Fx2Status {
            packets_committed: 104,
            packets_skipped: 700,
            ..Default::default()
        };
        w.on_status(&s);
        assert_eq!(w.packets_committed, 4);
        let s2 = Fx2Status {
            packets_committed: 110,
            packets_skipped: 742,
            ..Default::default()
        };
        w.on_status(&s2);
        assert_eq!(w.packets_committed, 10);
        assert_eq!(w.packets_skipped, 42);
        assert_eq!(w.outstanding(), 6);
        assert_eq!(w.packet_index(999), None);
    }
}
