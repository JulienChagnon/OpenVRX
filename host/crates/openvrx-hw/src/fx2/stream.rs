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

