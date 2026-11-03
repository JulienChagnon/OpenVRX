// SPDX-License-Identifier: GPL-3.0-or-later
//! Rolling RAM buffer: the last N seconds of raw blocks per receiver, so a
//! recording can start before the record button was pressed.
//!
//! Capture threads call [`RollingBuffer::push`] (a short mutex hold: a
//! deque push and, when over budget, pops of the oldest blocks). The
//! recorder thread calls [`RollingBuffer::drain_into`] repeatedly while
//! recording: the first call writes the whole buffered history, later calls
//! write only blocks that arrived since, so the recording continues live.
//! Blocks are shared as `Arc`s, so draining 30 s of history never copies
//! data while holding the lock. If the recorder falls so far behind that
//! unwritten blocks are evicted, the writer logs the hole as a `usb_gap`.
//!
//! Events (gain changes, markers...) can be buffered as well, with `byte`
//! as a stream position; they are written with the history, converted to
//! file offsets.

use crate::events::Event;
use crate::writer::RecordingWriter;
use crate::Result;
use openvrx_core::{RawBlock, RxId};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard};

/// Rolling buffer limits.
#[derive(Clone, Debug)]
pub struct RollingConfig {
    /// History to keep, seconds.
    pub seconds: f64,
    /// Hard cap per receiver in bytes, whatever the rate.
    pub max_bytes_per_rx: usize,
    /// Spare buffers kept for reuse per receiver.
    pub spare_buffers: usize,
}

impl Default for RollingConfig {
    fn default() -> Self {
        RollingConfig {
            seconds: 30.0,
            max_bytes_per_rx: 2 << 30,
            spare_buffers: 64,
        }
    }
}

struct Ring {
    rx: RxId,
    blocks: VecDeque<Arc<RawBlock>>,
    bytes: usize,
    budget: usize,
    /// Stream position up to which data reached the current recording.
    written_until: Option<u64>,
    spares: Vec<Vec<u8>>,
    max_spares: usize,
    evicted_unwritten: u64,
}

impl Ring {
    fn evict(&mut self) {
        while self.bytes > self.budget && self.blocks.len() > 1 {
            let b = self.blocks.pop_front().expect("non-empty");
            self.bytes -= b.data.len();
            if let Some(w) = self.written_until {
                if b.first_byte + b.data.len() as u64 > w {
                    self.evicted_unwritten += 1;
                }
            }
            if let Ok(block) = Arc::try_unwrap(b) {
                if self.spares.len() < self.max_spares {
                    let mut v = block.data;
                    v.clear();
                    self.spares.push(v);
                }
            }
        }
    }
}

/// Counters from one [`RollingBuffer::drain_into`] call.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DrainStats {
    pub blocks: u64,
    pub bytes: u64,
    pub events: u64,
    /// Blocks evicted before they were written (recorder too slow).
    pub evicted_unwritten: u64,
}

/// Per-receiver RAM ring of raw blocks.
pub struct RollingBuffer {
    ids: Vec<RxId>,
    rings: Vec<Mutex<Ring>>,
    events: Mutex<VecDeque<Event>>,
    max_events: usize,
    recording: Mutex<bool>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

impl RollingBuffer {
    /// One ring per `(rx, byte rate)`; each holds `seconds x rate` bytes,
    /// capped at `max_bytes_per_rx`.
    pub fn new(receivers: &[(RxId, f64)], cfg: &RollingConfig) -> RollingBuffer {
        let rings = receivers
            .iter()
            .map(|&(rx, rate)| {
                let budget = ((cfg.seconds.max(0.0) * rate) as usize).min(cfg.max_bytes_per_rx);
                Mutex::new(Ring {
                    rx,
                    blocks: VecDeque::new(),
                    bytes: 0,
                    budget,
                    written_until: None,
                    spares: Vec::new(),
                    max_spares: cfg.spare_buffers,
                    evicted_unwritten: 0,
                })
            })
            .collect();
        RollingBuffer {
            ids: receivers.iter().map(|r| r.0).collect(),
            rings,
            events: Mutex::new(VecDeque::new()),
            max_events: 100_000,
            recording: Mutex::new(false),
        }
    }

    fn ring(&self, rx: RxId) -> Option<&Mutex<Ring>> {
        let i = self.ids.iter().position(|&id| id == rx)?;
        Some(&self.rings[i])
    }

    /// Adds a block (capture thread). Unknown receivers are ignored.
    pub fn push(&self, block: RawBlock) {
        self.push_arc(Arc::new(block));
    }

    /// Adds a shared block (the same `Arc` can go to the DSP thread).
    pub fn push_arc(&self, block: Arc<RawBlock>) {
        if let Some(r) = self.ring(block.rx) {
            let mut g = lock(r);
            g.bytes += block.data.len();
            g.blocks.push_back(block);
            g.evict();
        }
    }

    /// A recycled data buffer (empty, with capacity) for the next block of
    /// `rx`, or a new one.
    pub fn take_spare(&self, rx: RxId, capacity: usize) -> Vec<u8> {
        if let Some(r) = self.ring(rx) {
            if let Some(mut v) = lock(r).spares.pop() {
                v.reserve(capacity);
                return v;
            }
        }
        Vec::with_capacity(capacity)
    }

    /// Buffers an event; `byte` is a stream position (as in
    /// `RawBlock::first_byte`).
    pub fn push_event(&self, ev: Event) {
        let mut g = lock(&self.events);
        g.push_back(ev);
        while g.len() > self.max_events {
            g.pop_front();
        }
    }

    /// Bytes currently held for a receiver.
    pub fn buffered_bytes(&self, rx: RxId) -> usize {
        self.ring(rx).map_or(0, |r| lock(r).bytes)
    }

    /// Byte budget of a receiver.
    pub fn budget_bytes(&self, rx: RxId) -> usize {
        self.ring(rx).map_or(0, |r| lock(r).budget)
    }

    /// Blocks currently held for a receiver.
    pub fn buffered_blocks(&self, rx: RxId) -> usize {
        self.ring(rx).map_or(0, |r| lock(r).blocks.len())
    }

    /// Stream byte range held for a receiver.
    pub fn buffered_range(&self, rx: RxId) -> Option<std::ops::Range<u64>> {
        let r = self.ring(rx)?;
        let g = lock(r);
        let first = g.blocks.front()?;
        let last = g.blocks.back()?;
        Some(first.first_byte..last.first_byte + last.data.len() as u64)
    }

    /// Copies of the buffered blocks of a receiver (cheap: shared).
    pub fn snapshot(&self, rx: RxId) -> Vec<Arc<RawBlock>> {
        self.ring(rx)
            .map_or_else(Vec::new, |r| lock(r).blocks.iter().cloned().collect())
    }

    /// True between the first `drain_into` and `stop_recording`.
    pub fn is_recording(&self) -> bool {
        *lock(&self.recording)
    }

    /// Writes everything not yet written to `writer`: on the first call
    /// after creation or [`stop_recording`], the whole history (and the
    /// buffered events that fall inside it); afterwards only new blocks
    /// and events. Call it periodically from the recorder thread.
    ///
    /// [`stop_recording`]: RollingBuffer::stop_recording
    pub fn drain_into(&self, writer: &mut RecordingWriter) -> Result<DrainStats> {
        let mut stats = DrainStats::default();
        let first = {
            let mut rec = lock(&self.recording);
            let first = !*rec;
            *rec = true;
            first
        };
        let mut start_bytes: Vec<(RxId, u64)> = Vec::new();
        let mut oldest_ns = u64::MAX;
        for r in &self.rings {
            let (rx, pending) = {
                let mut g = lock(r);
                if first {
                    g.written_until = None;
                    g.evicted_unwritten = 0;
                }
                let w = g.written_until;
                let pending: Vec<Arc<RawBlock>> = g
                    .blocks
                    .iter()
                    .filter(|b| w.is_none_or(|w| b.first_byte + b.data.len() as u64 > w))
                    .cloned()
                    .collect();
                if let Some(last) = pending.last() {
                    g.written_until = Some(last.first_byte + last.data.len() as u64);
                } else if g.written_until.is_none() {
                    g.written_until = Some(0);
                }
                stats.evicted_unwritten += std::mem::take(&mut g.evicted_unwritten);
                (g.rx, pending)
            };
            if first {
                if let Some(b) = pending.first() {
                    start_bytes.push((rx, b.first_byte));
                    oldest_ns = oldest_ns.min(b.host_ns);
                }
            }
            for b in &pending {
                stats.bytes += writer.write_block(b)?;
                stats.blocks += 1;
            }
        }
        // Events: on the first drain everything inside the history; later
        // everything new. Events stay buffered only until written.
        let evs: Vec<Event> = lock(&self.events).drain(..).collect();
        for ev in evs {
            if first {
                match (ev.rx, ev.byte) {
                    (Some(rx), Some(b)) => match start_bytes.iter().find(|s| s.0 == rx) {
                        Some(&(_, s)) if b >= s => {}
                        _ => continue,
                    },
                    _ => {
                        if ev.host_ns < oldest_ns && oldest_ns != u64::MAX {
                            continue;
                        }
                    }
                }
            }
            if writer.log_stream_event(&ev)? {
                stats.events += 1;
            }
        }
        Ok(stats)
    }

    /// Ends the current recording; the next `drain_into` starts a new one
    /// with the full history again.
    pub fn stop_recording(&self) {
        *lock(&self.recording) = false;
        for r in &self.rings {
            lock(r).written_until = None;
        }
    }
}
