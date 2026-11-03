// SPDX-License-Identifier: GPL-3.0-or-later
//! Real-time streaming generator for the live app's demo mode: yields
//! `RawBlock`s like the USB capture would, paced to real time (or any
//! speed), generated on a background thread.

use crate::engine::Engine;
use crate::{SimConfig, SimError};
use openvrx_core::{clock, RawBlock, RxId, SampleFormat};
use openvrx_format::{Event, Sidecar};
use std::collections::VecDeque;
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// Streaming options.
#[derive(Clone, Debug)]
pub struct SimSourceOptions {
    /// Playback speed (1.0 real time); None: as fast as generated.
    pub speed: Option<f64>,
    /// Bytes per block (whole words), like one USB transfer.
    pub block_bytes: usize,
    /// Restart the scenario at the end (stream positions keep growing).
    pub looping: bool,
    /// Chunks generated ahead.
    pub queue: usize,
}

impl Default for SimSourceOptions {
    fn default() -> Self {
        SimSourceOptions {
            speed: Some(1.0),
            block_bytes: 16384,
            looping: true,
            queue: 3,
        }
    }
}

enum Msg {
    Chunk(crate::Chunk),
    Error(SimError),
}

struct Lane {
    pending: VecDeque<u8>,
    stream_pos: u64,
    seq: u64,
    byte_rate: f64,
    word_bytes: usize,
}

/// Iterator of `RawBlock`s from the simulator.
pub struct SimSource {
    rx: mpsc::Receiver<Msg>,
    lanes: Vec<Lane>,
    events: Vec<Event>,
    opts: SimSourceOptions,
    wall_start: Option<Instant>,
    sidecar: Sidecar,
    done: bool,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl SimSource {
    /// Starts generating in the background.
    pub fn start(cfg: &SimConfig, opts: SimSourceOptions) -> Result<SimSource, SimError> {
        if opts.block_bytes == 0 {
            return Err(SimError::Config("block_bytes must be > 0".into()));
        }
        let engine = Engine::new(cfg)?;
        let sidecar = engine.sidecar();
        let lanes = engine
            .receivers()
            .iter()
            .map(|r| Lane {
                pending: VecDeque::new(),
                stream_pos: 0,
                seq: 0,
                byte_rate: r.byte_rate(),
                word_bytes: r.word_bytes() as usize,
            })
            .collect();
        let (tx, rx) = mpsc::sync_channel(opts.queue.max(1));
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop2 = stop.clone();
        let cfg2 = cfg.clone();
        let looping = opts.looping;
        let thread = std::thread::Builder::new()
            .name("openvrx-sim".into())
            .spawn(move || {
                let mut engine = engine;
                loop {
                    if stop2.load(std::sync::atomic::Ordering::Relaxed) {
                        return;
                    }
                    match engine.next_chunk() {
                        Ok(Some(c)) => {
                            if tx.send(Msg::Chunk(c)).is_err() {
                                return;
                            }
                        }
                        Ok(None) => {
                            if !looping {
                                return;
                            }
                            match Engine::new(&cfg2) {
                                Ok(e) => engine = e,
                                Err(e) => {
                                    let _ = tx.send(Msg::Error(e));
                                    return;
                                }
                            }
                        }
                        Err(e) => {
                            let _ = tx.send(Msg::Error(e));
                            return;
                        }
                    }
                }
            })
            .map_err(|e| SimError::Config(format!("cannot start generator thread: {e}")))?;
        Ok(SimSource {
            rx,
            lanes,
            events: Vec::new(),
            opts,
            wall_start: None,
            sidecar,
            done: false,
            stop,
            thread: Some(thread),
        })
    }

    /// Sidecar describing the streams (for a recorder or the UI).
    pub fn sidecar(&self) -> &Sidecar {
        &self.sidecar
    }

    pub fn sample_format(&self, rx: RxId) -> Option<&SampleFormat> {
        self.sidecar.receiver(rx).map(|r| &r.sample_format)
    }

    /// Events generated so far (byte offsets are positions in each
    /// receiver's delivered stream of the current loop).
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    fn fill(&mut self) -> Result<bool, SimError> {
        match self.rx.recv() {
            Ok(Msg::Chunk(c)) => {
                for (lane, bytes) in self.lanes.iter_mut().zip(c.rx_bytes) {
                    lane.pending.extend(bytes);
                }
                self.events.extend(c.events);
                Ok(true)
            }
            Ok(Msg::Error(e)) => Err(e),
            Err(_) => Ok(false),
        }
    }

    fn next_block(&mut self) -> Result<Option<RawBlock>, SimError> {
        loop {
            if self.done {
                return Ok(None);
            }
            // The lane with the earliest stream time that has a full
            // block ready (or any data once generation ended).
            let bb = self.opts.block_bytes;
            let pick = self
                .lanes
                .iter()
                .enumerate()
                .filter(|(_, l)| l.pending.len() >= bb.max(l.word_bytes))
                .min_by(|a, b| {
                    let ta = a.1.stream_pos as f64 / a.1.byte_rate;
                    let tb = b.1.stream_pos as f64 / b.1.byte_rate;
                    ta.total_cmp(&tb)
                })
                .map(|(i, _)| i);
            let Some(i) = pick else {
                if !self.fill()? {
                    // Generation ended: flush what is left.
                    if let Some(i) = self.lanes.iter().position(|l| !l.pending.is_empty()) {
                        return Ok(Some(self.emit(i)));
                    }
                    self.done = true;
                    return Ok(None);
                }
                continue;
            };
            return Ok(Some(self.emit(i)));
        }
    }

    fn emit(&mut self, i: usize) -> RawBlock {
        let lane = &mut self.lanes[i];
        let n = self.opts.block_bytes.min(lane.pending.len());
        let n = (n - n % lane.word_bytes).max(lane.word_bytes.min(lane.pending.len()));
        let data: Vec<u8> = lane.pending.drain(..n).collect();
        let first = lane.stream_pos;
        lane.stream_pos += n as u64;
        let t_end = lane.stream_pos as f64 / lane.byte_rate;
        let seq = lane.seq;
        lane.seq += 1;
        let wall = *self.wall_start.get_or_insert_with(Instant::now);
        if let Some(speed) = self.opts.speed {
            let target = wall + Duration::from_secs_f64(t_end / speed);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
        }
        RawBlock {
            rx: i as RxId,
            seq,
            first_byte: first,
            host_ns: clock::now_ns(),
            data,
        }
    }
}

impl Iterator for SimSource {
    type Item = Result<RawBlock, SimError>;
    fn next(&mut self) -> Option<Self::Item> {
        self.next_block().transpose()
    }
}

impl Drop for SimSource {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        // Unblock a sender waiting on a full queue.
        while self.rx.try_recv().is_ok() {}
        if let Some(t) = self.thread.take() {
            // Drain until the thread notices the flag and exits.
            loop {
                if t.is_finished() {
                    break;
                }
                match self.rx.recv_timeout(Duration::from_millis(5)) {
                    Ok(_) | Err(mpsc::RecvTimeoutError::Timeout) => {}
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                }
            }
            let _ = t.join();
        }
    }
}
