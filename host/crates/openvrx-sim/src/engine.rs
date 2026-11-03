// SPDX-License-Identifier: GPL-3.0-or-later
//! Chunked generation with carried state.
//!
//! The engine walks transmitter time in chunks of 2^21 bus samples (about
//! 26 ms). For each chunk it renders the composite lines it needs (in
//! parallel), runs the FM modulator (parallel work units, prefix-summed
//! phase, phase carried from the previous chunk), then gives each receiver
//! the bus samples whose transmitter position falls in the chunk, split
//! into independent work units that run in parallel across receivers.
//! Every per-sample quantity is a pure function of the bus index (noise
//! streams are keyed by index, fading is a function of time, IIR filters
//! warm up over 256 samples before each unit, far longer than their
//! impulse responses), so the output does not depend on thread count.

use crate::config::{Action, SimConfig};
use crate::rx::{Receiver, WARM};
use crate::tx::{Transmitter, TxBlock, BUS_HZ};
use crate::SimError;
use openvrx_core::{RxId, VideoStandard};
use openvrx_format::{Event, EventKind, ReceiverInfo, RecordingSource, Sidecar, SimInfo, TruthInfo};
use rayon::prelude::*;
use serde_json::{json, Value};

const _: () = assert!(WARM < HISTORY);

/// Kept bytes, kept (start, len) ranges and gap events (file byte, lost).
type LossCut = (Vec<u8>, Vec<(usize, usize)>, Vec<(u64, u64)>);

/// Transmitter samples per chunk.
const CHUNK: u64 = 1 << 21;
/// History kept before a chunk (warm-up, multipath delay, interpolation).
const HISTORY: u64 = 1024;
/// Bus samples per parallel work unit.
const UNIT: u64 = 1 << 15;
/// USB packet size used for loss events.
const PACKET: u64 = 512;

/// Output of one chunk.
#[derive(Debug, Default)]
pub struct Chunk {
    /// Bytes per receiver as delivered (after USB loss).
    pub rx_bytes: Vec<Vec<u8>>,
    /// Truth samples for rx0's delivered words.
    pub truth: Vec<f32>,
    /// Events, by host time.
    pub events: Vec<Event>,
    /// rx0 stream time reached (s).
    pub t_end: f64,
}

struct LossState {
    rate: f64,
    max_packets: u32,
    h: u64,
    next_byte: Option<u64>,
    /// Loss intervals (stream start, length), oldest first.
    history: Vec<(u64, u64)>,
    stream_pos: u64,
    file_pos: u64,
}

impl LossState {
    fn draw(&mut self) -> f64 {
        self.h = crate::dsp::splitmix(self.h);
        crate::dsp::unit_from(self.h)
    }

    fn schedule_after(&mut self, byte: u64, byte_rate: f64) {
        if self.rate <= 0.0 {
            self.next_byte = None;
            return;
        }
        let gap_s = -(1.0 - self.draw()).ln() / self.rate;
        let b = byte + (gap_s * byte_rate) as u64;
        self.next_byte = Some(b.div_ceil(PACKET) * PACKET);
    }

    /// File offset of a stream byte (inside a loss: where data resumes).
    fn file_of(&self, stream: u64) -> u64 {
        let mut lost = 0;
        for &(s, l) in &self.history {
            if s >= stream {
                break;
            }
            lost += l.min(stream - s);
        }
        stream - lost
    }

    /// Cuts losses out of `data` (stream bytes from `stream_pos`). Returns
    /// kept bytes, kept (start, len) ranges relative to `data`, and gap
    /// events as (file byte, lost bytes).
    fn apply(&mut self, data: Vec<u8>, byte_rate: f64) -> LossCut {
        let s0 = self.stream_pos;
        let s1 = s0 + data.len() as u64;
        let mut kept = Vec::new();
        let mut gaps = Vec::new();
        let mut pos = s0;
        while pos < s1 {
            // Inside an earlier loss?
            if let Some(&(ls, ll)) = self.history.last() {
                if pos >= ls && pos < ls + ll {
                    pos = (ls + ll).min(s1);
                    continue;
                }
            }
            match self.next_byte {
                Some(nb) if nb < s1 => {
                    let nb = nb.max(pos);
                    if nb > pos {
                        kept.push(((pos - s0) as usize, (nb - pos) as usize));
                        pos = nb;
                    }
                    let n = 1 + (self.draw() * self.max_packets.max(1) as f64) as u64;
                    let n = n.min(self.max_packets.max(1) as u64);
                    let len = n * PACKET;
                    let file_at = self.file_of(nb);
                    self.history.push((nb, len));
                    gaps.push((file_at, len));
                    self.schedule_after(nb + len, byte_rate);
                }
                _ => {
                    kept.push(((pos - s0) as usize, (s1 - pos) as usize));
                    pos = s1;
                }
            }
        }
        let out = if kept.len() == 1 && kept[0] == (0, data.len()) {
            data
        } else {
            let mut v = Vec::with_capacity(data.len());
            for &(a, n) in &kept {
                v.extend_from_slice(&data[a..a + n]);
            }
            v
        };
        self.stream_pos = s1;
        self.file_pos += out.len() as u64;
        (out, kept, gaps)
    }
}

/// The generator.
pub struct Engine {
    cfg: SimConfig,
    tx: Transmitter,
    rxs: Vec<Receiver>,
    loss: Vec<LossState>,
    k_next: u64,
    prev: Option<TxBlock>,
    markers: Vec<(f64, String)>,
    marker_next: usize,
    effects: Vec<Value>,
    start_host_ns: Vec<u64>,
    truth_delay: f64,
}

impl Engine {
    pub fn new(cfg: &SimConfig) -> Result<Engine, SimError> {
        cfg.validate()?;
        let tx = Transmitter::new(cfg)?;
        let p = cfg.standard.params();
        let lpf = p.lines_per_frame as u64;
        let start_line = match cfg.cvbs.start_line {
            Some(l) => (l.max(1) as u64 - 1) % lpf,
            None => crate::dsp::mix_seed(&[cfg.seed, 0x57A7]) % lpf,
        };
        // One whole frame of lead-in keeps every index positive.
        let frac = crate::dsp::unit_from(crate::dsp::mix_seed(&[cfg.seed, 0xF2AC]));
        let start_tx = ((lpf + start_line) as f64 * tx.line_period_tx()).floor() + frac;
        let rxs: Vec<Receiver> = (0..cfg.receivers.len())
            .map(|k| Receiver::new(cfg, k, start_tx))
            .collect();
        let loss = rxs
            .iter()
            .map(|r| {
                let mut l = LossState {
                    rate: r.cfg.usb_loss.rate_per_s,
                    max_packets: r.cfg.usb_loss.max_packets,
                    h: crate::dsp::mix_seed(&[cfg.seed, 0x05B, r.idx as u64]),
                    next_byte: None,
                    history: Vec::new(),
                    stream_pos: 0,
                    file_pos: 0,
                };
                l.schedule_after(0, r.byte_rate());
                l
            })
            .collect();
        let mut markers: Vec<(f64, String)> = cfg
            .events
            .iter()
            .filter_map(|e| match &e.action {
                Action::Marker { text } => Some((e.t_s, text.clone())),
                _ => None,
            })
            .collect();
        markers.sort_by(|a, b| a.0.total_cmp(&b.0));
        let start_host_ns = rxs
            .iter()
            .map(|r| (r.realised.start_delay_us * 1e3) as u64)
            .collect();
        let rx0 = &rxs[0];
        let truth_delay = rx0.filter_delay_bus(tx.blank_freq_hz()) / (rx0.cfg.divider as f64 / 3.0);
        let k_next = rxs.iter().map(|r| r.p0).fold(f64::MAX, f64::min).floor() as u64;
        Ok(Engine {
            cfg: cfg.clone(),
            tx,
            rxs,
            loss,
            k_next,
            prev: None,
            markers,
            marker_next: 0,
            effects: Vec::new(),
            start_host_ns,
            truth_delay,
        })
    }

    pub fn config(&self) -> &SimConfig {
        &self.cfg
    }

    pub fn receivers(&self) -> &[Receiver] {
        &self.rxs
    }

    /// Channel group delay in rx0 input samples.
    pub fn truth_delay(&self) -> f64 {
        self.truth_delay
    }

    pub fn finished(&self) -> bool {
        self.rxs.iter().all(|r| r.m_next >= r.m_total)
    }

    /// Effects with their exact positions (for the sidecar).
    pub fn effects(&self) -> &[Value] {
        &self.effects
    }

    /// Sidecar for a recording of this run (byte counts filled by the
    /// writer).
    pub fn sidecar(&self) -> Sidecar {
        let receivers = self
            .rxs
            .iter()
            .map(|r| {
                let mut info = ReceiverInfo::new(r.idx, r.format.clone());
                let (mhz, off) = r.initial_tuning();
                info.channel_mhz = mhz;
                info.offset_khz = off;
                info.gain = r.realised.initial_gain;
                info.p11 = r.cfg.filter == crate::config::ChannelFilter::P11;
                info.board = r.cfg.board.clone();
                info.c5_fw = "sim".into();
                info.fx2_fw = "sim".into();
                info.fx2_serial = format!("SIM{}", r.idx);
                info.start_host_ns = self.start_host_ns[r.idx as usize];
                info.antenna = r.cfg.antenna_name.clone();
                info
            })
            .collect();
        let mut s = Sidecar::new(RecordingSource::Sim, receivers);
        s.standard_hint = Some(self.cfg.standard);
        s.notes = self.cfg.notes.clone();
        let mut settings = serde_json::Map::new();
        settings.insert("generator".into(), json!(format!("openvrx-sim {}", env!("CARGO_PKG_VERSION"))));
        settings.insert("seed".into(), json!(self.cfg.seed));
        settings.insert("standard".into(), json!(self.cfg.standard));
        settings.insert("cn_db".into(), json!(self.cfg.channel.cn_db));
        settings.insert("deviation".into(), json!(self.cfg.vtx.deviation));
        settings.insert("fading".into(), json!(self.cfg.channel.fading));
        settings.insert("picture".into(), json!(self.cfg.picture));
        settings.insert("config".into(), serde_json::to_value(&self.cfg).expect("config serializes"));
        settings.insert(
            "realised".into(),
            serde_json::to_value(self.rxs.iter().map(|r| &r.realised).collect::<Vec<_>>())
                .expect("realised serializes"),
        );
        settings.insert("effects".into(), Value::Array(Vec::new()));
        s.sim = Some(SimInfo {
            truth: self.cfg.truth.then(|| TruthInfo {
                file: String::new(),
                rx: 0,
                unit: "ire".into(),
                delay_samples: self.truth_delay,
            }),
            settings,
        });
        s
    }

    /// Generates the next chunk; None when every receiver is done.
    pub fn next_chunk(&mut self) -> Result<Option<Chunk>, SimError> {
        if self.finished() {
            return Ok(None);
        }
        let k0 = self.k_next;
        let k1 = k0 + CHUNK;
        let a = k0 - HISTORY;
        let b = k1 + 64;
        let phase_a = match &self.prev {
            Some(prev) if a >= prev.a && a < prev.end() => {
                let ph = prev.phase[(a - prev.a) as usize];
                ph - ph.floor()
            }
            _ => 0.0,
        };
        let block = self.tx.block(a, b, phase_a)?;

        // Per receiver: gain loop, owned range, clock edges.
        struct Plan {
            m_lo: u64,
            m_hi: u64,
            edges: Vec<f64>,
            first_edge: u64,
        }
        let mut plans = Vec::with_capacity(self.rxs.len());
        for rx in &mut self.rxs {
            let m_lo = rx.m_next;
            if m_lo >= rx.m_total {
                plans.push(Plan { m_lo, m_hi: m_lo, edges: Vec::new(), first_edge: rx.edge_count });
                continue;
            }
            rx.run_agc(m_lo, &self.tx);
            let m_hi = rx.m_of(k1 as f64).min(rx.m_total).max(m_lo);
            let (edges, first_edge) = rx.gen_edges(m_hi);
            plans.push(Plan { m_lo, m_hi, edges, first_edge });
        }

        // Parallel work units over all receivers.
        let mut jobs = Vec::new();
        for (ri, plan) in plans.iter().enumerate() {
            let mut u = plan.m_lo;
            while u < plan.m_hi {
                let v = (u + UNIT).min(plan.m_hi);
                let e0 = plan.edges.partition_point(|&x| ((x / 3.0) as u64) < u);
                let e1 = plan.edges.partition_point(|&x| ((x / 3.0) as u64) < v);
                jobs.push((ri, u, v, e0, e1));
                u = v;
            }
        }
        let want_truth = self.cfg.truth;
        let rxs = &self.rxs;
        let tx = &self.tx;
        let blk = &block;
        let results: Vec<(Vec<u32>, Vec<f32>)> = jobs
            .par_iter()
            .map(|&(ri, u, v, e0, e1)| {
                let rx = &rxs[ri];
                let plan = &plans[ri];
                let adc = rx.front_end(blk, u, v);
                let edges = &plan.edges[e0..e1];
                let words = rx.words(&adc, u, edges, plan.first_edge + e0 as u64);
                let truth = if ri == 0 && want_truth {
                    edges
                        .iter()
                        .map(|&x| {
                            let m = crate::dsp::ffloor(x / 3.0);
                            tx.truth_at(blk, rx.p_of(m))
                        })
                        .collect()
                } else {
                    Vec::new()
                };
                (words, truth)
            })
            .collect();

        let mut chunk = Chunk {
            rx_bytes: vec![Vec::new(); self.rxs.len()],
            ..Default::default()
        };
        let mut words_per_rx: Vec<Vec<u32>> = vec![Vec::new(); self.rxs.len()];
        let mut truth_words = Vec::new();
        for (&(ri, ..), (w, t)) in jobs.iter().zip(results) {
            words_per_rx[ri].extend(w);
            if ri == 0 {
                truth_words.extend(t);
            }
        }

        for (ri, words) in words_per_rx.into_iter().enumerate() {
            let rx = &self.rxs[ri];
            let wb = rx.word_bytes() as usize;
            let mut bytes = Vec::with_capacity(words.len() * wb);
            for w in &words {
                if wb == 2 {
                    bytes.extend_from_slice(&(*w as u16).to_le_bytes());
                } else {
                    bytes.push(*w as u8);
                }
            }
            let byte_rate = rx.byte_rate();
            let host0 = self.start_host_ns[ri];
            let loss = &mut self.loss[ri];
            let (out, kept, gaps) = loss.apply(bytes, byte_rate);
            for (file_at, lost) in gaps {
                let t = file_at as f64 / byte_rate;
                chunk.events.push(Event::new(
                    Some(ri as RxId),
                    host0 + (t * 1e9) as u64,
                    Some(file_at),
                    EventKind::UsbGap { lost_bytes: lost, reason: "sim_usb_overflow".into() },
                ));
            }
            if ri == 0 && want_truth {
                for &(s, n) in &kept {
                    chunk.truth.extend_from_slice(&truth_words[s / wb..(s + n) / wb]);
                }
            }
            chunk.rx_bytes[ri] = out;

            // Effects that took place in this chunk.
            let plan = &plans[ri];
            for eff in rx.effects.iter().filter(|e| e.m >= plan.m_lo && e.m < plan.m_hi) {
                let k = plan.edges.partition_point(|&x| ((x / 3.0) as u64) < eff.m);
                let word = plan.first_edge + k as u64;
                let stream = word * wb as u64;
                let file = loss.file_of(stream);
                let latency = (rx.cfg.command_latency_ms * 1e-3 * byte_rate) as u64 / wb as u64 * wb as u64;
                let cmd = file.saturating_sub(latency);
                let t_eff = eff.m as f64 / BUS_HZ;
                let host = host0 + ((t_eff - rx.cfg.command_latency_ms * 1e-3).max(0.0) * 1e9) as u64;
                let kind = match &eff.action {
                    Action::Gain { index } => EventKind::Gain {
                        index: (*index).clamp(rx.cfg.gain_min, rx.cfg.gain_max),
                    },
                    Action::Tune { mhz, offset_khz } => EventKind::Tune { mhz: *mhz, offset_khz: *offset_khz },
                    Action::Clk { divider } => EventKind::Clk { divider: *divider },
                    Action::Slip { count } => EventKind::Slip { count: *count },
                    Action::Marker { .. } => continue,
                };
                self.effects.push(json!({
                    "rx": ri,
                    "kind": kind.name(),
                    "bus_sample": eff.m,
                    "effect_byte": file,
                    "effect_sample": file / wb as u64,
                    "command_byte": cmd,
                    "from_agc": eff.from_agc,
                    "action": eff.action,
                }));
                chunk.events.push(Event::new(Some(ri as RxId), host, Some(cmd), kind));
            }
        }

        // Session markers by rx0 time.
        let t_end_s = plans[0].m_hi as f64 / BUS_HZ;
        while self.marker_next < self.markers.len() && self.markers[self.marker_next].0 < t_end_s {
            let (t, text) = self.markers[self.marker_next].clone();
            chunk.events.push(Event::new(None, (t.max(0.0) * 1e9) as u64, None, EventKind::Marker { text }));
            self.marker_next += 1;
        }
        chunk.events.sort_by_key(|e| e.host_ns);
        chunk.t_end = t_end_s;

        for (rx, plan) in self.rxs.iter_mut().zip(&plans) {
            rx.m_next = plan.m_hi;
        }
        self.k_next = k1;
        self.prev = Some(block);
        Ok(Some(chunk))
    }

    pub fn standard(&self) -> VideoStandard {
        self.cfg.standard
    }
}
