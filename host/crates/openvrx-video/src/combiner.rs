// SPDX-License-Identifier: GPL-3.0-or-later
//! Diversity combining of line records from several receivers.
//!
//! Alignment: every receiver has its own field counter, but they all see
//! the same transmitter, so the raster is shared. Each receiver's counter
//! is mapped onto a common field counter by field parity and host arrival
//! time (a field record's estimated start time); the mapping is verified
//! on every line and re-locked when it stops matching. Lines are then
//! grouped by (common field, field line).
//!
//! Waiting: a group is emitted as soon as every live receiver delivered
//! its line, or when the newest line seen is `max_wait_lines` lines ahead.
//! The combiner never stalls on a dead receiver (a receiver is live while
//! it delivered a line within `stale_lines` lines). Output is in raster
//! order; lines arriving after their group was emitted are dropped.
//!
//! Sub-sample timing: after their TBCs the receivers may still disagree by
//! a fraction of a sample. Per line the offset of each receiver against
//! the reference line is estimated from the colour burst phase (precise,
//! +-2 samples), the rising edge of horizontal sync and, for monochrome
//! signals, a gradient cross-correlation of the active line; estimates are
//! smoothed per receiver and removed with a Lanczos fractional delay
//! before any mixing.
//!
//! Modes (`CombineMode`):
//! - `Select` (D1): best receiver per line by smoothed weight, with
//!   hysteresis.
//! - `Blend` (D2): weighted average with weights = smoothed inverse noise
//!   variance (`LineQuality::weight`), i.e. maximal-ratio combining after
//!   detection.
//! - `Repair` (D3, default): D2 with per-sample weights scaled down where
//!   a receiver's damage mask flags damage, with short crossfades, so a
//!   click or dropout in one receiver is filled from the others.
//! - `SelectRepair`: D1, patching only damaged samples from the others.
//! - `Segment`: the line is split into segments; each receiver's noise is
//!   estimated per segment (high-pass energy minus the common content,
//!   floored by the line's porch noise) and the segment weights are
//!   interpolated across the line, plus per-sample damage repair.
//! - `SegmentSelect`: as `Segment`, but each segment (about 6.5 us with
//!   the default 8 segments) takes only its best receiver, crossfading
//!   between segment centres; selection inside a line follows fades that
//!   are shorter than a line.

use crate::util::{dilate_u8, fractional_delay};
use openvrx_core::quality::damage;
use openvrx_core::{FieldParity, LineFlags, LineQuality, LineRecord, RxId, VideoStandard, COMBINED};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CombineMode {
    Select,
    Blend,
    Repair,
    SelectRepair,
    Segment,
    SegmentSelect,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CombinerConfig {
    pub mode: CombineMode,
    /// Lines to wait for a missing receiver before emitting without it.
    pub max_wait_lines: usize,
    /// A receiver silent for this many lines no longer holds output back.
    pub stale_lines: usize,
    /// D1 switch margin in dB of weight.
    pub hysteresis_db: f32,
    /// Weight smoothing factor per line for rising weights (falling
    /// weights take effect at once).
    pub weight_smoothing: f32,
    /// Estimate and remove sub-sample timing offsets.
    pub align: bool,
    /// Smoothing factor of the timing offset per line.
    pub align_smoothing: f32,
    /// Crossfade length (samples) around per-sample repairs.
    pub crossfade: usize,
    /// Segments per active line (Segment mode).
    pub segments: usize,
}

impl Default for CombinerConfig {
    fn default() -> Self {
        CombinerConfig {
            mode: CombineMode::Repair,
            max_wait_lines: 8,
            stale_lines: 400,
            hysteresis_db: 1.0,
            weight_smoothing: 0.25,
            align: true,
            align_smoothing: 0.3,
            crossfade: 6,
            segments: 8,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct CombinerStats {
    pub lines_in: u64,
    pub lines_out: u64,
    pub late_lines: u64,
    /// Groups emitted before every live receiver delivered.
    pub timeouts: u64,
    pub relocks: u64,
    pub blended_lines: u64,
    pub repaired_samples: u64,
}

#[derive(Clone, Debug)]
struct RxState {
    offset: Option<i64>,
    last_line: i64,
    weight: f32,
    tau: f64,
    mismatches: u32,
    selected_lines: u64,
}

#[derive(Copy, Clone, Debug)]
struct FieldRec {
    parity: FieldParity,
    start_ns: f64,
}

struct Group {
    lines: Vec<LineRecord>,
}

pub struct Combiner {
    cfg: CombinerConfig,
    standard: VideoStandard,
    rx: HashMap<RxId, RxState>,
    fields: BTreeMap<u64, FieldRec>,
    groups: BTreeMap<(u64, u16), Group>,
    last_emitted: Option<(u64, u16)>,
    newest_line: i64,
    selected: Option<RxId>,
    stats: CombinerStats,
    osc_c: Vec<f32>,
    osc_s: Vec<f32>,
    line_ns: f64,
    field_ns: f64,
    half_frame: i64,
}

/// Damage to weight factor.
#[inline]
fn damage_gain(d: u8) -> f32 {
    if d < damage::NOISY {
        1.0
    } else if d < damage::REPAIRED {
        0.5
    } else if d < damage::CLICK {
        0.25
    } else if d < damage::SEVERE {
        0.02
    } else if d < damage::LOST {
        0.002
    } else {
        0.0
    }
}

impl Combiner {
    pub fn new(standard: VideoStandard, cfg: &CombinerConfig) -> Combiner {
        let p = standard.params();
        let f = p.fsc_per_sample();
        let w = p.samples_per_line;
        let osc_c = (0..w).map(|x| (2.0 * std::f64::consts::PI * f * x as f64).cos() as f32).collect();
        let osc_s = (0..w).map(|x| (2.0 * std::f64::consts::PI * f * x as f64).sin() as f32).collect();
        Combiner {
            cfg: cfg.clone(),
            standard,
            rx: HashMap::new(),
            fields: BTreeMap::new(),
            groups: BTreeMap::new(),
            last_emitted: None,
            newest_line: i64::MIN,
            selected: None,
            stats: CombinerStats::default(),
            osc_c,
            osc_s,
            line_ns: 1e9 / p.line_hz,
            field_ns: 1e9 / p.field_hz,
            half_frame: p.lines_per_frame as i64 / 2,
        }
    }

    pub fn config(&self) -> &CombinerConfig {
        &self.cfg
    }

    pub fn set_config(&mut self, cfg: &CombinerConfig) {
        self.cfg = cfg.clone();
    }

    pub fn stats(&self) -> CombinerStats {
        self.stats
    }

    /// Lines each receiver fed as the selected receiver (Select modes).
    pub fn selected_counts(&self) -> Vec<(RxId, u64)> {
        let mut v: Vec<(RxId, u64)> = self.rx.iter().map(|(&k, s)| (k, s.selected_lines)).collect();
        v.sort();
        v
    }

    /// Smoothed timing offset (samples) of a receiver against the line
    /// reference, as last applied.
    pub fn timing_offset(&self, rx: RxId) -> Option<f64> {
        self.rx.get(&rx).map(|s| s.tau)
    }

    pub fn reset(&mut self) {
        let cfg = self.cfg.clone();
        *self = Combiner::new(self.standard, &cfg);
    }

    #[inline]
    fn line_index(&self, seq: u64, fl: u16) -> i64 {
        seq as i64 * self.half_frame + fl as i64
    }

    /// Maps a receiver's field counter to the common one.
    fn map_seq(&mut self, rec: &LineRecord) -> u64 {
        let start = rec.host_ns as f64 - rec.field_line as f64 * self.line_ns;
        let tol = 0.5 * self.field_ns;
        let any_offset = self.rx.values().any(|s| s.offset.is_some());
        let st = self.rx.entry(rec.rx).or_insert(RxState {
            offset: None,
            last_line: i64::MIN,
            weight: 0.0,
            tau: 0.0,
            mismatches: 0,
            selected_lines: 0,
        });
        // Verify an existing mapping.
        if let Some(off) = st.offset {
            let c = rec.field_seq as i64 + off;
            if c >= 0 {
                match self.fields.get(&(c as u64)) {
                    Some(fr) => {
                        if fr.parity == rec.parity && (fr.start_ns - start).abs() < tol {
                            st.mismatches = 0;
                            return c as u64;
                        }
                        st.mismatches += 1;
                    }
                    None => {
                        // A new field: plausible if parity and timing
                        // continue the sequence.
                        let ok = match self.fields.range(..c as u64).next_back() {
                            Some((&pc, pf)) => {
                                let expect = pf.start_ns + (c as u64 - pc) as f64 * self.field_ns;
                                (expect - start).abs() < tol
                                    && ((c as u64 - pc) & 1 == 0) == (pf.parity == rec.parity)
                            }
                            None => true,
                        };
                        if ok {
                            st.mismatches = 0;
                            self.fields.insert(c as u64, FieldRec { parity: rec.parity, start_ns: start });
                            return c as u64;
                        }
                        st.mismatches += 1;
                    }
                }
                if st.mismatches < 3 {
                    // Tolerate a few odd lines (glitches) with the old map.
                    return c.max(0) as u64;
                }
            }
            self.stats.relocks += 1;
            st.offset = None;
            st.mismatches = 0;
        }
        // Establish a mapping: find a known field with this parity and a
        // matching start time. A receiver alone has nothing to align with:
        // a field already emitted would only drop its lines as late (host
        // times bunch up when the USB thread reaps a backlog of transfers
        // at once, DSP-5), so it continues the sequence instead.
        let alone = !self.rx.iter().any(|(&k, s)| {
            k != rec.rx && s.offset.is_some() && s.last_line >= self.newest_line - self.cfg.stale_lines as i64
        });
        let emitted = self.last_emitted.map(|(f, _)| f);
        let found = self
            .fields
            .iter()
            .rev()
            .take(16)
            .find(|(&c, fr)| {
                fr.parity == rec.parity
                    && (fr.start_ns - start).abs() < tol
                    && !(alone && emitted.is_some_and(|e| c <= e))
            })
            .map(|(&c, _)| c);
        let c = match found {
            Some(c) => c,
            None => {
                if !any_offset || self.fields.is_empty() {
                    // First receiver (or nothing known): it defines the
                    // common counter.
                    rec.field_seq
                } else {
                    // Continue the sequence from the newest known field.
                    let (&nc, nf) = self.fields.iter().next_back().expect("fields");
                    let k = ((start - nf.start_ns) / self.field_ns).round().max(1.0) as u64;
                    let mut c = nc + k;
                    let same = nf.parity == rec.parity;
                    if ((c - nc) & 1 == 0) != same {
                        c += 1;
                    }
                    c
                }
            }
        };
        let st = self.rx.get_mut(&rec.rx).expect("rx");
        st.offset = Some(c as i64 - rec.field_seq as i64);
        self.fields.entry(c).or_insert(FieldRec { parity: rec.parity, start_ns: start });
        while self.fields.len() > 64 {
            let k = *self.fields.keys().next().expect("non-empty");
            self.fields.remove(&k);
        }
        c
    }

    /// Pushes one receiver's line; returns combined lines ready, in order.
    pub fn push(&mut self, rec: LineRecord) -> Vec<LineRecord> {
        self.stats.lines_in += 1;
        let p = self.standard.params();
        if rec.standard != self.standard || rec.samples.len() != p.samples_per_line || rec.damage.len() != p.samples_per_line {
            return Vec::new();
        }
        let c = self.map_seq(&rec);
        let key = (c, rec.field_line);
        // Liveness counts even for a line that arrives too late, so a
        // receiver that lags is waited for from the next line on.
        let li = self.line_index(c, rec.field_line);
        self.newest_line = self.newest_line.max(li);
        if let Some(st) = self.rx.get_mut(&rec.rx) {
            st.last_line = st.last_line.max(li);
        }
        if let Some(le) = self.last_emitted {
            if key <= le {
                self.stats.late_lines += 1;
                return Vec::new();
            }
        }
        let mut rec = rec;
        rec.field_seq = c;
        let g = self.groups.entry(key).or_insert(Group { lines: Vec::new() });
        if let Some(slot) = g.lines.iter_mut().find(|l| l.rx == rec.rx) {
            *slot = rec;
        } else {
            g.lines.push(rec);
        }
        self.drain(false)
    }

    /// Emits every pending group regardless of missing receivers.
    pub fn flush(&mut self) -> Vec<LineRecord> {
        self.drain(true)
    }

    fn drain(&mut self, force: bool) -> Vec<LineRecord> {
        let mut out = Vec::new();
        let stale = self.cfg.stale_lines as i64;
        while let Some((&key, g)) = self.groups.iter().next() {
            let li = self.line_index(key.0, key.1);
            let live: Vec<RxId> = self
                .rx
                .iter()
                .filter(|(_, s)| s.offset.is_some() && s.last_line >= self.newest_line - stale)
                .map(|(&k, _)| k)
                .collect();
            let complete = live.iter().all(|r| g.lines.iter().any(|l| l.rx == *r));
            let timed_out = self.newest_line - li >= self.cfg.max_wait_lines as i64;
            if !(force || complete || timed_out) {
                break;
            }
            if !complete {
                self.stats.timeouts += 1;
            }
            let g = self.groups.remove(&key).expect("group");
            self.last_emitted = Some(key);
            let line = self.combine(g.lines);
            self.stats.lines_out += 1;
            out.push(line);
        }
        out
    }

    /// Updates smoothed weights and returns them in line order.
    fn weights(&mut self, lines: &[LineRecord]) -> Vec<f32> {
        let beta = self.cfg.weight_smoothing.clamp(0.0, 1.0);
        lines
            .iter()
            .map(|l| {
                let mut w = l.quality.weight;
                if !w.is_finite() || w < 0.0 || l.quality.flags.contains(LineFlags::NO_CARRIER) {
                    w = 0.0;
                }
                let st = self.rx.get_mut(&l.rx).expect("rx state");
                if w <= st.weight || st.weight <= 0.0 {
                    st.weight = w;
                } else {
                    st.weight += beta * (w - st.weight);
                }
                st.weight
            })
            .collect()
    }

    fn combine(&mut self, mut lines: Vec<LineRecord>) -> LineRecord {
        if lines.len() == 1 {
            let l = lines.pop().expect("one line");
            let _ = self.weights(std::slice::from_ref(&l));
            if let Some(st) = self.rx.get_mut(&l.rx) {
                st.selected_lines += 1;
            }
            self.selected = Some(l.rx);
            return l;
        }
        lines.sort_by_key(|l| l.rx);
        let w = self.weights(&lines);
        let n = lines.len();
        let width = lines[0].samples.len();
        // Reference: highest weight.
        // Reference: highest weight (lowest receiver id on ties, so the
        // reference does not hop between equal receivers).
        let refi = (1..n).fold(0, |best, i| if w[i] > w[best] { i } else { best });

        // D1 selection with hysteresis (used by the select modes, and to
        // name the dominant receiver).
        let margin = 10f32.powf(self.cfg.hysteresis_db / 10.0);
        let sel_idx = match self.selected.and_then(|s| lines.iter().position(|l| l.rx == s)) {
            Some(cur) if w[refi] <= w[cur] * margin || w[cur] >= w[refi] => cur,
            _ => refi,
        };

        let select_only = self.cfg.mode == CombineMode::Select;
        if select_only {
            self.selected = Some(lines[sel_idx].rx);
            if let Some(st) = self.rx.get_mut(&lines[sel_idx].rx) {
                st.selected_lines += 1;
            }
            return lines.swap_remove(sel_idx);
        }

        // Sub-sample alignment against the reference line.
        let mut aligned: Vec<Vec<f32>> = Vec::with_capacity(n);
        let mut dmg: Vec<Vec<u8>> = Vec::with_capacity(n);
        for i in 0..n {
            if i == refi || !self.cfg.align {
                aligned.push(lines[i].samples.clone());
                dmg.push(lines[i].damage.clone());
                continue;
            }
            let est = self.estimate_offset(&lines[refi], &lines[i]);
            let alpha = self.cfg.align_smoothing.clamp(0.0, 1.0) as f64;
            let st = self.rx.get_mut(&lines[i].rx).expect("rx");
            if let Some(t) = est {
                st.tau += alpha * (t - st.tau);
            }
            let tau = st.tau;
            if tau.abs() > 0.02 {
                let mut buf = vec![0.0f32; width];
                fractional_delay(&lines[i].samples, -tau, &mut buf);
                aligned.push(buf);
                let sh = tau.round() as isize;
                let src = &lines[i].damage;
                let mut d = vec![0u8; width];
                for (x, o) in d.iter_mut().enumerate() {
                    let xs = (x as isize + sh).clamp(0, width as isize - 1) as usize;
                    *o = src[xs];
                }
                dmg.push(d);
            } else {
                aligned.push(lines[i].samples.clone());
                dmg.push(lines[i].damage.clone());
            }
        }

        // Per-sample weights.
        let r = (self.cfg.crossfade / 2).max(1);
        let gains: Vec<Vec<f32>> = dmg.iter().map(|d| damage_profile(d, r)).collect();
        let mut wts: Vec<Vec<f32>> = vec![vec![0.0; width]; n];
        let mut repaired = false;
        match self.cfg.mode {
            CombineMode::Blend => {
                for i in 0..n {
                    wts[i].fill(w[i]);
                }
            }
            CombineMode::Repair => {
                for i in 0..n {
                    for x in 0..width {
                        wts[i][x] = w[i] * gains[i][x];
                    }
                }
                repaired = true;
            }
            CombineMode::SelectRepair => {
                let s = sel_idx;
                let mut others_w = vec![0.0f32; width];
                for i in 0..n {
                    if i == s {
                        continue;
                    }
                    for x in 0..width {
                        others_w[x] += w[i].max(1e-6) * gains[i][x];
                    }
                }
                for x in 0..width {
                    let patch = 1.0 - gains[s][x];
                    let can = others_w[x] > 0.0;
                    if patch > 0.0 && can {
                        for i in 0..n {
                            wts[i][x] = if i == s {
                                1.0 - patch
                            } else {
                                patch * w[i].max(1e-6) * gains[i][x] / others_w[x]
                            };
                        }
                    } else {
                        wts[s][x] = 1.0;
                    }
                }
                repaired = true;
            }
            CombineMode::Segment => {
                let seg = self.segment_weights(&aligned, &dmg, &lines, &w, false);
                for i in 0..n {
                    for x in 0..width {
                        wts[i][x] = seg[i][x] * gains[i][x];
                    }
                }
                repaired = true;
            }
            CombineMode::SegmentSelect => {
                // Selected share per sample, reduced where the selected
                // receiver is damaged; the remainder goes to the others in
                // proportion to their (damage-scaled) line weights.
                let seg = self.segment_weights(&aligned, &dmg, &lines, &w, true);
                for x in 0..width {
                    let kept: f32 = (0..n).map(|i| seg[i][x] * gains[i][x]).sum();
                    let rest = (1.0 - kept).max(0.0);
                    let others: f32 = (0..n).map(|i| w[i].max(1e-6) * gains[i][x]).sum();
                    for i in 0..n {
                        let extra = if others > 0.0 { rest * w[i].max(1e-6) * gains[i][x] / others } else { 0.0 };
                        wts[i][x] = seg[i][x] * gains[i][x] + extra;
                    }
                }
                repaired = true;
            }
            CombineMode::Select => unreachable!("handled above"),
        }

        // Mix.
        let mut out = vec![0.0f32; width];
        let mut dout = vec![0u8; width];
        let mut share_sq = vec![0.0f64; n];
        let mut share_sum = vec![0.0f64; n];
        let mut patched = 0u64;
        for x in 0..width {
            let tot: f32 = (0..n).map(|i| wts[i][x]).sum();
            if tot <= 0.0 {
                // Nothing usable: least damaged receiver.
                let i = (0..n).min_by_key(|&i| dmg[i][x]).expect("lines");
                out[x] = aligned[i][x];
                dout[x] = dmg[i][x];
                share_sum[i] += 1.0;
                share_sq[i] += 1.0;
                continue;
            }
            let mut v = 0.0f32;
            let mut d = 0.0f32;
            let mut dmin = u8::MAX;
            for i in 0..n {
                let s = wts[i][x] / tot;
                v += s * aligned[i][x];
                d += s * dmg[i][x] as f32;
                dmin = dmin.min(dmg[i][x]);
                share_sum[i] += s as f64;
                share_sq[i] += (s * s) as f64;
            }
            out[x] = v;
            let dd = (d.round() as u8).max(dmin);
            if repaired && dd < dmg[refi][x] && dmg[refi][x] >= damage::CLICK {
                patched += 1;
            }
            dout[x] = dd;
        }
        self.stats.repaired_samples += patched;

        // Combined quality.
        let wf = width as f64;
        let mut noise_var = 0.0f64;
        let mut contributors = 0;
        let mut dominant = refi;
        let mut dom_share = 0.0;
        for i in 0..n {
            let sh = share_sum[i] / wf;
            let sq = share_sq[i] / wf;
            let ni = lines[i].quality.noise_ire;
            let ni = if ni.is_finite() { ni as f64 } else { 50.0 };
            noise_var += sq * ni * ni;
            if sh > 0.05 {
                contributors += 1;
            }
            if sh > dom_share {
                dom_share = sh;
                dominant = i;
            }
        }
        let noise = noise_var.sqrt() as f32;
        let mut q = lines[dominant].quality;
        q.noise_ire = noise;
        q.weight = LineQuality::weight_from_noise(noise);
        q.cn_db = lines.iter().map(|l| l.quality.cn_db).fold(f32::NAN, f32::max);
        q.sync_score = lines.iter().map(|l| l.quality.sync_score).fold(0.0, f32::max);
        q.power_db = lines.iter().map(|l| l.quality.power_db).fold(f32::NEG_INFINITY, f32::max);
        let mut flags = LineFlags(0);
        for l in &lines {
            flags.insert(LineFlags(l.quality.flags.0 & (LineFlags::BURST_OK.0 | LineFlags::VBI.0)));
        }
        if lines.iter().all(|l| l.quality.flags.contains(LineFlags::NO_CARRIER)) {
            flags.insert(LineFlags::NO_CARRIER);
        }
        if contributors >= 2 {
            flags.insert(LineFlags::BLENDED);
            self.stats.blended_lines += 1;
        }
        if patched > 0 {
            flags.insert(LineFlags::REPAIRED);
        }
        q.flags = flags;
        // Count remaining click runs.
        let mut clicks = 0u16;
        let mut inside = false;
        for &d in &dout {
            let c = (damage::CLICK..damage::SEVERE).contains(&d);
            if c && !inside {
                clicks = clicks.saturating_add(1);
            }
            inside = c;
        }
        q.clicks = clicks;

        let rx_out = if contributors >= 2 { COMBINED } else { lines[dominant].rx };
        self.selected = Some(lines[sel_idx].rx);
        if let Some(st) = self.rx.get_mut(&lines[dominant].rx) {
            st.selected_lines += 1;
        }
        let base = &lines[refi];
        LineRecord {
            rx: rx_out,
            standard: base.standard,
            field_seq: base.field_seq,
            parity: base.parity,
            field_line: base.field_line,
            frame_line: base.frame_line,
            samples: out,
            damage: dout,
            quality: q,
            src_position: base.src_position,
            src_line_len: base.src_line_len,
            host_ns: lines.iter().map(|l| l.host_ns).max().unwrap_or(base.host_ns),
        }
    }

    /// Per-sample weights from per-segment noise estimates.
    #[allow(clippy::needless_range_loop)]
    fn segment_weights(&self, aligned: &[Vec<f32>], dmg: &[Vec<u8>], lines: &[LineRecord], w: &[f32], select: bool) -> Vec<Vec<f32>> {
        let p = self.standard.params();
        let a = p.active_range();
        let n = aligned.len();
        let width = aligned[0].len();
        let segs = self.cfg.segments.max(1);
        let seg_len = a.len() / segs;
        let mut sw = vec![vec![0.0f32; segs]; n];
        for s in 0..segs {
            let r = (a.start + s * seg_len)..(a.start + (s + 1) * seg_len);
            let mut v = vec![0.0f32; n];
            let mut floor = vec![0.0f32; n];
            for i in 0..n {
                let mut acc = 0.0f32;
                let mut cnt = 0usize;
                for x in r.start + 1..r.end {
                    if dmg[i][x] >= damage::CLICK || dmg[i][x - 1] >= damage::CLICK {
                        continue;
                    }
                    let d = aligned[i][x] - aligned[i][x - 1];
                    acc += d * d;
                    cnt += 1;
                }
                v[i] = if cnt > 8 { acc / (2.0 * cnt as f32) } else { f32::INFINITY };
                let ni = lines[i].quality.noise_ire;
                floor[i] = if ni.is_finite() { ni * ni } else { 2500.0 };
            }
            // Common content energy: what the cleanest receiver shows beyond
            // its own porch noise.
            let content = (0..n)
                .filter(|&i| v[i].is_finite())
                .map(|i| (v[i] - floor[i]).max(0.0))
                .fold(f32::INFINITY, f32::min);
            let content = if content.is_finite() { content } else { 0.0 };
            for i in 0..n {
                sw[i][s] = if v[i].is_finite() && w[i] > 0.0 {
                    let noise = (v[i] - content).max(floor[i]).max(0.0025);
                    1.0 / noise
                } else {
                    0.0
                };
            }
        }
        if select {
            // One-hot per segment (lowest receiver index on ties).
            for s in 0..segs {
                let best = (1..n).fold(0, |b, i| if sw[i][s] > sw[b][s] { i } else { b });
                for i in 0..n {
                    sw[i][s] = if i == best && sw[i][s] > 0.0 { 1.0 } else { 0.0 };
                }
            }
        }
        // Interpolate between segment centres; outside the active range
        // use the line weights.
        let mut out = vec![vec![0.0f32; width]; n];
        for i in 0..n {
            for (x, o) in out[i].iter_mut().enumerate() {
                if x < a.start || x >= a.start + segs * seg_len {
                    *o = w[i];
                    continue;
                }
                let pos = (x - a.start) as f32 / seg_len as f32 - 0.5;
                let s0 = pos.floor().clamp(0.0, (segs - 1) as f32) as usize;
                let s1 = (s0 + 1).min(segs - 1);
                let t = (pos - s0 as f32).clamp(0.0, 1.0);
                *o = sw[i][s0] * (1.0 - t) + sw[i][s1] * t;
            }
        }
        out
    }

    /// Timing offset (samples) of `line` relative to `refl`: positive when
    /// `line` is late.
    fn estimate_offset(&self, refl: &LineRecord, line: &LineRecord) -> Option<f64> {
        let p = self.standard.params();
        let f = p.fsc_per_sample();
        // Sync rising edge.
        let edge = |l: &LineRecord| -> Option<f64> {
            let hs = p.hsync_samples();
            let lvl = p.sync_ire / 2.0;
            let lo = hs.saturating_sub(8);
            let hi = (hs + 8).min(l.samples.len() - 1);
            for x in lo..hi {
                let (a, b) = (l.samples[x], l.samples[x + 1]);
                if a < lvl && b >= lvl && l.damage[x] < damage::CLICK && l.damage[x + 1] < damage::CLICK {
                    return Some(x as f64 + ((lvl - a) / (b - a)) as f64);
                }
            }
            None
        };
        let sync = match (edge(refl), edge(line)) {
            (Some(a), Some(b)) => Some(b - a),
            _ => None,
        };
        // Burst phase.
        let b = p.burst_range();
        let core = (b.start + 4)..(b.end - 4);
        let mb = |l: &LineRecord| crate::color::burst::measure_burst(&l.samples, &l.damage, core.clone(), &self.osc_c, &self.osc_s);
        let (br, bl) = (mb(refl), mb(line));
        let nominal = p.burst_ire as f64;
        let burst = if br.clean && bl.clean && br.amp() > 0.4 * nominal && bl.amp() > 0.4 * nominal {
            let d = crate::util::wrap_pi(br.arg() - bl.arg());
            Some(d / (2.0 * std::f64::consts::PI * f))
        } else {
            None
        };
        let content = || -> Option<f64> {
            let a = p.active_range();
            let g = |l: &LineRecord, x: usize| l.samples[x + 1] - l.samples[x - 1];
            let mut best = (f64::NEG_INFINITY, 0i64);
            let mut cs = [0.0f64; 7];
            let mut e_ref = 0.0f64;
            let mut e_l = 0.0f64;
            for x in a.start + 8..a.end - 8 {
                e_ref += (g(refl, x) as f64).powi(2);
                e_l += (g(line, x) as f64).powi(2);
            }
            if e_ref < 1e3 || e_l < 1e3 {
                return None;
            }
            for (k, c) in cs.iter_mut().enumerate() {
                let lag = k as i64 - 3;
                let mut acc = 0.0f64;
                for x in a.start + 8..a.end - 8 {
                    acc += g(refl, x) as f64 * g(line, (x as i64 + lag) as usize) as f64;
                }
                *c = acc / (e_ref * e_l).sqrt();
                if *c > best.0 {
                    best = (*c, lag);
                }
            }
            if best.0 < 0.5 || best.1.abs() == 3 {
                return None;
            }
            let i = (best.1 + 3) as usize;
            let (y0, y1, y2) = (cs[i - 1], cs[i], cs[i + 1]);
            let den = y0 - 2.0 * y1 + y2;
            let frac = if den.abs() > 1e-12 { 0.5 * (y0 - y2) / den } else { 0.0 };
            Some(best.1 as f64 + frac.clamp(-0.5, 0.5))
        };
        match (burst, sync) {
            (Some(b), Some(s)) if (b - s).abs() < 1.2 => Some(b),
            (Some(b), None) => Some(b),
            (_, Some(s)) => match content() {
                Some(c) if (c - s).abs() < 1.0 => Some(0.3 * s + 0.7 * c),
                _ => Some(s),
            },
            (None, None) => content(),
        }
    }
}

/// Damage gain per sample with crossfades: the gain is eroded by `r` and
/// then smoothed over `2r + 1`, so damaged samples get the full penalty
/// and the transition happens just outside them.
fn damage_profile(d: &[u8], r: usize) -> Vec<f32> {
    let n = d.len();
    if d.iter().all(|&v| v < damage::NOISY) {
        return vec![1.0; n];
    }
    let mut dil = vec![0u8; n];
    dilate_u8(d, r, &mut dil);
    let g: Vec<f32> = dil.iter().map(|&v| damage_gain(v)).collect();
    let mut out = vec![0.0f32; n];
    for x in 0..n {
        let lo = x.saturating_sub(r);
        let hi = (x + r + 1).min(n);
        let s: f32 = g[lo..hi].iter().sum();
        out[x] = (s / (hi - lo) as f32).min(damage_gain(d[x]));
    }
    out
}
