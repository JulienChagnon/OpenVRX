// SPDX-License-Identifier: GPL-3.0-or-later
//! NTSC and PAL colour decoding: composite `Field` to component `YuvField`.
//!
//! Per line:
//!
//! 1. Burst measurement against a fixed line-locked oscillator, then the
//!    burst tracker (`burst`): Kalman phase PLL from line to line (exact
//!    half cycle per line for NTSC, 283.7516 cycles with the grid slip for
//!    PAL), PAL V-switch identification, ACC and a graceful colour killer.
//! 2. Luma/chroma separation, one of
//!    - `Notch`: 1D chroma bandpass, luma = composite - chroma;
//!    - `Comb2D`: adaptive 3-line comb on bandpassed lines (NTSC lines
//!      n-1/n+1, PAL n-2/n+2 where chroma is inverted). Each neighbour is
//!      weighted by how well it cancels the current line's chroma; where
//!      neither matches the decoder falls back to 1D; damaged neighbour
//!      samples are never used;
//!    - `Comb3D`: the same line one frame back (NTSC) or two frames back
//!      (PAL), where chroma is exactly inverted; luma and chroma motion
//!      detection falls back to the 2D comb where anything moved;
//!    - PAL `DelayLine`: 1D separation with U/V averaging of the current
//!      and previous line (PAL-D);
//!    - PAL `Transform2D` / `Transform3D`: frequency-domain separation
//!      (`transform`), past fields only.
//! 3. Chroma demodulation against the burst-locked reference, lowpass,
//!    V-switch, ACC, killer, then user controls (brightness, contrast,
//!    saturation, hue, chroma delay, mild luma sharpening).
//!
//! The decoder is incremental: `advance` decodes every line whose inputs
//! (lookahead lines included) are final, so a streaming caller gets
//! exactly the same output as a whole-field call.

pub mod burst;
pub mod transform;

use crate::util::{RowsMut, box5_into, design_bandpass, design_lowpass, dilate_u8, scratch, scratch_any, smoothstep};
use burst::{measure_burst, BurstMeasure, BurstTracker, LineColor, TrackerConfig};
use openvrx_core::quality::damage;
use openvrx_core::standard::StandardParams;
use openvrx_core::{Field, FieldParity, LineFlags, VideoStandard, YuvField};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::ops::Range;
use std::sync::Arc;
use transform::{TransformPal, BLOCK_H, BLOCK_W, DEPTH, HOP_X, HOP_Y};

/// NTSC luma/chroma separation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NtscSeparation {
    Notch,
    Comb2D,
    Comb3D,
}

/// PAL luma/chroma separation.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PalSeparation {
    Notch,
    DelayLine,
    Comb2D,
    Comb3D,
    Transform2D,
    Transform3D,
}

/// Colour decoder settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ColorConfig {
    pub ntsc: NtscSeparation,
    pub pal: PalSeparation,
    /// PAL: average U/V of the current and previous line after
    /// demodulation (always on for `DelayLine`). Cancels hue errors.
    pub pal_line_average: bool,
    /// Added to Y (0..1 scale).
    pub brightness: f32,
    /// Multiplies Y and chroma.
    pub contrast: f32,
    /// Multiplies chroma.
    pub saturation: f32,
    /// Chroma rotation in degrees.
    pub hue_deg: f32,
    /// Delays chroma relative to luma by this many grid samples (negative
    /// advances it), to align a transmitter's chroma/luma timing.
    pub chroma_delay: f32,
    /// Luma sharpening 0 (off) .. 1 (strong). Mild values (0.2-0.4) only.
    pub sharpness: f32,
    /// Automatic colour control from the burst amplitude.
    pub acc: bool,
    /// Automatic colour killer (fades colour with weak/noisy burst).
    pub killer: bool,
    /// Decode as monochrome regardless of burst (full-band luma).
    pub force_mono: bool,
    /// Chroma demodulation lowpass bandwidth (Hz).
    pub chroma_bandwidth_hz: f32,
    /// Transform PAL bin similarity threshold (0..1).
    pub transform_threshold: f32,
    /// Expected line-to-line subcarrier phase jitter after the TBC (deg).
    pub line_jitter_deg: f32,
}

impl Default for ColorConfig {
    fn default() -> Self {
        ColorConfig {
            ntsc: NtscSeparation::Comb2D,
            pal: PalSeparation::Comb2D,
            pal_line_average: true,
            brightness: 0.0,
            contrast: 1.0,
            saturation: 1.0,
            hue_deg: 0.0,
            chroma_delay: 0.0,
            sharpness: 0.0,
            acc: true,
            killer: true,
            force_mono: false,
            chroma_bandwidth_hz: 1.3e6,
            transform_threshold: 0.45,
            line_jitter_deg: 4.0,
        }
    }
}

/// Internal method selection.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Method {
    Notch,
    Comb2D,
    Comb3D,
    Transform2D,
    Transform3D,
}

/// Extra U/V samples demodulated on each side of the active line (for the
/// chroma delay and the sharpening kernel).
const UV_MARGIN: usize = 8;

/// Stand-in for "no neighbour" in the comb's similarity measures.
const BIG: f32 = 1e9;

/// Immutable decoding kernel: filters, tables, geometry.
struct Kernel {
    p: &'static StandardParams,
    method: Method,
    line_average: bool,
    mono: bool,
    span: Range<usize>,
    active: Range<usize>,
    bp: Vec<f32>,
    lp: Vec<f32>,
    osc_c: Vec<f32>,
    osc_s: Vec<f32>,
    /// Lines between a line and its inverted-chroma neighbour (comb).
    comb_dist: usize,
    /// Fields back to the inverted-chroma line of the 3D comb.
    frame_back: u64,
    /// Noise gain of the bandpass (RMS out / RMS white noise in).
    bp_noise_gain: f32,
    /// Damage dilation radius (samples).
    dilate: usize,
    transform: Option<TransformPal>,
    // Controls.
    brightness: f32,
    contrast: f32,
    sat_cos: f32,
    sat_sin: f32,
    chroma_delay: f32,
    sharpness: f32,
}

#[derive(Clone)]
struct HistField {
    seq: u64,
    parity: FieldParity,
    field: Arc<Field>,
    bp: Arc<Vec<f32>>,
}

/// Work state of the field being decoded.
struct FieldWork {
    seq: u64,
    parity: FieldParity,
    lines: usize,
    first: usize,
    active_lines: usize,
    bp: Vec<f32>,
    bp_done: usize,
    lc: Vec<LineColor>,
    lc_done: usize,
    /// Phase A outputs, index i = field line - (first - 1).
    y_raw: Vec<f32>,
    u_raw: Vec<f32>,
    v_raw: Vec<f32>,
    dmg_raw: Vec<u8>,
    a_done: usize,
    /// Active lines finished (phase B).
    b_done: usize,
    /// Transform chroma accumulator over decode lines (L + 1) x span.
    trans: Vec<f32>,
    trans_rows_done: usize,
    trans_rows: usize,
    /// 3D history available for this field (same-parity fields, newest
    /// first), for Comb3D (one entry) or Transform3D (three entries).
    hist3d: Vec<HistField>,
}

/// Incremental NTSC/PAL colour decoder.
pub struct ColorDecoder {
    k: Kernel,
    cfg: ColorConfig,
    tracker: BurstTracker,
    burst_core: Range<usize>,
    history: VecDeque<HistField>,
    work: Option<FieldWork>,
    /// Buffers of the previous field's work, reused for the next field.
    spare: Option<FieldWork>,
    last_key: Option<(u64, FieldParity)>,
}

impl ColorDecoder {
    pub fn new(standard: VideoStandard, cfg: &ColorConfig) -> ColorDecoder {
        let p = standard.params();
        let k = Kernel::new(p, cfg);
        let b = p.burst_range();
        // Burst core: skip the envelope ramps, keep whole cycles.
        let mut core = (b.start + 4)..(b.end - 4);
        let len = core.len() / 4 * 4;
        core.end = core.start + len;
        let tracker = BurstTracker::new(
            p,
            core.len(),
            TrackerConfig {
                acc: cfg.acc,
                killer: cfg.killer && !cfg.force_mono,
                line_jitter_deg: cfg.line_jitter_deg as f64,
            },
        );
        ColorDecoder {
            k,
            cfg: cfg.clone(),
            tracker,
            burst_core: core,
            history: VecDeque::new(),
            work: None,
            spare: None,
            last_key: None,
        }
    }

    pub fn config(&self) -> &ColorConfig {
        &self.cfg
    }

    pub fn standard(&self) -> VideoStandard {
        self.k.p.standard
    }

    /// Short name of the separation in use.
    pub fn method_name(&self) -> &'static str {
        match (self.k.method, self.k.p.standard, self.k.line_average) {
            (Method::Notch, VideoStandard::Pal, true) => "pal-d",
            (Method::Notch, _, _) => "notch",
            (Method::Comb2D, _, _) => "comb2d",
            (Method::Comb3D, _, _) => "comb3d",
            (Method::Transform2D, _, _) => "transform2d",
            (Method::Transform3D, _, _) => "transform3d",
        }
    }

    /// Composite lines below a line that must be final before the line
    /// can be decoded (worst case).
    pub fn lookahead_lines(&self) -> usize {
        match self.k.method {
            Method::Notch => 0,
            Method::Comb2D | Method::Comb3D => self.k.comb_dist,
            Method::Transform2D | Method::Transform3D => BLOCK_H,
        }
    }

    /// Changes the picture controls without resetting state.
    pub fn set_controls(&mut self, cfg: &ColorConfig) {
        let same_method = cfg.ntsc == self.cfg.ntsc
            && cfg.pal == self.cfg.pal
            && cfg.pal_line_average == self.cfg.pal_line_average
            && cfg.chroma_bandwidth_hz == self.cfg.chroma_bandwidth_hz
            && cfg.transform_threshold == self.cfg.transform_threshold
            && cfg.force_mono == self.cfg.force_mono;
        if !same_method || cfg.acc != self.cfg.acc || cfg.killer != self.cfg.killer {
            *self = ColorDecoder::new(self.k.p.standard, cfg);
            return;
        }
        self.k.set_controls(cfg);
        self.cfg = cfg.clone();
    }

    /// Burst tracker state for the OSD: (burst amplitude IRE, colour level
    /// 0..1, phase noise deg, locked).
    pub fn burst_status(&self) -> (f64, f64, f64, bool) {
        (
            self.tracker.burst_amplitude(),
            self.tracker.color_level(),
            self.tracker.phase_noise_deg(),
            self.tracker.is_locked(),
        )
    }

    pub fn reset(&mut self) {
        self.tracker.reset();
        self.history.clear();
        self.work = None;
        self.last_key = None;
    }

    /// Decodes a whole field (offline / field mode).
    pub fn decode_field(&mut self, field: Arc<Field>) -> YuvField {
        let mut out = YuvField::new(field.standard, field.seq, field.parity);
        self.advance(&field, field.lines, true, &mut out, true);
        self.finish_field(field, &mut out, true);
        out
    }

    fn start_field(&mut self, field: &Field) {
        let p = self.k.p;
        let continuous = match self.last_key {
            Some((s, par)) => field.seq == s + 1 && field.parity == par.other(),
            None => false,
        };
        if !continuous {
            self.tracker.unlock();
        }
        let lines = field.lines;
        let first = p.first_active_field_line(field.parity);
        let active_lines = p.active_lines_per_field as usize;
        let s = self.k.span.len();
        let w = p.active_width();
        let wu = w + 2 * UV_MARGIN;
        let n_dec = active_lines + 1;
        // 3D history: same parity, consecutive frames.
        let mut hist3d = Vec::new();
        let find = |back: u64| {
            self.history
                .iter()
                .find(|h| field.seq >= back && h.seq == field.seq - back && h.parity == field.parity)
                .cloned()
        };
        match self.k.method {
            Method::Comb3D => {
                // The inverted-chroma frame, plus (optionally) the
                // same-phase frame before it for chroma-band motion.
                if let Some(h) = find(self.k.frame_back) {
                    hist3d.push(h);
                    if let Some(h2) = find(2 * self.k.frame_back) {
                        hist3d.push(h2);
                    }
                }
            }
            Method::Transform3D => {
                for t in 1..DEPTH as u64 {
                    match find(2 * t) {
                        Some(h) => hist3d.push(h),
                        None => {
                            hist3d.clear();
                            break;
                        }
                    }
                }
            }
            _ => {}
        }
        let (trans_rows, trans_len) = match self.k.method {
            Method::Transform2D | Method::Transform3D => (n_dec.div_ceil(HOP_Y) + 1, n_dec * s),
            _ => (0, 0),
        };
        // Reuse the previous field's buffers. Every buffer except the
        // transform accumulator is written before it is read, so only that
        // one is cleared.
        let mut wk = self.spare.take().unwrap_or_else(|| FieldWork {
            seq: 0,
            parity: field.parity,
            lines: 0,
            first: 0,
            active_lines: 0,
            bp: Vec::new(),
            bp_done: 0,
            lc: Vec::new(),
            lc_done: 0,
            y_raw: Vec::new(),
            u_raw: Vec::new(),
            v_raw: Vec::new(),
            dmg_raw: Vec::new(),
            a_done: 0,
            b_done: 0,
            trans: Vec::new(),
            trans_rows_done: 0,
            trans_rows: 0,
            hist3d: Vec::new(),
        });
        wk.seq = field.seq;
        wk.parity = field.parity;
        wk.lines = lines;
        wk.first = first;
        wk.active_lines = active_lines;
        wk.bp.resize(lines * s, 0.0);
        wk.bp_done = 0;
        wk.lc.resize(lines, LineColor::default());
        wk.lc_done = 0;
        wk.y_raw.resize(n_dec * w, 0.0);
        wk.u_raw.resize(n_dec * wu, 0.0);
        wk.v_raw.resize(n_dec * wu, 0.0);
        wk.dmg_raw.resize(n_dec * w, 0);
        wk.a_done = 0;
        wk.b_done = 0;
        wk.trans.clear();
        wk.trans.resize(trans_len, 0.0);
        wk.trans_rows_done = 0;
        wk.trans_rows = trans_rows;
        wk.hist3d = hist3d;
        self.work = Some(wk);
    }

    /// Decodes every line that can be finished given that field lines
    /// `0..lines_final` are final (`complete` = the whole field is final).
    /// Writes into `out` and returns the newly finished active lines.
    pub fn advance(
        &mut self,
        field: &Field,
        lines_final: usize,
        complete: bool,
        out: &mut YuvField,
        parallel: bool,
    ) -> Range<usize> {
        let restart = match &self.work {
            Some(w) => w.seq != field.seq || w.parity != field.parity,
            None => true,
        };
        if restart {
            self.start_field(field);
        }
        let k = &self.k;
        let work = self.work.as_mut().expect("work");
        let lines = work.lines;
        let limit = if complete { lines } else { lines_final.min(lines) };
        let s = k.span.len();

        // 1. Bandpass cache.
        if limit > work.bp_done {
            let rows = &mut work.bp[work.bp_done * s..limit * s];
            let base = work.bp_done;
            let f = |(i, row): (usize, &mut [f32])| {
                crate::util::fir_range(field.line(base + i), &k.bp, k.span.clone(), row);
            };
            if parallel {
                rows.par_chunks_mut(s).enumerate().for_each(f);
            } else {
                rows.chunks_mut(s).enumerate().for_each(f);
            }
            work.bp_done = limit;
        }

        // 2. Burst tracker, strictly in line order.
        while work.lc_done < limit {
            let n = work.lc_done;
            let q = field.quality[n];
            let burst = if q.flags.contains(LineFlags::VBI) {
                None
            } else {
                let m: BurstMeasure = measure_burst(
                    field.line(n),
                    field.damage_line(n),
                    self.burst_core.clone(),
                    &k.osc_c,
                    &k.osc_s,
                );
                Some(m)
            };
            let mut lc = self.tracker.step(burst, q.noise_ire);
            if k.mono {
                lc.gain = 0.0;
                lc.sep = 0.0;
            }
            work.lc[n] = lc;
            work.lc_done += 1;
        }

        // 3. Transform block rows.
        if let Some(tp) = &k.transform {
            let y00 = work.first as isize - 1 - HOP_Y as isize;
            while work.trans_rows_done < work.trans_rows {
                let r = work.trans_rows_done;
                let y0 = y00 + (r * HOP_Y) as isize;
                let need = (y0 + BLOCK_H as isize - 1).clamp(0, lines as isize - 1) as usize;
                if !(complete || need < lines_final) {
                    break;
                }
                k.transform_row(tp, field, work, y0, parallel);
                work.trans_rows_done += 1;
            }
        }

        // 4. Phase A: separation, luma, raw demodulation.
        let n_dec = work.active_lines + 1;
        let mut a_new = work.a_done;
        while a_new < n_dec {
            let n = work.first - 1 + a_new;
            let ready = if complete {
                true
            } else {
                let look = match k.method {
                    Method::Notch => 0,
                    Method::Comb2D | Method::Comb3D => k.comb_dist,
                    Method::Transform2D | Method::Transform3D => 0,
                };
                let lines_ok = (n + look).min(lines - 1) < lines_final;
                let trans_ok = match k.method {
                    Method::Transform2D | Method::Transform3D => {
                        // Decode index i is covered by block rows i / 8 and i / 8 + 1.
                        let r_max = (a_new / HOP_Y + 1).min(work.trans_rows - 1);
                        work.trans_rows_done > r_max
                    }
                    _ => true,
                };
                lines_ok && trans_ok
            };
            if !ready {
                break;
            }
            a_new += 1;
        }
        if a_new > work.a_done {
            let w = k.p.active_width();
            let wu = w + 2 * UV_MARGIN;
            let a0 = work.a_done;
            let FieldWork {
                y_raw,
                u_raw,
                v_raw,
                dmg_raw,
                bp,
                lc,
                trans,
                hist3d,
                first,
                lines,
                ..
            } = work;
            let ctx = LineCtx {
                field,
                bp,
                lc,
                trans,
                hist3d,
                first: *first,
                lines: *lines,
            };
            let ys = &mut y_raw[a0 * w..a_new * w];
            let us = &mut u_raw[a0 * wu..a_new * wu];
            let vs = &mut v_raw[a0 * wu..a_new * wu];
            let ds = &mut dmg_raw[a0 * w..a_new * w];
            let job = |(i, (((y, u), v), d)): (usize, RowsMut)| {
                k.phase_a(&ctx, a0 + i, y, u, v, d);
            };
            if parallel {
                ys.par_chunks_mut(w)
                    .zip(us.par_chunks_mut(wu))
                    .zip(vs.par_chunks_mut(wu))
                    .zip(ds.par_chunks_mut(w))
                    .enumerate()
                    .for_each(job);
            } else {
                ys.chunks_mut(w)
                    .zip(us.chunks_mut(wu))
                    .zip(vs.chunks_mut(wu))
                    .zip(ds.chunks_mut(w))
                    .enumerate()
                    .for_each(job);
            }
            work.a_done = a_new;
        }

        // 5. Phase B: active line j needs phase A of decode lines j and j+1.
        let b_new = work.a_done.saturating_sub(1).min(work.active_lines);
        let b0 = work.b_done;
        if b_new > b0 {
            let w = k.p.active_width();
            let first = work.first;
            let wk: &FieldWork = work;
            let job = |(j, (((y, u), v), d)): (usize, RowsMut)| {
                k.phase_b(wk, b0 + j, y, u, v, d);
            };
            let ys = &mut out.y[b0 * w..b_new * w];
            let us = &mut out.u[b0 * w..b_new * w];
            let vs = &mut out.v[b0 * w..b_new * w];
            let ds = &mut out.damage[b0 * w..b_new * w];
            if parallel {
                ys.par_chunks_mut(w)
                    .zip(us.par_chunks_mut(w))
                    .zip(vs.par_chunks_mut(w))
                    .zip(ds.par_chunks_mut(w))
                    .enumerate()
                    .for_each(job);
            } else {
                ys.chunks_mut(w)
                    .zip(us.chunks_mut(w))
                    .zip(vs.chunks_mut(w))
                    .zip(ds.chunks_mut(w))
                    .enumerate()
                    .for_each(job);
            }
            for j in b0..b_new {
                out.quality[j] = field.quality[first + j];
                out.source[j] = field.source[first + j];
            }
            if out.first_ns == 0 {
                out.first_ns = field.first_ns;
            }
            out.last_ns = out.last_ns.max(field.last_ns);
            work.b_done = b_new;
        }
        b0..b_new
    }

