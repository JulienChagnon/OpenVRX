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

    /// Completes the field: decodes any remaining lines and keeps the
    /// field as history for the 3D methods. Returns the lines finished.
    pub fn finish_field(&mut self, field: Arc<Field>, out: &mut YuvField, parallel: bool) -> Range<usize> {
        let r = self.advance(&field, field.lines, true, out, parallel);
        out.first_ns = field.first_ns;
        out.last_ns = field.last_ns;
        if let Some(w) = self.work.take() {
            let keep = match self.k.method {
                Method::Comb3D => 2 * self.k.frame_back as usize,
                Method::Transform3D => 2 * (DEPTH - 1),
                _ => 0,
            };
            let mut w = w;
            if keep > 0 {
                self.history.push_front(HistField {
                    seq: w.seq,
                    parity: w.parity,
                    field: field.clone(),
                    bp: Arc::new(std::mem::take(&mut w.bp)),
                });
                self.history.truncate(keep);
            }
            w.hist3d.clear();
            self.spare = Some(w);
        }
        self.last_key = Some((field.seq, field.parity));
        r
    }
}

/// Final 2D comb mix (separate function with slice arguments, and every
/// load outside any branch, so that it vectorises).
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn comb2d_mix(c0: &[f32], a: &[f32], b: &[f32], vu: &[f32], vd: &[f32], su: &[f32], sd: &[f32], lev: &[f32], eps: f32, chroma: &mut [f32]) {
    let s = chroma.len();
    let (c0, a, b, vu, vd, su, sd, lev) = (&c0[..s], &a[..s], &b[..s], &vu[..s], &vd[..s], &su[..s], &sd[..s], &lev[..s]);
    for j in 0..s {
        let (vuj, vdj, suj, sdj, c, aj, bj) = (vu[j], vd[j], su[j], sd[j], c0[j], a[j], b[j]);
        // Missing neighbours carry a zero validity weight.
        let wu = vuj / (suj + eps);
        let wd = vdj / (sdj + eps);
        let wsum = wu + wd;
        let mix = (wu * (c - aj) + wd * (c - bj)) * 0.5 / wsum;
        let c2 = if wsum > 0.0 { mix } else { c };
        let best = if vuj > 0.0 { suj } else { BIG }.min(if vdj > 0.0 { sdj } else { BIG });
        let mism = best / (2.0 * lev[j] + eps);
        let k1 = smoothstep(0.6, 1.2, mism);
        chroma[j] = c2 + k1 * (c - c2);
    }
}

/// Read-only inputs of phase A.
struct LineCtx<'a> {
    field: &'a Field,
    bp: &'a [f32],
    lc: &'a [LineColor],
    trans: &'a [f32],
    hist3d: &'a [HistField],
    first: usize,
    lines: usize,
}

impl Kernel {
    fn new(p: &'static StandardParams, cfg: &ColorConfig) -> Kernel {
        let method = match p.standard {
            VideoStandard::Ntsc => match cfg.ntsc {
                NtscSeparation::Notch => Method::Notch,
                NtscSeparation::Comb2D => Method::Comb2D,
                NtscSeparation::Comb3D => Method::Comb3D,
            },
            VideoStandard::Pal => match cfg.pal {
                PalSeparation::Notch | PalSeparation::DelayLine => Method::Notch,
                PalSeparation::Comb2D => Method::Comb2D,
                PalSeparation::Comb3D => Method::Comb3D,
                PalSeparation::Transform2D => Method::Transform2D,
                PalSeparation::Transform3D => Method::Transform3D,
            },
        };
        let line_average = p.standard == VideoStandard::Pal
            && (cfg.pal == PalSeparation::DelayLine || cfg.pal_line_average);
        let rate = p.grid_rate_hz();
        let f = p.fsc_per_sample();
        let active = p.active_range();
        let pad = 20usize;
        let span = active.start.saturating_sub(pad)..(active.end + pad).min(p.samples_per_line);
        // Notch: chroma band about +-1.3 MHz; combs and transforms use a
        // wider band because the comb itself separates luma.
        let bp = match method {
            Method::Notch => design_bandpass(f, 1.45e6 / rate, 41),
            _ => design_bandpass(f, 2.0e6 / rate, 31),
        };
        let lp = design_lowpass(cfg.chroma_bandwidth_hz as f64 / rate, 31);
        let w = p.samples_per_line;
        let mut osc_c = vec![0.0f32; w];
        let mut osc_s = vec![0.0f32; w];
        for x in 0..w {
            let ph = 2.0 * std::f64::consts::PI * f * x as f64;
            osc_c[x] = ph.cos() as f32;
            osc_s[x] = ph.sin() as f32;
        }
        let bp_noise_gain = bp.iter().map(|v| v * v).sum::<f32>().sqrt();
        let transform = match method {
            Method::Transform2D | Method::Transform3D => Some(TransformPal::new(f, cfg.transform_threshold)),
            _ => None,
        };
        let mut k = Kernel {
            p,
            method,
            line_average,
            mono: cfg.force_mono,
            span,
            active,
            dilate: (bp.len() / 2 + lp.len() / 2) / 2,
            bp,
            lp,
            osc_c,
            osc_s,
            comb_dist: if p.standard == VideoStandard::Pal { 2 } else { 1 },
            frame_back: if p.standard == VideoStandard::Pal { 4 } else { 2 },
            bp_noise_gain,
            transform,
            brightness: 0.0,
            contrast: 1.0,
            sat_cos: 1.0,
            sat_sin: 0.0,
            chroma_delay: 0.0,
            sharpness: 0.0,
        };
        k.set_controls(cfg);
        k
    }

    fn set_controls(&mut self, cfg: &ColorConfig) {
        self.brightness = cfg.brightness;
        self.contrast = cfg.contrast;
        let h = cfg.hue_deg.to_radians();
        let g = cfg.saturation * cfg.contrast;
        self.sat_cos = g * h.cos();
        self.sat_sin = g * h.sin();
        self.chroma_delay = cfg.chroma_delay.clamp(-(UV_MARGIN as f32 - 2.0), UV_MARGIN as f32 - 2.0);
        self.sharpness = cfg.sharpness.clamp(0.0, 1.0);
    }

    /// Computes one row of transform blocks and overlap-adds it.
    fn transform_row(&self, tp: &TransformPal, field: &Field, work: &mut FieldWork, y0: isize, parallel: bool) {
        let s = self.span.len();
        let nx = s.div_ceil(HOP_X) + 1;
        let x00 = self.span.start as isize - HOP_X as isize;
        let w = self.p.samples_per_line as isize;
        let lines = field.lines as isize;
        let hist = &work.hist3d;
        let three_d = self.method == Method::Transform3D && hist.len() == DEPTH - 1;
        let block = |i: usize| -> [f32; BLOCK_W * BLOCK_H] {
            thread_local! {
                static SCRATCH: std::cell::RefCell<Option<transform::Scratch>> = const { std::cell::RefCell::new(None) };
            }
            let xo = x00 + (i * HOP_X) as isize;
            let mut out = [0.0f32; BLOCK_W * BLOCK_H];
            SCRATCH.with(|cell| {
                let mut opt = cell.borrow_mut();
                let sc = opt.get_or_insert_with(|| transform::Scratch::new(tp));
                if three_d {
                    let get = |t: usize, r: usize, c: usize| -> f32 {
                        let f: &Field = if t == 0 { field } else { &hist[t - 1].field };
                        let y = (y0 + r as isize).clamp(0, lines - 1) as usize;
                        let x = (xo + c as isize).clamp(0, w - 1) as usize;
                        f.video[y * f.width + x]
                    };
                    let motion = tp.block_3d(&get, sc, &mut out);
                    // Where the block moved, fall back toward 2D.
                    let k = smoothstep(0.02, 0.08, motion);
                    if k > 0.0 {
                        let get2 = |r: usize, c: usize| get(0, r, c);
                        let mut o2 = [0.0f32; BLOCK_W * BLOCK_H];
                        tp.block_2d(&get2, sc, &mut o2);
                        for (a, b) in out.iter_mut().zip(&o2) {
                            *a += k * (b - *a);
                        }
                    }
                } else {
                    let get = |r: usize, c: usize| -> f32 {
                        let y = (y0 + r as isize).clamp(0, lines - 1) as usize;
                        let x = (xo + c as isize).clamp(0, w - 1) as usize;
                        field.video[y * field.width + x]
                    };
                    tp.block_2d(&get, sc, &mut out);
                }
            });
            out
        };
        let blocks: Vec<[f32; BLOCK_W * BLOCK_H]> = if parallel {
            (0..nx).into_par_iter().map(block).collect()
        } else {
            (0..nx).map(block).collect()
        };
        // Overlap-add into decode lines (field line first-1 is index 0).
        let dec0 = work.first as isize - 1;
        let n_dec = (work.active_lines + 1) as isize;
        for (i, b) in blocks.iter().enumerate() {
            let xo = x00 + (i * HOP_X) as isize;
            for r in 0..BLOCK_H {
                let di = y0 + r as isize - dec0;
                if di < 0 || di >= n_dec {
                    continue;
                }
                let row = &mut work.trans[di as usize * s..(di as usize + 1) * s];
                for c in 0..BLOCK_W {
                    let sx = xo + c as isize - self.span.start as isize;
                    if sx >= 0 && (sx as usize) < s {
                        row[sx as usize] += b[r * BLOCK_W + c];
                    }
                }
            }
        }
    }

    /// Phase A for decode line `i` (field line first - 1 + i): chroma
    /// separation, luma, raw U/V demodulation and the damage footprint.
    fn phase_a(&self, ctx: &LineCtx, i: usize, y_out: &mut [f32], u_out: &mut [f32], v_out: &mut [f32], d_out: &mut [u8]) {
        #[cfg(target_arch = "x86_64")]
        {
            if crate::util::has_avx2() {
                // SAFETY: AVX2 and FMA are available (checked at run time).
                unsafe { self.phase_a_avx2(ctx, i, y_out, u_out, v_out, d_out) };
            return;
            }
        }
        self.phase_a_impl(ctx, i, y_out, u_out, v_out, d_out)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn phase_a_avx2(&self, ctx: &LineCtx, i: usize, y_out: &mut [f32], u_out: &mut [f32], v_out: &mut [f32], d_out: &mut [u8]) {
        self.phase_a_impl(ctx, i, y_out, u_out, v_out, d_out)
    }

    #[inline(always)]
    fn phase_a_impl(&self, ctx: &LineCtx, i: usize, y_out: &mut [f32], u_out: &mut [f32], v_out: &mut [f32], d_out: &mut [u8]) {
        let p = self.p;
        let n = ctx.first - 1 + i;
        let s = self.span.len();
        let x0 = self.span.start;
        let field = ctx.field;
        let line = field.line(n);
        let dmg = field.damage_line(n);
        let bp_n = &ctx.bp[n * s..(n + 1) * s];
        let lc = ctx.lc[n];
        let q = field.quality[n];
        let noise = if q.noise_ire.is_finite() { q.noise_ire.min(30.0) } else { 3.0 };
        let mut chroma = scratch_any(s);
        let mut dmg_extra: Option<Vec<u8>> = None;

        match self.method {
            Method::Notch => chroma.copy_from_slice(bp_n),
            Method::Comb2D | Method::Comb3D => {
                self.comb2d(ctx, n, noise, &mut chroma);
                if self.method == Method::Comb3D && !ctx.hist3d.is_empty() {
                    self.comb3d(ctx, n, noise, &mut chroma);
                }
            }
            Method::Transform2D | Method::Transform3D => {
                chroma.copy_from_slice(&ctx.trans[i * s..(i + 1) * s]);
                // The block transform spreads a damaged sample over its
                // vertical neighbours.
                let mut d = vec![0u8; s];
                for nn in [n.saturating_sub(1), (n + 1).min(ctx.lines - 1)] {
                    let dl = field.damage_line(nn);
                    for (o, &v) in d.iter_mut().zip(&dl[self.span.clone()]) {
                        if v >= damage::CLICK && *o < v {
                            *o = v;
                        }
                    }
                }
                dmg_extra = Some(d);
            }
        }

        // Luma over the active range.
        let a = self.active.clone();
        let black = p.black_ire;
        let inv_span = 1.0 / (p.white_ire - p.black_ire);
        let sep = lc.sep;
        let la = &line[a.clone()];
        let ca = &chroma[a.start - x0..a.end - x0];
        for ((o, &l), &c) in y_out.iter_mut().zip(la).zip(ca) {
            *o = (l - sep * c - black) * inv_span;
        }

        // Demodulation: mix to baseband, lowpass, rotate by the line phase.
        let mut mc = scratch_any(s);
        let mut ms = scratch_any(s);
        {
            let oc = &self.osc_c[x0..x0 + s];
            let os = &self.osc_s[x0..x0 + s];
            let (mc, ms, ch) = (&mut mc[..s], &mut ms[..s], &chroma[..s]);
            for j in 0..s {
                mc[j] = ch[j] * oc[j];
                ms[j] = ch[j] * os[j];
            }
        }
        let g = 2.0 * lc.gain * inv_span;
        let (cph, sph) = (lc.phase.cos(), lc.phase.sin());
        // Lowpass both mixer products at the U/V output positions (span
        // coordinates, clamped at the span edges).
        let ustart = a.start - UV_MARGIN - x0;
        let un = u_out.len();
        let mut zr = scratch_any(un);
        let mut zi = scratch_any(un);
        crate::util::fir_range(&mc, &self.lp, ustart..ustart + un, &mut zr);
        crate::util::fir_range(&ms, &self.lp, ustart..ustart + un, &mut zi);
        {
            let (zr, zi, v_out) = (&zr[..un], &zi[..un], &mut v_out[..un]);
            let vs = lc.vswitch;
            for k in 0..un {
                let zr = zr[k];
                let zi = -zi[k];
                // (V' - jU) = z * exp(-j phase)
                let wr = zr * cph + zi * sph;
                let wi = zi * cph - zr * sph;
                v_out[k] = g * wr * vs;
                u_out[k] = -g * wi;
            }
        }

        // Damage footprint of this line (fast path for clean lines).
        let none = |d: &[u8]| d.iter().fold(0u8, |a, &b| a | b) == 0;
        let clean = none(&dmg[self.span.clone()]) && dmg_extra.as_deref().map(none).unwrap_or(true);
        if clean {
            d_out.fill(0);
            return;
        }
        let mut dl = vec![0u8; p.samples_per_line];
        dilate_u8(dmg, self.dilate, &mut dl);
        if let Some(extra) = dmg_extra {
            let mut e = vec![0u8; s];
            dilate_u8(&extra, self.dilate, &mut e);
            for (j, &v) in e.iter().enumerate() {
                let x = x0 + j;
                if dl[x] < v {
                    dl[x] = v;
                }
            }
        }
        d_out.copy_from_slice(&dl[a]);
    }

    /// Adaptive 2D comb into `chroma` (span coordinates).
    #[inline(always)]
    fn comb2d(&self, ctx: &LineCtx, n: usize, noise: f32, chroma: &mut [f32]) {
        let s = self.span.len();
        let x0 = self.span.start;
        let d = self.comb_dist;
        let bp = ctx.bp;
        let c0 = &bp[n * s..(n + 1) * s];
        let up = if n >= d { Some(n - d) } else { None };
        let dn = if n + d < ctx.lines { Some(n + d) } else { None };
        let eps = 0.5 + 1.5 * noise * self.bp_noise_gain;
        // Similarity and validity rows: written in full by `sim`, or set to
        // "no neighbour" below.
        let mut su = scratch_any(s);
        let mut sd = scratch_any(s);
        let mut lev = scratch_any(s);
        let mut vu = scratch_any(s);
        let mut vd = scratch_any(s);
        // Neighbour similarity is judged on the chroma-free part of the
        // lines (composite minus its chroma band): a neighbour whose luma
        // matches the current line cancels luma in the comb.
        // All loops run over equal-length slices so that they vectorise.
        let span = x0..x0 + s;
        let cur = &ctx.field.line(n)[span.clone()];
        for (l, &c) in lev[..s].iter_mut().zip(c0) {
            *l = c.abs();
        }
        let sim = |nb: usize, sv: &mut [f32], vv: &mut [f32]| {
            let a = &bp[nb * s..(nb + 1) * s];
            let la = &ctx.field.line(nb)[span.clone()];
            let da = &ctx.field.damage_line(nb)[span.clone()];
            let (sv, vv) = (&mut sv[..s], &mut vv[..s]);
            for j in 0..s {
                sv[j] = ((cur[j] - c0[j]) - (la[j] - a[j])).abs();
                vv[j] = if da[j] >= damage::CLICK { 0.0 } else { 1.0 };
            }
        };
        match up {
            Some(u) => sim(u, &mut su, &mut vu),
            None => {
                su.fill(BIG);
                vu.fill(0.0);
            }
        }
        match dn {
            Some(dd) => sim(dd, &mut sd, &mut vd),
            None => {
                sd.fill(BIG);
                vd.fill(0.0);
            }
        }
        let (mut su_b, mut sd_b, mut lev_b) = (scratch_any(s), scratch_any(s), scratch_any(s));
        box5_into(&su, &mut su_b);
        box5_into(&sd, &mut sd_b);
        box5_into(&lev, &mut lev_b);
        let (su, sd, lev) = (su_b, sd_b, lev_b);
        let zero = scratch(s, 0.0);
        let a = &up.map(|u| &bp[u * s..(u + 1) * s]).unwrap_or(&zero)[..s];
        let b = &dn.map(|v| &bp[v * s..(v + 1) * s]).unwrap_or(&zero)[..s];
        comb2d_mix(c0, a, b, &vu, &vd, &su, &sd, &lev, eps, chroma);
    }

    /// 3D comb: blends in the frame comb where nothing moved.
    #[inline(always)]
    fn comb3d(&self, ctx: &LineCtx, n: usize, noise: f32, chroma: &mut [f32]) {
        let s = self.span.len();
        let x0 = self.span.start;
        let h = &ctx.hist3d[0];
        if n >= h.field.lines {
            return;
        }
        let c0 = &ctx.bp[n * s..(n + 1) * s];
        let pb = &h.bp[n * s..(n + 1) * s];
        let cur = ctx.field.line(n);
        let prev = h.field.line(n);
        let dprev = h.field.damage_line(n);
        // Luma motion: the composite minus its chroma band is identical in
        // both frames when nothing moved.
        let mut ld = scratch(s, 0.0);
        let mut cd = scratch(s, 0.0);
        for j in 0..s {
            let x = x0 + j;
            ld[j] = ((cur[x] - c0[j]) - (prev[x] - pb[j])).abs();
        }
        // Chroma-band motion: against the same-phase frame two frame
        // periods back when it exists, else (conservatively) the
        // inverted-chroma consistency, which also flags static luma in the
        // chroma band.
        match ctx.hist3d.get(1) {
            Some(h2) if n < h2.field.lines => {
                let b2 = &h2.bp[n * s..(n + 1) * s];
                for j in 0..s {
                    cd[j] = (c0[j] - b2[j]).abs();
                }
            }
            _ => {
                for j in 0..s {
                    cd[j] = (c0[j] + pb[j]).abs();
                }
            }
        }
        let mut ldf = scratch(s, 0.0);
        let mut cdf = scratch(s, 0.0);
        box5_into(&ld, &mut ldf);
        box5_into(&cd, &mut cdf);
        let (ld, cd) = (ldf, cdf);
        let thr_l = 1.5 + 2.5 * noise;
        let thr_c = 1.5 + 2.5 * noise * self.bp_noise_gain * 1.5;
        let mut mot = scratch(s, 0.0);
        for j in 0..s {
            mot[j] = (ld[j] / thr_l).max(cd[j] / thr_c);
        }
        // Dilate motion over +-4 samples.
        let mut md = scratch(s, 0.0);
        for j in 0..s {
            let lo = j.saturating_sub(4);
            let hi = (j + 5).min(s);
            md[j] = mot[lo..hi].iter().fold(0.0f32, |m, &v| m.max(v));
        }
        for j in 0..s {
            let valid = dprev[x0 + j] < damage::CLICK;
            if !valid {
                continue;
            }
            let k3 = 1.0 - smoothstep(0.7, 1.5, md[j]);
            let c3 = 0.5 * (c0[j] - pb[j]);
            chroma[j] = k3 * c3 + (1.0 - k3) * chroma[j];
        }
    }

    /// Phase B for active line j: PAL line averaging, controls, chroma
    /// delay, sharpening.
    fn phase_b(&self, work: &FieldWork, j: usize, y: &mut [f32], u: &mut [f32], v: &mut [f32], d: &mut [u8]) {
        #[cfg(target_arch = "x86_64")]
        {
            if crate::util::has_avx2() {
                // SAFETY: AVX2 and FMA are available (checked at run time).
                unsafe { self.phase_b_avx2(work, j, y, u, v, d) };
            return;
            }
        }
        self.phase_b_impl(work, j, y, u, v, d)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn phase_b_avx2(&self, work: &FieldWork, j: usize, y: &mut [f32], u: &mut [f32], v: &mut [f32], d: &mut [u8]) {
        self.phase_b_impl(work, j, y, u, v, d)
    }

    #[inline(always)]
    fn phase_b_impl(&self, work: &FieldWork, j: usize, y: &mut [f32], u: &mut [f32], v: &mut [f32], d: &mut [u8]) {
        let w = y.len();
        let wu = w + 2 * UV_MARGIN;
        let (u, v, d) = (&mut u[..w], &mut v[..w], &mut d[..w]);
        let i = j + 1; // decode index of this line
        let yr = &work.y_raw[i * w..(i + 1) * w];
        let ur = &work.u_raw[i * wu..(i + 1) * wu];
        let vr = &work.v_raw[i * wu..(i + 1) * wu];
        let dr = &work.dmg_raw[i * w..(i + 1) * w];
        let mut ub = scratch_any(wu);
        let mut vb = scratch_any(wu);
        if self.line_average {
            let up = &work.u_raw[(i - 1) * wu..i * wu];
            let vp = &work.v_raw[(i - 1) * wu..i * wu];
            let (ub, vb) = (&mut ub[..wu], &mut vb[..wu]);
            for k in 0..wu {
                ub[k] = 0.5 * (ur[k] + up[k]);
                vb[k] = 0.5 * (vr[k] + vp[k]);
            }
            let dp = &work.dmg_raw[(i - 1) * w..i * w];
            for k in 0..w {
                d[k] = dr[k].max(dp[k]);
            }
        } else {
            ub.copy_from_slice(ur);
            vb.copy_from_slice(vr);
            d.copy_from_slice(dr);
        }
        // Chroma delay (linear interpolation) and hue/saturation.
        let pos = UV_MARGIN as f32 - self.chroma_delay;
        let i0 = pos.floor();
        let t = pos - i0;
        let i0 = i0 as usize;
        let (ua, ub1) = (&ub[i0..i0 + w], &ub[i0 + 1..i0 + 1 + w]);
        let (va, vb1) = (&vb[i0..i0 + w], &vb[i0 + 1..i0 + 1 + w]);
        let (sc, ss) = (self.sat_cos, self.sat_sin);
        for k in 0..w {
            let uu = ua[k] + t * (ub1[k] - ua[k]);
            let vv = va[k] + t * (vb1[k] - va[k]);
            u[k] = uu * sc - vv * ss;
            v[k] = uu * ss + vv * sc;
        }
        // Luma: contrast, brightness, optional sharpening with coring.
        let c = self.contrast;
        let b = self.brightness;
        if self.sharpness > 0.0 {
            let amt = 0.8 * self.sharpness;
            for k in 0..w {
                let g = |o: isize| yr[(k as isize + o).clamp(0, w as isize - 1) as usize];
                let lp = (g(-2) + 2.0 * g(-1) + 2.0 * g(0) + 2.0 * g(1) + g(2)) * 0.125;
                let hp = g(0) - lp;
                let cored = if hp.abs() < 0.004 { 0.0 } else { hp - 0.004 * hp.signum() };
                y[k] = (g(0) + amt * cored) * c + b;
            }
        } else {
            for k in 0..w {
                y[k] = yr[k] * c + b;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encoder::{CompositeEncoder, EncoderConfig};
    use crate::testgen::{color_bars, field_info, image_to_field};

    #[test]
    fn bars_roughly_decode_all_methods() {
        for std in VideoStandard::ALL {
            let p = std.params();
            let img = color_bars(p.active_width(), p.frame_height(), 0.75);
            let enc = CompositeEncoder::new(std, EncoderConfig::default());
            let cfg = ColorConfig {
                ntsc: NtscSeparation::Notch,
                pal: PalSeparation::DelayLine,
                ..ColorConfig::default()
            };
            let mut dec = ColorDecoder::new(std, &cfg);
            let mut last = None;
            for seq in 0..4u64 {
                let (par, fi) = field_info(seq);
                let f = image_to_field(&img, std, par, seq);
                let recs = enc.encode_field(&f, fi, 0);
                let mut fld = Field::new(std, seq, par);
                for r in &recs {
                    fld.put_line(r);
                }
                last = Some((dec.decode_field(Arc::new(fld)), f));
            }
            let (out, truth) = last.unwrap();
            let w = out.width;
            let j = out.lines / 2;
            for bar in 0..8 {
                let x = (bar * 2 + 1) * w / 16;
                let k = j * w + x;
                assert!((out.y[k] - truth.y[k]).abs() < 0.02, "{std:?} bar {bar} y {} vs {}", out.y[k], truth.y[k]);
                assert!((out.u[k] - truth.u[k]).abs() < 0.03, "{std:?} bar {bar} u {} vs {}", out.u[k], truth.u[k]);
                assert!((out.v[k] - truth.v[k]).abs() < 0.03, "{std:?} bar {bar} v {} vs {}", out.v[k], truth.v[k]);
            }
        }
    }
}
