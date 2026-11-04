// SPDX-License-Identifier: GPL-3.0-or-later
//! Damage-mask driven concealment on decoded fields.
//!
//! - C1 sparkle/click concealment: samples the damage mask flags on an
//!   otherwise good line are filled from the previous same-parity field
//!   where the neighbourhood is static, else by edge-directed spatial
//!   interpolation from the lines above and below (ELA over five
//!   directions), else by vertical interpolation from the nearest clean
//!   lines, else horizontally for short runs.
//! - C2 bad-line/dropout replacement: a line whose damage covers at least
//!   `bad_line_fraction` of the picture is replaced from the previous
//!   same-parity field where the surrounding lines show no motion, else
//!   spatially; when no spatial source exists the past field is used as a
//!   best effort.
//!
//! Safety rules:
//! - Only samples at or above `min_damage` in the mask are ever modified.
//! - Filling from the past is limited per region (blocks of 16 field lines
//!   x 64 samples): a block may be filled from the past for at most
//!   `max_past_fields` consecutive fields. After that the raw picture shows
//!   again in that block until a field arrives that needs no past fill
//!   there. Breakup is the pilot's range warning and must stay visible.
//!
//! The per-line work depends only on the unrepaired field, the previous
//! repaired field and the block permissions frozen at field start, so the
//! streaming and the whole-field paths give identical results.

use openvrx_core::quality::damage;
use openvrx_core::{YuvField, VideoStandard};
use crate::util::RowsMut;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::ops::Range;
use std::sync::Arc;

/// Region size for the past-fill limit.
pub const BLOCK_LINES: usize = 16;
pub const BLOCK_WIDTH: usize = 64;
/// Lines below a line that must be decoded before it can be repaired.
pub const LOOKAHEAD: usize = 2;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RepairConfig {
    /// C1 sparkle concealment.
    pub sparkle: bool,
    /// C2 bad-line / dropout replacement.
    pub dropout: bool,
    /// Allow filling from the previous same-parity field.
    pub temporal: bool,
    /// Lowest damage value treated as damaged.
    pub min_damage: u8,
    /// Damaged fraction of a line that makes it a bad line (C2).
    pub bad_line_fraction: f32,
    /// Consecutive fields a region may be filled from the past.
    pub max_past_fields: u8,
    /// Mean absolute Y difference (0..1) below which a neighbourhood is
    /// static, on top of the noise allowance.
    pub motion_threshold: f32,
}

impl Default for RepairConfig {
    fn default() -> Self {
        RepairConfig {
            sparkle: true,
            dropout: true,
            temporal: true,
            min_damage: damage::CLICK,
            bad_line_fraction: 0.5,
            max_past_fields: 3,
            motion_threshold: 0.02,
        }
    }
}

/// Counters for the OSD and tests.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct RepairStats {
    pub temporal_samples: u64,
    pub spatial_samples: u64,
    pub unrepaired_samples: u64,
    /// Samples where a past fill was wanted but the region limit blocked it.
    pub limited_samples: u64,
    pub bad_lines: u64,
}

impl std::ops::AddAssign for RepairStats {
    fn add_assign(&mut self, o: RepairStats) {
        self.temporal_samples += o.temporal_samples;
        self.spatial_samples += o.spatial_samples;
        self.unrepaired_samples += o.unrepaired_samples;
        self.limited_samples += o.limited_samples;
        self.bad_lines += o.bad_lines;
    }
}

pub struct Repairer {
    cfg: RepairConfig,
    standard: VideoStandard,
    width: usize,
    lines: usize,
    bx: usize,
    by: usize,
    /// Consecutive past-filled fields per block.
    counters: Vec<u8>,
    /// Blocks allowed to take past fills in the current field.
    allowed: Vec<bool>,
    /// Blocks that wanted a past fill in the current field.
    demand: Vec<bool>,
    history: Vec<Arc<YuvField>>,
    cur_key: Option<(u64, openvrx_core::FieldParity)>,
    done: usize,
    field_stats: RepairStats,
    total: RepairStats,
}

impl Repairer {
    pub fn new(standard: VideoStandard, cfg: &RepairConfig) -> Repairer {
        let p = standard.params();
        let width = p.active_width();
        let lines = p.active_lines_per_field as usize;
        let bx = width.div_ceil(BLOCK_WIDTH);
        let by = lines.div_ceil(BLOCK_LINES);
        Repairer {
            cfg: cfg.clone(),
            standard,
            width,
            lines,
            bx,
            by,
            counters: vec![0; bx * by],
            allowed: vec![true; bx * by],
            demand: vec![false; bx * by],
            history: Vec::new(),
            cur_key: None,
            done: 0,
            field_stats: RepairStats::default(),
            total: RepairStats::default(),
        }
    }

    pub fn set_config(&mut self, cfg: &RepairConfig) {
        self.cfg = cfg.clone();
    }

    pub fn config(&self) -> &RepairConfig {
        &self.cfg
    }

    pub fn stats(&self) -> RepairStats {
        self.total
    }

    pub fn last_field_stats(&self) -> RepairStats {
        self.field_stats
    }

    /// Consecutive past-fill count of the block holding field line `j`,
    /// sample `x`.
    pub fn block_count(&self, j: usize, x: usize) -> u8 {
        self.counters[(j / BLOCK_LINES) * self.bx + x / BLOCK_WIDTH]
    }

    pub fn reset(&mut self) {
        let cfg = self.cfg.clone();
        *self = Repairer::new(self.standard, &cfg);
    }

    fn begin(&mut self, src: &YuvField) {
        let max = self.cfg.max_past_fields;
        for (a, &c) in self.allowed.iter_mut().zip(&self.counters) {
            *a = c < max;
        }
        self.demand.fill(false);
        self.cur_key = Some((src.seq, src.parity));
        self.done = 0;
        self.field_stats = RepairStats::default();
    }

    fn prev_same_parity(&self, src: &YuvField) -> Option<&YuvField> {
        self.history
            .iter()
            .find(|h| h.parity == src.parity && h.seq + 2 == src.seq)
            .map(|h| h.as_ref())
    }

    /// Repairs every line whose lookahead is decoded (`lines_ready` active
    /// lines of `src` are final, `complete` when all are). Lines are
    /// written to `out` (which must have the same geometry); returns the
    /// newly finished lines.
    pub fn advance(&mut self, src: &YuvField, lines_ready: usize, complete: bool, out: &mut YuvField, parallel: bool) -> Range<usize> {
        if self.cur_key != Some((src.seq, src.parity)) {
            self.begin(src);
        }
        let target = if complete {
            self.lines
        } else {
            lines_ready.saturating_sub(LOOKAHEAD).min(self.lines)
        };
        let start = self.done;
        if target <= start {
            return start..start;
        }
        let w = self.width;
        let enabled = self.cfg.sparkle || self.cfg.dropout;
        let prev = if self.cfg.temporal { self.prev_same_parity(src) } else { None };
        let ctx = LineCtx {
            cfg: &self.cfg,
            src,
            prev,
            allowed: &self.allowed,
            bx: self.bx,
            lines_avail: if complete { self.lines } else { lines_ready },
        };
        let job = |(i, (((y, u), v), d)): (usize, RowsMut)| -> (RepairStats, Vec<usize>) {
            let j = start + i;
            y.copy_from_slice(&src.y[j * w..(j + 1) * w]);
            u.copy_from_slice(&src.u[j * w..(j + 1) * w]);
            v.copy_from_slice(&src.v[j * w..(j + 1) * w]);
            d.copy_from_slice(&src.damage[j * w..(j + 1) * w]);
            if enabled {
                ctx.repair_line(j, y, u, v, d)
            } else {
                (RepairStats::default(), Vec::new())
            }
        };
        let ys = &mut out.y[start * w..target * w];
        let us = &mut out.u[start * w..target * w];
        let vs = &mut out.v[start * w..target * w];
        let ds = &mut out.damage[start * w..target * w];
        let results: Vec<(RepairStats, Vec<usize>)> = if parallel {
            ys.par_chunks_mut(w)
                .zip(us.par_chunks_mut(w))
                .zip(vs.par_chunks_mut(w))
                .zip(ds.par_chunks_mut(w))
                .enumerate()
                .map(job)
                .collect()
        } else {
            ys.chunks_mut(w)
                .zip(us.chunks_mut(w))
                .zip(vs.chunks_mut(w))
                .zip(ds.chunks_mut(w))
                .enumerate()
                .map(job)
                .collect()
        };
        for (st, blocks) in results {
            self.field_stats += st;
            for b in blocks {
                self.demand[b] = true;
            }
        }
        for j in start..target {
            out.quality[j] = src.quality[j];
            out.source[j] = src.source[j];
        }
        out.first_ns = src.first_ns;
        out.last_ns = src.last_ns;
        self.done = target;
        start..target
    }

    /// Finishes the field: updates the per-region past-fill counters and
    /// keeps the repaired field as the temporal source.
    pub fn finish_field(&mut self, src: &YuvField, out: &mut YuvField, parallel: bool) -> Range<usize> {
        let r = self.finish_lines(src, out, parallel);
        self.remember(Arc::new(out.clone()));
        r
    }

    /// `finish_field` without keeping the temporal source: the caller
    /// passes the finished field to `remember` (shared, no copy).
    pub(crate) fn finish_lines(&mut self, src: &YuvField, out: &mut YuvField, parallel: bool) -> Range<usize> {
        let r = self.advance(src, self.lines, true, out, parallel);
        for (c, &d) in self.counters.iter_mut().zip(&self.demand) {
            *c = if d { c.saturating_add(1) } else { 0 };
        }
        self.total += self.field_stats;
        self.cur_key = None;
        r
    }

    /// Keeps a repaired field as the temporal source of later fields.
    pub(crate) fn remember(&mut self, f: Arc<YuvField>) {
        self.history.insert(0, f);
        self.history.truncate(2);
    }

    /// Whole-field convenience.
    pub fn repair_field(&mut self, src: &YuvField) -> YuvField {
        let mut out = src.clone();
        self.finish_field(src, &mut out, true);
        out
    }

    /// Rows in the block grid (for tests).
    pub fn block_rows(&self) -> usize {
        self.by
    }
}

struct LineCtx<'a> {
    cfg: &'a RepairConfig,
    src: &'a YuvField,
    prev: Option<&'a YuvField>,
    allowed: &'a [bool],
    bx: usize,
    lines_avail: usize,
}

enum Fill {
    Temporal,
    Spatial,
}

impl LineCtx<'_> {
    #[inline]
    fn bad(&self, j: usize, x: usize) -> bool {
        self.src.damage[j * self.src.width + x] >= self.cfg.min_damage
    }

    fn repair_line(&self, j: usize, y: &mut [f32], u: &mut [f32], v: &mut [f32], d: &mut [u8]) -> (RepairStats, Vec<usize>) {
        let w = self.src.width;
        let thr = self.cfg.min_damage;
        let mut st = RepairStats::default();
        let mut blocks = Vec::new();
        let count = d.iter().filter(|&&v| v >= thr).count();
        if count == 0 {
            return (st, blocks);
        }
        let bad_line = count as f32 >= self.cfg.bad_line_fraction * w as f32;
        if bad_line {
            st.bad_lines += 1;
            if !self.cfg.dropout {
                st.unrepaired_samples += count as u64;
                return (st, blocks);
            }
        } else if !self.cfg.sparkle {
            st.unrepaired_samples += count as u64;
            return (st, blocks);
        }
        let q = &self.src.quality[j];
        let sigma = if q.noise_ire.is_finite() { (q.noise_ire / 100.0).min(0.3) } else { 0.02 };
        // Process damaged runs.
        let mut x = 0;
        while x < w {
            if d[x] < thr {
                x += 1;
                continue;
            }
            let rs = x;
            while x < w && d[x] >= thr {
                x += 1;
            }
            let run = rs..x;
            let static_state = self.prev.and_then(|p| self.static_check(p, j, run.clone(), sigma));
            for xx in run.clone() {
                let block = (j / BLOCK_LINES) * self.bx + xx / BLOCK_WIDTH;
                let temporal_ok = self
                    .prev
                    .map(|p| p.damage[j * w + xx] < thr)
                    .unwrap_or(false);
                let spatial = self.spatial(j, xx, run.clone());
                // Decide what we would like to use.
                let want_past = temporal_ok && (static_state == Some(true) || (bad_line && spatial.is_none()));
                let mut fill = None;
                if want_past {
                    if !blocks.contains(&block) {
                        blocks.push(block);
                    }
                    if self.allowed[block] {
                        fill = Some(Fill::Temporal);
                    } else {
                        st.limited_samples += 1;
                    }
                }
                if fill.is_none() && spatial.is_some() {
                    fill = Some(Fill::Spatial);
                }
                match fill {
                    Some(Fill::Temporal) => {
                        let p = self.prev.expect("prev");
                        let k = j * w + xx;
                        y[xx] = p.y[k];
                        u[xx] = p.u[k];
                        v[xx] = p.v[k];
                        d[xx] = d[xx].min(damage::REPAIRED);
                        st.temporal_samples += 1;
                    }
                    Some(Fill::Spatial) => {
                        let (a, b, c) = spatial.expect("spatial");
                        y[xx] = a;
                        u[xx] = b;
                        v[xx] = c;
                        d[xx] = d[xx].min(damage::REPAIRED);
                        st.spatial_samples += 1;
                    }
                    None => st.unrepaired_samples += 1,
                }
            }
        }
        (st, blocks)
    }

    /// Some(true) when the neighbourhood of `run` on line j matches the
    /// previous same-parity field, Some(false) when it moved, None when
    /// too few clean samples exist to tell.
    fn static_check(&self, prev: &YuvField, j: usize, run: Range<usize>, sigma: f32) -> Option<bool> {
        let w = self.src.width;
        let thr = self.cfg.min_damage;
        let lo = run.start.saturating_sub(12);
        let hi = (run.end + 12).min(w);
        let j0 = j.saturating_sub(2);
        let j1 = (j + 2).min(self.lines_avail.saturating_sub(1)).min(self.src.lines - 1);
        let mut sum = 0.0f32;
        let mut n = 0usize;
        for jj in j0..=j1 {
            for x in lo..hi {
                let k = jj * w + x;
                if self.src.damage[k] >= thr || prev.damage[k] >= thr {
                    continue;
                }
                sum += (self.src.y[k] - prev.y[k]).abs()
                    + 0.5 * ((self.src.u[k] - prev.u[k]).abs() + (self.src.v[k] - prev.v[k]).abs());
                n += 1;
            }
        }
        if n < 16 {
            return None;
        }
        let mean = sum / n as f32;
        Some(mean < self.cfg.motion_threshold + 1.6 * sigma)
    }

    /// Spatial estimate (Y, U, V) for sample x of line j.
    fn spatial(&self, j: usize, x: usize, run: Range<usize>) -> Option<(f32, f32, f32)> {
        let w = self.src.width;
        let lines = self.src.lines.min(self.lines_avail);
        let s = self.src;
        let at = |jj: usize, xx: usize| jj * w + xx;
        // ELA between the adjacent lines.
        if j >= 1 && j + 1 < lines {
            let (ja, jb) = (j - 1, j + 1);
            let mut best: Option<(f32, isize)> = None;
            for dd in -2isize..=2 {
                let xa = x as isize + dd;
                let xb = x as isize - dd;
                if xa - 1 < 0 || xb - 1 < 0 || xa + 1 >= w as isize || xb + 1 >= w as isize {
                    continue;
                }
                let (xa, xb) = (xa as usize, xb as usize);
                if self.bad(ja, xa) || self.bad(jb, xb) {
                    continue;
                }
                let mut cost = 0.0;
                let mut ok = true;
                for i in -1isize..=1 {
                    let pa = (xa as isize + i) as usize;
                    let pb = (xb as isize + i) as usize;
                    if self.bad(ja, pa) || self.bad(jb, pb) {
                        ok = false;
                        break;
                    }
                    cost += (s.y[at(ja, pa)] - s.y[at(jb, pb)]).abs();
                }
                if !ok {
                    continue;
                }
                cost += 0.004 * dd.unsigned_abs() as f32;
                if best.map(|b| cost < b.0).unwrap_or(true) {
                    best = Some((cost, dd));
                }
            }
            if let Some((_, dd)) = best {
                let ka = at(ja, (x as isize + dd) as usize);
                let kb = at(jb, (x as isize - dd) as usize);
                return Some((
                    0.5 * (s.y[ka] + s.y[kb]),
                    0.5 * (s.u[ka] + s.u[kb]),
                    0.5 * (s.v[ka] + s.v[kb]),
                ));
            }
        }
        // Nearest clean lines above and below within two lines.
        let mut above = None;
        for dj in 1..=2 {
            if j >= dj && !self.bad(j - dj, x) {
                above = Some(j - dj);
                break;
            }
        }
        let mut below = None;
        for dj in 1..=2 {
            if j + dj < lines && !self.bad(j + dj, x) {
                below = Some(j + dj);
                break;
            }
        }
        match (above, below) {
            (Some(a), Some(b)) => {
                let t = (j - a) as f32 / (b - a) as f32;
                let ka = at(a, x);
                let kb = at(b, x);
                return Some((
                    s.y[ka] + t * (s.y[kb] - s.y[ka]),
                    s.u[ka] + t * (s.u[kb] - s.u[ka]),
                    s.v[ka] + t * (s.v[kb] - s.v[ka]),
                ));
            }
            (Some(a), None) if j - a == 1 => {
                let k = at(a, x);
                return Some((s.y[k], s.u[k], s.v[k]));
            }
            (None, Some(b)) if b - j == 1 => {
                let k = at(b, x);
                return Some((s.y[k], s.u[k], s.v[k]));
            }
            _ => {}
        }
        // Horizontal interpolation across a short run.
        if run.len() <= 16 && run.start >= 1 && run.end < w {
            let ka = at(j, run.start - 1);
            let kb = at(j, run.end);
            let t = (x - run.start + 1) as f32 / (run.len() + 1) as f32;
            return Some((
                s.y[ka] + t * (s.y[kb] - s.y[ka]),
                s.u[ka] + t * (s.u[kb] - s.u[ka]),
                s.v[ka] + t * (s.v[kb] - s.v[ka]),
            ));
        }
        None
    }
}
