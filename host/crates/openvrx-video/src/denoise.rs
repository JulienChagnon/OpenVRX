// SPDX-License-Identifier: GPL-3.0-or-later
//! Past-only noise reduction on progressive frames (zero added delay).
//!
//! - Temporal: motion-gated recursive filter. Each pixel is blended with
//!   the previous output where the local (3x3 mean) difference is within
//!   the noise level, with separate luma and chroma strengths. The noise
//!   level comes from the lines' `LineQuality::noise_ire` (porch noise
//!   measured by the demodulator) and from a flat-region estimate of the
//!   previous frame (Laplacian residual, lower percentile of blocks).
//!   Damaged samples are never blended in either direction, and a scene
//!   cut (large mean difference) stops the recursion for the rest of the
//!   frame.
//! - Motion compensated (high-quality option): the previous output is
//!   first aligned with block motion vectors (half-sample, smoothly
//!   interpolated between blocks), so pans and moving objects are filtered
//!   too instead of being passed through.
//! - Spatial (optional, for low C/N): a light 3x3 sigma filter that only
//!   averages neighbours within the noise band, so edges are kept.
//!
//! Work is row-incremental for streaming: a row is finished once the rows
//! it depends on are available, and the result is identical to processing
//! the whole frame at once.

use crate::motion::{self, MotionField, Plane, SearchParams};
use crate::util::{scratch_any, smoothstep, Rows3Mut};
use openvrx_core::quality::damage;
use openvrx_core::{Frame, VideoStandard};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::ops::Range;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TemporalMode {
    Off,
    Recursive,
    MotionCompensated,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DenoiseConfig {
    pub temporal: TemporalMode,
    /// Luma temporal strength 0..1.
    pub luma_strength: f32,
    /// Chroma temporal strength 0..1.
    pub chroma_strength: f32,
    /// Spatial sigma-filter strength 0 (off) .. 1.
    pub spatial: f32,
    /// Normalised local difference below which pixels are fully filtered.
    pub motion_lo: f32,
    /// Normalised local difference above which pixels are not filtered.
    pub motion_hi: f32,
    /// Damage at or above this is never blended.
    pub damage_threshold: u8,
    /// Mean absolute Y difference that counts as a scene cut (raised to
    /// three times the noise level when the picture is noisier).
    pub scene_cut: f32,
}

impl Default for DenoiseConfig {
    fn default() -> Self {
        DenoiseConfig {
            temporal: TemporalMode::Recursive,
            luma_strength: 0.6,
            chroma_strength: 0.8,
            spatial: 0.0,
            motion_lo: 1.0,
            motion_hi: 2.5,
            damage_threshold: damage::CLICK,
            scene_cut: 0.06,
        }
    }
}

impl DenoiseConfig {
    pub fn off() -> DenoiseConfig {
        DenoiseConfig {
            temporal: TemporalMode::Off,
            spatial: 0.0,
            ..DenoiseConfig::default()
        }
    }

    pub fn is_active(&self) -> bool {
        self.temporal != TemporalMode::Off || self.spatial > 0.0
    }
}

const MC_BLOCK: usize = 16;

/// Statistics of the last frame.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct DenoiseStats {
    pub sigma_y: f32,
    pub sigma_c: f32,
    pub scene_cut: bool,
    /// Mean luma blend factor.
    pub mean_k: f32,
}

pub struct Denoiser {
    cfg: DenoiseConfig,
    span: f32,
    prev: Option<Frame>,
    prev_motion: Option<MotionField>,
    /// Noise measured on the previous frame (Y, chroma).
    measured: Option<(f32, f32)>,
    // Per-frame work.
    active: bool,
    sp: Option<Frame>,
    sp_done: usize,
    motion: Option<MotionField>,
    motion_rows_done: usize,
    out_done: usize,
    row_diff: Vec<f32>,
    cut: bool,
    cut_rows: usize,
    cut_sum: f64,
    k_sum: f64,
    sigma_frame: (f32, f32),
    last: DenoiseStats,
}

impl Denoiser {
    pub fn new(standard: VideoStandard, cfg: &DenoiseConfig) -> Denoiser {
        let p = standard.params();
        Denoiser {
            cfg: cfg.clone(),
            span: p.white_ire - p.black_ire,
            prev: None,
            prev_motion: None,
            measured: None,
            active: false,
            sp: None,
            sp_done: 0,
            motion: None,
            motion_rows_done: 0,
            out_done: 0,
            row_diff: Vec::new(),
            cut: false,
            cut_rows: 0,
            cut_sum: 0.0,
            k_sum: 0.0,
            sigma_frame: (0.0, 0.0),
            last: DenoiseStats::default(),
        }
    }

    pub fn set_config(&mut self, cfg: &DenoiseConfig) {
        if cfg.temporal != self.cfg.temporal {
            self.prev_motion = None;
        }
        self.cfg = cfg.clone();
    }

    pub fn config(&self) -> &DenoiseConfig {
        &self.cfg
    }

    pub fn stats(&self) -> DenoiseStats {
        self.last
    }

    pub fn reset(&mut self) {
        self.prev = None;
        self.prev_motion = None;
        self.measured = None;
        self.active = false;
    }

    /// Rows of `frame` below row r needed before r can be finished.
    pub fn lookahead_rows(&self) -> usize {
        let base = if self.cfg.spatial > 0.0 { 2 } else { 1 };
        if self.cfg.temporal == TemporalMode::MotionCompensated {
            base + MC_BLOCK + MC_BLOCK / 2
        } else {
            base
        }
    }

    fn begin(&mut self, input: &Frame) {
        let (w, h) = (input.width, input.height);
        let mut sp = self.sp.take().filter(|f| f.width == w && f.height == h).unwrap_or_else(|| Frame::new(w, h));
        sp.meta = input.meta.clone();
        self.sp = Some(sp);
        self.sp_done = 0;
        self.out_done = 0;
        self.row_diff = vec![0.0; h];
        self.cut = false;
        self.cut_rows = 0;
        self.cut_sum = 0.0;
        self.k_sum = 0.0;
        let prev_ok = self.prev.as_ref().map(|p| p.width == w && p.height == h).unwrap_or(false);
        if !prev_ok {
            self.prev = None;
        }
        self.motion = if self.cfg.temporal == TemporalMode::MotionCompensated && self.prev.is_some() {
            Some(MotionField::new(w, h, &mc_params()))
        } else {
            None
        };
        self.motion_rows_done = 0;
        // Noise level from the previous frame (line quality records and
        // the flat-region measurement), so that streaming and whole-frame
        // processing see the same value.
        let (sy, sc) = self.measured.unwrap_or((0.01, 0.015));
        self.sigma_frame = (sy.max(0.002), sc.max(0.002));
        self.active = true;
    }

    /// Advances: rows `0..rows_ready` of `input` are final (`complete` when
    /// all are). Writes finished rows into `out` and returns them.
    pub fn advance(&mut self, input: &Frame, rows_ready: usize, complete: bool, out: &mut Frame, parallel: bool) -> Range<usize> {
        if !self.active {
            self.begin(input);
        }
        let h = input.height;
        let w = input.width;
        let ready = if complete { h } else { rows_ready.min(h) };
        let (sy, sc) = self.sigma_frame;

        // 1. Spatial stage (or copy).
        let sp_target = if complete { h } else { ready.saturating_sub(1) };
        if sp_target > self.sp_done {
            let r0 = self.sp_done;
            let strength = self.cfg.spatial.clamp(0.0, 1.0);
            let sp = self.sp.as_mut().expect("spatial buffer");
            let ys = &mut sp.y[r0 * w..sp_target * w];
            let us = &mut sp.u[r0 * w..sp_target * w];
            let vs = &mut sp.v[r0 * w..sp_target * w];
            let job = |(i, ((y, u), v)): (usize, Rows3Mut)| {
                let r = r0 + i;
                let f = row_factor(input, r);
                spatial_row(&input.y, w, h, r, sy * f, strength, y);
                spatial_row(&input.u, w, h, r, sc * f, strength, u);
                spatial_row(&input.v, w, h, r, sc * f, strength, v);
            };
            if parallel {
                ys.par_chunks_mut(w).zip(us.par_chunks_mut(w)).zip(vs.par_chunks_mut(w)).enumerate().for_each(job);
            } else {
                ys.chunks_mut(w).zip(us.chunks_mut(w)).zip(vs.chunks_mut(w)).enumerate().for_each(job);
            }
            sp.damage[r0 * w..sp_target * w].copy_from_slice(&input.damage[r0 * w..sp_target * w]);
            self.sp_done = sp_target;
        }
        let sp = self.sp.as_ref().expect("spatial buffer");

        // 2. Motion estimation (MC mode), block rows in order.
        if let (Some(mf), Some(prev)) = (self.motion.as_mut(), self.prev.as_ref()) {
            let sp_par = mc_params();
            while self.motion_rows_done < mf.rows {
                let br = self.motion_rows_done;
                let need = ((br + 1) * MC_BLOCK).min(h);
                if !(complete || need <= self.sp_done) {
                    break;
                }
                motion::estimate_row(
                    &Plane::new(&sp.y, w, h),
                    &Plane::new(&prev.y, w, h),
                    &sp_par,
                    self.prev_motion.as_ref(),
                    mf,
                    br,
                    parallel,
                );
                self.motion_rows_done += 1;
            }
        }

        // 3. Temporal stage.
        let mut target = if complete { h } else { self.sp_done.saturating_sub(1) };
        if let Some(mf) = &self.motion {
            if !complete {
                // Row r uses vectors of block rows up to (r + 9) / 16.
                let rows_ok = if self.motion_rows_done >= mf.rows {
                    h
                } else {
                    (self.motion_rows_done * MC_BLOCK).saturating_sub(MC_BLOCK / 2 + 2)
                };
                target = target.min(rows_ok);
            }
        }
        let r0 = self.out_done;
        if target <= r0 {
            return r0..r0;
        }
        let temporal = self.cfg.temporal != TemporalMode::Off && self.prev.is_some();
        if !temporal {
            out.y[r0 * w..target * w].copy_from_slice(&sp.y[r0 * w..target * w]);
            out.u[r0 * w..target * w].copy_from_slice(&sp.u[r0 * w..target * w]);
            out.v[r0 * w..target * w].copy_from_slice(&sp.v[r0 * w..target * w]);
        } else {
            let prev = self.prev.as_ref().expect("prev");
            // Scene-cut bookkeeping: cumulative mean |dY| over rows so far.
            let diffs: Vec<f32> = (r0..target).map(|r| row_mean_diff(&sp.y, &prev.y, w, r)).collect();
            let mut cut_flags = Vec::with_capacity(target - r0);
            for (i, d) in diffs.iter().enumerate() {
                self.row_diff[r0 + i] = *d;
                self.cut_sum += *d as f64;
                self.cut_rows += 1;
                if !self.cut && self.cut_rows >= 16 && (self.cut_sum / self.cut_rows as f64) as f32 > self.cfg.scene_cut.max(3.0 * sy) {
                    self.cut = true;
                }
                cut_flags.push(self.cut);
            }
            let ctx = TemporalCtx {
                cfg: &self.cfg,
                cur: sp,
                prev,
                motion: self.motion.as_ref(),
                w,
                h,
                sy,
                sc,
            };
            let ys = &mut out.y[r0 * w..target * w];
            let us = &mut out.u[r0 * w..target * w];
            let vs = &mut out.v[r0 * w..target * w];
            let job = |(i, ((y, u), v)): (usize, Rows3Mut)| -> f64 {
                let r = r0 + i;
                if cut_flags[i] {
                    y.copy_from_slice(&sp.y[r * w..(r + 1) * w]);
                    u.copy_from_slice(&sp.u[r * w..(r + 1) * w]);
                    v.copy_from_slice(&sp.v[r * w..(r + 1) * w]);
                    0.0
                } else {
                    ctx.row(r, y, u, v)
                }
            };
            let ks: Vec<f64> = if parallel {
                ys.par_chunks_mut(w).zip(us.par_chunks_mut(w)).zip(vs.par_chunks_mut(w)).enumerate().map(job).collect()
            } else {
                ys.chunks_mut(w).zip(us.chunks_mut(w)).zip(vs.chunks_mut(w)).enumerate().map(job).collect()
            };
            self.k_sum += ks.iter().sum::<f64>();
        }
        out.damage[r0 * w..target * w].copy_from_slice(&input.damage[r0 * w..target * w]);
        out.row_source[r0..target].copy_from_slice(&input.row_source[r0..target]);
        out.row_quality[r0..target].copy_from_slice(&input.row_quality[r0..target]);
        self.out_done = target;
        r0..target
    }

    /// Completes the frame: finishes all rows, keeps the output as the
    /// recursion reference and measures noise for the next frame.
    pub fn finish(&mut self, input: &Frame, out: &mut Frame, parallel: bool) -> Range<usize> {
        let r = self.advance(input, input.height, true, out, parallel);
        out.meta = input.meta.clone();
        let (w, h) = (input.width, input.height);
        let flat_y = flat_noise(&input.y, w, h);
        let flat_c = 0.5 * (flat_noise(&input.u, w, h) + flat_noise(&input.v, w, h));
        let mut q: Vec<f32> = input.row_quality.iter().map(|q| q.noise_ire).filter(|n| n.is_finite()).collect();
        self.measured = Some(if q.is_empty() {
            (flat_y, flat_c)
        } else {
            let mid = q.len() / 2;
            q.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
            let a = q[mid] / self.span;
            (0.5 * (a + flat_y), flat_c.max(0.5 * a))
        });
        self.last = DenoiseStats {
            sigma_y: self.sigma_frame.0,
            sigma_c: self.sigma_frame.1,
            scene_cut: self.cut,
            mean_k: (self.k_sum / (w * h) as f64) as f32,
        };
        if self.cfg.temporal != TemporalMode::Off {
            match &mut self.prev {
                Some(p) if p.width == w && p.height == h => {
                    p.y.copy_from_slice(&out.y);
                    p.u.copy_from_slice(&out.u);
                    p.v.copy_from_slice(&out.v);
                    p.damage.copy_from_slice(&out.damage);
                }
                _ => self.prev = Some(out.clone()),
            }
            self.prev_motion = self.motion.take();
        }
        self.active = false;
        r
    }

    /// Whole-frame convenience.
    pub fn process(&mut self, input: &Frame) -> Frame {
        let mut out = input.clone();
        self.finish(input, &mut out, true);
        out
    }
}

fn mc_params() -> SearchParams {
    SearchParams {
        block_w: MC_BLOCK,
        block_h: MC_BLOCK,
        range_x: 16,
        range_y: 12,
        lambda: 0.0004,
        half_pel: true,
    }
}

/// Per-row noise factor from the line quality (a noisier line than the
/// frame median gets a proportionally higher noise allowance).
fn row_factor(f: &Frame, r: usize) -> f32 {
    let n = f.row_quality[r].noise_ire;
    if !n.is_finite() {
        return 1.0;
    }
    // Compare with the preceding rows' noise (robust local reference).
    // Only rows at or above r: they are final when r is processed.
    let lo = r.saturating_sub(16);
    let hi = r + 1;
    let mut buf = [0.0f32; 17];
    let mut k = 0;
    for x in f.row_quality[lo..hi].iter().map(|q| q.noise_ire).filter(|x| x.is_finite()) {
        buf[k] = x;
        k += 1;
    }
    let v = &mut buf[..k];
    if v.is_empty() {
        return 1.0;
    }
    let mid = v.len() / 2;
    v.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    if v[mid] <= 1e-6 {
        return 1.0;
    }
    (n / v[mid]).clamp(0.5, 4.0)
}

/// 3x3 sigma filter of one row (or a copy when strength is 0).
fn spatial_row(plane: &[f32], w: usize, h: usize, r: usize, sigma: f32, strength: f32, out: &mut [f32]) {
    let row = &plane[r * w..(r + 1) * w];
    if strength <= 0.0 {
        out.copy_from_slice(row);
        return;
    }
    let up = &plane[r.saturating_sub(1) * w..(r.saturating_sub(1) + 1) * w];
    let rd = (r + 1).min(h - 1);
    let dn = &plane[rd * w..(rd + 1) * w];
    let band = 2.5 * sigma;
    for x in 0..w {
        let c = row[x];
        let mut acc = c;
        let mut n = 1.0f32;
        for rr in [up, row, dn] {
            for dx in [-1isize, 0, 1] {
                let xx = (x as isize + dx).clamp(0, w as isize - 1) as usize;
                let v = rr[xx];
                if (v - c).abs() <= band {
                    acc += v;
                    n += 1.0;
                }
            }
        }
        // The centre was counted twice (as itself and as a neighbour).
        let avg = (acc - c) / (n - 1.0);
        out[x] = c + strength * (avg - c);
    }
}

fn row_mean_diff(a: &[f32], b: &[f32], w: usize, r: usize) -> f32 {
    let ra = &a[r * w..(r + 1) * w];
    let rb = &b[r * w..(r + 1) * w];
    ra.iter().zip(rb).map(|(x, y)| (x - y).abs()).sum::<f32>() / w as f32
}

/// Robust flat-region noise estimate (Laplacian residual over 16x16
/// blocks, lower-quartile block).
fn flat_noise(p: &[f32], w: usize, h: usize) -> f32 {
    let mut est = Vec::new();
    let bs = 16;
    let mut by = 1;
    while by + bs < h - 1 {
        let mut bx = 1;
        while bx + bs < w - 1 {
            let mut acc = 0.0f32;
            let mut n = 0usize;
            for y in (by..by + bs).step_by(2) {
                for x in bx..bx + bs {
                    let g = |dx: isize, dy: isize| p[(y as isize + dy) as usize * w + (x as isize + dx) as usize];
                    let l = g(-1, -1) - 2.0 * g(0, -1) + g(1, -1) - 2.0 * g(-1, 0) + 4.0 * g(0, 0) - 2.0 * g(1, 0) + g(-1, 1)
                        - 2.0 * g(0, 1)
                        + g(1, 1);
                    acc += l.abs();
                    n += 1;
                }
            }
            // E|L| = sigma * sqrt(2/pi) * 6 for white noise.
            est.push(acc / n as f32 * (std::f32::consts::FRAC_PI_2).sqrt() / 6.0);
            bx += bs * 2;
        }
        by += bs * 2;
    }
    if est.is_empty() {
        return 0.01;
    }
    let q = est.len() / 4;
    est.select_nth_unstable_by(q, |a, b| a.total_cmp(b));
    est[q]
}

/// Per-row constants of the temporal filter.
struct TemporalParams {
    kmax_y: f32,
    kmax_c: f32,
    lo: f32,
    hi: f32,
    inv_box_y: f32,
    inv_box_c: f32,
    thr: u8,
    single_max: f32,
}

/// `out = (c0 - r0) + (c1 - r1) + (c2 - r2)` (summed from 0 in this order).
#[inline(always)]
fn diff3(c0: &[f32], r0: &[f32], c1: &[f32], r1: &[f32], c2: &[f32], r2: &[f32], out: &mut [f32]) {
    let w = out.len();
    let (c0, r0, c1, r1, c2, r2) = (&c0[..w], &r0[..w], &c1[..w], &r1[..w], &c2[..w], &r2[..w]);
    for x in 0..w {
        let mut acc = 0.0f32;
        acc += c0[x] - r0[x];
        acc += c1[x] - r1[x];
        acc += c2[x] - r2[x];
        out[x] = acc;
    }
}

/// Horizontal 3-tap sum (clamped ends) divided by 9.
#[inline(always)]
fn box3_ninth(col: &[f32], out: &mut [f32]) {
    let w = out.len().min(col.len());
    if w < 3 {
        for x in 0..w {
            out[x] = (col[x.saturating_sub(1)] + col[x] + col[(x + 1).min(w - 1)]) * (1.0 / 9.0);
        }
        return;
    }
    let m = w - 2;
    let (a, b, c) = (&col[0..m], &col[1..m + 1], &col[2..m + 2]);
    for (j, o) in out[1..m + 1].iter_mut().enumerate() {
        *o = (a[j] + b[j] + c[j]) * (1.0 / 9.0);
    }
    out[0] = (col[0] + col[0] + col[1]) * (1.0 / 9.0);
    out[w - 1] = (col[w - 2] + col[w - 1] + col[w - 1]) * (1.0 / 9.0);
}

/// Motion-gated recursive blend of one row (all planes); writes the luma
/// blend factors to `ky`.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn temporal_kernel(
    p: &TemporalParams,
    cur: [&[f32]; 3],
    rf: [&[f32]; 3],
    bx: [&[f32]; 3],
    dc: &[u8],
    dp: &[u8],
    oy: &mut [f32],
    ou: &mut [f32],
    ov: &mut [f32],
    ky: &mut [f32],
) {
    let w = oy.len();
    let (cy, cu, cv) = (&cur[0][..w], &cur[1][..w], &cur[2][..w]);
    let (ry, ru, rv) = (&rf[0][..w], &rf[1][..w], &rf[2][..w]);
    let (by, bu, bv) = (&bx[0][..w], &bx[1][..w], &bx[2][..w]);
    let (dc, dp, ou, ov, ky) = (&dc[..w], &dp[..w], &mut ou[..w], &mut ov[..w], &mut ky[..w]);
    for x in 0..w {
        let (cyx, cux, cvx, ryx, rux, rvx) = (cy[x], cu[x], cv[x], ry[x], ru[x], rv[x]);
        let my = by[x].abs() * p.inv_box_y;
        let mc = bu[x].abs().max(bv[x].abs()) * p.inv_box_c;
        let damaged = (dc[x] >= p.thr) | (dp[x] >= p.thr);
        let off = damaged | ((cyx - ryx).abs() > p.single_max);
        let gy = 1.0 - smoothstep(p.lo, p.hi, my);
        let gc = 1.0 - smoothstep(p.lo, p.hi, my.max(mc));
        let k_y = if off { 0.0 } else { p.kmax_y * gy };
        let k_c = if off { 0.0 } else { p.kmax_c * gc };
        oy[x] = cyx + k_y * (ryx - cyx);
        ou[x] = cux + k_c * (rux - cux);
        ov[x] = cvx + k_c * (rvx - cvx);
        ky[x] = k_y;
    }
}

struct TemporalCtx<'a> {
    cfg: &'a DenoiseConfig,
    cur: &'a Frame,
    prev: &'a Frame,
    motion: Option<&'a MotionField>,
    w: usize,
    h: usize,
    sy: f32,
    sc: f32,
}

impl TemporalCtx<'_> {
    /// Reference row (previous output, motion compensated when enabled)
    /// of plane `plane` at row r.
    #[inline(always)]
    fn reference_row(&self, plane: usize, r: usize, out: &mut [f32]) {
        let data = match plane {
            0 => &self.prev.y,
            1 => &self.prev.u,
            _ => &self.prev.v,
        };
        let w = self.w;
        match self.motion {
            Some(mf) => {
                let pl = Plane::new(data, w, self.h);
                for (x, o) in out.iter_mut().enumerate() {
                    let (dx, dy) = mf.at_pixel(x, r);
                    *o = pl.sample(x as f32 + dx, r as f32 + dy);
                }
            }
            None => out.copy_from_slice(&data[r * w..(r + 1) * w]),
        }
    }

    /// 3x3 mean of (cur - ref) over rows r-1..r+1 for one plane into `out`;
    /// `centre_ref` receives the centre row's reference when motion
    /// compensated (without motion the reference rows are used in place).
    #[inline(always)]
    fn diff_box(&self, plane: usize, rows: [usize; 3], centre_ref: &mut [f32], out: &mut [f32]) {
        let w = self.w;
        let (cur, prev) = match plane {
            0 => (&self.cur.y, &self.prev.y),
            1 => (&self.cur.u, &self.prev.u),
            _ => (&self.cur.v, &self.prev.v),
        };
        fn row(p: &[f32], w: usize, r: usize) -> &[f32] {
            &p[r * w..(r + 1) * w]
        }
        let mut col = scratch_any(w);
        if self.motion.is_some() {
            let mut r0 = scratch_any(w);
            let mut r2 = scratch_any(w);
            self.reference_row(plane, rows[0], &mut r0);
            self.reference_row(plane, rows[1], centre_ref);
            self.reference_row(plane, rows[2], &mut r2);
            diff3(row(cur, w, rows[0]), &r0, row(cur, w, rows[1]), centre_ref, row(cur, w, rows[2]), &r2, &mut col);
        } else {
            diff3(
                row(cur, w, rows[0]),
                row(prev, w, rows[0]),
                row(cur, w, rows[1]),
                row(prev, w, rows[1]),
                row(cur, w, rows[2]),
                row(prev, w, rows[2]),
                &mut col,
            );
        }
        box3_ninth(&col, out);
    }

    /// Filters row r; returns the sum of luma blend factors.
    fn row(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32]) -> f64 {
        #[cfg(target_arch = "x86_64")]
        {
            if crate::util::has_avx2() {
                // SAFETY: AVX2 and FMA are available (checked at run time).
                return unsafe { self.row_avx2(r, oy, ou, ov) };
            }
        }
        self.row_impl(r, oy, ou, ov)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn row_avx2(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32]) -> f64 {
        self.row_impl(r, oy, ou, ov)
    }

    #[inline(always)]
    fn row_impl(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32]) -> f64 {
        let w = self.w;
        let rows = [r.saturating_sub(1), r, (r + 1).min(self.h - 1)];
        let mc = self.motion.is_some();
        let mut ry = scratch_any(if mc { w } else { 0 });
        let mut ru = scratch_any(if mc { w } else { 0 });
        let mut rv = scratch_any(if mc { w } else { 0 });
        let mut by = scratch_any(w);
        let mut bu = scratch_any(w);
        let mut bv = scratch_any(w);
        self.diff_box(0, rows, &mut ry, &mut by);
        self.diff_box(1, rows, &mut ru, &mut bu);
        self.diff_box(2, rows, &mut rv, &mut bv);
        let base = r * w;
        let span = base..base + w;
        let (ry, ru, rv): (&[f32], &[f32], &[f32]) = if mc {
            (&ry, &ru, &rv)
        } else {
            (&self.prev.y[span.clone()], &self.prev.u[span.clone()], &self.prev.v[span.clone()])
        };
        let f = row_factor(self.cur, r);
        let sy = self.sy * f;
        let sc = self.sc * f;
        let p = TemporalParams {
            kmax_y: 0.85 * self.cfg.luma_strength.clamp(0.0, 1.0),
            kmax_c: 0.9 * self.cfg.chroma_strength.clamp(0.0, 1.0),
            lo: self.cfg.motion_lo,
            hi: self.cfg.motion_hi,
            inv_box_y: 1.0 / (0.42 * sy + 0.002),
            inv_box_c: 1.0 / (0.42 * sc + 0.002),
            thr: self.cfg.damage_threshold,
            single_max: 4.0 * sy + 0.03,
        };
        let mut ky = scratch_any(w);
        temporal_kernel(
            &p,
            [&self.cur.y[span.clone()], &self.cur.u[span.clone()], &self.cur.v[span.clone()]],
            [ry, ru, rv],
            [&by, &bu, &bv],
            &self.cur.damage[span.clone()],
            &self.prev.damage[span],
            oy,
            ou,
            ov,
            &mut ky,
        );
        // Summed in order, outside the vectorised loop.
        let mut ksum = 0.0f64;
        for &k in ky.iter() {
            ksum += k as f64;
        }
        ksum
    }

}
