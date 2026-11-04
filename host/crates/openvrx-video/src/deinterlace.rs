// SPDX-License-Identifier: GPL-3.0-or-later
//! Deinterlacing: one progressive frame per field, with every field's own
//! lines placed at their true rows (`StandardParams::field_row_offset`) so
//! there is no bob jitter, and the missing rows interpolated.
//!
//! Past-only methods (no added delay, usable line by line while a field
//! arrives):
//! - `Bob`: 4-tap cubic vertical interpolation of the current field.
//! - `Weave`: missing rows from the previous (opposite parity) field.
//! - `LinearBlend`: weave followed by a [1 2 1]/4 vertical blend.
//! - `MotionAdaptive`: weave where the same-parity field differences show
//!   no motion, bob where they do (soft, noise-adaptive).
//! - `YadifPast`: yadif-style spatial (ELA) prediction clamped around the
//!   temporal prediction, using only past fields.
//! - `EdgeDirectedMA` (live default): motion from the current and two
//!   previous fields (same-parity differences plus a comb check of the
//!   previous field against the current one), weave where static,
//!   edge-directed interpolation over nine directions with a protective
//!   median against the temporal prediction where moving, damage aware.
//!
//! Offline methods with one field of lookahead (`latency_fields() == 1`):
//! - `Yadif`: the yadif algorithm with the previous and next fields.
//! - `Bwdif`: yadif's motion logic with a w3fdif-style interpolator (low
//!   vertical frequencies from the current field, high vertical frequencies
//!   from the temporal neighbours).
//! - `MotionCompensated`: block motion between same-parity fields, the
//!   previous and next fields sampled along the motion, used where both
//!   agree or match their blocks well, edge-directed interpolation where
//!   compensation fails.

use crate::motion::{self, MotionField, Plane, SearchParams};
use crate::util::{RowsMut, cubic, median3, scratch, scratch_any, smoothstep, Scratch};
use openvrx_core::quality::damage;
use openvrx_core::{Frame, FrameMeta, LineQuality, RxId, VideoStandard, YuvField};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::ops::Range;
use std::sync::Arc;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeinterlaceMethod {
    Bob,
    Weave,
    LinearBlend,
    MotionAdaptive,
    YadifPast,
    EdgeDirectedMA,
    Yadif,
    Bwdif,
    MotionCompensated,
}

impl DeinterlaceMethod {
    pub const ALL: [DeinterlaceMethod; 9] = [
        DeinterlaceMethod::Bob,
        DeinterlaceMethod::Weave,
        DeinterlaceMethod::LinearBlend,
        DeinterlaceMethod::MotionAdaptive,
        DeinterlaceMethod::YadifPast,
        DeinterlaceMethod::EdgeDirectedMA,
        DeinterlaceMethod::Yadif,
        DeinterlaceMethod::Bwdif,
        DeinterlaceMethod::MotionCompensated,
    ];

    pub fn name(self) -> &'static str {
        match self {
            DeinterlaceMethod::Bob => "bob",
            DeinterlaceMethod::Weave => "weave",
            DeinterlaceMethod::LinearBlend => "blend",
            DeinterlaceMethod::MotionAdaptive => "motion-adaptive",
            DeinterlaceMethod::YadifPast => "yadif-past",
            DeinterlaceMethod::EdgeDirectedMA => "edi-ma",
            DeinterlaceMethod::Yadif => "yadif",
            DeinterlaceMethod::Bwdif => "bwdif",
            DeinterlaceMethod::MotionCompensated => "mc",
        }
    }

    /// Fields of delay added (0 for past-only methods).
    pub fn latency_fields(self) -> u32 {
        if self.is_bidirectional() {
            1
        } else {
            0
        }
    }

    pub fn is_bidirectional(self) -> bool {
        matches!(
            self,
            DeinterlaceMethod::Yadif | DeinterlaceMethod::Bwdif | DeinterlaceMethod::MotionCompensated
        )
    }

    /// Previous fields used.
    pub fn history_len(self) -> usize {
        match self {
            DeinterlaceMethod::Bob => 0,
            DeinterlaceMethod::Weave | DeinterlaceMethod::LinearBlend => 1,
            DeinterlaceMethod::EdgeDirectedMA => 2,
            DeinterlaceMethod::MotionAdaptive | DeinterlaceMethod::YadifPast => 3,
            DeinterlaceMethod::Yadif | DeinterlaceMethod::Bwdif | DeinterlaceMethod::MotionCompensated => 2,
        }
    }
}

/// Common interface of all deinterlacers.
pub trait Deinterlacer: Send {
    fn method(&self) -> DeinterlaceMethod;
    /// Fields of delay between a field going in and its frame coming out.
    fn latency_fields(&self) -> u32 {
        self.method().latency_fields()
    }
    /// Pushes a field; returns the frame that became ready (for past-only
    /// methods the frame of this field, for lookahead methods the frame of
    /// the previous field).
    fn push_field(&mut self, field: Arc<YuvField>) -> Option<Frame>;
    /// Emits the frame still held back by lookahead methods.
    fn flush(&mut self) -> Option<Frame>;
    fn reset(&mut self);
}

/// Creates a deinterlacer.
pub fn create(method: DeinterlaceMethod, standard: VideoStandard) -> Box<dyn Deinterlacer> {
    if method.is_bidirectional() {
        Box::new(BidiDeinterlacer::new(method, standard))
    } else {
        Box::new(PastDeinterlacer::new(method, standard))
    }
}

// ---------------------------------------------------------------------
// Field access helpers.

/// A field seen in frame-row coordinates.
#[derive(Copy, Clone)]
struct View<'a> {
    f: &'a YuvField,
    off: usize,
}

impl<'a> View<'a> {
    fn new(f: &'a YuvField, std: VideoStandard) -> View<'a> {
        View {
            f,
            off: std.params().field_row_offset(f.parity),
        }
    }

    /// Field line holding frame row `r` (r must have this field's parity;
    /// rows outside the frame clamp to the first/last line).
    #[inline]
    fn line(&self, r: isize) -> usize {
        ((r - self.off as isize).div_euclid(2)).clamp(0, self.f.lines as isize - 1) as usize
    }

    #[inline]
    fn rows(&self, r: isize) -> (&'a [f32], &'a [f32], &'a [f32], &'a [u8]) {
        let k = self.line(r);
        let w = self.f.width;
        let a = k * w;
        (
            &self.f.y[a..a + w],
            &self.f.u[a..a + w],
            &self.f.v[a..a + w],
            &self.f.damage[a..a + w],
        )
    }

    #[inline]
    fn plane(&self, r: isize, p: usize) -> &'a [f32] {
        let (y, u, v, _) = self.rows(r);
        match p {
            0 => y,
            1 => u,
            _ => v,
        }
    }

    /// Sample of plane `p` at fractional (x, frame row) using the field's
    /// own lines (cubic vertically, linear horizontally).
    fn sample(&self, p: usize, x: f32, row: f32) -> f32 {
        let yl = (row - self.off as f32) * 0.5;
        let yf = yl.floor();
        let t = yl - yf;
        let base = yf as isize * 2 + self.off as isize;
        let g = |k: isize| crate::util::sample_linear(self.plane(base + 2 * k, p), x);
        cubic(g(-1), g(0), g(1), g(2), t)
    }
}

/// Median of the noise estimates of a field, in Y units.
fn field_sigma(f: &YuvField) -> f32 {
    let p = f.standard.params();
    let mut v: Vec<f32> = f.quality.iter().map(|q| q.noise_ire).filter(|n| n.is_finite()).collect();
    if v.is_empty() {
        return 0.0;
    }
    let mid = v.len() / 2;
    v.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    (v[mid] / (p.white_ire - p.black_ire)).clamp(0.0, 0.2)
}

/// Output frame rows of a field `rows` computable when `lines_ready` field
/// lines are final.
pub fn rows_ready(standard: VideoStandard, field: &YuvField, lines_ready: usize, complete: bool) -> usize {
    let h = 2 * field.lines;
    if complete {
        return h;
    }
    let off = standard.params().field_row_offset(field.parity) as isize;
    // Row r needs field lines up to the one holding frame row r + 3.
    let r = 2 * lines_ready as isize - 4 + off;
    r.clamp(0, h as isize) as usize
}

// ---------------------------------------------------------------------
// Row kernel.

/// Everything one output row needs.
struct Ctx<'a> {
    method: DeinterlaceMethod,
    cur: View<'a>,
    p: Option<View<'a>>,
    pp: Option<View<'a>>,
    ppp: Option<View<'a>>,
    n: Option<View<'a>>,
    width: usize,
    height: usize,
    sigma: f32,
    mc: Option<&'a McData>,
}

/// Motion fields for the motion-compensated method.
struct McData {
    /// Current field vs the same-parity field two back.
    back: MotionField,
    /// Next field vs the previous field.
    fwd: Option<MotionField>,
}

type RowOut<'b> = (&'b mut [f32], &'b mut [f32], &'b mut [f32], &'b mut [u8]);

impl Ctx<'_> {
    fn own(&self, r: usize) -> bool {
        r % 2 == self.cur.off
    }

    /// Renders output row `r`. Returns the row's source and quality.
    fn row(&self, r: usize, out: RowOut) -> (RxId, LineQuality) {
        #[cfg(target_arch = "x86_64")]
        {
            if crate::util::has_avx2() {
                // SAFETY: AVX2 and FMA are available (checked at run time).
                return unsafe { self.row_avx2(r, out) };
            }
        }
        self.row_impl(r, out)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn row_avx2(&self, r: usize, out: RowOut) -> (RxId, LineQuality) {
        self.row_impl(r, out)
    }

    #[inline(always)]
    fn row_impl(&self, r: usize, out: RowOut) -> (RxId, LineQuality) {
        let (oy, ou, ov, od) = out;
        let ri = r as isize;
        let k_src = if self.own(r) {
            self.cur.line(ri)
        } else if r == 0 {
            self.cur.line(ri + 1)
        } else {
            self.cur.line(ri - 1)
        };
        let meta = (self.cur.f.source[k_src], self.cur.f.quality[k_src]);
        if self.own(r) {
            let (y, u, v, d) = self.cur.rows(ri);
            if self.method == DeinterlaceMethod::LinearBlend && self.p.is_some() {
                self.blend_row(r, oy, ou, ov, od);
            } else {
                oy.copy_from_slice(y);
                ou.copy_from_slice(u);
                ov.copy_from_slice(v);
                od.copy_from_slice(d);
            }
            return meta;
        }
        match self.method {
            DeinterlaceMethod::Bob => self.bob(r, oy, ou, ov, od),
            DeinterlaceMethod::Weave => match self.p {
                Some(p) => {
                    let (y, u, v, d) = p.rows(ri);
                    oy.copy_from_slice(y);
                    ou.copy_from_slice(u);
                    ov.copy_from_slice(v);
                    od.copy_from_slice(d);
                }
                None => self.bob(r, oy, ou, ov, od),
            },
            DeinterlaceMethod::LinearBlend => match self.p {
                Some(_) => self.blend_row(r, oy, ou, ov, od),
                None => self.bob(r, oy, ou, ov, od),
            },
            DeinterlaceMethod::MotionAdaptive => self.motion_adaptive(r, oy, ou, ov, od),
            DeinterlaceMethod::YadifPast | DeinterlaceMethod::Yadif | DeinterlaceMethod::Bwdif => {
                self.yadif(r, oy, ou, ov, od)
            }
            DeinterlaceMethod::EdgeDirectedMA => self.edi_ma(r, oy, ou, ov, od),
            DeinterlaceMethod::MotionCompensated => self.mc(r, oy, ou, ov, od),
        }
        meta
    }

    #[inline(always)]
    fn bob(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let ri = r as isize;
        for (p, o) in [&mut *oy, &mut *ou, &mut *ov].into_iter().enumerate() {
            let a = self.cur.plane(ri - 3, p);
            let c = self.cur.plane(ri - 1, p);
            let e = self.cur.plane(ri + 1, p);
            let b = self.cur.plane(ri + 3, p);
            for x in 0..self.width {
                o[x] = (9.0 * (c[x] + e[x]) - a[x] - b[x]) * (1.0 / 16.0);
            }
        }
        let dc = self.cur.rows(ri - 1).3;
        let de = self.cur.rows(ri + 1).3;
        for x in 0..self.width {
            od[x] = dc[x].max(de[x]);
        }
    }

    /// Woven row r (own row from the current field, else previous field).
    #[inline(always)]
    fn woven(&self, r: isize) -> (&[f32], &[f32], &[f32], &[u8]) {
        let r = r.clamp(0, self.height as isize - 1);
        if r as usize % 2 == self.cur.off {
            self.cur.rows(r)
        } else {
            self.p.expect("previous field").rows(r)
        }
    }

    #[inline(always)]
    fn blend_row(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let ri = r as isize;
        let a = self.woven(ri - 1);
        let b = self.woven(ri);
        let c = self.woven(ri + 1);
        for x in 0..self.width {
            oy[x] = 0.25 * (a.0[x] + 2.0 * b.0[x] + c.0[x]);
            ou[x] = 0.25 * (a.1[x] + 2.0 * b.1[x] + c.1[x]);
            ov[x] = 0.25 * (a.2[x] + 2.0 * b.2[x] + c.2[x]);
            od[x] = a.3[x].max(b.3[x]).max(c.3[x]);
        }
    }

    fn noise_thr(&self) -> f32 {
        0.012 + 3.0 * self.sigma
    }

    #[inline(always)]
    fn motion_adaptive(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let Some(p) = self.p else {
            return self.bob(r, oy, ou, ov, od);
        };
        // Bob first, then pull toward the weave where static.
        self.bob(r, oy, ou, ov, od);
        let ri = r as isize;
        let w = self.width;
        let (ty, tu, tv, td) = p.rows(ri);
        let c = self.cur.rows(ri - 1).0;
        let e = self.cur.rows(ri + 1).0;
        let mut mot = scratch(w, 0.0);
        match self.pp {
            Some(pp) => {
                let c2 = pp.rows(ri - 1).0;
                let e2 = pp.rows(ri + 1).0;
                for x in 0..w {
                    mot[x] = (c[x] - c2[x]).abs().max((e[x] - e2[x]).abs());
                }
            }
            None => mot.fill(1.0),
        }
        if let Some(ppp) = self.ppp {
            let t3 = ppp.rows(ri).0;
            for x in 0..w {
                mot[x] = mot[x].max((ty[x] - t3[x]).abs());
            }
        }
        let thr = self.noise_thr();
        for x in 0..w {
            let m = mot[x.saturating_sub(1)].max(mot[x]).max(mot[(x + 1).min(w - 1)]);
            let mut k = smoothstep(thr, 3.0 * thr, m);
            if td[x] >= damage::CLICK {
                k = 1.0;
            }
            oy[x] = ty[x] + k * (oy[x] - ty[x]);
            ou[x] = tu[x] + k * (ou[x] - tu[x]);
            ov[x] = tv[x] + k * (ov[x] - tv[x]);
            if k < 0.5 {
                od[x] = td[x];
            }
        }
    }

    /// Yadif family. `YadifPast`: temporal prediction from the previous
    /// field only; `Yadif`/`Bwdif`: average of previous and next fields.
    #[inline(always)]
    fn yadif(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let Some(p) = self.p else {
            return self.bob(r, oy, ou, ov, od);
        };
        let ri = r as isize;
        let w = self.width;
        let bwdif = self.method == DeinterlaceMethod::Bwdif;
        let dp = p.rows(ri).3;
        let dc = self.cur.rows(ri - 1).3;
        let de = self.cur.rows(ri + 1).3;
        let outs = [oy, ou, ov];
        for (pl, o) in outs.into_iter().enumerate() {
            let c = self.cur.plane(ri - 1, pl);
            let e = self.cur.plane(ri + 1, pl);
            let cc = self.cur.plane(ri - 3, pl);
            let ee = self.cur.plane(ri + 3, pl);
            let p0 = p.plane(ri, pl);
            let pb = p.plane(ri - 2, pl);
            let pf = p.plane(ri + 2, pl);
            let (n0, nb, nf) = match self.n {
                Some(n) => (Some(n.plane(ri, pl)), Some(n.plane(ri - 2, pl)), Some(n.plane(ri + 2, pl))),
                None => (None, None, None),
            };
            let older = if self.n.is_some() { None } else { self.ppp.map(|v| v.plane(ri, pl)) };
            let (c2, e2) = match self.pp {
                Some(pp) => (Some(pp.plane(ri - 1, pl)), Some(pp.plane(ri + 1, pl))),
                None => (None, None),
            };
            let sp_eps = if pl == 0 { 1.0 / 255.0 } else { 0.0 };
            for x in 0..w {
                let (d, td0, b, f) = match (n0, nb, nf) {
                    (Some(n0), Some(nb), Some(nf)) => (
                        0.5 * (p0[x] + n0[x]),
                        (p0[x] - n0[x]).abs(),
                        0.5 * (pb[x] + nb[x]),
                        0.5 * (pf[x] + nf[x]),
                    ),
                    _ => (
                        p0[x],
                        older.map(|o| (p0[x] - o[x]).abs()).unwrap_or(f32::INFINITY),
                        pb[x],
                        pf[x],
                    ),
                };
                let (cx, ex) = (c[x], e[x]);
                let td1 = match (c2, e2) {
                    (Some(c2), Some(e2)) => 0.5 * ((c2[x] - cx).abs() + (e2[x] - ex).abs()),
                    _ => f32::INFINITY,
                };
                let mut diff = (td0 * 0.5).max(td1);
                let mx = (d - ex).max(d - cx).max((b - cx).min(f - ex));
                let mn = (d - ex).min(d - cx).min((b - cx).max(f - ex));
                diff = diff.max(mn).max(-mx);
                if dp[x] >= damage::CLICK {
                    diff = f32::INFINITY;
                }
                let spatial = if bwdif {
                    let lf = (9.0 * (cx + ex) - cc[x] - ee[x]) * (1.0 / 16.0);
                    match (n0, nb, nf) {
                        (Some(n0), Some(nb), Some(nf)) if (cx - ex).abs() > td0 => {
                            // Vertical high frequencies from the temporal
                            // average T = (P + N) / 2: T(m)/2 - (T(m-2) +
                            // T(m+2))/4 (zero at DC, unity at half the
                            // field's vertical Nyquist).
                            let t0 = 0.5 * (p0[x] + n0[x]);
                            let t2 = 0.25 * (pb[x] + pf[x] + nb[x] + nf[x]);
                            lf + 0.5 * (t0 - t2)
                        }
                        _ => lf,
                    }
                } else {
                    yadif_spatial(c, e, x, w, sp_eps)
                };
                o[x] = if diff.is_finite() {
                    spatial.clamp(d - diff, d + diff)
                } else {
                    spatial
                };
                if pl == 0 {
                    od[x] = if diff < 0.02 { dp[x] } else { dc[x].max(de[x]) };
                }
            }
        }
    }

    /// Edge-directed motion-adaptive interpolation.
    ///
    /// The per-sample work runs in whole-row kernels that take every row as
    /// a separate slice argument, so the compiler knows the rows do not
    /// overlap and vectorises each loop.
    #[inline(always)]
    fn edi_ma(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let ri = r as isize;
        let w = self.width;
        let (cy, cu, cv, cd) = self.cur.rows(ri - 1);
        let (ey, eu, ev, ed) = self.cur.rows(ri + 1);
        let ccy = self.cur.plane(ri - 3, 0);
        let eey = self.cur.plane(ri + 3, 0);
        let thr = self.noise_thr();
        let Some(p) = self.p else {
            let dir = edi_directions([cy, cu, cv], [ey, eu, ev], w, None);
            edi_spatial(
                [cy, cu, cv],
                [ey, eu, ev],
                ccy,
                eey,
                &dir.conf,
                [&dir.cand[0], &dir.cand[1], &dir.cand[2]],
                oy,
                ou,
                ov,
            );
            max_u8(cd, ed, od);
            return;
        };
        let (ty, tu, tv, td) = p.rows(ri);

        // Motion from same-parity differences (smoothed 3 taps).
        let mut mot = scratch(w, 0.0);
        if let Some(pp) = self.pp {
            let (c2, _, _, cd2) = pp.rows(ri - 1);
            let (e2, _, _, ed2) = pp.rows(ri + 1);
            let mut dc = scratch_any(w);
            let mut de = scratch_any(w);
            sub_rows(cy, c2, &mut dc);
            sub_rows(ey, e2, &mut de);
            let mut tmp = scratch_any(w);
            smooth3_abs(&dc, &mut mot);
            smooth3_abs(&de, &mut tmp);
            edi_motion_merge(&mut mot, &tmp, [cd, cd2, ed, ed2], 3.0 * thr);
        }
        // Comb check: the previous field against the current one.
        let mut comb = scratch_any(w);
        edi_comb(ty, cy, ey, p.plane(ri - 2, 0), p.plane(ri + 2, 0), &mut comb);
        let mut mw = scratch_any(w);
        let mut cw = scratch_any(w);
        max5_floor0(&mot, &mut mw);
        max5_floor0(&comb, &mut cw);
        let mut kk = scratch_any(w);
        edi_motion_k(&mw, &cw, self.pp.is_some(), thr, td, &mut kk);

        // The spatial prediction only matters where kk > 0 (static samples
        // take the temporal prediction whatever it is), so the direction
        // search skips blocks without motion.
        let dir = edi_directions([cy, cu, cv], [ey, eu, ev], w, Some(&kk));
        let mut sy = scratch_any(w);
        let mut su = scratch_any(w);
        let mut sv = scratch_any(w);
        edi_spatial(
            [cy, cu, cv],
            [ey, eu, ev],
            ccy,
            eey,
            &dir.conf,
            [&dir.cand[0], &dir.cand[1], &dir.cand[2]],
            &mut sy,
            &mut su,
            &mut sv,
        );
        edi_blend(&kk, [&sy, &su, &sv], [ty, tu, tv], td, [cy, cu, cv], [ey, eu, ev], cd, ed, oy, ou, ov, od);
    }

    /// Motion-compensated interpolation with EDI fallback.
    fn mc(&self, r: usize, oy: &mut [f32], ou: &mut [f32], ov: &mut [f32], od: &mut [u8]) {
        let (Some(p), Some(mc)) = (self.p, self.mc) else {
            return self.edi_ma(r, oy, ou, ov, od);
        };
        // Spatial fallback first.
        let mut sy = scratch(self.width, 0.0);
        let mut su = scratch(self.width, 0.0);
        let mut sv = scratch(self.width, 0.0);
        let mut sd = vec![0u8; self.width];
        let spatial_ctx = Ctx {
            method: DeinterlaceMethod::EdgeDirectedMA,
            cur: self.cur,
            p: None,
            pp: None,
            ppp: None,
            n: None,
            width: self.width,
            height: self.height,
            sigma: self.sigma,
            mc: None,
        };
        spatial_ctx.edi_ma(r, &mut sy, &mut su, &mut sv, &mut sd);
        let ri = r as isize;
        let w = self.width;
        let fl = self.cur.line(ri - 1).min(self.cur.f.lines - 1);
        let thr = self.noise_thr();
        let cthr = 0.015 + 2.0 * self.sigma;
        let cy = self.cur.plane(ri - 1, 0);
        let ey = self.cur.plane(ri + 1, 0);
        let dp = p.rows(ri).3;
        for x in 0..w {
            let (ax, ay) = mc.back.at_pixel(x, fl);
            let ca = mc.back.get(x / mc.back.block_w, fl / mc.back.block_h).cost;
            let rowf = r as f32;
            let pa = (x as f32 + 0.5 * ax, rowf + ay);
            let cand_p = [p.sample(0, pa.0, pa.1), p.sample(1, pa.0, pa.1), p.sample(2, pa.0, pa.1)];
            let good_p = ca < cthr && dp[x] < damage::CLICK;
            let (cand_n, good_n) = match (self.n, &mc.fwd) {
                (Some(n), Some(fwd)) => {
                    let (bx, by) = fwd.at_pixel(x, fl);
                    let cb = fwd.get(x / fwd.block_w, fl / fwd.block_h).cost;
                    let pn = (x as f32 - 0.5 * bx, rowf - by);
                    ([n.sample(0, pn.0, pn.1), n.sample(1, pn.0, pn.1), n.sample(2, pn.0, pn.1)], cb < cthr)
                }
                _ => ([0.0; 3], false),
            };
            let lo = cy[x].min(ey[x]) - thr - 0.5 * (cy[x] - ey[x]).abs();
            let hi = cy[x].max(ey[x]) + thr + 0.5 * (cy[x] - ey[x]).abs();
            let plaus = |v: f32| v >= lo && v <= hi;
            // Vertical phase: compensation is only sharp when the source
            // position falls on (or near) one of the field's own rows.
            let poff = p.off as f32;
            let vphase = |row: f32| {
                let u = (row - poff) * 0.5;
                2.0 * (u - u.round()).abs()
            };
            let agree = good_p && good_n && (cand_p[0] - cand_n[0]).abs() < thr;
            let wp = if good_p && (agree || plaus(cand_p[0])) {
                1.0 - smoothstep(0.25, 0.5, vphase(pa.1))
            } else {
                0.0
            };
            let wn = match (good_n, self.n) {
                (true, Some(_)) if agree || plaus(cand_n[0]) => {
                    let (_, by) = mc.fwd.as_ref().map(|f| f.at_pixel(x, fl)).unwrap_or((0.0, 0.0));
                    1.0 - smoothstep(0.25, 0.5, vphase(rowf - by))
                }
                _ => 0.0,
            };
            let ws = wp + wn;
            if ws > 0.0 {
                let conf = wp.max(wn);
                for (pl, o) in [&mut *oy, &mut *ou, &mut *ov].into_iter().enumerate() {
                    let sp = [sy[x], su[x], sv[x]][pl];
                    let c = (wp * cand_p[pl] + wn * cand_n[pl]) / ws;
                    o[x] = sp + conf * (c - sp);
                }
                od[x] = if conf > 0.5 { dp[x] } else { sd[x] };
            } else {
                oy[x] = sy[x];
                ou[x] = su[x];
                ov[x] = sv[x];
                od[x] = sd[x];
            }
        }
    }
}

/// yadif spatial prediction (ELA over +-2 with three-tap scores).
#[inline(always)]
fn yadif_spatial(c: &[f32], e: &[f32], x: usize, w: usize, eps: f32) -> f32 {
    let g = |row: &[f32], i: isize| row[(x as isize + i).clamp(0, w as isize - 1) as usize];
    let mut score = (g(c, -1) - g(e, -1)).abs() + (g(c, 0) - g(e, 0)).abs() + (g(c, 1) - g(e, 1)).abs() - eps;
    let mut pred = 0.5 * (g(c, 0) + g(e, 0));
    for dir in [-1isize, 1] {
        for j in [dir, 2 * dir] {
            let s = (g(c, j - 1) - g(e, -j - 1)).abs() + (g(c, j) - g(e, -j)).abs() + (g(c, j + 1) - g(e, -j + 1)).abs();
            if s < score {
                score = s;
                pred = 0.5 * (g(c, j) + g(e, -j));
            } else {
                break;
            }
        }
    }
    pred
}

/// `out = a - b`.
#[inline(always)]
fn sub_rows(a: &[f32], b: &[f32], out: &mut [f32]) {
    for ((o, &x), &y) in out.iter_mut().zip(a).zip(b) {
        *o = x - y;
    }
}

/// `out = max(a, b)` per byte.
#[inline(always)]
fn max_u8(a: &[u8], b: &[u8], out: &mut [u8]) {
    for ((o, &x), &y) in out.iter_mut().zip(a).zip(b) {
        *o = x.max(y);
    }
}

/// 3-tap mean (clamped ends) of `d`, then `max(|mean|, |d| / 2)`.
#[inline(always)]
fn smooth3_abs(d: &[f32], out: &mut [f32]) {
    let w = d.len().min(out.len());
    let f = |l: f32, c: f32, r: f32| ((l + c + r) / 3.0).abs().max(0.5 * c.abs());
    if w < 3 {
        for x in 0..w {
            out[x] = f(d[x.saturating_sub(1)], d[x], d[(x + 1).min(w - 1)]);
        }
        return;
    }
    let m = w - 2;
    let (a, b, c) = (&d[0..m], &d[1..m + 1], &d[2..m + 2]);
    for (j, o) in out[1..m + 1].iter_mut().enumerate() {
        *o = f(a[j], b[j], c[j]);
    }
    out[0] = f(d[0], d[0], d[1]);
    out[w - 1] = f(d[w - 2], d[w - 1], d[w - 1]);
}

/// Maximum over a 5-sample window (clipped at the ends), floored at 0.
#[inline(always)]
fn max5_floor0(v: &[f32], out: &mut [f32]) {
    let w = v.len().min(out.len());
    let edge = |x: usize| v[x.saturating_sub(2)..(x + 3).min(w)].iter().fold(0.0f32, |a, &b| a.max(b));
    if w < 5 {
        for (x, o) in out.iter_mut().enumerate().take(w) {
            *o = edge(x);
        }
        return;
    }
    let m = w - 4;
    let (a, b, c, d, e) = (&v[0..m], &v[1..m + 1], &v[2..m + 2], &v[3..m + 3], &v[4..m + 4]);
    for (j, o) in out[2..m + 2].iter_mut().enumerate() {
        *o = 0.0f32.max(a[j]).max(b[j]).max(c[j]).max(d[j]).max(e[j]);
    }
    for x in [0, 1, w - 2, w - 1] {
        out[x] = edge(x);
    }
}

/// Motion: the larger of the two smoothed differences, raised to `floor`
/// where any of the four lines is damaged.
#[inline(always)]
fn edi_motion_merge(mot: &mut [f32], other: &[f32], dmg: [&[u8]; 4], floor: f32) {
    let w = mot.len();
    let (o, d0, d1, d2, d3) = (&other[..w], &dmg[0][..w], &dmg[1][..w], &dmg[2][..w], &dmg[3][..w]);
    for x in 0..w {
        let m = mot[x].max(o[x]);
        let d = d0[x].max(d1[x]).max(d2[x]).max(d3[x]);
        mot[x] = if d >= damage::CLICK { m.max(floor) } else { m };
    }
}

/// Comb check of the previous field's row `t` (with its neighbours `pb`,
/// `pf`) against the current field's rows `c` above and `e` below.
#[inline(always)]
fn edi_comb(t: &[f32], c: &[f32], e: &[f32], pb: &[f32], pf: &[f32], out: &mut [f32]) {
    let w = out.len();
    let (t, c, e, pb, pf) = (&t[..w], &c[..w], &e[..w], &pb[..w], &pf[..w]);
    for x in 0..w {
        let tx = t[x];
        let mx = (tx - e[x]).max(tx - c[x]).max((pb[x] - c[x]).min(pf[x] - e[x]));
        let mn = (tx - e[x]).min(tx - c[x]).min((pb[x] - c[x]).max(pf[x] - e[x]));
        out[x] = mn.max(-mx);
    }
}

/// Spatial prediction: vertical cubic (luma) / two-tap average (chroma),
/// moved toward the edge-directed value where trusted; the diagonal value
/// is protected by a median against the vertical neighbours.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn edi_spatial(
    c: [&[f32]; 3],
    e: [&[f32]; 3],
    ccy: &[f32],
    eey: &[f32],
    conf: &[f32],
    cand: [&[f32]; 3],
    sy: &mut [f32],
    su: &mut [f32],
    sv: &mut [f32],
) {
    let w = sy.len();
    let (cy, cu, cv) = (&c[0][..w], &c[1][..w], &c[2][..w]);
    let (ey, eu, ev) = (&e[0][..w], &e[1][..w], &e[2][..w]);
    let (ay, au, av) = (&cand[0][..w], &cand[1][..w], &cand[2][..w]);
    let (ccy, eey, conf, su, sv) = (&ccy[..w], &eey[..w], &conf[..w], &mut su[..w], &mut sv[..w]);
    for x in 0..w {
        let vy = (9.0 * (cy[x] + ey[x]) - ccy[x] - eey[x]) * (1.0 / 16.0);
        let vu = 0.5 * (cu[x] + eu[x]);
        let vv = 0.5 * (cv[x] + ev[x]);
        let k = conf[x];
        let dy = median3(ay[x], cy[x], ey[x]);
        let du = median3(au[x], cu[x], eu[x]);
        let dv = median3(av[x], cv[x], ev[x]);
        let on = k > 0.0;
        sy[x] = if on { vy + k * (dy - vy) } else { vy };
        su[x] = if on { vu + k * (du - vu) } else { vu };
        sv[x] = if on { vv + k * (dv - vv) } else { vv };
    }
}

/// Motion weight per sample (0 static .. 1 moving) from the motion
/// (`mw`) and comb (`cw`) measures; 1 where the previous field is damaged.
#[inline(always)]
fn edi_motion_k(mw: &[f32], cw: &[f32], has_pp: bool, thr: f32, td: &[u8], out: &mut [f32]) {
    let w = out.len();
    let (mw, cw, td) = (&mw[..w], &cw[..w], &td[..w]);
    let (lo1, hi1, lo2) = (thr, 3.0 * thr, 0.3 * thr);
    for x in 0..w {
        let m = mw[x];
        let kc = smoothstep(lo1, hi1, cw[x]);
        // With two same-parity fields the motion detector is
        // authoritative; the comb check only sharpens the decision where
        // some motion was seen, so static detail always weaves (no
        // flicker).
        let kk = if has_pp {
            let km = smoothstep(lo1, hi1, m);
            km.max(kc.min(smoothstep(lo2, thr, m)))
        } else {
            kc
        };
        out[x] = if td[x] >= damage::CLICK { 1.0 } else { kk };
    }
}

/// Final edi-ma blend of the spatial prediction `s` with the temporal
/// prediction `t` by the motion weight `kk`.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn edi_blend(
    kk: &[f32],
    s: [&[f32]; 3],
    t: [&[f32]; 3],
    td: &[u8],
    c: [&[f32]; 3],
    e: [&[f32]; 3],
    cd: &[u8],
    ed: &[u8],
    oy: &mut [f32],
    ou: &mut [f32],
    ov: &mut [f32],
    od: &mut [u8],
) {
    let w = oy.len();
    let (kk, td, cd, ed) = (&kk[..w], &td[..w], &cd[..w], &ed[..w]);
    let (sy, su, sv) = (&s[0][..w], &s[1][..w], &s[2][..w]);
    let (ty, tu, tv) = (&t[0][..w], &t[1][..w], &t[2][..w]);
    let (cy, cu, cv) = (&c[0][..w], &c[1][..w], &c[2][..w]);
    let (ey, eu, ev) = (&e[0][..w], &e[1][..w], &e[2][..w]);
    let (ou, ov, od) = (&mut ou[..w], &mut ov[..w], &mut od[..w]);
    for x in 0..w {
        let kk = kk[x];
        // Every load is done unconditionally up front: loads inside a
        // branch cannot be speculated and would stop vectorisation.
        let (tdx, cdx, edx) = (td[x], cd[x], ed[x]);
        let dmg = tdx >= damage::CLICK;
        // Two-stage blend: static -> temporal prediction; slight motion ->
        // median of spatial, temporal and vertical average (keeps detail
        // that is still valid); clear motion -> spatial only.
        let k1 = (2.0 * kk).min(1.0);
        let k2 = (2.0 * kk - 1.0).max(0.0);
        let my = median3(sy[x], ty[x], 0.5 * (cy[x] + ey[x]));
        let mu = median3(su[x], tu[x], 0.5 * (cu[x] + eu[x]));
        let mv = median3(sv[x], tv[x], 0.5 * (cv[x] + ev[x]));
        let k1 = if dmg { 1.0 } else { k1 };
        let k2 = if dmg { 1.0 } else { k2 };
        let by = ty[x] + k1 * (my - ty[x]);
        let bu = tu[x] + k1 * (mu - tu[x]);
        let bv = tv[x] + k1 * (mv - tv[x]);
        oy[x] = by + k2 * (sy[x] - by);
        ou[x] = bu + k2 * (su[x] - bu);
        ov[x] = bv + k2 * (sv[x] - bv);
        let dsp = cdx.max(edx);
        od[x] = if kk < 0.5 { tdx } else { dsp };
    }
}

/// Edge direction search result: per sample a confidence 0..1 and the
/// two-tap average of Y, U and V along the chosen direction.
struct EdiDirs {
    conf: Scratch,
    cand: [Scratch; 3],
}

/// Search radius of the edge direction search (offsets -R..=R).
const EDI_R: isize = 4;
/// Samples per block of the direction search.
const EDI_L: usize = 8;

/// Chooses an interpolation direction per sample between rows `c` (above)
/// and `e` (below), planes Y, U, V: offsets -4..=4, cost = 5-sample window
/// of absolute luma differences along the direction. Returns per sample a
/// confidence 0..1 and the directional averages: a diagonal is trusted
/// only when it beats the vertical clearly, the vertical cost shows a real
/// edge, and the half-way direction is consistent (rejects aliased stripe
/// matches).
///
/// The interior is processed in blocks of eight samples with all nine
/// direction costs and the running best held in fixed-size arrays (which
/// the compiler keeps in vector registers); the few samples near the ends,
/// where indices clamp, go through `edi_pixel`. Both give the same values.
///
/// With `need`, samples where `need` is 0 get confidence 0 without a
/// search (their spatial prediction is not used).
#[inline(always)]
fn edi_directions(c: [&[f32]; 3], e: [&[f32]; 3], w: usize, need: Option<&[f32]>) -> EdiDirs {
    let mut conf = scratch_any(w);
    let mut cand = [scratch_any(w), scratch_any(w), scratch_any(w)];
    // Blocks need samples x0 - 6 .. x0 + 8 + 6 with unclamped indices and
    // the box filter's interior form; the last block may overlap the one
    // before it (same values). Only the first and last few samples take
    // the clamped per-sample path.
    const FIRST: usize = 6;
    let mut done = 0;
    if w >= FIRST + EDI_L + 6 {
        let last = w - EDI_L - 6;
        let mut x0 = FIRST;
        loop {
            let [o0, o1, o2] = &mut cand;
            let r = x0..x0 + EDI_L;
            if need.is_some_and(|n| n[r.clone()].iter().all(|&k| k == 0.0)) {
                conf[r].fill(0.0);
                if x0 == last {
                    break;
                }
                x0 = (x0 + EDI_L).min(last);
                continue;
            }
            #[cfg(target_arch = "x86_64")]
            let fast = crate::util::has_avx2();
            #[cfg(not(target_arch = "x86_64"))]
            let fast = false;
            if fast {
                #[cfg(target_arch = "x86_64")]
                // SAFETY: AVX2 is available; the bounds are checked inside.
                unsafe {
                    edi_block_avx2(c, e, x0, &mut conf[r.clone()], [&mut o0[r.clone()], &mut o1[r.clone()], &mut o2[r]])
                };
            } else {
                edi_block(c, e, x0, &mut conf[r.clone()], &mut o0[r.clone()], &mut o1[r.clone()], &mut o2[r]);
            }
            if x0 == last {
                break;
            }
            x0 = (x0 + EDI_L).min(last);
        }
        done = last + EDI_L;
    }
    let lead = if done == 0 { 0 } else { FIRST };
    for x in (0..lead).chain(done..w) {
        if need.is_some_and(|n| n[x] == 0.0) {
            conf[x] = 0.0;
            continue;
        }
        let (k, v) = edi_pixel(c, e, w, x);
        conf[x] = k;
        for p in 0..3 {
            cand[p][x] = v[p];
        }
    }
    EdiDirs { conf, cand }
}

/// `smoothstep` of eight values.
#[inline(always)]
fn smoothstep8(e0: f32, e1: f32, x: &[f32; EDI_L]) -> [f32; EDI_L] {
    let mut o = [0.0f32; EDI_L];
    for i in 0..EDI_L {
        o[i] = smoothstep(e0, e1, x[i]);
    }
    o
}

/// Direction search for samples `x0 .. x0 + 8` (interior only).
// `!(a > b)` on purpose: a NaN cost passes, as in the original search.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
#[inline(always)]
fn edi_block(
    c: [&[f32]; 3],
    e: [&[f32]; 3],
    x0: usize,
    conf: &mut [f32],
    o0: &mut [f32],
    o1: &mut [f32],
    o2: &mut [f32],
) {
    const L: usize = EDI_L;
    const ND: usize = (2 * EDI_R + 1) as usize;
    let (cy, ey) = (c[0], e[0]);
    // Costs: cost[d][i] = mean of |c[x + j + d] - e[x + j - d]|, j = -2..2.
    let mut cost = [[0.0f32; L]; ND];
    for (di, d) in (-EDI_R..=EDI_R).enumerate() {
        let a = &cy[(x0 as isize - 2 + d) as usize..][..L + 4];
        let b = &ey[(x0 as isize - 2 - d) as usize..][..L + 4];
        let mut t = [0.0f32; L + 4];
        for i in 0..L + 4 {
            t[i] = (a[i] - b[i]).abs();
        }
        for i in 0..L {
            cost[di][i] = (t[i] + t[i + 1] + t[i + 2] + t[i + 3] + t[i + 4]) * 0.2;
        }
    }
    let c0 = cost[EDI_R as usize];
    // Best direction: strictly better, in the order -4..-1, 1..4 after the
    // vertical; keeps its cost, the half-way direction's cost and the
    // directional averages.
    let mut bc = c0;
    let mut bd = [0.0f32; L];
    let mut bh = [0.0f32; L];
    let mut cv = [[0.0f32; L]; 3];
    for (di, d) in (-EDI_R..=EDI_R).enumerate() {
        if d == 0 {
            continue;
        }
        let pen = 0.002 * d.unsigned_abs() as f32;
        let half = cost[(d / 2 + EDI_R) as usize];
        let df = d as f32;
        let sa = (x0 as isize + d) as usize;
        let sb = (x0 as isize - d) as usize;
        let mut nv = [[0.0f32; L]; 3];
        for p in 0..3 {
            let a = &c[p][sa..sa + L];
            let b = &e[p][sb..sb + L];
            for i in 0..L {
                nv[p][i] = 0.5 * (a[i] + b[i]);
            }
        }
        for i in 0..L {
            let cc = cost[di][i] + pen;
            let better = cc < bc[i];
            bc[i] = if better { cc } else { bc[i] };
            bd[i] = if better { df } else { bd[i] };
            bh[i] = if better { half[i] } else { bh[i] };
            for p in 0..3 {
                cv[p][i] = if better { nv[p][i] } else { cv[p][i] };
            }
        }
    }
    // Confidence (costs are 5-sample means).
    let mut gain = [0.0f32; L];
    for i in 0..L {
        gain[i] = 1.0 - bc[i] / c0[i].max(1e-6);
    }
    let kg = smoothstep8(0.55, 0.85, &gain);
    let kc = smoothstep8(0.03, 0.07, &c0);
    for i in 0..L {
        let ok = (bd[i] != 0.0) & !(bh[i] > c0[i] + 0.004);
        conf[i] = if ok { kg[i] * kc[i] } else { 0.0 };
    }
    o0[..L].copy_from_slice(&cv[0]);
    o1[..L].copy_from_slice(&cv[1]);
    o2[..L].copy_from_slice(&cv[2]);
}

/// `edi_block` with AVX2 intrinsics (same operations in the same order,
/// so the results are identical).
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
unsafe fn edi_block_avx2(c: [&[f32]; 3], e: [&[f32]; 3], x0: usize, conf: &mut [f32], out: [&mut [f32]; 3]) {
    use std::arch::x86_64::*;
    const ND: usize = (2 * EDI_R + 1) as usize;
    // Bounds of every load below (the caller guarantees them).
    assert!(x0 >= 6 && x0 + EDI_L + 6 <= c[0].len().min(e[0].len()));
    assert!(c[1].len() >= c[0].len() && c[2].len() >= c[0].len() && e[1].len() >= e[0].len() && e[2].len() >= e[0].len());
    assert!(conf.len() >= EDI_L && out.iter().all(|o| o.len() >= EDI_L));
    // SAFETY: by the asserts every load reads c/e[x0 - 6 .. x0 + 14] and
    // every store writes [0 .. 8] of its output.
    unsafe {
        let cy = c[0].as_ptr().add(x0);
        let ey = e[0].as_ptr().add(x0);
        let absmask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7fff_ffff));
        let tap = |p: *const f32, q: *const f32, o: isize| _mm256_and_ps(_mm256_sub_ps(_mm256_loadu_ps(p.offset(o)), _mm256_loadu_ps(q.offset(o))), absmask);
        let box_cost = |d: isize| {
            let (p, q) = (cy.offset(d), ey.offset(-d));
            let s = _mm256_add_ps(_mm256_add_ps(_mm256_add_ps(_mm256_add_ps(tap(p, q, -2), tap(p, q, -1)), tap(p, q, 0)), tap(p, q, 1)), tap(p, q, 2));
            _mm256_mul_ps(s, _mm256_set1_ps(0.2))
        };
        let c0 = box_cost(0);
        // Flat block: smoothstep(0.03, 0.07, c0) is 0 everywhere, so is the
        // confidence; the averages are not used.
        if _mm256_movemask_ps(_mm256_cmp_ps::<_CMP_GT_OQ>(c0, _mm256_set1_ps(0.03))) == 0 {
            _mm256_storeu_ps(conf.as_mut_ptr(), _mm256_setzero_ps());
            return;
        }
        let mut cost = [_mm256_setzero_ps(); ND];
        for (di, d) in (-EDI_R..=EDI_R).enumerate() {
            cost[di] = if d == 0 { c0 } else { box_cost(d) };
        }
        let mut bc = c0;
        let mut bd = _mm256_setzero_ps();
        let mut bh = _mm256_setzero_ps();
        let mut cv = [_mm256_setzero_ps(); 3];
        let half_v = _mm256_set1_ps(0.5);
        for (di, d) in (-EDI_R..=EDI_R).enumerate() {
            if d == 0 {
                continue;
            }
            let pen = 0.002 * d.unsigned_abs() as f32;
            let cc = _mm256_add_ps(cost[di], _mm256_set1_ps(pen));
            let better = _mm256_cmp_ps::<_CMP_LT_OQ>(cc, bc);
            bc = _mm256_blendv_ps(bc, cc, better);
            bd = _mm256_blendv_ps(bd, _mm256_set1_ps(d as f32), better);
            bh = _mm256_blendv_ps(bh, cost[(d / 2 + EDI_R) as usize], better);
            for p in 0..3 {
                let a = _mm256_loadu_ps(c[p].as_ptr().add(x0).offset(d));
                let b = _mm256_loadu_ps(e[p].as_ptr().add(x0).offset(-d));
                let n = _mm256_mul_ps(half_v, _mm256_add_ps(a, b));
                cv[p] = _mm256_blendv_ps(cv[p], n, better);
            }
        }
        let one = _mm256_set1_ps(1.0);
        let zero = _mm256_setzero_ps();
        // `smoothstep` (NaN passes through the clamp as in `f32::clamp`).
        let sstep = |e0: f32, e1: f32, x: __m256| {
            let t = _mm256_div_ps(_mm256_sub_ps(x, _mm256_set1_ps(e0)), _mm256_set1_ps(e1 - e0));
            let t = _mm256_min_ps(one, _mm256_max_ps(zero, t));
            _mm256_mul_ps(_mm256_mul_ps(t, t), _mm256_sub_ps(_mm256_set1_ps(3.0), _mm256_mul_ps(_mm256_set1_ps(2.0), t)))
        };
        let gain = _mm256_sub_ps(one, _mm256_div_ps(bc, _mm256_max_ps(c0, _mm256_set1_ps(1e-6))));
        let k = _mm256_mul_ps(sstep(0.55, 0.85, gain), sstep(0.03, 0.07, c0));
        let diag = _mm256_cmp_ps::<_CMP_NEQ_OQ>(bd, zero);
        let bad_half = _mm256_cmp_ps::<_CMP_GT_OQ>(bh, _mm256_add_ps(c0, _mm256_set1_ps(0.004)));
        let ok = _mm256_andnot_ps(bad_half, diag);
        _mm256_storeu_ps(conf.as_mut_ptr(), _mm256_and_ps(ok, k));
        for (p, o) in out.into_iter().enumerate() {
            _mm256_storeu_ps(o.as_mut_ptr(), cv[p]);
        }
    }
}

/// Direction search for one sample with clamped indices (row ends):
/// returns the confidence and the directional Y, U, V averages.
fn edi_pixel(c: [&[f32]; 3], e: [&[f32]; 3], w: usize, x: usize) -> (f32, [f32; 3]) {
    let (cy, ey) = (c[0], e[0]);
    let wi = w as isize;
    let cl = |i: isize| i.clamp(0, wi - 1) as usize;
    let t = |i: usize, d: isize| (cy[cl(i as isize + d)] - ey[cl(i as isize - d)]).abs();
    // The 5-tap box with the end forms of `box5_into`.
    let cost = |d: isize| -> f32 {
        if w < 5 {
            let lo = x.saturating_sub(2);
            let hi = (x + 3).min(w);
            return (lo..hi).map(|i| t(i, d)).sum::<f32>() / (hi - lo) as f32;
        }
        if x == 0 {
            (t(0, d) + t(1, d) + t(2, d)) / 3.0
        } else if x == 1 {
            (t(0, d) + t(1, d) + t(2, d) + t(3, d)) * 0.25
        } else if x == w - 2 {
            (t(w - 4, d) + t(w - 3, d) + t(w - 2, d) + t(w - 1, d)) * 0.25
        } else if x == w - 1 {
            (t(w - 3, d) + t(w - 2, d) + t(w - 1, d)) / 3.0
        } else {
            (t(x - 2, d) + t(x - 1, d) + t(x, d) + t(x + 1, d) + t(x + 2, d)) * 0.2
        }
    };
    let c0 = cost(0);
    let mut bc = c0;
    let mut bd = 0isize;
    for d in (-EDI_R..=EDI_R).filter(|&d| d != 0) {
        let cc = cost(d) + 0.002 * d.unsigned_abs() as f32;
        if cc < bc {
            bc = cc;
            bd = d;
        }
    }
    let xi = x as isize;
    let (xa, xb) = (cl(xi + bd), cl(xi - bd));
    let v = [0.5 * (c[0][xa] + e[0][xb]), 0.5 * (c[1][xa] + e[1][xb]), 0.5 * (c[2][xa] + e[2][xb])];
    if bd == 0 || cost(bd / 2) > c0 + 0.004 {
        return (0.0, v);
    }
    let gain = 1.0 - bc / c0.max(1e-6);
    (smoothstep(0.55, 0.85, gain) * smoothstep(0.03, 0.07, c0), v)
}

// ---------------------------------------------------------------------
// Past-only deinterlacer (streaming capable).

pub struct PastDeinterlacer {
    method: DeinterlaceMethod,
    standard: VideoStandard,
    /// Previous fields, newest first.
    history: VecDeque<Arc<YuvField>>,
    frames: u64,
}

impl PastDeinterlacer {
    pub fn new(method: DeinterlaceMethod, standard: VideoStandard) -> PastDeinterlacer {
        assert!(!method.is_bidirectional(), "past-only deinterlacer");
        PastDeinterlacer {
            method,
            standard,
            history: VecDeque::new(),
            frames: 0,
        }
    }

    /// Previous field `back` fields before `cur` if it is in history.
    fn hist(&self, cur: &YuvField, back: u64) -> Option<&YuvField> {
        let want_parity = if back & 1 == 0 { cur.parity } else { cur.parity.other() };
        self.history
            .iter()
            .find(|h| cur.seq >= back && h.seq == cur.seq - back && h.parity == want_parity)
            .map(|h| h.as_ref())
    }

    /// Renders output rows `rows` of the frame for field `cur` into `out`.
    pub fn render_rows(&self, cur: &YuvField, rows: Range<usize>, out: &mut Frame, parallel: bool) {
        let hl = self.method.history_len();
        let std = self.standard;
        let v = |b: u64| if hl as u64 >= b { self.hist(cur, b).map(|f| View::new(f, std)) } else { None };
        let ctx = Ctx {
            method: self.method,
            cur: View::new(cur, std),
            p: v(1),
            pp: v(2),
            ppp: v(3),
            n: None,
            width: cur.width,
            height: 2 * cur.lines,
            sigma: field_sigma(cur),
            mc: None,
        };
        render(&ctx, rows, out, parallel);
    }

    /// Records a finished field as history.
    pub fn end_field(&mut self, cur: Arc<YuvField>) {
        let hl = self.method.history_len();
        if hl > 0 {
            self.history.push_front(cur);
            self.history.truncate(hl);
        }
    }
}

fn render(ctx: &Ctx, rows: Range<usize>, out: &mut Frame, parallel: bool) {
    let w = out.width;
    let r0 = rows.start;
    let r1 = rows.end;
    let ys = &mut out.y[r0 * w..r1 * w];
    let us = &mut out.u[r0 * w..r1 * w];
    let vs = &mut out.v[r0 * w..r1 * w];
    let ds = &mut out.damage[r0 * w..r1 * w];
    let job = |(i, (((y, u), v), d)): (usize, RowsMut)| ctx.row(r0 + i, (y, u, v, d));
    let meta: Vec<(RxId, LineQuality)> = if parallel {
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
    for (i, (s, q)) in meta.into_iter().enumerate() {
        out.row_source[r0 + i] = s;
        out.row_quality[r0 + i] = q;
    }
}

fn frame_meta(cur: &YuvField, seq: u64, method: DeinterlaceMethod) -> FrameMeta {
    FrameMeta {
        standard: Some(cur.standard),
        seq,
        field_seq: cur.seq,
        parity: Some(cur.parity),
        newest_line_ns: cur.last_ns,
        ready_ns: openvrx_core::clock::now_ns(),
        pipeline: method.name().to_string(),
    }
}

impl Deinterlacer for PastDeinterlacer {
    fn method(&self) -> DeinterlaceMethod {
        self.method
    }

    fn push_field(&mut self, field: Arc<YuvField>) -> Option<Frame> {
        let mut f = Frame::new(field.width, 2 * field.lines);
        self.render_rows(&field, 0..f.height, &mut f, true);
        f.meta = frame_meta(&field, self.frames, self.method);
        self.frames += 1;
        self.end_field(field);
        Some(f)
    }

    fn flush(&mut self) -> Option<Frame> {
        None
    }

    fn reset(&mut self) {
        self.history.clear();
    }
}

// ---------------------------------------------------------------------
// Lookahead deinterlacers (offline).

pub struct BidiDeinterlacer {
    method: DeinterlaceMethod,
    standard: VideoStandard,
    /// Fields newest first: [next, cur, p, pp].
    queue: VecDeque<Arc<YuvField>>,
    frames: u64,
    prev_motion: Option<MotionField>,
}

impl BidiDeinterlacer {
    pub fn new(method: DeinterlaceMethod, standard: VideoStandard) -> BidiDeinterlacer {
        assert!(method.is_bidirectional(), "lookahead deinterlacer");
        BidiDeinterlacer {
            method,
            standard,
            queue: VecDeque::new(),
            frames: 0,
            prev_motion: None,
        }
    }

    fn render_cur(&mut self, cur_idx: usize) -> Option<Frame> {
        let cur = self.queue.get(cur_idx)?.clone();
        let std = self.standard;
        let rel = |back: i64| -> Option<Arc<YuvField>> {
            let want = cur.seq as i64 - back;
            if want < 0 {
                return None;
            }
            let want_parity = if back.rem_euclid(2) == 0 { cur.parity } else { cur.parity.other() };
            self.queue
                .iter()
                .find(|h| h.seq as i64 == want && h.parity == want_parity)
                .cloned()
        };
        let n = rel(-1);
        let p = rel(1);
        let pp = rel(2);
        let mc = if self.method == DeinterlaceMethod::MotionCompensated {
            pp.as_ref().map(|pp| {
                let sp = SearchParams {
                    block_w: 16,
                    block_h: 8,
                    range_x: 16,
                    range_y: 6,
                    lambda: 0.0004,
                    half_pel: true,
                };
                let (w, l) = (cur.width, cur.lines);
                let back = motion::estimate(
                    &Plane::new(&cur.y, w, l),
                    &Plane::new(&pp.y, w, l),
                    &sp,
                    self.prev_motion.as_ref(),
                    true,
                );
                let fwd = match (&n, &p) {
                    (Some(n), Some(p)) => Some(motion::estimate(
                        &Plane::new(&n.y, w, l),
                        &Plane::new(&p.y, w, l),
                        &sp,
                        Some(&back),
                        true,
                    )),
                    _ => None,
                };
                McData { back, fwd }
            })
        } else {
            None
        };
        let ctx = Ctx {
            method: self.method,
            cur: View::new(&cur, std),
            p: p.as_deref().map(|f| View::new(f, std)),
            pp: pp.as_deref().map(|f| View::new(f, std)),
            ppp: None,
            n: n.as_deref().map(|f| View::new(f, std)),
            width: cur.width,
            height: 2 * cur.lines,
            sigma: field_sigma(&cur),
            mc: mc.as_ref(),
        };
        let mut f = Frame::new(cur.width, 2 * cur.lines);
        render(&ctx, 0..f.height, &mut f, true);
        f.meta = frame_meta(&cur, self.frames, self.method);
        self.frames += 1;
        if let Some(m) = mc {
            self.prev_motion = Some(m.back);
        }
        Some(f)
    }
}

impl Deinterlacer for BidiDeinterlacer {
    fn method(&self) -> DeinterlaceMethod {
        self.method
    }

    fn push_field(&mut self, field: Arc<YuvField>) -> Option<Frame> {
        self.queue.push_front(field);
        self.queue.truncate(4);
        if self.queue.len() >= 2 {
            self.render_cur(1)
        } else {
            None
        }
    }

    fn flush(&mut self) -> Option<Frame> {
        if self.queue.is_empty() {
            return None;
        }
        // Render the newest field without lookahead, then forget it.
        let newest = self.queue[0].clone();
        let mut tmp = std::mem::take(&mut self.queue);
        tmp.pop_front();
        self.queue = std::iter::once(newest).chain(tmp).collect();
        let f = self.render_cur(0);
        self.queue.clear();
        f
    }

    fn reset(&mut self) {
        self.queue.clear();
        self.prev_motion = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testgen::{image_to_field, YuvImage};

    /// The AVX2 direction block, the portable block and the per-sample
    /// path agree exactly.
    #[test]
    fn edi_direction_paths_agree() {
        let mut rng = crate::util::Rng::new(7);
        let w = 64;
        let mut rows: Vec<Vec<f32>> = (0..6).map(|_| (0..w).map(|_| rng.uniform() as f32).collect()).collect();
        // Some flat and some edge content.
        let stripe = |x: usize, s: usize| if x < 20 { 0.5 } else if ((x + s) / 3).is_multiple_of(2) { 0.9 } else { 0.1 };
        rows[0] = (0..w).map(|x| stripe(x, 0)).collect();
        rows[3] = (0..w).map(|x| stripe(x, 1)).collect();
        let c = [&rows[0][..], &rows[1][..], &rows[2][..]];
        let e = [&rows[3][..], &rows[4][..], &rows[5][..]];
        for x0 in (6..=w - 14).step_by(4) {
            let mut k = [0.0f32; EDI_L];
            let mut o = [[0.0f32; EDI_L]; 3];
            {
                let [o0, o1, o2] = &mut o;
                edi_block(c, e, x0, &mut k, o0, o1, o2);
            }
            for i in 0..EDI_L {
                let (kp, vp) = edi_pixel(c, e, w, x0 + i);
                assert_eq!(k[i].to_bits(), kp.to_bits(), "x {}", x0 + i);
                if kp > 0.0 {
                    for p in 0..3 {
                        assert_eq!(o[p][i].to_bits(), vp[p].to_bits());
                    }
                }
            }
            #[cfg(target_arch = "x86_64")]
            if crate::util::has_avx2() {
                let mut k2 = [0.0f32; EDI_L];
                let mut o2 = [[0.0f32; EDI_L]; 3];
                {
                    let [a, b, cc] = &mut o2;
                    // SAFETY: AVX2 checked above.
                    unsafe { edi_block_avx2(c, e, x0, &mut k2, [a, b, cc]) };
                }
                for i in 0..EDI_L {
                    assert_eq!(k[i].to_bits(), k2[i].to_bits(), "avx2 x {}", x0 + i);
                    if k[i] > 0.0 {
                        for p in 0..3 {
                            assert_eq!(o[p][i].to_bits(), o2[p][i].to_bits());
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn static_weave_is_exact_and_rows_are_placed() {
        for std in VideoStandard::ALL {
            let p = std.params();
            let mut img = YuvImage::for_standard(std);
            for y in 0..img.height {
                for x in 0..img.width {
                    img.y[y * img.width + x] = ((x * 7 + y * 13) % 17) as f32 / 17.0;
                }
            }
            for m in [DeinterlaceMethod::Weave, DeinterlaceMethod::EdgeDirectedMA] {
                let mut d = create(m, std);
                let mut last = None;
                for seq in 0..4u64 {
                    let (par, _) = crate::testgen::field_info(seq);
                    let f = Arc::new(image_to_field(&img, std, par, seq));
                    last = d.push_field(f);
                }
                let fr = last.unwrap();
                assert_eq!(fr.height, p.frame_height());
                let err = fr.y.iter().zip(&img.y).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
                assert!(err < 1e-6, "{std:?} {m:?} err {err}");
            }
        }
    }
}
