// SPDX-License-Identifier: GPL-3.0-or-later
//! Motion estimation: confidence-weighted hierarchical block matching with
//! sub-pixel refinement, interpolated to a dense per-pixel field.
//!
//! The estimate answers "where is this pixel of picture A in picture B":
//! `A(x) ~ B(x + F(x))`. Matching cost per block is a mean absolute
//! difference weighted by A's confidence, so damaged pixels of A take no
//! part in the match; damaged pixels of B cost a fixed penalty so vectors
//! are not drawn into broken areas. Coarse to fine: each level starts from
//! the parent vectors of the block and its neighbours, searches a small
//! window around the best of them, and the field is cleaned with a 3x3
//! vector median. The finest level adds a parabolic sub-pixel fit on the
//! squared-error surface.

use crate::picture::{blur121, downsample2, sample_bilinear};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

/// Block matching parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlowParams {
    /// Block size in pixels at every pyramid level.
    pub block: usize,
    /// Full search radius at the coarsest level (pixels of that level).
    pub coarse_search: i32,
    /// Search radius around the best predictor on finer levels.
    pub refine_search: i32,
    /// Most pyramid levels used (1 = full resolution only).
    pub max_levels: usize,
}

impl Default for FlowParams {
    fn default() -> Self {
        FlowParams {
            block: 8,
            coarse_search: 6,
            refine_search: 2,
            max_levels: 5,
        }
    }
}

/// Dense motion field.
#[derive(Clone, Debug)]
pub struct Flow {
    pub width: usize,
    pub height: usize,
    pub dx: Vec<f32>,
    pub dy: Vec<f32>,
}

impl Flow {
    pub fn zero(width: usize, height: usize) -> Flow {
        Flow {
            width,
            height,
            dx: vec![0.0; width * height],
            dy: vec![0.0; width * height],
        }
    }

    /// Bilinear lookup of the vector at a fractional position.
    #[inline]
    pub fn at(&self, x: f32, y: f32) -> (f32, f32) {
        (
            sample_bilinear(&self.dx, self.width, self.height, x, y),
            sample_bilinear(&self.dy, self.width, self.height, x, y),
        )
    }
}

/// Matching penalty for landing on an untrusted pixel of B (luma units).
const BAD_REF_PENALTY: f32 = 0.12;
/// Cost per pixel of departing from the predictor (keeps flat areas calm).
const SMOOTHNESS: f32 = 0.0015;

struct Level {
    w: usize,
    h: usize,
    a: Vec<f32>,
    ca: Vec<f32>,
    b: Vec<f32>,
    cb: Vec<f32>,
}

#[derive(Copy, Clone, Debug, Default)]
struct Vector {
    x: f32,
    y: f32,
    valid: bool,
}

/// Estimates the motion from `a` (with confidence `ca`) into `b`.
pub fn estimate(
    a: &[f32],
    ca: &[f32],
    b: &[f32],
    cb: &[f32],
    w: usize,
    h: usize,
    params: &FlowParams,
) -> Flow {
    let bs = params.block.max(4);
    let mut levels = vec![Level {
        w,
        h,
        a: blur121(a, w, h),
        ca: ca.to_vec(),
        b: blur121(b, w, h),
        cb: cb.to_vec(),
    }];
    while levels.len() < params.max_levels.max(1) {
        let top = levels.last().expect("non-empty");
        if top.w.min(top.h) / 2 < 3 * bs {
            break;
        }
        let (a, nw, nh) = downsample2(&top.a, top.w, top.h);
        let (ca, _, _) = downsample2(&top.ca, top.w, top.h);
        let (b, _, _) = downsample2(&top.b, top.w, top.h);
        let (cb, _, _) = downsample2(&top.cb, top.w, top.h);
        levels.push(Level {
            w: nw,
            h: nh,
            a,
            ca,
            b,
            cb,
        });
    }

    let mut parent: Option<(Vec<Vector>, usize, usize)> = None;
    let nlev = levels.len();
    for (li, lv) in levels.iter().enumerate().rev() {
        let nbx = lv.w.div_ceil(bs);
        let nby = lv.h.div_ceil(bs);
        let finest = li == 0;
        let coarsest = li == nlev - 1;
        let radius = if coarsest {
            params.coarse_search
        } else {
            params.refine_search
        };
        let par = parent.as_ref();
        let mut field: Vec<Vector> = (0..nbx * nby)
            .into_par_iter()
            .map(|k| {
                let bx = k % nbx;
                let by = k / nbx;
                let x0 = bx * bs;
                let y0 = by * bs;
                let x1 = (x0 + bs).min(lv.w);
                let y1 = (y0 + bs).min(lv.h);
                let mut wsum = 0.0;
                for y in y0..y1 {
                    for x in x0..x1 {
                        wsum += lv.ca[y * lv.w + x];
                    }
                }
                let parent_vec = |pbx: isize, pby: isize| -> Option<(f32, f32)> {
                    let (pf, pnx, pny) = par?;
                    let pbx = pbx.clamp(0, *pnx as isize - 1) as usize;
                    let pby = pby.clamp(0, *pny as isize - 1) as usize;
                    let v = pf[pby * pnx + pbx];
                    Some((2.0 * v.x, 2.0 * v.y))
                };
                // Parent block covering this block's centre.
                let cx = ((x0 + x1) / 2 / 2 / bs) as isize;
                let cy = ((y0 + y1) / 2 / 2 / bs) as isize;
                let pred = parent_vec(cx, cy).unwrap_or((0.0, 0.0));
                if wsum < 0.15 * (bs * bs) as f32 {
                    return Vector {
                        x: pred.0,
                        y: pred.1,
                        valid: false,
                    };
                }
                let cost = |vx: f32, vy: f32| {
                    block_cost(lv, x0, y0, x1, y1, vx, vy, false)
                        + SMOOTHNESS * ((vx - pred.0).abs() + (vy - pred.1).abs())
                };
                let mut cands = vec![pred, (0.0, 0.0)];
                for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                    if let Some(v) = parent_vec(cx + dx, cy + dy) {
                        cands.push(v);
                    }
                }
                let mut best = (pred.0.round(), pred.1.round());
                let mut best_cost = f32::INFINITY;
                for c in cands {
                    let c = (c.0.round(), c.1.round());
                    let k = cost(c.0, c.1);
                    if k < best_cost {
                        best_cost = k;
                        best = c;
                    }
                }
                let centre = best;
                for dy in -radius..=radius {
                    for dx in -radius..=radius {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let c = (centre.0 + dx as f32, centre.1 + dy as f32);
                        let k = cost(c.0, c.1);
                        if k < best_cost {
                            best_cost = k;
                            best = c;
                        }
                    }
                }
                let (mut vx, mut vy) = best;
                if finest {
                    let sq = |x: f32, y: f32| block_cost(lv, x0, y0, x1, y1, x, y, true);
                    let c0 = sq(vx, vy);
                    let off = |cm: f32, cp: f32| {
                        let den = cm - 2.0 * c0 + cp;
                        if den > 1e-9 {
                            (0.5 * (cm - cp) / den).clamp(-0.5, 0.5)
                        } else {
                            0.0
                        }
                    };
                    let ox = off(sq(vx - 1.0, vy), sq(vx + 1.0, vy));
                    let oy = off(sq(vx, vy - 1.0), sq(vx, vy + 1.0));
                    vx += ox;
                    vy += oy;
                }
                Vector {
                    x: vx,
                    y: vy,
                    valid: true,
                }
            })
            .collect();
        field = vector_median(&field, nbx, nby);
        parent = Some((field, nbx, nby));
    }

    let (field, nbx, nby) = parent.expect("at least one level");
    densify(&field, nbx, nby, bs, w, h)
}

#[allow(clippy::too_many_arguments)]
#[inline]
fn block_cost(
    lv: &Level,
    x0: usize,
    y0: usize,
    x1: usize,
    y1: usize,
    vx: f32,
    vy: f32,
    squared: bool,
) -> f32 {
    let w = lv.w;
    let integer = vx.fract() == 0.0 && vy.fract() == 0.0;
    let (ix, iy) = (vx as isize, vy as isize);
    let mut acc = 0.0;
    let mut wsum = 0.0;
    for y in y0..y1 {
        for x in x0..x1 {
            let wa = lv.ca[y * w + x];
            if wa <= 0.0 {
                continue;
            }
            let (bv, cbv, inside) = if integer {
                let px = x as isize + ix;
                let py = y as isize + iy;
                if px < 0 || py < 0 || px >= w as isize || py >= lv.h as isize {
                    (0.0, 0.0, false)
                } else {
                    let i = py as usize * w + px as usize;
                    (lv.b[i], lv.cb[i], true)
                }
            } else {
                let px = x as f32 + vx;
                let py = y as f32 + vy;
                if px < 0.0 || py < 0.0 || px > (w - 1) as f32 || py > (lv.h - 1) as f32 {
                    (0.0, 0.0, false)
                } else {
                    (
                        sample_bilinear(&lv.b, w, lv.h, px, py),
                        sample_bilinear(&lv.cb, w, lv.h, px, py),
                        true,
                    )
                }
            };
            let e = if inside {
                let d = (lv.a[y * w + x] - bv).abs();
                let d = if squared { d * d } else { d };
                let pen = if squared {
                    BAD_REF_PENALTY * BAD_REF_PENALTY
                } else {
                    BAD_REF_PENALTY
                };
                cbv * d + (1.0 - cbv) * pen
            } else if squared {
                4.0 * BAD_REF_PENALTY * BAD_REF_PENALTY
            } else {
                2.0 * BAD_REF_PENALTY
            };
            acc += wa * e;
            wsum += wa;
        }
    }
    if wsum > 0.0 {
        acc / wsum
    } else {
        f32::INFINITY
    }
}

/// 3x3 vector median: each block takes the valid neighbour vector with the
/// smallest summed L1 distance to the other valid neighbours.
fn vector_median(field: &[Vector], nbx: usize, nby: usize) -> Vec<Vector> {
    (0..nbx * nby)
        .into_par_iter()
        .map(|k| {
            let bx = (k % nbx) as isize;
            let by = (k / nbx) as isize;
            let mut set = [Vector::default(); 9];
            let mut n = 0;
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let x = bx + dx;
                    let y = by + dy;
                    if x < 0 || y < 0 || x >= nbx as isize || y >= nby as isize {
                        continue;
                    }
                    let v = field[y as usize * nbx + x as usize];
                    if v.valid {
                        set[n] = v;
                        n += 1;
                    }
                }
            }
            if n == 0 {
                return field[k];
            }
            let mut best = set[0];
            let mut best_d = f32::INFINITY;
            for i in 0..n {
                let mut d = 0.0;
                for j in 0..n {
                    d += (set[i].x - set[j].x).abs() + (set[i].y - set[j].y).abs();
                }
                if d < best_d {
                    best_d = d;
                    best = set[i];
                }
            }
            Vector {
                x: best.x,
                y: best.y,
                valid: field[k].valid || n >= 3,
            }
        })
        .collect()
}

/// Bilinear interpolation of block-centre vectors to every pixel.
fn densify(field: &[Vector], nbx: usize, nby: usize, bs: usize, w: usize, h: usize) -> Flow {
    let mut flow = Flow::zero(w, h);
    let half = 0.5 * bs as f32 - 0.5;
    flow.dx
        .par_chunks_mut(w)
        .zip(flow.dy.par_chunks_mut(w))
        .enumerate()
        .for_each(|(y, (rx, ry))| {
            let fy = ((y as f32 - half) / bs as f32).clamp(0.0, (nby - 1) as f32);
            let y0 = fy.floor() as usize;
            let y1 = (y0 + 1).min(nby - 1);
            let ty = fy - y0 as f32;
            for x in 0..w {
                let fx = ((x as f32 - half) / bs as f32).clamp(0.0, (nbx - 1) as f32);
                let x0 = fx.floor() as usize;
                let x1 = (x0 + 1).min(nbx - 1);
                let tx = fx - x0 as f32;
                let a = field[y0 * nbx + x0];
                let b = field[y0 * nbx + x1];
                let c = field[y1 * nbx + x0];
                let d = field[y1 * nbx + x1];
                let lerp = |p: f32, q: f32, r: f32, s: f32| {
                    let t = p + (q - p) * tx;
                    let u = r + (s - r) * tx;
                    t + (u - t) * ty
                };
                rx[x] = lerp(a.x, b.x, c.x, d.x);
                ry[x] = lerp(a.y, b.y, c.y, d.y);
            }
        });
    flow
}

/// Forward/backward consistency in 0..1 for each pixel of the forward
/// field: 1 where following the forward vector and then the backward one
/// returns to the start, falling off with the round-trip error (`sigma`
/// pixels). Low values mark occlusions and unreliable vectors.
pub fn consistency(fwd: &Flow, bwd: &Flow, sigma: f32) -> Vec<f32> {
    let w = fwd.width;
    let inv = 1.0 / (2.0 * sigma * sigma);
    let mut out = vec![0.0; w * fwd.height];
    out.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        for (x, o) in row.iter_mut().enumerate() {
            let i = y * w + x;
            let (fx, fy) = (fwd.dx[i], fwd.dy[i]);
            let (bx, by) = bwd.at(x as f32 + fx, y as f32 + fy);
            let ex = fx + bx;
            let ey = fy + by;
            *o = (-(ex * ex + ey * ey) * inv).exp();
        }
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn texture(w: usize, h: usize, ox: f32, oy: f32) -> Vec<f32> {
        (0..w * h)
            .map(|k| {
                let x = (k % w) as f32 + ox;
                let y = (k / w) as f32 + oy;
                0.5 + 0.2 * (x * 0.21).sin() * (y * 0.17).cos()
                    + 0.15 * ((x * 0.05 + y * 0.09).sin())
                    + 0.1 * ((x * 0.37 - y * 0.11).cos())
            })
            .collect()
    }

    #[test]
    fn recovers_subpixel_translation() {
        let (w, h) = (192, 144);
        // a(x) = f(x + 0), b(x) = f(x - d) so a(x) = b(x + d).
        let (dx, dy) = (7.25f32, -3.5f32);
        let a = texture(w, h, 0.0, 0.0);
        let b = texture(w, h, -dx, -dy);
        let c = vec![1.0; w * h];
        let f = estimate(&a, &c, &b, &c, w, h, &FlowParams::default());
        let mut ex = 0.0;
        let mut n = 0.0;
        for y in 20..h - 20 {
            for x in 20..w - 20 {
                ex += ((f.dx[y * w + x] - dx).abs() + (f.dy[y * w + x] - dy).abs()) as f64;
                n += 1.0;
            }
        }
        let mean_err = ex / n;
        assert!(mean_err < 0.3, "mean endpoint error {mean_err}");
    }

    #[test]
    fn ignores_damaged_rows_in_target() {
        let (w, h) = (192, 144);
        let (dx, dy) = (-4.0f32, 2.0f32);
        let mut a = texture(w, h, 0.0, 0.0);
        let b = texture(w, h, -dx, -dy);
        let mut ca = vec![1.0; w * h];
        for y in 60..72 {
            for x in 0..w {
                a[y * w + x] = if (x * 7 + y * 13) % 5 < 2 { 1.0 } else { 0.0 };
                ca[y * w + x] = 0.0;
            }
        }
        let cb = vec![1.0; w * h];
        let f = estimate(&a, &ca, &b, &cb, w, h, &FlowParams::default());
        for y in 60..72 {
            for x in 30..w - 30 {
                let e = (f.dx[y * w + x] - dx).abs() + (f.dy[y * w + x] - dy).abs();
                assert!(e < 0.75, "vector at ({x},{y}) off by {e}");
            }
        }
        let back = estimate(&b, &cb, &a, &ca, w, h, &FlowParams::default());
        let cons = consistency(&f, &back, 1.0);
        assert!(cons[30 * w + 90] > 0.8);
    }
}
