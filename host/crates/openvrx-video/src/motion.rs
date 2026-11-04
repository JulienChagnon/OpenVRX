// SPDX-License-Identifier: GPL-3.0-or-later
//! Block motion estimation shared by the motion-compensated deinterlacer
//! and the motion-compensated temporal denoiser.
//!
//! For each block of the current plane the estimator finds the
//! displacement into the reference plane with the smallest mean absolute
//! difference plus a small vector-length penalty. Search: candidate
//! vectors (zero, the co-located vector of the previous estimate, the
//! three vectors of the block row above), a subsampled coarse grid over
//! the whole range, a diamond refinement at full resolution, and a final
//! half-sample refinement with bilinear interpolation.
//!
//! Rows are estimated in order (the row above feeds candidates) while the
//! blocks of one row are independent, so the result does not depend on
//! how many threads ran or how the rows were batched.

use rayon::prelude::*;

/// Read-only view of one plane.
#[derive(Copy, Clone)]
pub struct Plane<'a> {
    pub data: &'a [f32],
    pub width: usize,
    pub height: usize,
}

impl<'a> Plane<'a> {
    pub fn new(data: &'a [f32], width: usize, height: usize) -> Plane<'a> {
        debug_assert!(data.len() >= width * height);
        Plane { data, width, height }
    }

    #[inline]
    pub fn at(&self, x: isize, y: isize) -> f32 {
        let x = x.clamp(0, self.width as isize - 1) as usize;
        let y = y.clamp(0, self.height as isize - 1) as usize;
        self.data[y * self.width + x]
    }

    /// Bilinear sample at a fractional position (clamped edges).
    #[inline]
    pub fn sample(&self, x: f32, y: f32) -> f32 {
        let xf = x.floor();
        let yf = y.floor();
        let tx = x - xf;
        let ty = y - yf;
        let (xi, yi) = (xf as isize, yf as isize);
        let a = self.at(xi, yi);
        let b = self.at(xi + 1, yi);
        let c = self.at(xi, yi + 1);
        let d = self.at(xi + 1, yi + 1);
        let top = a + (b - a) * tx;
        let bot = c + (d - c) * tx;
        top + (bot - top) * ty
    }
}

/// One block's vector (reference position = block position + vector) and
/// its matching cost (mean absolute difference).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct MotionVector {
    pub dx: f32,
    pub dy: f32,
    pub cost: f32,
}

/// Search settings.
#[derive(Copy, Clone, Debug)]
pub struct SearchParams {
    pub block_w: usize,
    pub block_h: usize,
    pub range_x: i32,
    pub range_y: i32,
    /// Penalty per sample of vector length (in mean-absolute-difference
    /// units), favours smooth, short vectors in flat areas.
    pub lambda: f32,
    pub half_pel: bool,
}

/// A field of block vectors.
#[derive(Clone, Debug, Default)]
pub struct MotionField {
    pub cols: usize,
    pub rows: usize,
    pub block_w: usize,
    pub block_h: usize,
    pub mv: Vec<MotionVector>,
}

impl MotionField {
    pub fn new(width: usize, height: usize, sp: &SearchParams) -> MotionField {
        let cols = width.div_ceil(sp.block_w);
        let rows = height.div_ceil(sp.block_h);
        MotionField {
            cols,
            rows,
            block_w: sp.block_w,
            block_h: sp.block_h,
            mv: vec![MotionVector::default(); cols * rows],
        }
    }

    #[inline]
    pub fn get(&self, col: usize, row: usize) -> MotionVector {
        self.mv[row.min(self.rows - 1) * self.cols + col.min(self.cols - 1)]
    }

    /// Vector at a pixel, bilinearly interpolated between block centres
    /// (smooth compensation without block edges).
    pub fn at_pixel(&self, x: usize, y: usize) -> (f32, f32) {
        let fx = (x as f32 + 0.5) / self.block_w as f32 - 0.5;
        let fy = (y as f32 + 0.5) / self.block_h as f32 - 0.5;
        let cx = fx.floor().max(0.0);
        let cy = fy.floor().max(0.0);
        let tx = (fx - cx).clamp(0.0, 1.0);
        let ty = (fy - cy).clamp(0.0, 1.0);
        let (c0, r0) = (cx as usize, cy as usize);
        let a = self.get(c0, r0);
        let b = self.get(c0 + 1, r0);
        let c = self.get(c0, r0 + 1);
        let d = self.get(c0 + 1, r0 + 1);
        let lerp = |p: f32, q: f32, t: f32| p + (q - p) * t;
        (
            lerp(lerp(a.dx, b.dx, tx), lerp(c.dx, d.dx, tx), ty),
            lerp(lerp(a.dy, b.dy, tx), lerp(c.dy, d.dy, tx), ty),
        )
    }
}

/// Mean absolute difference of a block at an integer displacement, with
/// optional 2:1 subsampling.
#[allow(clippy::too_many_arguments)]
fn sad_int(cur: &Plane, refp: &Plane, bx: usize, by: usize, bw: usize, bh: usize, dx: i32, dy: i32, step: usize) -> f32 {
    let x1 = (bx + bw).min(cur.width);
    let y1 = (by + bh).min(cur.height);
    let rx0 = bx as isize + dx as isize;
    let ry0 = by as isize + dy as isize;
    let inside = rx0 >= 0
        && ry0 >= 0
        && (x1 as isize + dx as isize) <= refp.width as isize
        && (y1 as isize + dy as isize) <= refp.height as isize;
    let mut acc = 0.0f32;
    let mut n = 0usize;
    let mut y = by;
    while y < y1 {
        let crow = &cur.data[y * cur.width..y * cur.width + cur.width];
        if inside {
            let ry = (y as isize + dy as isize) as usize;
            let rrow = &refp.data[ry * refp.width..ry * refp.width + refp.width];
            let mut x = bx;
            while x < x1 {
                acc += (crow[x] - rrow[(x as isize + dx as isize) as usize]).abs();
                n += 1;
                x += step;
            }
        } else {
            let mut x = bx;
            while x < x1 {
                acc += (crow[x] - refp.at(x as isize + dx as isize, y as isize + dy as isize)).abs();
                n += 1;
                x += step;
            }
        }
        y += step;
    }
    acc / n.max(1) as f32
}

#[allow(clippy::too_many_arguments)]
fn sad_frac(cur: &Plane, refp: &Plane, bx: usize, by: usize, bw: usize, bh: usize, dx: f32, dy: f32) -> f32 {
    let x1 = (bx + bw).min(cur.width);
    let y1 = (by + bh).min(cur.height);
    let mut acc = 0.0f32;
    let mut n = 0usize;
    for y in by..y1 {
        for x in bx..x1 {
            acc += (cur.data[y * cur.width + x] - refp.sample(x as f32 + dx, y as f32 + dy)).abs();
            n += 1;
        }
    }
    acc / n.max(1) as f32
}

/// Estimates one block.
fn estimate_block(cur: &Plane, refp: &Plane, sp: &SearchParams, col: usize, row: usize, cands: &[(i32, i32)]) -> MotionVector {
    let bx = col * sp.block_w;
    let by = row * sp.block_h;
    let (bw, bh) = (sp.block_w, sp.block_h);
    let pen = |dx: f32, dy: f32| sp.lambda * (dx.abs() + dy.abs());
    let cost_int = |dx: i32, dy: i32, step: usize| sad_int(cur, refp, bx, by, bw, bh, dx, dy, step) + pen(dx as f32, dy as f32);
    let clampv = |dx: i32, dy: i32| (dx.clamp(-sp.range_x, sp.range_x), dy.clamp(-sp.range_y, sp.range_y));
    let mut best = (0i32, 0i32);
    let mut best_cost = cost_int(0, 0, 1);
    for &(cx, cy) in cands {
        let (cx, cy) = clampv(cx, cy);
        if (cx, cy) == (0, 0) {
            continue;
        }
        let c = cost_int(cx, cy, 1);
        if c < best_cost {
            best_cost = c;
            best = (cx, cy);
        }
    }
    // Coarse grid, subsampled.
    let gx = (sp.range_x / 4).max(1);
    let gy = (sp.range_y / 4).max(1);
    let mut coarse_best = best;
    let mut coarse_cost = f32::INFINITY;
    let mut ty = -sp.range_y;
    while ty <= sp.range_y {
        let mut tx = -sp.range_x;
        while tx <= sp.range_x {
            let c = cost_int(tx, ty, 2);
            if c < coarse_cost {
                coarse_cost = c;
                coarse_best = (tx, ty);
            }
            tx += gx;
        }
        ty += gy;
    }
    // Refine around both the candidate winner and the coarse winner.
    for start in [best, coarse_best] {
        let mut pos = start;
        let mut cost = cost_int(pos.0, pos.1, 1);
        for step in [gx.max(gy).max(2) / 2, 1, 1] {
            let step = step.max(1);
            loop {
                let mut improved = false;
                for (ox, oy) in [(step, 0), (-step, 0), (0, step), (0, -step), (step, step), (-step, -step), (step, -step), (-step, step)] {
                    let (nx, ny) = clampv(pos.0 + ox, pos.1 + oy);
                    if (nx, ny) == pos {
                        continue;
                    }
                    let c = cost_int(nx, ny, 1);
                    if c < cost {
                        cost = c;
                        pos = (nx, ny);
                        improved = true;
                    }
                }
                if !improved {
                    break;
                }
            }
        }
        if cost < best_cost {
            best_cost = cost;
            best = pos;
        }
    }
    let mut mv = MotionVector {
        dx: best.0 as f32,
        dy: best.1 as f32,
        cost: best_cost,
    };
    if sp.half_pel {
        let (bx0, by0) = (mv.dx, mv.dy);
        for (ox, oy) in [(0.5f32, 0.0f32), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5), (0.5, 0.5), (-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5)] {
            let (nx, ny) = (bx0 + ox, by0 + oy);
            let c = sad_frac(cur, refp, bx, by, bw, bh, nx, ny) + pen(nx, ny);
            if c < mv.cost {
                mv = MotionVector { dx: nx, dy: ny, cost: c };
            }
        }
    }
    mv.cost -= pen(mv.dx, mv.dy);
    mv
}

/// Estimates one row of blocks given the row above (already estimated in
/// `field`) and an optional previous motion field.
pub fn estimate_row(cur: &Plane, refp: &Plane, sp: &SearchParams, prev: Option<&MotionField>, field: &mut MotionField, row: usize, parallel: bool) {
    let cols = field.cols;
    let above: Vec<MotionVector> = if row > 0 {
        field.mv[(row - 1) * cols..row * cols].to_vec()
    } else {
        Vec::new()
    };
    let job = |col: usize| -> MotionVector {
        let mut cands: Vec<(i32, i32)> = Vec::with_capacity(6);
        if let Some(p) = prev {
            if p.cols == cols && p.rows == field.rows {
                let v = p.get(col, row);
                cands.push((v.dx.round() as i32, v.dy.round() as i32));
            }
        }
        if !above.is_empty() {
            for c in [col.saturating_sub(1), col, (col + 1).min(cols - 1)] {
                let v = above[c];
                cands.push((v.dx.round() as i32, v.dy.round() as i32));
            }
        }
        estimate_block(cur, refp, sp, col, row, &cands)
    };
    let out = &mut field.mv[row * cols..(row + 1) * cols];
    if parallel {
        let v: Vec<MotionVector> = (0..cols).into_par_iter().map(job).collect();
        out.copy_from_slice(&v);
    } else {
        for (col, o) in out.iter_mut().enumerate() {
            *o = job(col);
        }
    }
}

/// Estimates a whole motion field.
pub fn estimate(cur: &Plane, refp: &Plane, sp: &SearchParams, prev: Option<&MotionField>, parallel: bool) -> MotionField {
    let mut f = MotionField::new(cur.width, cur.height, sp);
    for row in 0..f.rows {
        estimate_row(cur, refp, sp, prev, &mut f, row, parallel);
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_shift() {
        let (w, h) = (160usize, 96usize);
        let tex = |x: f32, y: f32| ((x * 0.31).sin() * (y * 0.23).cos() + (x * 0.07 + y * 0.11).sin()) * 0.5;
        let refd: Vec<f32> = (0..w * h).map(|i| tex((i % w) as f32, (i / w) as f32)).collect();
        // Current = reference shifted: cur(x, y) = ref(x + 5.5, y - 3).
        let curd: Vec<f32> = (0..w * h).map(|i| tex((i % w) as f32 + 5.5, (i / w) as f32 - 3.0)).collect();
        let sp = SearchParams {
            block_w: 16,
            block_h: 16,
            range_x: 12,
            range_y: 8,
            lambda: 0.0005,
            half_pel: true,
        };
        let f = estimate(&Plane::new(&curd, w, h), &Plane::new(&refd, w, h), &sp, None, true);
        for r in 1..f.rows - 1 {
            for c in 1..f.cols - 1 {
                let v = f.get(c, r);
                assert!((v.dx - 5.5).abs() < 0.01 && (v.dy + 3.0).abs() < 0.01, "{c},{r}: {v:?}");
            }
        }
    }
}
