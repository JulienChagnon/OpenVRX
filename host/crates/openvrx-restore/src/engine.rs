// SPDX-License-Identifier: GPL-3.0-or-later
//! Streaming reconstruction engine.
//!
//! Pictures are pushed in display order and come out in the same order
//! once enough look-ahead has arrived. For each picture:
//!
//! 1. Normal picture: motion is estimated from it to up to `radius`
//!    neighbours on each side (same group, i.e. same field parity, and not
//!    broken). Every flagged pixel (confidence below 1) is replaced by
//!    `c * original + (1 - c) * estimate`, where the estimate is a Huber
//!    mean (initialised at the weighted median) of the motion-compensated
//!    neighbour samples weighted by their own confidence, the
//!    forward/backward flow agreement and temporal distance. Where the
//!    neighbours hold too little trusted weight, a vertical fill from the
//!    nearest trusted rows makes up the difference. Trusted pixels are left
//!    exactly as they were unless `denoise > 0`, which averages them with
//!    neighbour samples that match their 3x3 neighbourhood (non-local means
//!    weights on the aligned stack).
//! 2. Broken picture (mean confidence under `broken_threshold`): it is
//!    rebuilt by motion-compensated interpolation between the nearest good
//!    pictures before and after it (at most `max_gap` steps away), or held
//!    from the one side available.
//!
//! Memory is bounded: the window holds `radius` pictures behind the
//! current one, `max(radius, max_gap) + 1` ahead, and the last good
//! picture of each group.

use crate::config::RestoreConfig;
use crate::flow::{consistency, estimate, Flow};
use crate::picture::{box_mean, estimate_noise, sample_bicubic, sample_bilinear, Picture};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

/// Minimum summed neighbour weight for a fully temporal rebuild.
const W_MIN: f32 = 0.3;
/// Rows searched above and below for the spatial fill.
const FILL_REACH: usize = 48;

/// What happened to one picture.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PictureStats {
    /// Input order index.
    pub index: usize,
    /// Fraction of pixels that were flagged (confidence below 1).
    pub flagged: f32,
    /// Fraction of pixels whose rebuild had full neighbour support.
    pub rebuilt: f32,
    /// Fraction that needed the spatial fill.
    pub spatial_fill: f32,
    /// The picture was broken and rebuilt by interpolation or hold.
    pub interpolated: bool,
    /// Broken picture with no good picture in reach: left as it was.
    pub unrecoverable: bool,
    /// Neighbours used for the rebuild.
    pub neighbours: usize,
    /// Measured luma noise sigma.
    pub noise_sigma: f32,
}

struct Entry {
    index: usize,
    group: u8,
    pic: Picture,
    sigma: f32,
    broken: bool,
}

/// Streaming restorer core over `Picture`s.
pub struct Engine {
    cfg: RestoreConfig,
    groups: usize,
    window: VecDeque<Arc<Entry>>,
    next_in: usize,
    next_out: usize,
    last_good: HashMap<u8, Arc<Entry>>,
    flows: HashMap<(usize, usize), Arc<Flow>>,
    cons: HashMap<(usize, usize), Arc<Vec<f32>>>,
}

impl Engine {
    /// `groups` is 1 for progressive frames and 2 for alternating fields
    /// (neighbours are only taken from the same group).
    pub fn new(cfg: RestoreConfig, groups: usize) -> Engine {
        Engine {
            cfg,
            groups: groups.max(1),
            window: VecDeque::new(),
            next_in: 0,
            next_out: 0,
            last_good: HashMap::new(),
            flows: HashMap::new(),
            cons: HashMap::new(),
        }
    }

    fn ahead(&self) -> usize {
        (self.cfg.radius.max(self.cfg.max_gap) + 1) * self.groups
    }

    /// Adds the next picture; returns every picture that became ready.
    pub fn push(&mut self, pic: Picture, group: u8) -> Vec<(Picture, PictureStats)> {
        let sigma = estimate_noise(&pic.y, &pic.conf, pic.width, pic.height);
        let broken = pic.mean_conf() < self.cfg.broken_threshold;
        self.window.push_back(Arc::new(Entry {
            index: self.next_in,
            group,
            pic,
            sigma,
            broken,
        }));
        self.next_in += 1;
        let mut out = Vec::new();
        while self.next_out + self.ahead() < self.next_in {
            out.push(self.process_next());
        }
        out
    }

    /// Flushes the remaining pictures.
    pub fn finish(&mut self) -> Vec<(Picture, PictureStats)> {
        let mut out = Vec::new();
        while self.next_out < self.next_in {
            out.push(self.process_next());
        }
        out
    }

    fn get(&self, index: usize) -> Option<&Arc<Entry>> {
        let front = self.window.front()?.index;
        if index < front {
            return self.last_good.values().find(|e| e.index == index);
        }
        self.window.get(index - front)
    }

    fn process_next(&mut self) -> (Picture, PictureStats) {
        let idx = self.next_out;
        let cur = self.get(idx).expect("current picture in window").clone();
        let result = if cur.broken {
            self.rebuild_broken(&cur)
        } else {
            self.rebuild_normal(&cur)
        };
        if !cur.broken {
            self.last_good.insert(cur.group, cur.clone());
        }
        self.next_out += 1;
        self.evict();
        result
    }

    fn evict(&mut self) {
        let keep_from = self.next_out.saturating_sub(self.cfg.radius * self.groups);
        while self.window.front().is_some_and(|e| e.index < keep_from) {
            self.window.pop_front();
        }
        let good: Vec<usize> = self.last_good.values().map(|e| e.index).collect();
        let alive = |i: usize| i >= keep_from || good.contains(&i);
        self.flows.retain(|&(a, b), _| alive(a) && alive(b));
        self.cons.retain(|&(a, b), _| alive(a) && alive(b));
    }

    /// Flow from picture `a` into picture `b`, cached.
    fn flow(&mut self, a: &Entry, b: &Entry) -> Arc<Flow> {
        if let Some(f) = self.flows.get(&(a.index, b.index)) {
            return f.clone();
        }
        let f = Arc::new(compute_flow(&self.cfg, a, b));
        self.flows.insert((a.index, b.index), f.clone());
        f
    }

    /// Computes (in parallel) every missing flow for the given pairs.
    fn ensure_flows(&mut self, pairs: &[(Arc<Entry>, Arc<Entry>)]) {
        let missing: Vec<&(Arc<Entry>, Arc<Entry>)> = pairs
            .iter()
            .filter(|(a, b)| !self.flows.contains_key(&(a.index, b.index)))
            .collect();
        let cfg = &self.cfg;
        let done: Vec<((usize, usize), Flow)> = missing
            .par_iter()
            .map(|(a, b)| ((a.index, b.index), compute_flow(cfg, a, b)))
            .collect();
        for (k, f) in done {
            self.flows.insert(k, Arc::new(f));
        }
    }

    fn consistency_map(&mut self, a: &Entry, b: &Entry) -> Option<Arc<Vec<f32>>> {
        if !self.cfg.consistency {
            return None;
        }
        if let Some(c) = self.cons.get(&(a.index, b.index)) {
            return Some(c.clone());
        }
        let f = self.flow(a, b);
        let g = self.flow(b, a);
        let c = Arc::new(consistency(&f, &g, 1.0));
        self.cons.insert((a.index, b.index), c.clone());
        Some(c)
    }

    fn neighbours(&self, cur: &Entry) -> Vec<Arc<Entry>> {
        let r = self.cfg.radius;
        let mut out = Vec::new();
        for dir in [-1isize, 1] {
            let mut found = 0;
            let mut k = 1;
            while found < r && k <= r * self.groups * 4 {
                let i = cur.index as isize + dir * k as isize;
                k += 1;
                if i < 0 {
                    break;
                }
                let Some(e) = self.window_only(i as usize) else {
                    break;
                };
                if e.group != cur.group {
                    continue;
                }
                found += 1;
                if !e.broken {
                    out.push(e.clone());
                }
            }
        }
        out
    }

    fn window_only(&self, index: usize) -> Option<&Arc<Entry>> {
        let front = self.window.front()?.index;
        if index < front {
            return None;
        }
        self.window.get(index - front)
    }

    fn rebuild_normal(&mut self, cur: &Arc<Entry>) -> (Picture, PictureStats) {
        let pic = &cur.pic;
        let (w, h) = (pic.width, pic.height);
        let n = w * h;
        let flagged = pic.conf.iter().filter(|&&c| c < 1.0).count();
        let mut stats = PictureStats {
            index: cur.index,
            flagged: flagged as f32 / n as f32,
            noise_sigma: cur.sigma,
            ..PictureStats::default()
        };
        let denoise = self.cfg.denoise > 0.0;
        if flagged == 0 && !denoise {
            return (pic.clone(), stats);
        }
        let neigh = self.neighbours(cur);
        stats.neighbours = neigh.len();
        let mut pairs: Vec<(Arc<Entry>, Arc<Entry>)> =
            neigh.iter().map(|e| (cur.clone(), e.clone())).collect();
        if self.cfg.consistency {
            pairs.extend(neigh.iter().map(|e| (e.clone(), cur.clone())));
        }
        self.ensure_flows(&pairs);
        let sources: Vec<Warped> = neigh
            .iter()
            .map(|e| {
                let f = self.flow(cur, e);
                let c = self.consistency_map(cur, e);
                let d = cur.index.abs_diff(e.index).div_ceil(self.groups);
                (e.clone(), f, c, d)
            })
            .collect::<Vec<_>>()
            .into_par_iter()
            .map(|(e, f, c, d)| warp(&e.pic, &f, c.as_ref().map(|v| v.as_slice()), d))
            .collect();

        let patch: Vec<Vec<f32>> = if denoise {
            sources
                .par_iter()
                .map(|s| {
                    let d2: Vec<f32> = s.y.iter().zip(&pic.y).map(|(a, b)| (a - b) * (a - b)).collect();
                    box_mean(&d2, w, h, 1)
                })
                .collect()
        } else {
            Vec::new()
        };

        let sigma = cur.sigma.max(0.002);
        let delta = (3.0 * sigma).max(0.04);
        let h2 = (self.cfg.denoise * sigma).powi(2) * 4.0 + 1e-7;
        let two_s2 = 2.0 * sigma * sigma;
        let mut out = pic.clone();
        let counts: (usize, usize) = out
            .y
            .par_chunks_mut(w)
            .zip(out.u.par_chunks_mut(w))
            .zip(out.v.par_chunks_mut(w))
            .zip(out.conf.par_chunks_mut(w))
            .enumerate()
            .map(|(yy, (((ry, ru), rv), rc))| {
                let mut rebuilt = 0;
                let mut filled = 0;
                let mut ws = Vec::with_capacity(sources.len());
                for x in 0..w {
                    let i = yy * w + x;
                    let c = pic.conf[i];
                    if c >= 1.0 {
                        if denoise {
                            let (mut sy, mut su, mut sv, mut sw) = (pic.y[i], pic.u[i], pic.v[i], 1.0);
                            for (s, d) in sources.iter().zip(&patch) {
                                let a = s.w[i] * (-(d[i] - two_s2).max(0.0) / h2).exp();
                                sy += a * s.y[i];
                                su += a * s.u[i];
                                sv += a * s.v[i];
                                sw += a;
                            }
                            ry[x] = sy / sw;
                            ru[x] = su / sw;
                            rv[x] = sv / sw;
                        }
                        continue;
                    }
                    ws.clear();
                    let mut wsum = 0.0;
                    for (k, s) in sources.iter().enumerate() {
                        if s.w[i] > 0.01 {
                            ws.push((s.y[i], s.w[i], k));
                            wsum += s.w[i];
                        }
                    }
                    let mut est = None;
                    if wsum > 0.0 {
                        let m = weighted_median(&mut ws);
                        let (mut ey, mut eu, mut ev, mut ew) = (0.0, 0.0, 0.0, 0.0);
                        for &(val, wt, k) in ws.iter() {
                            let hub = wt * (delta / (val - m).abs().max(delta));
                            ey += hub * val;
                            eu += hub * sources[k].u[i];
                            ev += hub * sources[k].v[i];
                            ew += hub;
                        }
                        est = Some((ey / ew, eu / ew, ev / ew));
                    }
                    let (ey, eu, ev) = if wsum >= W_MIN {
                        rebuilt += 1;
                        est.expect("weights present")
                    } else {
                        filled += 1;
                        let (fy, fu, fv) = spatial_fill(pic, x, yy);
                        match est {
                            Some((ey, eu, ev)) => {
                                let a = wsum / W_MIN;
                                (
                                    a * ey + (1.0 - a) * fy,
                                    a * eu + (1.0 - a) * fu,
                                    a * ev + (1.0 - a) * fv,
                                )
                            }
                            None => (fy, fu, fv),
                        }
                    };
                    ry[x] = c * pic.y[i] + (1.0 - c) * ey;
                    ru[x] = c * pic.u[i] + (1.0 - c) * eu;
                    rv[x] = c * pic.v[i] + (1.0 - c) * ev;
                    rc[x] = c.max((wsum / W_MIN).min(1.0));
                }
                (rebuilt, filled)
            })
            .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
        stats.rebuilt = counts.0 as f32 / n as f32;
        stats.spatial_fill = counts.1 as f32 / n as f32;
        (out, stats)
    }

    fn rebuild_broken(&mut self, cur: &Arc<Entry>) -> (Picture, PictureStats) {
        let pic = &cur.pic;
        let n = pic.len();
        let mut stats = PictureStats {
            index: cur.index,
            flagged: pic.conf.iter().filter(|&&c| c < 1.0).count() as f32 / n as f32,
            noise_sigma: cur.sigma,
            ..PictureStats::default()
        };
        let reach = self.cfg.max_gap * self.groups;
        let prev = self
            .last_good
            .get(&cur.group)
            .filter(|e| cur.index - e.index <= reach)
            .cloned();
        let next = self
            .window
            .iter()
            .find(|e| e.index > cur.index && e.group == cur.group && !e.broken && e.index - cur.index <= reach)
            .cloned();
        let base = match (&prev, &next) {
            (Some(p), Some(q)) => {
                let t = (cur.index - p.index) as f32 / (q.index - p.index) as f32;
                self.ensure_flows(&[(p.clone(), q.clone())]);
                let f = self.flow(p, q);
                let c = if self.cfg.consistency {
                    self.ensure_flows(&[(q.clone(), p.clone())]);
                    self.consistency_map(p, q)
                } else {
                    None
                };
                stats.neighbours = 2;
                interpolate(&p.pic, &q.pic, &f, c.as_ref().map(|v| v.as_slice()), t)
            }
            (Some(e), None) | (None, Some(e)) => {
                stats.neighbours = 1;
                e.pic.clone()
            }
            (None, None) => {
                stats.unrecoverable = true;
                return (pic.clone(), stats);
            }
        };
        stats.interpolated = true;
        let mut out = base;
        out.y
            .par_iter_mut()
            .zip(out.u.par_iter_mut())
            .zip(out.v.par_iter_mut())
            .zip(out.conf.par_iter_mut())
            .enumerate()
            .for_each(|(i, (((y, u), v), c))| {
                let k = pic.conf[i];
                *y = k * pic.y[i] + (1.0 - k) * *y;
                *u = k * pic.u[i] + (1.0 - k) * *u;
                *v = k * pic.v[i] + (1.0 - k) * *v;
                *c = 1.0;
            });
        stats.rebuilt = stats.flagged;
        (out, stats)
    }
}

fn compute_flow(cfg: &RestoreConfig, a: &Entry, b: &Entry) -> Flow {
    estimate(
        &a.pic.y,
        &a.pic.conf,
        &b.pic.y,
        &b.pic.conf,
        a.pic.width,
        a.pic.height,
        &cfg.flow,
    )
}

/// A neighbour resampled onto the current picture's grid, with the weight
/// each sample earns (its confidence, flow agreement, temporal distance).
struct Warped {
    y: Vec<f32>,
    u: Vec<f32>,
    v: Vec<f32>,
    w: Vec<f32>,
}

fn warp(src: &Picture, f: &Flow, cons: Option<&[f32]>, distance: usize) -> Warped {
    let (w, h) = (src.width, src.height);
    let tw = 1.0 / (1.0 + 0.35 * (distance.max(1) - 1) as f32);
    let n = w * h;
    let mut out = Warped {
        y: vec![0.0; n],
        u: vec![0.0; n],
        v: vec![0.0; n],
        w: vec![0.0; n],
    };
    out.y
        .par_chunks_mut(w)
        .zip(out.u.par_chunks_mut(w))
        .zip(out.v.par_chunks_mut(w))
        .zip(out.w.par_chunks_mut(w))
        .enumerate()
        .for_each(|(yy, (((ry, ru), rv), rw))| {
            for x in 0..w {
                let i = yy * w + x;
                let px = x as f32 + f.dx[i];
                let py = yy as f32 + f.dy[i];
                let inside = px >= -0.5 && py >= -0.5 && px <= w as f32 - 0.5 && py <= h as f32 - 0.5;
                ry[x] = sample_bicubic(&src.y, w, h, px, py);
                ru[x] = sample_bilinear(&src.u, w, h, px, py);
                rv[x] = sample_bilinear(&src.v, w, h, px, py);
                let c = sample_bilinear(&src.conf, w, h, px, py);
                // Interpolating across a damaged neighbour sample spreads
                // it; only trust samples whose whole footprint is trusted.
                let c = if c < 0.999 { c * c } else { 1.0 };
                let k = cons.map(|k| k[i]).unwrap_or(1.0);
                rw[x] = if inside { c * k * tw } else { 0.0 };
            }
        });
    out
}

/// Bidirectional motion-compensated interpolation at time `t` between `p`
/// (t = 0) and `q` (t = 1), given the flow from p into q.
fn interpolate(p: &Picture, q: &Picture, f: &Flow, cons: Option<&[f32]>, t: f32) -> Picture {
    let (w, h) = (p.width, p.height);
    let mut out = Picture::new(w, h);
    out.y
        .par_chunks_mut(w)
        .zip(out.u.par_chunks_mut(w))
        .zip(out.v.par_chunks_mut(w))
        .enumerate()
        .for_each(|(yy, ((ry, ru), rv))| {
            for x in 0..w {
                // Find where this pixel sits in p: x = s + t * F(s).
                let (mut sx, mut sy) = (x as f32, yy as f32);
                let mut fv = (0.0, 0.0);
                for _ in 0..4 {
                    fv = f.at(sx, sy);
                    sx = x as f32 - t * fv.0;
                    sy = yy as f32 - t * fv.1;
                }
                let qx = x as f32 + (1.0 - t) * fv.0;
                let qy = yy as f32 + (1.0 - t) * fv.1;
                let k = cons
                    .map(|c| sample_bilinear(c, w, h, sx, sy))
                    .unwrap_or(1.0)
                    .max(0.05);
                let wp = (1.0 - t) * sample_bilinear(&p.conf, w, h, sx, sy).max(0.02);
                let wq = t * sample_bilinear(&q.conf, w, h, qx, qy).max(0.02);
                // Occluded vectors: fall back to the nearer picture.
                let (wp, wq) = if t < 0.5 {
                    (wp, wq * k)
                } else {
                    (wp * k, wq)
                };
                let s = wp + wq;
                let (a, b) = (wp / s, wq / s);
                ry[x] = a * sample_bicubic(&p.y, w, h, sx, sy) + b * sample_bicubic(&q.y, w, h, qx, qy);
                ru[x] = a * sample_bilinear(&p.u, w, h, sx, sy) + b * sample_bilinear(&q.u, w, h, qx, qy);
                rv[x] = a * sample_bilinear(&p.v, w, h, sx, sy) + b * sample_bilinear(&q.v, w, h, qx, qy);
            }
        });
    out
}

/// Weighted median of (value, weight, tag) triples (sorts in place).
fn weighted_median(v: &mut [(f32, f32, usize)]) -> f32 {
    v.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
    let total: f32 = v.iter().map(|e| e.1).sum();
    let mut acc = 0.0;
    for e in v.iter() {
        acc += e.1;
        if acc >= 0.5 * total {
            return e.0;
        }
    }
    v.last().map(|e| e.0).unwrap_or(0.0)
}

/// Vertical fill from the nearest trusted rows above and below.
fn spatial_fill(p: &Picture, x: usize, y: usize) -> (f32, f32, f32) {
    let w = p.width;
    let up = (1..=FILL_REACH.min(y)).map(|d| y - d).find(|&r| p.conf[r * w + x] >= 0.5);
    let dn = (1..=FILL_REACH)
        .map(|d| y + d)
        .take_while(|&r| r < p.height)
        .find(|&r| p.conf[r * w + x] >= 0.5);
    let at = |r: usize| {
        let i = r * w + x;
        (p.y[i], p.u[i], p.v[i])
    };
    match (up, dn) {
        (Some(a), Some(b)) => {
            let t = (y - a) as f32 / (b - a) as f32;
            let (ya, ua, va) = at(a);
            let (yb, ub, vb) = at(b);
            (ya + t * (yb - ya), ua + t * (ub - ua), va + t * (vb - va))
        }
        (Some(a), None) => at(a),
        (None, Some(b)) => at(b),
        (None, None) => at(y),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn weighted_median_picks_heavy_value() {
        let mut v = vec![(0.1, 1.0, 0), (0.9, 5.0, 1), (0.2, 1.0, 2)];
        assert_eq!(weighted_median(&mut v), 0.9);
    }

    #[test]
    fn untouched_when_clean_and_no_denoise() {
        let mut e = Engine::new(RestoreConfig::fast(), 1);
        let mut outs = Vec::new();
        for k in 0..5 {
            let mut p = Picture::new(32, 24);
            p.y.iter_mut().enumerate().for_each(|(i, v)| *v = ((i + k) % 7) as f32 / 7.0);
            outs.extend(e.push(p, 0));
        }
        outs.extend(e.finish());
        assert_eq!(outs.len(), 5);
        for (k, (p, s)) in outs.iter().enumerate() {
            assert_eq!(s.index, k);
            assert!(p.y.iter().enumerate().all(|(i, &v)| v == ((i + k) % 7) as f32 / 7.0));
        }
    }
}
