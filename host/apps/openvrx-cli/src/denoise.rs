// SPDX-License-Identifier: GPL-3.0-or-later
//! Learned post-flight denoise filter (`render`).
//!
//! A linear filter over a 5x5 neighbourhood in each of three consecutive
//! frames and all three planes (Y, Cb, Cr from the 8-bit RGB frame), one
//! output per plane, fitted by least squares on simulated recordings by
//! `tools/denoise/denoise_filter.py` (which also defines the tap order and
//! the colour formulas mirrored here). Simulated against an averaged
//! reference, NTSC 4x4top: about +5 dB RGB PSNR at 16 dB C/N and +6.5 dB
//! at 12 dB on a scene the filter was not trained on.

use rayon::prelude::*;

/// The filter trained on the 2026-11-06 flight's settings (4x4top, clk 9).
pub const DEFAULT_FILTER: &str = include_str!("../filters/denoise-4x4top.json");

pub struct DenoiseFilter {
    radius: usize,
    /// Frames read before and after the output frame: 1 and 1 for the
    /// post-flight filter (version 1, `time`), 2 and 0 for a causal one that
    /// adds no latency live (version 2, `offsets` -2,-1,0).
    before: usize,
    after: usize,
    /// Per output plane: taps in (dt, plane, dy, dx) order, then the bias.
    coef: [Vec<f32>; 3],
}

impl DenoiseFilter {
    pub fn from_json(text: &str) -> Result<DenoiseFilter, String> {
        let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("filter: {e}"))?;
        if v["format"] != "openvrx-denoise-filter" {
            return Err("filter: not an openvrx-denoise-filter file".into());
        }
        let radius = v["radius"].as_u64().ok_or("filter: radius")? as usize;
        let (before, after) = match v["offsets"].as_array() {
            Some(offs) => {
                let offs: Vec<i64> = offs.iter().map(|x| x.as_i64().ok_or("filter: offsets")).collect::<Result<_, _>>()?;
                let consecutive = offs.windows(2).all(|w| w[1] == w[0] + 1);
                match (offs.first(), offs.last()) {
                    (Some(&lo), Some(&hi)) if consecutive && lo <= 0 && hi >= 0 => ((-lo) as usize, hi as usize),
                    _ => return Err("filter: offsets must be consecutive frames including 0".into()),
                }
            }
            None => {
                let time = v["time"].as_u64().ok_or("filter: time")? as usize;
                (time, time)
            }
        };
        let side = 2 * radius + 1;
        let taps = (before + after + 1) * 3 * side * side + 1;
        let plane = |k: usize| -> Result<Vec<f32>, String> {
            let c: Vec<f32> = v["coef"][k]
                .as_array()
                .ok_or("filter: coef")?
                .iter()
                .map(|x| x.as_f64().unwrap_or(0.0) as f32)
                .collect();
            if c.len() != taps {
                return Err(format!("filter: plane {k} has {} taps, expected {taps}", c.len()));
            }
            Ok(c)
        };
        Ok(DenoiseFilter {
            radius,
            before,
            after,
            coef: [plane(0)?, plane(1)?, plane(2)?],
        })
    }

    /// Frames read before the output frame.
    pub fn before(&self) -> usize {
        self.before
    }

    /// Frames read after the output frame (the latency it adds, in fields).
    pub fn after(&self) -> usize {
        self.after
    }

    /// Filters frame `before` of `frames` (before + after + 1 of them, oldest
    /// first). Pixels within `radius` of the border are kept as they are.
    pub fn apply(&self, frames: &[&Planes]) -> Planes {
        assert_eq!(frames.len(), self.before + self.after + 1, "filter window");
        let cur = frames[self.before];
        let (w, h, r) = (cur.width, cur.height, self.radius);
        let side = 2 * r + 1;
        let mut out = cur.clone();
        if w <= 2 * r || h <= 2 * r {
            return out;
        }
        let [oy, ou, ov] = &mut out.p;
        oy.par_chunks_mut(w)
            .zip(ou.par_chunks_mut(w))
            .zip(ov.par_chunks_mut(w))
            .enumerate()
            .filter(|(y, _)| *y >= r && *y < h - r)
            .for_each(|(y, ((ry, ru), rv))| {
                let n = w - 2 * r;
                let mut acc = [vec![0f32; n], vec![0f32; n], vec![0f32; n]];
                let mut t = 0;
                for f in frames {
                    for plane in &f.p {
                        for dy in 0..side {
                            let row = &plane[(y + dy - r) * w..(y + dy - r + 1) * w];
                            for dx in 0..side {
                                let src = &row[dx..dx + n];
                                for (o, a) in acc.iter_mut().enumerate() {
                                    let c = self.coef[o][t];
                                    for (a, s) in a.iter_mut().zip(src) {
                                        *a += c * s;
                                    }
                                }
                                t += 1;
                            }
                        }
                    }
                }
                for (o, (a, dst)) in acc.iter().zip([ry, ru, rv]).enumerate() {
                    let bias = self.coef[o][t];
                    for (d, a) in dst[r..w - r].iter_mut().zip(a) {
                        *d = a + bias;
                    }
                }
            });
        out
    }
}

/// A frame as Y, Cb, Cr planes (BT.601 from 8-bit RGB, 0..255 scale).
#[derive(Clone)]
pub struct Planes {
    pub width: usize,
    pub height: usize,
    pub p: [Vec<f32>; 3],
}

impl Planes {
    pub fn from_rgba8(width: usize, height: usize, rgba: &[u8]) -> Planes {
        let n = width * height;
        let mut p = [Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n)];
        for px in rgba.chunks_exact(4) {
            let (r, g, b) = (px[0] as f32, px[1] as f32, px[2] as f32);
            let y = 0.299 * r + 0.587 * g + 0.114 * b;
            p[0].push(y);
            p[1].push(0.564 * (b - y));
            p[2].push(0.713 * (r - y));
        }
        Planes { width, height, p }
    }

    pub fn to_rgb8(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.width * self.height * 3);
        for k in 0..self.width * self.height {
            let (y, cb, cr) = (self.p[0][k], self.p[1][k], self.p[2][k]);
            let r = y + cr / 0.713;
            let b = y + cb / 0.564;
            let g = (y - 0.299 * r - 0.114 * b) / 0.587;
            for v in [r, g, b] {
                out.push((v + 0.5).clamp(0.0, 255.0) as u8);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_filter_loads_and_keeps_a_flat_frame() {
        let f = DenoiseFilter::from_json(DEFAULT_FILTER).unwrap();
        let (w, h) = (32, 24);
        let rgba: Vec<u8> = (0..w * h).flat_map(|_| [120u8, 100, 80, 255]).collect();
        let p = Planes::from_rgba8(w, h, &rgba);
        let frames: Vec<&Planes> = (0..f.before() + f.after() + 1).map(|_| &p).collect();
        let out = f.apply(&frames).to_rgb8();
        // A flat colour is what the filter should leave alone (within a
        // few codes: it is fitted, not constrained).
        for px in out.chunks_exact(3) {
            assert!((px[0] as i32 - 120).abs() <= 4 && (px[1] as i32 - 100).abs() <= 4 && (px[2] as i32 - 80).abs() <= 4, "{px:?}");
        }
    }

    #[test]
    fn causal_offsets_load_and_read_only_past_frames() {
        // Version 2, frames -2,-1,0, radius 0: Y out = the newest frame's Y,
        // the two older frames' taps zero, so they must not matter.
        let mut y = vec![0.0; 3 * 3 + 1];
        y[6] = 1.0; // dt 0, plane Y
        let mut cb = vec![0.0; 10];
        cb[7] = 1.0;
        let mut cr = vec![0.0; 10];
        cr[8] = 1.0;
        let json = serde_json::json!({"format": "openvrx-denoise-filter", "version": 2, "radius": 0,
                                      "offsets": [-2, -1, 0], "coef": [y, cb, cr]})
        .to_string();
        let f = DenoiseFilter::from_json(&json).unwrap();
        assert_eq!((f.before(), f.after()), (2, 0));
        let old = Planes::from_rgba8(1, 1, &[0, 0, 0, 255]);
        let new = Planes::from_rgba8(1, 1, &[200, 50, 10, 255]);
        assert_eq!(f.apply(&[&old, &old, &new]).to_rgb8(), vec![200, 50, 10]);
        let bad = json.replace("[-2,-1,0]", "[-2,0]");
        assert!(DenoiseFilter::from_json(&bad).is_err());
    }

    #[test]
    fn rgb_round_trip() {
        let rgba = [10u8, 200, 90, 255, 255, 0, 128, 255];
        let p = Planes::from_rgba8(2, 1, &rgba);
        assert_eq!(p.to_rgb8(), vec![10, 200, 90, 255, 0, 128]);
    }
}
