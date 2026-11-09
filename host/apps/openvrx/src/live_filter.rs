// SPDX-License-Identifier: GPL-3.0-or-later
//! The learned linear denoise filter for the live view ("learned" preset).
//!
//! Same filter family as `openvrx-cli render` (tools/denoise/denoise_filter.py
//! fits it): 5x5 pixels x 3 fields over Y, Cb and Cr, one output per plane,
//! but causal: it reads fields k-2, k-1 and k (filter file version 2,
//! `offsets` -2,-1,0), so it adds no latency. The fit works on 8-bit RGB
//! frames turned into Y/Cb/Cr on a 0..255 scale; the live picture is Y 0..1
//! and BT.601 U/V. Both are linear in each other (Cb = 0.564 (B - Y) and
//! B - Y = 2.03211 U, so Cb = 1.14611 x 255 U; Cr = 0.713 x 1.13983 x 255 V),
//! so the coefficients are converted once here and the GPU pass works on the
//! live planes directly. Only the 0..255 clipping of the training frames is
//! not reproduced (it matters only in saturated areas).
//!
//! Two filters ship: one fitted on fields at their true half-line position
//! and one on steady-OSD fields (line-doubled on the same rows, the app's
//! default view); the field geometry differs, so each view uses the one that
//! matches its steady OSD setting.

/// Filter for steady OSD off (fields at their half-line position).
pub const FILTER_FIELDS: &str = include_str!("../../openvrx-cli/filters/denoise-live.json");
/// Filter for steady OSD on.
pub const FILTER_STEADY: &str = include_str!("../../openvrx-cli/filters/denoise-live-steady.json");

/// Spatial radius (5x5) and frames read; fixed by the GPU pass.
pub const RADIUS: usize = 2;
pub const FRAMES: usize = 3;
/// Taps per output plane: frames x planes x 5 x 5, then the bias.
pub const TAPS: usize = FRAMES * 3 * (2 * RADIUS + 1) * (2 * RADIUS + 1) + 1;

/// Live-scale size of one unit of each training plane: Y, Cb, Cr per 255
/// of Y, U, V.
const SCALE: [f32; 3] = [1.0, 0.564 * 2.032_11, 0.713 * 1.139_83];

/// Coefficients on the live scale, per output plane (Y, U, V): taps in
/// (frame oldest first, plane, dy, dx) order, then the bias.
#[derive(Clone, Debug, PartialEq)]
pub struct LiveFilter {
    pub coef: [Vec<f32>; 3],
}

impl LiveFilter {
    /// Reads a causal filter file (version 2, offsets -2,-1,0, radius 2).
    pub fn from_json(text: &str) -> Result<LiveFilter, String> {
        let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("filter: {e}"))?;
        if v["format"] != "openvrx-denoise-filter" {
            return Err("filter: not an openvrx-denoise-filter file".into());
        }
        if v["radius"].as_u64() != Some(RADIUS as u64) {
            return Err(format!("filter: the live pass needs radius {RADIUS}"));
        }
        let offs: Vec<i64> = v["offsets"]
            .as_array()
            .ok_or("filter: no offsets (a version 1 filter reads the next field: not for live use)")?
            .iter()
            .filter_map(|x| x.as_i64())
            .collect();
        if offs != [-2, -1, 0] {
            return Err(format!("filter: the live pass needs offsets -2,-1,0, not {offs:?}"));
        }
        let mut coef: [Vec<f32>; 3] = Default::default();
        for (o, out) in coef.iter_mut().enumerate() {
            let c: Vec<f64> = v["coef"][o]
                .as_array()
                .ok_or("filter: coef")?
                .iter()
                .map(|x| x.as_f64().unwrap_or(0.0))
                .collect();
            if c.len() != TAPS {
                return Err(format!("filter: plane {o} has {} taps, expected {TAPS}", c.len()));
            }
            let side = 2 * RADIUS + 1;
            *out = c
                .iter()
                .enumerate()
                .map(|(t, &x)| {
                    if t == TAPS - 1 {
                        // Bias: 0..255 units of the output plane.
                        (x / (255.0 * SCALE[o] as f64)) as f32
                    } else {
                        let plane = (t / (side * side)) % 3;
                        (x * SCALE[plane] as f64 / SCALE[o] as f64) as f32
                    }
                })
                .collect();
        }
        Ok(LiveFilter { coef })
    }

    /// The coefficients as one array for the GPU: Y taps, then U, then V.
    pub fn packed(&self) -> Vec<f32> {
        self.coef.iter().flatten().copied().collect()
    }

    /// CPU reference of the GPU pass: filters `cur` from `frames` (oldest
    /// first, each Y, U, V planes of w x h). Border pixels keep `cur`.
    #[cfg(test)]
    pub fn apply(&self, w: usize, h: usize, frames: [&[Vec<f32>; 3]; FRAMES], cur: &[Vec<f32>; 3]) -> [Vec<f32>; 3] {
        let mut out = cur.clone();
        let r = RADIUS;
        let side = 2 * r + 1;
        for y in r..h.saturating_sub(r) {
            for x in r..w.saturating_sub(r) {
                let mut acc = [0f32; 3];
                for (f, fr) in frames.iter().enumerate() {
                    for dy in 0..side {
                        for dx in 0..side {
                            let k = (y + dy - r) * w + (x + dx - r);
                            for plane in 0..3 {
                                let t = ((f * 3 + plane) * side + dy) * side + dx;
                                for (o, a) in acc.iter_mut().enumerate() {
                                    *a += self.coef[o][t] * fr[plane][k];
                                }
                            }
                        }
                    }
                }
                for (o, a) in acc.iter().enumerate() {
                    out[o][y * w + x] = a + self.coef[o][TAPS - 1];
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity_json(bias_y: f64) -> String {
        // Y out = newest Y, Cb out = newest Cb, Cr out = newest Cr (centre tap).
        let mut coef = vec![vec![0.0f64; TAPS]; 3];
        for (o, c) in coef.iter_mut().enumerate() {
            c[((2 * 3 + o) * 5 + 2) * 5 + 2] = 1.0;
        }
        coef[0][TAPS - 1] = bias_y;
        serde_json::json!({"format": "openvrx-denoise-filter", "version": 2, "radius": 2,
                           "offsets": [-2, -1, 0], "coef": coef})
        .to_string()
    }

    #[test]
    fn shipped_filters_load() {
        for text in [FILTER_FIELDS, FILTER_STEADY] {
            let f = LiveFilter::from_json(text).unwrap();
            assert_eq!(f.packed().len(), 3 * TAPS);
        }
    }

    #[test]
    fn scale_conversion_keeps_identity_and_scales_bias() {
        let f = LiveFilter::from_json(&identity_json(25.5)).unwrap();
        let centre = |p: usize| ((2 * 3 + p) * 5 + 2) * 5 + 2;
        for o in 0..3 {
            assert!((f.coef[o][centre(o)] - 1.0).abs() < 1e-6);
        }
        assert!((f.coef[0][TAPS - 1] - 0.1).abs() < 1e-6);
        // A cross-plane tap is scaled by the ratio of the plane scales.
        let mut j: serde_json::Value = serde_json::from_str(&identity_json(0.0)).unwrap();
        j["coef"][0][centre(1)] = serde_json::json!(1.0); // Y out from Cb in
        let f = LiveFilter::from_json(&j.to_string()).unwrap();
        assert!((f.coef[0][centre(1)] - SCALE[1]).abs() < 1e-6);
    }

    #[test]
    fn rejects_non_causal() {
        let sym = identity_json(0.0).replace("\"offsets\":[-2,-1,0]", "\"offsets\":[-1,0,1]");
        assert!(LiveFilter::from_json(&sym).is_err());
        let v1 = include_str!("../../openvrx-cli/filters/denoise-4x4top.json");
        assert!(LiveFilter::from_json(v1).is_err());
    }
}
