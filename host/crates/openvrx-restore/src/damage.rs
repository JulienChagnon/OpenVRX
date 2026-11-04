// SPDX-License-Identifier: GPL-3.0-or-later
//! Confidence maps: exact ones from the decoder's damage masks and line
//! quality, and a simple blind estimate for ordinary DVR files.

use crate::picture::{estimate_noise, Picture};
use openvrx_core::{damage, LineFlags, LineQuality};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

/// Confidence of a pixel from its damage mask byte.
///
/// Clean samples are fully trusted (and never modified by the rebuild);
/// graded levels are partly replaced; carrier loss and missing data are
/// replaced entirely.
#[inline]
pub fn conf_from_damage(d: u8) -> f32 {
    if d < damage::NOISY {
        1.0
    } else if d < damage::REPAIRED {
        0.6
    } else if d < damage::CLICK {
        0.4
    } else if d < damage::SEVERE {
        0.1
    } else {
        0.0
    }
}

/// Upper bound on the confidence of a whole row from its line quality.
///
/// Only information the decoder actually measured lowers it: lines with no
/// carrier or lost samples carry nothing, flywheel lines (no sync found)
/// are half trusted, and a finite measured noise above 3 IRE scales the
/// trust down. `LineQuality::UNKNOWN` leaves the row alone.
pub fn row_conf(q: &LineQuality) -> f32 {
    if q.flags.contains(LineFlags::NO_CARRIER) || q.flags.contains(LineFlags::SAMPLES_LOST) {
        return 0.0;
    }
    let mut c: f32 = 1.0;
    if q.flags.contains(LineFlags::FLYWHEEL) {
        c = c.min(0.5);
    }
    if q.noise_ire.is_finite() && q.noise_ire > 3.0 {
        let r = q.noise_ire / 6.0;
        c = c.min(1.0 / (1.0 + r * r));
    }
    c
}

/// Fills `pic.conf` from a damage mask and optional per-row quality.
pub fn apply_mask(pic: &mut Picture, mask: &[u8], rows: Option<&[LineQuality]>) {
    let w = pic.width;
    pic.conf
        .par_chunks_mut(w)
        .enumerate()
        .for_each(|(y, row)| {
            let cap = rows.and_then(|r| r.get(y)).map(row_conf).unwrap_or(1.0);
            for (x, c) in row.iter_mut().enumerate() {
                let m = mask.get(y * w + x).copied().unwrap_or(damage::NONE);
                *c = conf_from_damage(m).min(cap);
            }
        });
}

/// True when a mask or row quality carries any damage information.
pub fn has_damage_info(mask: &[u8], rows: &[LineQuality]) -> bool {
    mask.iter().any(|&d| d >= damage::NOISY) || rows.iter().any(|q| row_conf(q) < 1.0)
}

/// What the blind estimate found in one picture.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BlindSummary {
    /// Whole picture is static (noise with no vertical structure).
    pub static_frame: bool,
    /// Whole picture is a flat field (black or blue mute screen).
    pub flat_frame: bool,
    /// Fraction of pixels marked untrusted.
    pub damaged_fraction: f32,
    /// Luma noise sigma measured on trusted pixels.
    pub noise_sigma: f32,
}

const SEG: usize = 32;
const REF_ROWS: usize = 24;

/// Blind damage estimate for pictures that come without a mask.
///
/// - Noise line bursts: per row segment (32 px), the horizontal
///   high-frequency energy and the error of predicting the row from the
///   rows above and below, each relative to the median of the same segment
///   column over +-24 rows. Both must be well above the local norm.
/// - Sparkles: pixels that jump away from both vertical neighbours while
///   those agree with each other (FM clicks are one line high).
/// - Whole static frames (high energy, no correlation between adjacent
///   rows) and flat mute screens (black or blue) are marked fully broken.
///
/// The result is combined into `pic.conf` with `min`; `sensitivity` scales
/// the thresholds (1.0 nominal, higher finds more).
pub fn blind_estimate(pic: &mut Picture, sensitivity: f32) -> BlindSummary {
    let w = pic.width;
    let h = pic.height;
    let sens = sensitivity.clamp(0.25, 4.0);
    let mut summary = BlindSummary::default();
    if w < 8 || h < 8 {
        return summary;
    }
    let y = &pic.y;

    let (_, sy) = mean_std(y);
    let (_, su) = mean_std(&pic.u);
    let (_, sv) = mean_std(&pic.v);
    if sy < 0.02 && su < 0.02 && sv < 0.02 {
        summary.flat_frame = true;
        pic.conf.iter_mut().for_each(|c| *c = 0.0);
        summary.damaged_fraction = 1.0;
        return summary;
    }

    // Per-row statistics: horizontal HF energy and adjacent-row correlation.
    let row_stats: Vec<(f32, f32)> = (0..h - 1)
        .into_par_iter()
        .map(|r| {
            let a = &y[r * w..r * w + w];
            let b = &y[(r + 1) * w..(r + 1) * w + w];
            let (mut e, mut sab, mut saa, mut sbb) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
            for x in 2..w - 2 {
                let ha = a[x] - 0.2 * (a[x - 2] + a[x - 1] + a[x] + a[x + 1] + a[x + 2]);
                let hb = b[x] - 0.2 * (b[x - 2] + b[x - 1] + b[x] + b[x + 1] + b[x + 2]);
                sab += ha * hb;
                saa += ha * ha;
                sbb += hb * hb;
                let d = a[x] - 0.5 * (a[x - 1] + a[x + 1]);
                e += d * d;
            }
            let corr = if saa > 0.0 && sbb > 0.0 {
                sab / (saa * sbb).sqrt()
            } else {
                1.0
            };
            (e / (w - 4) as f32, corr)
        })
        .collect();
    let mut energies: Vec<f32> = row_stats.iter().map(|s| s.0).collect();
    let mut corrs: Vec<f32> = row_stats.iter().map(|s| s.1).collect();
    let e_med = median(&mut energies);
    let c_med = median(&mut corrs);
    if e_med > 0.002 / sens && c_med < 0.3 {
        summary.static_frame = true;
        pic.conf.iter_mut().for_each(|c| *c = 0.0);
        summary.damaged_fraction = 1.0;
        return summary;
    }

    let sigma = estimate_noise(y, &pic.conf, w, h);
    summary.noise_sigma = sigma;
    let floor = 2.0 * sigma * sigma + 1e-5;

    // Segment statistics.
    let nseg = w.div_ceil(SEG);
    let stats: Vec<(f32, f32)> = (0..h * nseg)
        .into_par_iter()
        .map(|k| {
            let r = k / nseg;
            let s = k % nseg;
            let x0 = (s * SEG).max(1);
            let x1 = ((s + 1) * SEG).min(w - 1);
            let up = r.saturating_sub(1);
            let dn = (r + 1).min(h - 1);
            let (mut e, mut v) = (0.0, 0.0);
            for x in x0..x1 {
                let c = y[r * w + x];
                let dh = c - 0.5 * (y[r * w + x - 1] + y[r * w + x + 1]);
                let dv = c - 0.5 * (y[up * w + x] + y[dn * w + x]);
                e += dh * dh;
                v += dv * dv;
            }
            let n = (x1.saturating_sub(x0)).max(1) as f32;
            (e / n, v / n)
        })
        .collect();
    let seg_damage: Vec<f32> = (0..h * nseg)
        .into_par_iter()
        .map(|k| {
            let r = k / nseg;
            let s = k % nseg;
            let lo = r.saturating_sub(REF_ROWS);
            let hi = (r + REF_ROWS + 1).min(h);
            let mut es: Vec<f32> = (lo..hi).map(|rr| stats[rr * nseg + s].0).collect();
            let mut vs: Vec<f32> = (lo..hi).map(|rr| stats[rr * nseg + s].1).collect();
            let re = stats[k].0 / (median(&mut es) + floor);
            let rv = stats[k].1 / (median(&mut vs) + floor);
            let ratio = re.min(rv) * sens;
            smoothstep(ratio, 3.0, 8.0)
        })
        .collect();

    let thr = (0.1f32).max(6.0 * sigma) / sens;
    let conf = &mut pic.conf;
    let damaged: usize = conf
        .par_chunks_mut(w)
        .enumerate()
        .map(|(r, row)| {
            let up = r.saturating_sub(1);
            let dn = (r + 1).min(h - 1);
            let mut count = 0;
            let mut spark = vec![false; w];
            if r > 0 && r + 1 < h {
                for x in 0..w {
                    let a = y[up * w + x];
                    let b = y[dn * w + x];
                    let imp = (y[r * w + x] - 0.5 * (a + b)).abs() - 0.5 * (a - b).abs();
                    if imp > thr {
                        spark[x.saturating_sub(1)] = true;
                        spark[x] = true;
                        spark[(x + 1).min(w - 1)] = true;
                    }
                }
            }
            for (x, c) in row.iter_mut().enumerate() {
                let s = x / SEG;
                let d0 = seg_damage[r * nseg + s];
                let du = seg_damage[up * nseg + s];
                let dd = seg_damage[dn * nseg + s];
                let mut d = d0.max(0.5 * du.max(dd));
                if spark[x] {
                    d = d.max(0.9);
                }
                *c = c.min(1.0 - d);
                if *c < 1.0 {
                    count += 1;
                }
            }
            count
        })
        .sum();
    summary.damaged_fraction = damaged as f32 / (w * h) as f32;
    summary
}

fn mean_std(p: &[f32]) -> (f32, f32) {
    let n = p.len().max(1) as f64;
    let m = p.iter().map(|&v| v as f64).sum::<f64>() / n;
    let v = p.iter().map(|&v| (v as f64 - m).powi(2)).sum::<f64>() / n;
    (m as f32, v.sqrt() as f32)
}

fn median(v: &mut [f32]) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    let k = v.len() / 2;
    let (_, m, _) = v.select_nth_unstable_by(k, |a, b| a.total_cmp(b));
    *m
}

#[inline]
fn smoothstep(x: f32, lo: f32, hi: f32) -> f32 {
    let t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scene(w: usize, h: usize) -> Picture {
        let mut p = Picture::new(w, h);
        for k in 0..w * h {
            let x = (k % w) as f32;
            let yy = (k / w) as f32;
            p.y[k] = 0.45 + 0.2 * (x * 0.07).sin() * (yy * 0.05).cos() + 0.1 * (x * 0.31 + yy * 0.2).sin();
            p.u[k] = 0.05 * (yy * 0.02).sin();
            p.v[k] = 0.05 * (x * 0.02).cos();
        }
        p
    }

    #[test]
    fn mask_levels() {
        assert_eq!(conf_from_damage(damage::NONE), 1.0);
        assert_eq!(conf_from_damage(damage::LOST), 0.0);
        assert!(conf_from_damage(damage::CLICK) < conf_from_damage(damage::NOISY));
        assert_eq!(row_conf(&LineQuality::UNKNOWN), 1.0);
        let mut q = LineQuality::UNKNOWN;
        q.flags.insert(LineFlags::NO_CARRIER);
        assert_eq!(row_conf(&q), 0.0);
    }

    #[test]
    fn finds_bursts_and_sparkles_but_not_texture() {
        let (w, h) = (256, 192);
        let mut p = scene(w, h);
        let mut s = 99u64;
        let mut rnd = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (s >> 40) as f32 / (1u64 << 24) as f32
        };
        for r in 100..106 {
            for x in 0..w {
                p.y[r * w + x] = rnd();
            }
        }
        p.y[40 * w + 50] = 1.0;
        p.y[40 * w + 51] = 1.0;
        let s = blind_estimate(&mut p, 1.0);
        assert!(!s.static_frame && !s.flat_frame);
        let burst_mean: f32 = (100..106).flat_map(|r| (0..w).map(move |x| (r, x))).map(|(r, x)| p.conf[r * w + x]).sum::<f32>() / (6 * w) as f32;
        assert!(burst_mean < 0.2, "burst confidence {burst_mean}");
        assert!(p.conf[40 * w + 50] < 0.2);
        let clean: usize = (0..h).filter(|r| !(95..111).contains(r) && *r != 40).map(|r| (0..w).filter(|&x| p.conf[r * w + x] < 1.0).count()).sum();
        assert!(clean < w * h / 100, "false positives {clean}");
    }

    #[test]
    fn static_and_mute_frames() {
        let (w, h) = (128, 96);
        let mut p = scene(w, h);
        let mut s = 7u64;
        for v in p.y.iter_mut() {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *v = (s >> 40) as f32 / (1u64 << 24) as f32;
        }
        assert!(blind_estimate(&mut p, 1.0).static_frame);
        let mut blue = Picture::new(w, h);
        blue.y.iter_mut().for_each(|v| *v = 0.1);
        blue.u.iter_mut().for_each(|v| *v = 0.3);
        assert!(blind_estimate(&mut blue, 1.0).flat_frame);
        assert!(blue.conf.iter().all(|&c| c == 0.0));
    }
}
