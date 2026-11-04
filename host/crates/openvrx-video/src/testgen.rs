// SPDX-License-Identifier: GPL-3.0-or-later
//! Test pictures and sequences: progressive Y/U/V images, colour bars,
//! zone plates, a band-limited moving scene, and helpers that split frames
//! into fields and degrade line records (noise, clicks, dropouts).
//!
//! Everything here is deterministic (seeded) so tests and benchmarks are
//! reproducible.

use crate::util::{cubic, Rng};
use openvrx_core::quality::damage;
use openvrx_core::records::rgb_to_yuv;
use openvrx_core::{
    FieldParity, Frame, LineFlags, LineQuality, LineRecord, VideoStandard, YuvField, COMBINED,
};
use std::f64::consts::PI;

/// A progressive Y/U/V picture (Y 0..1, U/V BT.601 scale).
#[derive(Clone, Debug, PartialEq)]
pub struct YuvImage {
    pub width: usize,
    pub height: usize,
    pub y: Vec<f32>,
    pub u: Vec<f32>,
    pub v: Vec<f32>,
}

impl YuvImage {
    pub fn new(width: usize, height: usize) -> YuvImage {
        let n = width * height;
        YuvImage {
            width,
            height,
            y: vec![0.0; n],
            u: vec![0.0; n],
            v: vec![0.0; n],
        }
    }

    /// Full visible frame size of a standard (active width x 2 fields).
    pub fn for_standard(standard: VideoStandard) -> YuvImage {
        let p = standard.params();
        YuvImage::new(p.active_width(), p.frame_height())
    }

    #[inline]
    pub fn set_rgb(&mut self, x: usize, y: usize, r: f32, g: f32, b: f32) {
        let (yy, u, v) = rgb_to_yuv(r, g, b);
        let k = y * self.width + x;
        self.y[k] = yy;
        self.u[k] = u;
        self.v[k] = v;
    }

    /// Copies a frame's planes into an image.
    pub fn from_frame(f: &Frame) -> YuvImage {
        YuvImage {
            width: f.width,
            height: f.height,
            y: f.y.clone(),
            u: f.u.clone(),
            v: f.v.clone(),
        }
    }
}

/// Colour bars: white, yellow, cyan, green, magenta, red, blue, black with
/// RGB primaries at `amplitude` (0.75 for 75 % bars).
pub fn color_bars(width: usize, height: usize, amplitude: f32) -> YuvImage {
    let mut img = YuvImage::new(width, height);
    for x in 0..width {
        let bar = x * 8 / width;
        let (r, g, b) = BAR_RGB[bar];
        for y in 0..height {
            img.set_rgb(x, y, r * amplitude, g * amplitude, b * amplitude);
        }
    }
    img
}

/// RGB (0/1) of the eight bars, left to right.
pub const BAR_RGB: [(f32, f32, f32); 8] = [
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 0.0),
    (0.0, 1.0, 1.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, 0.0),
];

/// Luma-only circular zone plate. Horizontal frequency rises linearly from
/// 0 at the centre to `fx_max` cycles/sample at the left/right edges and
/// vertical frequency to `fy_max` cycles/frame-row at the top/bottom.
pub fn zone_plate(width: usize, height: usize, fx_max: f64, fy_max: f64, amplitude: f32) -> YuvImage {
    let mut img = YuvImage::new(width, height);
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;
    let ax = fx_max / (2.0 * cx);
    let ay = fy_max / (2.0 * cy);
    for y in 0..height {
        for x in 0..width {
            let dx = x as f64 - cx;
            let dy = y as f64 - cy;
            let ph = 2.0 * PI * (ax * dx * dx + ay * dy * dy);
            img.y[y * width + x] = 0.5 + amplitude * ph.cos() as f32;
        }
    }
    img
}

/// Flat mid-grey luma with chroma that sweeps hue across the picture and
/// saturation down it (cross-luma / dot-crawl test).
pub fn chroma_sweep(width: usize, height: usize, saturation: f32) -> YuvImage {
    let mut img = YuvImage::new(width, height);
    for y in 0..height {
        let s = saturation * (0.3 + 0.7 * y as f32 / height as f32);
        for x in 0..width {
            let h = 2.0 * PI as f32 * x as f32 / width as f32;
            let k = y * width + x;
            img.y[k] = 0.5;
            img.u[k] = 0.3 * s * h.cos();
            img.v[k] = 0.3 * s * h.sin();
        }
    }
    img
}

/// A band-limited, textured, moving test scene. The background is a large
/// random texture (edges, gradients, fine stripes, text-like blocks) panned
/// at `pan` pixels per field; a disc moves over it at `object` pixels per
/// field. Frames are sampled with bicubic interpolation, so sub-pixel
/// motion is rendered correctly.
pub struct Scene {
    tex_w: usize,
    tex_h: usize,
    ty: Vec<f32>,
    tu: Vec<f32>,
    tv: Vec<f32>,
    pub pan: (f64, f64),
    pub object: (f64, f64),
    pub object_radius: f64,
}

impl Scene {
    pub fn new(seed: u64, pan: (f64, f64), object: (f64, f64)) -> Scene {
        let tex_w = 1400;
        let tex_h = 900;
        let mut rng = Rng::new(seed);
        let n = tex_w * tex_h;
        let mut r = vec![0.0f32; n];
        let mut g = vec![0.0f32; n];
        let mut b = vec![0.0f32; n];
        // Smooth background gradient.
        for y in 0..tex_h {
            for x in 0..tex_w {
                let k = y * tex_w + x;
                let fx = x as f32 / tex_w as f32;
                let fy = y as f32 / tex_h as f32;
                r[k] = 0.25 + 0.3 * fx;
                g[k] = 0.3 + 0.25 * fy;
                b[k] = 0.45 - 0.2 * fx * fy;
            }
        }
        // Rectangles with hard edges (anti-aliased by the blur below).
        for _ in 0..140 {
            let w = 8 + rng.below(120) as usize;
            let h = 6 + rng.below(90) as usize;
            let x0 = rng.below((tex_w - w) as u64) as usize;
            let y0 = rng.below((tex_h - h) as u64) as usize;
            let col = (rng.uniform() as f32, rng.uniform() as f32, rng.uniform() as f32);
            let striped = rng.below(4) == 0;
            let period = 3 + rng.below(6) as usize;
            for y in y0..y0 + h {
                for x in x0..x0 + w {
                    let k = y * tex_w + x;
                    let on = !striped || ((x + y) / period) & 1 == 0;
                    if on {
                        r[k] = col.0 * 0.9 + 0.05;
                        g[k] = col.1 * 0.9 + 0.05;
                        b[k] = col.2 * 0.9 + 0.05;
                    }
                }
            }
        }
        // Diagonal lines (edge-directed interpolation test material).
        for _ in 0..40 {
            let x0 = rng.uniform() * tex_w as f64;
            let y0 = rng.uniform() * tex_h as f64;
            let ang = rng.uniform() * PI;
            let len = 60.0 + rng.uniform() * 300.0;
            let lum = if rng.below(2) == 0 { 0.95 } else { 0.05 };
            let steps = (len * 2.0) as usize;
            for s in 0..steps {
                let t = s as f64 / 2.0;
                let x = x0 + t * ang.cos();
                let y = y0 + t * ang.sin();
                for dy in 0..2 {
                    let xi = x as isize;
                    let yi = y as isize + dy;
                    if xi >= 0 && yi >= 0 && (xi as usize) < tex_w && (yi as usize) < tex_h {
                        let k = yi as usize * tex_w + xi as usize;
                        r[k] = lum;
                        g[k] = lum;
                        b[k] = lum;
                    }
                }
            }
        }
        // Light Gaussian blur (sigma ~0.7 px): sharp, camera-like detail
        // that still stays mostly inside the frame's vertical bandwidth.
        let k = [0.03f32, 0.22, 0.5, 0.22, 0.03];
        for plane in [&mut r, &mut g, &mut b] {
            blur_sep(plane, tex_w, tex_h, &k);
        }
        let mut ty = vec![0.0; n];
        let mut tu = vec![0.0; n];
        let mut tv = vec![0.0; n];
        for i in 0..n {
            let (y, u, v) = rgb_to_yuv(r[i], g[i], b[i]);
            ty[i] = y;
            tu[i] = u;
            tv[i] = v;
        }
        Scene {
            tex_w,
            tex_h,
            ty,
            tu,
            tv,
            pan,
            object,
            object_radius: 40.0,
        }
    }

    /// Renders the progressive picture at field time `t` (in fields).
    pub fn render(&self, width: usize, height: usize, t: f64) -> YuvImage {
        let mut img = YuvImage::new(width, height);
        let ox = 100.0 + self.pan.0 * t;
        let oy = 100.0 + self.pan.1 * t;
        let cx = width as f64 * 0.3 + self.object.0 * t;
        let cy = height as f64 * 0.5 + self.object.1 * t;
        for y in 0..height {
            for x in 0..width {
                let sx = x as f64 + ox;
                let sy = y as f64 + oy;
                let k = y * width + x;
                img.y[k] = self.sample(&self.ty, sx, sy);
                img.u[k] = self.sample(&self.tu, sx, sy);
                img.v[k] = self.sample(&self.tv, sx, sy);
                // Moving disc with a soft (band-limited) rim and a pattern
                // that moves with it.
                let dx = x as f64 - cx;
                let dy = y as f64 - cy;
                let d = (dx * dx + dy * dy).sqrt();
                let a = ((self.object_radius - d) / 1.5).clamp(-1.0, 1.0) * 0.5 + 0.5;
                if a > 0.0 {
                    let pat = 0.5 + 0.35 * ((dx * 0.25).sin() * (dy * 0.2).cos());
                    let a = a as f32;
                    img.y[k] = img.y[k] * (1.0 - a) + pat as f32 * a;
                    img.u[k] = img.u[k] * (1.0 - a) + 0.12 * a;
                    img.v[k] = img.v[k] * (1.0 - a) - 0.08 * a;
                }
            }
        }
        img
    }

    fn sample(&self, plane: &[f32], x: f64, y: f64) -> f32 {
        let xf = x.floor();
        let yf = y.floor();
        let tx = (x - xf) as f32;
        let ty = (y - yf) as f32;
        let xi = xf as isize;
        let yi = yf as isize;
        let w = self.tex_w as isize;
        let h = self.tex_h as isize;
        let px = |xx: isize, yy: isize| -> f32 {
            let xx = xx.rem_euclid(w) as usize;
            let yy = yy.rem_euclid(h) as usize;
            plane[yy * self.tex_w + xx]
        };
        let mut col = [0.0f32; 4];
        for (j, c) in col.iter_mut().enumerate() {
            let yy = yi - 1 + j as isize;
            *c = cubic(px(xi - 1, yy), px(xi, yy), px(xi + 1, yy), px(xi + 2, yy), tx);
        }
        cubic(col[0], col[1], col[2], col[3], ty)
    }
}

fn blur_sep(plane: &mut [f32], w: usize, h: usize, k: &[f32; 5]) {
    let mut tmp = vec![0.0f32; plane.len()];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (j, kv) in k.iter().enumerate() {
                let xx = (x as isize + j as isize - 2).clamp(0, w as isize - 1) as usize;
                acc += plane[y * w + xx] * kv;
            }
            tmp[y * w + x] = acc;
        }
    }
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (j, kv) in k.iter().enumerate() {
                let yy = (y as isize + j as isize - 2).clamp(0, h as isize - 1) as usize;
                acc += tmp[yy * w + x] * kv;
            }
            plane[y * w + x] = acc;
        }
    }
}

/// Field parity and frame index of the `seq`-th field of a sequence that
/// starts with field 1 at seq 0.
#[inline]
pub fn field_info(seq: u64) -> (FieldParity, u64) {
    let parity = if seq & 1 == 0 {
        FieldParity::First
    } else {
        FieldParity::Second
    };
    (parity, seq / 2)
}

/// Takes the rows of `parity` from a frame-sized image as a `YuvField`.
pub fn image_to_field(img: &YuvImage, standard: VideoStandard, parity: FieldParity, seq: u64) -> YuvField {
    let p = standard.params();
    let mut f = YuvField::new(standard, seq, parity);
    assert_eq!(img.width, f.width, "image width must equal the active width");
    assert_eq!(img.height, 2 * f.lines, "image height must equal the frame height");
    let off = p.field_row_offset(parity);
    for k in 0..f.lines {
        let src = (2 * k + off) * img.width;
        let dst = k * f.width;
        f.y[dst..dst + f.width].copy_from_slice(&img.y[src..src + img.width]);
        f.u[dst..dst + f.width].copy_from_slice(&img.u[src..src + img.width]);
        f.v[dst..dst + f.width].copy_from_slice(&img.v[src..src + img.width]);
    }
    for q in &mut f.quality {
        *q = clean_quality(0.0);
    }
    f
}

/// A clean line's quality record with the given RMS noise.
pub fn clean_quality(noise_ire: f32) -> LineQuality {
    LineQuality {
        cn_db: 30.0 - 20.0 * (noise_ire.max(0.05) / 0.05).log10() * 0.5,
        noise_ire,
        sync_score: 1.0,
        clicks: 0,
        timing_err: 0.0,
        power_db: 40.0,
        weight: LineQuality::weight_from_noise(noise_ire),
        flags: LineFlags::BURST_OK,
    }
}

/// Adds white Gaussian noise of `sigma` IRE to a line and updates its
/// quality (noise estimate, weight, C/N).
pub fn add_noise(rec: &mut LineRecord, sigma: f32, rng: &mut Rng) {
    for s in &mut rec.samples {
        *s += sigma * rng.gauss() as f32;
    }
    let n0 = if rec.quality.noise_ire.is_finite() {
        rec.quality.noise_ire
    } else {
        0.0
    };
    let n = (n0 * n0 + sigma * sigma).sqrt();
    rec.quality.noise_ire = n;
    rec.quality.weight = LineQuality::weight_from_noise(n);
    rec.quality.cn_db = 10.0 + 20.0 * (20.0 / n.max(0.05)).log10();
}

/// Inserts `count` FM clicks (impulses of about +-60 IRE, 1-3 samples
/// wide) into the active part of a line and flags them in the damage mask.
pub fn add_clicks(rec: &mut LineRecord, count: usize, rng: &mut Rng) {
    let p = rec.standard.params();
    let a = p.active_range();
    for _ in 0..count {
        let x = a.start + rng.below(a.len() as u64 - 4) as usize;
        let w = 1 + rng.below(3) as usize;
        let sign = if rng.below(2) == 0 { 1.0 } else { -1.0 };
        for i in x..x + w {
            rec.samples[i] += sign * (50.0 + 20.0 * rng.uniform() as f32);
            rec.damage[i] = damage::CLICK;
        }
        rec.quality.clicks += 1;
    }
}

/// Replaces samples `range` with noise (carrier loss) and flags them
/// SEVERE; whole lines get the NO_CARRIER flag and zero weight.
pub fn add_dropout(rec: &mut LineRecord, range: std::ops::Range<usize>, rng: &mut Rng) {
    for i in range.clone() {
        rec.samples[i] = 30.0 + 45.0 * rng.gauss() as f32;
        rec.damage[i] = damage::SEVERE;
    }
    if range.len() >= rec.samples.len() / 2 {
        rec.quality.flags.insert(LineFlags::NO_CARRIER);
        rec.quality.weight = 0.0;
        rec.quality.noise_ire = 45.0;
    }
}

/// Mean of a slice.
pub fn mean(v: &[f32]) -> f64 {
    v.iter().map(|&x| x as f64).sum::<f64>() / v.len().max(1) as f64
}

/// Builds a `Frame` from an image (for comparisons and PNG output).
pub fn image_to_frame(img: &YuvImage) -> Frame {
    let mut f = Frame::new(img.width, img.height);
    f.y.copy_from_slice(&img.y);
    f.u.copy_from_slice(&img.u);
    f.v.copy_from_slice(&img.v);
    f.row_source.fill(COMBINED);
    f
}
