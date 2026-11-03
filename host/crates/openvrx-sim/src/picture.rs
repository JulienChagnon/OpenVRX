// SPDX-License-Identifier: GPL-3.0-or-later
//! Picture sources: test patterns, a moving scene, still images, video
//! through ffmpeg, and the FPV-style OSD overlay.
//!
//! A source renders frame rows at a time `t` (seconds). The engine asks for
//! one field at a time with that field's own capture time, so moving
//! pictures show real interlaced motion (each field is a separate moment),
//! which is what deinterlacers must cope with.
//!
//! Geometry: `width` picture samples cover the active line (the output
//! grid's active range: 754 for NTSC, 922 for PAL) and `height` rows cover
//! the visible frame (480 / 576). The picture is 4:3, so samples are not
//! square. Pixel values are Y (0 black .. 1 white, slightly outside allowed
//! for test signals such as PLUGE) and BT.601 U/V.

use crate::config::PictureConfig;
use crate::font;
use crate::SimError;
use openvrx_core::records::rgb_to_yuv;
use openvrx_core::VideoStandard;
use rayon::prelude::*;
use std::io::Read;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;

/// One picture sample.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Yuv {
    pub y: f32,
    pub u: f32,
    pub v: f32,
}

impl Yuv {
    pub fn rgb(r: f32, g: f32, b: f32) -> Yuv {
        let (y, u, v) = rgb_to_yuv(r, g, b);
        Yuv { y, u, v }
    }
}

/// Picture geometry.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Geometry {
    pub standard: VideoStandard,
    pub width: usize,
    pub height: usize,
}

impl Geometry {
    pub fn of(standard: VideoStandard) -> Geometry {
        let p = standard.params();
        Geometry {
            standard,
            width: p.active_width(),
            height: p.frame_height(),
        }
    }

    /// Width of the picture in square units when the height is 1 (4:3).
    pub fn aspect(&self) -> f32 {
        4.0 / 3.0
    }
}

/// Something that renders frame rows.
pub trait PictureSource: Send {
    /// Renders frame rows `rows` (0-based, top to bottom) at time `t` into
    /// `out` (`rows.len() x width`, row-major).
    fn render(&mut self, t: f64, rows: &[usize], out: &mut [Yuv]) -> Result<(), SimError>;
}

/// Builds the configured source (with the OSD overlay when asked).
pub fn build(cfg: &PictureConfig, osd: bool, geo: Geometry) -> Result<Box<dyn PictureSource>, SimError> {
    let inner: Box<dyn PictureSource> = match cfg {
        PictureConfig::Bars => Box::new(Procedural::new(Pattern::Bars, geo)),
        PictureConfig::Ramp => Box::new(Procedural::new(Pattern::Ramp, geo)),
        PictureConfig::Multiburst => Box::new(Procedural::new(Pattern::Multiburst, geo)),
        PictureConfig::ZonePlate => Box::new(Procedural::new(Pattern::ZonePlate, geo)),
        PictureConfig::Scene => Box::new(Procedural::new(Pattern::Scene, geo)),
        PictureConfig::Flat { y, u, v } => Box::new(Procedural::new(
            Pattern::Flat(Yuv { y: *y, u: *u, v: *v }),
            geo,
        )),
        PictureConfig::Image { path } => Box::new(StillImage::load(path, geo)?),
        PictureConfig::Video { path } => Box::new(FfmpegVideo::spawn(path, geo)?),
    };
    Ok(if osd { Box::new(Osd { inner, geo }) } else { inner })
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Pattern {
    Bars,
    Ramp,
    Multiburst,
    ZonePlate,
    Scene,
    Flat(Yuv),
}

/// Patterns computed per sample.
struct Procedural {
    pattern: Pattern,
    geo: Geometry,
}

impl Procedural {
    fn new(pattern: Pattern, geo: Geometry) -> Procedural {
        Procedural { pattern, geo }
    }
}

impl PictureSource for Procedural {
    fn render(&mut self, t: f64, rows: &[usize], out: &mut [Yuv]) -> Result<(), SimError> {
        let geo = self.geo;
        let pattern = self.pattern;
        out.par_chunks_mut(geo.width)
            .zip(rows.par_iter())
            .for_each(|(line, &row)| {
                for (col, px) in line.iter_mut().enumerate() {
                    *px = sample(pattern, geo, t, col, row);
                }
            });
        Ok(())
    }
}

fn sample(p: Pattern, geo: Geometry, t: f64, col: usize, row: usize) -> Yuv {
    let x = (col as f32 + 0.5) / geo.width as f32;
    let y = (row as f32 + 0.5) / geo.height as f32;
    match p {
        Pattern::Flat(v) => v,
        Pattern::Bars => bars(geo.standard, x, y),
        Pattern::Ramp => ramp(x, y),
        Pattern::Multiburst => multiburst(geo.standard, x),
        Pattern::ZonePlate => zone_plate(geo, t, col, row),
        Pattern::Scene => scene(geo, t, col, row),
    }
}

/// SMPTE EG 1 bars (NTSC) or EBU 100/0/75/0 bars (PAL).
fn bars(std: VideoStandard, x: f32, y: f32) -> Yuv {
    let c = 0.75;
    let top = [
        (c, c, c),
        (c, c, 0.0),
        (0.0, c, c),
        (0.0, c, 0.0),
        (c, 0.0, c),
        (c, 0.0, 0.0),
        (0.0, 0.0, c),
    ];
    match std {
        VideoStandard::Pal => {
            let ebu = [
                (1.0, 1.0, 1.0),
                (c, c, 0.0),
                (0.0, c, c),
                (0.0, c, 0.0),
                (c, 0.0, c),
                (c, 0.0, 0.0),
                (0.0, 0.0, c),
                (0.0, 0.0, 0.0),
            ];
            let k = ((x * 8.0) as usize).min(7);
            let (r, g, b) = ebu[k];
            Yuv::rgb(r, g, b)
        }
        VideoStandard::Ntsc => {
            let bx = x * 7.0;
            if y < 0.67 {
                let (r, g, b) = top[(bx as usize).min(6)];
                Yuv::rgb(r, g, b)
            } else if y < 0.75 {
                let mid = [
                    (0.0, 0.0, c),
                    (0.0, 0.0, 0.0),
                    (c, 0.0, c),
                    (0.0, 0.0, 0.0),
                    (0.0, c, c),
                    (0.0, 0.0, 0.0),
                    (c, c, c),
                ];
                let (r, g, b) = mid[(bx as usize).min(6)];
                Yuv::rgb(r, g, b)
            } else {
                // -I, white, +Q, black, PLUGE (-4, 0, +4 IRE), black.
                let a33 = 33f32.to_radians();
                let setup_span = 92.5;
                if bx < 1.25 {
                    // -I at 20 IRE amplitude.
                    let m = 20.0 / setup_span;
                    Yuv { y: 0.0, u: m * a33.sin(), v: -m * a33.cos() }
                } else if bx < 2.5 {
                    Yuv { y: 1.0, u: 0.0, v: 0.0 }
                } else if bx < 3.75 {
                    let m = 20.0 / setup_span;
                    Yuv { y: 0.0, u: m * a33.cos(), v: m * a33.sin() }
                } else if (5.0..5.333).contains(&bx) {
                    Yuv { y: -4.0 / setup_span, u: 0.0, v: 0.0 }
                } else if (5.667..6.0).contains(&bx) {
                    Yuv { y: 4.0 / setup_span, u: 0.0, v: 0.0 }
                } else {
                    Yuv::default()
                }
            }
        }
    }
}

/// Luma ramp (top half), 6-step staircase, hue ramp (bottom quarter).
fn ramp(x: f32, y: f32) -> Yuv {
    if y < 0.5 {
        Yuv { y: x, u: 0.0, v: 0.0 }
    } else if y < 0.75 {
        Yuv { y: ((x * 6.0).floor() / 5.0).min(1.0), u: 0.0, v: 0.0 }
    } else {
        let a = x * std::f32::consts::TAU;
        Yuv { y: 0.5, u: 0.3 * a.cos(), v: 0.3 * a.sin() }
    }
}

/// White/black flag then six frequency packets at 60 IRE peak to peak.
fn multiburst(std: VideoStandard, x: f32) -> Yuv {
    let p = std.params();
    let freqs: [f64; 6] = match std {
        VideoStandard::Ntsc => [0.5e6, 1.0e6, 2.0e6, 3.0e6, 3.58e6, 4.2e6],
        VideoStandard::Pal => [0.5e6, 1.0e6, 2.0e6, 4.0e6, 4.8e6, 5.8e6],
    };
    if x < 0.05 {
        return Yuv { y: 1.0, u: 0.0, v: 0.0 };
    }
    if x < 0.1 {
        return Yuv { y: 0.0, u: 0.0, v: 0.0 };
    }
    let xr = (x - 0.1) / 0.9;
    let k = ((xr * 6.0) as usize).min(5);
    let local = xr * 6.0 - k as f32;
    if !(0.08..0.92).contains(&local) {
        return Yuv { y: 0.5, u: 0.0, v: 0.0 };
    }
    let t_us = x as f64 * p.active_us;
    let s = (std::f64::consts::TAU * freqs[k] * t_us * 1e-6).sin() as f32;
    Yuv { y: 0.5 + 0.3 * s, u: 0.0, v: 0.0 }
}

/// Circular zone plate reaching about 0.35 cycles per sample at the left
/// and right edges and 0.5 cycles per frame row at top and bottom.
fn zone_plate(geo: Geometry, t: f64, col: usize, row: usize) -> Yuv {
    let dx = col as f64 + 0.5 - geo.width as f64 / 2.0;
    let dy = row as f64 + 0.5 - geo.height as f64 / 2.0;
    let a = 0.7 / geo.width as f64;
    let b = 1.0 / geo.height as f64;
    let ph = std::f64::consts::PI * (a * dx * dx + b * dy * dy) + std::f64::consts::TAU * 0.25 * t;
    Yuv { y: (0.5 + 0.5 * ph.cos()) as f32, u: 0.0, v: 0.0 }
}

/// Moving scene with several speeds of motion.
fn scene(geo: Geometry, t: f64, col: usize, row: usize) -> Yuv {
    let w = geo.width as f64;
    let h = geo.height as f64;
    // Square units: x in 0..4/3, y in 0..1.
    let sx = (col as f64 + 0.5) / w * 4.0 / 3.0;
    let sy = (row as f64 + 0.5) / h;
    // Slow vertical bob of the whole scene (exercises vertical motion).
    let bob = 0.02 * (std::f64::consts::TAU * t / 4.0).sin();
    let yy = sy + bob;
    let horizon = 0.55;
    let mut px = if yy < horizon {
        // Sky gradient.
        let k = (yy / horizon) as f32;
        Yuv::rgb(0.35 + 0.3 * k, 0.55 + 0.25 * k, 0.95 - 0.1 * k)
    } else {
        // Ground: perspective checkerboard panning slowly sideways.
        let d = (yy - horizon).max(1e-3);
        let z = 0.25 / d;
        let gx = (sx - 2.0 / 3.0) * z + 0.6 * t;
        let gz = z + 0.0;
        let check = ((gx.floor() + gz.floor()) as i64).rem_euclid(2) == 0;
        let fog = (1.0 - (d / 0.45)).clamp(0.0, 1.0) as f32 * 0.4;
        let (r, g, b) = if check { (0.2, 0.55, 0.2) } else { (0.35, 0.7, 0.3) };
        Yuv::rgb(r + fog * 0.3, g + fog * 0.2, b + fog * 0.4)
    };
    // Static fine stripes (still-area detail) in the bottom-left corner.
    if sx < 0.25 && sy > 0.85 {
        let stripe = ((col / 2) & 1 == 0) as i32 as f32;
        px = Yuv { y: 0.15 + 0.7 * stripe, u: 0.0, v: 0.0 };
    }
    // Rotating wheel (medium motion): 8 coloured spokes, 0.5 rev/s.
    let (cx, cy, rad) = (0.95, 0.38 + bob, 0.17);
    let (dx, dy) = (sx - cx, sy - cy);
    let r = (dx * dx + dy * dy).sqrt();
    if r < rad {
        let ang = dy.atan2(dx) - std::f64::consts::TAU * 0.5 * t;
        let sector = ((ang / std::f64::consts::TAU * 8.0).floor() as i64).rem_euclid(8) as usize;
        let colours = [
            (1.0, 0.1, 0.1),
            (1.0, 0.9, 0.1),
            (0.1, 0.9, 0.1),
            (0.1, 0.9, 0.9),
            (0.1, 0.2, 1.0),
            (0.9, 0.1, 0.9),
            (1.0, 1.0, 1.0),
            (0.1, 0.1, 0.1),
        ];
        let (cr, cg, cb) = colours[sector];
        px = if r > rad * 0.93 || r < rad * 0.12 {
            Yuv::rgb(0.9, 0.9, 0.9)
        } else {
            Yuv::rgb(cr, cg, cb)
        };
    }
    // Bouncing ball (fast motion).
    let bx = tri(t * 0.55) * (4.0 / 3.0 - 0.16) + 0.08;
    let by = tri(t * 0.8 + 0.3) * 0.5 + 0.3;
    let (dx, dy) = (sx - bx, sy - by);
    let br = (dx * dx + dy * dy).sqrt();
    if br < 0.06 {
        let shade = (1.0 - br / 0.06 * 0.5) as f32;
        px = Yuv::rgb(0.95 * shade, 0.25 * shade, 0.1 * shade);
    }
    // Scrolling text (very fast motion), right to left.
    let text = "OpenVRX SIM 5.8 GHZ";
    let scale_x = 5i64;
    let scale_y = 4i64;
    let text_w = (text.len() * font::CELL_W) as f64 * scale_x as f64;
    let speed = 420.0; // picture samples per second
    let span = w + text_w;
    let x0 = (w - (t * speed).rem_euclid(span)) as i64;
    let y0 = (0.12 * h) as i64;
    match font::text_coverage(text, x0, y0, scale_x, scale_y, col as i64, row as i64) {
        2 => px = Yuv::rgb(1.0, 0.95, 0.2),
        1 => px = Yuv::rgb(0.05, 0.05, 0.05),
        _ => {}
    }
    px
}

/// Triangle wave 0..1..0 with period 1.
fn tri(x: f64) -> f64 {
    let f = x.rem_euclid(1.0);
    if f < 0.5 {
        2.0 * f
    } else {
        2.0 - 2.0 * f
    }
}

/// Letterboxed content size inside the 4:3 picture for a display aspect.
fn letterbox(geo: Geometry, display_aspect: f64) -> (usize, usize) {
    let pic = 4.0 / 3.0;
    if display_aspect > pic {
        let h = (geo.height as f64 * pic / display_aspect).round() as usize;
        (geo.width, h.clamp(1, geo.height))
    } else {
        let w = (geo.width as f64 * display_aspect / pic).round() as usize;
        (w.clamp(1, geo.width), geo.height)
    }
}

/// A still image, letterboxed to 4:3 and resampled once.
struct StillImage {
    geo: Geometry,
    pixels: Vec<Yuv>,
}

impl StillImage {
    fn load(path: &str, geo: Geometry) -> Result<StillImage, SimError> {
        let img = image::open(path).map_err(|e| SimError::Picture(format!("{path}: {e}")))?;
        let rgb = img.to_rgb8();
        let (iw, ih) = rgb.dimensions();
        let (cw, ch) = letterbox(geo, iw as f64 / ih.max(1) as f64);
        let scaled = image::imageops::resize(&rgb, cw as u32, ch as u32, image::imageops::FilterType::Lanczos3);
        let mut pixels = vec![Yuv::default(); geo.width * geo.height];
        let (ox, oy) = ((geo.width - cw) / 2, (geo.height - ch) / 2);
        for (x, y, p) in scaled.enumerate_pixels() {
            let [r, g, b] = p.0;
            pixels[(y as usize + oy) * geo.width + x as usize + ox] =
                Yuv::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0);
        }
        Ok(StillImage { geo, pixels })
    }
}

impl PictureSource for StillImage {
    fn render(&mut self, _t: f64, rows: &[usize], out: &mut [Yuv]) -> Result<(), SimError> {
        let w = self.geo.width;
        for (k, &r) in rows.iter().enumerate() {
            out[k * w..(k + 1) * w].copy_from_slice(&self.pixels[r * w..(r + 1) * w]);
        }
        Ok(())
    }
}

/// Video or image sequence decoded by an ffmpeg child process (rawvideo
/// RGB24 on a pipe), one frame per field, looped forever, letterboxed.
struct FfmpegVideo {
    geo: Geometry,
    field_hz: f64,
    child: Child,
    frames: mpsc::Receiver<Result<Vec<u8>, String>>,
    current: Vec<Yuv>,
    index: Option<u64>,
}

impl FfmpegVideo {
    fn spawn(path: &str, geo: Geometry) -> Result<FfmpegVideo, SimError> {
        let p = geo.standard.params();
        let (w, h) = (geo.width, geo.height);
        let fps = match geo.standard {
            VideoStandard::Ntsc => "60000/1001",
            VideoStandard::Pal => "50",
        };
        let filter = format!(
            "fps={fps},scale=w=if(gt(dar\\,4/3)\\,{w}\\,2*trunc({w}*dar*3/8)):h=if(gt(dar\\,4/3)\\,2*trunc({h}*2/(3*dar))\\,{h}):flags=lanczos,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2,setsar=1"
        );
        let mut child = Command::new("ffmpeg")
            .args(["-v", "error", "-nostdin", "-stream_loop", "-1", "-i", path, "-vf", &filter])
            .args(["-f", "rawvideo", "-pix_fmt", "rgb24", "-"])
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| SimError::Picture(format!("cannot start ffmpeg: {e}")))?;
        let mut stdout = child.stdout.take().expect("piped");
        let mut stderr = child.stderr.take().expect("piped");
        let (tx, rx) = mpsc::sync_channel(6);
        let frame_bytes = w * h * 3;
        std::thread::Builder::new()
            .name("openvrx-sim-ffmpeg".into())
            .spawn(move || loop {
                let mut buf = vec![0u8; frame_bytes];
                match stdout.read_exact(&mut buf) {
                    Ok(()) => {
                        if tx.send(Ok(buf)).is_err() {
                            return;
                        }
                    }
                    Err(e) => {
                        let mut msg = String::new();
                        let _ = stderr.read_to_string(&mut msg);
                        let _ = tx.send(Err(format!("ffmpeg stream ended ({e}): {}", msg.trim())));
                        return;
                    }
                }
            })
            .map_err(|e| SimError::Picture(e.to_string()))?;
        Ok(FfmpegVideo {
            geo,
            field_hz: p.field_hz,
            child,
            frames: rx,
            current: vec![Yuv::default(); w * h],
            index: None,
        })
    }

    fn next_frame(&mut self) -> Result<(), SimError> {
        let buf = self
            .frames
            .recv()
            .map_err(|_| SimError::Picture("ffmpeg reader stopped".into()))?
            .map_err(SimError::Picture)?;
        self.current
            .par_iter_mut()
            .zip(buf.par_chunks_exact(3))
            .for_each(|(px, c)| {
                *px = Yuv::rgb(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0)
            });
        Ok(())
    }
}

impl PictureSource for FfmpegVideo {
    fn render(&mut self, t: f64, rows: &[usize], out: &mut [Yuv]) -> Result<(), SimError> {
        let want = (t * self.field_hz + 1e-6).floor().max(0.0) as u64;
        loop {
            match self.index {
                Some(i) if i >= want => break,
                _ => {
                    self.next_frame()?;
                    self.index = Some(self.index.map_or(0, |i| i + 1));
                }
            }
        }
        let w = self.geo.width;
        for (k, &r) in rows.iter().enumerate() {
            out[k * w..(k + 1) * w].copy_from_slice(&self.current[r * w..(r + 1) * w]);
        }
        Ok(())
    }
}

impl Drop for FfmpegVideo {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// FPV-style OSD: battery voltage, flight timer, a centre cross and a
/// label, white with a black outline, chunky like a character-generator
/// OSD (one font pixel is 3 samples by 2 frame rows).
struct Osd {
    inner: Box<dyn PictureSource>,
    geo: Geometry,
}

impl Osd {
    fn texts(&self, t: f64) -> Vec<(String, i64, i64)> {
        let w = self.geo.width as i64;
        let h = self.geo.height as i64;
        let volts = (16.8 - 0.012 * t).max(13.2);
        let secs = t.max(0.0) as u64;
        let timer = format!("{:02}:{:02}", secs / 60 % 100, secs % 60);
        let cw = font::CELL_W as i64 * 3;
        vec![
            (format!("{volts:.1}V"), w / 12, h / 12),
            (timer.clone(), w - w / 12 - cw * timer.len() as i64, h / 12),
            ("+".to_string(), w / 2 - cw / 2, h / 2 - 8),
            ("OpenVRX".to_string(), w / 2 - cw * 5 / 2, h - h / 8),
        ]
    }
}

impl PictureSource for Osd {
    fn render(&mut self, t: f64, rows: &[usize], out: &mut [Yuv]) -> Result<(), SimError> {
        self.inner.render(t, rows, out)?;
        let texts = self.texts(t);
        let w = self.geo.width;
        out.par_chunks_mut(w).zip(rows.par_iter()).for_each(|(line, &row)| {
            for (text, x0, y0) in &texts {
                if (row as i64) < y0 - 2 || (row as i64) > y0 + 2 * font::CELL_H as i64 + 2 {
                    continue;
                }
                for (col, px) in line.iter_mut().enumerate() {
                    match font::text_coverage(text, *x0, *y0, 3, 2, col as i64, row as i64) {
                        2 => *px = Yuv { y: 1.0, u: 0.0, v: 0.0 },
                        1 => *px = Yuv { y: 0.0, u: 0.0, v: 0.0 },
                        _ => {}
                    }
                }
            }
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn render_all(src: &mut dyn PictureSource, geo: Geometry, t: f64) -> Vec<Yuv> {
        let rows: Vec<usize> = (0..geo.height).collect();
        let mut out = vec![Yuv::default(); geo.width * geo.height];
        src.render(t, &rows, &mut out).unwrap();
        out
    }

    #[test]
    fn bars_levels() {
        let geo = Geometry::of(VideoStandard::Pal);
        let mut s = Procedural::new(Pattern::Bars, geo);
        let img = render_all(&mut s, geo, 0.0);
        // First bar white, last black, yellow luma 0.75*0.886.
        assert!((img[10].y - 1.0).abs() < 1e-5);
        assert!(img[geo.width - 5].y.abs() < 1e-6);
        let yellow = img[geo.width / 8 + geo.width / 16];
        assert!((yellow.y - 0.75 * 0.886).abs() < 1e-3);
        assert!(yellow.u < -0.2);
        let geo = Geometry::of(VideoStandard::Ntsc);
        let mut s = Procedural::new(Pattern::Bars, geo);
        let img = render_all(&mut s, geo, 0.0);
        let pluge_low = img[(geo.height - 5) * geo.width + (geo.width as f32 * 5.15 / 7.0) as usize];
        assert!(pluge_low.y < 0.0);
    }

    #[test]
    fn scene_moves_between_fields() {
        let geo = Geometry::of(VideoStandard::Ntsc);
        let mut s = Procedural::new(Pattern::Scene, geo);
        let a = render_all(&mut s, geo, 1.0);
        let b = render_all(&mut s, geo, 1.0 + 1.0 / 59.94);
        let diff = a.iter().zip(&b).filter(|(p, q)| (p.y - q.y).abs() > 0.05).count();
        assert!(diff > 500, "{diff}");
        let c = render_all(&mut s, geo, 1.0);
        assert_eq!(a, c, "deterministic");
    }

    #[test]
    fn osd_draws_text() {
        let geo = Geometry::of(VideoStandard::Pal);
        let mut s = build(&PictureConfig::Flat { y: 0.5, u: 0.0, v: 0.0 }, true, geo).unwrap();
        let img = render_all(s.as_mut(), geo, 3.0);
        let white = img.iter().filter(|p| p.y == 1.0).count();
        let black = img.iter().filter(|p| p.y == 0.0).count();
        assert!(white > 100 && black > 100, "{white} {black}");
    }

    #[test]
    fn still_image_letterboxed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("wide.png");
        let img = image::RgbImage::from_fn(320, 90, |x, _| image::Rgb([255, (x % 256) as u8, 0]));
        img.save(&path).unwrap();
        let geo = Geometry::of(VideoStandard::Ntsc);
        let mut s = StillImage::load(path.to_str().unwrap(), geo).unwrap();
        let out = render_all(&mut s, geo, 0.0);
        // Black bars top and bottom, content in the middle.
        assert!(out[geo.width * 2 + 100].y.abs() < 1e-6);
        assert!(out[geo.width * (geo.height / 2) + 100].y > 0.3);
        assert!(StillImage::load("does/not/exist.png", geo).is_err());
    }

    #[test]
    fn letterbox_sizes() {
        let geo = Geometry::of(VideoStandard::Pal);
        assert_eq!(letterbox(geo, 4.0 / 3.0), (geo.width, geo.height));
        let (w, h) = letterbox(geo, 16.0 / 9.0);
        assert_eq!(w, geo.width);
        assert_eq!(h, 432);
        let (w, _) = letterbox(geo, 1.0);
        assert_eq!(w, (geo.width as f64 * 0.75).round() as usize);
    }
}
