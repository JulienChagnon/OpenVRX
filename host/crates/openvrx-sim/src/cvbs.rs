// SPDX-License-Identifier: GPL-3.0-or-later
//! Composite video (CVBS) encoder for NTSC-M and PAL-B/G.
//!
//! The composite is produced on the output grid of `openvrx_core::standard`
//! (910 samples per line for NTSC, 1135 for PAL, sample 0 at 0H), one line
//! at a time, in IRE (blanking 0, peak white 100). Absolute line `L` counts
//! lines since frame 0 line 1; frame line numbers and field membership are
//! exactly those of `openvrx_core::standard`.
//!
//! - Raster: horizontal sync, equalising and broad (serrated vertical sync)
//!   pulses on the standard half-line slots (NTSC: pre-equalising from line
//!   1 and from line 263.5; PAL: from line 623.5 and line 311), edges shaped
//!   as raised cosines with the standard rise times.
//! - Picture: luma band-limited (4.2 / 5.0 MHz), U and V band-limited
//!   (1.3 MHz) and QAM-modulated on the subcarrier, optional 7.5 IRE setup,
//!   optional camera noise. Visible lines are exactly the core's active
//!   lines (PAL lines 23 and 623 carry half a line of picture).
//! - Colour burst: NTSC on the -U axis, PAL at +-135 degrees with the V
//!   switch alternating every line.
//! - Subcarrier phase is a function of absolute time (exact rational
//!   arithmetic), so the NTSC phase alternates line to line, the PAL
//!   sequence spans 8 fields, and PAL's 0.0064 sample-per-line slip on the
//!   1135 grid appears by itself.

use crate::config::CvbsConfig;
use crate::dsp::{lowpass_fir, sincos_turns};
use crate::picture::Yuv;
use openvrx_core::{FieldParity, StandardParams, VideoStandard};
use rand::{Rng, SeedableRng};
use rand_distr::StandardNormal;

/// Pulse kinds on a half-line slot.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Pulse {
    None,
    HSync,
    Eq,
    Broad,
}

/// Picture content of a line.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Content {
    Blank,
    Full,
    FirstHalf,
    SecondHalf,
}

/// Structure of one frame line.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LineLayout {
    /// Pulses starting at 0H and at H/2.
    pub pulses: [Pulse; 2],
    pub content: Content,
    pub burst: bool,
}

/// Layout of a frame line (1-based) of a standard.
pub fn line_layout(std: VideoStandard, frame_line: u16) -> LineLayout {
    use Pulse::*;
    let p = std.params();
    let pulses = match std {
        VideoStandard::Ntsc => match frame_line {
            1..=3 | 7..=9 => [Eq, Eq],
            4..=6 => [Broad, Broad],
            263 => [HSync, Eq],
            264 | 265 | 270 | 271 => [Eq, Eq],
            266 => [Eq, Broad],
            267 | 268 => [Broad, Broad],
            269 => [Broad, Eq],
            272 => [Eq, None],
            _ => [HSync, None],
        },
        VideoStandard::Pal => match frame_line {
            1 | 2 | 314 | 315 => [Broad, Broad],
            3 => [Broad, Eq],
            4 | 5 | 311 | 312 | 316 | 317 | 624 | 625 => [Eq, Eq],
            313 => [Eq, Broad],
            318 => [Eq, None],
            623 => [HSync, Eq],
            _ => [HSync, None],
        },
    };
    let parity = if frame_line <= p.lines_per_field[0] {
        FieldParity::First
    } else {
        FieldParity::Second
    };
    let first = p.first_active_line[parity.index()];
    let last = first + p.active_lines_per_field - 1;
    let mut content = if (first..=last).contains(&frame_line) {
        Content::Full
    } else {
        Content::Blank
    };
    if std == VideoStandard::Pal {
        if frame_line == 23 {
            content = Content::SecondHalf;
        } else if frame_line == 623 {
            content = Content::FirstHalf;
        }
    }
    LineLayout {
        pulses,
        content,
        burst: pulses[0] == HSync,
    }
}

/// Frame line (1-based), parity and frame index of an absolute line.
pub fn line_position(p: &StandardParams, abs_line: u64) -> (u64, u16, FieldParity) {
    let lpf = p.lines_per_frame as u64;
    let frame = abs_line / lpf;
    let fl = (abs_line % lpf) as u16 + 1;
    let parity = if fl <= p.lines_per_field[0] {
        FieldParity::First
    } else {
        FieldParity::Second
    };
    (frame, fl, parity)
}

/// Field index (2 per frame) of an absolute line.
pub fn field_index(p: &StandardParams, abs_line: u64) -> u64 {
    let (frame, _, parity) = line_position(p, abs_line);
    2 * frame + parity.index() as u64
}

/// Picture row within its field image for a frame line, if visible.
pub fn field_row(p: &StandardParams, frame_line: u16) -> Option<usize> {
    p.output_row(frame_line).map(|r| r / 2)
}

/// Raised-cosine step: 0 below -d/2, 1 above d/2, 0.5 at 0.
#[inline]
fn step(x: f64, d: f64) -> f64 {
    if x <= -0.5 * d {
        0.0
    } else if x >= 0.5 * d {
        1.0
    } else {
        0.5 + 0.5 * (std::f64::consts::PI * x / d).sin()
    }
}

/// Raised-cosine window between 50 % points `a` and `b`.
#[inline]
fn window(t: f64, a: f64, b: f64, d: f64) -> f64 {
    step(t - a, d) - step(t - b, d)
}

/// The encoder.
#[derive(Clone, Debug)]
pub struct Encoder {
    pub standard: VideoStandard,
    p: &'static StandardParams,
    black_ire: f32,
    luma_fir: Vec<f32>,
    chroma_fir: Vec<f32>,
    colour: bool,
    camera_noise_ire: f32,
    seed: u64,
    /// Edge durations (0-100 %) in microseconds.
    sync_edge_us: f64,
    burst_edge_us: f64,
    blank_edge_us: f64,
    /// Subcarrier cycles per grid sample as an exact fraction.
    sc_num: u128,
    sc_den: u128,
}

impl Encoder {
    pub fn new(standard: VideoStandard, cfg: &CvbsConfig, seed: u64) -> Encoder {
        let p = standard.params();
        let setup = cfg.setup.unwrap_or(standard == VideoStandard::Ntsc);
        let grid = p.grid_rate_hz();
        let luma_bw = cfg.luma_bw_hz.unwrap_or(match standard {
            VideoStandard::Ntsc => 4.2e6,
            VideoStandard::Pal => 5.0e6,
        });
        let (sc_num, sc_den) = match standard {
            VideoStandard::Ntsc => (1, 4),
            VideoStandard::Pal => (709_379, 2_837_500),
        };
        // Raised cosine 10-90 % time = 0.5903 x its 0-100 % duration.
        let rc = 0.5903;
        Encoder {
            standard,
            p,
            black_ire: if setup { 7.5 } else { 0.0 },
            luma_fir: lowpass_fir(31, (luma_bw / grid).min(0.49), 6.0),
            chroma_fir: lowpass_fir(31, cfg.chroma_bw_hz / grid, 5.0),
            colour: cfg.colour,
            camera_noise_ire: cfg.camera_noise_ire,
            seed,
            sync_edge_us: match standard {
                VideoStandard::Ntsc => 0.14,
                VideoStandard::Pal => 0.20,
            } / rc,
            burst_edge_us: 0.30 / rc,
            blank_edge_us: 0.20 / rc,
            sc_num,
            sc_den,
        }
    }

    pub fn params(&self) -> &'static StandardParams {
        self.p
    }

    /// Black level in IRE (7.5 with setup).
    pub fn black_ire(&self) -> f32 {
        self.black_ire
    }

    /// Subcarrier phase in cycles [0, 1) at absolute grid sample `g`.
    #[inline]
    pub fn subcarrier_phase(&self, g: u64) -> f64 {
        ((g as u128 * self.sc_num) % self.sc_den) as f64 / self.sc_den as f64
    }

    /// Sync contribution (0 or 1 at full depth) of a line's pulses at time
    /// `t` microseconds after that line's 0H.
    fn pulses_at(&self, layout: &LineLayout, t: f64) -> f64 {
        let half = 0.5e6 / self.p.line_hz;
        let mut s = 0.0;
        for (k, pulse) in layout.pulses.iter().enumerate() {
            let w = match pulse {
                Pulse::None => continue,
                Pulse::HSync => self.p.hsync_us,
                Pulse::Eq => self.p.eq_pulse_us,
                Pulse::Broad => self.p.broad_pulse_us,
            };
            let a = k as f64 * half;
            s += window(t, a, a + w, self.sync_edge_us);
        }
        s
    }

    /// Renders absolute line `abs_line` into `out` (`samples_per_line`).
    /// `field` is the picture of the line's field (`active_lines_per_field
    /// x active_width`), needed for visible lines; None renders black.
    pub fn render_line(&self, abs_line: u64, field: Option<&[Yuv]>, out: &mut [f32]) {
        let p = self.p;
        let spl = p.samples_per_line;
        debug_assert_eq!(out.len(), spl);
        let lpf = p.lines_per_frame as u64;
        let (_, fl, _) = line_position(p, abs_line);
        let layout = line_layout(self.standard, fl);
        let next_layout = line_layout(self.standard, ((abs_line + 1) % lpf) as u16 + 1);
        let prev_layout = line_layout(self.standard, ((abs_line + lpf - 1) % lpf) as u16 + 1);
        let us_per = 1e6 / p.grid_rate_hz();
        let h_us = 1e6 / p.line_hz;
        let sync = p.sync_ire as f64;
        let g0 = abs_line * spl as u64;

        // Sync and blanking.
        for (i, o) in out.iter_mut().enumerate() {
            let t = i as f64 * us_per;
            let s = self.pulses_at(&layout, t)
                + self.pulses_at(&next_layout, t - h_us)
                + self.pulses_at(&prev_layout, t + h_us);
            *o = (sync * s) as f32;
        }

        // Burst.
        if self.colour && layout.burst {
            let start = p.burst_start_us;
            let len = p.burst_cycles / p.fsc_hz * 1e6;
            let pal_sign = if abs_line & 1 == 0 { 1.0 } else { -1.0 };
            let i0 = ((start - self.burst_edge_us) / us_per).floor().max(0.0) as usize;
            let i1 = (((start + len + self.burst_edge_us) / us_per).ceil() as usize).min(spl);
            let dph = p.fsc_per_sample();
            let ph0 = self.subcarrier_phase(g0 + i0 as u64);
            for (j, o) in out.iter_mut().enumerate().take(i1).skip(i0) {
                let t = j as f64 * us_per;
                // Edges inside the nominal burst window, so the porch
                // after it stays flat.
                let e = 0.5 * self.burst_edge_us;
                let env = window(t, start + e, start + len - e, self.burst_edge_us);
                let (sn, cs) = sincos_turns(ph0 + (j - i0) as f64 * dph);
                let b = match self.standard {
                    VideoStandard::Ntsc => -sn as f64,
                    VideoStandard::Pal => {
                        std::f64::consts::FRAC_1_SQRT_2 * (-sn as f64 + pal_sign * cs as f64)
                    }
                };
                *o += (p.burst_ire as f64 * env * b) as f32;
            }
        }

        // Picture.
        if layout.content == Content::Blank {
            return;
        }
        let ar = p.active_range();
        let width = ar.len();
        let (a_us, b_us) = match layout.content {
            Content::Full => (p.active_start_us, p.active_start_us + p.active_us),
            Content::FirstHalf => (p.active_start_us, 0.5 * h_us - 1.5),
            Content::SecondHalf => (0.5 * h_us, p.active_start_us + p.active_us),
            Content::Blank => unreachable!(),
        };
        let row = field_row(p, fl);
        // Picture samples padded by the filter half-length on both sides.
        let pad = self.luma_fir.len() / 2;
        let n = width + 2 * pad;
        let mut y = vec![0.0f32; n];
        let mut u = vec![0.0f32; n];
        let mut v = vec![0.0f32; n];
        if let (Some(img), Some(r)) = (field, row) {
            let src = &img[r * width..(r + 1) * width];
            for (k, px) in src.iter().enumerate() {
                y[pad + k] = px.y;
                u[pad + k] = px.u;
                v[pad + k] = px.v;
            }
        }
        let span = (p.white_ire - self.black_ire) as f64;
        if self.camera_noise_ire > 0.0 {
            let mut rng = rand::rngs::SmallRng::seed_from_u64(crate::dsp::mix_seed(&[self.seed, 0xCA, abs_line]));
            let sy = self.camera_noise_ire / span as f32;
            for k in pad..pad + width {
                let a: f32 = rng.sample(StandardNormal);
                let b: f32 = rng.sample(StandardNormal);
                let c: f32 = rng.sample(StandardNormal);
                y[k] += sy * a;
                u[k] += 0.5 * sy * b;
                v[k] += 0.5 * sy * c;
            }
        }
        // Band-limit along the line (arrays are zero padded by `pad`, the
        // filters' half length, so the convolution needs no checks).
        let conv = |x: &[f32], h: &[f32]| -> Vec<f32> {
            let m = h.len() / 2;
            let mut out = vec![0.0f32; x.len()];
            for k in m..x.len() - m {
                let src = &x[k - m..k - m + h.len()];
                let mut acc = 0.0f32;
                for j in 0..h.len() {
                    acc += h[j] * src[j];
                }
                out[k] = acc;
            }
            out
        };
        let yf = conv(&y, &self.luma_fir);
        let (uf, vf) = if self.colour {
            (conv(&u, &self.chroma_fir), conv(&v, &self.chroma_fir))
        } else {
            (Vec::new(), Vec::new())
        };
        let dph = p.fsc_per_sample();
        let pal_sign = if abs_line & 1 == 0 { 1.0 } else { -1.0 };
        let lo = ((a_us - self.blank_edge_us) / us_per).floor().max(0.0) as usize;
        let hi = (((b_us + self.blank_edge_us) / us_per).ceil() as usize).min(spl);
        let ph0 = self.subcarrier_phase(g0 + lo as u64);
        for (i, o) in out.iter_mut().enumerate().take(hi).skip(lo) {
            let t = i as f64 * us_per;
            let w = window(t, a_us, b_us, self.blank_edge_us);
            if w <= 0.0 {
                continue;
            }
            // Picture sample k sits at grid sample ar.start + k.
            let k = i as isize - ar.start as isize + pad as isize;
            if k < 0 || k as usize >= n {
                *o += (w * self.black_ire as f64) as f32;
                continue;
            }
            let k = k as usize;
            let mut val = self.black_ire as f64 + span * yf[k] as f64;
            if self.colour {
                let (sn, cs) = sincos_turns(ph0 + (i - lo) as f64 * dph);
                let vs = match self.standard {
                    VideoStandard::Ntsc => 1.0,
                    VideoStandard::Pal => pal_sign,
                };
                val += span * (uf[k] as f64 * sn as f64 + vs * vf[k] as f64 * cs as f64);
            }
            *o += (w * val) as f32;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::standard::{NTSC, PAL};

    fn enc(std: VideoStandard) -> Encoder {
        Encoder::new(std, &CvbsConfig::default(), 3)
    }

    fn flat_field(std: VideoStandard, y: f32, u: f32, v: f32) -> Vec<Yuv> {
        let p = std.params();
        vec![Yuv { y, u, v }; p.active_width() * p.active_lines_per_field as usize]
    }

    #[test]
    fn vertical_interval_pulse_counts() {
        for std in VideoStandard::ALL {
            let p = std.params();
            let (mut eq, mut broad, mut h) = (0, 0, 0);
            for fl in 1..=p.lines_per_frame {
                for pulse in line_layout(std, fl).pulses {
                    match pulse {
                        Pulse::Eq => eq += 1,
                        Pulse::Broad => broad += 1,
                        Pulse::HSync => h += 1,
                        Pulse::None => {}
                    }
                }
            }
            // Two fields x (pre + post equalising) and two x broad.
            let per = if std == VideoStandard::Ntsc { 6 } else { 5 };
            assert_eq!(eq, 4 * per, "{std:?}");
            assert_eq!(broad, 2 * per, "{std:?}");
            // Every slot of the frame (2 per line) has a pulse except the
            // second halves of normal lines and of the last post-eq line.
            assert!(h > p.lines_per_frame as usize - 20);
        }
        // Visible lines match the core's active lines.
        for std in VideoStandard::ALL {
            let p = std.params();
            let visible = (1..=p.lines_per_frame)
                .filter(|&fl| line_layout(std, fl).content != Content::Blank)
                .count();
            assert_eq!(visible, p.frame_height());
        }
    }

    #[test]
    fn levels_sync_blank_white() {
        for std in VideoStandard::ALL {
            let p = std.params();
            let e = enc(std);
            let field = flat_field(std, 1.0, 0.0, 0.0);
            let fl = p.first_active_line[0] + 10;
            let mut out = vec![0.0; p.samples_per_line];
            e.render_line(fl as u64 - 1, Some(&field), &mut out);
            let tip = p.sync_tip_range();
            for i in tip {
                assert!((out[i] - p.sync_ire).abs() < 0.01, "{std:?} sync {}", out[i]);
            }
            for i in p.back_porch_range() {
                assert!(out[i].abs() < 0.01, "{std:?} porch {}", out[i]);
            }
            let mid = p.active_range().start + p.active_width() / 2;
            assert!((out[mid] - 100.0).abs() < 0.2, "{std:?} white {}", out[mid]);
            // Black with setup for NTSC.
            let field = flat_field(std, 0.0, 0.0, 0.0);
            e.render_line(fl as u64 - 1, Some(&field), &mut out);
            assert!((out[mid] - e.black_ire()).abs() < 0.05);
            // 0H is the 50 % point of the falling sync edge: the previous
            // line's last sample and this line's first are each side of it.
            assert!(out[0] < p.sync_ire * 0.3 && out[0] > p.sync_ire * 0.7);
        }
    }

    #[test]
    fn burst_amplitude_and_phase() {
        let p = &NTSC;
        let e = enc(VideoStandard::Ntsc);
        let mut a = vec![0.0; p.samples_per_line];
        let mut b = vec![0.0; p.samples_per_line];
        e.render_line(40, None, &mut a);
        e.render_line(41, None, &mut b);
        let br = p.burst_range();
        let mid = br.start + 10..br.end - 10;
        let peak = mid.clone().map(|i| a[i].abs()).fold(0.0, f32::max);
        assert!((peak - 20.0).abs() < 0.5, "{peak}");
        // NTSC: subcarrier flips 180 degrees from line to line on the grid.
        for i in mid {
            assert!((a[i] + b[i]).abs() < 0.05, "{} {}", a[i], b[i]);
        }
    }

    #[test]
    fn pal_v_switch_and_grid_slip() {
        let p = &PAL;
        let e = enc(VideoStandard::Pal);
        // Grid slip: phase at the same grid sample one line later advances
        // by 1135 x fsc_per_sample = 283.75 + 0.0016 cycles.
        let d = e.subcarrier_phase(1135 + 300) - e.subcarrier_phase(300);
        let expect = (1135.0 * p.fsc_per_sample()).fract();
        assert!(((d.rem_euclid(1.0)) - expect).abs() < 1e-9);
        // Burst phase alternates +-135: the V component flips.
        let mut a = vec![0.0; p.samples_per_line];
        let mut b = vec![0.0; p.samples_per_line];
        e.render_line(100, None, &mut a);
        e.render_line(101, None, &mut b);
        let br = p.burst_range();
        // Demodulate V (cos) and U (sin) over the burst.
        let demod = |x: &[f32], line: u64| {
            let (mut su, mut sv) = (0.0, 0.0);
            for (i, &xi) in x.iter().enumerate().take(br.end - 8).skip(br.start + 8) {
                let ph = std::f64::consts::TAU * e.subcarrier_phase(line * 1135 + i as u64);
                su += xi as f64 * ph.sin();
                sv += xi as f64 * ph.cos();
            }
            (su, sv)
        };
        let (ua, va) = demod(&a, 100);
        let (ub, vb) = demod(&b, 101);
        assert!(ua < 0.0 && ub < 0.0, "burst on -U");
        assert!(va * vb < 0.0, "V switch");
        assert!((ua - ub).abs() / ua.abs() < 0.05);
    }

    #[test]
    fn chroma_amplitude_matches_uv() {
        let p = &PAL;
        let e = enc(VideoStandard::Pal);
        let field = flat_field(VideoStandard::Pal, 0.5, 0.2, 0.0);
        let mut out = vec![0.0; p.samples_per_line];
        e.render_line(100, Some(&field), &mut out);
        let a = p.active_range();
        let mid = a.start + 200..a.start + 400;
        let n = mid.len() as f32;
        let mean: f32 = mid.clone().map(|i| out[i]).sum::<f32>() / n;
        let rms = (mid.map(|i| (out[i] - mean).powi(2)).sum::<f32>() / n).sqrt();
        assert!((mean - 50.0).abs() < 0.5, "{mean}");
        assert!((rms * std::f32::consts::SQRT_2 - 20.0).abs() < 0.3, "{rms}");
    }

    #[test]
    fn camera_noise_is_seeded() {
        let cfg = CvbsConfig {
            camera_noise_ire: 2.0,
            ..Default::default()
        };
        let e = Encoder::new(VideoStandard::Ntsc, &cfg, 9);
        let field = flat_field(VideoStandard::Ntsc, 0.5, 0.0, 0.0);
        let mut a = vec![0.0; 910];
        let mut b = vec![0.0; 910];
        e.render_line(50, Some(&field), &mut a);
        e.render_line(50, Some(&field), &mut b);
        assert_eq!(a, b);
        let mid = &a[300..600];
        let mean: f32 = mid.iter().sum::<f32>() / 300.0;
        let var: f32 = mid.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / 300.0;
        assert!(var.sqrt() > 0.3 && var.sqrt() < 2.5, "{}", var.sqrt());
    }
}
