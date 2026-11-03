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

