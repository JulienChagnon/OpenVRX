// SPDX-License-Identifier: GPL-3.0-or-later
//! Composite encoder: Y/U/V fields to NTSC or PAL composite line records in
//! IRE on the line-locked grid (sample 0 at 0H), with shaped sync edges,
//! vertical sync pulse lines, colour burst and band-limited chroma.
//!
//! Conventions shared with the decoder (`color`):
//!
//! ```text
//! composite = black + (white - black) * (Y + U sin(P) + s V cos(P))
//! P(x, line) = 2 pi (f (x - offset) + f W line_abs) + phase0
//! ```
//!
//! with `f` the subcarrier frequency in cycles per grid sample, `W` the
//! samples per line, `line_abs` the absolute line count and `s` the PAL
//! V-switch (+1 on even absolute lines, -1 on odd; always +1 for NTSC). The
//! burst is `burst_ire` along -U (NTSC) or along -U +- V at 135/225 degrees
//! (PAL).

use crate::util::{cubic, design_lowpass, fir_range};
use openvrx_core::quality::damage;
use openvrx_core::standard::StandardParams;
use openvrx_core::{FieldParity, LineFlags, LineQuality, LineRecord, RxId, VideoStandard, YuvField};
use std::f64::consts::PI;

/// Encoder options.
#[derive(Clone, Debug)]
pub struct EncoderConfig {
    /// Chroma (U/V) lowpass bandwidth in Hz before modulation; 0 disables.
    pub chroma_bandwidth_hz: f64,
    /// Luma lowpass bandwidth in Hz; 0 disables.
    pub luma_bandwidth_hz: f64,
    /// Whole-signal delay in grid samples (models residual timing offsets
    /// between receivers).
    pub time_offset: f64,
    /// Subcarrier phase offset in degrees.
    pub subcarrier_phase_deg: f64,
    /// Emit colour (burst and chroma). False models a monochrome camera.
    pub color: bool,
    /// Receiver id stamped on the records.
    pub rx: RxId,
    /// Sync/blanking edge rise time in microseconds.
    pub rise_us: f64,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        EncoderConfig {
            chroma_bandwidth_hz: 1.3e6,
            luma_bandwidth_hz: 0.0,
            time_offset: 0.0,
            subcarrier_phase_deg: 0.0,
            color: true,
            rx: 0,
            rise_us: 0.2,
        }
    }
}

/// What a frame line carries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum LineKind {
    Equalising,
    Broad,
    Normal,
}

/// Composite encoder for one standard.
pub struct CompositeEncoder {
    p: &'static StandardParams,
    pub cfg: EncoderConfig,
    chroma_lp: Vec<f32>,
    luma_lp: Vec<f32>,
}

impl CompositeEncoder {
    pub fn new(standard: VideoStandard, cfg: EncoderConfig) -> CompositeEncoder {
        let p = standard.params();
        let rate = p.grid_rate_hz();
        let chroma_lp = if cfg.chroma_bandwidth_hz > 0.0 {
            design_lowpass(cfg.chroma_bandwidth_hz / rate, 31)
        } else {
            vec![1.0]
        };
        let luma_lp = if cfg.luma_bandwidth_hz > 0.0 {
            design_lowpass(cfg.luma_bandwidth_hz / rate, 31)
        } else {
            vec![1.0]
        };
        CompositeEncoder {
            p,
            cfg,
            chroma_lp,
            luma_lp,
        }
    }

    pub fn params(&self) -> &'static StandardParams {
        self.p
    }

    fn line_kind(&self, frame_line: u16) -> LineKind {
        match self.p.standard {
            VideoStandard::Ntsc => match frame_line {
                4..=6 | 267..=269 => LineKind::Broad,
                1..=9 | 264..=272 => LineKind::Equalising,
                _ => LineKind::Normal,
            },
            VideoStandard::Pal => match frame_line {
                1..=2 | 314..=315 => LineKind::Broad,
                3..=5 | 311..=313 | 316..=318 | 623..=625 => LineKind::Equalising,
                _ => LineKind::Normal,
            },
        }
    }

    /// Absolute line count used for the subcarrier phase and V-switch.
    #[inline]
    pub fn abs_line(&self, frame_index: u64, frame_line: u16) -> u64 {
        frame_index * self.p.lines_per_frame as u64 + (frame_line as u64 - 1)
    }

    /// Encodes every line of one field. `field` holds the active picture of
    /// that field; `frame_index` counts frames (for the subcarrier
    /// sequence); `host_ns` stamps the first line.
    pub fn encode_field(&self, field: &YuvField, frame_index: u64, host_ns: u64) -> Vec<LineRecord> {
        let p = self.p;
        let n = p.field_lines(field.parity);
        let line_ns = 1e9 / p.line_hz;
        (0..n)
            .map(|k| {
                let samples = self.encode_line(field, frame_index, k);
                let frame_line = p.frame_line(field.parity, k);
                let mut q = LineQuality {
                    cn_db: 40.0,
                    noise_ire: 0.0,
                    sync_score: 1.0,
                    clicks: 0,
                    timing_err: 0.0,
                    power_db: 40.0,
                    weight: LineQuality::weight_from_noise(0.0),
                    flags: LineFlags(0),
                };
                if self.line_kind(frame_line) != LineKind::Normal {
                    q.flags.insert(LineFlags::VBI);
                } else if self.cfg.color {
                    q.flags.insert(LineFlags::BURST_OK);
                }
                LineRecord {
                    rx: self.cfg.rx,
                    standard: p.standard,
                    field_seq: field.seq,
                    parity: field.parity,
                    field_line: k as u16,
                    frame_line,
                    damage: vec![damage::NONE; samples.len()],
                    samples,
                    quality: q,
                    src_position: 0.0,
                    src_line_len: p.samples_per_line as f64,
                    host_ns: host_ns + (k as f64 * line_ns) as u64,
                }
            })
            .collect()
    }

    /// Encodes field line `k` of `field`.
    pub fn encode_line(&self, field: &YuvField, frame_index: u64, k: usize) -> Vec<f32> {
        let p = self.p;
        let w = p.samples_per_line;
        let frame_line = p.frame_line(field.parity, k);
        let kind = self.line_kind(frame_line);
        let abs = self.abs_line(frame_index, frame_line);
        let f = p.fsc_per_sample();
        let cyc_line = f * w as f64;
        let line_phase = (cyc_line * abs as f64).fract() + self.cfg.subcarrier_phase_deg / 360.0;
        let vsw = if p.standard == VideoStandard::Pal && abs % 2 == 1 {
            -1.0
        } else {
            1.0
        };
        let tau = self.cfg.time_offset;
        let rise = p.us_to_samples(self.cfg.rise_us);
        let sync = p.sync_ire as f64;
        let mut out = vec![0.0f32; w];

        // Sync pulses of this line and the start of the next line's pulse.
        let hs = p.us_to_samples(p.hsync_us);
        let eq = p.us_to_samples(p.eq_pulse_us);
        let broad = p.us_to_samples(p.broad_pulse_us);
        let half = w as f64 / 2.0;
        let mut pulses: Vec<(f64, f64)> = match kind {
            LineKind::Normal => vec![(0.0, hs)],
            LineKind::Equalising => vec![(0.0, eq), (half, half + eq)],
            LineKind::Broad => vec![(0.0, broad), (half, half + broad)],
        };
        pulses.push((w as f64, w as f64 + hs));
        pulses.push((-(w as f64), -(w as f64) + hs));

        // Active content for this line, if any.
        let first = p.first_active_field_line(field.parity);
        let active_line = if kind == LineKind::Normal && k >= first && k - first < field.lines {
            Some(k - first)
        } else {
            None
        };
        let a = p.active_range();
        let (ry, ru, rv) = if let Some(j) = active_line {
            let wdt = field.width;
            let src_y = &field.y[j * wdt..(j + 1) * wdt];
            let src_u = &field.u[j * wdt..(j + 1) * wdt];
            let src_v = &field.v[j * wdt..(j + 1) * wdt];
            let mut ly = vec![0.0f32; wdt];
            let mut lu = vec![0.0f32; wdt];
            let mut lv = vec![0.0f32; wdt];
            fir_range(src_y, &self.luma_lp, 0..wdt, &mut ly);
            fir_range(src_u, &self.chroma_lp, 0..wdt, &mut lu);
            fir_range(src_v, &self.chroma_lp, 0..wdt, &mut lv);
            (ly, lu, lv)
        } else {
            (Vec::new(), Vec::new(), Vec::new())
        };

        let b = p.burst_range();
        let burst_on = kind == LineKind::Normal && self.cfg.color;
        let span = (p.white_ire - p.black_ire) as f64;
        for (x, o) in out.iter_mut().enumerate() {
            let t = x as f64 - tau;
            let mut v = 0.0f64;
            for &(s, e) in &pulses {
                v += sync * (edge(t - s, rise) - edge(t - e, rise));
            }
            let phase = 2.0 * PI * (f * t + line_phase);
            if burst_on {
                let env = edge(t - b.start as f64, 4.0) - edge(t - b.end as f64, 4.0);
                if env > 0.0 {
                    let amp = p.burst_ire as f64 * env;
                    v += match p.standard {
                        VideoStandard::Ntsc => -amp * phase.sin(),
                        VideoStandard::Pal => amp * (-phase.sin() + vsw * phase.cos()) / 2f64.sqrt(),
                    };
                }
            }
            if active_line.is_some() {
                let win = edge(t - a.start as f64 + 1.0, rise) - edge(t - a.end as f64 - 1.0, rise);
                if win > 0.0 {
                    let pos = (t - a.start as f64) as f32;
                    let yv = interp(&ry, pos) as f64;
                    let mut pic = yv;
                    if self.cfg.color {
                        let uv = interp(&ru, pos) as f64;
                        let vv = interp(&rv, pos) as f64;
                        pic += uv * phase.sin() + vsw * vv * phase.cos();
                    }
                    v += win * (p.black_ire as f64 + span * pic);
                }
            }
            *o = v as f32;
        }
        out
    }

    /// Encodes a run of fields from progressive pictures. `pictures[i]` is
    /// the frame-sized picture shown at field time `first_seq + i`; each
    /// field takes its own rows. Returns all line records in order.
    pub fn encode_sequence(
        &self,
        pictures: &[crate::testgen::YuvImage],
        first_seq: u64,
    ) -> Vec<LineRecord> {
        let field_ns = 1e9 / self.p.field_hz;
        let mut out = Vec::new();
        for (i, pic) in pictures.iter().enumerate() {
            let seq = first_seq + i as u64;
            let (parity, frame_index) = crate::testgen::field_info(seq);
            let fld = crate::testgen::image_to_field(pic, self.p.standard, parity, seq);
            out.extend(self.encode_field(&fld, frame_index, (seq as f64 * field_ns) as u64));
        }
        out
    }
}

/// Smooth step from 0 (t << 0) to 1 (t >> 0), raised cosine over `rise`
/// samples, 0.5 at t = 0.
#[inline]
fn edge(t: f64, rise: f64) -> f64 {
    let u = t / rise + 0.5;
    if u <= 0.0 {
        0.0
    } else if u >= 1.0 {
        1.0
    } else {
        0.5 - 0.5 * (PI * u).cos()
    }
}

#[inline]
fn interp(row: &[f32], x: f32) -> f32 {
    let n = row.len() as isize;
    let xf = x.floor();
    let t = x - xf;
    let i = xf as isize;
    let g = |k: isize| row[k.clamp(0, n - 1) as usize];
    cubic(g(i - 1), g(i), g(i + 1), g(i + 2), t)
}

/// Parity helper for callers that build fields by hand.
pub fn parity_of_seq(seq: u64) -> FieldParity {
    crate::testgen::field_info(seq).0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testgen::{color_bars, image_to_field};

    #[test]
    fn levels_and_burst() {
        for std in VideoStandard::ALL {
            let p = std.params();
            let enc = CompositeEncoder::new(std, EncoderConfig::default());
            let img = color_bars(p.active_width(), p.frame_height(), 0.75);
            let f = image_to_field(&img, std, FieldParity::First, 0);
            let recs = enc.encode_field(&f, 0, 0);
            assert_eq!(recs.len(), p.field_lines(FieldParity::First));
            let line = &recs[p.first_active_field_line(FieldParity::First) + 10].samples;
            // Sync tip and blanking.
            let tip = line[p.sync_tip_range()].iter().sum::<f32>() / p.sync_tip_range().len() as f32;
            assert!((tip - p.sync_ire).abs() < 0.1, "{std:?} tip {tip}");
            let bp = p.back_porch_range();
            for &s in &line[bp] {
                assert!(s.abs() < 0.2);
            }
            // Burst amplitude (quadrature correlation).
            let br = p.burst_range();
            let core = (br.start + 4)..(br.end - 4);
            let w = p.samples_per_line;
            let f = p.fsc_per_sample();
            let c: Vec<f32> = (0..w).map(|x| (2.0 * PI * f * x as f64).cos() as f32).collect();
            let s: Vec<f32> = (0..w).map(|x| (2.0 * PI * f * x as f64).sin() as f32).collect();
            let m = crate::color::burst::measure_burst(line, &recs[0].damage, core, &c, &s);
            assert!((m.amp() - p.burst_ire as f64).abs() < 0.3, "{std:?} burst {}", m.amp());
            // 50 % point of the sync falling edge at 0H.
            assert!((line[0] - p.sync_ire / 2.0).abs() < 0.5, "{std:?} 0H {}", line[0]);
        }
    }
}
