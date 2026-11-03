// SPDX-License-Identifier: GPL-3.0-or-later
//! Transmitter: picture -> composite on the line-locked grid -> FM at the
//! 80 MS/s MODEM_DIAG bus rate.
//!
//! Time base: transmitter sample `k` is `k / 80 MHz` of the camera's own
//! clock; frame 0 line 1 0H is `k = 0`. The composite grid relates to it by
//! an exact fraction (NTSC 63/352, PAL 227/1024 grid samples per
//! transmitter sample), so interpolation phases are exact.
//!
//! FM model (after C5VRX sim.py, extended): instantaneous frequency =
//! blanking offset + carrier error (constant, drift, wander) + deviation x
//! 6.7 MHz x (modulator curve of the normalised video), where the video
//! optionally passes a pre-emphasis shelf first and the modulator curve
//! has a mild bow (integral nonlinearity). Phase is the trapezoidal
//! integral of frequency, so a cubic Hermite interpolation with the
//! frequency as derivative gives the phase at any fractional position
//! (receiver clocks, multipath delays).

use crate::config::{SimConfig, DEV_PP_HZ};
use crate::cvbs::{field_index, line_position, Encoder};
use crate::dsp::{Cascade, IirState, Polyphase};
use crate::picture::{self, Geometry, PictureSource, Yuv};
use crate::SimError;
use openvrx_core::{StandardParams, VideoStandard};
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::sync::Arc;

/// MODEM_DIAG bus rate.
pub const BUS_HZ: f64 = 80e6;
/// Interpolator taps (even).
const TAPS: usize = 16;
/// Pre-emphasis warm-up in transmitter samples.
const PRE_WARM: u64 = 512;
/// Parallel work unit.
const SUB: usize = 1 << 16;

/// Contiguous composite grid samples from absolute grid index `g0`.
pub struct GridSlice {
    pub g0: u64,
    pub data: Vec<f32>,
}

impl GridSlice {
    /// Interpolated composite (IRE) at fractional grid position `g`.
    #[inline]
    pub fn value_at(&self, pp: &Polyphase, g: f64) -> f32 {
        let gi = crate::dsp::ffloor(g);
        let w = pp.weights(g - gi);
        let start = gi as i64 - (TAPS as i64 / 2 - 1) - self.g0 as i64;
        let start = start.max(0) as usize;
        let src = &self.data[start..start + TAPS];
        w.iter().zip(src).map(|(a, b)| a * b).sum()
    }
}

/// One block of transmitter output, indices `[a, a + len)`.
pub struct TxBlock {
    pub a: u64,
    /// Instantaneous frequency in cycles per bus sample.
    pub freq: Vec<f32>,
    /// Phase in cycles (relative: only differences and fractions matter).
    pub phase: Vec<f64>,
    pub grid: GridSlice,
}

impl TxBlock {
    pub fn end(&self) -> u64 {
        self.a + self.freq.len() as u64
    }

    /// Phase (cycles) at fractional transmitter position `p`.
    #[inline(always)]
    pub fn phase_at(&self, p: f64) -> f64 {
        let k = crate::dsp::ffloor(p);
        let u = p - k;
        let i = (k as u64 - self.a) as usize;
        let (p0, p1) = (self.phase[i], self.phase[i + 1]);
        let (d0, d1) = (self.freq[i] as f64, self.freq[i + 1] as f64);
        let u2 = u * u;
        let u3 = u2 * u;
        p0 + u * d0 + u2 * (3.0 * (p1 - p0) - 2.0 * d0 - d1) + u3 * (2.0 * (p0 - p1) + d0 + d1)
    }
}

/// The transmitter: picture source, encoder, caches and FM parameters.
pub struct Transmitter {
    pub enc: Encoder,
    p: &'static StandardParams,
    /// Behind a mutex so the transmitter is Sync (only used from the
    /// engine thread).
    picture: std::sync::Mutex<Box<dyn PictureSource>>,
    geo: Geometry,
    pub pp: Polyphase,
    /// Grid samples per transmitter sample as num / den.
    grid_num: u128,
    grid_den: u128,
    fields: BTreeMap<u64, Arc<Vec<Yuv>>>,
    lines: BTreeMap<u64, Arc<Vec<f32>>>,
    dev_hz: f64,
    blank_hz: f64,
    err_hz: f64,
    drift: f64,
    wander_hz: f64,
    wander_period: f64,
    pre: Option<Cascade>,
    nl2: f64,
    nl3: f64,
    sync_ire: f64,
    span_ire: f64,
    w_blank: f64,
}

impl Transmitter {
    pub fn new(cfg: &SimConfig) -> Result<Transmitter, SimError> {
        let std = cfg.standard;
        let p = std.params();
        let geo = Geometry::of(std);
        let picture = picture::build(&cfg.picture, cfg.osd, geo)?;
        let (grid_num, grid_den) = match std {
            VideoStandard::Ntsc => (63, 352),
            VideoStandard::Pal => (227, 1024),
        };
        let pre = cfg.vtx.pre_emphasis.as_ref().map(|pe| {
            let fz = pe.zero_hz / BUS_HZ;
            let fp = pe.pole_hz / BUS_HZ;
            let unit = Cascade::shelf(fz, fp, 1.0);
            let g = 1.0 / unit.magnitude(pe.crossover_hz / BUS_HZ);
            Cascade::shelf(fz, fp, g)
        });
        let sync_ire = p.sync_ire as f64;
        let span_ire = (p.white_ire - p.sync_ire) as f64;
        let mut t = Transmitter {
            enc: Encoder::new(std, &cfg.cvbs, crate::dsp::mix_seed(&[cfg.seed, 0xC4B5])),
            p,
            picture: std::sync::Mutex::new(picture),
            geo,
            // Cutoff 0.42 of the grid rate: 6.0 MHz NTSC, 7.4 MHz PAL.
            pp: Polyphase::new(TAPS, 512, 0.42, 7.0),
            grid_num,
            grid_den,
            fields: BTreeMap::new(),
            lines: BTreeMap::new(),
            dev_hz: cfg.vtx.deviation * DEV_PP_HZ,
            blank_hz: cfg.vtx.blank_hz,
            err_hz: cfg.vtx.freq_error_hz,
            drift: cfg.vtx.drift_hz_per_s,
            wander_hz: cfg.vtx.wander_hz,
            wander_period: cfg.vtx.wander_period_s.max(1e-3),
            pre,
            nl2: cfg.vtx.nonlinearity,
            nl3: cfg.vtx.nonlinearity3,
            sync_ire,
            span_ire,
            w_blank: 0.0,
        };
        t.w_blank = t.curve(-sync_ire / span_ire);
        Ok(t)
    }

    pub fn params(&self) -> &'static StandardParams {
        self.p
    }

    /// Grid position (fractional) of a transmitter position.
    #[inline]
    pub fn grid_pos(&self, p: f64) -> f64 {
        p * self.grid_num as f64 / self.grid_den as f64
    }

    /// Transmitter samples per line.
    pub fn line_period_tx(&self) -> f64 {
        BUS_HZ / self.p.line_hz
    }

    /// Transmitter position where field `f`'s vertical interval starts
    /// (first equalising or broad pulse).
    pub fn field_start_tx(&self, f: u64) -> f64 {
        let half_lines = self.p.lines_per_frame as f64 / 2.0;
        let offset = match self.p.standard {
            VideoStandard::Ntsc => 0.0,
            VideoStandard::Pal => -1.5,
        };
        (f as f64 * half_lines + offset) * self.line_period_tx()
    }

    /// Modulator transfer curve (normalised video 0 = sync tip, 1 = white).
    #[inline]
    fn curve(&self, u: f64) -> f64 {
        u + 4.0 * self.nl2 * u * (1.0 - u) + 10.39 * self.nl3 * u * (1.0 - u) * (1.0 - 2.0 * u)
    }

    /// Carrier offset (error, drift, wander) at transmitter time `t`.
    #[inline]
    pub fn carrier_hz(&self, t: f64) -> f64 {
        self.err_hz
            + self.drift * t
            + self.wander_hz * (std::f64::consts::TAU * t / self.wander_period).sin()
    }

    /// Frequency of the blanking level at time 0 (relative to the channel).
    pub fn blank_freq_hz(&self) -> f64 {
        self.blank_hz + self.carrier_hz(0.0)
    }

    /// Instantaneous frequency in Hz for video `v` (IRE, after any
    /// pre-emphasis) at time `t`.
    #[inline]
    pub fn freq_hz(&self, v: f64, t: f64) -> f64 {
        self.video_freq_hz(v) + self.carrier_hz(t)
    }

    /// Frequency without the carrier error terms.
    #[inline(always)]
    fn video_freq_hz(&self, v: f64) -> f64 {
        let u = (v - self.sync_ire) / self.span_ire;
        self.blank_hz + self.dev_hz * (self.curve(u) - self.w_blank)
    }

    fn field_time(&self, f: u64) -> f64 {
        f as f64 * (self.p.lines_per_frame as f64 / 2.0) / self.p.line_hz
    }

    /// Makes sure lines `[l0, l1]` are rendered; drops older ones.
    fn ensure_lines(&mut self, l0: u64, l1: u64) -> Result<(), SimError> {
        let p = self.p;
        let fields_needed: Vec<u64> = {
            let mut v: Vec<u64> = (l0..=l1).map(|l| field_index(p, l)).collect();
            v.dedup();
            v
        };
        for f in fields_needed {
            if self.fields.contains_key(&f) {
                continue;
            }
            let parity = if f % 2 == 0 {
                openvrx_core::FieldParity::First
            } else {
                openvrx_core::FieldParity::Second
            };
            let off = p.field_row_offset(parity);
            let rows: Vec<usize> = (0..p.active_lines_per_field as usize).map(|k| 2 * k + off).collect();
            let mut img = vec![Yuv::default(); rows.len() * self.geo.width];
            let t = self.field_time(f);
            self.picture
                .get_mut()
                .unwrap_or_else(|e| e.into_inner())
                .render(t, &rows, &mut img)?;
            self.fields.insert(f, Arc::new(img));
        }
        let missing: Vec<u64> = (l0..=l1).filter(|l| !self.lines.contains_key(l)).collect();
        let spl = p.samples_per_line;
        let rendered: Vec<(u64, Arc<Vec<f32>>)> = missing
            .par_iter()
            .map(|&l| {
                let (_, fl, _) = line_position(p, l);
                let field = self.fields.get(&field_index(p, l)).map(|a| a.as_slice());
                let visible = crate::cvbs::field_row(p, fl).is_some();
                let mut out = vec![0.0f32; spl];
                self.enc.render_line(l, if visible { field } else { None }, &mut out);
                (l, Arc::new(out))
            })
            .collect();
        self.lines.extend(rendered);
        self.lines = self.lines.split_off(&l0);
        let f0 = field_index(p, l0);
        self.fields = self.fields.split_off(&f0);
        Ok(())
    }

    /// Transmitter output for `[a, b)`, with `phase[a] = phase_a`.
    pub fn block(&mut self, a: u64, b: u64, phase_a: f64) -> Result<TxBlock, SimError> {
        assert!(a >= PRE_WARM + 64 && b > a);
        let spl = self.p.samples_per_line as u64;
        let lo = a - PRE_WARM;
        let g_lo = ((lo as u128 * self.grid_num) / self.grid_den) as u64 - TAPS as u64;
        let g_hi = ((b as u128 * self.grid_num) / self.grid_den) as u64 + TAPS as u64 + 2;
        let (l0, l1) = (g_lo / spl, g_hi / spl);
        self.ensure_lines(l0, l1)?;
        let mut data = Vec::with_capacity(((l1 - l0 + 1) * spl) as usize);
        for l in l0..=l1 {
            data.extend_from_slice(&self.lines[&l]);
        }
        let grid = GridSlice {
            g0: l0 * spl,
            data,
        };

        // Video at every transmitter sample in [lo, b).
        let n = (b - lo) as usize;
        let mut video = vec![0.0f32; n];
        let (num, den) = (self.grid_num, self.grid_den);
        let pp = &self.pp;
        let (num64, den64) = (num as u64, den as u64);
        video.par_chunks_mut(SUB).enumerate().for_each(|(c, out)| {
            let k0 = lo + (c * SUB) as u64;
            let t = k0 as u128 * num;
            let mut gi = (t / den) as u64;
            let mut rem = (t % den) as u64;
            let inv = 1.0 / den as f64;
            for o in out.iter_mut() {
                let w = pp.weights(rem as f64 * inv);
                let s = (gi - (TAPS as u64 / 2 - 1) - grid.g0) as usize;
                let src = &grid.data[s..s + TAPS];
                let mut acc = [0.0f32; 4];
                for j in 0..TAPS / 4 {
                    for (q, a) in acc.iter_mut().enumerate() {
                        *a += w[4 * j + q] * src[4 * j + q];
                    }
                }
                *o = (acc[0] + acc[1]) + (acc[2] + acc[3]);
                rem += num64;
                while rem >= den64 {
                    rem -= den64;
                    gi += 1;
                }
            }
        });

        // Pre-emphasis (independent warm-up per work unit) and frequency.
        let warm = PRE_WARM as usize;
        let m = (b - a) as usize;
        let mut freq = vec![0.0f32; m];
        let this = &*self;
        freq.par_chunks_mut(SUB).enumerate().for_each(|(c, out)| {
            let i0 = c * SUB; // index relative to a
            let mut st = this.pre.as_ref().map(IirState::new);
            if let Some(st) = st.as_mut() {
                // video index of a + i0 is warm + i0; warm up on the
                // preceding `warm` samples.
                for &v in &video[i0..warm + i0] {
                    st.step(v);
                }
            }
            // Carrier offset changes slowly: evaluate it every 4096
            // samples and interpolate.
            let n = out.len();
            let k_first = a + i0 as u64;
            let car = |j: usize| this.carrier_hz((k_first + j as u64) as f64 / BUS_HZ);
            let mut seg_c0 = car(0);
            let mut seg_c1 = car(4096.min(n));
            for (j, o) in out.iter_mut().enumerate() {
                if j % 4096 == 0 && j > 0 {
                    seg_c0 = seg_c1;
                    seg_c1 = car((j + 4096).min(n));
                }
                let mut v = video[warm + i0 + j];
                if let Some(st) = st.as_mut() {
                    v = st.step(v);
                }
                let f = (j % 4096) as f64 / 4096.0;
                let c = seg_c0 + (seg_c1 - seg_c0) * f;
                *o = ((this.video_freq_hz(v as f64) + c) * (1.0 / BUS_HZ)) as f32;
            }
        });

        // Phase: trapezoidal integral, parallel prefix over work units.
        let mut phase = vec![0.0f64; m];
        let ends: Vec<f64> = phase
            .par_chunks_mut(SUB)
            .zip(freq.par_chunks(SUB))
            .map(|(ph, f)| {
                let mut acc = 0.0f64;
                ph[0] = 0.0;
                for j in 1..f.len() {
                    acc += 0.5 * (f[j - 1] as f64 + f[j] as f64);
                    ph[j] = acc;
                }
                acc
            })
            .collect();
        let mut offsets = Vec::with_capacity(ends.len());
        let mut off = phase_a;
        for (c, e) in ends.iter().enumerate() {
            offsets.push(off);
            let last = ((c + 1) * SUB).min(m) - 1;
            let next = (last + 1).min(m - 1);
            off += e + 0.5 * (freq[last] as f64 + freq[next] as f64);
        }
        phase
            .par_chunks_mut(SUB)
            .zip(offsets.par_iter())
            .for_each(|(ph, &o)| ph.iter_mut().for_each(|x| *x += o));
        Ok(TxBlock {
            a,
            freq,
            phase,
            grid,
        })
    }

    /// Composite truth (IRE) at fractional transmitter position `p` from a
    /// block's grid.
    #[inline]
    pub fn truth_at(&self, block: &TxBlock, p: f64) -> f32 {
        block.grid.value_at(&self.pp, self.grid_pos(p))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PictureConfig;

    fn tx(std: VideoStandard) -> Transmitter {
        let cfg = SimConfig {
            standard: std,
            picture: PictureConfig::Flat { y: 1.0, u: 0.0, v: 0.0 },
            vtx: crate::config::VtxConfig {
                nonlinearity: 0.0,
                ..Default::default()
            },
            ..Default::default()
        };
        Transmitter::new(&cfg).unwrap()
    }

    #[test]
    fn grid_ratio_exact() {
        for std in VideoStandard::ALL {
            let t = tx(std);
            let p = std.params();
            let r = t.grid_num as f64 / t.grid_den as f64;
            assert!((r - p.grid_rate_hz() / BUS_HZ).abs() < 1e-12);
        }
    }

    #[test]
    fn deviation_levels() {
        let t = tx(VideoStandard::Ntsc);
        let p = &openvrx_core::standard::NTSC;
        let f_sync = t.freq_hz(p.sync_ire as f64, 0.0);
        let f_blank = t.freq_hz(0.0, 0.0);
        let f_white = t.freq_hz(100.0, 0.0);
        assert!((f_blank + 1.0e6).abs() < 1.0);
        assert!((f_white - f_sync - 6.7e6).abs() < 1.0);
    }

    #[test]
    fn block_phase_is_continuous_and_matches_frequency() {
        let mut t = tx(VideoStandard::Pal);
        let a = 100_000;
        let b1 = t.block(a, a + 300_000, 0.25).unwrap();
        assert_eq!(b1.phase[0], 0.25);
        // Phase differences follow the trapezoid rule across work units.
        for i in [1usize, 65535, 65536, 65537, 200_000] {
            let d = b1.phase[i] - b1.phase[i - 1];
            let e = 0.5 * (b1.freq[i] as f64 + b1.freq[i - 1] as f64);
            assert!((d - e).abs() < 1e-9, "{i}");
        }
        // A second block over an overlapping range agrees.
        let start = a + 200_000;
        let b2 = t.block(start, start + 100_000, b1.phase[200_000]).unwrap();
        for j in [0usize, 10, 50_000, 99_000] {
            assert!((b2.phase[j] - b1.phase[200_000 + j]).abs() < 1e-6);
        }
        // Hermite interpolation hits the nodes and stays between.
        let p = (a + 1234) as f64;
        assert!((b1.phase_at(p) - b1.phase[1234]).abs() < 1e-12);
        let mid = b1.phase_at(p + 0.5);
        assert!(mid > b1.phase[1234].min(b1.phase[1235]) - 0.1);
    }

    #[test]
    fn field_starts() {
        let t = tx(VideoStandard::Ntsc);
        assert_eq!(t.field_start_tx(0), 0.0);
        assert!((t.field_start_tx(1) - 262.5 * t.line_period_tx()).abs() < 1e-6);
    }
}
