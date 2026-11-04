// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX doctor: offline restoration of analog FPV video.
//!
//! The main use is OpenVRX's own recordings. The decoder knows exactly which
//! samples broke up (per-sample damage masks and per-line quality), so the
//! restorer does not guess: it rebuilds only the flagged pixels from
//! motion-compensated neighbour frames on both sides and leaves every
//! unflagged pixel bit-exact (unless the optional temporal denoise is on).
//! Whole frames that are broken are rebuilt by motion-compensated
//! interpolation between the nearest good frames.
//!
//! Ordinary DVR files go through the same engine with a blind damage
//! estimate (noise line bursts, sparkles, static and mute frames) in place
//! of the mask, decoded and re-encoded with ffmpeg (x264, audio kept).
//!
//! Entry points:
//! - [`restore_frames`] / [`restore_fields`]: in-memory sequences of
//!   [`Frame`] or [`YuvField`] with their damage masks.
//! - [`Restorer::run`]: streaming over any [`FrameSource`] with bounded
//!   memory.
//! - [`ffmpeg::restore_file`]: a video file in, a video file out.
//!
//! Measured on synthetic sequences (panning texture, rotating disc, moving
//! text; 320x240, 24 frames; see `tests/mask_restore.rs`), mask-driven,
//! luma PSNR / SSIM against ground truth:
//!
//! | damage                        | damaged         | fast            | balanced        | max             |
//! |-------------------------------|-----------------|-----------------|-----------------|-----------------|
//! | noise line bursts + sparkles  | 28.6 dB / 0.951 | 40.3 dB / 0.997 | 40.9 dB / 0.997 | 40.2 dB / 0.997 |
//! | same plus 3 lost frames       | 18.6 dB / 0.836 | 32.4 dB / 0.988 | 32.0 dB / 0.988 | 33.1 dB / 0.989 |

pub mod config;
pub mod damage;
pub mod engine;
pub mod ffmpeg;
pub mod flow;
pub mod picture;

pub use config::{DamageSource, EncodeConfig, RestoreConfig};
pub use engine::{Engine, PictureStats};
pub use flow::{Flow, FlowParams};
pub use picture::Picture;

use openvrx_core::{damage as dmg, Frame, LineFlags, LineQuality, YuvField};
use std::collections::VecDeque;
use std::time::Instant;

/// Errors from restoration.
#[derive(Debug, thiserror::Error)]
pub enum RestoreError {
    #[error("configuration: {0}")]
    Config(String),
    #[error("frame size changed from {0}x{1} to {2}x{3}")]
    SizeChanged(usize, usize, usize, usize),
    #[error("i/o: {0}")]
    Io(#[from] std::io::Error),
    #[error("ffmpeg: {0}")]
    Ffmpeg(String),
}

/// Anything that yields frames in display order.
pub trait FrameSource {
    fn next_frame(&mut self) -> Result<Option<Frame>, RestoreError>;
    /// Number of frames, when known (for progress).
    fn len_hint(&self) -> Option<usize> {
        None
    }
}

/// Adapts an iterator of frames.
pub struct IterSource<I> {
    iter: I,
    len: Option<usize>,
}

impl<I: Iterator<Item = Frame>> IterSource<I> {
    pub fn new(iter: I) -> Self {
        let (lo, hi) = iter.size_hint();
        IterSource {
            len: (Some(lo) == hi).then_some(lo),
            iter,
        }
    }
}

impl<I: Iterator<Item = Frame>> FrameSource for IterSource<I> {
    fn next_frame(&mut self) -> Result<Option<Frame>, RestoreError> {
        Ok(self.iter.next())
    }
    fn len_hint(&self) -> Option<usize> {
        self.len
    }
}

/// Progress report passed to callbacks.
#[derive(Clone, Debug)]
pub struct Progress {
    pub done: usize,
    pub total: Option<usize>,
    /// Output frames per second so far.
    pub fps: f64,
}

/// Result of a run.
#[derive(Clone, Debug, Default)]
pub struct RunSummary {
    pub frames: usize,
    pub seconds: f64,
    pub stats: Vec<PictureStats>,
}

impl RunSummary {
    pub fn fps(&self) -> f64 {
        if self.seconds > 0.0 {
            self.frames as f64 / self.seconds
        } else {
            0.0
        }
    }
}

/// Builds the engine picture (with confidence) for one frame's planes.
fn to_picture(
    cfg: &RestoreConfig,
    width: usize,
    height: usize,
    planes: (&[f32], &[f32], &[f32]),
    mask: &[u8],
    rows: &[LineQuality],
) -> Picture {
    let mut p = Picture {
        width,
        height,
        y: planes.0.to_vec(),
        u: planes.1.to_vec(),
        v: planes.2.to_vec(),
        conf: vec![1.0; width * height],
    };
    let use_mask = match cfg.damage_source {
        DamageSource::Mask | DamageSource::MaskAndBlind => true,
        DamageSource::Blind => false,
        DamageSource::Auto => damage::has_damage_info(mask, rows),
    };
    let use_blind = match cfg.damage_source {
        DamageSource::Blind | DamageSource::MaskAndBlind => true,
        DamageSource::Mask => false,
        DamageSource::Auto => !use_mask,
    };
    if use_mask {
        damage::apply_mask(&mut p, mask, Some(rows));
    }
    if use_blind {
        damage::blind_estimate(&mut p, cfg.blind_sensitivity);
    }
    p
}

/// Writes restored pixels back and updates the damage bookkeeping:
/// rebuilt pixels lose their damage mark, rows with changes get the
/// `REPAIRED` flag.
fn write_back(
    pic: &Picture,
    y: &mut [f32],
    u: &mut [f32],
    v: &mut [f32],
    mask: &mut [u8],
    rows: &mut [LineQuality],
) {
    let w = pic.width;
    for r in 0..pic.height {
        let mut changed = false;
        for x in 0..w {
            let i = r * w + x;
            if pic.y[i] != y[i] || pic.u[i] != u[i] || pic.v[i] != v[i] {
                changed = true;
            }
            if pic.conf[i] >= 1.0 {
                if let Some(m) = mask.get_mut(i) {
                    *m = dmg::NONE;
                }
            }
        }
        if changed {
            if let Some(q) = rows.get_mut(r) {
                q.flags.insert(LineFlags::REPAIRED);
            }
        }
    }
    y.copy_from_slice(&pic.y);
    u.copy_from_slice(&pic.u);
    v.copy_from_slice(&pic.v);
}

/// Streaming restorer over frames.
pub struct Restorer {
    cfg: RestoreConfig,
}

impl Restorer {
    pub fn new(cfg: RestoreConfig) -> Result<Restorer, RestoreError> {
        cfg.validate()?;
        Ok(Restorer { cfg })
    }

    pub fn config(&self) -> &RestoreConfig {
        &self.cfg
    }

    /// Restores every frame of `src` in order, handing each result to
    /// `sink`. Memory stays bounded by the engine window.
    pub fn run<S, F, P>(&self, src: &mut S, mut sink: F, mut progress: P) -> Result<RunSummary, RestoreError>
    where
        S: FrameSource + ?Sized,
        F: FnMut(Frame, &PictureStats) -> Result<(), RestoreError>,
        P: FnMut(&Progress),
    {
        let t0 = Instant::now();
        let total = src.len_hint();
        let mut engine = Engine::new(self.cfg.clone(), 1);
        let mut pending: VecDeque<Frame> = VecDeque::new();
        let mut summary = RunSummary::default();
        let mut size: Option<(usize, usize)> = None;
        let mut emit = |ready: Vec<(Picture, PictureStats)>,
                        pending: &mut VecDeque<Frame>,
                        summary: &mut RunSummary|
         -> Result<(), RestoreError> {
            for (pic, st) in ready {
                let mut f = pending.pop_front().expect("frame metadata queued");
                f.y = vec![0.0; pic.len()];
                f.u = vec![0.0; pic.len()];
                f.v = vec![0.0; pic.len()];
                write_back(&pic, &mut f.y, &mut f.u, &mut f.v, &mut f.damage, &mut f.row_quality);
                if !f.meta.pipeline.is_empty() {
                    f.meta.pipeline.push('+');
                }
                f.meta.pipeline.push_str("doctor");
                sink(f, &st)?;
                summary.stats.push(st);
                summary.frames += 1;
                progress(&Progress {
                    done: summary.frames,
                    total,
                    fps: summary.frames as f64 / t0.elapsed().as_secs_f64().max(1e-9),
                });
            }
            Ok(())
        };
        while let Some(mut f) = src.next_frame()? {
            match size {
                None => size = Some((f.width, f.height)),
                Some((w, h)) if (w, h) != (f.width, f.height) => {
                    return Err(RestoreError::SizeChanged(w, h, f.width, f.height));
                }
                _ => {}
            }
            let pic = to_picture(
                &self.cfg,
                f.width,
                f.height,
                (&f.y, &f.u, &f.v),
                &f.damage,
                &f.row_quality,
            );
            f.y = Vec::new();
            f.u = Vec::new();
            f.v = Vec::new();
            pending.push_back(f);
            let ready = engine.push(pic, 0);
            emit(ready, &mut pending, &mut summary)?;
        }
        let ready = engine.finish();
        emit(ready, &mut pending, &mut summary)?;
        summary.seconds = t0.elapsed().as_secs_f64();
        Ok(summary)
    }
}

/// Restores an in-memory frame sequence (damage masks and row quality are
/// used as configured by `cfg.damage_source`).
pub fn restore_frames(frames: &[Frame], cfg: &RestoreConfig) -> Result<Vec<Frame>, RestoreError> {
    let r = Restorer::new(cfg.clone())?;
    let mut out = Vec::with_capacity(frames.len());
    let mut src = IterSource::new(frames.iter().cloned());
    r.run(
        &mut src,
        |f, _| {
            out.push(f);
            Ok(())
        },
        |_| {},
    )?;
    Ok(out)
}

/// Restores a sequence of alternating fields straight from the decoder.
/// Neighbours are taken from fields of the same parity, so no vertical
/// half-line offset enters the motion compensation.
pub fn restore_fields(fields: &[YuvField], cfg: &RestoreConfig) -> Result<Vec<YuvField>, RestoreError> {
    cfg.validate()?;
    let mut engine = Engine::new(cfg.clone(), 2);
    let mut out: Vec<YuvField> = Vec::with_capacity(fields.len());
    let mut done = Vec::with_capacity(fields.len());
    for f in fields {
        if let Some(first) = fields.first() {
            if (first.width, first.lines) != (f.width, f.lines) {
                return Err(RestoreError::SizeChanged(first.width, first.lines, f.width, f.lines));
            }
        }
        let pic = to_picture(cfg, f.width, f.lines, (&f.y, &f.u, &f.v), &f.damage, &f.quality);
        done.extend(engine.push(pic, f.parity.index() as u8));
    }
    done.extend(engine.finish());
    for ((pic, _), src) in done.into_iter().zip(fields) {
        let mut f = src.clone();
        write_back(&pic, &mut f.y, &mut f.u, &mut f.v, &mut f.damage, &mut f.quality);
        out.push(f);
    }
    Ok(out)
}
