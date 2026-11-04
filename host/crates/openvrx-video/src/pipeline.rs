// SPDX-License-Identifier: GPL-3.0-or-later
//! The video pipeline: combiner -> field assembly -> colour decode ->
//! repair -> deinterlace -> denoise, with a low-latency streaming mode.
//!
//! Streaming (`push_line`): every stage advances as far as its inputs
//! allow after each line, so the current frame's rows are updated a few
//! lines behind the newest received line (colour decode needs at most the
//! comb's lookahead, repair two lines, deinterlacing two field lines,
//! denoising one or two rows). Rows not yet updated still hold the previous
//! frame, which is what a beam-racing display wants to show.
//!
//! Field mode (`push_field`): the same per-line work over a whole field,
//! spread over threads. Every stage is deterministic per line, so both
//! modes produce bit-identical frames.

use crate::assembler::{AssemblerStats, FieldAssembler};
use crate::color::{ColorConfig, ColorDecoder, NtscSeparation, PalSeparation};
use crate::combiner::{CombineMode, Combiner, CombinerConfig, CombinerStats};
use crate::deinterlace::{self, BidiDeinterlacer, DeinterlaceMethod, Deinterlacer, PastDeinterlacer};
use crate::denoise::{DenoiseConfig, Denoiser, TemporalMode};
use crate::repair::{RepairConfig, Repairer};
use openvrx_core::{Field, Frame, LineRecord, VideoStandard, YuvField};
use serde::{Deserialize, Serialize};
use std::ops::Range;
use std::sync::Arc;
use std::time::Instant;

/// Complete pipeline settings (serde-serialisable).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VideoConfig {
    pub standard: VideoStandard,
    pub combiner: CombinerConfig,
    pub color: ColorConfig,
    /// Damage-mask concealment (C1/C2).
    pub repair_enabled: bool,
    pub repair: RepairConfig,
    pub deinterlace: DeinterlaceMethod,
    pub denoise: DenoiseConfig,
    /// Update the current frame line by line as lines arrive.
    pub streaming: bool,
    /// Streaming: run the stages every this many received lines (1 = after
    /// every line). Batches below `STREAM_PARALLEL_MIN_LINES` run on the
    /// calling thread: waking worker threads for a few lines costs more
    /// than the work itself.
    pub stream_batch_lines: usize,
    /// Use worker threads for whole-field work.
    pub parallel: bool,
    /// `push_line` returns each finished frame in
    /// `PipelineOutput::frames_completed` (a copy of the frame, which for
    /// PAL costs about 1.5 ms per field in memory allocation and copying).
    /// A live display that reads `current_frame()` and only needs to know
    /// that a frame finished can turn this off and use
    /// `PipelineOutput::frames_done`. `push_field` and `flush` always
    /// return their frames.
    pub emit_frames: bool,
}

impl Default for VideoConfig {
    fn default() -> Self {
        VideoConfig::balanced(VideoStandard::Pal)
    }
}

impl VideoConfig {
    /// Lowest latency with a clean picture: 2D comb, repair, edge-directed
    /// motion-adaptive deinterlacing, no temporal filtering.
    pub fn low_latency(standard: VideoStandard) -> VideoConfig {
        VideoConfig {
            standard,
            combiner: CombinerConfig {
                max_wait_lines: 4,
                ..CombinerConfig::default()
            },
            color: ColorConfig {
                ntsc: NtscSeparation::Comb2D,
                pal: PalSeparation::Comb2D,
                ..ColorConfig::default()
            },
            repair_enabled: true,
            repair: RepairConfig::default(),
            deinterlace: DeinterlaceMethod::EdgeDirectedMA,
            denoise: DenoiseConfig::off(),
            streaming: true,
            stream_batch_lines: 8,
            parallel: true,
            emit_frames: true,
        }
    }

    /// Live default: as `low_latency` plus a light motion-gated recursive
    /// filter.
    pub fn balanced(standard: VideoStandard) -> VideoConfig {
        VideoConfig {
            combiner: CombinerConfig::default(),
            denoise: DenoiseConfig {
                temporal: TemporalMode::Recursive,
                luma_strength: 0.5,
                chroma_strength: 0.7,
                ..DenoiseConfig::default()
            },
            ..VideoConfig::low_latency(standard)
        }
    }

    /// Best live picture, still zero added delay: 3D comb (NTSC) or 3D
    /// transform (PAL), motion-compensated recursive denoising.
    pub fn quality(standard: VideoStandard) -> VideoConfig {
        VideoConfig {
            color: ColorConfig {
                ntsc: NtscSeparation::Comb3D,
                pal: PalSeparation::Transform3D,
                ..ColorConfig::default()
            },
            denoise: DenoiseConfig {
                temporal: TemporalMode::MotionCompensated,
                luma_strength: 0.7,
                chroma_strength: 0.85,
                ..DenoiseConfig::default()
            },
            // Too heavy for one thread: larger batches spread over workers
            // (about 2 ms of added latency).
            stream_batch_lines: STREAM_PARALLEL_MIN_LINES,
            ..VideoConfig::balanced(standard)
        }
    }

    /// All cleanup off: plain blend of receivers, no repair, no temporal
    /// processing, bob deinterlacing. The pilot sees exactly what arrived.
    pub fn raw(standard: VideoStandard) -> VideoConfig {
        VideoConfig {
            standard,
            combiner: CombinerConfig {
                mode: CombineMode::Blend,
                ..CombinerConfig::default()
            },
            color: ColorConfig::default(),
            repair_enabled: false,
            repair: RepairConfig::default(),
            deinterlace: DeinterlaceMethod::Bob,
            denoise: DenoiseConfig::off(),
            streaming: true,
            stream_batch_lines: 8,
            parallel: true,
            emit_frames: true,
        }
    }

    /// Preset by name (`low_latency`, `balanced`, `quality`, `raw`).
    pub fn preset(name: &str, standard: VideoStandard) -> Option<VideoConfig> {
        match name {
            "low_latency" => Some(VideoConfig::low_latency(standard)),
            "balanced" => Some(VideoConfig::balanced(standard)),
            "quality" => Some(VideoConfig::quality(standard)),
            "raw" => Some(VideoConfig::raw(standard)),
            _ => None,
        }
    }
}

/// Result of one `push_line`.
#[derive(Debug, Default)]
pub struct PipelineOutput {
    /// Frames finished by this push (normally zero or one).
    pub frames_completed: Vec<Frame>,
    /// Frames finished by this push, including those not returned because
    /// `VideoConfig::emit_frames` is off.
    pub frames_done: usize,
    /// Rows of `current_frame()` changed by this push.
    pub updated_rows: Option<Range<usize>>,
}

/// Per-stage processing time of one field (nanoseconds of CPU time spent
/// in each stage, summed over all calls that worked on the field).
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct StageTimings {
    pub combine_ns: u64,
    pub color_ns: u64,
    pub repair_ns: u64,
    pub deinterlace_ns: u64,
    pub denoise_ns: u64,
}

impl StageTimings {
    pub fn total_ns(&self) -> u64 {
        self.combine_ns + self.color_ns + self.repair_ns + self.deinterlace_ns + self.denoise_ns
    }
}

/// Counters for the OSD.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PipelineStats {
    pub combiner: CombinerStats,
    pub assembler: AssemblerStats,
    pub repair: crate::repair::RepairStats,
    pub frames: u64,
    /// Last field's stage times and an exponential average.
    pub last: StageTimings,
    pub average: StageTimings,
}

enum Deint {
    Past(PastDeinterlacer),
    Bidi(BidiDeinterlacer),
}

/// Streaming work of the field being received.
struct Work {
    seq: u64,
    parity: openvrx_core::FieldParity,
    yuv: YuvField,
    rep: YuvField,
    yuv_ready: usize,
    rep_ready: usize,
    di_rows: usize,
    dn_rows: usize,
}

pub struct VideoPipeline {
    cfg: VideoConfig,
    combiner: Combiner,
    assembler: FieldAssembler,
    color: ColorDecoder,
    repair: Repairer,
    deint: Deint,
    denoise: Denoiser,
    work: Option<Work>,
    /// Colour output buffer of the previous field, reused.
    spare_yuv: Option<YuvField>,
    /// Repaired fields handed to the repairer and deinterlacer histories;
    /// one no longer referenced there is reused for a new field.
    rep_pool: Vec<Arc<YuvField>>,
    pending_lines: usize,
    di_frame: Frame,
    current: Frame,
    frames: u64,
    timing: StageTimings,
    stats: PipelineStats,
    name: String,
}

/// Smallest streaming batch (lines) spread over worker threads.
pub const STREAM_PARALLEL_MIN_LINES: usize = 32;

fn ns(t: Instant) -> u64 {
    t.elapsed().as_nanos() as u64
}

impl VideoPipeline {
    pub fn new(cfg: &VideoConfig) -> VideoPipeline {
        let std = cfg.standard;
        let p = std.params();
        let (w, h) = (p.active_width(), p.frame_height());
        let color = ColorDecoder::new(std, &cfg.color);
        let deint = if cfg.deinterlace.is_bidirectional() {
            Deint::Bidi(BidiDeinterlacer::new(cfg.deinterlace, std))
        } else {
            Deint::Past(PastDeinterlacer::new(cfg.deinterlace, std))
        };
        let name = pipeline_name(cfg, &color);
        VideoPipeline {
            combiner: Combiner::new(std, &cfg.combiner),
            assembler: FieldAssembler::new(std),
            color,
            repair: Repairer::new(std, &cfg.repair),
            deint,
            denoise: Denoiser::new(std, &cfg.denoise),
            work: None,
            spare_yuv: None,
            rep_pool: Vec::new(),
            pending_lines: 0,
            di_frame: Frame::new(w, h),
            current: Frame::new(w, h),
            frames: 0,
            timing: StageTimings::default(),
            stats: PipelineStats::default(),
            name,
            cfg: cfg.clone(),
        }
    }

    pub fn config(&self) -> &VideoConfig {
        &self.cfg
    }

    /// Applies new settings. Picture controls, repair and denoise
    /// settings change in place; a different standard, deinterlacer or
    /// separation method rebuilds the pipeline (the displayed frame is
    /// kept).
    pub fn set_config(&mut self, cfg: &VideoConfig) {
        let structural = cfg.standard != self.cfg.standard
            || cfg.deinterlace != self.cfg.deinterlace
            || cfg.color.ntsc != self.cfg.color.ntsc
            || cfg.color.pal != self.cfg.color.pal;
        if structural {
            let keep = std::mem::replace(&mut self.current, Frame::new(1, 1));
            *self = VideoPipeline::new(cfg);
            if keep.width == self.current.width && keep.height == self.current.height {
                self.current = keep;
            }
            return;
        }
        self.combiner.set_config(&cfg.combiner);
        self.color.set_controls(&cfg.color);
        self.repair.set_config(&cfg.repair);
        self.denoise.set_config(&cfg.denoise);
        self.cfg = cfg.clone();
        self.name = pipeline_name(cfg, &self.color);
    }

    /// The frame being displayed: rows up to the last update belong to the
    /// field being received, the rest to the previous field's frame.
    pub fn current_frame(&self) -> &Frame {
        &self.current
    }

    /// Added delay of the configured deinterlacer in fields.
    pub fn latency_fields(&self) -> u32 {
        self.cfg.deinterlace.latency_fields()
    }

    pub fn stats(&self) -> PipelineStats {
        let mut s = self.stats;
        s.combiner = self.combiner.stats();
        s.assembler = self.assembler.stats();
        s.repair = self.repair.stats();
        s
    }

    pub fn combiner(&self) -> &Combiner {
        &self.combiner
    }

    pub fn color_decoder(&self) -> &ColorDecoder {
        &self.color
    }

    pub fn denoiser(&self) -> &Denoiser {
        &self.denoise
    }

    pub fn repairer(&self) -> &Repairer {
        &self.repair
    }

    /// Feeds one receiver's line.
    pub fn push_line(&mut self, rec: LineRecord) -> PipelineOutput {
        let mut out = PipelineOutput::default();
        let t = Instant::now();
        let lines = self.combiner.push(rec);
        self.timing.combine_ns += ns(t);
        for l in lines {
            self.feed_combined(&l, &mut out, self.cfg.emit_frames);
        }
        out
    }

    /// Feeds an already combined line (or a single receiver's line) and
    /// bypasses the combiner.
    pub fn push_combined_line(&mut self, rec: &LineRecord) -> PipelineOutput {
        let mut out = PipelineOutput::default();
        self.feed_combined(rec, &mut out, self.cfg.emit_frames);
        out
    }

    fn feed_combined(&mut self, l: &LineRecord, out: &mut PipelineOutput, emit: bool) {
        let res = self.assembler.push(l);
        for f in res.completed {
            if let Some(r) = self.finish_field(Arc::new(f), out, emit) {
                merge(&mut out.updated_rows, r);
            }
        }
        if res.stored.is_some() && self.cfg.streaming {
            self.pending_lines += 1;
            if self.pending_lines >= self.cfg.stream_batch_lines.max(1) {
                self.pending_lines = 0;
                if let Some(r) = self.advance_streaming() {
                    merge(&mut out.updated_rows, r);
                }
            }
        }
    }

    /// Emits everything still pending (end of a recording).
    pub fn flush(&mut self) -> Vec<Frame> {
        let mut out = PipelineOutput::default();
        for l in self.combiner.flush() {
            self.feed_combined(&l, &mut out, true);
        }
        if let Some(f) = self.assembler.finish() {
            self.finish_field(Arc::new(f), &mut out, true);
        }
        if let Deint::Bidi(d) = &mut self.deint {
            if let Some(fr) = d.flush() {
                let fr = self.finish_frame(fr);
                out.frames_done += 1;
                out.frames_completed.push(fr);
            }
        }
        out.frames_completed
    }

    /// Offline: processes a complete composite field (already combined).
    pub fn push_field(&mut self, field: Field) -> Vec<Frame> {
        let mut out = PipelineOutput::default();
        self.work = None;
        self.finish_field(Arc::new(field), &mut out, true);
        out.frames_completed
    }

    fn start_work(&mut self, seq: u64, parity: openvrx_core::FieldParity) {
        let std = self.cfg.standard;
        // Recycled buffers: every line is written by its stage before any
        // later stage reads it, so only the metadata is reset.
        let yuv = match self.spare_yuv.take() {
            Some(mut f) => {
                reset_yuv(&mut f, seq, parity);
                f
            }
            None => YuvField::new(std, seq, parity),
        };
        let free = self.rep_pool.iter().position(|a| Arc::strong_count(a) == 1 && Arc::weak_count(a) == 0);
        let rep = match free.and_then(|i| Arc::try_unwrap(self.rep_pool.remove(i)).ok()) {
            Some(mut f) => {
                reset_yuv(&mut f, seq, parity);
                f
            }
            None => YuvField::new(std, seq, parity),
        };
        self.work = Some(Work {
            seq,
            parity,
            yuv,
            rep,
            yuv_ready: 0,
            rep_ready: 0,
            di_rows: 0,
            dn_rows: 0,
        });
    }

    /// Advances all stages for the field being received.
    fn advance_streaming(&mut self) -> Option<Range<usize>> {
        let fld = self.assembler.current()?;
        let (seq, parity) = (fld.seq, fld.parity);
        if self.work.as_ref().map(|w| (w.seq, w.parity)) != Some((seq, parity)) {
            self.start_work(seq, parity);
        }
        let fld = self.assembler.current()?;
        let lines_final = self.assembler.lines_final();
        let w = self.work.as_mut().expect("work");
        let par = self.cfg.parallel && self.cfg.stream_batch_lines >= STREAM_PARALLEL_MIN_LINES;

        let t = Instant::now();
        let r = self.color.advance(fld, lines_final, false, &mut w.yuv, par);
        w.yuv_ready = w.yuv_ready.max(r.end);
        self.timing.color_ns += ns(t);

        let t = Instant::now();
        if self.cfg.repair_enabled {
            let r = self.repair.advance(&w.yuv, w.yuv_ready, false, &mut w.rep, par);
            w.rep_ready = w.rep_ready.max(r.end);
        } else {
            copy_lines(&w.yuv, &mut w.rep, w.rep_ready..w.yuv_ready);
            w.rep_ready = w.yuv_ready;
        }
        self.timing.repair_ns += ns(t);

        let Deint::Past(d) = &self.deint else {
            return None;
        };
        let t = Instant::now();
        let rows = deinterlace::rows_ready(self.cfg.standard, &w.rep, w.rep_ready, false);
        let denoise_on = self.cfg.denoise.is_active();
        let mut updated = None;
        if rows > w.di_rows {
            let target = if denoise_on { &mut self.di_frame } else { &mut self.current };
            d.render_rows(&w.rep, w.di_rows..rows, target, par);
            if !denoise_on {
                updated = Some(w.di_rows..rows);
            }
            w.di_rows = rows;
        }
        self.timing.deinterlace_ns += ns(t);

        if denoise_on {
            let t = Instant::now();
            let r = self.denoise.advance(&self.di_frame, w.di_rows, false, &mut self.current, par);
            if !r.is_empty() {
                updated = Some(r.clone());
            }
            w.dn_rows = w.dn_rows.max(r.end);
            self.timing.denoise_ns += ns(t);
        }
        updated
    }

    /// Completes a field through all stages.
    fn finish_field(&mut self, field: Arc<Field>, out: &mut PipelineOutput, emit: bool) -> Option<Range<usize>> {
        let same = self.work.as_ref().map(|w| (w.seq, w.parity)) == Some((field.seq, field.parity));
        // A streamed field has only its last few lines left: finish them
        // on this thread unless streaming batches are large.
        let streamed = same && self.cfg.streaming && self.cfg.stream_batch_lines < STREAM_PARALLEL_MIN_LINES;
        let par = self.cfg.parallel && !streamed;
        if !same {
            self.start_work(field.seq, field.parity);
        }
        let mut w = self.work.take().expect("work");
        self.pending_lines = 0;

        let t = Instant::now();
        self.color.finish_field(field.clone(), &mut w.yuv, par);
        if let Ok(f) = Arc::try_unwrap(field) {
            self.assembler.recycle(f);
        }
        self.timing.color_ns += ns(t);

        let t = Instant::now();
        if self.cfg.repair_enabled {
            self.repair.finish_lines(&w.yuv, &mut w.rep, par);
        } else {
            let n = w.yuv.lines;
            copy_lines(&w.yuv, &mut w.rep, w.rep_ready..n);
            w.rep.first_ns = w.yuv.first_ns;
            w.rep.last_ns = w.yuv.last_ns;
        }
        self.spare_yuv = Some(w.yuv);
        let rep = Arc::new(w.rep);
        if self.cfg.repair_enabled {
            self.repair.remember(rep.clone());
        }
        self.rep_pool.push(rep.clone());
        if self.rep_pool.len() > 6 {
            self.rep_pool.remove(0);
        }
        self.timing.repair_ns += ns(t);

        let t = Instant::now();
        let denoise_on = self.cfg.denoise.is_active();
        let h = self.current.height;
        let frame = match &mut self.deint {
            Deint::Past(d) => {
                let target = if denoise_on { &mut self.di_frame } else { &mut self.current };
                d.render_rows(&rep, w.di_rows..h, target, par);
                target.meta = openvrx_core::FrameMeta {
                    standard: Some(rep.standard),
                    seq: 0,
                    field_seq: rep.seq,
                    parity: Some(rep.parity),
                    newest_line_ns: rep.last_ns,
                    ready_ns: 0,
                    pipeline: String::new(),
                };
                d.end_field(rep.clone());
                None
            }
            Deint::Bidi(d) => d.push_field(rep.clone()),
        };
        self.timing.deinterlace_ns += ns(t);

        match &self.deint {
            Deint::Past(_) => {
                let t = Instant::now();
                if denoise_on {
                    self.denoise.finish(&self.di_frame, &mut self.current, par);
                    self.current.meta = self.di_frame.meta.clone();
                }
                self.timing.denoise_ns += ns(t);
                let meta = &mut self.current.meta;
                meta.seq = self.frames;
                meta.ready_ns = openvrx_core::clock::now_ns();
                meta.pipeline.clone_from(&self.name);
                self.frames += 1;
                out.frames_done += 1;
                if emit {
                    out.frames_completed.push(self.current.clone());
                }
                self.end_field_timing();
                let from = if denoise_on { w.dn_rows } else { w.di_rows };
                Some(from.min(h)..h)
            }
            Deint::Bidi(_) => {
                let r = frame.map(|fr| {
                    let fr = self.finish_frame(fr);
                    out.frames_done += 1;
                    if emit {
                        out.frames_completed.push(fr);
                    }
                    0..h
                });
                self.end_field_timing();
                r
            }
        }
    }

    /// Denoise and stamp a frame from the lookahead deinterlacer.
    fn finish_frame(&mut self, fr: Frame) -> Frame {
        let t = Instant::now();
        let mut fr = if self.cfg.denoise.is_active() {
            let mut out = fr.clone();
            self.denoise.finish(&fr, &mut out, self.cfg.parallel);
            out
        } else {
            fr
        };
        self.timing.denoise_ns += ns(t);
        fr.meta.seq = self.frames;
        fr.meta.ready_ns = openvrx_core::clock::now_ns();
        fr.meta.pipeline = self.name.clone();
        self.frames += 1;
        self.current = fr.clone();
        fr
    }

    fn end_field_timing(&mut self) {
        let t = std::mem::take(&mut self.timing);
        self.stats.last = t;
        let a = &mut self.stats.average;
        if self.stats.frames == 0 {
            *a = t;
        } else {
            let ema = |o: &mut u64, n: u64| *o = (*o as f64 * 0.9 + n as f64 * 0.1) as u64;
            ema(&mut a.combine_ns, t.combine_ns);
            ema(&mut a.color_ns, t.color_ns);
            ema(&mut a.repair_ns, t.repair_ns);
            ema(&mut a.deinterlace_ns, t.deinterlace_ns);
            ema(&mut a.denoise_ns, t.denoise_ns);
        }
        self.stats.frames += 1;
    }

    /// Drops all state (after a channel change).
    pub fn reset(&mut self) {
        let cfg = self.cfg.clone();
        *self = VideoPipeline::new(&cfg);
    }
}

fn reset_yuv(f: &mut YuvField, seq: u64, parity: openvrx_core::FieldParity) {
    f.seq = seq;
    f.parity = parity;
    f.quality.fill(openvrx_core::LineQuality::UNKNOWN);
    f.source.fill(openvrx_core::COMBINED);
    f.first_ns = 0;
    f.last_ns = 0;
}

fn copy_lines(src: &YuvField, dst: &mut YuvField, r: Range<usize>) {
    if r.is_empty() {
        return;
    }
    let w = src.width;
    let a = r.start * w;
    let b = r.end * w;
    dst.y[a..b].copy_from_slice(&src.y[a..b]);
    dst.u[a..b].copy_from_slice(&src.u[a..b]);
    dst.v[a..b].copy_from_slice(&src.v[a..b]);
    dst.damage[a..b].copy_from_slice(&src.damage[a..b]);
    dst.quality[r.clone()].copy_from_slice(&src.quality[r.clone()]);
    dst.source[r.clone()].copy_from_slice(&src.source[r]);
}

fn merge(acc: &mut Option<Range<usize>>, r: Range<usize>) {
    if r.is_empty() {
        return;
    }
    *acc = Some(match acc.take() {
        Some(a) => a.start.min(r.start)..a.end.max(r.end),
        None => r,
    });
}

/// Builds a pipeline for a whole recording-style run: helper used by tests
/// and tools to decode fields without streaming.
pub fn decode_fields(cfg: &VideoConfig, fields: impl IntoIterator<Item = Field>) -> Vec<Frame> {
    let mut p = VideoPipeline::new(cfg);
    let mut out = Vec::new();
    for f in fields {
        out.extend(p.push_field(f));
    }
    out.extend(p.flush());
    out
}

fn pipeline_name(cfg: &VideoConfig, color: &ColorDecoder) -> String {
    let mut name = format!("{}+{}", color.method_name(), cfg.deinterlace.name());
    if cfg.repair_enabled {
        name.push_str("+repair");
    }
    match cfg.denoise.temporal {
        TemporalMode::Off => {}
        TemporalMode::Recursive => name.push_str("+tnr"),
        TemporalMode::MotionCompensated => name.push_str("+mctnr"),
    }
    if cfg.denoise.spatial > 0.0 {
        name.push_str("+snr");
    }
    name
}
