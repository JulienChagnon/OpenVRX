// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX video stage: everything between the per-receiver line records and
//! the displayed or recorded progressive frame.
//!
//! ```text
//! LineRecord (per rx) -> combiner -> assembler -> color -> repair
//!                     -> deinterlace -> denoise -> Frame
//! ```
//!
//! - [`combiner`]: diversity across receivers (select, blend, per-sample
//!   repair, segment scoring) with field alignment, bounded waiting and
//!   sub-sample timing alignment.
//! - [`assembler`]: lines to composite fields, tolerant of gaps.
//! - [`color`]: NTSC/PAL decoding with burst PLL, ACC, colour killer and
//!   1D/2D/3D comb, PAL-D and Transform PAL separation.
//! - [`repair`]: damage-mask driven sparkle and dropout concealment with
//!   the three-field past-fill limit.
//! - [`deinterlace`]: bob, weave, blend, motion adaptive, past-only yadif,
//!   edge-directed motion adaptive; offline yadif, bwdif and motion
//!   compensated.
//! - [`denoise`]: past-only motion-gated recursive filter (optionally
//!   motion compensated) and a light spatial filter.
//! - [`pipeline`]: the composed, configurable, streaming pipeline.
//! - [`encoder`], [`testgen`], [`metrics`]: composite encoder, test
//!   pictures and quality measures for tests and tools.

pub mod assembler;
pub mod color;
pub mod combiner;
#[cfg(feature = "debug-png")]
pub mod debug_png;
pub mod deinterlace;
pub mod denoise;
pub mod encoder;
pub mod metrics;
pub mod motion;
pub mod pipeline;
pub mod repair;
pub mod testgen;
pub mod util;

pub use assembler::FieldAssembler;
pub use color::{ColorConfig, ColorDecoder, NtscSeparation, PalSeparation};
pub use combiner::{CombineMode, Combiner, CombinerConfig};
pub use deinterlace::{DeinterlaceMethod, Deinterlacer};
pub use denoise::{DenoiseConfig, Denoiser, TemporalMode};
pub use encoder::{CompositeEncoder, EncoderConfig};
pub use pipeline::{PipelineOutput, StageTimings, VideoConfig, VideoPipeline};
pub use repair::{RepairConfig, Repairer};
