//! OpenVRX shared types.
//!
//! Every other crate in the workspace builds against these definitions:
//!
//! - [`standard`]: NTSC/PAL raster geometry on the line-locked output grid
//!   (910 samples per line for NTSC, 1135 for PAL, sample 0 at 0H).
//! - [`lanes`]: how ADC bits from the C5's MODEM_DIAG bus are wired onto the
//!   FX2 port and how to turn a captured word back into I/Q.
//! - [`records`]: the data that flows between pipeline stages (raw USB
//!   blocks, I/Q chunks, demodulated chunks, line records, fields, frames).
//! - [`quality`]: per-line quality metrics and damage-mask conventions.
//! - [`clock`]: one monotonic host clock shared by capture, DSP and display.
//!
//! Units used everywhere:
//! - I/Q samples are `Complex32` in 10-bit ADC units (full scale +-512),
//!   half-step centred, whatever lane format carried them.
//! - Demodulated video is instantaneous frequency in Hz until sync
//!   normalisation, then IRE (blanking 0, peak white 100, sync tip -40 NTSC
//!   or -300/7 PAL).
//! - Component video after colour decode: Y 0.0 (black) .. 1.0 (white),
//!   U and V scaled as BT.601 (about +-0.436 and +-0.615 at full saturation).

pub mod clock;
pub mod lanes;
pub mod quality;
pub mod records;
pub mod standard;

pub use lanes::{LaneFormat, LanePreset, SampleFormat};
pub use num_complex::Complex32;
pub use quality::{damage, LineFlags, LineQuality};
pub use records::{
    DemodChunk, Field, Frame, FrameMeta, IqChunk, LineRecord, RawBlock, RxId, YuvField, COMBINED,
};
pub use standard::{FieldParity, StandardParams, VideoStandard};

/// Errors shared by the core helpers.
#[derive(Debug, thiserror::Error)]
pub enum CoreError {
    #[error("invalid lane format: {0}")]
    LaneFormat(String),
    #[error("unknown preset `{0}`")]
    UnknownPreset(String),
}
