// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX recording format (version 1), as specified in
//! `docs/RECORDING_FORMAT.md`.
//!
//! A recording named `NAME` is a set of files in one directory:
//!
//! - `NAME.openvrx.json`: the [`Sidecar`] (written at start, rewritten at close),
//! - `NAME.rx0.bin`, `NAME.rx1.bin`, ...: raw lane words per receiver,
//! - `NAME.events.jsonl`: one [`Event`] per line, appended live,
//! - `NAME.truth.f32`: simulator truth (clean composite, f32 IRE).
//!
//! Main entry points:
//!
//! - [`RecordingWriter`]: creates a recording and appends raw data, events
//!   and truth; crash-safe (a recording that was never closed is still
//!   readable, byte counts then come from the file sizes).
//! - [`Recording`]: opens a recording by any of its files; gives typed
//!   sidecar and events, seekable [`RawReader`]s, a paced [`Replay`]
//!   iterator and [`TruthReader`] access.
//! - [`RollingBuffer`]: the live app's per-receiver RAM history that can be
//!   drained into a writer when recording starts.
//! - [`tools`]: listing, summaries, and cutting a time range into a new
//!   recording.

pub mod events;
pub mod paths;
pub mod reader;
pub mod replay;
pub mod rolling;
pub mod sidecar;
pub mod tools;
pub mod writer;

pub use events::{Event, EventKind, EventLog, ProbeVerdict};
pub use paths::RecordingPaths;
pub use reader::{RawBlocks, RawReader, Recording, Timeline, TruthReader};
pub use replay::{Replay, ReplayOptions};
pub use rolling::{DrainStats, RollingBuffer, RollingConfig};
pub use sidecar::{ReceiverInfo, RecordingSource, Sidecar, SimInfo, TruthInfo};
pub use tools::{extract_range, list_recordings, split, Listing, ReceiverSummary, RecordingSummary};
pub use writer::{RecordingWriter, WriterOptions};

use std::path::{Path, PathBuf};

/// Value of the sidecar's `format` field.
pub const FORMAT_NAME: &str = "openvrx-recording";
/// Recording format version written by this crate.
pub const FORMAT_VERSION: u32 = 1;

/// Errors from reading or writing recordings.
#[derive(Debug, thiserror::Error)]
pub enum FormatError {
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("{}: invalid JSON: {source}", path.display())]
    Json {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("not an OpenVRX recording: {0}")]
    NotRecording(String),
    #[error("unsupported recording format version {0} (this build reads up to {FORMAT_VERSION})")]
    Version(u32),
    #[error("receiver {0} is not in this recording")]
    NoReceiver(u8),
    #[error("recording has no truth file")]
    NoTruth,
    #[error("recording already exists: {}", .0.display())]
    Exists(PathBuf),
    #[error("invalid argument: {0}")]
    Invalid(String),
}

pub type Result<T> = std::result::Result<T, FormatError>;

pub(crate) fn io_err(path: &Path) -> impl FnOnce(std::io::Error) -> FormatError + '_ {
    move |source| FormatError::Io {
        path: path.to_path_buf(),
        source,
    }
}

/// Current UTC time in the sidecar's `created_utc` form
/// (`2026-10-29T21:14:03Z`).
pub fn utc_now_string() -> String {
    let fmt = time::macros::format_description!(
        "[year]-[month]-[day]T[hour]:[minute]:[second]Z"
    );
    time::OffsetDateTime::now_utc()
        .format(&fmt)
        .unwrap_or_else(|_| "1970-01-01T00:00:00Z".to_string())
}

/// Software string written into new sidecars.
pub fn software_string() -> String {
    format!("openvrx {}", env!("CARGO_PKG_VERSION"))
}
