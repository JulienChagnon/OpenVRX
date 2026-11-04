// SPDX-License-Identifier: GPL-3.0-or-later
//! The contract between the video pipeline and the display.
//!
//! A view's picture comes from a [`FrameSource`]. The display thread calls
//! [`FrameSource::poll`] whenever it is woken (or at least at the OSD
//! refresh rate); the call never blocks for longer than a short memcpy and
//! copies only the rows that changed since the previous poll into the
//! caller's [`UploadStaging`], which the display uploads to the GPU.
//!
//! Most producers do not implement the trait by hand: the video thread
//! keeps a [`FrameProducer`] (from [`FrameExchange`]) and calls
//! [`FrameProducer::publish`] with its `openvrx_core::Frame` and the row
//! range it just updated; [`ExchangeSource`] is the matching
//! `FrameSource`. Status for the OSD goes through a [`StatusCell`].

pub mod exchange;
pub mod test_pattern;

pub use exchange::{DirtyRows, FrameExchange, FrameProducer, RowUpdate, UploadStaging};

use openvrx_core::{RxId, VideoStandard};
use std::sync::{Arc, Mutex};

/// Wakes the display thread. Cheap and callable from any thread; the
/// display coalesces repeated wakes.
pub type Waker = Arc<dyn Fn() + Send + Sync>;

/// A waker that does nothing (for tests and offscreen rendering).
#[cfg(test)]
pub fn noop_waker() -> Waker {
    Arc::new(|| {})
}

/// What a poll delivered.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct UpdateInfo {
    pub width: usize,
    pub height: usize,
    /// Frame size changed: GPU textures must be recreated; all rows are in
    /// the staging ranges.
    pub resized: bool,
    /// Rows in the staging ranges.
    pub rows: u32,
    /// Field lines that arrived since the previous poll.
    pub lines: u32,
    /// Fields and frames completed since the previous poll.
    pub fields_completed: u32,
    pub frames_completed: u32,
    /// Host time of the newest line contributing to the delivered rows
    /// (`FrameMeta::newest_line_ns`).
    pub newest_line_ns: u64,
    pub field_seq: u64,
    pub frame_seq: u64,
    pub standard: Option<VideoStandard>,
}

/// Lock state of a view.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum LockState {
    #[default]
    NoSignal,
    Searching,
    /// Sync found but riding the flywheel on some lines.
    Flywheel,
    Locked,
}

impl LockState {
    pub fn name(self) -> &'static str {
        match self {
            LockState::NoSignal => "NO SIGNAL",
            LockState::Searching => "SEARCHING",
            LockState::Flywheel => "FLYWHEEL",
            LockState::Locked => "LOCKED",
        }
    }
}

/// One receiver feeding a view.
#[derive(Clone, Debug, PartialEq)]
pub struct RxStatus {
    pub rx: RxId,
    /// Envelope C/N in dB (NaN when unknown, e.g. angle-only lanes).
    pub cn_db: f32,
    /// Raw I/Q power, dB re 1 ADC unit squared.
    pub power_db: f32,
    pub gain: u8,
    pub sync_locked: bool,
    pub freq_mhz: u32,
    pub sample_rate_hz: f64,
    /// USB payload rate, bytes per second.
    pub usb_bytes_per_s: f64,
    pub dropped_samples: u64,
    pub dropped_transfers: u64,
    /// Fraction of the picture rows this receiver fed in the last frame.
    pub share: f32,
}

/// Snapshot of a view's source for the OSD.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SourceStatus {
    /// Short name shown in the OSD ("TEST mix", "R1 5658").
    pub label: String,
    pub standard: Option<VideoStandard>,
    pub lock: LockState,
    pub freq_mhz: Option<u32>,
    pub receivers: Vec<RxStatus>,
    /// Measured field rate.
    pub field_rate_hz: f32,
    /// Deinterlacer and filters producing the picture.
    pub pipeline: String,
    /// Pipeline-side drops (lines or fields thrown away to keep up).
    pub dropped_lines: u64,
    pub warnings: Vec<String>,
    /// True for simulated data (test sources).
    pub simulated: bool,
}

/// A picture producer for one view. Implementations must never block the
/// caller on producer work.
pub trait FrameSource: Send {
    /// Short label for logs and the OSD.
    fn label(&self) -> &str;
    /// Called once by the display; the source calls it after publishing.
    fn set_waker(&mut self, waker: Waker);
    /// Copies changed rows into `staging`. None when nothing changed.
    fn poll(&mut self, staging: &mut UploadStaging) -> Option<UpdateInfo>;
    /// Copies the latest status into `out` when it changed since the last
    /// call; returns whether it did.
    fn status(&mut self, out: &mut SourceStatus) -> bool;
    /// Lines that may accumulate before the source wakes the display
    /// (field and frame ends always wake). 0 wakes on every update.
    fn set_wake_lines(&mut self, lines: u32);
}

/// A value shared between a producer thread and the display, with a
/// change counter so readers copy only when something changed.
pub struct StatusCell<T> {
    inner: Mutex<(u64, T)>,
}

impl<T: Clone + Default> Default for StatusCell<T> {
    fn default() -> Self {
        StatusCell::new(T::default())
    }
}

impl<T: Clone> StatusCell<T> {
    pub fn new(value: T) -> StatusCell<T> {
        StatusCell {
            inner: Mutex::new((1, value)),
        }
    }

    /// Modifies the value and bumps the change counter.
    pub fn publish(&self, f: impl FnOnce(&mut T)) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        f(&mut g.1);
        g.0 += 1;
    }

    /// Copies the value into `out` when the counter moved past `seen`.
    pub fn read_into(&self, out: &mut T, seen: &mut u64) -> bool {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.0 == *seen {
            return false;
        }
        out.clone_from(&g.1);
        *seen = g.0;
        true
    }

}

/// [`FrameSource`] over a [`FrameExchange`] and a status cell. `guard`
/// owns whatever keeps the producer alive (a worker thread handle); it is
/// dropped with the source.
pub struct ExchangeSource {
    label: String,
    exchange: Arc<FrameExchange>,
    status: Arc<StatusCell<SourceStatus>>,
    seen: u64,
    _guard: Option<Box<dyn Send>>,
}

impl ExchangeSource {
    pub fn new(
        label: impl Into<String>,
        exchange: Arc<FrameExchange>,
        status: Arc<StatusCell<SourceStatus>>,
        guard: Option<Box<dyn Send>>,
    ) -> ExchangeSource {
        ExchangeSource {
            label: label.into(),
            exchange,
            status,
            seen: 0,
            _guard: guard,
        }
    }
}

impl FrameSource for ExchangeSource {
    fn label(&self) -> &str {
        &self.label
    }
    fn set_waker(&mut self, waker: Waker) {
        self.exchange.set_waker(waker);
    }
    fn poll(&mut self, staging: &mut UploadStaging) -> Option<UpdateInfo> {
        self.exchange.take(staging)
    }
    fn status(&mut self, out: &mut SourceStatus) -> bool {
        self.status.read_into(out, &mut self.seen)
    }
    fn set_wake_lines(&mut self, lines: u32) {
        self.exchange.set_wake_lines(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_cell_copies_only_on_change() {
        let cell = StatusCell::new(SourceStatus::default());
        let mut out = SourceStatus::default();
        let mut seen = 0;
        assert!(cell.read_into(&mut out, &mut seen));
        assert!(!cell.read_into(&mut out, &mut seen));
        cell.publish(|s| s.label = "abc".into());
        assert!(cell.read_into(&mut out, &mut seen));
        assert_eq!(out.label, "abc");
        assert!(!cell.read_into(&mut out, &mut seen));
    }
}
