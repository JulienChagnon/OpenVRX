// SPDX-License-Identifier: GPL-3.0-or-later
//! Row-level frame exchange between the video thread and the display.
//!
//! The producer converts the rows it just finished to half floats in its
//! own scratch buffers (outside the lock), then copies them into the
//! shared frame under a short lock and marks them dirty. The display
//! copies only the dirty rows into its staging buffers and uploads those
//! rows. No allocation happens per frame: buffers grow only when the
//! frame size changes.

use super::{UpdateInfo, Waker};
use half::f16;
use half::slice::HalfFloatSliceExt;
use openvrx_core::{Frame, VideoStandard, COMBINED};
use std::ops::Range;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

/// Set of dirty rows (bit set).
#[derive(Clone, Debug, Default)]
pub struct DirtyRows {
    bits: Vec<u64>,
    rows: usize,
}

impl DirtyRows {
    #[cfg(test)]
    pub fn new(rows: usize) -> DirtyRows {
        let mut d = DirtyRows::default();
        d.resize(rows);
        d
    }

    /// Changes the row count and clears everything.
    pub fn resize(&mut self, rows: usize) {
        self.rows = rows;
        self.bits.clear();
        self.bits.resize(rows.div_ceil(64), 0);
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn mark(&mut self, range: Range<usize>) {
        let end = range.end.min(self.rows);
        let mut r = range.start;
        while r < end {
            let w = r / 64;
            let b0 = r % 64;
            let b1 = (end - w * 64).min(64);
            let mask = if b1 - b0 == 64 {
                u64::MAX
            } else {
                ((1u64 << (b1 - b0)) - 1) << b0
            };
            self.bits[w] |= mask;
            r = w * 64 + b1;
        }
    }

    pub fn mark_all(&mut self) {
        let rows = self.rows;
        self.mark(0..rows);
    }

    pub fn is_empty(&self) -> bool {
        self.bits.iter().all(|&w| w == 0)
    }

    #[cfg(test)]
    pub fn is_set(&self, row: usize) -> bool {
        row < self.rows && self.bits[row / 64] & (1 << (row % 64)) != 0
    }

    /// Appends the dirty rows as coalesced ranges to `out` and clears them.
    pub fn drain_ranges(&mut self, out: &mut Vec<Range<u32>>) {
        let mut start: Option<usize> = None;
        for row in 0..self.rows {
            let w = self.bits[row / 64];
            if w == 0 && row % 64 == 0 {
                // Whole word clean: close any open run and skip ahead.
                if let Some(s) = start.take() {
                    out.push(s as u32..row as u32);
                }
                continue;
            }
            let set = w & (1 << (row % 64)) != 0;
            match (set, start) {
                (true, None) => start = Some(row),
                (false, Some(s)) => {
                    out.push(s as u32..row as u32);
                    start = None;
                }
                _ => {}
            }
        }
        if let Some(s) = start {
            out.push(s as u32..self.rows as u32);
        }
        self.bits.fill(0);
    }
}

/// Rows the producer just updated.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RowUpdate {
    /// Output rows that changed.
    pub rows: Range<usize>,
    /// Field lines that arrived (counts toward slice presentation).
    pub lines: u32,
    /// The last line of a field arrived.
    pub field_done: bool,
    /// The last line of a frame arrived.
    pub frame_done: bool,
    /// Host time of the newest line in these rows; 0 uses
    /// `frame.meta.newest_line_ns`.
    pub newest_line_ns: u64,
}

/// Display-side copy of the changed rows, laid out as full planes so each
/// range uploads straight from its offset.
#[derive(Debug, Default)]
pub struct UploadStaging {
    pub width: usize,
    pub height: usize,
    /// Luma, width x height.
    pub y: Vec<f16>,
    /// Interleaved U, V, width x height x 2.
    pub uv: Vec<f16>,
    /// Damage mask, width x height.
    pub damage: Vec<u8>,
    /// Receiver per row.
    pub row_source: Vec<u8>,
    /// Rows to upload (coalesced, ascending).
    pub ranges: Vec<Range<u32>>,
    /// With field history on (the live filter): the last complete fields,
    /// slot = field number % HISTORY, and per row 1 when it already holds
    /// the field in progress (0: still the previous field).
    pub history: [HistoryField; HISTORY],
    /// Complete fields so far (the newest is number history_count - 1).
    pub history_count: u64,
    /// Field numbers copied into `history` by the last take, oldest first.
    pub history_new: Vec<u64>,
    pub row_new: Vec<u8>,
}

/// Complete fields kept for the live filter (it reads k-3 .. k-1).
pub const HISTORY: usize = 3;

/// One complete field (planes as in [`UploadStaging`]).
#[derive(Clone, Debug, Default)]
pub struct HistoryField {
    pub y: Vec<f16>,
    pub uv: Vec<f16>,
}

impl UploadStaging {
    /// Resizes the planes (allocates only when the size changes).
    pub fn ensure(&mut self, width: usize, height: usize) {
        if self.width == width && self.height == height {
            return;
        }
        let n = width * height;
        self.width = width;
        self.height = height;
        self.y.clear();
        self.y.resize(n, f16::ZERO);
        self.uv.clear();
        self.uv.resize(2 * n, f16::ZERO);
        self.damage.clear();
        self.damage.resize(n, 0);
        self.row_source.clear();
        self.row_source.resize(height, COMBINED);
    }
}

#[derive(Default)]
struct Shared {
    width: usize,
    height: usize,
    y: Vec<f16>,
    uv: Vec<f16>,
    damage: Vec<u8>,
    row_source: Vec<u8>,
    dirty: DirtyRows,
    resized: bool,
    lines: u32,
    /// Lines published since the last wake.
    lines_since_wake: u32,
    fields: u32,
    frames: u32,
    newest_line_ns: u64,
    field_seq: u64,
    frame_seq: u64,
    standard: Option<VideoStandard>,
    /// Field history for the live filter (off: none of the below is kept).
    history_on: bool,
    history: [HistoryField; HISTORY],
    history_count: u64,
    row_new: Vec<u8>,
}

/// Shared frame with dirty-row tracking.
#[derive(Default)]
pub struct FrameExchange {
    shared: Mutex<Shared>,
    waker: Mutex<Option<Waker>>,
    /// Wake the display every this many lines (0 = every publish); field
    /// and frame ends always wake.
    wake_lines: AtomicU32,
}

impl FrameExchange {
    pub fn new() -> Arc<FrameExchange> {
        Arc::new(FrameExchange::default())
    }

    /// Sets how many published lines may accumulate between wakes.
    pub fn set_wake_lines(&self, lines: u32) {
        self.wake_lines.store(lines, Ordering::Relaxed);
    }

    pub fn set_waker(&self, waker: Waker) {
        *self.waker.lock().unwrap_or_else(|e| e.into_inner()) = Some(waker);
    }

    /// Keeps the last complete fields and the per-row field flags (the
    /// live filter's inputs): about 2 MB copied per field, so only when a
    /// view uses them. Switching it on starts the history afresh.
    pub fn set_history(&self, on: bool) {
        let mut s = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        if s.history_on != on {
            s.history_on = on;
            s.history_count = 0;
            let h = s.height;
            s.row_new.clear();
            s.row_new.resize(h, 0);
        }
    }

    fn wake(&self) {
        let w = self.waker.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(w) = w {
            w();
        }
    }

    /// A producer handle with its own conversion scratch.
    pub fn producer(self: &Arc<Self>) -> FrameProducer {
        FrameProducer {
            exchange: Arc::clone(self),
            y: Vec::new(),
            uv: Vec::new(),
        }
    }

    /// Copies dirty rows into `staging`. Never waits on producer work
    /// beyond the producer's own memcpy.
    pub fn take(&self, staging: &mut UploadStaging) -> Option<UpdateInfo> {
        let mut s = self.shared.lock().unwrap_or_else(|e| e.into_inner());
        if s.width == 0 || (s.dirty.is_empty() && !s.resized && s.fields == 0) {
            return None;
        }
        staging.ensure(s.width, s.height);
        staging.ranges.clear();
        let resized = s.resized;
        if resized {
            s.dirty.mark_all();
        }
        let shared = &mut *s;
        shared.dirty.drain_ranges(&mut staging.ranges);
        let w = shared.width;
        let mut rows = 0;
        for r in &staging.ranges {
            let (a, b) = (r.start as usize, r.end as usize);
            rows += (b - a) as u32;
            staging.y[a * w..b * w].copy_from_slice(&shared.y[a * w..b * w]);
            staging.uv[2 * a * w..2 * b * w].copy_from_slice(&shared.uv[2 * a * w..2 * b * w]);
            staging.damage[a * w..b * w].copy_from_slice(&shared.damage[a * w..b * w]);
            staging.row_source[a..b].copy_from_slice(&shared.row_source[a..b]);
        }
        staging.history_new.clear();
        if shared.history_on {
            // Fields completed since the last take (at most HISTORY: older
            // ones are gone), then the row flags.
            if resized || staging.history_count > shared.history_count {
                staging.history_count = 0;
            }
            let first = staging.history_count.max(shared.history_count.saturating_sub(HISTORY as u64));
            for n in first..shared.history_count {
                let slot = (n % HISTORY as u64) as usize;
                let (src, dst) = (&shared.history[slot], &mut staging.history[slot]);
                dst.y.clone_from(&src.y);
                dst.uv.clone_from(&src.uv);
                staging.history_new.push(n);
            }
            staging.history_count = shared.history_count;
            staging.row_new.clone_from(&shared.row_new);
        }
        let info = UpdateInfo {
            width: shared.width,
            height: shared.height,
            resized,
            rows,
            lines: shared.lines,
            fields_completed: shared.fields,
            frames_completed: shared.frames,
            newest_line_ns: shared.newest_line_ns,
            field_seq: shared.field_seq,
            frame_seq: shared.frame_seq,
            standard: shared.standard,
        };
        shared.resized = false;
        shared.lines = 0;
        shared.fields = 0;
        shared.frames = 0;
        Some(info)
    }
}

/// Producer handle (one per video thread).
pub struct FrameProducer {
    exchange: Arc<FrameExchange>,
    y: Vec<f16>,
    uv: Vec<f16>,
}

impl FrameProducer {
    /// Publishes `update.rows` of `frame` and wakes the display.
    pub fn publish(&mut self, frame: &Frame, update: RowUpdate) {
        let w = frame.width;
        let h = frame.height;
        let a = update.rows.start.min(h);
        let b = update.rows.end.min(h);
        let n = (b - a) * w;
        // Convert outside the lock.
        self.y.resize(n, f16::ZERO);
        self.y
            .as_mut_slice()
            .convert_from_f32_slice(&frame.y[a * w..b * w]);
        self.uv.resize(2 * n, f16::ZERO);
        for (k, pair) in self.uv.as_chunks_mut::<2>().0.iter_mut().enumerate() {
            pair[0] = f16::from_f32(frame.u[a * w + k]);
            pair[1] = f16::from_f32(frame.v[a * w + k]);
        }
        let wake = {
            let mut s = self
                .exchange
                .shared
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            let mut resized_now = false;
            if s.width != w || s.height != h {
                resized_now = true;
                s.width = w;
                s.height = h;
                s.y.clear();
                s.y.resize(w * h, f16::ZERO);
                s.uv.clear();
                s.uv.resize(2 * w * h, f16::ZERO);
                s.damage.clear();
                s.damage.resize(w * h, 0);
                s.row_source.clear();
                s.row_source.resize(h, COMBINED);
                s.dirty.resize(h);
                s.resized = true;
                s.history_count = 0;
                s.row_new.clear();
                s.row_new.resize(h, 0);
            }
            s.y[a * w..b * w].copy_from_slice(&self.y);
            s.uv[2 * a * w..2 * b * w].copy_from_slice(&self.uv);
            if frame.damage.len() == w * h {
                s.damage[a * w..b * w].copy_from_slice(&frame.damage[a * w..b * w]);
            }
            if frame.row_source.len() == h {
                s.row_source[a..b].copy_from_slice(&frame.row_source[a..b]);
            }
            s.dirty.mark(a..b);
            s.lines = s.lines.saturating_add(update.lines);
            if update.field_done {
                s.fields += 1;
            }
            if s.history_on {
                // These rows now hold the field in progress; at its last
                // line the whole picture is that field: keep it.
                s.row_new[a..b].fill(1);
                if update.field_done {
                    let slot = (s.history_count % HISTORY as u64) as usize;
                    let sh = &mut *s;
                    sh.history[slot].y.clone_from(&sh.y);
                    sh.history[slot].uv.clone_from(&sh.uv);
                    sh.history_count += 1;
                    sh.row_new.fill(0);
                }
            }
            if update.frame_done {
                s.frames += 1;
            }
            let stamp = if update.newest_line_ns != 0 {
                update.newest_line_ns
            } else {
                frame.meta.newest_line_ns
            };
            s.newest_line_ns = s.newest_line_ns.max(stamp);
            s.field_seq = frame.meta.field_seq;
            s.frame_seq = frame.meta.seq;
            s.standard = frame.meta.standard;
            s.lines_since_wake = s.lines_since_wake.saturating_add(update.lines);
            let every = self.exchange.wake_lines.load(Ordering::Relaxed);
            let wake = update.field_done || update.frame_done || resized_now || s.lines_since_wake >= every;
            if wake {
                s.lines_since_wake = 0;
            }
            wake
        };
        if wake {
            self.exchange.wake();
        }
    }

    /// Publishes the whole frame as one completed frame.
    pub fn publish_frame(&mut self, frame: &Frame, lines: u32) {
        self.publish(
            frame,
            RowUpdate {
                rows: 0..frame.height,
                lines,
                field_done: true,
                frame_done: true,
                newest_line_ns: 0,
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn ranges(d: &mut DirtyRows) -> Vec<Range<u32>> {
        let mut v = Vec::new();
        d.drain_ranges(&mut v);
        v
    }

    #[test]
    fn dirty_rows_coalesce() {
        let mut d = DirtyRows::new(200);
        assert!(d.is_empty());
        d.mark(3..5);
        d.mark(5..9);
        d.mark(60..70);
        d.mark(128..192);
        d.mark(199..400);
        assert!(d.is_set(64) && !d.is_set(70));
        assert_eq!(ranges(&mut d), vec![3..9, 60..70, 128..192, 199..200]);
        assert!(d.is_empty());
        d.mark_all();
        assert_eq!(ranges(&mut d), vec![0..200]);
        d.mark(10..10);
        assert!(d.is_empty());
    }

    #[test]
    fn dirty_rows_exact_word() {
        let mut d = DirtyRows::new(128);
        d.mark(0..64);
        assert_eq!(ranges(&mut d), vec![0..64]);
        d.mark(63..65);
        assert_eq!(ranges(&mut d), vec![63..65]);
    }

    fn frame(w: usize, h: usize, val: f32) -> Frame {
        let mut f = Frame::new(w, h);
        f.y.iter_mut().for_each(|x| *x = val);
        f.u.iter_mut().for_each(|x| *x = -0.25);
        f.v.iter_mut().for_each(|x| *x = 0.5);
        f
    }

    #[test]
    fn exchange_delivers_only_dirty_rows() {
        let ex = FrameExchange::new();
        let wakes = Arc::new(AtomicUsize::new(0));
        let wc = Arc::clone(&wakes);
        ex.set_waker(Arc::new(move || {
            wc.fetch_add(1, Ordering::SeqCst);
        }));
        let mut st = UploadStaging::default();
        assert!(ex.take(&mut st).is_none());

        let mut p = ex.producer();
        let mut f = frame(8, 6, 0.5);
        f.meta.newest_line_ns = 1234;
        f.row_source[2] = 1;
        p.publish(
            &f,
            RowUpdate {
                rows: 2..4,
                lines: 1,
                field_done: false,
                frame_done: false,
                newest_line_ns: 0,
            },
        );
        assert_eq!(wakes.load(Ordering::SeqCst), 1);
        let info = ex.take(&mut st).unwrap();
        // First publish resizes: everything uploads.
        assert!(info.resized);
        assert_eq!(st.ranges, vec![0..6]);
        assert_eq!(info.rows, 6);
        assert_eq!(info.newest_line_ns, 1234);
        assert_eq!(st.y[2 * 8].to_f32(), 0.5);
        assert_eq!(st.y[0].to_f32(), 0.0);
        assert_eq!(st.uv[2 * 2 * 8].to_f32(), -0.25);
        assert_eq!(st.uv[2 * 2 * 8 + 1].to_f32(), 0.5);
        assert_eq!(st.row_source[2], 1);
        assert!(ex.take(&mut st).is_none());

        let f2 = frame(8, 6, 1.0);
        p.publish(
            &f2,
            RowUpdate {
                rows: 4..6,
                lines: 1,
                field_done: true,
                frame_done: false,
                newest_line_ns: 0,
            },
        );
        let info = ex.take(&mut st).unwrap();
        assert!(!info.resized);
        assert_eq!(st.ranges, vec![4..6]);
        assert_eq!(info.lines, 1);
        assert_eq!(info.fields_completed, 1);
        assert_eq!(st.y[4 * 8].to_f32(), 1.0);
        // Earlier rows keep their staged values.
        assert_eq!(st.y[2 * 8].to_f32(), 0.5);
    }

    #[test]
    fn wake_threshold() {
        let ex = FrameExchange::new();
        let wakes = Arc::new(AtomicUsize::new(0));
        let wc = Arc::clone(&wakes);
        ex.set_waker(Arc::new(move || {
            wc.fetch_add(1, Ordering::SeqCst);
        }));
        ex.set_wake_lines(4);
        let mut p = ex.producer();
        let f = frame(4, 8, 0.2);
        let up = |k: usize, field_done: bool| RowUpdate {
            rows: k..k + 1,
            lines: 1,
            field_done,
            frame_done: false,
            newest_line_ns: 0,
        };
        p.publish(&f, up(0, false)); // resize wakes
        p.publish(&f, up(1, false));
        p.publish(&f, up(2, false));
        p.publish(&f, up(3, false));
        assert_eq!(wakes.load(Ordering::SeqCst), 1);
        p.publish(&f, up(4, false));
        assert_eq!(wakes.load(Ordering::SeqCst), 2);
        p.publish(&f, up(5, true));
        assert_eq!(wakes.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn exchange_resizes() {
        let ex = FrameExchange::new();
        let mut p = ex.producer();
        let mut st = UploadStaging::default();
        p.publish_frame(&frame(8, 6, 0.1), 3);
        let i = ex.take(&mut st).unwrap();
        assert_eq!((i.width, i.height, i.frames_completed), (8, 6, 1));
        p.publish_frame(&frame(10, 4, 0.2), 2);
        let i = ex.take(&mut st).unwrap();
        assert!(i.resized);
        assert_eq!((st.width, st.height), (10, 4));
        assert_eq!(st.ranges, vec![0..4]);
        assert_eq!(st.y.len(), 40);
    }

    #[test]
    fn history_keeps_complete_fields_and_row_flags() {
        let ex = FrameExchange::new();
        let mut p = ex.producer();
        let mut st = UploadStaging::default();
        let up = |rows: Range<usize>, field_done: bool| RowUpdate {
            rows,
            lines: 1,
            field_done,
            frame_done: false,
            newest_line_ns: 0,
        };
        p.publish(&frame(4, 4, 0.0), up(0..4, true)); // sizes the exchange
        ex.set_history(true);
        let _ = ex.take(&mut st);
        // Fields 0, 1, 2 (values 0.1, 0.2, 0.3), each in two halves.
        for (n, val) in [0.1f32, 0.2, 0.3].iter().enumerate() {
            p.publish(&frame(4, 4, *val), up(0..2, false));
            p.publish(&frame(4, 4, *val), up(2..4, true));
            if n == 0 {
                let _ = ex.take(&mut st);
                assert_eq!(st.history_new, vec![0]);
            }
        }
        // Half of field 3 (0.4).
        p.publish(&frame(4, 4, 0.4), up(0..2, false));
        let _ = ex.take(&mut st).unwrap();
        assert_eq!(st.history_new, vec![1, 2]);
        assert_eq!(st.history_count, 3);
        for (n, val) in [(0usize, 0.1f32), (1, 0.2), (2, 0.3)] {
            let v = st.history[n % HISTORY].y[5].to_f32();
            assert!((v - val).abs() < 1e-3, "field {n}: {v}");
        }
        assert_eq!(st.row_new, vec![1, 1, 0, 0]);
        // More fields than slots between takes: only the newest HISTORY come.
        for val in [0.5f32, 0.6, 0.7, 0.8] {
            p.publish(&frame(4, 4, val), up(0..4, true));
        }
        let _ = ex.take(&mut st).unwrap();
        assert_eq!(st.history_new, vec![4, 5, 6]);
        assert!((st.history[6 % HISTORY].y[0].to_f32() - 0.8).abs() < 1e-3);
        assert_eq!(st.row_new, vec![0, 0, 0, 0]);
    }

    #[test]
    fn publish_clamps_rows() {
        let ex = FrameExchange::new();
        let mut p = ex.producer();
        let mut st = UploadStaging::default();
        p.publish(
            &frame(4, 4, 0.3),
            RowUpdate {
                rows: 3..10,
                lines: 1,
                field_done: false,
                frame_done: false,
                newest_line_ns: 0,
            },
        );
        let _ = ex.take(&mut st).unwrap();
        p.publish(
            &frame(4, 4, 0.3),
            RowUpdate {
                rows: 3..10,
                lines: 1,
                field_done: false,
                frame_done: false,
                newest_line_ns: 0,
            },
        );
        let _ = ex.take(&mut st).unwrap();
        assert_eq!(st.ranges, vec![3..4]);
    }
}
