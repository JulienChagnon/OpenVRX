// SPDX-License-Identifier: GPL-3.0-or-later
//! When to present, and which swap-chain mode to ask for.
//!
//! Sources report new rows, finished fields and finished frames. The
//! scheduler turns those into present decisions according to the trigger
//! (every frame, every field, every N lines), with an optional rate cap
//! and a hold limit so a stalled trigger never freezes the picture.

use crate::settings::{PresentModeSetting, PresentTrigger};
use crate::source::UpdateInfo;

/// Longest time new rows may wait for their trigger before being shown.
pub const MAX_HOLD_NS: u64 = 45_000_000;

#[derive(Clone, Debug, PartialEq)]
pub struct PresentScheduler {
    trigger: PresentTrigger,
    lines_per_slice: u32,
    min_interval_ns: u64,
    pending_lines: u32,
    pending_fields: u32,
    pending_frames: u32,
    /// Time the oldest unpresented update arrived (0 = nothing pending).
    pending_since_ns: u64,
    last_present_ns: u64,
}

impl PresentScheduler {
    pub fn new(trigger: PresentTrigger, lines_per_slice: u32, max_present_hz: u32) -> Self {
        let mut s = PresentScheduler {
            trigger,
            lines_per_slice: 1,
            min_interval_ns: 0,
            pending_lines: 0,
            pending_fields: 0,
            pending_frames: 0,
            pending_since_ns: 0,
            last_present_ns: 0,
        };
        s.configure(trigger, lines_per_slice, max_present_hz);
        s
    }

    pub fn configure(&mut self, trigger: PresentTrigger, lines_per_slice: u32, max_present_hz: u32) {
        self.trigger = trigger;
        self.lines_per_slice = lines_per_slice.max(1);
        self.min_interval_ns = if max_present_hz == 0 {
            0
        } else {
            1_000_000_000 / max_present_hz as u64
        };
    }

    /// Records a source update received at `now_ns`.
    pub fn on_update(&mut self, info: &UpdateInfo, now_ns: u64) {
        if info.rows == 0 && info.fields_completed == 0 {
            return;
        }
        self.pending_lines = self.pending_lines.saturating_add(info.lines);
        self.pending_fields += info.fields_completed;
        self.pending_frames += info.frames_completed;
        if self.pending_since_ns == 0 {
            self.pending_since_ns = now_ns.max(1);
        }
        // A resize must show at once (new textures, new geometry).
        if info.resized {
            self.pending_frames += 1;
            self.pending_fields += 1;
        }
    }

    pub fn has_pending(&self) -> bool {
        self.pending_since_ns != 0
    }

    /// True when new content should be presented now.
    pub fn due(&self, now_ns: u64) -> bool {
        if !self.has_pending() {
            return false;
        }
        if self.min_interval_ns > 0
            && now_ns.saturating_sub(self.last_present_ns) < self.min_interval_ns
        {
            return false;
        }
        now_ns.saturating_sub(self.pending_since_ns) >= MAX_HOLD_NS || self.would_trigger()
    }

    /// Earliest time `due` can become true without a new update (rate cap
    /// or hold limit), for the event loop's wake-up deadline.
    pub fn next_deadline_ns(&self) -> Option<u64> {
        if !self.has_pending() {
            return None;
        }
        let hold = self.pending_since_ns + MAX_HOLD_NS;
        let cap = self.last_present_ns + self.min_interval_ns;
        Some(if self.would_trigger() { cap } else { hold.max(cap) })
    }

    fn would_trigger(&self) -> bool {
        match self.trigger {
            PresentTrigger::Frame => self.pending_frames > 0,
            PresentTrigger::Field => self.pending_fields > 0,
            PresentTrigger::Lines => {
                self.pending_fields > 0 || self.pending_lines >= self.lines_per_slice
            }
        }
    }

    /// Everything pending was just presented.
    pub fn presented(&mut self, now_ns: u64) {
        self.pending_lines = 0;
        self.pending_fields = 0;
        self.pending_frames = 0;
        self.pending_since_ns = 0;
        self.last_present_ns = now_ns;
    }
}

/// Picks the closest supported present mode. Immediate and Mailbox fall
/// back to each other before Fifo, which every surface supports.
pub fn choose_present_mode(
    wanted: PresentModeSetting,
    supported: &[wgpu::PresentMode],
) -> wgpu::PresentMode {
    use wgpu::PresentMode as P;
    let order: &[P] = match wanted {
        PresentModeSetting::Immediate => &[P::Immediate, P::Mailbox, P::Fifo],
        PresentModeSetting::Mailbox => &[P::Mailbox, P::Immediate, P::Fifo],
        PresentModeSetting::Fifo => &[P::Fifo],
    };
    order
        .iter()
        .copied()
        .find(|m| supported.contains(m))
        .unwrap_or(P::Fifo)
}

/// Short name of a wgpu present mode.
pub fn mode_name(mode: wgpu::PresentMode) -> &'static str {
    match mode {
        wgpu::PresentMode::Immediate => "Immediate",
        wgpu::PresentMode::Mailbox => "Mailbox",
        wgpu::PresentMode::Fifo => "Fifo",
        wgpu::PresentMode::FifoRelaxed => "FifoRelaxed",
        wgpu::PresentMode::AutoVsync => "AutoVsync",
        wgpu::PresentMode::AutoNoVsync => "AutoNoVsync",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upd(lines: u32, fields: u32, frames: u32) -> UpdateInfo {
        UpdateInfo {
            rows: lines * 2,
            lines,
            fields_completed: fields,
            frames_completed: frames,
            ..UpdateInfo::default()
        }
    }

    const MS: u64 = 1_000_000;

    #[test]
    fn field_trigger() {
        let mut s = PresentScheduler::new(PresentTrigger::Field, 32, 0);
        assert!(!s.due(10 * MS));
        s.on_update(&upd(100, 0, 0), 10 * MS);
        assert!(!s.due(11 * MS));
        s.on_update(&upd(140, 1, 0), 12 * MS);
        assert!(s.due(12 * MS));
        s.presented(12 * MS);
        assert!(!s.due(13 * MS));
        assert!(!s.has_pending());
    }

    #[test]
    fn frame_trigger() {
        let mut s = PresentScheduler::new(PresentTrigger::Frame, 32, 0);
        s.on_update(&upd(240, 1, 0), MS);
        assert!(!s.due(2 * MS));
        s.on_update(&upd(240, 1, 1), 3 * MS);
        assert!(s.due(3 * MS));
    }

    #[test]
    fn slice_trigger() {
        let mut s = PresentScheduler::new(PresentTrigger::Lines, 32, 0);
        s.on_update(&upd(16, 0, 0), MS);
        assert!(!s.due(MS));
        s.on_update(&upd(16, 0, 0), 2 * MS);
        assert!(s.due(2 * MS));
        s.presented(2 * MS);
        // The tail of a field is shown even when shorter than a slice.
        s.on_update(&upd(8, 1, 0), 3 * MS);
        assert!(s.due(3 * MS));
    }

    #[test]
    fn rate_cap() {
        let mut s = PresentScheduler::new(PresentTrigger::Lines, 8, 500);
        s.on_update(&upd(8, 0, 0), 10 * MS);
        assert!(s.due(10 * MS));
        s.presented(10 * MS);
        s.on_update(&upd(8, 0, 0), 10 * MS + 500_000);
        assert!(!s.due(10 * MS + 500_000));
        assert_eq!(s.next_deadline_ns(), Some(12 * MS));
        assert!(s.due(12 * MS));
    }

    #[test]
    fn hold_limit() {
        let mut s = PresentScheduler::new(PresentTrigger::Frame, 32, 0);
        s.on_update(&upd(10, 0, 0), 100 * MS);
        assert!(!s.due(100 * MS + MAX_HOLD_NS - 1));
        assert_eq!(s.next_deadline_ns(), Some(100 * MS + MAX_HOLD_NS));
        assert!(s.due(100 * MS + MAX_HOLD_NS));
    }

    #[test]
    fn resize_presents_at_once() {
        let mut s = PresentScheduler::new(PresentTrigger::Frame, 32, 0);
        let mut u = upd(8, 0, 0);
        u.resized = true;
        s.on_update(&u, MS);
        assert!(s.due(MS));
    }

    #[test]
    fn empty_update_ignored() {
        let mut s = PresentScheduler::new(PresentTrigger::Lines, 1, 0);
        s.on_update(&UpdateInfo::default(), MS);
        assert!(!s.has_pending());
    }

    #[test]
    fn present_mode_fallbacks() {
        use wgpu::PresentMode as P;
        let all = [P::Fifo, P::Mailbox, P::Immediate];
        assert_eq!(choose_present_mode(PresentModeSetting::Immediate, &all), P::Immediate);
        assert_eq!(
            choose_present_mode(PresentModeSetting::Immediate, &[P::Fifo, P::Mailbox]),
            P::Mailbox
        );
        assert_eq!(
            choose_present_mode(PresentModeSetting::Mailbox, &[P::Fifo, P::Immediate]),
            P::Immediate
        );
        assert_eq!(choose_present_mode(PresentModeSetting::Mailbox, &[P::Fifo]), P::Fifo);
        assert_eq!(choose_present_mode(PresentModeSetting::Fifo, &all), P::Fifo);
        assert_eq!(choose_present_mode(PresentModeSetting::Fifo, &[]), P::Fifo);
    }
}
