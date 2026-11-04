// SPDX-License-Identifier: GPL-3.0-or-later
//! On-screen display: formatting (pure, tested) and painting (a handful
//! of shapes per view, no layout widgets, so it stays cheap).
//!
//! Safety rule: while any picture cleanup is active the per-receiver C/N
//! bars are drawn even when the OSD is switched off.

use crate::commands::{BackendStatus, RecordingStatus};
use crate::gpu::{palette_index, PALETTE};
use crate::source::{LockState, SourceStatus};
use crate::stats::Summary;
use egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Stroke, StrokeKind, Vec2};

/// C/N at which a bar is full.
pub const CN_FULL_DB: f32 = 30.0;
/// Below this the bar is red (breakup imminent).
pub const CN_RED_DB: f32 = 8.0;
/// Below this the bar is yellow.
pub const CN_YELLOW_DB: f32 = 14.0;

/// "1.6 GB", "820 MB", "12 kB".
pub fn fmt_bytes(b: u64) -> String {
    let x = b as f64;
    if x >= 1e9 {
        format!("{:.1} GB", x / 1e9)
    } else if x >= 1e6 {
        format!("{:.0} MB", x / 1e6)
    } else if x >= 1e3 {
        format!("{:.0} kB", x / 1e3)
    } else {
        format!("{b} B")
    }
}

/// "26.7 MB/s".
pub fn fmt_byte_rate(bps: f64) -> String {
    format!("{:.1} MB/s", bps / 1e6)
}

/// "13.33 MS/s".
pub fn fmt_sample_rate(hz: f64) -> String {
    format!("{:.2} MS/s", hz / 1e6)
}

/// "12:34" or "1:02:03".
pub fn fmt_elapsed(ns: u64) -> String {
    let s = ns / 1_000_000_000;
    let (h, m, s) = (s / 3600, (s / 60) % 60, s % 60);
    if h > 0 {
        format!("{h}:{m:02}:{s:02}")
    } else {
        format!("{m}:{s:02}")
    }
}

/// "lat 2.1/3.0/6.4 ms" (min/avg/p99) or "lat --".
pub fn fmt_latency(s: &Summary) -> String {
    if s.count == 0 {
        "lat --".into()
    } else {
        format!("lat {:.1}/{:.1}/{:.1} ms", s.min_ms, s.avg_ms, s.p99_ms)
    }
}

/// "C/N 17.2" or "C/N --".
pub fn fmt_cn(cn: f32) -> String {
    if cn.is_finite() {
        format!("{cn:.1}")
    } else {
        "--".into()
    }
}

/// Bar fill 0..1.
pub fn cn_fraction(cn: f32) -> f32 {
    if cn.is_finite() {
        (cn / CN_FULL_DB).clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// Bar colour for a C/N.
pub fn cn_color(cn: f32) -> Color32 {
    if !cn.is_finite() || cn < CN_RED_DB {
        Color32::from_rgb(235, 50, 50)
    } else if cn < CN_YELLOW_DB {
        Color32::from_rgb(240, 200, 40)
    } else {
        Color32::from_rgb(60, 220, 90)
    }
}

/// "REC 2:13 1.6 GB".
pub fn fmt_rec(r: &RecordingStatus, now_ns: u64) -> Option<String> {
    r.active.then(|| {
        format!(
            "REC {} {}",
            fmt_elapsed(now_ns.saturating_sub(r.started_ns)),
            fmt_bytes(r.bytes)
        )
    })
}

fn palette_color(rx: u8) -> Color32 {
    let c = PALETTE[palette_index(rx)];
    Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8)
}

fn lock_color(l: LockState) -> Color32 {
    match l {
        LockState::Locked => Color32::from_rgb(60, 220, 90),
        LockState::Flywheel => Color32::from_rgb(240, 200, 40),
        _ => Color32::from_rgb(235, 50, 50),
    }
}

/// Text with a dark backing so it reads on any picture.
fn label(p: &Painter, pos: Pos2, anchor: Align2, text: &str, size: f32, color: Color32) -> Rect {
    let galley = p.layout_no_wrap(text.to_string(), FontId::monospace(size), color);
    let rect = anchor.anchor_size(pos, galley.size()).expand(2.0);
    p.rect_filled(rect, 2.0, Color32::from_black_alpha(150));
    p.galley(rect.min + Vec2::splat(2.0), galley, color);
    rect
}

/// What to draw for one view.
pub struct ViewOsd<'a> {
    pub status: &'a SourceStatus,
    pub latency: Summary,
    pub full: bool,
    pub goggle: bool,
    pub cleanup: bool,
    pub error: Option<&'a str>,
}

/// Paints a view's OSD inside `rect` (the picture area).
pub fn paint_view(p: &Painter, rect: Rect, o: &ViewOsd<'_>) {
    let scale = if o.goggle { 1.5 } else { 1.0 };
    let margin = if o.goggle { rect.width() * 0.06 } else { 8.0 };
    let inner = rect.shrink(margin);
    let text = 13.0 * scale;
    let s = o.status;

    if let Some(err) = o.error {
        label(p, rect.center(), Align2::CENTER_CENTER, err, text, Color32::from_rgb(255, 120, 120));
    }
    // C/N bars: always while cleanup is active (safety rule).
    if o.full || o.cleanup {
        let bar_w = 12.0 * scale;
        let bar_h = (inner.height() * 0.3).clamp(40.0, 160.0);
        let mut x = inner.right() - bar_w;
        for r in s.receivers.iter().rev() {
            let base = Pos2::new(x, inner.bottom() - text - 6.0);
            let frame = Rect::from_min_max(Pos2::new(x, base.y - bar_h), Pos2::new(x + bar_w, base.y));
            p.rect_filled(frame, 1.0, Color32::from_black_alpha(140));
            let f = cn_fraction(r.cn_db);
            let fill = Rect::from_min_max(Pos2::new(x, base.y - bar_h * f), base + Vec2::new(bar_w, 0.0));
            p.rect_filled(fill, 1.0, cn_color(r.cn_db));
            p.rect_stroke(frame, 1.0, Stroke::new(1.5, palette_color(r.rx)), StrokeKind::Outside);
            label(
                p,
                Pos2::new(x + bar_w / 2.0, inner.bottom()),
                Align2::CENTER_BOTTOM,
                &fmt_cn(r.cn_db),
                text * 0.8,
                Color32::WHITE,
            );
            x -= bar_w + 22.0 * scale;
        }
    }
    if !o.full {
        return;
    }
    // Top left: channel, standard, lock.
    let std = s.standard.map_or("--", |v| v.name());
    let head = format!("{}  {}", s.label, std);
    let r = label(p, inner.left_top(), Align2::LEFT_TOP, &head, text, Color32::WHITE);
    label(
        p,
        Pos2::new(r.right() + 6.0, r.top() + 2.0),
        Align2::LEFT_TOP,
        s.lock.name(),
        text,
        lock_color(s.lock),
    );
    if o.goggle {
        label(
            p,
            inner.left_bottom(),
            Align2::LEFT_BOTTOM,
            &fmt_latency(&o.latency),
            text,
            Color32::WHITE,
        );
        return;
    }
    // Bottom left: rates, drops, latency.
    let mut y = inner.bottom();
    let mut line = |t: &str, c: Color32| {
        let r = label(p, Pos2::new(inner.left(), y), Align2::LEFT_BOTTOM, t, text * 0.85, c);
        y = r.top() - 2.0;
    };
    line(&fmt_latency(&o.latency), Color32::WHITE);
    for r in &s.receivers {
        let drops = r.dropped_samples + r.dropped_transfers;
        let c = if drops > 0 {
            Color32::from_rgb(255, 180, 80)
        } else {
            Color32::LIGHT_GRAY
        };
        line(
            &format!(
                "rx{} {} {} {} gain {} drop {}",
                r.rx,
                if r.freq_mhz == 0 {
                    "ch on C5".to_string()
                } else {
                    crate::channels::label(r.freq_mhz)
                },
                fmt_sample_rate(r.sample_rate_hz),
                fmt_byte_rate(r.usb_bytes_per_s),
                if r.gain_on_c5 { "C5".to_string() } else { r.gain.to_string() },
                drops
            ),
            c,
        );
    }
    if !s.pipeline.is_empty() {
        line(
            &format!("{}  {:.2} fields/s", s.pipeline, s.field_rate_hz),
            Color32::LIGHT_GRAY,
        );
    }
    for w in &s.warnings {
        line(w, Color32::from_rgb(255, 120, 120));
    }
}

/// Paints the window-wide indicators (REC, cleanup state, backend,
/// warnings, recent messages).
pub fn paint_global(
    p: &Painter,
    rect: Rect,
    backend: &BackendStatus,
    cleanup: bool,
    messages: &[String],
    now_ns: u64,
    goggle: bool,
) {
    let scale = if goggle { 1.5 } else { 1.0 };
    let margin = if goggle { rect.width() * 0.06 } else { 8.0 };
    let inner = rect.shrink(margin);
    let text = 14.0 * scale;
    let mut x = inner.right();
    if let Some(rec) = fmt_rec(&backend.recording, now_ns) {
        let r = label(p, Pos2::new(x, inner.top()), Align2::RIGHT_TOP, &rec, text, Color32::WHITE);
        p.circle_filled(Pos2::new(r.left() - 10.0, r.center().y), 6.0 * scale, Color32::RED);
        x = r.left() - 22.0;
    }
    let (state, color) = if cleanup {
        ("CLEAN", Color32::from_rgb(120, 200, 255))
    } else {
        ("RAW", Color32::from_rgb(255, 210, 80))
    };
    let r = label(p, Pos2::new(x, inner.top()), Align2::RIGHT_TOP, state, text, color);
    if !backend.live && !backend.name.is_empty() {
        label(
            p,
            Pos2::new(r.left() - 8.0, inner.top()),
            Align2::RIGHT_TOP,
            "TEST",
            text,
            Color32::from_rgb(255, 150, 255),
        );
    }
    if goggle {
        return;
    }
    let mut y = inner.top() + text + 12.0;
    for w in backend.warnings.iter().chain(messages.iter()) {
        let r = label(
            p,
            Pos2::new(inner.right(), y),
            Align2::RIGHT_TOP,
            w,
            text * 0.8,
            Color32::from_rgb(255, 200, 160),
        );
        y = r.bottom() + 2.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats() {
        assert_eq!(fmt_bytes(500), "500 B");
        assert_eq!(fmt_bytes(12_000), "12 kB");
        assert_eq!(fmt_bytes(820_000_000), "820 MB");
        assert_eq!(fmt_bytes(1_600_000_000), "1.6 GB");
        assert_eq!(fmt_byte_rate(26.67e6), "26.7 MB/s");
        assert_eq!(fmt_sample_rate(240e6 / 18.0), "13.33 MS/s");
        assert_eq!(fmt_elapsed(754_000_000_000), "12:34");
        assert_eq!(fmt_elapsed(3_723_000_000_000), "1:02:03");
        assert_eq!(fmt_cn(f32::NAN), "--");
        assert_eq!(fmt_cn(17.25), "17.2");
    }

    #[test]
    fn latency_text() {
        assert_eq!(fmt_latency(&Summary::default()), "lat --");
        let s = Summary {
            count: 3,
            min_ms: 2.14,
            avg_ms: 3.0,
            p99_ms: 6.44,
            max_ms: 7.0,
        };
        assert_eq!(fmt_latency(&s), "lat 2.1/3.0/6.4 ms");
    }

    #[test]
    fn bars() {
        assert_eq!(cn_fraction(f32::NAN), 0.0);
        assert_eq!(cn_fraction(-3.0), 0.0);
        assert_eq!(cn_fraction(15.0), 0.5);
        assert_eq!(cn_fraction(50.0), 1.0);
        assert_eq!(cn_color(5.0), cn_color(f32::NAN));
        assert_ne!(cn_color(10.0), cn_color(20.0));
        assert_ne!(cn_color(5.0), cn_color(10.0));
    }

    #[test]
    fn rec_text() {
        let mut r = RecordingStatus::default();
        assert_eq!(fmt_rec(&r, 10), None);
        r.active = true;
        r.started_ns = 1_000_000_000;
        r.bytes = 1_600_000_000;
        assert_eq!(fmt_rec(&r, 62_000_000_000).unwrap(), "REC 1:01 1.6 GB");
    }
}
