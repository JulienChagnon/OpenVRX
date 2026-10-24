//! Analog video raster geometry.
//!
//! Every line is resampled (time-base corrected) onto a line-locked grid of
//! `samples_per_line` samples, with sample 0 at 0H (the 50 % point of the
//! falling edge of horizontal sync) and the last sample just before the
//! next line's 0H.
//!
//! - NTSC-M: 910 samples per line = exactly 4 x fsc (14.318181 MHz).
//! - PAL-B/G: 1135 samples per line (17.734375 MHz). True 4 x fsc for PAL
//!   is 1135.0064 samples per line; the subcarrier therefore slips by
//!   0.0064 samples a line on this grid and colour decoders must lock to
//!   each line's burst rather than assume a fixed phase.
//!
//! Frame lines use the standard 1-based numbering (NTSC 1..=525, PAL
//! 1..=625). Field 1 is frame lines 1..=263 (NTSC) or 1..=313 (PAL), field
//! 2 the rest. The half line in the middle of the frame belongs to field 1.

use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum VideoStandard {
    Ntsc,
    Pal,
}

impl VideoStandard {
    pub const ALL: [VideoStandard; 2] = [VideoStandard::Ntsc, VideoStandard::Pal];

    pub fn params(self) -> &'static StandardParams {
        match self {
            VideoStandard::Ntsc => &NTSC,
            VideoStandard::Pal => &PAL,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            VideoStandard::Ntsc => "NTSC",
            VideoStandard::Pal => "PAL",
        }
    }
}

impl std::str::FromStr for VideoStandard {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s.to_ascii_lowercase().as_str() {
            "ntsc" => Ok(VideoStandard::Ntsc),
            "pal" => Ok(VideoStandard::Pal),
            other => Err(format!("unknown video standard `{other}` (ntsc|pal)")),
        }
    }
}

/// Field 1 (frame starts with it) or field 2.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FieldParity {
    First,
    Second,
}

impl FieldParity {
    #[inline]
    pub fn index(self) -> usize {
        match self {
            FieldParity::First => 0,
            FieldParity::Second => 1,
        }
    }
    #[inline]
    pub fn other(self) -> FieldParity {
        match self {
            FieldParity::First => FieldParity::Second,
            FieldParity::Second => FieldParity::First,
        }
    }
}

/// Raster constants. Times are in microseconds from 0H, converted to grid
/// samples by the helper methods.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StandardParams {
    pub standard: VideoStandard,
    /// Line frequency in Hz.
    pub line_hz: f64,
    /// Colour subcarrier frequency in Hz.
    pub fsc_hz: f64,
    pub lines_per_frame: u16,
    /// Lines in field 1 and field 2 (field 1 holds the half line).
    pub lines_per_field: [u16; 2],
    /// Output grid: samples per line (sample 0 at 0H).
    pub samples_per_line: usize,
    /// Horizontal sync width (us).
    pub hsync_us: f64,
    /// Equalising pulse width (us).
    pub eq_pulse_us: f64,
    /// Broad (vertical sync) pulse width (us).
    pub broad_pulse_us: f64,
    /// Burst start after 0H (us) and length in subcarrier cycles.
    pub burst_start_us: f64,
    pub burst_cycles: f64,
    /// Active picture start after 0H and width (us).
    pub active_start_us: f64,
    pub active_us: f64,
    /// Levels in IRE (blanking 0, peak white 100).
    pub sync_ire: f32,
    pub black_ire: f32,
    pub white_ire: f32,
    /// Burst amplitude, half peak to peak, IRE.
    pub burst_ire: f32,
    /// Visible lines per field and the frame line where each field's first
    /// visible line sits ([field 1, field 2]).
    pub active_lines_per_field: u16,
    pub first_active_line: [u16; 2],
    /// Which field holds the top output row.
    pub top_field: FieldParity,
    /// Nominal field rate in Hz.
    pub field_hz: f64,
}

pub static NTSC: StandardParams = StandardParams {
    standard: VideoStandard::Ntsc,
    line_hz: 4.5e6 / 286.0,
    fsc_hz: 315.0e6 / 88.0,
    lines_per_frame: 525,
    lines_per_field: [263, 262],
    samples_per_line: 910,
    hsync_us: 4.7,
    eq_pulse_us: 2.3,
    broad_pulse_us: 27.1,
    burst_start_us: 5.3,
    burst_cycles: 9.0,
    active_start_us: 9.4,
    active_us: 52.66,
    sync_ire: -40.0,
    black_ire: 7.5,
    white_ire: 100.0,
    burst_ire: 20.0,
    active_lines_per_field: 240,
    first_active_line: [21, 283],
    top_field: FieldParity::Second,
    field_hz: 60000.0 / 1001.0,
};

pub static PAL: StandardParams = StandardParams {
    standard: VideoStandard::Pal,
    line_hz: 15625.0,
    fsc_hz: 4_433_618.75,
    lines_per_frame: 625,
    lines_per_field: [313, 312],
    samples_per_line: 1135,
    hsync_us: 4.7,
    eq_pulse_us: 2.35,
    broad_pulse_us: 27.3,
    burst_start_us: 5.6,
    burst_cycles: 10.0,
    active_start_us: 10.4,
    active_us: 52.0,
    sync_ire: -300.0 / 7.0,
    black_ire: 0.0,
    white_ire: 100.0,
    burst_ire: 150.0 / 7.0,
    active_lines_per_field: 288,
    first_active_line: [23, 336],
    top_field: FieldParity::First,
    field_hz: 50.0,
};

impl StandardParams {
    /// Grid sample rate in Hz (samples_per_line x line rate).
    #[inline]
    pub fn grid_rate_hz(&self) -> f64 {
        self.samples_per_line as f64 * self.line_hz
    }

    /// Line period in seconds.
    #[inline]
    pub fn line_period_s(&self) -> f64 {
        1.0 / self.line_hz
    }

    /// Converts microseconds after 0H into grid samples (fractional).
    #[inline]
    pub fn us_to_samples(&self, us: f64) -> f64 {
        us * 1e-6 * self.grid_rate_hz()
    }

    /// Subcarrier cycles per grid sample (0.25 for NTSC, ~0.25 for PAL).
    #[inline]
    pub fn fsc_per_sample(&self) -> f64 {
        self.fsc_hz / self.grid_rate_hz()
    }

    pub fn hsync_samples(&self) -> usize {
        self.us_to_samples(self.hsync_us).round() as usize
    }

    /// Burst window [start, end) in grid samples.
    pub fn burst_range(&self) -> std::ops::Range<usize> {
        let start = self.us_to_samples(self.burst_start_us).round() as usize;
        let len = (self.burst_cycles / self.fsc_per_sample()).round() as usize;
        start..start + len
    }

    /// Active picture [start, end) in grid samples.
    pub fn active_range(&self) -> std::ops::Range<usize> {
        let start = self.us_to_samples(self.active_start_us).round() as usize;
        let len = self.us_to_samples(self.active_us).round() as usize;
        start..(start + len).min(self.samples_per_line)
    }

    /// Active picture width in grid samples.
    pub fn active_width(&self) -> usize {
        self.active_range().len()
    }

    /// A flat stretch of the back porch after the burst ends, used for
    /// noise and blanking level measurement.
    pub fn back_porch_range(&self) -> std::ops::Range<usize> {
        let b = self.burst_range();
        let a = self.active_range();
        (b.end + 2)..a.start.saturating_sub(2).max(b.end + 3)
    }

    /// Flat part of the sync tip (skips the edges).
    pub fn sync_tip_range(&self) -> std::ops::Range<usize> {
        let w = self.hsync_samples();
        (w / 6)..(w - w / 6)
    }

    /// Visible frame height (both fields).
    pub fn frame_height(&self) -> usize {
        2 * self.active_lines_per_field as usize
    }

    /// Display aspect ratio of the active picture.
    pub fn display_aspect(&self) -> f64 {
        4.0 / 3.0
    }

    /// Lines in the given field.
    #[inline]
    pub fn field_lines(&self, parity: FieldParity) -> usize {
        self.lines_per_field[parity.index()] as usize
    }

    /// First frame line (1-based) of the given field.
    #[inline]
    pub fn field_first_frame_line(&self, parity: FieldParity) -> u16 {
        match parity {
            FieldParity::First => 1,
            FieldParity::Second => self.lines_per_field[0] + 1,
        }
    }

    /// Frame line (1-based) for a 0-based line index within a field.
    #[inline]
    pub fn frame_line(&self, parity: FieldParity, field_line: usize) -> u16 {
        self.field_first_frame_line(parity) + field_line as u16
    }

    /// Output row (0-based, top to bottom) of a frame line, or None when the
    /// line is outside the visible area.
    pub fn output_row(&self, frame_line: u16) -> Option<usize> {
        let parity = if frame_line <= self.lines_per_field[0] {
            FieldParity::First
        } else {
            FieldParity::Second
        };
        let first = self.first_active_line[parity.index()];
        if frame_line < first {
            return None;
        }
        let k = (frame_line - first) as usize;
        if k >= self.active_lines_per_field as usize {
            return None;
        }
        let offset = if parity == self.top_field { 0 } else { 1 };
        Some(2 * k + offset)
    }

    /// Row offset (0 or 1) of a field's lines in the woven frame.
    #[inline]
    pub fn field_row_offset(&self, parity: FieldParity) -> usize {
        if parity == self.top_field {
            0
        } else {
            1
        }
    }

    /// Index within the field (0-based) of the first visible line.
    #[inline]
    pub fn first_active_field_line(&self, parity: FieldParity) -> usize {
        (self.first_active_line[parity.index()] - self.field_first_frame_line(parity)) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_rates() {
        assert!((NTSC.grid_rate_hz() - 4.0 * NTSC.fsc_hz).abs() < 1e-3);
        assert!((PAL.grid_rate_hz() - 17_734_375.0).abs() < 1e-6);
        assert!((NTSC.fsc_per_sample() - 0.25).abs() < 1e-12);
        assert!((PAL.fsc_per_sample() - 0.25).abs() < 1e-5);
    }

    #[test]
    fn field_line_counts() {
        for p in [&NTSC, &PAL] {
            assert_eq!(
                p.lines_per_field[0] + p.lines_per_field[1],
                p.lines_per_frame
            );
        }
    }

    #[test]
    fn rows_cover_frame_once() {
        for p in [&NTSC, &PAL] {
            let mut seen = vec![false; p.frame_height()];
            for line in 1..=p.lines_per_frame {
                if let Some(r) = p.output_row(line) {
                    assert!(!seen[r], "{:?} row {r} twice", p.standard);
                    seen[r] = true;
                }
            }
            assert!(seen.iter().all(|&s| s), "{:?} missing rows", p.standard);
        }
    }

    #[test]
    fn regions_ordered() {
        for p in [&NTSC, &PAL] {
            let b = p.burst_range();
            let a = p.active_range();
            assert!(p.hsync_samples() < b.start);
            assert!(b.end < p.back_porch_range().start);
            assert!(p.back_porch_range().end <= a.start);
            assert!(a.end <= p.samples_per_line);
        }
        assert_eq!(NTSC.burst_range().len(), 36);
        assert_eq!(PAL.burst_range().len(), 40);
    }
}
