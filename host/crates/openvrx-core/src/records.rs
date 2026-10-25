//! Data passed between pipeline stages.
//!
//! ```text
//! USB capture  -> RawBlock    (lane words as received, per receiver)
//! unpack       -> IqChunk     (Complex32, 10-bit ADC units)
//! demodulate   -> DemodChunk  (instantaneous frequency, envelope, damage)
//! sync / TBC   -> LineRecord  (one line on the line-locked grid, IRE)
//! combiner     -> LineRecord  (rx = COMBINED)
//! assembler    -> Field       (composite field, IRE)
//! colour       -> YuvField    (active picture only)
//! deinterlace  -> Frame       (progressive Y/U/V + damage)
//! ```
//!
//! Sample indices (`first_sample`, `src_position`) count words since the
//! receiver's stream started, so a recording can be replayed and any line
//! traced back to the raw bytes.

use crate::quality::LineQuality;
use crate::standard::{FieldParity, VideoStandard};
use crate::Complex32;
use serde::{Deserialize, Serialize};

/// Receiver index (0-based).
pub type RxId = u8;

/// `rx` value of a line produced by the diversity combiner.
pub const COMBINED: RxId = 0xff;

/// Bytes exactly as received from one FX2 bulk transfer.
#[derive(Clone, Debug)]
pub struct RawBlock {
    pub rx: RxId,
    /// Transfer sequence number (gaps mean lost transfers).
    pub seq: u64,
    /// Offset of `data[0]` in the receiver's byte stream.
    pub first_byte: u64,
    /// Host time when the transfer completed (clock::now_ns).
    pub host_ns: u64,
    pub data: Vec<u8>,
}

/// Unpacked complex samples.
#[derive(Clone, Debug)]
pub struct IqChunk {
    pub rx: RxId,
    /// Index of `iq[0]` in the receiver's sample stream.
    pub first_sample: u64,
    pub sample_rate_hz: f64,
    pub host_ns: u64,
    pub iq: Vec<Complex32>,
}

/// Demodulator output at the input sample rate.
#[derive(Clone, Debug)]
pub struct DemodChunk {
    pub rx: RxId,
    pub first_sample: u64,
    pub sample_rate_hz: f64,
    pub host_ns: u64,
    /// Instantaneous frequency in Hz (positive = above the tuned centre).
    pub freq_hz: Vec<f32>,
    /// Envelope |z| after the channel filter, ADC units.
    pub mag: Vec<f32>,
    /// Damage per sample (see `quality::damage`).
    pub damage: Vec<u8>,
}

/// One line resampled onto the standard's line-locked grid.
#[derive(Clone, Debug)]
pub struct LineRecord {
    pub rx: RxId,
    pub standard: VideoStandard,
    /// Monotonic field counter of this receiver (combined lines use the
    /// reference receiver's counter).
    pub field_seq: u64,
    pub parity: FieldParity,
    /// 0-based line within the field.
    pub field_line: u16,
    /// 1-based frame line number (standard numbering).
    pub frame_line: u16,
    /// Composite video in IRE, `samples_per_line` long, sample 0 at 0H.
    pub samples: Vec<f32>,
    /// Damage per grid sample, same length.
    pub damage: Vec<u8>,
    pub quality: LineQuality,
    /// Position of 0H in the receiver's input sample stream (fractional).
    pub src_position: f64,
    /// Measured input samples per line (tracks the transmitter's line rate
    /// against this receiver's clock).
    pub src_line_len: f64,
    /// Host time when the line was produced.
    pub host_ns: u64,
}

/// A whole composite field.
#[derive(Clone, Debug)]
pub struct Field {
    pub standard: VideoStandard,
    pub seq: u64,
    pub parity: FieldParity,
    /// Grid samples per line.
    pub width: usize,
    /// Lines in this field.
    pub lines: usize,
    /// Row-major IRE, lines x width.
    pub video: Vec<f32>,
    pub damage: Vec<u8>,
    pub quality: Vec<LineQuality>,
    /// Receiver that fed each line (COMBINED for blends).
    pub source: Vec<RxId>,
    /// Host time of the first and last line.
    pub first_ns: u64,
    pub last_ns: u64,
}

impl Field {
    pub fn new(standard: VideoStandard, seq: u64, parity: FieldParity) -> Field {
        let p = standard.params();
        let lines = p.field_lines(parity);
        let width = p.samples_per_line;
        Field {
            standard,
            seq,
            parity,
            width,
            lines,
            video: vec![0.0; lines * width],
            damage: vec![crate::quality::damage::LOST; lines * width],
            quality: vec![LineQuality::UNKNOWN; lines],
            source: vec![COMBINED; lines],
            first_ns: 0,
            last_ns: 0,
        }
    }

    #[inline]
    pub fn line(&self, k: usize) -> &[f32] {
        &self.video[k * self.width..(k + 1) * self.width]
    }

    #[inline]
    pub fn line_mut(&mut self, k: usize) -> &mut [f32] {
        &mut self.video[k * self.width..(k + 1) * self.width]
    }

    #[inline]
    pub fn damage_line(&self, k: usize) -> &[u8] {
        &self.damage[k * self.width..(k + 1) * self.width]
    }

    /// Copies a line record into its slot.
    pub fn put_line(&mut self, rec: &LineRecord) {
        let k = rec.field_line as usize;
        if k >= self.lines || rec.samples.len() != self.width {
            return;
        }
        self.line_mut(k).copy_from_slice(&rec.samples);
        self.damage[k * self.width..(k + 1) * self.width].copy_from_slice(&rec.damage);
        self.quality[k] = rec.quality;
        self.source[k] = rec.rx;
        if self.first_ns == 0 {
            self.first_ns = rec.host_ns;
        }
        self.last_ns = self.last_ns.max(rec.host_ns);
    }
}

/// Component (colour decoded) visible part of a field.
#[derive(Clone, Debug)]
pub struct YuvField {
    pub standard: VideoStandard,
    pub seq: u64,
    pub parity: FieldParity,
    /// Active samples per line (grid units, not square pixels).
    pub width: usize,
    /// Visible lines in this field.
    pub lines: usize,
    pub y: Vec<f32>,
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    pub damage: Vec<u8>,
    pub quality: Vec<LineQuality>,
    pub source: Vec<RxId>,
    pub first_ns: u64,
    pub last_ns: u64,
}

impl YuvField {
    pub fn new(standard: VideoStandard, seq: u64, parity: FieldParity) -> YuvField {
        let p = standard.params();
        let width = p.active_width();
        let lines = p.active_lines_per_field as usize;
        let n = width * lines;
        YuvField {
            standard,
            seq,
            parity,
            width,
            lines,
            y: vec![0.0; n],
            u: vec![0.0; n],
            v: vec![0.0; n],
            damage: vec![0; n],
            quality: vec![LineQuality::UNKNOWN; lines],
            source: vec![COMBINED; lines],
            first_ns: 0,
            last_ns: 0,
        }
    }
}

/// Metadata carried with a displayed or recorded frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct FrameMeta {
    pub standard: Option<VideoStandard>,
    /// Output frame counter.
    pub seq: u64,
    /// Newest field that contributed.
    pub field_seq: u64,
    /// Parity of the newest field.
    pub parity: Option<FieldParity>,
    /// Host time when the newest contributing line was demodulated.
    pub newest_line_ns: u64,
    /// Host time when the frame was finished.
    pub ready_ns: u64,
    /// Name of the deinterlacer and filters that produced it.
    pub pipeline: String,
}

/// Progressive component frame.
#[derive(Clone, Debug)]
pub struct Frame {
    pub width: usize,
    pub height: usize,
    /// Y 0..1, U/V BT.601 scale, row-major.
    pub y: Vec<f32>,
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    pub damage: Vec<u8>,
    /// Per output row: receiver that fed it and its quality.
    pub row_source: Vec<RxId>,
    pub row_quality: Vec<LineQuality>,
    pub meta: FrameMeta,
}

impl Frame {
    pub fn new(width: usize, height: usize) -> Frame {
        let n = width * height;
        Frame {
            width,
            height,
            y: vec![0.0; n],
            u: vec![0.0; n],
            v: vec![0.0; n],
            damage: vec![0; n],
            row_source: vec![COMBINED; height],
            row_quality: vec![LineQuality::UNKNOWN; height],
            meta: FrameMeta::default(),
        }
    }

    /// Converts to packed RGBA8 (BT.601, full range). For tests and file
    /// output; the live display converts on the GPU.
    pub fn to_rgba8(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.width * self.height * 4);
        for k in 0..self.width * self.height {
            let (r, g, b) = yuv_to_rgb(self.y[k], self.u[k], self.v[k]);
            out.extend_from_slice(&[to_u8(r), to_u8(g), to_u8(b), 255]);
        }
        out
    }
}

/// BT.601 YUV (Y 0..1) to linear-coded RGB 0..1.
#[inline]
pub fn yuv_to_rgb(y: f32, u: f32, v: f32) -> (f32, f32, f32) {
    (
        y + 1.139_83 * v,
        y - 0.394_65 * u - 0.580_60 * v,
        y + 2.032_11 * u,
    )
}

/// BT.601 RGB 0..1 to YUV.
#[inline]
pub fn rgb_to_yuv(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    let y = 0.299 * r + 0.587 * g + 0.114 * b;
    (y, 0.492 * (b - y), 0.877 * (r - y))
}

#[inline]
fn to_u8(x: f32) -> u8 {
    (x.clamp(0.0, 1.0) * 255.0 + 0.5) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yuv_roundtrip() {
        for (r, g, b) in [(1.0, 0.0, 0.0), (0.2, 0.7, 0.4), (1.0, 1.0, 1.0)] {
            let (y, u, v) = rgb_to_yuv(r, g, b);
            let (r2, g2, b2) = yuv_to_rgb(y, u, v);
            assert!((r - r2).abs() < 2e-3 && (g - g2).abs() < 2e-3 && (b - b2).abs() < 2e-3);
        }
    }

    #[test]
    fn field_slots() {
        let mut f = Field::new(VideoStandard::Ntsc, 0, FieldParity::Second);
        assert_eq!(f.lines, 262);
        let rec = LineRecord {
            rx: 1,
            standard: VideoStandard::Ntsc,
            field_seq: 0,
            parity: FieldParity::Second,
            field_line: 5,
            frame_line: 269,
            samples: vec![50.0; 910],
            damage: vec![0; 910],
            quality: LineQuality::UNKNOWN,
            src_position: 0.0,
            src_line_len: 847.0,
            host_ns: 7,
        };
        f.put_line(&rec);
        assert_eq!(f.line(5)[0], 50.0);
        assert_eq!(f.source[5], 1);
        assert_eq!(f.damage_line(4)[0], crate::quality::damage::LOST);
    }
}
