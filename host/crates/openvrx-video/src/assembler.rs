// SPDX-License-Identifier: GPL-3.0-or-later
//! Field assembly: line records into composite `Field`s.
//!
//! Lines may arrive with gaps (lost lines stay at blanking with the LOST
//! damage value and UNKNOWN quality) and slightly out of order inside a
//! field. A field is complete when its last line arrives or when a line of
//! another field shows up. While a field is being filled the assembler
//! exposes it, together with how many leading lines are final, so a
//! streaming consumer can work line by line.

use openvrx_core::{Field, FieldParity, LineRecord, VideoStandard};

/// Result of pushing one line.
#[derive(Debug, Default)]
pub struct PushResult {
    /// Fields finished by this push, oldest first (normally zero or one).
    pub completed: Vec<Field>,
    /// The line was stored in the current field (field line index).
    pub stored: Option<usize>,
}

/// Counters for the OSD.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct AssemblerStats {
    pub fields: u64,
    pub lines: u64,
    pub missing_lines: u64,
    pub late_lines: u64,
    pub wrong_standard: u64,
}

pub struct FieldAssembler {
    standard: VideoStandard,
    current: Option<Field>,
    received: Vec<bool>,
    /// Highest line index stored in the current field, plus one.
    high_water: usize,
    /// (seq, parity) of the last completed field.
    last_done: Option<(u64, FieldParity)>,
    stats: AssemblerStats,
    /// A finished field handed back for reuse (avoids a fresh allocation,
    /// and its page faults, for every field).
    spare: Option<Field>,
}

impl FieldAssembler {
    pub fn new(standard: VideoStandard) -> FieldAssembler {
        FieldAssembler {
            standard,
            current: None,
            received: Vec::new(),
            high_water: 0,
            last_done: None,
            stats: AssemblerStats::default(),
            spare: None,
        }
    }

    /// Hands a field that is no longer needed back to the assembler, which
    /// reuses its buffers for a later field.
    pub fn recycle(&mut self, f: Field) {
        if f.standard == self.standard {
            self.spare = Some(f);
        }
    }

    /// A blank field (as `Field::new`), reusing the spare buffers if any.
    fn new_field(&mut self, seq: u64, parity: FieldParity) -> Field {
        let Some(mut f) = self.spare.take() else {
            return Field::new(self.standard, seq, parity);
        };
        let p = self.standard.params();
        let lines = p.field_lines(parity);
        let n = lines * f.width;
        f.seq = seq;
        f.parity = parity;
        f.lines = lines;
        f.video.clear();
        f.video.resize(n, 0.0);
        f.damage.clear();
        f.damage.resize(n, openvrx_core::quality::damage::LOST);
        f.quality.clear();
        f.quality.resize(lines, openvrx_core::LineQuality::UNKNOWN);
        f.source.clear();
        f.source.resize(lines, openvrx_core::COMBINED);
        f.first_ns = 0;
        f.last_ns = 0;
        f
    }

    pub fn standard(&self) -> VideoStandard {
        self.standard
    }

    pub fn stats(&self) -> AssemblerStats {
        self.stats
    }

    /// The field being filled, if any.
    pub fn current(&self) -> Option<&Field> {
        self.current.as_ref()
    }

    /// Number of leading lines of the current field that are final: every
    /// line below the highest line received is either present or lost
    /// (lines are produced in order, so a gap does not fill later).
    pub fn lines_final(&self) -> usize {
        self.high_water
    }

    /// Whether line `k` of the current field was received.
    pub fn has_line(&self, k: usize) -> bool {
        self.received.get(k).copied().unwrap_or(false)
    }

    pub fn push(&mut self, rec: &LineRecord) -> PushResult {
        let mut res = PushResult::default();
        if rec.standard != self.standard {
            self.stats.wrong_standard += 1;
            return res;
        }
        let p = self.standard.params();
        if rec.samples.len() != p.samples_per_line || rec.damage.len() != p.samples_per_line {
            self.stats.wrong_standard += 1;
            return res;
        }
        let key = (rec.field_seq, rec.parity);
        let same = self
            .current
            .as_ref()
            .map(|f| (f.seq, f.parity) == key)
            .unwrap_or(false);
        if !same {
            if let Some((s, _)) = self.last_done {
                if rec.field_seq < s || self.last_done == Some(key) {
                    self.stats.late_lines += 1;
                    return res;
                }
            }
            if let Some(f) = self.finish() {
                res.completed.push(f);
            }
            self.current = Some(self.new_field(rec.field_seq, rec.parity));
            self.received.clear();
            self.received.resize(p.field_lines(rec.parity), false);
            self.high_water = 0;
        }
        let k = rec.field_line as usize;
        let field = self.current.as_mut().expect("current field");
        if k >= field.lines {
            self.stats.late_lines += 1;
            return res;
        }
        if k < self.high_water && !self.received[k] {
            // A line arriving after later lines were already declared final
            // is too late for streaming consumers; drop it so that the
            // streamed and the completed field stay identical.
            self.stats.late_lines += 1;
            return res;
        }
        field.put_line(rec);
        self.received[k] = true;
        self.high_water = self.high_water.max(k + 1);
        self.stats.lines += 1;
        res.stored = Some(k);
        if k + 1 == field.lines {
            if let Some(f) = self.finish() {
                res.completed.push(f);
            }
        }
        res
    }

    /// Completes the current field (if any) and returns it.
    pub fn finish(&mut self) -> Option<Field> {
        let f = self.current.take()?;
        let missing = self.received.iter().filter(|&&r| !r).count();
        self.stats.missing_lines += missing as u64;
        self.stats.fields += 1;
        self.last_done = Some((f.seq, f.parity));
        self.high_water = 0;
        Some(f)
    }

    /// Drops any partial field and history.
    pub fn reset(&mut self) {
        self.current = None;
        self.received.clear();
        self.high_water = 0;
        self.last_done = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::quality::damage;
    use openvrx_core::LineQuality;

    fn rec(seq: u64, parity: FieldParity, line: u16, v: f32) -> LineRecord {
        let p = VideoStandard::Ntsc.params();
        LineRecord {
            rx: 0,
            standard: VideoStandard::Ntsc,
            field_seq: seq,
            parity,
            field_line: line,
            frame_line: p.frame_line(parity, line as usize),
            samples: vec![v; p.samples_per_line],
            damage: vec![0; p.samples_per_line],
            quality: LineQuality::UNKNOWN,
            src_position: 0.0,
            src_line_len: 0.0,
            host_ns: 1 + line as u64,
        }
    }

    #[test]
    fn missing_lines_and_completion() {
        let mut a = FieldAssembler::new(VideoStandard::Ntsc);
        for l in 0..263u16 {
            if l == 40 || l == 41 {
                continue;
            }
            let r = a.push(&rec(0, FieldParity::First, l, l as f32));
            if l < 262 {
                assert!(r.completed.is_empty());
            } else {
                assert_eq!(r.completed.len(), 1);
                let f = &r.completed[0];
                assert_eq!(f.line(10)[5], 10.0);
                assert_eq!(f.damage_line(40)[0], damage::LOST);
            }
        }
        assert_eq!(a.stats().missing_lines, 2);
        // A partial field is completed by the next field's first line.
        a.push(&rec(1, FieldParity::Second, 0, 1.0));
        a.push(&rec(1, FieldParity::Second, 1, 1.0));
        assert_eq!(a.lines_final(), 2);
        let r = a.push(&rec(2, FieldParity::First, 0, 1.0));
        assert_eq!(r.completed.len(), 1);
        assert_eq!(r.completed[0].seq, 1);
        // Late line of a finished field is dropped.
        let r = a.push(&rec(1, FieldParity::Second, 5, 1.0));
        assert!(r.stored.is_none());
        assert_eq!(a.stats().late_lines, 1);
    }
}
