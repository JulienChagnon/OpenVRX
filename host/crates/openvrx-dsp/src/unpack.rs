// SPDX-License-Identifier: GPL-3.0-or-later
//! Raw FX2 bytes to I/Q.
//!
//! Output is structure-of-arrays (`re`, `im` in 10-bit ADC units) because
//! every later stage vectorises over SoA. Three decode paths:
//!
//! - 8-bit words: one 256-entry table per component.
//! - 16-bit words whose I and Q lanes each sit inside one byte (all the
//!   16-bit presets): two 256-entry tables, both L1 resident.
//! - Any other 16-bit layout: the full 65536-entry table.
//!
//! A 16-bit word split across two `push` calls is carried over. The raw
//! words can be kept as `u16` for the LUT angle demodulator and the phase
//! probe.

use openvrx_core::lanes::{LaneEncoding, LaneFormat};

#[derive(Clone, Debug)]
enum Path {
    /// 8-bit words.
    Byte { re: Box<[f32; 256]>, im: Box<[f32; 256]> },
    /// 16-bit words, I from byte `i_hi`(0 = low byte), Q from byte `q_hi`.
    Split {
        i_from_hi: bool,
        q_from_hi: bool,
        re: Box<[f32; 256]>,
        im: Box<[f32; 256]>,
    },
    /// Full table.
    Full { re: Vec<f32>, im: Vec<f32> },
}

/// Streaming unpacker.
#[derive(Clone, Debug)]
pub struct Unpacker {
    fmt: LaneFormat,
    path: Path,
    carry: Option<u8>,
}

impl Unpacker {
    pub fn new(fmt: &LaneFormat) -> Unpacker {
        let path = if fmt.word_bytes == 1 {
            let mut re = Box::new([0.0f32; 256]);
            let mut im = Box::new([0.0f32; 256]);
            for w in 0..256u32 {
                let z = fmt.decode(w);
                re[w as usize] = z.re;
                im[w as usize] = z.im;
            }
            Path::Byte { re, im }
        } else {
            match &fmt.encoding {
                LaneEncoding::Iq { i, q }
                    if one_byte(i.word_mask()).is_some() && one_byte(q.word_mask()).is_some() =>
                {
                    let i_from_hi = one_byte(i.word_mask()) == Some(true);
                    let q_from_hi = one_byte(q.word_mask()) == Some(true);
                    let mut re = Box::new([0.0f32; 256]);
                    let mut im = Box::new([0.0f32; 256]);
                    for b in 0..256u32 {
                        let wi = if i_from_hi { b << 8 } else { b };
                        let wq = if q_from_hi { b << 8 } else { b };
                        re[b as usize] = i.decode(wi);
                        im[b as usize] = q.decode(wq);
                    }
                    Path::Split {
                        i_from_hi,
                        q_from_hi,
                        re,
                        im,
                    }
                }
                _ => {
                    let lut = fmt.build_lut();
                    Path::Full {
                        re: lut.iter().map(|z| z.re).collect(),
                        im: lut.iter().map(|z| z.im).collect(),
                    }
                }
            }
        };
        Unpacker {
            fmt: fmt.clone(),
            path,
            carry: None,
        }
    }

    pub fn format(&self) -> &LaneFormat {
        &self.fmt
    }

    /// Drops a pending half word (after a gap).
    pub fn reset(&mut self) {
        self.carry = None;
    }

    /// Number of complete words `push` will produce for `n` more bytes.
    pub fn words_for(&self, n: usize) -> usize {
        if self.fmt.word_bytes == 1 {
            n
        } else {
            (n + self.carry.is_some() as usize) / 2
        }
    }

    /// Appends decoded samples (and raw words when `words` is given).
    pub fn push(
        &mut self,
        data: &[u8],
        re: &mut Vec<f32>,
        im: &mut Vec<f32>,
        mut words: Option<&mut Vec<u16>>,
    ) {
        let n_words = self.words_for(data.len());
        let r0 = re.len();
        re.resize(r0 + n_words, 0.0);
        im.resize(r0 + n_words, 0.0);
        let (ro, io) = (&mut re[r0..], &mut im[r0..]);
        match &self.path {
            Path::Byte { re: lr, im: li } => {
                for (k, &b) in data.iter().enumerate() {
                    ro[k] = lr[b as usize];
                    io[k] = li[b as usize];
                }
                if let Some(w) = words.as_deref_mut() {
                    w.extend(data.iter().map(|&b| b as u16));
                }
            }
            _ => {
                // Assemble the first word from the carry, then whole words.
                let mut data = data;
                let mut out = 0usize;
                if let Some(lo) = self.carry.take() {
                    if let Some((&hi, rest)) = data.split_first() {
                        let w = lo as u16 | (hi as u16) << 8;
                        self.decode16(&[w.to_le_bytes()[0], w.to_le_bytes()[1]], &mut ro[..1], &mut io[..1]);
                        if let Some(ws) = words.as_deref_mut() {
                            ws.push(w);
                        }
                        out = 1;
                        data = rest;
                    } else {
                        self.carry = Some(lo);
                        return;
                    }
                }
                let whole = data.len() / 2 * 2;
                self.decode16(&data[..whole], &mut ro[out..], &mut io[out..]);
                if let Some(ws) = words {
                    ws.extend(data[..whole].as_chunks::<2>().0.iter().map(|c| c[0] as u16 | (c[1] as u16) << 8));
                }
                if whole < data.len() {
                    self.carry = Some(data[whole]);
                }
            }
        }
    }

    fn decode16(&self, bytes: &[u8], ro: &mut [f32], io: &mut [f32]) {
        let n = bytes.len() / 2;
        let (ro, io) = (&mut ro[..n], &mut io[..n]);
        match &self.path {
            Path::Split {
                i_from_hi,
                q_from_hi,
                re,
                im,
            } => {
                let (ib, qb) = (*i_from_hi as usize, *q_from_hi as usize);
                for (k, c) in bytes.as_chunks::<2>().0.iter().enumerate() {
                    ro[k] = re[c[ib] as usize];
                    io[k] = im[c[qb] as usize];
                }
            }
            Path::Full { re, im } => {
                for (k, c) in bytes.as_chunks::<2>().0.iter().enumerate() {
                    let w = c[0] as usize | (c[1] as usize) << 8;
                    ro[k] = re[w];
                    io[k] = im[w];
                }
            }
            Path::Byte { .. } => unreachable!("8-bit words never reach decode16"),
        }
    }
}

/// Some(true) when the mask lies inside the high byte, Some(false) when
/// inside the low byte, None when it spans both.
fn one_byte(mask: u32) -> Option<bool> {
    if mask & !0xff == 0 {
        Some(false)
    } else if mask & !0xff00 == 0 {
        Some(true)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::lanes::{ComponentLanes, LanePreset};

    fn bytes_for(fmt: &LaneFormat, vals: &[(i32, i32)]) -> Vec<u8> {
        let mut out = Vec::new();
        for &(i, q) in vals {
            let w = fmt.encode(i, q);
            if fmt.word_bytes == 1 {
                out.push(w as u8);
            } else {
                out.extend_from_slice(&(w as u16).to_le_bytes());
            }
        }
        out
    }

    #[test]
    fn every_preset_matches_core_decode() {
        let vals: Vec<(i32, i32)> = (0..997)
            .map(|k| (((k * 37) % 1024) - 512, ((k * 101 + 5) % 1024) - 512))
            .collect();
        for p in LanePreset::ALL {
            let fmt = p.format();
            let bytes = bytes_for(&fmt, &vals);
            // Push in awkward pieces so 16-bit words split across calls.
            for split in [1usize, 3, 7, 64, 511] {
                let mut u = Unpacker::new(&fmt);
                let (mut re, mut im, mut ws) = (Vec::new(), Vec::new(), Vec::new());
                for chunk in bytes.chunks(split) {
                    u.push(chunk, &mut re, &mut im, Some(&mut ws));
                }
                assert_eq!(re.len(), vals.len(), "{p} split {split}");
                assert_eq!(ws.len(), vals.len());
                for (k, &(i, q)) in vals.iter().enumerate() {
                    let w = fmt.encode(i, q);
                    let z = fmt.decode(w);
                    assert_eq!((re[k], im[k]), (z.re, z.im), "{p} k={k}");
                    assert_eq!(ws[k] as u32, w);
                }
            }
        }
    }

    #[test]
    fn non_byte_aligned_layout_uses_full_table() {
        // I on word bits 11..4 (spans both bytes), Q on bits 3..0 and 15..12 is
        // not allowed (must be contiguous), so put Q on 15..12.
        let fmt = LaneFormat {
            word_bytes: 2,
            encoding: LaneEncoding::Iq {
                i: ComponentLanes::new(11, &[9, 8, 7, 6, 5, 4, 3, 2]),
                q: ComponentLanes::new(15, &[9, 8, 7, 6]),
            },
        };
        fmt.validate().unwrap();
        let u = Unpacker::new(&fmt);
        assert!(matches!(u.path, Path::Full { .. }));
        let vals = [(100, -200), (-512, 511), (3, 4)];
        let bytes = bytes_for(&fmt, &vals);
        let mut u = u;
        let (mut re, mut im) = (Vec::new(), Vec::new());
        u.push(&bytes, &mut re, &mut im, None);
        for (k, &(i, q)) in vals.iter().enumerate() {
            let z = fmt.decode(fmt.encode(i, q));
            assert_eq!((re[k], im[k]), (z.re, z.im));
        }
    }

    #[test]
    fn carry_survives_empty_and_reset() {
        let fmt = LanePreset::I8Q8.format();
        let mut u = Unpacker::new(&fmt);
        let (mut re, mut im) = (Vec::new(), Vec::new());
        let b = bytes_for(&fmt, &[(40, -40)]);
        u.push(&b[..1], &mut re, &mut im, None);
        u.push(&[], &mut re, &mut im, None);
        assert!(re.is_empty());
        u.push(&b[1..], &mut re, &mut im, None);
        assert_eq!(re.len(), 1);
        assert_eq!(re[0], 42.0);
        u.push(&b[..1], &mut re, &mut im, None);
        u.reset();
        u.push(&b, &mut re, &mut im, None);
        assert_eq!(re.len(), 2);
    }
}
