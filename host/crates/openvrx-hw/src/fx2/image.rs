// SPDX-License-Identifier: GPL-3.0-or-later
//! FX2 firmware images: Intel HEX (RAM upload) and the C2 EEPROM boot format.
//!
//! The FX2LP loads RAM through vendor request `0xA0` into its internal
//! program/data RAM (`0x0000..0x4000`) and the scratch RAM
//! (`0xE000..0xE200`). The same segments packed in Cypress' "C2" format can
//! be written to a large I2C EEPROM so the chip boots the firmware without
//! the host.

use crate::error::{HwError, Result};

/// Internal program/data RAM of the CY7C68013A (16 KB).
pub const MAIN_RAM: std::ops::Range<u32> = 0x0000..0x4000;
/// Scratch RAM reachable by the loader.
pub const SCRATCH_RAM: std::ops::Range<u32> = 0xE000..0xE200;

/// A contiguous run of bytes at an address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Segment {
    pub addr: u32,
    pub data: Vec<u8>,
}

impl Segment {
    pub fn end(&self) -> u32 {
        self.addr + self.data.len() as u32
    }
}

/// A parsed firmware image: sorted, merged, non-overlapping segments.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FirmwareImage {
    pub segments: Vec<Segment>,
}

fn hex_err(line: usize, msg: impl Into<String>) -> HwError {
    HwError::Image { line, msg: msg.into() }
}

fn parse_hex_bytes(s: &str, line: usize) -> Result<Vec<u8>> {
    if !s.len().is_multiple_of(2) {
        return Err(hex_err(line, "odd number of hex digits"));
    }
    (0..s.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&s[i..i + 2], 16)
                .map_err(|_| hex_err(line, format!("bad hex digits `{}`", &s[i..i + 2])))
        })
        .collect()
}

impl FirmwareImage {
    /// Parses Intel HEX text. Supports record types 00 (data), 01 (end of
    /// file), 02 (extended segment address), 04 (extended linear address);
    /// 03 and 05 (start address) are accepted and ignored. Every checksum is
    /// verified; overlapping data with different contents is rejected.
    pub fn parse_ihex(text: &str) -> Result<FirmwareImage> {
        let mut raw: Vec<Segment> = Vec::new();
        let mut base: u32 = 0;
        let mut saw_eof = false;
        for (k, line_text) in text.lines().enumerate() {
            let line = k + 1;
            let t = line_text.trim();
            if t.is_empty() {
                continue;
            }
            if saw_eof {
                return Err(hex_err(line, "data after end-of-file record"));
            }
            let body = t
                .strip_prefix(':')
                .ok_or_else(|| hex_err(line, "record does not start with ':'"))?;
            let bytes = parse_hex_bytes(body, line)?;
            if bytes.len() < 5 {
                return Err(hex_err(line, "record too short"));
            }
            let count = bytes[0] as usize;
            if bytes.len() != count + 5 {
                return Err(hex_err(
                    line,
                    format!("length byte says {count} data bytes, record has {}", bytes.len() - 5),
                ));
            }
            let sum = bytes.iter().fold(0u8, |a, &b| a.wrapping_add(b));
            if sum != 0 {
                return Err(hex_err(line, "checksum mismatch"));
            }
            let offset = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
            let rtype = bytes[3];
            let data = &bytes[4..4 + count];
            match rtype {
                0x00 => {
                    if !data.is_empty() {
                        raw.push(Segment {
                            addr: base + offset,
                            data: data.to_vec(),
                        });
                    }
                }
                0x01 => saw_eof = true,
                0x02 => {
                    if count != 2 {
                        return Err(hex_err(line, "extended segment address needs 2 bytes"));
                    }
                    base = (u16::from_be_bytes([data[0], data[1]]) as u32) << 4;
                }
                0x04 => {
                    if count != 2 {
                        return Err(hex_err(line, "extended linear address needs 2 bytes"));
                    }
                    base = (u16::from_be_bytes([data[0], data[1]]) as u32) << 16;
                }
                0x03 | 0x05 => {}
                other => return Err(hex_err(line, format!("unknown record type {other:02X}"))),
            }
        }
        if !saw_eof {
            return Err(hex_err(text.lines().count(), "missing end-of-file record"));
        }
        Self::from_segments(raw)
    }

    /// Builds an image from arbitrary segments: sorts, checks overlaps and
    /// merges adjacent runs.
    pub fn from_segments(mut raw: Vec<Segment>) -> Result<FirmwareImage> {
        raw.sort_by_key(|s| s.addr);
        let mut out: Vec<Segment> = Vec::new();
        for seg in raw {
            if let Some(last) = out.last_mut() {
                if seg.addr < last.end() {
                    // Overlap: allowed only if the bytes agree.
                    let overlap_start = (seg.addr - last.addr) as usize;
                    let overlap_len = ((last.end() - seg.addr) as usize).min(seg.data.len());
                    if last.data[overlap_start..overlap_start + overlap_len] != seg.data[..overlap_len] {
                        return Err(hex_err(0, format!("conflicting data for address 0x{:04X}", seg.addr)));
                    }
                    if seg.end() > last.end() {
                        last.data.extend_from_slice(&seg.data[overlap_len..]);
                    }
                    continue;
                }
                if seg.addr == last.end() {
                    last.data.extend_from_slice(&seg.data);
                    continue;
                }
            }
            out.push(seg);
        }
        Ok(FirmwareImage { segments: out })
    }

    /// Total data bytes.
    pub fn len(&self) -> usize {
        self.segments.iter().map(|s| s.data.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Checks that every byte lands in RAM the `0xA0` loader can write.
    pub fn check_fx2_ram(&self) -> Result<()> {
        for s in &self.segments {
            let inside = |r: &std::ops::Range<u32>| s.addr >= r.start && s.end() <= r.end;
            if !inside(&MAIN_RAM) && !inside(&SCRATCH_RAM) {
                return Err(HwError::Invalid(format!(
                    "segment 0x{:04X}..0x{:04X} is outside the FX2 loadable RAM (0x0000..0x4000, 0xE000..0xE200)",
                    s.addr,
                    s.end()
                )));
            }
        }
        Ok(())
    }

    /// Splits the image into upload chunks of at most `max` bytes that do
    /// not cross a segment boundary.
    pub fn chunks(&self, max: usize) -> impl Iterator<Item = (u16, &[u8])> + '_ {
        let max = max.max(1);
        self.segments.iter().flat_map(move |s| {
            s.data
                .chunks(max)
                .enumerate()
                .map(move |(k, c)| ((s.addr as usize + k * max) as u16, c))
        })
    }

    /// Byte at an address, if the image covers it (for tests and checks).
    pub fn byte_at(&self, addr: u32) -> Option<u8> {
        self.segments
            .iter()
            .find(|s| addr >= s.addr && addr < s.end())
            .map(|s| s.data[(addr - s.addr) as usize])
    }

    /// Writes the image as Intel HEX (16 data bytes per record).
    pub fn to_ihex(&self) -> String {
        let mut out = String::new();
        let mut upper: u32 = 0;
        for (addr, chunk) in self.records(16) {
            if addr >> 16 != upper {
                upper = addr >> 16;
                let hi = (upper as u16).to_be_bytes();
                out.push_str(&ihex_record(0x04, 0, &hi));
            }
            out.push_str(&ihex_record(0x00, addr as u16, chunk));
        }
        out.push_str(":00000001FF\n");
        out
    }

    fn records(&self, n: usize) -> impl Iterator<Item = (u32, &[u8])> + '_ {
        self.segments.iter().flat_map(move |s| {
            s.data
                .chunks(n)
                .enumerate()
                .map(move |(k, c)| (s.addr + (k * n) as u32, c))
        })
    }
}

fn ihex_record(rtype: u8, addr: u16, data: &[u8]) -> String {
    let mut bytes = vec![data.len() as u8];
    bytes.extend_from_slice(&addr.to_be_bytes());
    bytes.push(rtype);
    bytes.extend_from_slice(data);
    let sum = bytes.iter().fold(0u8, |a, &b| a.wrapping_add(b));
    bytes.push(sum.wrapping_neg());
    let mut s = String::from(":");
    for b in bytes {
        s.push_str(&format!("{b:02X}"));
    }
    s.push('\n');
    s
}

/// C2 EEPROM boot image header fields.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct IicHeader {
    pub vid: u16,
    pub pid: u16,
    pub did: u16,
    /// Configuration byte (bit 6: 400 kHz I2C boot; bit 0: disconnect on
    /// boot). 0 is the conservative default.
    pub config: u8,
}

/// Maximum data bytes per C2 record (10-bit length field).
pub const IIC_MAX_RECORD: usize = 1023;

/// A parsed C2 EEPROM image.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IicImage {
    pub header: IicHeader,
    pub firmware: FirmwareImage,
}

impl IicImage {
    /// Packs a RAM image into the C2 format: header `C2 VIDL VIDH PIDL PIDH
    /// DIDL DIDH CONFIG`, then records `LENH LENL ADDRH ADDRL DATA...`, then
    /// the terminator `80 01 E6 00 00` that writes 0 to CPUCS (run).
    pub fn build(header: IicHeader, fw: &FirmwareImage) -> Result<Vec<u8>> {
        fw.check_fx2_ram()?;
        let mut out = vec![0xC2];
        out.extend_from_slice(&header.vid.to_le_bytes());
        out.extend_from_slice(&header.pid.to_le_bytes());
        out.extend_from_slice(&header.did.to_le_bytes());
        out.push(header.config);
        for (addr, chunk) in fw.chunks(IIC_MAX_RECORD) {
            let len = chunk.len() as u16;
            out.push((len >> 8) as u8 & 0x03);
            out.push(len as u8);
            out.extend_from_slice(&addr.to_be_bytes());
            out.extend_from_slice(chunk);
        }
        out.extend_from_slice(&[0x80, 0x01, 0xE6, 0x00, 0x00]);
        Ok(out)
    }

    /// Parses and validates a C2 image (as produced by `build` or by
    /// Cypress/libfx2 tools). Trailing bytes after the terminator (EEPROM
    /// padding) are ignored.
    pub fn parse(bytes: &[u8]) -> Result<IicImage> {
        let err = |pos: usize, msg: &str| HwError::Image {
            line: pos,
            msg: format!("C2 image byte {pos}: {msg}"),
        };
        if bytes.len() < 8 {
            return Err(err(0, "shorter than the 8-byte header"));
        }
        if bytes[0] != 0xC2 {
            return Err(err(0, &format!("first byte is 0x{:02X}, not 0xC2", bytes[0])));
        }
        let header = IicHeader {
            vid: u16::from_le_bytes([bytes[1], bytes[2]]),
            pid: u16::from_le_bytes([bytes[3], bytes[4]]),
            did: u16::from_le_bytes([bytes[5], bytes[6]]),
            config: bytes[7],
        };
        let mut pos = 8;
        let mut segs = Vec::new();
        loop {
            if pos + 4 > bytes.len() {
                return Err(err(pos, "image ends without the CPUCS terminator"));
            }
            let lenh = bytes[pos];
            let len = (((lenh & 0x03) as usize) << 8) | bytes[pos + 1] as usize;
            let addr = u16::from_be_bytes([bytes[pos + 2], bytes[pos + 3]]) as u32;
            if lenh & 0x80 != 0 {
                if len != 1 || addr != 0xE600 || pos + 5 > bytes.len() {
                    return Err(err(pos, "malformed terminator record"));
                }
                break;
            }
            pos += 4;
            if pos + len > bytes.len() {
                return Err(err(pos, "record runs past the end of the image"));
            }
            segs.push(Segment {
                addr,
                data: bytes[pos..pos + len].to_vec(),
            });
            pos += len;
        }
        let firmware = FirmwareImage::from_segments(segs)?;
        Ok(IicImage { header, firmware })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = ":0300000002000CEF\n\
:0A000C00758180E4F590F5A080FEF8\n\
:00000001FF\n";

    #[test]
    fn parses_and_merges() {
        let img = FirmwareImage::parse_ihex(SAMPLE).unwrap();
        assert_eq!(img.segments.len(), 2);
        assert_eq!(img.segments[0].addr, 0);
        assert_eq!(img.segments[0].data, vec![0x02, 0x00, 0x0C]);
        assert_eq!(img.segments[1].addr, 0x0C);
        assert_eq!(img.len(), 13);
        img.check_fx2_ram().unwrap();
        // Round trip through the writer.
        let again = FirmwareImage::parse_ihex(&img.to_ihex()).unwrap();
        assert_eq!(again, img);
    }

    #[test]
    fn adjacent_records_merge() {
        let mut fw = FirmwareImage::default();
        fw.segments.push(Segment {
            addr: 0x100,
            data: vec![1; 40],
        });
        let text = fw.to_ihex();
        // 40 bytes become 3 records but parse back into one segment.
        assert_eq!(text.lines().count(), 4);
        let back = FirmwareImage::parse_ihex(&text).unwrap();
        assert_eq!(back.segments.len(), 1);
        assert_eq!(back.segments[0].data.len(), 40);
    }

    #[test]
    fn rejects_bad_checksum_and_garbage() {
        let bad = SAMPLE.replace(":0300000002000CEF", ":0300000002000CEE");
        assert!(matches!(
            FirmwareImage::parse_ihex(&bad),
            Err(HwError::Image { line: 1, .. })
        ));
        assert!(FirmwareImage::parse_ihex("hello\n:00000001FF\n").is_err());
        assert!(FirmwareImage::parse_ihex(":0300000002000CEF\n").is_err(), "no EOF");
        assert!(
            FirmwareImage::parse_ihex(":0400000002000CEF\n:00000001FF\n").is_err(),
            "length"
        );
        assert!(
            FirmwareImage::parse_ihex(":00000001FF\n:0300000002000CEF\n").is_err(),
            "after EOF"
        );
    }

    #[test]
    fn extended_addresses() {
        // Type 02 base 0x1000 -> data at 0x10000 + 0x10.
        let text = format!(
            "{}{}:00000001FF\n",
            ihex_record(0x02, 0, &[0x10, 0x00]),
            ihex_record(0x00, 0x0010, &[0xAA, 0xBB])
        );
        let img = FirmwareImage::parse_ihex(&text).unwrap();
        assert_eq!(img.segments[0].addr, 0x10010);
        assert!(img.check_fx2_ram().is_err());
        let text = format!(
            "{}{}:00000001FF\n",
            ihex_record(0x04, 0, &[0x00, 0x00]),
            ihex_record(0x00, 0xE000, &[1, 2, 3])
        );
        let img = FirmwareImage::parse_ihex(&text).unwrap();
        img.check_fx2_ram().unwrap();
        assert_eq!(img.byte_at(0xE002), Some(3));
    }

    #[test]
    fn overlap_rules() {
        let same = vec![
            Segment {
                addr: 0,
                data: vec![1, 2, 3, 4],
            },
            Segment {
                addr: 2,
                data: vec![3, 4, 5],
            },
        ];
        let img = FirmwareImage::from_segments(same).unwrap();
        assert_eq!(img.segments[0].data, vec![1, 2, 3, 4, 5]);
        let conflict = vec![
            Segment {
                addr: 0,
                data: vec![1, 2, 3, 4],
            },
            Segment { addr: 2, data: vec![9] },
        ];
        assert!(FirmwareImage::from_segments(conflict).is_err());
    }

    #[test]
    fn chunks_do_not_cross_segments() {
        let img = FirmwareImage {
            segments: vec![
                Segment {
                    addr: 0,
                    data: vec![0; 2500],
                },
                Segment {
                    addr: 0x3000,
                    data: vec![1; 10],
                },
            ],
        };
        let c: Vec<(u16, usize)> = img.chunks(1024).map(|(a, d)| (a, d.len())).collect();
        assert_eq!(c, vec![(0, 1024), (1024, 1024), (2048, 452), (0x3000, 10)]);
    }

    #[test]
    fn iic_roundtrip() {
        let img = FirmwareImage {
            segments: vec![
                Segment {
                    addr: 0,
                    data: (0..2000u32).map(|x| x as u8).collect(),
                },
                Segment {
                    addr: 0xE000,
                    data: vec![7; 5],
                },
            ],
        };
        let hdr = IicHeader {
            vid: 0x1209,
            pid: 0x0001,
            did: 0x0100,
            config: 0x40,
        };
        let bytes = IicImage::build(hdr, &img).unwrap();
        assert_eq!(&bytes[..8], &[0xC2, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0x40]);
        assert_eq!(&bytes[bytes.len() - 5..], &[0x80, 0x01, 0xE6, 0x00, 0x00]);
        // First record: 1023 bytes at 0.
        assert_eq!(&bytes[8..12], &[0x03, 0xFF, 0x00, 0x00]);
        let mut padded = bytes.clone();
        padded.extend_from_slice(&[0xFF; 32]);
        let back = IicImage::parse(&padded).unwrap();
        assert_eq!(back.header, hdr);
        assert_eq!(back.firmware, img);
        assert!(IicImage::parse(&bytes[..bytes.len() - 5]).is_err());
        let mut wrong = bytes.clone();
        wrong[0] = 0xC0;
        assert!(IicImage::parse(&wrong).is_err());
    }
}
