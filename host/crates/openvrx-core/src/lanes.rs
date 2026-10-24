//! Lane formats: which C5 ADC bits travel on which FX2 port bits.
//!
//! The C5's MODEM_DIAG bus carries Q on bits 0..9 and I on bits 10..19,
//! each a 10-bit two's complement ADC value (only bits 9..2 are live). The
//! C5 routes a chosen subset of those bits to pads through the GPIO matrix;
//! the FX2 latches the pads as an 8-bit or 16-bit word on every sample clock
//! edge.
//!
//! A component (I or Q) is described by the ADC bits it carries, most
//! significant first, and the word bit holding the first of them; the
//! following ADC bits sit on the following lower word bits. The carried
//! bits are read back as an n-bit two's complement number `t` and scaled
//! to 10-bit ADC units as `t * 2^lsb + 2^lsb / 2`, where `lsb` is the last
//! listed ADC bit. For contiguous bits from the sign down this is ordinary
//! truncation; for a gap after the sign bit (C5VRX's ultrafine {9,6,5,4}
//! slice) it is a folded window of +-2^(lsb+n-1) ADC units.
//!
//! 16-bit words: Q on the low byte (FX2 PB0..PB7 = FD0..FD7), I on the high
//! byte (PD0..PD7 = FD8..FD15), MSB aligned. Unused low lanes are tied to
//! GND. 8-bit words with 4+4 bits: Q on FD0..FD3, I on FD4..FD7, as C5VRX's
//! PARLIO RX byte.

use crate::{Complex32, CoreError};
use serde::{Deserialize, Serialize};

/// PARLIO TX clock source on the C5: PLL_F240M.
pub const C5_PLL_HZ: f64 = 240e6;
/// The modem's MODEM_DIAG bus rate (one value per 3 PLL ticks).
pub const C5_BUS_HZ: f64 = 80e6;

/// One component's lanes.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ComponentLanes {
    /// Word bit carrying the first (most significant) listed ADC bit.
    pub word_msb: u8,
    /// ADC bit numbers 0..=9, strictly descending. The first acts as sign.
    pub adc_bits: Vec<u8>,
}

impl ComponentLanes {
    pub fn new(word_msb: u8, adc_bits: &[u8]) -> Self {
        ComponentLanes {
            word_msb,
            adc_bits: adc_bits.to_vec(),
        }
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.adc_bits.len() as u32
    }

    /// Word bit of the least significant carried ADC bit.
    #[inline]
    pub fn word_lsb(&self) -> u8 {
        self.word_msb + 1 - self.adc_bits.len() as u8
    }

    /// Mask of word bits used.
    pub fn word_mask(&self) -> u32 {
        ((1u32 << self.width()) - 1) << self.word_lsb()
    }

    /// Lowest carried ADC bit.
    #[inline]
    pub fn adc_lsb(&self) -> u8 {
        *self.adc_bits.last().expect("validated non-empty")
    }

    /// Reads the carried bits as two's complement and scales them to 10-bit
    /// ADC units with half-step centring.
    #[inline]
    pub fn decode(&self, word: u32) -> f32 {
        let n = self.width();
        let raw = (word >> self.word_lsb()) & ((1 << n) - 1);
        let t = ((raw << (32 - n)) as i32) >> (32 - n);
        let step = (1u32 << self.adc_lsb()) as f32;
        t as f32 * step + step * 0.5
    }

    /// Extracts the carried bits from a 10-bit ADC value and places them in
    /// the word (used by the simulator and by self-test checks).
    #[inline]
    pub fn encode(&self, adc10: i32) -> u32 {
        let v = (adc10.clamp(-512, 511) as u32) & 0x3ff;
        let mut out = 0u32;
        let mut wb = self.word_msb as i32;
        for &b in &self.adc_bits {
            out |= ((v >> b) & 1) << wb;
            wb -= 1;
        }
        out
    }
}

/// How samples are encoded in a word.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum LaneEncoding {
    /// I and Q bits.
    Iq { i: ComponentLanes, q: ComponentLanes },
    /// Phase angle only (computed on the C5), `bits` wide, MSB at
    /// `word_msb`. 0 = 0 rad, full scale = 2 pi. No magnitude.
    Angle { word_msb: u8, bits: u8 },
}

/// A complete lane format.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LaneFormat {
    /// 1 (8-bit FX2 port, WORDWIDE=0) or 2 (16-bit, WORDWIDE=1, little
    /// endian on the wire: FD0..7 first).
    pub word_bytes: u8,
    pub encoding: LaneEncoding,
}

/// Named lane formats. The name is what the C5 console `lanes` command,
/// the recording sidecar and the CLI use.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LanePreset {
    /// 16 data pins: ADC bits 9..2 of I and Q.
    #[serde(rename = "8x8")]
    I8Q8,
    /// 14 data pins: bits 9..3 (fits the XIAO with its back pads).
    #[serde(rename = "7x7")]
    I7Q7,
    /// 12 data pins: bits 9..4.
    #[serde(rename = "6x6")]
    I6Q6,
    /// 10 data pins: bits 9..5 (fits the XIAO's 11 front pins with the clock).
    #[serde(rename = "5x5")]
    I5Q5,
    /// 8 data pins, 8-bit words: bits 9..6.
    #[serde(rename = "4x4top")]
    I4Q4Top,
    /// 8 data pins: bits {9,7,6,5} (folded +-256 window).
    #[serde(rename = "4x4fine")]
    I4Q4Fine,
    /// 8 data pins: C5VRX's ultrafine bits {9,6,5,4} (folded +-128 window).
    #[serde(rename = "4x4ultra")]
    I4Q4Ultra,
    /// 8 data pins, 8-bit words: phase angle computed on the C5.
    #[serde(rename = "angle8")]
    Angle8,
}

impl LanePreset {
    pub const ALL: [LanePreset; 8] = [
        LanePreset::I8Q8,
        LanePreset::I7Q7,
        LanePreset::I6Q6,
        LanePreset::I5Q5,
        LanePreset::I4Q4Top,
        LanePreset::I4Q4Fine,
        LanePreset::I4Q4Ultra,
        LanePreset::Angle8,
    ];

    pub fn name(self) -> &'static str {
        match self {
            LanePreset::I8Q8 => "8x8",
            LanePreset::I7Q7 => "7x7",
            LanePreset::I6Q6 => "6x6",
            LanePreset::I5Q5 => "5x5",
            LanePreset::I4Q4Top => "4x4top",
            LanePreset::I4Q4Fine => "4x4fine",
            LanePreset::I4Q4Ultra => "4x4ultra",
            LanePreset::Angle8 => "angle8",
        }
    }

    /// Data pins on the C5 (the sample clock needs one more).
    pub fn data_pins(self) -> usize {
        match self.format().encoding {
            LaneEncoding::Iq { i, q } => i.adc_bits.len() + q.adc_bits.len(),
            LaneEncoding::Angle { bits, .. } => bits as usize,
        }
    }

    pub fn format(self) -> LaneFormat {
        let iq16 = |n: usize| {
            let bits: Vec<u8> = (0..n).map(|k| 9 - k as u8).collect();
            LaneFormat {
                word_bytes: 2,
                encoding: LaneEncoding::Iq {
                    i: ComponentLanes::new(15, &bits),
                    q: ComponentLanes::new(7, &bits),
                },
            }
        };
        let iq8 = |bits: [u8; 4]| LaneFormat {
            word_bytes: 1,
            encoding: LaneEncoding::Iq {
                i: ComponentLanes::new(7, &bits),
                q: ComponentLanes::new(3, &bits),
            },
        };
        match self {
            LanePreset::I8Q8 => iq16(8),
            LanePreset::I7Q7 => iq16(7),
            LanePreset::I6Q6 => iq16(6),
            LanePreset::I5Q5 => iq16(5),
            LanePreset::I4Q4Top => iq8([9, 8, 7, 6]),
            LanePreset::I4Q4Fine => iq8([9, 7, 6, 5]),
            LanePreset::I4Q4Ultra => iq8([9, 6, 5, 4]),
            LanePreset::Angle8 => LaneFormat {
                word_bytes: 1,
                encoding: LaneEncoding::Angle {
                    word_msb: 7,
                    bits: 8,
                },
            },
        }
    }
}

impl std::str::FromStr for LanePreset {
    type Err = CoreError;
    fn from_str(s: &str) -> Result<Self, CoreError> {
        LanePreset::ALL
            .iter()
            .copied()
            .find(|p| p.name().eq_ignore_ascii_case(s))
            .ok_or_else(|| CoreError::UnknownPreset(s.to_string()))
    }
}

impl std::fmt::Display for LanePreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

impl LaneFormat {
    pub fn validate(&self) -> Result<(), CoreError> {
        if self.word_bytes != 1 && self.word_bytes != 2 {
            return Err(CoreError::LaneFormat("word_bytes must be 1 or 2".into()));
        }
        let word_bits = 8 * self.word_bytes as u32;
        let check = |c: &ComponentLanes, name: &str| -> Result<u32, CoreError> {
            if c.adc_bits.is_empty() || c.adc_bits.len() > 10 {
                return Err(CoreError::LaneFormat(format!("{name}: 1..=10 bits")));
            }
            if c.adc_bits.windows(2).any(|w| w[0] <= w[1]) || c.adc_bits[0] > 9 {
                return Err(CoreError::LaneFormat(format!(
                    "{name}: ADC bits must be strictly descending and <= 9"
                )));
            }
            if (c.word_msb as u32) >= word_bits || (c.word_msb as usize) + 1 < c.adc_bits.len() {
                return Err(CoreError::LaneFormat(format!("{name}: does not fit the word")));
            }
            Ok(c.word_mask())
        };
        match &self.encoding {
            LaneEncoding::Iq { i, q } => {
                let mi = check(i, "I")?;
                let mq = check(q, "Q")?;
                if mi & mq != 0 {
                    return Err(CoreError::LaneFormat("I and Q lanes overlap".into()));
                }
            }
            LaneEncoding::Angle { word_msb, bits } => {
                if *bits == 0 || *bits as u32 > word_bits || (*word_msb as u32) >= word_bits {
                    return Err(CoreError::LaneFormat("angle does not fit the word".into()));
                }
            }
        }
        Ok(())
    }

    /// True when samples carry amplitude (I/Q); false for angle-only.
    pub fn has_magnitude(&self) -> bool {
        matches!(self.encoding, LaneEncoding::Iq { .. })
    }

    /// Amplitude assigned to angle-only samples (10-bit ADC units).
    pub const ANGLE_MAGNITUDE: f32 = 64.0;

    /// Decodes one word to a complex sample in 10-bit ADC units.
    #[inline]
    pub fn decode(&self, word: u32) -> Complex32 {
        match &self.encoding {
            LaneEncoding::Iq { i, q } => Complex32::new(i.decode(word), q.decode(word)),
            LaneEncoding::Angle { word_msb, bits } => {
                let n = *bits as u32;
                let lsb = *word_msb as u32 + 1 - n;
                let raw = (word >> lsb) & ((1 << n) - 1);
                let a = (raw as f32 + 0.5) * std::f32::consts::TAU / (1u32 << n) as f32;
                Complex32::from_polar(Self::ANGLE_MAGNITUDE, a)
            }
        }
    }

    /// Encodes 10-bit ADC values (or, for angle formats, the angle of the
    /// pair) into a word. Used by the simulator.
    #[inline]
    pub fn encode(&self, i_adc10: i32, q_adc10: i32) -> u32 {
        match &self.encoding {
            LaneEncoding::Iq { i, q } => i.encode(i_adc10) | q.encode(q_adc10),
            LaneEncoding::Angle { word_msb, bits } => {
                let n = *bits as u32;
                let lsb = *word_msb as u32 + 1 - n;
                let a = (q_adc10 as f32).atan2(i_adc10 as f32).rem_euclid(std::f32::consts::TAU);
                let code = ((a / std::f32::consts::TAU) * (1u32 << n) as f32) as u32 & ((1 << n) - 1);
                code << lsb
            }
        }
    }

    /// Full decode table indexed by word value (256 or 65536 entries).
    pub fn build_lut(&self) -> Vec<Complex32> {
        let n = 1usize << (8 * self.word_bytes as usize);
        (0..n as u32).map(|w| self.decode(w)).collect()
    }

    /// Word bits that carry data (others are tied low on the board).
    pub fn used_word_mask(&self) -> u32 {
        match &self.encoding {
            LaneEncoding::Iq { i, q } => i.word_mask() | q.word_mask(),
            LaneEncoding::Angle { word_msb, bits } => {
                ((1u32 << *bits) - 1) << (*word_msb as u32 + 1 - *bits as u32)
            }
        }
    }

    /// Largest decodable magnitude per component, 10-bit ADC units.
    pub fn component_full_scale(&self) -> f32 {
        match &self.encoding {
            LaneEncoding::Iq { i, .. } => {
                (1u32 << (i.adc_lsb() as u32 + i.width() - 1)) as f32
            }
            LaneEncoding::Angle { .. } => Self::ANGLE_MAGNITUDE,
        }
    }
}

/// Lane format plus sample clock.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SampleFormat {
    /// Preset name if the lanes match one (informational).
    pub preset: Option<LanePreset>,
    pub lanes: LaneFormat,
    /// PARLIO TX divider of the 240 MHz PLL that makes the sample clock.
    pub divider: u32,
}

impl SampleFormat {
    pub fn from_preset(preset: LanePreset, divider: u32) -> Self {
        SampleFormat {
            preset: Some(preset),
            lanes: preset.format(),
            divider,
        }
    }

    /// Sample (word) rate in Hz.
    #[inline]
    pub fn sample_rate_hz(&self) -> f64 {
        C5_PLL_HZ / self.divider as f64
    }

    /// Bytes per second on USB.
    #[inline]
    pub fn byte_rate(&self) -> f64 {
        self.sample_rate_hz() * self.lanes.word_bytes as f64
    }

    /// True when the clock keeps a whole number of bus samples per tick, so
    /// it never drifts against MODEM_DIAG.
    pub fn bus_locked(&self) -> bool {
        self.divider.is_multiple_of(3)
    }

    /// Bus samples per sample clock period.
    pub fn bus_decimation(&self) -> f64 {
        C5_BUS_HZ / self.sample_rate_hz()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_validate() {
        for p in LanePreset::ALL {
            p.format().validate().unwrap();
            let s: LanePreset = p.name().parse().unwrap();
            assert_eq!(s, p);
        }
        assert_eq!(LanePreset::I7Q7.data_pins(), 14);
        assert_eq!(LanePreset::I8Q8.data_pins(), 16);
        assert_eq!(LanePreset::I4Q4Ultra.data_pins(), 8);
    }

    #[test]
    fn roundtrip_contiguous() {
        let f = LanePreset::I8Q8.format();
        for (i, q) in [(0, 0), (-512, 511), (100, -37), (-3, 2), (255, -256)] {
            let z = f.decode(f.encode(i, q));
            assert!((z.re - i as f32).abs() <= 2.0, "{i} -> {}", z.re);
            assert!((z.im - q as f32).abs() <= 2.0, "{q} -> {}", z.im);
        }
        // 7x7: the GND-tied low lanes read as the 2v+1 centring.
        let f = LanePreset::I7Q7.format();
        let w = f.encode(8, -8);
        assert_eq!(w & 0x0101, 0);
        assert_eq!(f.decode(w), Complex32::new(12.0, -4.0));
    }

    #[test]
    fn ultrafine_folds() {
        let f = LanePreset::I4Q4Ultra.format();
        // Inside the +-128 window: 16-unit steps.
        assert_eq!(f.decode(f.encode(40, -40)), Complex32::new(40.0, -40.0));
        // Bits 8 and 7 are not carried: 168 folds by 128 onto 40.
        let z = f.decode(f.encode(168, 0));
        assert_eq!(z.re, 40.0);
        assert_eq!(f.component_full_scale(), 128.0);
    }

    #[test]
    fn lut_matches_decode() {
        let f = LanePreset::I4Q4Top.format();
        let lut = f.build_lut();
        assert_eq!(lut.len(), 256);
        for w in 0..256u32 {
            assert_eq!(lut[w as usize], f.decode(w));
        }
        // I on the high nibble, Q on the low nibble.
        let w = f.encode(-512, 0);
        assert_eq!(w & 0xf0, 0x80);
    }

    #[test]
    fn angle_roundtrip() {
        let f = LanePreset::Angle8.format();
        let w = f.encode(0, 100);
        let z = f.decode(w);
        assert!((z.arg() - std::f32::consts::FRAC_PI_2).abs() < 0.03);
    }

    #[test]
    fn rates() {
        let s = SampleFormat::from_preset(LanePreset::I8Q8, 18);
        assert!((s.sample_rate_hz() - 13.333_333e6).abs() < 1.0);
        assert!((s.byte_rate() - 26.666_666e6).abs() < 2.0);
        assert!(s.bus_locked());
        assert!((s.bus_decimation() - 6.0).abs() < 1e-9);
    }
}
