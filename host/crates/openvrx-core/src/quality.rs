//! Per-line quality and per-sample damage conventions.
//!
//! Every line record carries a [`LineQuality`] and a damage mask with one
//! byte per grid sample. Diversity combining, dropout repair, temporal
//! filters and the OSD all read these; nothing downstream guesses damage
//! from pixels when the demodulator already knows.

use serde::{Deserialize, Serialize};

/// Damage mask values (one byte per grid sample). Anything in between is a
/// graded severity; consumers compare with `>=`.
pub mod damage {
    /// Clean.
    pub const NONE: u8 = 0;
    /// Raised noise (low envelope, near threshold).
    pub const NOISY: u8 = 64;
    /// FM click / phase wrap that was repaired by the demodulator.
    pub const REPAIRED: u8 = 96;
    /// FM click / impulse not repaired.
    pub const CLICK: u8 = 160;
    /// Carrier gone, sample is noise.
    pub const SEVERE: u8 = 224;
    /// No data (USB loss, sync flywheel gap).
    pub const LOST: u8 = 255;
}

/// Line flags (bit set).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct LineFlags(pub u32);

impl LineFlags {
    /// No sync pulse was found; position predicted by the line flywheel.
    pub const FLYWHEEL: LineFlags = LineFlags(1 << 0);
    /// I/Q reached the lane format's full scale.
    pub const CLIPPED: LineFlags = LineFlags(1 << 1);
    /// Envelope says there was no carrier for most of the line.
    pub const NO_CARRIER: LineFlags = LineFlags(1 << 2);
    /// A gain change landed inside this line.
    pub const GAIN_STEP: LineFlags = LineFlags(1 << 3);
    /// Input samples were lost (USB overflow or gap) inside this line.
    pub const SAMPLES_LOST: LineFlags = LineFlags(1 << 4);
    /// Line content was replaced or patched by repair or diversity.
    pub const REPAIRED: LineFlags = LineFlags(1 << 5);
    /// Line is part of the vertical interval (no picture).
    pub const VBI: LineFlags = LineFlags(1 << 6);
    /// Colour burst found and locked on this line.
    pub const BURST_OK: LineFlags = LineFlags(1 << 7);
    /// Line is a blend of several receivers.
    pub const BLENDED: LineFlags = LineFlags(1 << 8);
    /// Sync timing jumped (retrace/glitch) and the TBC re-locked.
    pub const RELOCK: LineFlags = LineFlags(1 << 9);

    #[inline]
    pub fn contains(self, other: LineFlags) -> bool {
        self.0 & other.0 == other.0
    }
    #[inline]
    pub fn insert(&mut self, other: LineFlags) {
        self.0 |= other.0;
    }
    #[inline]
    pub fn remove(&mut self, other: LineFlags) {
        self.0 &= !other.0;
    }
}

impl std::ops::BitOr for LineFlags {
    type Output = LineFlags;
    fn bitor(self, rhs: LineFlags) -> LineFlags {
        LineFlags(self.0 | rhs.0)
    }
}

/// Quality metrics for one line from one receiver (or the combined line).
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LineQuality {
    /// Envelope carrier-to-noise estimate in dB (C/N per channel bandwidth
    /// of the capture). NaN for angle-only lane formats.
    pub cn_db: f32,
    /// RMS noise in IRE measured on flat regions (sync tip, back porch).
    pub noise_ire: f32,
    /// 0 (no sync) .. 1 (clean, full-depth, sharp sync).
    pub sync_score: f32,
    /// Detected FM clicks (repaired or not) in the line.
    pub clicks: u16,
    /// Distance of the measured line start from the flywheel prediction,
    /// in grid samples.
    pub timing_err: f32,
    /// Mean raw I/Q power, dB relative to 1 ADC unit squared.
    pub power_db: f32,
    /// Combining weight (inverse noise variance after detection), >= 0.
    pub weight: f32,
    pub flags: LineFlags,
}

impl LineQuality {
    /// A line with no information at all.
    pub const UNKNOWN: LineQuality = LineQuality {
        cn_db: f32::NAN,
        noise_ire: f32::INFINITY,
        sync_score: 0.0,
        clicks: 0,
        timing_err: f32::INFINITY,
        power_db: f32::NEG_INFINITY,
        weight: 0.0,
        flags: LineFlags(0),
    };

    /// Weight from a noise estimate: inverse variance, floored so a perfect
    /// line never gets infinite weight.
    #[inline]
    pub fn weight_from_noise(noise_ire: f32) -> f32 {
        let n = noise_ire.max(0.05);
        1.0 / (n * n)
    }
}

impl Default for LineQuality {
    fn default() -> Self {
        LineQuality::UNKNOWN
    }
}
