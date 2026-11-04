// SPDX-License-Identifier: GPL-3.0-or-later
//! 5.8 GHz analog FPV band/channel table.
//!
//! Raceband first: when a frequency appears in several bands (5880 MHz is
//! both R7 and F8) the name from the earlier band is shown.

/// One band of eight channels.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Band {
    pub letter: char,
    pub name: &'static str,
    pub mhz: [u32; 8],
}

/// Common bands in name-preference order.
pub const BANDS: [Band; 6] = [
    Band {
        letter: 'R',
        name: "Raceband",
        mhz: [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917],
    },
    Band {
        letter: 'F',
        name: "Fatshark / IRC",
        mhz: [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],
    },
    Band {
        letter: 'A',
        name: "Boscam A",
        mhz: [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725],
    },
    Band {
        letter: 'B',
        name: "Boscam B",
        mhz: [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],
    },
    Band {
        letter: 'E',
        name: "Boscam E",
        mhz: [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945],
    },
    Band {
        letter: 'L',
        name: "Lowband",
        mhz: [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621],
    },
];

/// Lowest frequency accepted for raw tuning.
pub const MIN_MHZ: u32 = 5000;
/// Highest frequency accepted for raw tuning.
pub const MAX_MHZ: u32 = 6000;

/// Looks up a band letter (case-insensitive) and 1-based channel.
pub fn lookup(letter: char, channel: u8) -> Option<u32> {
    let band = band(letter)?;
    let k = (channel as usize).checked_sub(1)?;
    band.mhz.get(k).copied()
}

/// Band by letter (case-insensitive).
pub fn band(letter: char) -> Option<&'static Band> {
    let l = letter.to_ascii_uppercase();
    BANDS.iter().find(|b| b.letter == l)
}

/// Band letter and 1-based channel of an exact frequency, preferring the
/// earlier band in [`BANDS`].
pub fn name_for(mhz: u32) -> Option<(char, u8)> {
    BANDS.iter().find_map(|b| {
        b.mhz
            .iter()
            .position(|&f| f == mhz)
            .map(|k| (b.letter, k as u8 + 1))
    })
}

/// "R1 5658" for table channels, "5700 MHz" otherwise.
pub fn label(mhz: u32) -> String {
    match name_for(mhz) {
        Some((l, c)) => format!("{l}{c} {mhz}"),
        None => format!("{mhz} MHz"),
    }
}

/// Clamps a raw frequency to the tunable range.
pub fn clamp_mhz(mhz: u32) -> u32 {
    mhz.clamp(MIN_MHZ, MAX_MHZ)
}

/// The next table channel above (`up`) or below the given frequency, in
/// frequency order across all bands (duplicates collapse).
pub fn step(mhz: u32, up: bool) -> u32 {
    let mut best: Option<u32> = None;
    for b in &BANDS {
        for &f in &b.mhz {
            let better = match (up, best) {
                (true, None) => f > mhz,
                (true, Some(x)) => f > mhz && f < x,
                (false, None) => f < mhz,
                (false, Some(x)) => f < mhz && f > x,
            };
            if better {
                best = Some(f);
            }
        }
    }
    best.unwrap_or(mhz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookups() {
        assert_eq!(lookup('r', 1), Some(5658));
        assert_eq!(lookup('R', 8), Some(5917));
        assert_eq!(lookup('F', 4), Some(5800));
        assert_eq!(lookup('R', 0), None);
        assert_eq!(lookup('R', 9), None);
        assert_eq!(lookup('Z', 1), None);
    }

    #[test]
    fn names_prefer_raceband() {
        assert_eq!(name_for(5880), Some(('R', 7)));
        assert_eq!(name_for(5740), Some(('F', 1)));
        assert_eq!(name_for(5700), None);
        assert_eq!(label(5658), "R1 5658");
        assert_eq!(label(5700), "5700 MHz");
    }

    #[test]
    fn every_band_has_distinct_channels() {
        for b in &BANDS {
            for (i, f) in b.mhz.iter().enumerate() {
                assert!((MIN_MHZ..=MAX_MHZ).contains(f));
                assert!(!b.mhz[i + 1..].contains(f), "{} repeats {f}", b.letter);
            }
        }
    }

    #[test]
    fn stepping() {
        assert_eq!(step(5658, true), 5665);
        assert_eq!(step(5665, false), 5658);
        assert_eq!(step(5945, true), 5945);
        assert_eq!(step(5362, false), 5362);
        assert_eq!(step(5700, true), 5705);
        assert_eq!(clamp_mhz(7000), MAX_MHZ);
    }
}
