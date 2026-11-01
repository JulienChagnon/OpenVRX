// SPDX-License-Identifier: GPL-3.0-or-later
//! Sampling-phase calibration (DESIGN 5.4, PROTOCOL.md 3).
//!
//! The sample clock edge has three positions against the 80 MS/s
//! MODEM_DIAG bus and at most one of them lands while the lanes change.
//! The procedure: ask the C5 for `clk 6` (40 MS/s; the FX2 FIFO overflows
//! but every 512-byte packet is contiguous), capture packets, score each
//! with the fourth-difference probe, decide with robust statistics, `slip`
//! and retry while the verdict is mixed, optionally repeat with IFCLK
//! inverted, then restore the working divider (the phase position survives
//! divider changes).

use crate::calib::probe::{judge, probe_packet, ProbeConfig, ProbeStats, Verdict};
use crate::error::{HwError, Result};
use crate::fx2::protocol::PACKET_BYTES;
use openvrx_core::LaneFormat;
use serde::{Deserialize, Serialize};

/// What calibration needs from the hardware (implemented by `Receiver`
/// over a C5 client and an FX2, and by simulations in tests).
pub trait CalibIo {
    /// Sets the C5 sample clock divider without changing the remembered
    /// working divider.
    fn set_divider(&mut self, divider: u32) -> Result<()>;
    /// Moves the clock edge once; returns the C5's slip total.
    fn slip(&mut self) -> Result<u32>;
    /// Captures `packets` whole 512-byte packets with the FX2 latching on
    /// the normal or inverted IFCLK edge.
    fn capture_packets(&mut self, ifclk_inv: bool, packets: usize) -> Result<Vec<Vec<u8>>>;
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibConfig {
    /// Probe divider (6 = 40 MS/s).
    pub probe_divider: u32,
    /// Packets per measurement.
    pub packets: usize,
    /// Slips to try per IFCLK polarity (3 positions exist).
    pub max_slips: u32,
    /// Try the inverted IFCLK edge when no position is clean.
    pub try_ifclk_inversion: bool,
    /// Measure all three positions before choosing (reports margins).
    pub survey: bool,
    /// Re-measurements of an undecided or marginal position.
    pub retries: u32,
    pub probe: ProbeConfig,
}

impl Default for CalibConfig {
    fn default() -> Self {
        CalibConfig {
            probe_divider: 6,
            packets: 64,
            max_slips: 4,
            try_ifclk_inversion: true,
            survey: false,
            retries: 2,
            probe: ProbeConfig::default(),
        }
    }
}

/// One measured position.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PositionResult {
    /// Slips applied since calibration began when this was measured.
    pub slips: u32,
    pub ifclk_inv: bool,
    pub stats: ProbeStats,
}

/// Full calibration report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibReport {
    pub success: bool,
    /// The position left in place.
    pub chosen: Option<PositionResult>,
    /// Use this IFCLK polarity for streaming.
    pub ifclk_inverted: bool,
    pub slips_applied: u32,
    /// The C5's slip total at the end.
    pub c5_slip_total: Option<u32>,
    pub positions: Vec<PositionResult>,
    pub working_divider: u32,
    pub restored: bool,
    /// Plain-language summary.
    pub message: String,
}

fn measure(
    io: &mut dyn CalibIo,
    lanes: &LaneFormat,
    inv: bool,
    cfg: &CalibConfig,
    slips: u32,
) -> Result<PositionResult> {
    let mut last = None;
    for _ in 0..=cfg.retries {
        let packets = io.capture_packets(inv, cfg.packets)?;
        let scores: Vec<Option<f32>> = packets
            .iter()
            .filter(|p| p.len() == PACKET_BYTES)
            .map(|p| probe_packet(p, lanes))
            .collect();
        let stats = judge(&scores, &cfg.probe);
        let decided = matches!(stats.verdict, Verdict::Clean | Verdict::Mixed);
        last = Some(PositionResult {
            slips,
            ifclk_inv: inv,
            stats,
        });
        if decided {
            break;
        }
    }
    Ok(last.expect("at least one measurement"))
}

/// Runs the calibration and always tries to restore `working_divider`.
pub fn calibrate(
    io: &mut dyn CalibIo,
    lanes: &LaneFormat,
    working_divider: u32,
    cfg: &CalibConfig,
) -> Result<CalibReport> {
    io.set_divider(cfg.probe_divider)?;
    let result = run(io, lanes, cfg);
    let restored = io.set_divider(working_divider);
    let mut report = result?;
    report.working_divider = working_divider;
    report.restored = restored.is_ok();
    if let Err(e) = restored {
        report
            .message
            .push_str(&format!(" Restoring clk {working_divider} failed: {e}."));
    }
    Ok(report)
}

fn run(io: &mut dyn CalibIo, lanes: &LaneFormat, cfg: &CalibConfig) -> Result<CalibReport> {
    let mut positions = Vec::new();
    let mut slips = 0u32;
    let mut c5_total = None;
    let polarities: &[bool] = if cfg.try_ifclk_inversion {
        &[false, true]
    } else {
        &[false]
    };
    for &inv in polarities {
        let mut seen_clean = false;
        for step in 0..=cfg.max_slips.max(2) {
            let r = measure(io, lanes, inv, cfg, slips)?;
            let clean = r.stats.verdict == Verdict::Clean;
            positions.push(r.clone());
            seen_clean |= clean;
            // Without survey, stop at the first clean position. With
            // survey, look at three positions first, then stop at the next
            // clean one.
            if clean && (!cfg.survey || step >= 2) {
                let message = format!(
                    "phase clean after {slips} slip(s){}: median score {:.2}, {:.0}% mixed packets",
                    if inv { " with IFCLK inverted" } else { "" },
                    r.stats.median,
                    100.0 * r.stats.mixed_fraction
                );
                return Ok(CalibReport {
                    success: true,
                    chosen: Some(r),
                    ifclk_inverted: inv,
                    slips_applied: slips,
                    c5_slip_total: c5_total,
                    positions,
                    working_divider: 0,
                    restored: false,
                    message,
                });
            }
            if step == cfg.max_slips.max(2) || (step >= 2 && !seen_clean && !cfg.survey) {
                break;
            }
            c5_total = Some(io.slip()?);
            slips += 1;
        }
    }
    let all_undecided = positions.iter().all(|p| p.stats.verdict == Verdict::Undecided);
    let message = if all_undecided {
        "no position could be judged: the capture has no signal above the quantisation floor \
         (check that the sample clock reaches the FX2, that the lanes are wired, and raise the gain)"
            .to_string()
    } else {
        let best = positions.iter().filter(|p| p.stats.median.is_finite()).min_by(|a, b| {
            a.stats
                .median
                .partial_cmp(&b.stats.median)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        format!(
            "no clean sampling position found in {} measurement(s){}; best median score {:.1}. \
             Lower the lane drive strength or add 22-33 ohm series resistors at the C5, keep the clock \
             wire short with a ground beside it, then retry.",
            positions.len(),
            if cfg.try_ifclk_inversion {
                " with both IFCLK edges"
            } else {
                ""
            },
            best.map_or(f32::NAN, |b| b.stats.median)
        )
    };
    if positions.is_empty() {
        return Err(HwError::Invalid("calibration made no measurement".into()));
    }
    Ok(CalibReport {
        success: false,
        chosen: None,
        ifclk_inverted: false,
        slips_applied: slips,
        c5_slip_total: c5_total,
        positions,
        working_divider: 0,
        restored: false,
        message,
    })
}

#[cfg(test)]
pub(crate) mod sim {
    //! A simulated C5 + FX2 pair for calibration tests.
    use super::*;
    use crate::calib::probe::testsig::{mix, signal, to_bytes, Rng};

    pub struct SimPhase {
        pub lanes: LaneFormat,
        pub position: u32,
        /// Positions (mod 3) that produce mixed reads, per IFCLK polarity.
        pub bad_normal: Vec<u32>,
        pub bad_inverted: Vec<u32>,
        pub late_mask: u32,
        pub divider: u32,
        pub slips: u32,
        pub amp: f32,
        pub noise: f32,
        pub rng: Rng,
        pub captures: u32,
    }

    impl SimPhase {
        pub fn new(lanes: LaneFormat) -> SimPhase {
            SimPhase {
                lanes,
                position: 0,
                bad_normal: vec![],
                bad_inverted: vec![],
                late_mask: 0x8000 | 0x4000 | 0x0080,
                divider: 18,
                slips: 0,
                amp: 150.0,
                noise: 8.0,
                rng: Rng(0x1234_5678),
                captures: 0,
            }
        }
    }

    impl CalibIo for SimPhase {
        fn set_divider(&mut self, divider: u32) -> Result<()> {
            self.divider = divider;
            Ok(())
        }
        fn slip(&mut self) -> Result<u32> {
            self.position = (self.position + 1) % 3;
            self.slips += 1;
            Ok(self.slips)
        }
        fn capture_packets(&mut self, inv: bool, packets: usize) -> Result<Vec<Vec<u8>>> {
            assert_eq!(self.divider, 6, "calibration must probe at 40 MS/s");
            self.captures += 1;
            let wpp = PACKET_BYTES / self.lanes.word_bytes as usize;
            let bad = if inv { &self.bad_inverted } else { &self.bad_normal };
            let is_bad = bad.contains(&self.position);
            Ok((0..packets)
                .map(|_| {
                    let s = signal(wpp, self.amp, self.noise, &mut self.rng);
                    let w: Vec<u32> = s.iter().map(|&(i, q)| self.lanes.encode(i, q)).collect();
                    let w = if is_bad { mix(&w, self.late_mask) } else { w };
                    to_bytes(&w, self.lanes.word_bytes)
                })
                .collect())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::SimPhase;
    use super::*;
    use openvrx_core::LanePreset;

    #[test]
    fn clean_at_start_needs_no_slip() {
        let mut io = SimPhase::new(LanePreset::I7Q7.format());
        let r = calibrate(&mut io, &LanePreset::I7Q7.format(), 18, &CalibConfig::default()).unwrap();
        assert!(r.success, "{}", r.message);
        assert_eq!(r.slips_applied, 0);
        assert!(!r.ifclk_inverted);
        assert!(r.restored);
        assert_eq!(io.divider, 18);
    }

    #[test]
    fn slips_away_from_the_bad_position() {
        let f = LanePreset::I8Q8.format();
        let mut io = SimPhase::new(f.clone());
        io.bad_normal = vec![0];
        let r = calibrate(&mut io, &f, 12, &CalibConfig::default()).unwrap();
        assert!(r.success, "{}", r.message);
        assert_eq!(r.slips_applied, 1);
        assert_eq!(r.positions[0].stats.verdict, Verdict::Mixed);
        assert_eq!(r.positions[1].stats.verdict, Verdict::Clean);
        assert_eq!(io.divider, 12);
        assert_eq!(r.c5_slip_total, Some(1));
    }

    #[test]
    fn falls_back_to_inverted_ifclk() {
        let f = LanePreset::I7Q7.format();
        let mut io = SimPhase::new(f.clone());
        io.bad_normal = vec![0, 1, 2];
        io.bad_inverted = vec![0];
        let r = calibrate(&mut io, &f, 18, &CalibConfig::default()).unwrap();
        assert!(r.success, "{}", r.message);
        assert!(r.ifclk_inverted);
        assert!(r.message.contains("IFCLK inverted"));
        let chosen = r.chosen.unwrap();
        assert!(chosen.ifclk_inv);
        assert_eq!(io.divider, 18);
    }

    #[test]
    fn reports_failure_in_plain_language() {
        let f = LanePreset::I7Q7.format();
        let mut io = SimPhase::new(f.clone());
        io.bad_normal = vec![0, 1, 2];
        io.bad_inverted = vec![0, 1, 2];
        let r = calibrate(&mut io, &f, 18, &CalibConfig::default()).unwrap();
        assert!(!r.success);
        assert!(r.message.contains("series resistors"), "{}", r.message);
        assert!(r.restored);
        assert_eq!(io.divider, 18);
        // No signal at all: undecided everywhere.
        let mut io = SimPhase::new(f.clone());
        io.amp = 0.0;
        io.noise = 0.0;
        let r = calibrate(
            &mut io,
            &f,
            18,
            &CalibConfig {
                try_ifclk_inversion: false,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(!r.success);
        assert!(r.message.contains("no signal"), "{}", r.message);
    }

    #[test]
    fn survey_measures_three_positions() {
        let f = LanePreset::I8Q8.format();
        let mut io = SimPhase::new(f.clone());
        io.bad_normal = vec![1];
        let r = calibrate(
            &mut io,
            &f,
            18,
            &CalibConfig {
                survey: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(r.success);
        assert!(r.positions.len() >= 3);
        assert_eq!(r.positions[1].stats.verdict, Verdict::Mixed);
        // Ends on a clean position (position 2 after two slips).
        assert_eq!(io.position, 2);
    }
}
