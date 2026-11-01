// SPDX-License-Identifier: GPL-3.0-or-later
//! Sampling-phase calibration (fourth-difference probe, slip, IFCLK
//! inversion) and the lane self-test analyser.

pub mod phase;
pub mod probe;
pub mod selftest;

pub use phase::{calibrate, CalibConfig, CalibIo, CalibReport, PositionResult};
pub use probe::{judge, probe_packet, ProbeConfig, ProbeStats, Verdict, PROBE_THRESHOLD};
pub use selftest::{analyze_walk, const_word, lanes_of, match_fraction, Issue, IssueKind, LaneInfo, WalkReport};
