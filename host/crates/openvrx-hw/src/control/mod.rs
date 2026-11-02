// SPDX-License-Identifier: GPL-3.0-or-later
//! Control logic without hardware: gain loop, AFC retune decisions,
//! channel table and scanner, watch-mode promotion.

pub mod afc;
pub mod gain;
pub mod scan;
pub mod watch;

pub use afc::{Afc, AfcConfig, Retune};
pub use gain::{GainChange, GainConfig, GainLoop, GainReason, GainTable};
pub use scan::{all_channels, channel, run_scan, Band, Channel, ChannelMetrics, ScanConfig, ScanIo, ScanReport};
pub use watch::{PromoteConfig, Switch, WatchManager};
