// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX video filter plugin host: the app's `plugin` video preset and
//! `openvrx-cli decode --filter-plugin` load plugin libraries
//! ([`openvrx_filter_abi`], the contract) and feed them every displayed
//! field.
//!
//! ```text
//! video thread: field complete -> LiveFilter (live)  -> Instance mailbox -> worker
//!               show the plugin's picture if it came within the deadline,
//!               else the field itself
//! worker thread (one per library): Library::open, Plugin::create / process / destroy
//! offline (CLI): Plugin on the calling thread, Feed and FieldBuffers directly
//! ```
//!
//! - [`library`]: loading and the ABI checks; libraries stay loaded.
//! - [`plugin`]: one plugin instance and its checked info JSON.
//! - [`field`]: host-owned field buffers with side planes, and the feed
//!   that numbers fields and flags breaks in their history.
//! - [`worker`]: one worker thread per library, instances with a one-field
//!   mailbox.
//! - [`live`]: the live stage: deadline, late fields, the OSD report.
//! - [`stats`]: per-field timing percentiles.

pub mod field;
#[cfg(feature = "fixture")]
pub mod fixture;
pub mod library;
pub mod live;
pub mod plugin;
pub mod stats;
pub mod worker;

pub use field::{Feed, FieldBuffers, FieldMeta};
pub use library::{register_static, Library};
pub use live::{FilterReport, LiveFilter, ReportPhase, DEFAULT_DEADLINE_MS};
pub use openvrx_filter_abi as abi;
pub use plugin::{Geometry, Plugin, PluginInfo, PIPELINES};
pub use stats::{Timing, TimingSummary};
pub use worker::{Instance, InstanceStats, Outcome, Phase, Submit};
