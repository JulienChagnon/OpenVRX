// SPDX-License-Identifier: GPL-3.0-or-later
//! The live stage of one view: each complete field goes to the plugin, the
//! caller waits at most the deadline for the result and shows the plugin's
//! picture only when it came in time.
//!
//! ```text
//! field complete -> Feed (index, parity, flags, C/N) -> buffer copy
//!   worker idle -> mailbox, worker starts it -> wait <= deadline
//!                    in time: show the plugin's picture
//!                    not written (1): show the field
//!                    too late: show the field, count it late
//!   worker busy -> mailbox (replacing a waiting field: skipped)
//!                    show the field now, count it late
//! ```
//!
//! The caller is the view's video thread, which publishes whatever this
//! says to show; it is never held longer than the deadline, and a stuck
//! plugin only makes every field late.

use crate::field::Feed;
use crate::stats::TimingSummary;
use crate::worker::{Instance, Outcome, Phase, Submit};
use openvrx_core::Frame;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// A worker busy on one field this long is reported as stuck.
const STUCK_MS: f32 = 1000.0;

/// Default deadline (the app's `plugin_deadline_ms`).
pub const DEFAULT_DEADLINE_MS: f32 = 6.0;

/// The plugin of one view.
pub struct LiveFilter {
    instance: Instance,
    config: String,
    feed: Feed,
    deadline: Duration,
    /// The picture shown when the plugin's output came in time.
    out: Frame,
    fields: u64,
    shown: u64,
    late: u64,
    not_written: u64,
}

/// State of a view's plugin for the OSD and logs.
#[derive(Clone, Debug, PartialEq)]
pub enum ReportPhase {
    Loading,
    Ready,
    Failed(String),
}

/// A snapshot of a [`LiveFilter`] for the OSD.
#[derive(Clone, Debug, PartialEq)]
pub struct FilterReport {
    pub path: PathBuf,
    /// The plugin's name (the file name until it is ready).
    pub name: String,
    pub provider: Option<String>,
    pub phase: ReportPhase,
    pub timing: TimingSummary,
    /// Fields offered, shown filtered, late (deadline missed or worker
    /// busy), skipped in the mailbox, and left unwritten by the plugin.
    pub fields: u64,
    pub shown: u64,
    pub late: u64,
    pub skipped: u64,
    pub not_written: u64,
    /// Time the worker has been on its current field (0 when idle).
    pub busy_ms: f32,
    pub deadline_ms: f32,
}

impl FilterReport {
    /// A report for a plugin that cannot start at all (no path set...).
    pub fn failed(path: &Path, why: impl Into<String>) -> FilterReport {
        FilterReport {
            path: path.to_path_buf(),
            name: crate::library::short_name(path),
            provider: None,
            phase: ReportPhase::Failed(why.into()),
            timing: TimingSummary::default(),
            fields: 0,
            shown: 0,
            late: 0,
            skipped: 0,
            not_written: 0,
            busy_ms: 0.0,
            deadline_ms: 0.0,
        }
    }

    /// One OSD line while the plugin runs, e.g.
    /// `plugin cnn (cuda) 3.1 / 4.9 ms p50/p99, late 2 of 950`.
    pub fn osd_line(&self) -> String {
        let who = match &self.provider {
            Some(p) => format!("{} ({p})", self.name),
            None => self.name.clone(),
        };
        match &self.phase {
            ReportPhase::Loading => format!("plugin {who} loading"),
            ReportPhase::Failed(_) => format!("plugin {who} off"),
            ReportPhase::Ready if self.timing.count == 0 => format!("plugin {who} ready"),
            ReportPhase::Ready => format!(
                "plugin {who} {:.1} / {:.1} ms p50/p99, late {} of {}{}",
                self.timing.p50_ms,
                self.timing.p99_ms,
                self.late,
                self.fields,
                if self.skipped > 0 {
                    format!(", {} skipped", self.skipped)
                } else {
                    String::new()
                }
            ),
        }
    }

    /// A warning when the plugin is not filtering (or stuck), else None.
    pub fn warning(&self) -> Option<String> {
        match &self.phase {
            ReportPhase::Failed(why) => Some(format!("plugin off ({why}): plain picture")),
            ReportPhase::Ready if self.busy_ms > STUCK_MS => Some(format!(
                "plugin busy on one field for {:.1} s: plain picture",
                self.busy_ms / 1e3
            )),
            _ => None,
        }
    }
}

impl LiveFilter {
    /// Starts the plugin at `path` with `config` (loading runs on the
    /// library's worker; fields are shown plain until it is ready).
    pub fn start(path: &Path, config: &str, deadline_ms: f32) -> LiveFilter {
        LiveFilter {
            instance: Instance::start(path, config),
            config: config.to_string(),
            feed: Feed::new(),
            deadline: deadline(deadline_ms),
            out: Frame::new(1, 1),
            fields: 0,
            shown: 0,
            late: 0,
            not_written: 0,
        }
    }

    /// True when this runs the plugin at `path` with `config`.
    pub fn is(&self, path: &Path, config: &str) -> bool {
        self.instance.path() == path && self.config == config
    }

    pub fn set_deadline_ms(&mut self, ms: f32) {
        self.deadline = deadline(ms);
    }

    pub fn phase(&self) -> Phase {
        self.instance.phase()
    }

    /// The plugin's info once it is ready.
    pub fn info(&self) -> Option<std::sync::Arc<crate::PluginInfo>> {
        self.instance.info()
    }

    /// Waits at most `timeout` for loading to end (tests, tools).
    pub fn wait_ready(&self, timeout: Duration) -> Phase {
        self.instance.wait_ready(timeout)
    }

    /// The next field starts a new history (pipeline restart, retune...).
    pub fn mark_break(&mut self) {
        self.feed.mark_break();
    }

    /// One complete field (`frame` as the view shows it, `steady` when
    /// line-doubled): returns the plugin's picture when it came within the
    /// deadline, None to show `frame` itself.
    pub fn field(&mut self, frame: &Frame, steady: bool) -> Option<&Frame> {
        let info = self.instance.info()?;
        let meta = self.feed.next(frame, steady);
        let mut job = self.instance.buffer();
        job.fill(frame, meta, &info.aux);
        self.fields += 1;
        match self.instance.submit(job) {
            Submit::Started => match self.instance.wait(meta.index, Instant::now() + self.deadline) {
                Some((mut job, Outcome::Written)) => {
                    job.take_output(frame, &mut self.out);
                    self.instance.recycle(job);
                    self.shown += 1;
                    Some(&self.out)
                }
                Some((job, outcome)) => {
                    if outcome == Outcome::NotWritten {
                        self.not_written += 1;
                    }
                    self.instance.recycle(job);
                    None
                }
                None => {
                    self.late += 1;
                    None
                }
            },
            Submit::Queued { .. } => {
                self.late += 1;
                None
            }
            Submit::NotReady => None,
        }
    }

    pub fn report(&self) -> FilterReport {
        let st = self.instance.stats();
        let (name, provider, phase) = match self.instance.phase() {
            Phase::Ready(i) => (i.name.clone(), i.provider.clone(), ReportPhase::Ready),
            Phase::Loading => (crate::library::short_name(self.instance.path()), None, ReportPhase::Loading),
            Phase::Failed(e) => (crate::library::short_name(self.instance.path()), None, ReportPhase::Failed(e)),
            Phase::Closed => (
                crate::library::short_name(self.instance.path()),
                None,
                ReportPhase::Failed("closed".into()),
            ),
        };
        FilterReport {
            path: self.instance.path().to_path_buf(),
            name,
            provider,
            phase,
            timing: st.timing,
            fields: self.fields,
            shown: self.shown,
            late: self.late,
            skipped: st.skipped,
            not_written: self.not_written,
            busy_ms: st.busy_ms,
            deadline_ms: self.deadline.as_secs_f32() * 1e3,
        }
    }
}

fn deadline(ms: f32) -> Duration {
    let ms = if ms.is_finite() { ms.clamp(0.0, 1000.0) } else { DEFAULT_DEADLINE_MS };
    Duration::from_secs_f64(ms as f64 / 1e3)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib_path() -> PathBuf {
        let c = std::env::consts::DLL_PREFIX;
        PathBuf::from(format!("/x/{c}cnn{}", std::env::consts::DLL_SUFFIX))
    }

    fn report(phase: ReportPhase) -> FilterReport {
        FilterReport {
            phase,
            timing: TimingSummary {
                count: 10,
                p50_ms: 3.14,
                p99_ms: 4.9,
                max_ms: 5.0,
                mean_ms: 3.0,
            },
            fields: 950,
            late: 2,
            provider: Some("cuda".into()),
            name: "cnn".into(),
            ..FilterReport::failed(&lib_path(), "")
        }
    }

    #[test]
    fn osd_texts() {
        assert_eq!(
            report(ReportPhase::Ready).osd_line(),
            "plugin cnn (cuda) 3.1 / 4.9 ms p50/p99, late 2 of 950"
        );
        assert!(report(ReportPhase::Ready).warning().is_none());
        let f = report(ReportPhase::Failed("plugin error -2 in process".into()));
        assert_eq!(f.osd_line(), "plugin cnn (cuda) off");
        assert!(f.warning().unwrap().contains("error -2"));
        let mut stuck = report(ReportPhase::Ready);
        stuck.busy_ms = 2500.0;
        assert!(stuck.warning().unwrap().contains("2.5 s"));
        assert_eq!(FilterReport::failed(&lib_path(), "none").name, "cnn");
    }

    #[test]
    fn deadline_is_bounded() {
        assert_eq!(deadline(6.0), Duration::from_micros(6000));
        assert_eq!(deadline(f32::NAN), Duration::from_micros(6000));
        assert_eq!(deadline(-3.0), Duration::ZERO);
        assert_eq!(deadline(5e6), Duration::from_secs(1));
    }
}
