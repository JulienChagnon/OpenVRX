// SPDX-License-Identifier: GPL-3.0-or-later
//! Messages between the app and its backend.
//!
//! The app never calls into the backend directly: it sends
//! [`AppCommand`]s over a channel and receives [`BackendEvent`]s plus a
//! shared [`BackendStatus`] snapshot. The PC owns all settings, so on
//! start (and whenever something changes) the app pushes the full state
//! for each receiver; a backend can treat every command as idempotent.

use crate::settings::{AppSettings, ReceiverSettings, VideoSettings, ViewSource};
use crate::source::FrameSource;
use openvrx_core::RxId;
use std::path::PathBuf;

/// App to backend.
#[derive(Clone, Debug, PartialEq)]
pub enum AppCommand {
    /// Start feeding view slot `view`; answered by `ViewOpened` or
    /// `ViewFailed`. Replaces any previous source of that slot.
    OpenView { view: usize, source: ViewSource },
    CloseView { view: usize },
    /// Full state of one receiver (tune, gain, lanes, divider, enable).
    ConfigureReceiver { rx: RxId, config: ReceiverSettings },
    /// Run the sampling-phase probe and slip until clean.
    Calibrate { rx: RxId },
    /// Video pipeline configuration (preset, cleanup master switch).
    SetVideo(VideoSettings),
    StartRecording { folder: PathBuf },
    StopRecording,
    Shutdown,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MessageLevel {
    Info,
    Warning,
    Error,
}

/// Backend to app.
pub enum BackendEvent {
    ViewOpened {
        view: usize,
        source: Box<dyn FrameSource>,
    },
    ViewFailed {
        view: usize,
        error: String,
    },
    Message {
        level: MessageLevel,
        text: String,
    },
}

impl std::fmt::Debug for BackendEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BackendEvent::ViewOpened { view, source } => f
                .debug_struct("ViewOpened")
                .field("view", view)
                .field("source", &source.label())
                .finish(),
            BackendEvent::ViewFailed { view, error } => f
                .debug_struct("ViewFailed")
                .field("view", view)
                .field("error", error)
                .finish(),
            BackendEvent::Message { level, text } => write!(f, "Message({level:?}, {text})"),
        }
    }
}

/// Recorder state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RecordingStatus {
    /// The backend can record at all.
    pub available: bool,
    pub active: bool,
    /// Host time recording started (clock::now_ns).
    pub started_ns: u64,
    pub bytes: u64,
    pub path: Option<PathBuf>,
}

/// Global backend snapshot for the OSD and panels.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BackendStatus {
    pub name: String,
    /// Real hardware (false for the test backend).
    pub live: bool,
    pub recording: RecordingStatus,
    /// One line per receiver found ("rx0: COM7 + FX2 1a2b3c4d").
    pub devices: Vec<String>,
    pub warnings: Vec<String>,
}

/// Commands that bring a backend to the state described by `settings`
/// for the first `views` view slots.
pub fn full_state(settings: &AppSettings, views: usize) -> Vec<AppCommand> {
    let mut out = Vec::new();
    for (k, r) in settings.receivers.iter().enumerate() {
        out.push(AppCommand::ConfigureReceiver {
            rx: k as RxId,
            config: r.clone(),
        });
    }
    out.push(AppCommand::SetVideo(settings.video.clone()));
    for (v, view) in settings.views.iter().enumerate().take(views) {
        out.push(AppCommand::OpenView {
            view: v,
            source: view.source.clone(),
        });
    }
    out
}

/// Commands needed to go from `old` to `new` settings (view slots beyond
/// `views` are closed).
pub fn diff(
    old: &AppSettings,
    new: &AppSettings,
    old_views: usize,
    views: usize,
) -> Vec<AppCommand> {
    let mut out = Vec::new();
    for (k, r) in new.receivers.iter().enumerate() {
        if old.receivers.get(k) != Some(r) {
            out.push(AppCommand::ConfigureReceiver {
                rx: k as RxId,
                config: r.clone(),
            });
        }
    }
    if old.video != new.video {
        out.push(AppCommand::SetVideo(new.video.clone()));
    }
    for v in 0..views.max(old_views) {
        let was_open = v < old_views;
        let is_open = v < views;
        let changed = old.views.get(v) != new.views.get(v);
        if is_open && (!was_open || changed) {
            out.push(AppCommand::OpenView {
                view: v,
                source: new.views[v].source.clone(),
            });
        } else if was_open && !is_open {
            out.push(AppCommand::CloseView { view: v });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::{GainMode, TestPattern, VideoPreset};
    use openvrx_core::VideoStandard;

    #[test]
    fn full_state_covers_everything() {
        let s = AppSettings::default();
        let cmds = full_state(&s, 2);
        let configure = cmds
            .iter()
            .filter(|c| matches!(c, AppCommand::ConfigureReceiver { .. }))
            .count();
        assert_eq!(configure, s.receivers.len());
        let opens = cmds
            .iter()
            .filter(|c| matches!(c, AppCommand::OpenView { .. }))
            .count();
        assert_eq!(opens, 2);
        assert!(cmds.iter().any(|c| matches!(c, AppCommand::SetVideo(_))));
    }

    #[test]
    fn diff_is_minimal() {
        let a = AppSettings::default();
        assert!(diff(&a, &a, 1, 1).is_empty());

        let mut b = a.clone();
        b.receivers[1].freq_mhz = 5917;
        b.receivers[1].gain = GainMode::Manual { index: 70 };
        assert_eq!(
            diff(&a, &b, 1, 1),
            vec![AppCommand::ConfigureReceiver {
                rx: 1,
                config: b.receivers[1].clone()
            }]
        );

        let mut c = a.clone();
        c.video.preset = VideoPreset::Quality;
        assert!(matches!(diff(&a, &c, 1, 1)[..], [AppCommand::SetVideo(_)]));
    }

    #[test]
    fn diff_views() {
        let a = AppSettings::default();
        let opened: Vec<usize> = diff(&a, &a, 1, 4)
            .iter()
            .filter_map(|c| match c {
                AppCommand::OpenView { view, .. } => Some(*view),
                _ => None,
            })
            .collect();
        assert_eq!(opened, vec![1, 2, 3]);
        let closed = diff(&a, &a, 4, 1)
            .iter()
            .filter(|c| matches!(c, AppCommand::CloseView { .. }))
            .count();
        assert_eq!(closed, 3);

        let mut b = a.clone();
        b.views[0].source = ViewSource::Test {
            pattern: TestPattern::Sweep,
            standard: VideoStandard::Pal,
        };
        assert!(matches!(
            diff(&a, &b, 1, 1)[..],
            [AppCommand::OpenView { view: 0, .. }]
        ));
    }
}
