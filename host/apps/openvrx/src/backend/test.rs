// SPDX-License-Identifier: GPL-3.0-or-later
//! Built-in backend without hardware: test pattern views with simulated
//! receivers. View `v` simulates receivers `2v` and `2v + 1`.

use super::BackendContext;
use crate::commands::{AppCommand, BackendEvent, MessageLevel};
use crate::settings::{ReceiverSettings, ViewSource};
use crate::source::test_pattern::{TestPatternSource, TestSourceConfig};
use openvrx_core::RxId;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// Channel shown by a view whose receivers are not configured.
fn default_freq(view: usize) -> u32 {
    crate::channels::lookup('R', (1 + 2 * view as u8).min(8)).unwrap_or(5658)
}

/// Runs the test backend until shutdown.
pub fn run(ctx: BackendContext) {
    ctx.status.publish(|s| {
        s.name = "test".into();
        s.live = false;
        s.recording.available = false;
        s.devices = vec!["test patterns, simulated receivers".into()];
    });
    let mut receivers: Vec<ReceiverSettings> = Vec::new();
    let mut freqs: Vec<Option<Arc<AtomicU32>>> = vec![None; crate::settings::VIEW_SLOTS];
    loop {
        let cmd = match ctx.next_command(Duration::from_millis(200)) {
            Ok(Some(c)) => c,
            Ok(None) => continue,
            Err(()) => return,
        };
        match cmd {
            AppCommand::Shutdown => return,
            AppCommand::OpenView { view, source } => match source {
                ViewSource::Test { pattern, standard } => {
                    let rx_base = (2 * view) as RxId;
                    let cfg = TestSourceConfig::new(pattern, standard, rx_base);
                    let f = receivers
                        .get(rx_base as usize)
                        .map_or_else(|| default_freq(view), |r| r.freq_mhz);
                    cfg.freq_mhz.store(f, Ordering::Relaxed);
                    if let Some(slot) = freqs.get_mut(view) {
                        *slot = Some(Arc::clone(&cfg.freq_mhz));
                    }
                    ctx.emit(BackendEvent::ViewOpened {
                        view,
                        source: Box::new(TestPatternSource::start(cfg)),
                    });
                }
                ViewSource::Receivers { .. } => ctx.emit(BackendEvent::ViewFailed {
                    view,
                    error: "the test backend has no receivers (start with --source live)".into(),
                }),
            },
            AppCommand::CloseView { view } => {
                if let Some(slot) = freqs.get_mut(view) {
                    *slot = None;
                }
            }
            AppCommand::ConfigureReceiver { rx, config } => {
                let k = rx as usize;
                if receivers.len() <= k {
                    receivers.resize(k + 1, ReceiverSettings::default());
                }
                if let Some(Some(f)) = freqs.get(k / 2) {
                    if k.is_multiple_of(2) {
                        f.store(config.freq_mhz, Ordering::Relaxed);
                    }
                }
                receivers[k] = config;
            }
            AppCommand::Calibrate { rx } => ctx.message(
                MessageLevel::Info,
                format!("rx{rx}: the test backend has no sampling phase to calibrate"),
            ),
            AppCommand::SetVideo(_) => {}
            AppCommand::StartRecording { .. } => ctx.message(
                MessageLevel::Warning,
                "recording needs the live backend: test patterns have no raw I/Q",
            ),
            AppCommand::StopRecording => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::BackendHandle;
    use crate::settings::TestPattern;
    use crate::source::noop_waker;
    use openvrx_core::VideoStandard;

    fn wait_event(h: &BackendHandle) -> BackendEvent {
        let t0 = std::time::Instant::now();
        loop {
            if let Some(e) = h.try_event() {
                return e;
            }
            assert!(t0.elapsed() < Duration::from_secs(3), "no event");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn opens_test_views_and_rejects_live() {
        let h = BackendHandle::spawn("test", noop_waker(), run).unwrap();
        h.send(AppCommand::OpenView {
            view: 1,
            source: ViewSource::Test {
                pattern: TestPattern::Bars,
                standard: VideoStandard::Ntsc,
            },
        });
        assert!(matches!(wait_event(&h), BackendEvent::ViewOpened { view: 1, .. }));
        h.send(AppCommand::OpenView {
            view: 0,
            source: ViewSource::Receivers { rx: vec![0] },
        });
        assert!(matches!(wait_event(&h), BackendEvent::ViewFailed { view: 0, .. }));
        h.send(AppCommand::StartRecording {
            folder: std::env::temp_dir(),
        });
        assert!(matches!(wait_event(&h), BackendEvent::Message { .. }));
        assert_eq!(default_freq(0), 5658);
        assert_eq!(default_freq(3), 5880);
    }
}
