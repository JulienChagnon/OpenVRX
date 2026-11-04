// SPDX-License-Identifier: GPL-3.0-or-later
//! Backend hosting: a thread that receives [`AppCommand`]s, emits
//! [`BackendEvent`]s and keeps a [`BackendStatus`] snapshot.
//!
//! A backend is just a function taking a [`BackendContext`]; it runs on
//! its own thread and must return when it receives
//! [`AppCommand::Shutdown`] or when the command channel closes.

pub mod live;
pub mod test;

use crate::commands::{AppCommand, BackendEvent, BackendStatus, MessageLevel};
use crate::source::{StatusCell, Waker};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

/// What a backend thread gets.
pub struct BackendContext {
    pub commands: Receiver<AppCommand>,
    events: Sender<BackendEvent>,
    pub status: Arc<StatusCell<BackendStatus>>,
    pub waker: Waker,
}

impl BackendContext {
    /// Sends an event and wakes the display thread.
    pub fn emit(&self, ev: BackendEvent) {
        if self.events.send(ev).is_ok() {
            (self.waker)();
        }
    }

    pub fn message(&self, level: MessageLevel, text: impl Into<String>) {
        let text = text.into();
        match level {
            MessageLevel::Info => log::info!("{text}"),
            MessageLevel::Warning => log::warn!("{text}"),
            MessageLevel::Error => log::error!("{text}"),
        }
        self.emit(BackendEvent::Message { level, text });
    }

    /// Next command, waiting at most `timeout`. Err(()) when the app is
    /// gone.
    pub fn next_command(&self, timeout: Duration) -> Result<Option<AppCommand>, ()> {
        match self.commands.recv_timeout(timeout) {
            Ok(c) => Ok(Some(c)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(()),
        }
    }
}

/// The app's side of a running backend.
pub struct BackendHandle {
    name: String,
    tx: Sender<AppCommand>,
    events: Receiver<BackendEvent>,
    status: Arc<StatusCell<BackendStatus>>,
    thread: Option<JoinHandle<()>>,
}

impl BackendHandle {
    /// Starts `run` on a new thread.
    pub fn spawn(
        name: &str,
        waker: Waker,
        run: impl FnOnce(BackendContext) + Send + 'static,
    ) -> std::io::Result<BackendHandle> {
        let (tx, commands) = mpsc::channel();
        let (events_tx, events) = mpsc::channel();
        let status = Arc::new(StatusCell::new(BackendStatus {
            name: name.to_string(),
            ..BackendStatus::default()
        }));
        let ctx = BackendContext {
            commands,
            events: events_tx,
            status: Arc::clone(&status),
            waker,
        };
        let thread = std::thread::Builder::new()
            .name(format!("backend-{name}"))
            .spawn(move || run(ctx))?;
        Ok(BackendHandle {
            name: name.to_string(),
            tx,
            events,
            status,
            thread: Some(thread),
        })
    }

    pub fn send(&self, cmd: AppCommand) {
        if self.tx.send(cmd).is_err() {
            log::warn!("backend {} is not running", self.name);
        }
    }

    pub fn try_event(&self) -> Option<BackendEvent> {
        self.events.try_recv().ok()
    }

    pub fn read_status(&self, out: &mut BackendStatus, seen: &mut u64) -> bool {
        self.status.read_into(out, seen)
    }
}

impl Drop for BackendHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(AppCommand::Shutdown);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::noop_waker;

    #[test]
    fn spawn_send_receive_shutdown() {
        let h = BackendHandle::spawn("echo", noop_waker(), |ctx| loop {
            match ctx.next_command(Duration::from_millis(50)) {
                Ok(Some(AppCommand::Shutdown)) | Err(()) => return,
                Ok(Some(c)) => {
                    ctx.status.publish(|s| s.live = true);
                    ctx.message(MessageLevel::Info, format!("{c:?}"));
                }
                Ok(None) => {}
            }
        })
        .unwrap();
        h.send(AppCommand::StopRecording);
        let t0 = std::time::Instant::now();
        let ev = loop {
            if let Some(e) = h.try_event() {
                break e;
            }
            assert!(t0.elapsed() < Duration::from_secs(2));
            std::thread::sleep(Duration::from_millis(5));
        };
        assert!(matches!(ev, BackendEvent::Message { .. }));
        let mut st = BackendStatus::default();
        let mut seen = 0;
        assert!(h.read_status(&mut st, &mut seen));
        assert!(st.live);
        drop(h);
    }
}
