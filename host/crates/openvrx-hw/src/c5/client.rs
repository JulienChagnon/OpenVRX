// SPDX-License-Identifier: GPL-3.0-or-later
//! C5 console client: one I/O thread per radio head.
//!
//! Commands are sent one at a time and matched to their `OK`/`ERR` reply by
//! the echoed command word, with a timeout. `EVT` and `LOG` lines (and every
//! raw line, for the session log) are delivered on an event channel. The
//! client owns the desired [`C5Settings`]: typed setters update them, and
//! after every (re)connect and every `EVT boot` the full state is pushed so
//! a C5 reset is harmless. Lost links are reopened with backoff.

use crate::c5::proto::{
    check_command, command_word, parse_line, BoardKind, C5Settings, C5Status, Event, FilterCode, GainsInfo, Line,
    PinMap, Reply, SelftestMode, SelftestReply, VerInfo,
};
use crate::c5::transport::{SerialConnector, SerialLink};
use crate::error::{HwError, Result};
use crossbeam_channel::{bounded, Receiver, Sender};
use openvrx_core::clock::now_ns;
use openvrx_core::LanePreset;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Client settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClientConfig {
    /// Reply timeout for ordinary commands.
    pub timeout: Duration,
    /// Reply timeout for slow commands (tune, selftest, save, reboot).
    pub slow_timeout: Duration,
    pub reconnect_min: Duration,
    pub reconnect_max: Duration,
    /// Push the desired settings on connect and after `EVT boot`.
    pub push_on_connect: bool,
    /// Consecutive timeouts that mark the link dead.
    pub max_timeouts: u32,
    /// Deliver every raw line as `C5Event::Line`.
    pub log_lines: bool,
}

impl Default for ClientConfig {
    fn default() -> Self {
        ClientConfig {
            timeout: Duration::from_millis(1000),
            slow_timeout: Duration::from_millis(5000),
            reconnect_min: Duration::from_millis(200),
            reconnect_max: Duration::from_millis(2000),
            push_on_connect: true,
            max_timeouts: 3,
            log_lines: true,
        }
    }
}

impl ClientConfig {
    fn timeout_for(&self, line: &str) -> Duration {
        match command_word(line) {
            "tune" | "selftest" | "save" | "reboot" | "dl" | "lanes" | "board" => self.slow_timeout,
            _ => self.timeout,
        }
    }
}

/// Events from a C5.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum C5Event {
    Connected {
        description: String,
        ver: Option<VerInfo>,
        host_ns: u64,
    },
    /// Full state push finished; `errors` lists commands the C5 refused.
    StatePushed {
        errors: Vec<String>,
        host_ns: u64,
    },
    Disconnected {
        reason: String,
        host_ns: u64,
    },
    Reconnecting {
        attempt: u32,
        error: String,
        host_ns: u64,
    },
    /// The C5 reset (`EVT boot`); its phase calibration is lost.
    Boot {
        fw: String,
        reason: String,
        host_ns: u64,
    },
    Phy {
        state: String,
        host_ns: u64,
    },
    /// Any other `EVT`.
    Event {
        event: Event,
        host_ns: u64,
    },
    Log {
        text: String,
        host_ns: u64,
    },
    /// Raw console line (`tx` = sent by the host).
    Line {
        tx: bool,
        text: String,
        host_ns: u64,
    },
}

struct Request {
    line: String,
    timeout: Duration,
    reply: Option<Sender<Result<Reply>>>,
    /// Part of a state push.
    push: bool,
}

enum Msg {
    Req(Request),
    Push,
    Stop,
}

#[derive(Default)]
struct Shared {
    desired: Option<C5Settings>,
    ver: Option<VerInfo>,
    connected: bool,
    boots: u64,
}

/// Handle to a C5 radio head. Dropping it stops the I/O thread.
pub struct C5Client {
    tx: Sender<Msg>,
    events: Receiver<C5Event>,
    shared: Arc<Mutex<Shared>>,
    cfg: ClientConfig,
    thread: Option<JoinHandle<()>>,
    description: String,
}

impl C5Client {
    /// Starts the I/O thread. `settings` (when given) is pushed on every
    /// connect.
    pub fn start<C: SerialConnector>(
        connector: C,
        settings: Option<C5Settings>,
        cfg: ClientConfig,
    ) -> Result<C5Client> {
        if let Some(s) = &settings {
            s.validate()?;
        }
        let (tx, rx) = bounded(256);
        let (ev_tx, ev_rx) = bounded(4096);
        let shared = Arc::new(Mutex::new(Shared {
            desired: settings,
            ..Default::default()
        }));
        let description = connector.describe();
        let io = Io {
            connector,
            rx,
            events: ev_tx,
            shared: shared.clone(),
            cfg: cfg.clone(),
        };
        let thread = std::thread::Builder::new()
            .name(format!("openvrx-c5-{description}"))
            .spawn(move || io.run())
            .map_err(HwError::Io)?;
        Ok(C5Client {
            tx,
            events: ev_rx,
            shared,
            cfg,
            thread: Some(thread),
            description,
        })
    }

    pub fn describe(&self) -> &str {
        &self.description
    }

    pub fn events(&self) -> &Receiver<C5Event> {
        &self.events
    }

    pub fn is_connected(&self) -> bool {
        self.shared.lock().map(|s| s.connected).unwrap_or(false)
    }

    /// Firmware version from the last connect.
    pub fn version(&self) -> Option<VerInfo> {
        self.shared.lock().ok().and_then(|s| s.ver.clone())
    }

    /// `EVT boot` lines seen.
    pub fn boots(&self) -> u64 {
        self.shared.lock().map(|s| s.boots).unwrap_or(0)
    }

    /// Waits until connected (and the initial push queued) or times out.
    pub fn wait_connected(&self, timeout: Duration) -> bool {
        let t0 = Instant::now();
        while t0.elapsed() < timeout {
            if self.is_connected() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        false
    }

    pub fn settings(&self) -> Option<C5Settings> {
        self.shared.lock().ok().and_then(|s| s.desired.clone())
    }

    /// Replaces the desired settings and pushes them now.
    pub fn set_settings(&self, s: C5Settings) -> Result<()> {
        s.validate()?;
        if let Ok(mut sh) = self.shared.lock() {
            sh.desired = Some(s);
        }
        self.push_state();
        Ok(())
    }

    /// Queues a full state push.
    pub fn push_state(&self) {
        let _ = self.tx.send(Msg::Push);
    }

    /// Sends one raw command line and waits for its reply.
    pub fn command(&self, line: &str) -> Result<Reply> {
        let t = self.cfg.timeout_for(line);
        self.command_with_timeout(line, t)
    }

    pub fn command_with_timeout(&self, line: &str, timeout: Duration) -> Result<Reply> {
        check_command(line)?;
        let (rtx, rrx) = bounded(1);
        self.tx
            .send(Msg::Req(Request {
                line: line.to_string(),
                timeout,
                reply: Some(rtx),
                push: false,
            }))
            .map_err(|_| HwError::Disconnected("C5 client stopped".into()))?;
        // The I/O thread enforces `timeout` once the command is on the wire;
        // allow for commands queued ahead of this one.
        match rrx.recv_timeout(timeout * 4 + self.cfg.slow_timeout) {
            Ok(r) => r,
            Err(_) => Err(HwError::Timeout(format!("`{line}`: no reply"))),
        }
    }

    /// Updates the desired settings, sends the command, and reverts the
    /// settings if the C5 refuses it.
    fn set_and_send(&self, update: impl FnOnce(&mut C5Settings), line: String) -> Result<Reply> {
        let (prev, next) = {
            let mut sh = self
                .shared
                .lock()
                .map_err(|_| HwError::Invalid("poisoned state".into()))?;
            let prev = sh.desired.clone();
            if let Some(d) = sh.desired.as_mut() {
                update(d);
            }
            (prev, sh.desired.clone())
        };
        let r = self.command(&line);
        if let Err(HwError::C5 { .. }) = &r {
            if let Ok(mut sh) = self.shared.lock() {
                if sh.desired == next {
                    sh.desired = prev;
                }
            }
        }
        r
    }

    pub fn ping(&self) -> Result<()> {
        self.command("ping").map(|_| ())
    }

    pub fn ver(&self) -> Result<VerInfo> {
        VerInfo::from_reply(&self.command("ver")?)
    }

    pub fn status(&self) -> Result<C5Status> {
        C5Status::from_reply(&self.command("status")?)
    }

    pub fn board(&self, b: BoardKind) -> Result<Reply> {
        self.set_and_send(|s| s.board = b, format!("board {}", b.name()))
    }

    pub fn pins(&self) -> Result<PinMap> {
        PinMap::from_reply(&self.command("pins")?)
    }

    /// Returns the number of data pins of the preset.
    pub fn lanes(&self, p: LanePreset) -> Result<u32> {
        let r = self.set_and_send(|s| s.lanes = p, format!("lanes {}", p.name()))?;
        r.fields.parse_as("lanes", "data_pins")
    }

    /// Tunes; returns the (MHz, kHz offset) the C5 applied.
    pub fn tune(&self, mhz: u32, offset_khz: i32) -> Result<(u32, i32)> {
        let r = self.set_and_send(
            |s| {
                s.mhz = mhz;
                s.offset_khz = offset_khz;
            },
            format!("tune {mhz} {offset_khz}"),
        )?;
        Ok((r.fields.parse_as("tune", "mhz")?, r.fields.parse_as("tune", "off_khz")?))
    }

    pub fn gain(&self, index: u32) -> Result<u32> {
        let r = self.set_and_send(|s| s.gain = index, format!("gain {index}"))?;
        r.fields.parse_as("gain", "index")
    }

    pub fn gains(&self) -> Result<GainsInfo> {
        GainsInfo::from_reply(&self.command("gains")?)
    }

    /// Sets the working divider (remembered); returns the clock in Hz.
    pub fn clk(&self, divider: u32) -> Result<u32> {
        if !(6..=1023).contains(&divider) {
            return Err(HwError::Invalid(format!("divider {divider} not in 6..=1023")));
        }
        let r = self.set_and_send(|s| s.divider = divider, format!("clk {divider}"))?;
        r.fields.parse_as("clk", "hz")
    }

    /// Sets a divider without changing the remembered one (calibration).
    pub fn clk_transient(&self, divider: u32) -> Result<u32> {
        let r = self.command(&format!("clk {divider}"))?;
        r.fields.parse_as("clk", "hz")
    }

    pub fn clk_enable(&self, on: bool) -> Result<()> {
        self.command(if on { "clk on" } else { "clk off" }).map(|_| ())
    }

    /// Moves the sample clock edge `n` times; returns the total slips.
    pub fn slip(&self, n: u32) -> Result<u32> {
        let r = self.command(&format!("slip {n}"))?;
        r.fields.parse_as("slip", "total")
    }

    pub fn p11(&self, on: bool) -> Result<()> {
        self.set_and_send(|s| s.p11 = on, format!("p11 {}", if on { "on" } else { "off" }))
            .map(|_| ())
    }

    pub fn filter(&self, code: FilterCode, skirt: u32) -> Result<()> {
        let line = match code {
            FilterCode::Cal => format!("filter cal {skirt}"),
            FilterCode::Code(c) => format!("filter {c} {skirt}"),
        };
        self.set_and_send(
            |s| {
                s.filter = code;
                s.filter_skirt = skirt;
            },
            line,
        )
        .map(|_| ())
    }

    pub fn bw(&self, mhz: u32) -> Result<()> {
        if mhz != 20 && mhz != 40 {
            return Err(HwError::Invalid("bandwidth is 20 or 40 MHz".into()));
        }
        self.set_and_send(|s| s.bw_mhz = Some(mhz), format!("bw {mhz}"))
            .map(|_| ())
    }

    pub fn drive(&self, lanes: u8, clk: u8) -> Result<()> {
        if lanes > 3 || clk > 3 {
            return Err(HwError::Invalid("drive strengths are 0..=3".into()));
        }
        self.set_and_send(|s| s.drive = Some((lanes, clk)), format!("drive {lanes} {clk}"))
            .map(|_| ())
    }

    pub fn selftest(&self, mode: &SelftestMode) -> Result<SelftestReply> {
        SelftestReply::from_reply(&self.command(&mode.command())?)
    }

    pub fn save(&self) -> Result<()> {
        self.command("save").map(|_| ())
    }

    /// Reboots the C5; the client reconnects and pushes state by itself.
    pub fn reboot(&self) -> Result<()> {
        match self.command("reboot") {
            // The reset can drop the USB link before the reply is read.
            Ok(_) | Err(HwError::Disconnected(_)) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Reboots into the ROM download mode (for flashing).
    pub fn dl(&self) -> Result<()> {
        match self.command("dl") {
            // The reset can drop the USB link before the reply is read.
            Ok(_) | Err(HwError::Disconnected(_)) => Ok(()),
            Err(e) => Err(e),
        }
    }
}

impl Drop for C5Client {
    fn drop(&mut self) {
        let _ = self.tx.send(Msg::Stop);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct Io<C: SerialConnector> {
    connector: C,
    rx: Receiver<Msg>,
    events: Sender<C5Event>,
    shared: Arc<Mutex<Shared>>,
    cfg: ClientConfig,
}

enum End {
    Stop,
    Lost(String),
}

struct Session {
    queue: VecDeque<Request>,
    inflight: Option<(Request, Instant)>,
    push_errors: Option<Vec<String>>,
    timeouts: u32,
    buf: Vec<u8>,
    connected_emitted: bool,
}

impl<C: SerialConnector> Io<C> {
    fn emit(&self, e: C5Event) {
        let _ = self.events.try_send(e);
    }

    fn run(mut self) {
        let mut attempt = 0u32;
        let mut backoff = self.cfg.reconnect_min;
        loop {
            match self.connector.connect() {
                Ok(link) => {
                    attempt = 0;
                    backoff = self.cfg.reconnect_min;
                    match self.session(link) {
                        End::Stop => break,
                        End::Lost(reason) => {
                            if let Ok(mut s) = self.shared.lock() {
                                s.connected = false;
                            }
                            self.emit(C5Event::Disconnected {
                                reason,
                                host_ns: now_ns(),
                            });
                        }
                    }
                }
                Err(e) => {
                    attempt += 1;
                    self.emit(C5Event::Reconnecting {
                        attempt,
                        error: e.to_string(),
                        host_ns: now_ns(),
                    });
                    if self.idle_wait(backoff) {
                        break;
                    }
                    backoff = (backoff * 2).min(self.cfg.reconnect_max);
                }
            }
        }
        if let Ok(mut s) = self.shared.lock() {
            s.connected = false;
        }
    }

    /// Waits while disconnected, failing requests at once. True on stop.
    fn idle_wait(&self, d: Duration) -> bool {
        let deadline = Instant::now() + d;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return false;
            }
            match self.rx.recv_timeout(left) {
                Ok(Msg::Stop) => return true,
                Ok(Msg::Req(r)) => fail(
                    r,
                    HwError::Disconnected(format!("{} is not connected", self.connector.describe())),
                ),
                Ok(Msg::Push) => {}
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => return false,
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return true,
            }
        }
    }

    fn enqueue_push(&self, s: &mut Session) {
        let desired = self.shared.lock().ok().and_then(|sh| sh.desired.clone());
        let Some(d) = desired else { return };
        if !self.cfg.push_on_connect {
            return;
        }
        // Drop a push already waiting in the queue; this one supersedes it.
        s.queue.retain(|r| !r.push);
        s.push_errors = Some(Vec::new());
        for line in d.push_commands() {
            let timeout = self.cfg.timeout_for(&line);
            s.queue.push_back(Request {
                line,
                timeout,
                reply: None,
                push: true,
            });
        }
    }

    fn session(&mut self, mut link: Box<dyn SerialLink>) -> End {
        let mut s = Session {
            queue: VecDeque::new(),
            inflight: None,
            push_errors: None,
            timeouts: 0,
            buf: Vec::new(),
            connected_emitted: false,
        };
        // Terminate any partial line the C5 holds, then identify it.
        if let Err(e) = link.write_all(b"\n") {
            return End::Lost(e.to_string());
        }
        s.queue.push_back(Request {
            line: "ver".into(),
            timeout: self.cfg.timeout,
            reply: None,
            push: false,
        });
        self.enqueue_push(&mut s);
        let mut rbuf = [0u8; 512];
        let end = loop {
            // Requests from the API.
            let mut stop = false;
            while let Ok(m) = self.rx.try_recv() {
                match m {
                    Msg::Stop => stop = true,
                    Msg::Push => self.enqueue_push(&mut s),
                    Msg::Req(r) => s.queue.push_back(r),
                }
            }
            if stop {
                break End::Stop;
            }
            // Send the next command when idle.
            if s.inflight.is_none() {
                if let Some(r) = s.queue.pop_front() {
                    let mut bytes = r.line.clone().into_bytes();
                    bytes.push(b'\n');
                    if let Err(e) = link.write_all(&bytes) {
                        fail(r, HwError::Disconnected(e.to_string()));
                        break End::Lost(format!("write failed: {e}"));
                    }
                    if self.cfg.log_lines {
                        self.emit(C5Event::Line {
                            tx: true,
                            text: r.line.clone(),
                            host_ns: now_ns(),
                        });
                    }
                    let deadline = Instant::now() + r.timeout;
                    s.inflight = Some((r, deadline));
                }
            }
            // Receive.
            match link.read(&mut rbuf, Duration::from_millis(5)) {
                Ok(n) => {
                    s.buf.extend_from_slice(&rbuf[..n]);
                    while let Some(pos) = s.buf.iter().position(|&b| b == b'\n') {
                        let raw: Vec<u8> = s.buf.drain(..=pos).collect();
                        let text = String::from_utf8_lossy(&raw).trim_end_matches(['\r', '\n']).to_string();
                        if !text.is_empty() {
                            self.on_line(&mut s, &text);
                        }
                    }
                    if s.buf.len() > 4096 {
                        // A runaway line (binary noise): keep the log bounded.
                        let text = String::from_utf8_lossy(&s.buf).to_string();
                        s.buf.clear();
                        self.on_line(&mut s, &text);
                    }
                }
                Err(e) => break End::Lost(format!("read failed: {e}")),
            }
            // Reply timeout.
            if let Some((_, deadline)) = &s.inflight {
                if Instant::now() > *deadline {
                    let (r, _) = s.inflight.take().expect("checked");
                    s.timeouts += 1;
                    let msg = format!("`{}`: no reply within {} ms", r.line, r.timeout.as_millis());
                    self.finish(&mut s, r, Err(HwError::Timeout(msg)));
                    if s.timeouts >= self.cfg.max_timeouts {
                        break End::Lost(format!("{} replies timed out in a row", s.timeouts));
                    }
                }
            }
        };
        let reason = match &end {
            End::Stop => "client stopped".to_string(),
            End::Lost(r) => r.clone(),
        };
        if let Some((r, _)) = s.inflight.take() {
            fail(r, HwError::Disconnected(reason.clone()));
        }
        for r in s.queue.drain(..) {
            fail(r, HwError::Disconnected(reason.clone()));
        }
        end
    }

    fn on_line(&mut self, s: &mut Session, text: &str) {
        let host_ns = now_ns();
        if self.cfg.log_lines {
            self.emit(C5Event::Line {
                tx: false,
                text: text.to_string(),
                host_ns,
            });
        }
        match parse_line(text) {
            Line::Ok(reply) => {
                if s.inflight
                    .as_ref()
                    .is_some_and(|(r, _)| command_word(&r.line) == reply.command)
                {
                    let (r, _) = s.inflight.take().expect("checked");
                    s.timeouts = 0;
                    self.finish(s, r, Ok(reply));
                } else {
                    log::debug!("C5 reply with no pending command: {text}");
                }
            }
            Line::Err(err) => {
                if s.inflight
                    .as_ref()
                    .is_some_and(|(r, _)| command_word(&r.line) == err.command)
                {
                    let (r, _) = s.inflight.take().expect("checked");
                    s.timeouts = 0;
                    self.finish(s, r, Err(err.into_error()));
                } else {
                    log::debug!("C5 error with no pending command: {text}");
                }
            }
            Line::Evt(ev) => match ev.name.as_str() {
                "boot" => {
                    if let Ok(mut sh) = self.shared.lock() {
                        sh.boots += 1;
                    }
                    self.emit(C5Event::Boot {
                        fw: ev.fields.get("fw").unwrap_or("").to_string(),
                        reason: ev.fields.get("reason").unwrap_or("").to_string(),
                        host_ns,
                    });
                    // The C5 forgot everything: push the state again (unless
                    // the push for this connection has not started yet).
                    let push_pending = s.queue.iter().any(|r| r.push && r.line.starts_with("board"));
                    if !push_pending {
                        self.enqueue_push(s);
                    }
                }
                "phy" => self.emit(C5Event::Phy {
                    state: ev.fields.get("state").unwrap_or("").to_string(),
                    host_ns,
                }),
                _ => self.emit(C5Event::Event { event: ev, host_ns }),
            },
            Line::Log(t) => self.emit(C5Event::Log { text: t, host_ns }),
            Line::Other(_) => {}
        }
    }

    fn finish(&mut self, s: &mut Session, r: Request, result: Result<Reply>) {
        if r.line == "ver" && r.reply.is_none() {
            let ver = result.as_ref().ok().and_then(|rep| VerInfo::from_reply(rep).ok());
            if let Some(v) = &ver {
                if v.proto != crate::c5::proto::PROTO_VERSION {
                    log::warn!(
                        "C5 speaks protocol {} (host implements {})",
                        v.proto,
                        crate::c5::proto::PROTO_VERSION
                    );
                }
            }
            if let Ok(mut sh) = self.shared.lock() {
                sh.ver = ver.clone();
                sh.connected = true;
            }
            if !s.connected_emitted {
                s.connected_emitted = true;
                self.emit(C5Event::Connected {
                    description: self.connector.describe(),
                    ver,
                    host_ns: now_ns(),
                });
            }
            return;
        }
        if r.push {
            if let (Err(e), Some(errs)) = (&result, s.push_errors.as_mut()) {
                errs.push(format!("{}: {e}", r.line));
            }
            if !s.queue.iter().any(|q| q.push) && s.inflight.as_ref().is_none_or(|(q, _)| !q.push) {
                let errors = s.push_errors.take().unwrap_or_default();
                self.emit(C5Event::StatePushed {
                    errors,
                    host_ns: now_ns(),
                });
            }
        }
        if let Some(tx) = r.reply {
            let _ = tx.send(result);
        }
    }
}

fn fail(r: Request, e: HwError) {
    if let Some(tx) = r.reply {
        let _ = tx.send(Err(e));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::c5::transport::sim::SimC5;

    fn client(c5: &SimC5, settings: Option<C5Settings>) -> C5Client {
        let cfg = ClientConfig {
            timeout: Duration::from_millis(200),
            slow_timeout: Duration::from_millis(400),
            reconnect_min: Duration::from_millis(10),
            reconnect_max: Duration::from_millis(40),
            ..Default::default()
        };
        let c = C5Client::start(c5.connector(), settings, cfg).unwrap();
        assert!(c.wait_connected(Duration::from_secs(2)));
        c
    }

    fn wait_for(c: &C5Client, mut pred: impl FnMut(&C5Event) -> bool) -> Vec<C5Event> {
        let mut seen = Vec::new();
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(3) {
            if let Ok(e) = c.events().recv_timeout(Duration::from_millis(50)) {
                let hit = pred(&e);
                seen.push(e);
                if hit {
                    return seen;
                }
            }
        }
        panic!("event not seen; got {seen:?}");
    }

    #[test]
    fn connect_pushes_state_and_typed_api_works() {
        let c5 = SimC5::new();
        let s = C5Settings {
            mhz: 5800,
            gain: 60,
            ..Default::default()
        };
        let c = client(&c5, Some(s));
        let ev = wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        assert!(ev
            .iter()
            .any(|e| matches!(e, C5Event::Connected { ver: Some(v), .. } if v.fw == "0.1.0")));
        if let Some(C5Event::StatePushed { errors, .. }) = ev.last() {
            assert!(errors.is_empty(), "{errors:?}");
        }
        assert_eq!(
            c5.with(|d| (d.mhz, d.gain, d.p11, d.lanes.clone())),
            (5800, 60, true, "7x7".into())
        );
        let received = c5.with(|d| d.received.clone());
        assert_eq!(&received[..3], &["ver", "board xiao", "lanes 7x7"]);

        assert_eq!(c.tune(5732, -40).unwrap(), (5732, -40));
        assert_eq!(c.gain(70).unwrap(), 70);
        assert_eq!(c.clk(12).unwrap(), 20_000_000);
        assert_eq!(c.slip(1).unwrap(), 1);
        assert_eq!(c.slip(2).unwrap(), 3);
        c.p11(false).unwrap();
        c.filter(FilterCode::Code(3), 1).unwrap();
        c.bw(40).unwrap();
        c.drive(0, 3).unwrap();
        assert_eq!(c.lanes(LanePreset::I5Q5).unwrap(), 10);
        let st = c.status().unwrap();
        assert_eq!((st.mhz, st.off_khz, st.gain, st.clk, st.slips), (5732, -40, 70, 12, 3));
        assert_eq!(c.pins().unwrap().i.len(), 5);
        assert_eq!(c.selftest(&SelftestMode::Walk).unwrap().lanes, 10);
        c.save().unwrap();
        c.ping().unwrap();
        assert_eq!(c.ver().unwrap().board, "xiao");
        let d = c.settings().unwrap();
        assert_eq!(
            (d.mhz, d.offset_khz, d.gain, d.divider, d.p11),
            (5732, -40, 70, 12, false)
        );
        assert_eq!(d.drive, Some((0, 3)));
        // Transient clock changes are not remembered.
        c.clk_transient(6).unwrap();
        assert_eq!(c.settings().unwrap().divider, 12);
    }

    #[test]
    fn refused_setting_is_reverted() {
        let c5 = SimC5::new();
        let c = client(&c5, Some(C5Settings::default()));
        wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        match c.tune(4000, 0) {
            Err(HwError::C5 { command, code, .. }) => {
                assert_eq!(command, "tune");
                assert_eq!(code, "range");
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(c.settings().unwrap().mhz, 5732);
        assert!(c.command("bogus").is_err());
        assert!(c.command("two  spaces").is_err());
    }

    #[test]
    fn timeouts_and_events() {
        let c5 = SimC5::new();
        let c = client(&c5, None);
        c5.with(|d| d.ignore.push("status".into()));
        assert!(matches!(c.status(), Err(HwError::Timeout(_))));
        // The link still works after one timeout.
        c.ping().unwrap();
        c5.emit("EVT phy state=restored");
        c5.emit("LOG retuned");
        c5.emit("EVT custom x=1");
        let ev = wait_for(&c, |e| matches!(e, C5Event::Event { .. }));
        assert!(ev
            .iter()
            .any(|e| matches!(e, C5Event::Phy { state, .. } if state == "restored")));
        assert!(ev
            .iter()
            .any(|e| matches!(e, C5Event::Log { text, .. } if text == "retuned")));
    }

    #[test]
    fn reset_in_place_repushes_state() {
        let c5 = SimC5::new();
        let c = client(
            &c5,
            Some(C5Settings {
                mhz: 5917,
                ..Default::default()
            }),
        );
        wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        c5.reset_in_place();
        assert_eq!(c5.with(|d| d.mhz), 5865);
        let ev = wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        assert!(ev
            .iter()
            .any(|e| matches!(e, C5Event::Boot { reason, .. } if reason == "panic")));
        assert_eq!(c5.with(|d| d.mhz), 5917);
        assert_eq!(c.boots(), 1);
    }

    #[test]
    fn reconnects_after_unplug_and_reboot() {
        let c5 = SimC5::new();
        let c = client(
            &c5,
            Some(C5Settings {
                gain: 33,
                ..Default::default()
            }),
        );
        wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        c5.set_unplugged(true);
        wait_for(&c, |e| matches!(e, C5Event::Disconnected { .. }));
        assert!(matches!(c.ping(), Err(HwError::Disconnected(_))));
        wait_for(&c, |e| matches!(e, C5Event::Reconnecting { .. }));
        c5.set_unplugged(false);
        wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        assert_eq!(c5.with(|d| d.gain), 33);
        // `reboot` replies, the link drops and comes back with the state.
        c.gain(41).unwrap();
        c.reboot().unwrap();
        wait_for(&c, |e| matches!(e, C5Event::StatePushed { .. }));
        assert_eq!(c5.with(|d| d.gain), 41);
    }
}
