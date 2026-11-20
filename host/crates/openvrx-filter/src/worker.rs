// SPDX-License-Identifier: GPL-3.0-or-later
//! Plugin instances served by one worker thread per library.
//!
//! The ABI promises a plugin that every call into its library comes from
//! one thread, one at a time. So each library path gets one worker thread
//! for the life of the process; it loads the library, runs `create`,
//! `process` and `destroy` for every instance of it, in the order asked.
//! An [`Instance`] is the caller's side: a one-field mailbox, the newest
//! result, recycled buffers and timing, all behind one short lock that the
//! worker never holds during a plugin call.

use crate::field::FieldBuffers;
use crate::library::Library;
use crate::plugin::{Plugin, PluginInfo};
use crate::stats::{Timing, TimingSummary};
use openvrx_filter_abi as abi;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// Process times kept for the percentiles (10 s of NTSC fields).
const TIMING_WINDOW: usize = 600;
/// Free field buffers an instance keeps (about 10 MB each for NTSC).
const SPARE_BUFFERS: usize = 3;
/// How long dropping an instance waits for its `destroy`.
const DROP_WAIT: Duration = Duration::from_millis(100);

/// What became of a field the worker processed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// The plugin wrote the output planes.
    Written,
    /// The plugin left them: show the input.
    NotWritten,
    /// A plugin error: the instance is failed and destroyed.
    Error(i32),
}

/// An instance's state.
#[derive(Clone, Debug, PartialEq)]
pub enum Phase {
    /// The worker is loading the library or running `create`.
    Loading,
    Ready(Arc<PluginInfo>),
    /// Loading, `create` or `process` failed; nothing more is called.
    Failed(String),
    /// Destroyed.
    Closed,
}

/// The result of offering a field.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Submit {
    /// The instance had nothing in hand: the worker starts this field now,
    /// [`Instance::wait`] for it.
    Started,
    /// The worker is busy with an earlier field; this one waits in the
    /// mailbox. `replaced`: it took the place of a waiting field, which is
    /// skipped (this one then carries `OPENVRX_FIELD_DISCONTINUITY`).
    Queued { replaced: bool },
    /// Not ready (loading, failed or closed): the field was not taken.
    NotReady,
}

/// Counters of an instance.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct InstanceStats {
    /// `process` time per field.
    pub timing: TimingSummary,
    /// Fields replaced in the mailbox before the worker took them.
    pub skipped: u64,
    /// Time the worker has spent on the field in hand (0 when idle).
    pub busy_ms: f32,
}

struct State {
    phase: Phase,
    /// The mailbox: the next field for the worker.
    queued: Option<Box<FieldBuffers>>,
    /// The field in the worker's hands and since when.
    busy: Option<(u64, Instant)>,
    /// The newest finished field.
    done: Option<(Box<FieldBuffers>, Outcome)>,
    spare: Vec<Box<FieldBuffers>>,
    timing: Timing,
    skipped: u64,
}

struct Shared {
    state: Mutex<State>,
    cv: Condvar,
}

impl Shared {
    fn new() -> Shared {
        Shared {
            state: Mutex::new(State {
                phase: Phase::Loading,
                queued: None,
                busy: None,
                done: None,
                spare: Vec::new(),
                timing: Timing::new(TIMING_WINDOW),
                skipped: 0,
            }),
            cv: Condvar::new(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn set_phase(&self, phase: Phase) {
        let mut s = self.lock();
        s.phase = phase;
        self.cv.notify_all();
    }
}

fn recycle(s: &mut State, b: Box<FieldBuffers>) {
    if s.spare.len() < SPARE_BUFFERS {
        s.spare.push(b);
    }
}

enum Msg {
    Create { id: u64, config: String, shared: Arc<Shared> },
    Run { id: u64 },
    Destroy { id: u64, shared: Arc<Shared> },
}

/// One library's worker thread.
struct LibWorker {
    tx: Mutex<mpsc::Sender<Msg>>,
}

impl LibWorker {
    fn send(&self, m: Msg) {
        // The worker only stops with the process.
        let _ = self.tx.lock().unwrap_or_else(|e| e.into_inner()).send(m);
    }
}

/// Workers by library path (canonical when the file exists).
static WORKERS: Mutex<Vec<(PathBuf, Arc<LibWorker>)>> = Mutex::new(Vec::new());
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

fn worker_for(path: &Path) -> Result<Arc<LibWorker>, String> {
    let key = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    let mut all = WORKERS.lock().unwrap_or_else(|e| e.into_inner());
    if let Some((_, w)) = all.iter().find(|(p, _)| *p == key) {
        return Ok(Arc::clone(w));
    }
    let (tx, rx) = mpsc::channel();
    let p = path.to_path_buf();
    std::thread::Builder::new()
        .name("openvrx-filter".into())
        .spawn(move || worker_loop(p, rx))
        .map_err(|e| format!("filter worker thread: {e}"))?;
    let w = Arc::new(LibWorker { tx: Mutex::new(tx) });
    all.push((key, Arc::clone(&w)));
    Ok(w)
}

/// The worker: loads the library on first use and serves its instances.
fn worker_loop(path: PathBuf, rx: mpsc::Receiver<Msg>) {
    let mut lib: Option<Library> = None;
    let mut plugins: Vec<(u64, Plugin, Arc<Shared>)> = Vec::new();
    while let Ok(msg) = rx.recv() {
        match msg {
            Msg::Create { id, config, shared } => {
                // A failed load is retried with the next instance (the file
                // may have been fixed meanwhile).
                let loaded = match &lib {
                    Some(l) => Ok(l.clone()),
                    None => Library::open(&path),
                };
                let made = loaded.and_then(|l| {
                    lib = Some(l.clone());
                    Plugin::create(&l, &config)
                });
                match made {
                    Ok(p) => {
                        shared.set_phase(Phase::Ready(Arc::new(p.info().clone())));
                        plugins.push((id, p, shared));
                    }
                    Err(e) => shared.set_phase(Phase::Failed(e)),
                }
            }
            Msg::Run { id } => {
                let Some(k) = plugins.iter().position(|e| e.0 == id) else {
                    continue;
                };
                let (_, plugin, shared) = &mut plugins[k];
                let mut job = {
                    let mut s = shared.lock();
                    // Nothing waiting: a newer field took this run's place.
                    let Some(j) = s.queued.take() else { continue };
                    s.busy = Some((j.meta.index, Instant::now()));
                    j
                };
                let t0 = Instant::now();
                let r = plugin.process(&mut job);
                let ms = t0.elapsed().as_secs_f32() * 1e3;
                let outcome = match r {
                    Ok(true) => Outcome::Written,
                    Ok(false) => Outcome::NotWritten,
                    Err(c) => Outcome::Error(c),
                };
                let failed = {
                    let mut s = shared.lock();
                    s.busy = None;
                    s.timing.push(ms);
                    if let Some((old, _)) = s.done.replace((job, outcome)) {
                        recycle(&mut s, old);
                    }
                    if let Outcome::Error(c) = outcome {
                        s.phase = Phase::Failed(if c == -1000 {
                            "the host passed an empty or inconsistent field".into()
                        } else {
                            format!("plugin error {c} in process")
                        });
                        if let Some(q) = s.queued.take() {
                            recycle(&mut s, q);
                        }
                    }
                    shared.cv.notify_all();
                    matches!(outcome, Outcome::Error(_))
                };
                if failed {
                    // Destroyed here, on the worker thread.
                    plugins.remove(k);
                }
            }
            Msg::Destroy { id, shared } => {
                if let Some(k) = plugins.iter().position(|e| e.0 == id) {
                    plugins.remove(k);
                }
                shared.set_phase(Phase::Closed);
            }
        }
    }
}

/// The caller's side of one plugin instance on its library's worker.
pub struct Instance {
    worker: Option<Arc<LibWorker>>,
    shared: Arc<Shared>,
    id: u64,
    path: PathBuf,
}

impl Instance {
    /// Starts an instance of the plugin at `path` with `config`. Loading
    /// and `create` run on the worker: watch [`Instance::phase`].
    pub fn start(path: &Path, config: &str) -> Instance {
        let shared = Arc::new(Shared::new());
        let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
        let worker = match worker_for(path) {
            Ok(w) => {
                w.send(Msg::Create {
                    id,
                    config: config.to_string(),
                    shared: Arc::clone(&shared),
                });
                Some(w)
            }
            Err(e) => {
                shared.set_phase(Phase::Failed(e));
                None
            }
        };
        Instance {
            worker,
            shared,
            id,
            path: path.to_path_buf(),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn phase(&self) -> Phase {
        self.shared.lock().phase.clone()
    }

    /// The plugin's info once it is ready.
    pub fn info(&self) -> Option<Arc<PluginInfo>> {
        match &self.shared.lock().phase {
            Phase::Ready(i) => Some(Arc::clone(i)),
            _ => None,
        }
    }

    /// Waits until the instance is no longer loading, at most `timeout`.
    pub fn wait_ready(&self, timeout: Duration) -> Phase {
        let end = Instant::now() + timeout;
        let mut s = self.shared.lock();
        while s.phase == Phase::Loading {
            let now = Instant::now();
            if now >= end {
                break;
            }
            s = self.shared.cv.wait_timeout(s, end - now).unwrap_or_else(|e| e.into_inner()).0;
        }
        s.phase.clone()
    }

    /// A field buffer to fill: a recycled one when free.
    pub fn buffer(&self) -> Box<FieldBuffers> {
        self.shared.lock().spare.pop().unwrap_or_default()
    }

    /// Returns a buffer for reuse.
    pub fn recycle(&self, b: Box<FieldBuffers>) {
        recycle(&mut self.shared.lock(), b);
    }

    /// Offers a field to the worker (see [`Submit`]). A finished field that
    /// nobody took is stale from here on and recycled.
    pub fn submit(&self, mut job: Box<FieldBuffers>) -> Submit {
        let result = {
            let mut s = self.shared.lock();
            if !matches!(s.phase, Phase::Ready(_)) {
                recycle(&mut s, job);
                return Submit::NotReady;
            }
            if let Some((old, _)) = s.done.take() {
                recycle(&mut s, old);
            }
            let idle = s.busy.is_none() && s.queued.is_none();
            let replaced = match s.queued.take() {
                Some(old) => {
                    s.skipped += 1;
                    job.meta.flags |= abi::OPENVRX_FIELD_DISCONTINUITY;
                    recycle(&mut s, old);
                    true
                }
                None => false,
            };
            s.queued = Some(job);
            if idle {
                Submit::Started
            } else {
                Submit::Queued { replaced }
            }
        };
        if let Some(w) = &self.worker {
            w.send(Msg::Run { id: self.id });
        }
        result
    }

    /// Waits until field `index` is done or `deadline` passes, and takes
    /// it: None when it is not done in time (or the instance failed).
    pub fn wait(&self, index: u64, deadline: Instant) -> Option<(Box<FieldBuffers>, Outcome)> {
        let mut s = self.shared.lock();
        loop {
            if s.done.as_ref().is_some_and(|(j, _)| j.meta.index == index) {
                return s.done.take();
            }
            if !matches!(s.phase, Phase::Ready(_)) {
                return None;
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            s = self.shared.cv.wait_timeout(s, deadline - now).unwrap_or_else(|e| e.into_inner()).0;
        }
    }

    pub fn stats(&self) -> InstanceStats {
        let s = self.shared.lock();
        InstanceStats {
            timing: s.timing.summary(),
            skipped: s.skipped,
            busy_ms: s.busy.map_or(0.0, |(_, t)| t.elapsed().as_secs_f32() * 1e3),
        }
    }
}

impl Drop for Instance {
    /// Asks the worker to destroy the instance and waits briefly, so a
    /// replacement normally finds its resources released; never longer
    /// (the worker may be stuck in a slow plugin call).
    fn drop(&mut self) {
        let Some(w) = &self.worker else { return };
        w.send(Msg::Destroy {
            id: self.id,
            shared: Arc::clone(&self.shared),
        });
        let end = Instant::now() + DROP_WAIT;
        let mut s = self.shared.lock();
        while s.phase != Phase::Closed {
            let now = Instant::now();
            if now >= end {
                break;
            }
            s = self.shared.cv.wait_timeout(s, end - now).unwrap_or_else(|e| e.into_inner()).0;
        }
    }
}
