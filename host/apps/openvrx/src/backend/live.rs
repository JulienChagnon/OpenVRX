// SPDX-License-Identifier: GPL-3.0-or-later
//! Live backend: FX2/C5 receivers through `openvrx-hw`, per-receiver DSP
//! (`openvrx-dsp`), per-view video pipelines (`openvrx-video`) and raw
//! recording (`openvrx-format`).
//!
//! ```text
//! [USB thread per FX2, openvrx-hw] -> RawBlock channel
//! [DSP thread per receiver]  RxDsp::push_bytes -> LineRecord -> views
//!                            \-> RawBlock copy -> recorder (when on)
//! [video thread per view]    VideoPipeline::push_line -> FrameProducer
//! [recorder thread]          RecordingWriter::write_block
//! [backend thread]           commands, C5 events, gain loop, status
//! ```
//!
//! Hardware is opened on the first command that needs it (opening a
//! receiver view, calibrating or recording), so the app starts instantly
//! and a missing board is reported instead of blocking.

use super::BackendContext;
use crate::commands::{AppCommand, BackendEvent, MessageLevel, RecordingStatus};
use crate::settings::{
    ControlMode, DiversityMode, GainMode, ReceiverSettings, VideoSettings, ViewSource, DEFAULT_GAIN,
};
use crate::source::{
    ExchangeSource, FrameExchange, FrameProducer, LockState, RowUpdate, RxStatus, SourceStatus,
    StatusCell,
};
use openvrx_core::clock::now_ns;
use openvrx_core::{LanePreset, LineRecord, RawBlock, RxId, VideoStandard};
use openvrx_dsp::{RxDsp, RxDspConfig, RxEvent, RxStats};
use openvrx_format::{ReceiverInfo, RecordingSource, RecordingWriter, Sidecar, WriterOptions};
use openvrx_hw::c5::{find_c5_ports, C5Client, C5Event, C5Settings, ClientConfig, PortConnector, PortSelector};
use openvrx_hw::control::{GainConfig, GainLoop, GainTable};
use openvrx_hw::fx2::{self, list_fx2, Fx2Found, Fx2Kind, Fx2Selector, StreamStats};
use openvrx_hw::receiver::{auto_pair, LivePairIo, Receiver, ReceiverConfig, ReceiverEvent};
use openvrx_video::{CombineMode, DeinterlaceMethod, VideoConfig, VideoPipeline};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use crossbeam_channel::{RecvTimeoutError, TrySendError};
use std::sync::mpsc::{self, Receiver as MpscReceiver, Sender, SyncSender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

/// Lines buffered between the DSP and a view's video thread.
const LINE_QUEUE: usize = 2048;
/// Queued lines beyond which the video thread drops its backlog and
/// restarts at the next field (about 65 ms behind, see `video_loop`).
const LINE_BACKLOG: usize = LINE_QUEUE / 2;
/// Raw blocks buffered toward the recorder.
const REC_QUEUE: usize = 1024;
/// Minimum agreement for pairing a C5 with an FX2 by self-test pattern.
const PAIR_MIN_MATCH: f32 = 0.9;
/// Status refresh period.
const STATUS_NS: u64 = 100_000_000;

// ---------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------

/// True when the C5 steers its own gain for this receiver: gain Auto on a
/// layout that carries the top ADC bits (the C5 reads its level back from
/// the lane pads). Folded layouts and angle8 keep the host gain loop.
pub fn gain_on_c5(r: &ReceiverSettings) -> bool {
    r.gain == GainMode::Auto
        && matches!(
            r.lanes,
            LanePreset::I8Q8 | LanePreset::I7Q7 | LanePreset::I6Q6 | LanePreset::I5Q5 | LanePreset::I4Q4Top
        )
}

/// C5 settings for a receiver at a gain index.
pub fn c5_settings(r: &ReceiverSettings, gain: u32) -> C5Settings {
    C5Settings {
        board: r.board,
        lanes: r.lanes,
        divider: r.divider as u32,
        mhz: r.freq_mhz,
        gain,
        agc: Some(gain_on_c5(r)),
        ..C5Settings::default()
    }
}

/// Initial gain index of a receiver.
pub fn initial_gain(r: &ReceiverSettings) -> u32 {
    match r.gain {
        GainMode::Auto => DEFAULT_GAIN as u32,
        GainMode::Manual { index } => index as u32,
    }
}

/// DSP configuration for a receiver.
pub fn dsp_config(r: &ReceiverSettings, standard: Option<VideoStandard>) -> RxDspConfig {
    RxDspConfig {
        sample_format: openvrx_core::SampleFormat::from_preset(r.lanes, r.divider as u32),
        standard,
        ..RxDspConfig::default()
    }
}

/// Video pipeline configuration for the app's video settings.
pub fn video_config(v: &VideoSettings, standard: VideoStandard) -> VideoConfig {
    let mut cfg = VideoConfig::preset(v.effective_preset().name(), standard)
        .unwrap_or_else(|| VideoConfig::raw(standard));
    cfg.combiner.mode = match v.diversity {
        DiversityMode::Select => CombineMode::Select,
        DiversityMode::Blend => CombineMode::Blend,
        DiversityMode::SampleRepair => CombineMode::Repair,
    };
    if v.steady_osd {
        cfg.deinterlace = DeinterlaceMethod::LineDouble;
    }
    cfg.streaming = true;
    // The display reads current_frame(); finished-frame copies are not needed.
    cfg.emit_frames = false;
    cfg
}

/// Recording base path in `folder` for a `created_utc` string
/// (`2026-10-29T21:14:03Z` gives `openvrx-20261029-211403`).
pub fn recording_base(folder: &Path, created_utc: &str) -> PathBuf {
    let digits: String = created_utc.chars().filter(|c| c.is_ascii_digit()).collect();
    let name = if digits.len() >= 14 {
        format!("openvrx-{}-{}", &digits[..8], &digits[8..14])
    } else {
        format!("openvrx-{digits}")
    };
    folder.join(name)
}

/// Stable selector for a found FX2.
pub fn fx2_selector(f: &Fx2Found) -> Fx2Selector {
    match &f.serial {
        Some(s) if !s.is_empty() => Fx2Selector::Serial(s.clone()),
        _ => Fx2Selector::Location {
            bus_id: f.location.bus_id.clone(),
            port_chain: f.location.port_chain.clone(),
        },
    }
}

/// OSD lock state from the DSP's.
pub fn lock_state(l: openvrx_dsp::LockState) -> LockState {
    match l {
        openvrx_dsp::LockState::Searching => LockState::Searching,
        openvrx_dsp::LockState::Locked => LockState::Locked,
        openvrx_dsp::LockState::Coasting => LockState::Flywheel,
    }
}

/// OSD receiver line from DSP and stream statistics.
pub fn rx_status(
    rx: RxId,
    r: &ReceiverSettings,
    gain: u32,
    stats: Option<&RxStats>,
    stream: Option<&StreamStats>,
) -> RxStatus {
    RxStatus {
        rx,
        cn_db: stats.map_or(f32::NAN, |s| s.cn_db),
        power_db: stats.map_or(f32::NAN, |s| s.power_db),
        gain: gain.min(255) as u8,
        gain_on_c5: r.control == ControlMode::Fx2Only || gain_on_c5(r),
        sync_locked: stats.is_some_and(|s| s.lock == openvrx_dsp::LockState::Locked),
        // 0 = unknown: in FX2-only mode the C5 keeps its saved channel.
        freq_mhz: match r.control {
            ControlMode::C5Console => r.freq_mhz,
            ControlMode::Fx2Only => r.fx2_only_mhz.unwrap_or(0),
        },
        sample_rate_hz: stats.map_or(crate::settings::divider_hz(r.divider), |s| s.sample_rate_hz),
        usb_bytes_per_s: stream.map_or(0.0, |s| s.rate_bps),
        dropped_samples: stats.map_or(0, |s| s.lost_samples),
        dropped_transfers: stream.map_or(0, |s| s.dropped_blocks + s.transfer_errors),
        share: 0.0,
    }
}

/// Share of clipped samples above which the OSD warns of overload. The
/// C5 clips I and Q separately, so a heavily clipped FM signal keeps only
/// a few phase steps: the picture gets rainbow stripes in its colour band.
const OVERLOAD_CLIP_PCT: f32 = 30.0;
/// I/Q RMS (in steps of the lowest carried bit) under which the OSD warns
/// that the signal is too small for the lanes: a picture of lines.
const WEAK_RMS_STEPS: f32 = 1.0;

/// OSD warning when the C5's gain does not fit the signal, else None.
/// `i_rms`/`q_rms` in 10-bit ADC units as in `RxStats`.
pub fn level_warning(rx: RxId, r: &ReceiverSettings, clip_pct: f32, i_rms: f32, q_rms: f32) -> Option<String> {
    if clip_pct >= OVERLOAD_CLIP_PCT {
        return Some(format!("rx{rx} overload: {clip_pct:.0}% of samples clipped, lower the C5 gain"));
    }
    let fmt = openvrx_core::SampleFormat::from_preset(r.lanes, r.divider as u32);
    if let openvrx_core::lanes::LaneEncoding::Iq { i, q } = &fmt.lanes.encoding {
        let step = (1u32 << i.adc_lsb().min(q.adc_lsb())) as f32;
        let rms = i_rms.hypot(q_rms) / std::f32::consts::SQRT_2 / step;
        if rms < WEAK_RMS_STEPS {
            return Some(format!("rx{rx} signal under one ADC step ({rms:.1}), raise the C5 gain"));
        }
    }
    None
}

// ---------------------------------------------------------------------
// DSP threads
// ---------------------------------------------------------------------

enum DspCmd {
    Config(Box<RxDspConfig>),
    Event(RxEvent),
}

/// Where a receiver's lines go: (view, sender, receivers of the view).
type Routes = Mutex<Vec<(usize, crossbeam_channel::Sender<LineRecord>, Vec<RxId>)>>;

/// Recorder input shared by all DSP threads.
#[derive(Default)]
struct RecTap {
    tx: Mutex<Option<SyncSender<RawBlock>>>,
    dropped: AtomicU64,
}

#[derive(Default)]
struct DspShared {
    stats: Mutex<Option<RxStats>>,
    lines_dropped: AtomicU64,
}

struct DspThread {
    cmd: Sender<DspCmd>,
    shared: Arc<DspShared>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for DspThread {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct DspCtx {
    rx: RxId,
    word_bytes: u64,
    blocks: crossbeam_channel::Receiver<RawBlock>,
    cmds: MpscReceiver<DspCmd>,
    routes: Arc<Routes>,
    tap: Arc<RecTap>,
    shared: Arc<DspShared>,
    stop: Arc<AtomicBool>,
}

fn spawn_dsp(ctx: DspCtx, cfg: RxDspConfig, cmd: Sender<DspCmd>) -> std::io::Result<DspThread> {
    let shared = Arc::clone(&ctx.shared);
    let stop = Arc::clone(&ctx.stop);
    let thread = std::thread::Builder::new()
        .name(format!("openvrx-dsp-rx{}", ctx.rx))
        .spawn(move || dsp_loop(ctx, cfg))?;
    Ok(DspThread {
        cmd,
        shared,
        stop,
        thread: Some(thread),
    })
}

fn dsp_loop(mut c: DspCtx, cfg: RxDspConfig) {
    let mut dsp = RxDsp::new(c.rx, cfg);
    let mut lines: Vec<LineRecord> = Vec::with_capacity(256);
    let mut next_byte: Option<u64> = None;
    let mut last_stats = 0u64;
    // Drop report (DSP-5): lines the video views could not take, so the next
    // picture loss says whether the DSP or the video side ran short.
    let (mut last_drop_log, mut drops_logged) = (0u64, 0u64);
    while !c.stop.load(Ordering::Relaxed) {
        while let Ok(cmd) = c.cmds.try_recv() {
            match cmd {
                DspCmd::Config(cfg) => {
                    c.word_bytes = cfg.sample_format.lanes.word_bytes as u64;
                    dsp.set_config(*cfg);
                }
                DspCmd::Event(e) => dsp.note_event(e),
            }
        }
        let block = match c.blocks.recv_timeout(Duration::from_millis(100)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        if let Some(expected) = next_byte {
            if block.first_byte > expected {
                dsp.mark_gap((block.first_byte - expected) / c.word_bytes.max(1));
            }
        }
        next_byte = Some(block.first_byte + block.data.len() as u64);
        {
            let tap = c.tap.tx.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(tx) = tap.as_ref() {
                if tx.try_send(block.clone()).is_err() {
                    c.tap.dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        lines.clear();
        dsp.push_bytes(&block.data, block.host_ns, &mut lines);
        if !lines.is_empty() {
            route_lines(c.rx, &mut lines, &c.routes, &c.shared);
        }
        let now = now_ns();
        if now - last_stats >= STATUS_NS {
            *c.shared.stats.lock().unwrap_or_else(|e| e.into_inner()) = Some(dsp.stats());
            last_stats = now;
        }
        if now - last_drop_log >= 5_000_000_000 {
            let dropped = c.shared.lines_dropped.load(Ordering::Relaxed);
            if dropped > drops_logged {
                let s = dsp.stats();
                eprintln!(
                    "openvrx: rx{} {} lines dropped in 5 s (video queue full); dsp lines_out {} load {:.2} lock {:?}",
                    c.rx,
                    dropped - drops_logged,
                    s.lines_out,
                    s.load,
                    s.lock
                );
                drops_logged = dropped;
            }
            last_drop_log = now;
        }
    }
}

fn route_lines(rx: RxId, lines: &mut Vec<LineRecord>, routes: &Routes, shared: &DspShared) {
    let mut routes = routes.lock().unwrap_or_else(|e| e.into_inner());
    let targets: Vec<usize> = routes
        .iter()
        .enumerate()
        .filter(|(_, r)| r.2.contains(&rx))
        .map(|(k, _)| k)
        .collect();
    let mut dead = Vec::new();
    for (n, &k) in targets.iter().enumerate() {
        let last = n + 1 == targets.len();
        let batch: Vec<LineRecord> = if last {
            std::mem::take(lines)
        } else {
            lines.clone()
        };
        for line in batch {
            match routes[k].1.try_send(line) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) => {
                    shared.lines_dropped.fetch_add(1, Ordering::Relaxed);
                }
                Err(TrySendError::Disconnected(_)) => {
                    dead.push(k);
                    break;
                }
            }
        }
    }
    for k in dead.into_iter().rev() {
        routes.remove(k);
    }
}

// ---------------------------------------------------------------------
// Video threads
// ---------------------------------------------------------------------

struct VideoWorker {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for VideoWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Video-path health of one view (DSP-5), written by its video thread and
/// read into the view's status.
#[derive(Default)]
struct VideoHealth {
    /// Lines the display lost since the view opened: the assembler's missing
    /// and late lines plus the combiner's late lines, over every rebuild.
    lost_lines: AtomicU64,
    /// Pipeline rebuilds by the health check.
    rebuilds: AtomicU64,
    last_rebuild_ns: AtomicU64,
}

/// Lost lines per field above which a second counts as unhealthy, and the
/// run of such seconds that rebuilds the pipeline. Calibrated on the
/// 2026-11-04 flights (live path replayed with `openvrx-cli decode --live`):
/// range-edge relocks reach 17.5 lines per field for single seconds, at
/// most two in a row; the persistent black-row fault lasted minutes.
const HEALTH_LOST_PER_FIELD: f64 = 4.0;
const HEALTH_BAD_SECONDS: u32 = 5;
const HEALTH_WINDOW_FIELDS: u64 = 60;

/// One second's verdict: the new run of unhealthy seconds and whether it
/// calls for a rebuild.
fn health_verdict(bad_seconds: u32, lost_per_field: f64) -> (u32, bool) {
    let bad = if lost_per_field > HEALTH_LOST_PER_FIELD { bad_seconds + 1 } else { 0 };
    (bad, bad >= HEALTH_BAD_SECONDS)
}

/// Rows that stop arriving every field (the black bars of 2026-11-04,
/// DSP-5) are state in the pipeline: the same recording replays clean. A
/// sustained loss rebuilds the pipeline, which resets the combiner, the
/// field assembler and the repair/deinterlace histories, as an app restart
/// did; it is logged on stderr so the next occurrence leaves evidence.
struct HealthCheck {
    fields: u64,
    lost_at_window: u64,
    bad_seconds: u32,
    logged_ns: u64,
}

impl HealthCheck {
    fn new() -> HealthCheck {
        HealthCheck {
            fields: 0,
            lost_at_window: 0,
            bad_seconds: 0,
            logged_ns: 0,
        }
    }

    fn lost(p: &VideoPipeline) -> u64 {
        let s = p.stats();
        s.assembler.missing_lines + s.assembler.late_lines + s.combiner.late_lines
    }

    /// Called after each displayed field; true when the pipeline must be rebuilt.
    fn field_done(&mut self, p: &VideoPipeline, health: &VideoHealth) -> bool {
        self.fields += 1;
        if self.fields % HEALTH_WINDOW_FIELDS != 0 {
            return false;
        }
        let lost = Self::lost(p);
        let window = lost.saturating_sub(self.lost_at_window);
        health.lost_lines.fetch_add(window, Ordering::Relaxed);
        self.lost_at_window = lost;
        let per_field = window as f64 / HEALTH_WINDOW_FIELDS as f64;
        let now = now_ns();
        if per_field > 1.0 && now.saturating_sub(self.logged_ns) > 10_000_000_000 {
            let s = p.stats();
            eprintln!(
                "openvrx: video lost {per_field:.1} lines/field (combiner {:?}; assembler {:?}; \
                 stages {:.2} ms per field)",
                s.combiner,
                s.assembler,
                s.average.total_ns() as f64 * 1e-6
            );
            self.logged_ns = now;
        }
        let rebuild;
        (self.bad_seconds, rebuild) = health_verdict(self.bad_seconds, per_field);
        if !rebuild {
            return false;
        }
        let s = p.stats();
        eprintln!(
            "openvrx: video lost more than {HEALTH_LOST_PER_FIELD} lines/field for {HEALTH_BAD_SECONDS} s \
             (last {per_field:.1}); rebuilding the video pipeline. combiner {:?}; assembler {:?}",
            s.combiner, s.assembler
        );
        health.rebuilds.fetch_add(1, Ordering::Relaxed);
        health.last_rebuild_ns.store(now, Ordering::Relaxed);
        *self = HealthCheck::new();
        true
    }
}

fn video_loop(
    lines: crossbeam_channel::Receiver<LineRecord>,
    settings: Arc<Mutex<VideoSettings>>,
    mut producer: FrameProducer,
    stop: Arc<AtomicBool>,
    health: Arc<VideoHealth>,
) {
    let mut current = settings.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let mut pipe: Option<(VideoStandard, VideoPipeline)> = None;
    let mut check = HealthCheck::new();
    let (mut catchups, mut catchup_logged_ns) = (0u64, 0u64);
    while !stop.load(Ordering::Relaxed) {
        // Fields short of lines cost this thread about twice a full one
        // (colour and repair still run over the lost lines), so once it
        // falls behind it stays behind with the queue full and most rows
        // missing (black bars of 2026-11-06, laptop on battery, weak
        // signal; the recording replays clean). Dropping the backlog and
        // restarting at the next field gets back to full fields.
        if lines.len() > LINE_BACKLOG {
            let dropped = lines.try_iter().take(LINE_QUEUE).count() as u64;
            health.lost_lines.fetch_add(dropped, Ordering::Relaxed);
            health.rebuilds.fetch_add(1, Ordering::Relaxed);
            health.last_rebuild_ns.store(now_ns(), Ordering::Relaxed);
            catchups += 1;
            let now = now_ns();
            if now.saturating_sub(catchup_logged_ns) > 10_000_000_000 {
                eprintln!(
                    "openvrx: video thread behind, dropped {dropped} queued lines and restarted the pipeline \
                     ({catchups} times so far); stages {:.2} ms per field",
                    pipe.as_ref().map_or(0.0, |(_, p)| p.stats().average.total_ns() as f64 * 1e-6)
                );
                catchup_logged_ns = now;
            }
            pipe = None;
            continue;
        }
        let line = match lines.recv_timeout(Duration::from_millis(100)) {
            Ok(l) => l,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        {
            let s = settings.lock().unwrap_or_else(|e| e.into_inner());
            if *s != current {
                current = s.clone();
                if let Some((std, p)) = pipe.as_mut() {
                    p.set_config(&video_config(&current, *std));
                }
            }
        }
        let standard = current.standard.fixed().unwrap_or(line.standard);
        if line.standard != standard {
            continue;
        }
        if pipe.as_ref().is_none_or(|(s, _)| *s != standard) {
            pipe = Some((standard, VideoPipeline::new(&video_config(&current, standard))));
            check = HealthCheck::new();
        }
        let Some((_, p)) = pipe.as_mut() else {
            continue;
        };
        let stamp = line.host_ns;
        let out = p.push_line(line);
        if let Some(rows) = out.updated_rows {
            let f = p.current_frame();
            let field_done = rows.end + 1 >= f.height;
            producer.publish(
                f,
                RowUpdate {
                    rows,
                    lines: 1,
                    field_done,
                    frame_done: out.frames_done > 0,
                    newest_line_ns: stamp,
                },
            );
            if field_done && check.field_done(p, &health) {
                // The display keeps its texture; the new pipeline rewrites
                // every row as its lines arrive.
                *p = VideoPipeline::new(&video_config(&current, standard));
            }
        }
    }
}

struct LiveView {
    rxs: Vec<RxId>,
    status: Arc<StatusCell<SourceStatus>>,
    health: Arc<VideoHealth>,
}

// ---------------------------------------------------------------------
// Recorder
// ---------------------------------------------------------------------

struct Recorder {
    thread: Option<JoinHandle<Result<Sidecar, String>>>,
    bytes: Arc<AtomicU64>,
    started_ns: u64,
    path: PathBuf,
}

fn recorder_loop(
    mut writer: RecordingWriter,
    blocks: MpscReceiver<RawBlock>,
    bytes: Arc<AtomicU64>,
) -> Result<Sidecar, String> {
    let mut first_error: Option<String> = None;
    for b in blocks.iter() {
        match writer.write_block(&b) {
            Ok(n) => {
                bytes.fetch_add(n, Ordering::Relaxed);
            }
            Err(e) => {
                if first_error.is_none() {
                    first_error = Some(e.to_string());
                }
            }
        }
    }
    let side = writer.close().map_err(|e| e.to_string())?;
    match first_error {
        Some(e) => Err(format!("write error during recording: {e}")),
        None => Ok(side),
    }
}

// ---------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------

struct LiveRx {
    rx: RxId,
    receiver: Receiver,
    dsp: DspThread,
    settings: ReceiverSettings,
    gain: u32,
    gain_loop: Option<GainLoop>,
    description: String,
    fx2_serial: String,
    connected: bool,
}

impl LiveRx {
    fn make_gain_loop(r: &ReceiverSettings, gain: u32) -> Option<GainLoop> {
        // FX2 only: the C5 keeps its saved gain (and its own loop); with the
        // C5's loop on, nothing to steer here either.
        if r.gain != GainMode::Auto || r.control == ControlMode::Fx2Only || gain_on_c5(r) {
            return None;
        }
        let lanes = r.lanes.format();
        // Index-to-dB table: 0.5 dB per step until per-board tables are
        // measured on the bench.
        GainConfig::for_lanes(&lanes).map(|cfg| GainLoop::new(cfg, &lanes, GainTable::linear(78, 0.5), gain))
    }
}

/// One receiver to open: C5 port (None when FX2 only), FX2 and their
/// descriptions.
struct Pairing {
    c5_port: Option<String>,
    fx2: Fx2Selector,
    fx2_serial: String,
}

/// Uploads firmware to blank FX2s, then pairs C5 consoles with FX2s.
fn discover(ctx: &BackendContext, wanted: &[ReceiverSettings]) -> Result<Vec<Pairing>, String> {
    let found = list_fx2().map_err(|e| format!("listing FX2 boards: {e}"))?;
    if found.iter().any(|f| f.kind == Fx2Kind::Bootloader) {
        match fx2::firmware::embedded() {
            Ok(fw) => match fx2::upload_all(&fw) {
                Ok(results) => {
                    for (dev, r) in results {
                        match r {
                            Ok(_) => ctx.message(
                                MessageLevel::Info,
                                format!("FX2 firmware loaded: {}", dev.describe()),
                            ),
                            Err(e) => ctx.message(
                                MessageLevel::Warning,
                                format!("FX2 firmware upload failed on {}: {e}", dev.describe()),
                            ),
                        }
                    }
                }
                Err(e) => ctx.message(MessageLevel::Warning, format!("FX2 upload: {e}")),
            },
            Err(e) => ctx.message(MessageLevel::Warning, format!("blank FX2 found: {e}")),
        }
    }
    let mut fx2s: Vec<Fx2Found> = list_fx2()
        .map_err(|e| format!("listing FX2 boards: {e}"))?
        .into_iter()
        .filter(|f| f.kind == Fx2Kind::Openvrx)
        .collect();
    if fx2s.is_empty() {
        return Err("no OpenVRX FX2 board found (check USB and the WinUSB driver)".into());
    }
    let serial_of = |f: &Fx2Found| f.serial.clone().unwrap_or_default();
    if fx2_only(wanted) {
        // No C5 to identify boards by pattern: explicit serials first, the
        // rest in serial order so the mapping is stable between runs.
        fx2s.sort_by_key(serial_of);
        let mut out = Vec::new();
        for (k, r) in wanted.iter().enumerate().take(fx2s.len().max(1)) {
            let pick = match &r.fx2_serial {
                Some(want) => fx2s.iter().position(|f| f.serial.as_deref() == Some(want.as_str())),
                None => (!fx2s.is_empty()).then_some(0),
            };
            let Some(i) = pick else {
                ctx.message(
                    MessageLevel::Warning,
                    format!("rx{k}: FX2 {} not found", r.fx2_serial.as_deref().unwrap_or("?")),
                );
                continue;
            };
            let f = fx2s.remove(i);
            out.push(Pairing {
                c5_port: None,
                fx2: fx2_selector(&f),
                fx2_serial: serial_of(&f),
            });
            if fx2s.is_empty() {
                break;
            }
        }
        return Ok(out);
    }
    let ports: Vec<String> = find_c5_ports()
        .map_err(|e| format!("listing C5 ports: {e}"))?
        .into_iter()
        .map(|p| p.port_name)
        .collect();
    if ports.is_empty() {
        return Err("no ESP32-C5 console port found (use FX2-only control if the C5 is not on USB)".into());
    }

    // Explicit mapping from the settings wins.
    let explicit: Vec<Pairing> = wanted
        .iter()
        .filter_map(|r| {
            let port = r.c5_port.clone()?;
            let serial = r.fx2_serial.clone()?;
            Some(Pairing {
                c5_port: Some(port),
                fx2: Fx2Selector::Serial(serial.clone()),
                fx2_serial: serial,
            })
        })
        .collect();
    if !explicit.is_empty() {
        return Ok(explicit);
    }
    if fx2s.len() == 1 && ports.len() == 1 {
        return Ok(vec![Pairing {
            c5_port: Some(ports[0].clone()),
            fx2: fx2_selector(&fx2s[0]),
            fx2_serial: serial_of(&fx2s[0]),
        }]);
    }
    // Several boards: drive a pattern from each C5 and find its FX2.
    ctx.message(
        MessageLevel::Info,
        format!("pairing {} C5 with {} FX2 by self-test pattern", ports.len(), fx2s.len()),
    );
    let clients: Vec<C5Client> = ports
        .iter()
        .enumerate()
        .filter_map(|(k, p)| {
            let mut s = wanted
                .get(k)
                .map(|r| c5_settings(r, initial_gain(r)))
                .unwrap_or_default();
            s.mhz = s.mhz.max(5000);
            C5Client::start(
                PortConnector::new(PortSelector::Name(p.clone())),
                Some(s),
                ClientConfig::default(),
            )
            .map_err(|e| log::warn!("{p}: {e}"))
            .ok()
        })
        .collect();
    for c in &clients {
        c.wait_connected(Duration::from_secs(3));
    }
    let selectors: Vec<Fx2Selector> = fx2s.iter().map(fx2_selector).collect();
    let pairs = {
        let mut io = LivePairIo {
            c5: clients.iter().collect(),
            fx2: selectors.clone(),
        };
        auto_pair(&mut io, PAIR_MIN_MATCH).map_err(|e| format!("pairing: {e}"))?
    };
    drop(clients);
    let out: Vec<Pairing> = pairs
        .iter()
        .enumerate()
        .filter_map(|(c, f)| {
            f.map(|f| Pairing {
                c5_port: Some(ports[c].clone()),
                fx2: selectors[f].clone(),
                fx2_serial: serial_of(&fx2s[f]),
            })
        })
        .collect();
    if out.is_empty() {
        return Err("no C5 could be paired with an FX2 (check the lane wiring)".into());
    }
    Ok(out)
}

/// True when receivers run on the FX2 alone (also with no receiver
/// configured yet: FX2 only is the default).
fn fx2_only(wanted: &[ReceiverSettings]) -> bool {
    match wanted.first() {
        Some(r) => r.control == ControlMode::Fx2Only,
        None => ControlMode::default() == ControlMode::Fx2Only,
    }
}

struct Live {
    rxs: Vec<LiveRx>,
    routes: Arc<Routes>,
    tap: Arc<RecTap>,
}

fn open_hardware(ctx: &BackendContext, wanted: &[ReceiverSettings]) -> Result<Live, String> {
    let pairs = discover(ctx, wanted)?;
    let default_rx = [ReceiverSettings::default()];
    let wanted = if wanted.is_empty() { &default_rx[..] } else { wanted };
    let routes: Arc<Routes> = Arc::new(Mutex::new(Vec::new()));
    let tap = Arc::new(RecTap::default());
    let mut rxs = Vec::new();
    for (k, p) in pairs.into_iter().enumerate() {
        let rx = k as RxId;
        let settings = wanted.get(k).cloned().unwrap_or_default();
        let gain = initial_gain(&settings);
        let cfg = ReceiverConfig::new(
            rx,
            PortSelector::Name(p.c5_port.clone().unwrap_or_default()),
            p.fx2.clone(),
            c5_settings(&settings, gain),
        );
        let started = match &p.c5_port {
            Some(_) => Receiver::start(cfg),
            None => Receiver::start_fx2_only(cfg),
        };
        let receiver = match started {
            Ok(r) => r,
            Err(e) => {
                let via = p.c5_port.clone().unwrap_or_else(|| format!("FX2 {}", p.fx2_serial));
                ctx.message(MessageLevel::Error, format!("rx{rx} ({via}): {e}"));
                continue;
            }
        };
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let dctx = DspCtx {
            rx,
            word_bytes: settings.lanes.format().word_bytes as u64,
            blocks: receiver.blocks().clone(),
            cmds: cmd_rx,
            routes: Arc::clone(&routes),
            tap: Arc::clone(&tap),
            shared: Arc::new(DspShared::default()),
            stop: Arc::new(AtomicBool::new(false)),
        };
        let dsp = spawn_dsp(dctx, dsp_config(&settings, None), cmd_tx).map_err(|e| e.to_string())?;
        let description = match &p.c5_port {
            Some(port) => format!("rx{rx}: C5 {port} + FX2 {}", p.fx2_serial),
            None => format!(
                "rx{rx}: FX2 {} only (C5 saved settings, {} clk {})",
                p.fx2_serial,
                settings.lanes.name(),
                settings.divider
            ),
        };
        ctx.message(MessageLevel::Info, format!("opened {description}"));
        rxs.push(LiveRx {
            rx,
            receiver,
            dsp,
            gain_loop: LiveRx::make_gain_loop(&settings, gain),
            settings,
            gain,
            description,
            fx2_serial: p.fx2_serial,
            connected: true,
        });
    }
    if rxs.is_empty() {
        return Err("no receiver could be started".into());
    }
    Ok(Live { rxs, routes, tap })
}

// ---------------------------------------------------------------------
// Backend thread
// ---------------------------------------------------------------------

struct State {
    receivers: Vec<ReceiverSettings>,
    video: Arc<Mutex<VideoSettings>>,
    live: Option<Live>,
    views: Vec<Option<LiveView>>,
    recorder: Option<Recorder>,
    hw_error: Option<String>,
}

/// Runs the live backend until shutdown.
pub fn run(ctx: BackendContext) {
    ctx.status.publish(|s| {
        s.name = "live".into();
        s.live = true;
        s.recording.available = true;
    });
    let mut st = State {
        receivers: Vec::new(),
        video: Arc::new(Mutex::new(VideoSettings::default())),
        live: None,
        views: (0..crate::settings::VIEW_SLOTS).map(|_| None).collect(),
        recorder: None,
        hw_error: None,
    };
    let mut last_status = 0u64;
    loop {
        match ctx.next_command(Duration::from_millis(20)) {
            Ok(Some(AppCommand::Shutdown)) | Err(()) => break,
            Ok(Some(cmd)) => handle(&ctx, &mut st, cmd),
            Ok(None) => {}
        }
        let now = now_ns();
        tick(&ctx, &mut st, now);
        if now - last_status >= STATUS_NS {
            publish_status(&ctx, &mut st);
            last_status = now;
        }
    }
    stop_recording(&ctx, &mut st);
    // Views and receivers stop when dropped (DSP threads join, streams
    // stop).
    st.views.clear();
    st.live = None;
}

fn ensure_live(ctx: &BackendContext, st: &mut State) -> bool {
    if st.live.is_some() {
        return true;
    }
    match open_hardware(ctx, &st.receivers) {
        Ok(l) => {
            st.hw_error = None;
            st.live = Some(l);
            true
        }
        Err(e) => {
            ctx.message(MessageLevel::Error, e.clone());
            st.hw_error = Some(e);
            false
        }
    }
}

fn handle(ctx: &BackendContext, st: &mut State, cmd: AppCommand) {
    match cmd {
        AppCommand::Shutdown => {}
        AppCommand::OpenView { view, source } => open_view(ctx, st, view, source),
        AppCommand::CloseView { view } => {
            if let Some(slot) = st.views.get_mut(view) {
                *slot = None;
            }
        }
        AppCommand::ConfigureReceiver { rx, config } => configure(ctx, st, rx, config),
        AppCommand::Calibrate { rx } => {
            if !ensure_live(ctx, st) {
                return;
            }
            let Some(r) = st
                .live
                .as_mut()
                .and_then(|l| l.rxs.iter_mut().find(|r| r.rx == rx))
            else {
                ctx.message(MessageLevel::Warning, format!("rx{rx} is not running"));
                return;
            };
            ctx.message(MessageLevel::Info, format!("rx{rx}: calibrating sampling phase"));
            match r.receiver.calibrate() {
                Ok(rep) => {
                    let _ = r.dsp.cmd.send(DspCmd::Event(RxEvent::Reset));
                    let level = if rep.success {
                        MessageLevel::Info
                    } else {
                        MessageLevel::Warning
                    };
                    ctx.message(level, format!("rx{rx}: calibration: {}", rep.message));
                }
                Err(e) => ctx.message(MessageLevel::Error, format!("rx{rx}: calibration failed: {e}")),
            }
        }
        AppCommand::SetVideo(v) => {
            *st.video.lock().unwrap_or_else(|e| e.into_inner()) = v;
        }
        AppCommand::StartRecording { folder } => start_recording(ctx, st, &folder),
        AppCommand::StopRecording => stop_recording(ctx, st),
    }
}

fn open_view(ctx: &BackendContext, st: &mut State, view: usize, source: ViewSource) {
    let rxs = match source {
        ViewSource::Receivers { rx } => rx,
        ViewSource::Test { pattern, standard } => {
            // A clean pattern next to the receivers: the reference for the
            // side-by-side comparison (no hardware, no simulated damage).
            let mut cfg = crate::source::test_pattern::TestSourceConfig::new(pattern, standard, (2 * view) as RxId);
            cfg.clean = true;
            ctx.emit(BackendEvent::ViewOpened {
                view,
                source: Box::new(crate::source::test_pattern::TestPatternSource::start(cfg)),
            });
            return;
        }
    };
    if !ensure_live(ctx, st) {
        ctx.emit(BackendEvent::ViewFailed {
            view,
            error: st.hw_error.clone().unwrap_or_default(),
        });
        return;
    }
    let Some(live) = st.live.as_ref() else {
        return;
    };
    let available: Vec<RxId> = live.rxs.iter().map(|r| r.rx).collect();
    let mut use_rx: Vec<RxId> = rxs.into_iter().filter(|r| available.contains(r)).collect();
    if use_rx.is_empty() {
        use_rx = available;
    }
    let exchange = FrameExchange::new();
    let status = Arc::new(StatusCell::new(SourceStatus {
        label: "LIVE".into(),
        lock: LockState::Searching,
        ..SourceStatus::default()
    }));
    let (tx, rx) = crossbeam_channel::bounded(LINE_QUEUE);
    let stop = Arc::new(AtomicBool::new(false));
    let health = Arc::new(VideoHealth::default());
    let thread = {
        let producer = exchange.producer();
        let settings = Arc::clone(&st.video);
        let stop = Arc::clone(&stop);
        let health = Arc::clone(&health);
        std::thread::Builder::new()
            .name(format!("openvrx-video-{view}"))
            .spawn(move || video_loop(rx, settings, producer, stop, health))
    };
    let thread = match thread {
        Ok(t) => t,
        Err(e) => {
            ctx.emit(BackendEvent::ViewFailed {
                view,
                error: format!("video thread: {e}"),
            });
            return;
        }
    };
    {
        let mut routes = live.routes.lock().unwrap_or_else(|e| e.into_inner());
        routes.retain(|r| r.0 != view);
        routes.push((view, tx, use_rx.clone()));
    }
    let worker = VideoWorker {
        stop,
        thread: Some(thread),
    };
    let label = format!(
        "LIVE {}",
        use_rx.iter().map(|r| format!("rx{r}")).collect::<Vec<_>>().join("+")
    );
    let source = ExchangeSource::new(label, exchange, Arc::clone(&status), Some(Box::new(worker)));
    if let Some(slot) = st.views.get_mut(view) {
        *slot = Some(LiveView {
            rxs: use_rx,
            status,
            health,
        });
    }
    ctx.emit(BackendEvent::ViewOpened {
        view,
        source: Box::new(source),
    });
}

fn configure(ctx: &BackendContext, st: &mut State, rx: RxId, config: ReceiverSettings) {
    let k = rx as usize;
    if st.receivers.len() <= k {
        st.receivers.resize(k + 1, ReceiverSettings::default());
    }
    st.receivers[k] = config.clone();
    let Some(r) = st
        .live
        .as_mut()
        .and_then(|l| l.rxs.iter_mut().find(|r| r.rx == rx))
    else {
        return;
    };
    let old = std::mem::replace(&mut r.settings, config.clone());
    if old == config {
        return;
    }
    if let GainMode::Manual { index } = config.gain {
        r.gain = index as u32;
    }
    if old.gain != config.gain || old.lanes != config.lanes {
        r.gain_loop = LiveRx::make_gain_loop(&config, r.gain);
    }
    if let Err(e) = r.receiver.set_settings(c5_settings(&config, r.gain)) {
        ctx.message(MessageLevel::Error, format!("rx{rx}: {e}"));
        return;
    }
    if old.lanes != config.lanes || old.divider != config.divider {
        let _ = r.dsp.cmd.send(DspCmd::Config(Box::new(dsp_config(&config, None))));
    }
    if old.freq_mhz != config.freq_mhz {
        let _ = r.dsp.cmd.send(DspCmd::Event(RxEvent::Retuned {
            mhz: config.freq_mhz,
            offset_khz: 0,
        }));
    }
    if old.gain != config.gain {
        let _ = r.dsp.cmd.send(DspCmd::Event(RxEvent::GainChanged {
            index: r.gain as u16,
        }));
    }
}

fn start_recording(ctx: &BackendContext, st: &mut State, folder: &Path) {
    if st.recorder.is_some() {
        return;
    }
    if !ensure_live(ctx, st) {
        return;
    }
    let Some(live) = st.live.as_ref() else {
        return;
    };
    let now = now_ns();
    let infos: Vec<ReceiverInfo> = live
        .rxs
        .iter()
        .map(|r| {
            let s = r.receiver.settings();
            let mut info = ReceiverInfo::new(r.rx, s.sample_format());
            // FX2 only: the channel is the C5's saved one, known only from --mhz.
            info.channel_mhz = match r.settings.control {
                ControlMode::C5Console => s.mhz,
                ControlMode::Fx2Only => r.settings.fx2_only_mhz.unwrap_or(0),
            };
            info.offset_khz = s.offset_khz;
            info.gain = s.gain;
            info.p11 = s.p11;
            info.board = s.board.name().to_string();
            info.c5_fw = r
                .receiver
                .c5()
                .and_then(|c| c.version())
                .map(|v| v.fw)
                .unwrap_or_default();
            info.fx2_serial = r.fx2_serial.clone();
            info.ifclk_inverted = r.receiver.config().ifclk_inverted;
            info.start_host_ns = now;
            info
        })
        .collect();
    let mut sidecar = Sidecar::new(RecordingSource::Hardware, infos);
    sidecar.standard_hint = st.video.lock().unwrap_or_else(|e| e.into_inner()).standard.fixed();
    if let Err(e) = std::fs::create_dir_all(folder) {
        ctx.message(MessageLevel::Error, format!("{}: {e}", folder.display()));
        return;
    }
    let base = recording_base(folder, &sidecar.created_utc);
    let writer = match RecordingWriter::create(&base, sidecar, WriterOptions::default()) {
        Ok(w) => w,
        Err(e) => {
            ctx.message(MessageLevel::Error, format!("cannot start recording: {e}"));
            return;
        }
    };
    let path = writer.paths().sidecar();
    let (tx, rx) = mpsc::sync_channel(REC_QUEUE);
    let bytes = Arc::new(AtomicU64::new(0));
    let thread = {
        let bytes = Arc::clone(&bytes);
        std::thread::Builder::new()
            .name("openvrx-recorder".into())
            .spawn(move || recorder_loop(writer, rx, bytes))
    };
    match thread {
        Ok(t) => {
            live.tap.dropped.store(0, Ordering::Relaxed);
            *live.tap.tx.lock().unwrap_or_else(|e| e.into_inner()) = Some(tx);
            ctx.message(MessageLevel::Info, format!("recording to {}", path.display()));
            st.recorder = Some(Recorder {
                thread: Some(t),
                bytes,
                started_ns: now,
                path,
            });
        }
        Err(e) => ctx.message(MessageLevel::Error, format!("recorder thread: {e}")),
    }
}

fn stop_recording(ctx: &BackendContext, st: &mut State) {
    let Some(mut rec) = st.recorder.take() else {
        return;
    };
    let mut dropped = 0;
    if let Some(live) = st.live.as_ref() {
        *live.tap.tx.lock().unwrap_or_else(|e| e.into_inner()) = None;
        dropped = live.tap.dropped.load(Ordering::Relaxed);
    }
    let result = rec.thread.take().map(|t| t.join());
    let mb = rec.bytes.load(Ordering::Relaxed) as f64 / 1e6;
    match result {
        Some(Ok(Ok(_))) => {
            let note = if dropped > 0 {
                format!(" ({dropped} blocks dropped: disk too slow)")
            } else {
                String::new()
            };
            ctx.message(
                MessageLevel::Info,
                format!("recording saved: {} ({mb:.1} MB){note}", rec.path.display()),
            );
        }
        Some(Ok(Err(e))) => ctx.message(MessageLevel::Error, format!("recording: {e}")),
        _ => ctx.message(MessageLevel::Error, "recorder thread failed"),
    }
}

/// Receiver housekeeping: C5 events, calibration on boot, gain loop.
fn tick(ctx: &BackendContext, st: &mut State, now: u64) {
    let Some(live) = st.live.as_mut() else {
        return;
    };
    for r in &mut live.rxs {
        for ev in r.receiver.poll() {
            match ev {
                ReceiverEvent::C5(C5Event::Disconnected { reason, .. }) => {
                    r.connected = false;
                    ctx.message(MessageLevel::Warning, format!("rx{}: C5 disconnected: {reason}", r.rx));
                }
                ReceiverEvent::C5(C5Event::Connected { .. }) => r.connected = true,
                ReceiverEvent::C5(C5Event::Boot { reason, .. }) => {
                    let _ = r.dsp.cmd.send(DspCmd::Event(RxEvent::Reset));
                    ctx.message(MessageLevel::Warning, format!("rx{}: C5 reset ({reason})", r.rx));
                }
                ReceiverEvent::C5(C5Event::StatePushed { errors, .. }) if !errors.is_empty() => {
                    ctx.message(
                        MessageLevel::Warning,
                        format!("rx{}: C5 refused: {}", r.rx, errors.join(", ")),
                    );
                }
                ReceiverEvent::Calibrated(rep) => ctx.message(
                    MessageLevel::Info,
                    format!("rx{}: calibration: {}", r.rx, rep.message),
                ),
                ReceiverEvent::CalibrationFailed(e) => {
                    ctx.message(MessageLevel::Warning, format!("rx{}: calibration failed: {e}", r.rx))
                }
                ReceiverEvent::C5(_) => {}
            }
        }
        let Some(gl) = r.gain_loop.as_mut() else {
            continue;
        };
        let stats = r.dsp.shared.stats.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(s) = stats {
            gl.measure(now, s.i_rms.max(s.q_rms), s.clip_pct / 100.0);
        }
        // The C5 applies a gain step at once; the DSP locates the step in
        // the data from the GainChanged note.
        if let Some(change) = gl.poll(now, true) {
            r.gain = change.to;
            match r.receiver.set_settings(c5_settings(&r.settings, r.gain)) {
                Ok(()) => {
                    let _ = r.dsp.cmd.send(DspCmd::Event(RxEvent::GainChanged {
                        index: r.gain as u16,
                    }));
                }
                Err(e) => ctx.message(MessageLevel::Warning, format!("rx{}: gain: {e}", r.rx)),
            }
        }
    }
}

fn publish_status(ctx: &BackendContext, st: &mut State) {
    let mut devices = Vec::new();
    let mut warnings = Vec::new();
    if let Some(e) = &st.hw_error {
        warnings.push(e.clone());
    }
    let mut per_rx: Vec<(RxId, RxStatus, Option<RxStats>)> = Vec::new();
    if let Some(live) = st.live.as_ref() {
        for r in &live.rxs {
            devices.push(r.description.clone());
            if !r.connected {
                warnings.push(format!("rx{} C5 disconnected", r.rx));
            }
            let stats = r.dsp.shared.stats.lock().unwrap_or_else(|e| e.into_inner()).clone();
            let stream = r.receiver.stream_stats();
            let level = stats
                .as_ref()
                .filter(|s| s.samples_processed > 0)
                .and_then(|s| level_warning(r.rx, &r.settings, s.clip_pct, s.i_rms, s.q_rms));
            if let Some(w) = level {
                warnings.push(w);
            }
            let mut status = rx_status(r.rx, &r.settings, r.gain, stats.as_ref(), stream.as_ref());
            status.dropped_samples += r.dsp.shared.lines_dropped.load(Ordering::Relaxed);
            per_rx.push((r.rx, status, stats));
        }
    }
    for view in st.views.iter().flatten() {
        let mine: Vec<&(RxId, RxStatus, Option<RxStats>)> =
            per_rx.iter().filter(|p| view.rxs.contains(&p.0)).collect();
        let best = mine
            .iter()
            .filter_map(|p| p.2.as_ref())
            .max_by(|a, b| a.cn_db.partial_cmp(&b.cn_db).unwrap_or(std::cmp::Ordering::Equal));
        let lock = best.map_or(LockState::NoSignal, |s| lock_state(s.lock));
        let freq = mine.first().map(|p| p.1.freq_mhz);
        let preset = st.video.lock().unwrap_or_else(|e| e.into_inner()).effective_preset();
        let lost = view.health.lost_lines.load(Ordering::Relaxed);
        let rebuilds = view.health.rebuilds.load(Ordering::Relaxed);
        let since = now_ns().saturating_sub(view.health.last_rebuild_ns.load(Ordering::Relaxed));
        // View-only warnings (the backend-wide ones reach the OSD on their own).
        let mut view_warnings = Vec::new();
        if rebuilds > 0 && since < 10_000_000_000 {
            view_warnings.push(format!("video rows lost: pipeline reset ({rebuilds}x)"));
        }
        view.status.publish(|s| {
            s.dropped_lines = lost;
            s.warnings = view_warnings;
            s.label = freq.map_or_else(|| "LIVE".into(), crate::channels::label);
            s.standard = best.and_then(|b| b.standard);
            s.lock = lock;
            s.freq_mhz = freq;
            s.field_rate_hz = best.map_or(0.0, |b| b.field_rate_hz as f32);
            s.pipeline = preset.name().into();
            s.receivers = mine.iter().map(|p| p.1.clone()).collect();
            s.simulated = false;
        });
    }
    let recording = match &st.recorder {
        Some(r) => RecordingStatus {
            available: true,
            active: true,
            started_ns: r.started_ns,
            bytes: r.bytes.load(Ordering::Relaxed),
            path: Some(r.path.clone()),
        },
        None => RecordingStatus {
            available: true,
            ..RecordingStatus::default()
        },
    };
    ctx.status.publish(|s| {
        s.devices = devices;
        s.warnings = warnings;
        s.recording = recording;
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::{StandardChoice, VideoPreset};
    use openvrx_core::LanePreset;

    #[test]
    fn c5_mapping() {
        let r = ReceiverSettings {
            freq_mhz: 5917,
            lanes: LanePreset::I8Q8,
            divider: 12,
            gain: GainMode::Manual { index: 70 },
            ..ReceiverSettings::default()
        };
        let g = initial_gain(&r);
        assert_eq!(g, 70);
        let s = c5_settings(&r, g);
        assert_eq!((s.mhz, s.divider, s.gain, s.lanes), (5917, 12, 70, LanePreset::I8Q8));
        assert!(s.validate().is_ok());
        assert_eq!(initial_gain(&ReceiverSettings::default()), DEFAULT_GAIN as u32);
        let d = dsp_config(&r, Some(VideoStandard::Pal));
        assert_eq!(d.sample_format.divider, 12);
        assert_eq!(d.standard, Some(VideoStandard::Pal));
    }

    #[test]
    fn video_mapping() {
        let mut v = VideoSettings {
            preset: VideoPreset::Quality,
            standard: StandardChoice::Auto,
            diversity: DiversityMode::Select,
            cleanup: true,
            steady_osd: false,
        };
        let q = video_config(&v, VideoStandard::Ntsc);
        assert_eq!(q.combiner.mode, CombineMode::Select);
        assert!(q.streaming);
        assert_eq!(q, {
            let mut e = VideoConfig::quality(VideoStandard::Ntsc);
            e.combiner.mode = CombineMode::Select;
            e.streaming = true;
            e.emit_frames = false;
            e
        });
        v.cleanup = false;
        let raw = video_config(&v, VideoStandard::Ntsc);
        assert!(!raw.repair_enabled);
        v.steady_osd = true;
        assert_eq!(video_config(&v, VideoStandard::Ntsc).deinterlace, DeinterlaceMethod::LineDouble);
    }

    #[test]
    fn level_warnings() {
        // 4x4top: one step is 64 ADC units.
        let r = ReceiverSettings {
            lanes: openvrx_core::LanePreset::I4Q4Top,
            ..ReceiverSettings::default()
        };
        assert_eq!(level_warning(0, &r, 0.0, 300.0, 300.0), None);
        assert!(level_warning(0, &r, 99.0, 450.0, 450.0).unwrap().contains("overload"));
        assert!(level_warning(0, &r, 0.0, 40.0, 40.0).unwrap().contains("raise"));
    }

    #[test]
    fn recording_names() {
        let p = recording_base(Path::new("rec"), "2026-10-29T21:14:03Z");
        assert_eq!(p, Path::new("rec").join("openvrx-20261029-211403"));
    }

    #[test]
    fn health_ignores_range_edge_and_catches_persistent_loss() {
        let run = |per_second: &[f64]| {
            let mut bad = 0;
            per_second.iter().any(|&v| {
                let (b, rebuild) = health_verdict(bad, v);
                bad = b;
                rebuild
            })
        };
        // 2026-11-04 range test (231926), lost lines per field per second
        // through the live path: never a rebuild.
        let range = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.4, 0.0, 17.5, 4.3, 0.0, 0.0, 17.5, 0.0];
        assert!(!run(&range));
        // Four bad seconds, a good one, four more: still not persistent.
        assert!(!run(&[9.0, 9.0, 9.0, 9.0, 0.0, 9.0, 9.0, 9.0, 9.0]));
        // Rows lost every field for five seconds: rebuild.
        assert!(run(&[0.0, 12.0, 12.0, 12.0, 12.0, 12.0]));
        assert_eq!(health_verdict(4, 12.0), (5, true));
        assert_eq!(health_verdict(4, 4.0), (0, false));
    }

    #[test]
    fn status_without_stats() {
        let r = ReceiverSettings::default();
        let s = rx_status(3, &r, 64, None, None);
        assert!(s.cn_db.is_nan());
        assert!(!s.sync_locked);
        // FX2 only (default): the C5's saved channel is unknown without --mhz.
        assert_eq!(s.freq_mhz, 0);
        let known = ReceiverSettings {
            fx2_only_mhz: Some(5880),
            ..r.clone()
        };
        assert_eq!(rx_status(3, &known, 64, None, None).freq_mhz, 5880);
        let console = ReceiverSettings {
            control: ControlMode::C5Console,
            ..r
        };
        assert_eq!(rx_status(3, &console, 64, None, None).freq_mhz, 5658);
        assert!((s.sample_rate_hz - 240e6 / 18.0).abs() < 1.0);
    }
}
