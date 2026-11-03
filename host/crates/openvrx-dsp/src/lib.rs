// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX per-receiver DSP: raw FX2 bytes to time-base-corrected
//! [`LineRecord`]s on the standard's line-locked grid.
//!
//! ```text
//! bytes -> unpack (LUT) -> DC + I/Q correction -> NCO (fine AFC)
//!       -> channel FIR (adaptive) -> FM demodulator -> click/wrap repair
//!       -> de-emphasis -> video low-pass -> sync/flywheel/TBC -> LineRecord
//!                 \-> spectrum tap, raw stats, glitch counter, C/N
//! ```
//!
//! Every stage has a fixed integer delay; the chain compensates them so
//! history index `j` of the video is input sample `j` (the discriminator's
//! inherent half sample is accounted for in `LineRecord::src_position`).
//!
//! Throughput: the hot loops are plain fixed-width lane loops compiled for
//! AVX2+FMA as well as the baseline target (see [`simd`]), and the chain is
//! split after the click repair so the video low-pass and the sync/TBC run
//! on a second thread, overlapped with the front half of the next block
//! (see [`RxDsp`]).
//!
//! Other public pieces: [`probe`] (sampling-phase lottery probe),
//! [`predetect`] (pre-detection diversity combining of complex I/Q from
//! several receivers), [`frontend::FrontEnd`] (unpack-side corrections
//! usable on their own), and `testsig` (synthetic signals, behind the
//! `testsig` feature).

// DSP loops index several arrays in step; explicit indices read better.
#![allow(clippy::needless_range_loop)]

#[macro_use]
pub mod simd;
pub mod channel;
pub mod clicks;
pub mod cn;
pub mod config;
pub mod demod;
pub mod fastmath;
pub mod filters;
pub mod frontend;
pub mod predetect;
pub mod probe;
pub mod spectrum;
pub mod sync;
#[cfg(any(test, feature = "testsig"))]
pub mod testsig;
pub mod unpack;
pub mod vmath;

#[cfg(test)]
mod tests;

pub use config::*;
pub use sync::LockState;

use channel::{ChannelFilter, Nco};
use clicks::ClickRepair;
use demod::{ChannelState, Demod};
use filters::{design_lowpass, first_order_shelf, kaiser_beta, kaiser_len, Biquad, SymFir};
use frontend::FrontEnd;
use openvrx_core::lanes::LaneEncoding;
use openvrx_core::{damage, Complex32, DemodChunk, LineRecord, RxId, VideoStandard};
use serde::{Deserialize, Serialize};
use spectrum::SpectrumTap;
use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use sync::{LineCtx, Tbc};

/// Control events the DSP needs to know about.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RxEvent {
    /// A gain change was commanded (the step is located in the data).
    GainChanged { index: u16 },
    /// The C5 was retuned. The same channel with a different offset is an
    /// AFC correction and moves the carrier by the offset difference.
    Retuned { mhz: u32, offset_khz: i32 },
    /// The sampling clock was slipped (timing jumps by a few samples).
    Slipped { count: u32 },
    /// The radio was reset: start over.
    Reset,
}

/// Receiver statistics for the control loops and the OSD.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RxStats {
    pub rx: RxId,
    pub sample_rate_hz: f64,
    pub samples_processed: u64,
    pub bytes_processed: u64,
    /// Raw I/Q RMS per component (DC removed), 10-bit units.
    pub i_rms: f32,
    pub q_rms: f32,
    pub i_dc: f32,
    pub q_dc: f32,
    pub clip_pct: f32,
    pub iq_gain_db: f32,
    pub iq_phase_deg: f32,
    pub power_db: f32,
    /// Envelope C/N per capture bandwidth (dB), NaN for angle-only lanes.
    pub cn_db: f32,
    /// Blanking (back porch) frequency relative to the tuned centre, Hz
    /// (NaN before sync; then the mean frequency is the best estimate).
    pub carrier_offset_hz: f32,
    /// Mean demodulated frequency relative to the tuned centre, Hz.
    pub mean_freq_hz: f32,
    /// Digital shift applied by the NCO, Hz.
    pub nco_shift_hz: f32,
    /// Sync tip to peak white in units of 6.7 MHz (VTX deviation scale).
    pub deviation_scale: f32,
    pub lock: LockState,
    pub standard: Option<VideoStandard>,
    pub line_rate_hz: f64,
    pub field_rate_hz: f64,
    pub clicks_per_s: f32,
    /// Off-ring sampling glitches per million samples (NaN when the carrier
    /// is too weak to judge).
    pub glitch_ppm: f32,
    pub wraps: u64,
    pub lines_out: u64,
    pub relocks: u64,
    pub coasted_lines: u64,
    pub gaps: u64,
    pub lost_samples: u64,
    pub gain_steps: u64,
    pub demod: DemodKind,
    pub channel_cutoff_hz: f32,
    /// Processing time over real time of the processed samples.
    pub load: f32,
    /// Mean processing time per input sample of each stage in [`STAGES`]
    /// since the chain started (ns).
    pub stage_ns: Vec<f32>,
}

dispatch! {
    /// out[k] = (d[k] * k_hz + shift) in Hz.
    fn to_hz(d: &[f32], k_hz: f64, shift: f64, out: &mut [f32]) {
        for (o, &x) in out.iter_mut().zip(d) {
            *o = (x as f64 * k_hz + shift) as f32;
        }
    }
}

dispatch! {
    /// out[k] = |z[k]|^2.
    fn power(re: &[f32], im: &[f32], out: &mut [f32]) {
        for ((o, &x), &y) in out.iter_mut().zip(re).zip(im) {
            *o = x * x + y * y;
        }
    }
}

/// Simple FIFO delay line with a fixed delay.
#[derive(Clone, Debug)]
struct Delay<T: Copy> {
    buf: Vec<T>,
    delay: usize,
}

impl<T: Copy> Delay<T> {
    fn new(delay: usize, fill: T) -> Self {
        Delay {
            buf: vec![fill; delay],
            delay,
        }
    }

    fn run(&mut self, input: &[T], out: &mut Vec<T>) {
        out.clear();
        let n = input.len();
        if n >= self.delay {
            // One copy of the block: the held tail, then the input head.
            out.extend_from_slice(&self.buf);
            out.extend_from_slice(&input[..n - self.delay]);
            self.buf.clear();
            self.buf.extend_from_slice(&input[n - self.delay..]);
        } else {
            self.buf.extend_from_slice(input);
            out.extend_from_slice(&self.buf[..n]);
            self.buf.drain(..n);
        }
        debug_assert_eq!(self.buf.len(), self.delay);
    }
}

/// Click-repaired frequency of one block on its way to the back half
/// (video low-pass and sync/TBC), with everything the back half needs.
#[derive(Default)]
struct Mid {
    /// Input index of the block's first sample.
    start: u64,
    /// Frequency in Hz (NCO shift added back), delayed by `d_pre + d_clicks`.
    hz: Vec<f32>,
    /// Damage aligned with `hz`.
    dmg: Vec<u8>,
    /// Raw |z|^2 and clip flags of the block (undelayed).
    env: Vec<f32>,
    clip: Vec<u8>,
    /// |z| aligned with `hz`, only while the video tap is on.
    mag_c: Option<Vec<f32>>,
    /// Total chain delay and the restart point (warm-up marking).
    total: usize,
    seg_start: u64,
    /// Gain steps and clicks (aligned input indices) found in this block.
    steps: Vec<u64>,
    clicks: Vec<u64>,
    host_ns: u64,
}

impl Mid {
    fn clear(&mut self) {
        self.hz.clear();
        self.dmg.clear();
        self.env.clear();
        self.clip.clear();
        self.mag_c = None;
        self.steps.clear();
        self.clicks.clear();
    }
}

/// What the front half needs to know about the TBC after a block.
#[derive(Copy, Clone, Debug)]
struct TbcSnap {
    levels: Option<(f64, f64)>,
    lock: LockState,
    standard: Option<VideoStandard>,
    report: sync::SyncReport,
    video_ns: u64,
    sync_ns: u64,
}

/// Back half of the chain: de-emphasis, video low-pass, alignment delays
/// and the sync/TBC.
struct Back {
    rx: RxId,
    fs: f64,
    deemph: Option<Biquad>,
    lpf: SymFir,
    tbc: Tbc,
    env_delay: Delay<f32>,
    clip_delay: Delay<u8>,
    mag_delay_v: Delay<f32>,
    dmg_delay: Delay<u8>,
    video: Vec<f32>,
    dmg: Vec<u8>,
    env_al: Vec<f32>,
    clip_al: Vec<u8>,
    mag_v: Vec<f32>,
    /// Video tap output.
    tap_buf: Option<DemodChunk>,
    video_ns: u64,
    sync_ns: u64,
}

impl Back {
    /// Restart point: clears the filters, rebuilds the alignment delays.
    fn restart(&mut self, total: usize) {
        self.lpf.reset();
        if let Some(b) = &mut self.deemph {
            b.reset();
        }
        let dv = self.lpf.delay();
        self.env_delay = Delay::new(total, 0.0);
        self.clip_delay = Delay::new(total, 0);
        self.mag_delay_v = Delay::new(dv, 0.0);
        self.dmg_delay = Delay::new(dv, damage::LOST);
    }

    fn snap(&self) -> TbcSnap {
        TbcSnap {
            levels: self.tbc.levels_hz(),
            lock: self.tbc.lock_state(),
            standard: self.tbc.standard(),
            report: self.tbc.report,
            video_ns: self.video_ns,
            sync_ns: self.sync_ns,
        }
    }

    fn process(&mut self, mid: &mut Mid, out: &mut Vec<LineRecord>) {
        let t0 = Instant::now();
        let n = mid.hz.len();
        self.tbc.steps.extend(mid.steps.iter().copied());
        self.tbc.clicks.extend(mid.clicks.iter().copied());
        if let Some(bq) = &mut self.deemph {
            for v in mid.hz.iter_mut() {
                *v = bq.run(*v as f64) as f32;
            }
        }
        self.video.resize(n, 0.0);
        self.lpf.process(&mid.hz, &mut self.video);
        self.dmg_delay.run(&mid.dmg, &mut self.dmg);
        self.env_delay.run(&mid.env, &mut self.env_al);
        self.clip_delay.run(&mid.clip, &mut self.clip_al);
        // Aligned start and warm-up marking.
        let total = mid.total as i64;
        let v_first = mid.start as i64 - total;
        let warm_end = mid.seg_start as i64 + total;
        let warm = (warm_end - v_first).clamp(0, n as i64) as usize;
        self.dmg[..warm].fill(damage::LOST);
        if let Some(mag_c) = &mid.mag_c {
            self.mag_delay_v.run(mag_c, &mut self.mag_v);
            self.mag_v.resize(n, 0.0);
            tap_push(&mut self.tap_buf, self.rx, self.fs, v_first, &self.video, &self.mag_v, &self.dmg, mid.host_ns);
        }
        let skip = (-v_first).max(0) as usize;
        if skip < n {
            let s = (v_first + skip as i64) as u64;
            self.tbc.append(s, &self.video[skip..], &self.dmg[skip..], &self.env_al[skip..], &self.clip_al[skip..]);
        }
        let t1 = Instant::now();
        self.tbc.run(LineCtx { rx: self.rx, host_ns: mid.host_ns }, out);
        self.video_ns += (t1 - t0).as_nanos() as u64;
        self.sync_ns += t1.elapsed().as_nanos() as u64;
    }
}

/// Appends demodulator output to a tap chunk (restarting it after a gap).
#[allow(clippy::too_many_arguments)]
fn tap_push(
    tap_buf: &mut Option<DemodChunk>,
    rx: RxId,
    fs: f64,
    first: i64,
    f: &[f32],
    mag: &[f32],
    dmg: &[u8],
    host_ns: u64,
) {
    let skip = (-first).max(0) as usize;
    if skip >= f.len() {
        return;
    }
    let first = (first + skip as i64) as u64;
    let buf = tap_buf.get_or_insert_with(|| DemodChunk {
        rx,
        first_sample: first,
        sample_rate_hz: fs,
        host_ns,
        freq_hz: Vec::new(),
        mag: Vec::new(),
        damage: Vec::new(),
    });
    if buf.first_sample + buf.freq_hz.len() as u64 != first {
        // Discontinuity (gap): restart the chunk.
        buf.first_sample = first;
        buf.freq_hz.clear();
        buf.mag.clear();
        buf.damage.clear();
    }
    buf.freq_hz.extend_from_slice(&f[skip..]);
    buf.mag.extend_from_slice(&mag[skip.min(mag.len())..]);
    buf.mag.resize(buf.freq_hz.len(), 0.0);
    buf.damage.extend_from_slice(&dmg[skip..]);
    buf.host_ns = host_ns;
    const CAP: usize = 1 << 23;
    if buf.freq_hz.len() > CAP {
        let cut = buf.freq_hz.len() - CAP;
        buf.freq_hz.drain(..cut);
        buf.mag.drain(..cut);
        buf.damage.drain(..cut);
        buf.first_sample += cut as u64;
    }
}

/// A finished back-half block.
struct Done {
    mid: Mid,
    lines: Vec<LineRecord>,
    snap: TbcSnap,
}

/// Locks the back half (a panic on the worker leaves the data usable).
fn lock(back: &Mutex<Back>) -> MutexGuard<'_, Back> {
    back.lock().unwrap_or_else(|e| e.into_inner())
}

/// Receives with a short spin before blocking: blocks arrive tens of
/// microseconds apart while a transfer is processed, and waking a parked
/// thread costs about as much.
fn spin_recv<T>(rx: &Receiver<T>, spin: Duration) -> Option<T> {
    let t = Instant::now();
    let mut k = 0u32;
    loop {
        match rx.try_recv() {
            Ok(v) => return Some(v),
            Err(TryRecvError::Disconnected) => return None,
            Err(TryRecvError::Empty) => {}
        }
        k = k.wrapping_add(1);
        if k.is_multiple_of(64) && t.elapsed() > spin {
            return rx.recv().ok();
        }
        std::hint::spin_loop();
    }
}

fn worker(back: Arc<Mutex<Back>>, jobs: Receiver<Mid>, done: Sender<Done>) {
    while let Some(mut mid) = spin_recv(&jobs, Duration::from_micros(300)) {
        let mut lines = Vec::new();
        let snap = {
            let mut b = lock(&back);
            b.process(&mut mid, &mut lines);
            b.snap()
        };
        if done.send(Done { mid, lines, snap }).is_err() {
            return;
        }
    }
}

/// Where the back half runs.
enum Runner {
    /// On the caller's thread, one block behind the front half.
    Inline { pending: Option<Mid> },
    /// On a worker thread, overlapped with the next front-half block.
    Thread {
        jobs: Option<Sender<Mid>>,
        done: Receiver<Done>,
        busy: bool,
        handle: Option<JoinHandle<()>>,
    },
}

/// The per-receiver DSP chain.
///
/// The chain is split in two halves: the front (unpack, front end, NCO,
/// channel filter, demodulator, click repair) and the back (video
/// low-pass, sync, TBC). Block `j` of the front runs while the back works
/// on block `j - 1`, on a worker thread when
/// [`RxDspConfig::pipeline_thread`] is set. Each call to
/// [`RxDsp::push_bytes`] drains the back half before it returns, so every
/// line of the pushed data is in `out` and the result does not depend on
/// the threading.
pub struct RxDsp {
    rx: RxId,
    cfg: RxDspConfig,
    fs: f64,
    step_i: f32,
    unpack: unpack::Unpacker,
    fe: FrontEnd,
    nco: Nco,
    chan: ChannelFilter,
    demod: Demod,
    clicks: ClickRepair,
    spectrum: Option<SpectrumTap>,
    back: Arc<Mutex<Back>>,
    runner: Runner,
    snap: TbcSnap,
    spare: Vec<Mid>,
    // Pipeline bookkeeping.
    in_index: u64,
    seg_start: u64,
    lut_path: bool,
    d_pre: usize,
    d_clicks: usize,
    d_video: usize,
    shift_hist: VecDeque<(u64, f64)>,
    mag_delay_c: Delay<f32>,
    // AFC.
    mean_hz: f64,
    mean_valid: bool,
    afc_lines: u64,
    cn_smooth_db: f32,
    // Scratch.
    re: Vec<f32>,
    im: Vec<f32>,
    words: Vec<u16>,
    envd: Vec<f32>,
    d: Vec<f32>,
    d2: Vec<f32>,
    tmp_f: Vec<f32>,
    // Tap.
    tap: Option<TapPoint>,
    tap_buf: Option<DemodChunk>,
    // Stats.
    bytes: u64,
    gaps: u64,
    lost_samples: u64,
    gain_steps: u64,
    proc_ns: u64,
    proc_samples: u64,
    load: f32,
    clicks_seen: u64,
    clicks_window: (u64, u64),
    clicks_rate: f32,
    tuned: Option<(u32, i32)>,
    stage_ns: [u64; STAGES.len()],
    stage_samples: u64,
}

/// Names of the timed stages in [`RxStats::stage_ns`]. With the pipeline
/// thread, "video" and "sync+tbc" run on the second thread.
pub const STAGES: [&str; 7] = ["frontend", "nco+channel", "demod", "clicks", "video", "sync+tbc", "other"];

impl Drop for RxDsp {
    fn drop(&mut self) {
        if let Runner::Thread { jobs, handle, .. } = &mut self.runner {
            // Closing the job channel ends the worker.
            jobs.take();
            if let Some(h) = handle.take() {
                let _ = h.join();
            }
        }
    }
}

impl RxDsp {
    pub fn new(rx: RxId, cfg: RxDspConfig) -> Self {
        let fs = cfg.sample_rate_hz();
        let lanes = &cfg.sample_format.lanes;
        let (full_scale, step_i) = match &lanes.encoding {
            LaneEncoding::Iq { i, .. } => (lanes.component_full_scale(), (1u32 << i.adc_lsb()) as f32),
            LaneEncoding::Angle { .. } => (openvrx_core::LaneFormat::ANGLE_MAGNITUDE, 1.0),
        };
        let has_mag = lanes.has_magnitude();
        let fe = FrontEnd::new(
            fs,
            has_mag,
            full_scale,
            step_i,
            cfg.dc.clone(),
            cfg.iq_correction.clone(),
            cfg.glitch.clone(),
            cfg.stats_window_ms,
        );
        let chan = ChannelFilter::new(&cfg.channel_filter, fs);
        let demod = Demod::new(
            cfg.demod,
            fs,
            Some(lanes),
            &cfg.least_squares,
            &cfg.pll,
            &cfg.ekf,
        );
        let tracking = matches!(cfg.demod, DemodKind::Pll | DemodKind::Ekf);
        let clicks = ClickRepair::new(&cfg.clicks, tracking);
        let lpf = if cfg.video_lpf.enabled {
            // Downsampling onto the grid needs the cutoff below the grid
            // Nyquist; the narrower of the configured and safe cutoff wins.
            let grid_nyq = 0.5 * 910.0 * openvrx_core::standard::NTSC.line_hz;
            let cutoff = (cfg.video_lpf.cutoff_hz as f64).min(0.9 * grid_nyq).min(0.49 * fs);
            let n = kaiser_len(cfg.video_lpf.atten_db as f64, cfg.video_lpf.transition_hz as f64 / fs, cfg.video_lpf.max_taps);
            SymFir::new(design_lowpass(n, cutoff / fs, kaiser_beta(cfg.video_lpf.atten_db as f64)))
        } else {
            SymFir::identity()
        };
        let deemph = cfg
            .deemphasis
            .enabled
            .then(|| first_order_shelf(cfg.deemphasis.pole_hz as f64, cfg.deemphasis.zero_hz as f64, fs));
        let mut tbc = Tbc::new(
            &cfg.sync,
            fs,
            cfg.standard,
            cfg.invert,
            cfg.resampler.taps,
            cfg.resampler.phases,
            if cfg.video_lpf.enabled { cfg.video_lpf.cutoff_hz as f64 } else { 0.5 * fs },
        );
        tbc.set_has_magnitude(has_mag);
        let d_video = lpf.delay();
        let back = Back {
            rx,
            fs,
            deemph,
            lpf,
            tbc,
            env_delay: Delay::new(0, 0.0),
            clip_delay: Delay::new(0, 0),
            mag_delay_v: Delay::new(0, 0.0),
            dmg_delay: Delay::new(0, 0),
            video: Vec::new(),
            dmg: Vec::new(),
            env_al: Vec::new(),
            clip_al: Vec::new(),
            mag_v: Vec::new(),
            tap_buf: None,
            video_ns: 0,
            sync_ns: 0,
        };
        let snap = back.snap();
        let back = Arc::new(Mutex::new(back));
        let runner = if cfg.pipeline_thread {
            let (jtx, jrx) = mpsc::channel::<Mid>();
            let (dtx, drx) = mpsc::channel::<Done>();
            let b = Arc::clone(&back);
            match std::thread::Builder::new()
                .name(format!("openvrx-tbc-rx{rx}"))
                .spawn(move || worker(b, jrx, dtx))
            {
                Ok(h) => Runner::Thread {
                    jobs: Some(jtx),
                    done: drx,
                    busy: false,
                    handle: Some(h),
                },
                Err(_) => Runner::Inline { pending: None },
            }
        } else {
            Runner::Inline { pending: None }
        };
        let spectrum = cfg.spectrum.enabled.then(|| SpectrumTap::new(&cfg.spectrum, fs));
        let mut s = RxDsp {
            rx,
            fs,
            step_i,
            unpack: unpack::Unpacker::new(lanes),
            fe,
            nco: Nco::new(fs),
            chan,
            demod,
            clicks,
            spectrum,
            back,
            runner,
            snap,
            spare: Vec::new(),
            in_index: 0,
            seg_start: 0,
            lut_path: false,
            d_pre: 0,
            d_clicks: 0,
            d_video,
            shift_hist: VecDeque::new(),
            mag_delay_c: Delay::new(0, 0.0),
            mean_hz: 0.0,
            mean_valid: false,
            afc_lines: 0,
            cn_smooth_db: f32::NAN,
            re: Vec::new(),
            im: Vec::new(),
            words: Vec::new(),
            envd: Vec::new(),
            d: Vec::new(),
            d2: Vec::new(),
            tmp_f: Vec::new(),
            tap: None,
            tap_buf: None,
            bytes: 0,
            gaps: 0,
            lost_samples: 0,
            gain_steps: 0,
            proc_ns: 0,
            proc_samples: 0,
            load: 0.0,
            clicks_seen: 0,
            clicks_window: (0, 0),
            clicks_rate: 0.0,
            tuned: None,
            stage_ns: [0; STAGES.len()],
            stage_samples: 0,
            cfg,
        };
        let shift = if s.cfg.afc.enabled { 0.0 } else { s.cfg.afc.manual_shift_hz as f64 };
        s.nco.set_shift(shift);
        s.shift_hist.push_back((0, shift));
        s.restart_chain(false);
        s
    }

