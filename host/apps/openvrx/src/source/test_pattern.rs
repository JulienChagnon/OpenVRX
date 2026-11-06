// SPDX-License-Identifier: GPL-3.0-or-later
//! Built-in test source: animated patterns delivered line by line on the
//! real raster timing (59.94 fields/s NTSC, 50 fields/s PAL), with two
//! simulated receivers whose C/N fades so the OSD, the row-source strip
//! and the damage overlay have something to show.
//!
//! Each field line becomes two output rows rendered at the field's time
//! (bob), published in chunks of a few lines exactly when the last line
//! of the chunk would have left the sync stage. The display therefore
//! sees the same arrival pattern as from the live pipeline, and the
//! capture-to-present latency it measures is the display path's own.

use super::{
    ExchangeSource, FrameExchange, FrameSource, LockState, RowUpdate, RxStatus, SourceStatus,
    StatusCell, UpdateInfo, UploadStaging, Waker,
};
use crate::settings::TestPattern;
use openvrx_core::clock::now_ns;
use openvrx_core::quality::damage;
use openvrx_core::records::rgb_to_yuv;
use openvrx_core::{FieldParity, Frame, RxId, StandardParams, VideoStandard, COMBINED};
use std::f32::consts::TAU;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

/// Test source parameters.
#[derive(Clone, Debug)]
pub struct TestSourceConfig {
    pub pattern: TestPattern,
    pub standard: VideoStandard,
    /// First simulated receiver id (the source simulates two).
    pub rx_base: RxId,
    /// Channel shown in the OSD; the backend updates it on Tune.
    pub freq_mhz: Arc<AtomicU32>,
    /// Field lines per published chunk.
    pub chunk_lines: usize,
    /// Simulated sample clock (Hz) for the rate read-outs.
    pub sample_rate_hz: f64,
    /// Reference view: no simulated damage or receiver switching, just the
    /// pattern (side-by-side comparison against a live picture).
    pub clean: bool,
}

impl TestSourceConfig {
    pub fn new(pattern: TestPattern, standard: VideoStandard, rx_base: RxId) -> Self {
        TestSourceConfig {
            pattern,
            standard,
            rx_base,
            freq_mhz: Arc::new(AtomicU32::new(5658)),
            chunk_lines: 8,
            sample_rate_hz: 240e6 / 18.0,
            clean: false,
        }
    }
}

// ---------------------------------------------------------------------
// Pattern rendering (pure)
// ---------------------------------------------------------------------

/// 75 % bars: white, yellow, cyan, green, magenta, red, blue.
const BARS: [[f32; 3]; 7] = [
    [0.75, 0.75, 0.75],
    [0.75, 0.75, 0.0],
    [0.0, 0.75, 0.75],
    [0.0, 0.75, 0.0],
    [0.75, 0.0, 0.75],
    [0.75, 0.0, 0.0],
    [0.0, 0.0, 0.75],
];

fn frac(x: f32) -> f32 {
    x - x.floor()
}

/// Triangle wave 0..1..0 with period 1.
fn tri(x: f32) -> f32 {
    let f = frac(x);
    if f < 0.5 {
        2.0 * f
    } else {
        2.0 - 2.0 * f
    }
}

fn bars_rgb(nx: f32, ny: f32, t: f32) -> [f32; 3] {
    if ny < 0.67 {
        let k = (frac(nx + t * 0.1) * 7.0) as usize;
        BARS[k.min(6)]
    } else if ny < 0.75 {
        const REV: [[f32; 3]; 7] = [
            [0.0, 0.0, 0.75],
            [0.0, 0.0, 0.0],
            [0.75, 0.0, 0.75],
            [0.0, 0.0, 0.0],
            [0.0, 0.75, 0.75],
            [0.0, 0.0, 0.0],
            [0.75, 0.75, 0.75],
        ];
        REV[((nx * 7.0) as usize).min(6)]
    } else {
        let g = nx.clamp(0.0, 1.0);
        [g, g, g]
    }
}

fn zone_plate(nx: f32, ny: f32, t: f32) -> f32 {
    let cx = 0.5 + 0.08 * (0.5 * t).cos();
    let cy = 0.5 + 0.08 * (0.5 * t).sin();
    let dx = (nx - cx) * (4.0 / 3.0);
    let dy = ny - cy;
    let r2 = dx * dx + dy * dy;
    0.5 + 0.45 * (TAU * 140.0 * r2 - 4.0 * t).cos()
}

/// Bouncing box positions (top-left, normalised) at time t.
fn box_pos(t: f32, w: f32, h: f32, sx: f32, sy: f32) -> (f32, f32) {
    (tri(t * sx) * (1.0 - w), tri(t * sy) * (1.0 - h))
}

fn in_rect(nx: f32, ny: f32, x: f32, y: f32, w: f32, h: f32) -> bool {
    nx >= x && nx < x + w && ny >= y && ny < y + h
}

fn moving_box_rgb(nx: f32, ny: f32, t: f32) -> [f32; 3] {
    let (bx, by) = box_pos(t, 0.18, 0.24, 0.23, 0.31);
    if in_rect(nx, ny, bx, by, 0.18, 0.24) {
        return [0.9, 0.9, 0.9];
    }
    let (rx, ry) = box_pos(t + 1.7, 0.1, 0.13, 0.37, 0.19);
    if in_rect(nx, ny, rx, ry, 0.1, 0.13) {
        return [0.8, 0.1, 0.1];
    }
    let grid = frac(nx * 8.0) < 0.012 || frac(ny * 6.0) < 0.016;
    let g = if grid { 0.45 } else { 0.22 };
    [g, g, g]
}

fn sweep_rgb(nx: f32, ny: f32, t: f32) -> [f32; 3] {
    let x = frac(t * 1.0);
    if (nx - x).abs() < 0.012 {
        return [1.0, 1.0, 1.0];
    }
    if frac(ny * 8.0) < 0.01 {
        return [0.35, 0.35, 0.35];
    }
    [0.05, 0.05, 0.05]
}

/// The bench ESP32's patterns in IRE (black 7.5, white 100), on its grid of
/// 106 picture samples by 240 field rows: the same function as
/// `ntsc_pattern_ire` in tools/bench/att_ctl/ntsc_frame.c.
fn bench_ire(pattern: TestPattern, x: i32, row: i32) -> f32 {
    const W: i32 = 106;
    const BLACK: f32 = 7.5;
    const WHITE: f32 = 100.0;
    let grey_step = |steps: i32| BLACK + (WHITE - BLACK) * (x * steps / W) as f32 / (steps - 1) as f32;
    let ramp = || BLACK + (WHITE - BLACK) * x as f32 / (W - 1) as f32;
    let check = |r: i32| if ((x / 8) + (r / 20)) & 1 == 1 { WHITE } else { BLACK };
    let stripes = || {
        let half = 1 << (x * 4 / W);
        if (x / half) & 1 == 1 { WHITE } else { BLACK }
    };
    match pattern {
        TestPattern::BenchCard => match row {
            0..=59 => grey_step(8),
            60..=119 => stripes(),
            120..=179 => ramp(),
            _ => check(row - 180),
        },
        TestPattern::BenchBars => grey_step(8),
        TestPattern::BenchRamp => ramp(),
        TestPattern::BenchCheck => check(row),
        TestPattern::BenchGrey => 50.0,
        TestPattern::BenchBlack => BLACK,
        _ => WHITE,
    }
}

/// The resistor-ladder ESP32's colour patterns as the decoder should show
/// them (R'G'B' 0..1, black 0): the same layout as `picture_ire` and
/// `band_of` in tools/bench/att_ctl/ladder.cpp, on its 754 active samples
/// (4 x fsc) by 240 field rows.
fn ladder_rgb(pattern: TestPattern, nx: f32, ny: f32) -> [f32; 3] {
    const ACTW: i32 = 754;
    // 75 % bars: white, yellow, cyan, green, magenta, red, blue.
    const BARS: [[f32; 3]; 7] = [
        [0.75, 0.75, 0.75],
        [0.75, 0.75, 0.0],
        [0.0, 0.75, 0.75],
        [0.0, 0.75, 0.0],
        [0.75, 0.0, 0.75],
        [0.75, 0.0, 0.0],
        [0.0, 0.0, 0.75],
    ];
    const MULTI_MHZ: [f32; 5] = [0.5, 1.0, 2.0, 3.0, 4.0];
    let x = ((nx * ACTW as f32) as i32).clamp(0, ACTW - 1);
    let row = ((ny * 240.0) as i32).clamp(0, 239);
    let bars = || BARS[(x * 7 / ACTW) as usize];
    let grey = |y: f32| [y, y, y];
    match (pattern, row) {
        (TestPattern::BenchColourBars, _) | (_, 0..=119) => bars(),
        (_, 120..=159) => {
            let zone = (x * 5 / ACTW) as usize;
            let cyc = MULTI_MHZ[zone] * 1e6 / (4.0 * 3_579_545.45);
            grey(0.5 + 0.4 * (2.0 * std::f32::consts::PI * cyc * x as f32).sin())
        }
        (_, 160..=199) => grey(x as f32 / (ACTW - 1) as f32),
        _ => grey(0.5),
    }
}

/// One pattern pixel as RGB 0..1 (gamma coded).
pub fn pattern_rgb(pattern: TestPattern, nx: f32, ny: f32, t: f32) -> [f32; 3] {
    match pattern {
        TestPattern::BenchColourCard | TestPattern::BenchColourBars => ladder_rgb(pattern, nx, ny),
        TestPattern::BenchCard
        | TestPattern::BenchBars
        | TestPattern::BenchRamp
        | TestPattern::BenchCheck
        | TestPattern::BenchGrey
        | TestPattern::BenchBlack
        | TestPattern::BenchWhite => {
            let x = ((nx * 106.0) as i32).clamp(0, 105);
            let row = ((ny * 240.0) as i32).clamp(0, 239);
            // Picture level 0..1 from black to white, as the decoder shows it.
            let l = ((bench_ire(pattern, x, row) - 7.5) / 92.5).clamp(0.0, 1.0);
            [l, l, l]
        }
        TestPattern::Bars => bars_rgb(nx, ny, t),
        TestPattern::ZonePlate => {
            let l = zone_plate(nx, ny, t);
            [l, l, l]
        }
        TestPattern::MovingBox => moving_box_rgb(nx, ny, t),
        TestPattern::Sweep => sweep_rgb(nx, ny, t),
        TestPattern::Mix => {
            let (bx, by) = box_pos(t, 0.12, 0.16, 0.21, 0.29);
            if in_rect(nx, ny, bx, by, 0.12, 0.16) {
                return [0.95, 0.95, 0.95];
            }
            if ny < 0.5 {
                bars_rgb(nx, ny * 1.34, t)
            } else if nx < 0.5 {
                let l = zone_plate(nx * 2.0, (ny - 0.5) * 2.0, t);
                [l, l, l]
            } else if ny < 0.75 {
                let g = (nx - 0.5) * 2.0;
                [g, g, g]
            } else {
                sweep_rgb((nx - 0.5) * 2.0, (ny - 0.75) * 4.0, t)
            }
        }
    }
}

/// Renders one row of `width` pixels into Y/U/V slices.
pub fn render_row(pattern: TestPattern, t: f32, ny: f32, y: &mut [f32], u: &mut [f32], v: &mut [f32]) {
    let width = y.len();
    let inv = 1.0 / width as f32;
    for x in 0..width {
        let nx = (x as f32 + 0.5) * inv;
        let [r, g, b] = pattern_rgb(pattern, nx, ny, t);
        let (yy, uu, vv) = rgb_to_yuv(r, g, b);
        y[x] = yy;
        u[x] = uu;
        v[x] = vv;
    }
}

/// Frame size of the active picture for a standard.
pub fn frame_size(standard: VideoStandard) -> (usize, usize) {
    let p = standard.params();
    (p.active_width(), p.frame_height())
}

/// Renders a whole progressive frame at time `t` (for snapshots).
pub fn render_frame(pattern: TestPattern, standard: VideoStandard, t: f32) -> Frame {
    let (w, h) = frame_size(standard);
    let mut f = Frame::new(w, h);
    for row in 0..h {
        let ny = (row as f32 + 0.5) / h as f32;
        let s = row * w..(row + 1) * w;
        let (y, u, v) = (&mut f.y[s.clone()], &mut f.u[s.clone()], &mut f.v[s]);
        render_row(pattern, t, ny, y, u, v);
    }
    f.meta.standard = Some(standard);
    f.meta.pipeline = "test pattern".into();
    f
}

// ---------------------------------------------------------------------
// Simulated receivers (pure)
// ---------------------------------------------------------------------

/// C/N of the two simulated receivers at time t (dB).
pub fn simulated_cn(t: f32, phase: f32) -> [f32; 2] {
    [
        17.0 + 9.0 * (TAU * t / 9.0 + phase).sin(),
        15.0 + 10.0 * (TAU * t / 6.0 + 1.3 + phase).sin(),
    ]
}

/// Receiver feeding a field line: the better one, or a blend when close.
pub fn simulated_row_source(cn: [f32; 2], field_line: usize, t: f32, rx_base: RxId) -> RxId {
    let diff = cn[0] - cn[1] + 2.5 * (field_line as f32 * 0.05 + t * 3.0).sin();
    if diff.abs() < 1.0 {
        COMBINED
    } else if diff > 0.0 {
        rx_base
    } else {
        rx_base + 1
    }
}

/// Cheap integer hash for reproducible damage sprinkles.
fn hash(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^ (x >> 16)
}

/// Probability (0..1) that a sample is flagged at the given C/N.
pub fn damage_probability(cn_db: f32) -> f32 {
    ((9.0 - cn_db) / 40.0).clamp(0.0, 0.25)
}

fn fill_damage(row: &mut [u8], prob: f32, seed: u32) {
    if prob <= 0.0 {
        row.iter_mut().for_each(|d| *d = damage::NONE);
        return;
    }
    let thresh = (prob * u32::MAX as f32) as u32;
    for (x, d) in row.iter_mut().enumerate() {
        *d = if hash(seed ^ (x as u32).wrapping_mul(0x9e37_79b9)) < thresh {
            damage::CLICK
        } else {
            damage::NONE
        };
    }
}

// ---------------------------------------------------------------------
// Source with worker thread
// ---------------------------------------------------------------------

struct Worker {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Test pattern [`FrameSource`].
pub struct TestPatternSource {
    inner: ExchangeSource,
}

impl TestPatternSource {
    pub fn start(config: TestSourceConfig) -> TestPatternSource {
        let exchange = FrameExchange::new();
        let status = Arc::new(StatusCell::new(SourceStatus {
            label: format!("TEST {}", config.pattern.name()),
            standard: Some(config.standard),
            lock: LockState::Searching,
            simulated: true,
            ..SourceStatus::default()
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let label = format!("TEST {} {}", config.pattern.name(), config.standard.name());
        let thread = {
            let exchange = Arc::clone(&exchange);
            let status = Arc::clone(&status);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name(format!("test-source-{}", config.rx_base / 2))
                .spawn(move || run(config, exchange, status, stop))
                .ok()
        };
        if thread.is_none() {
            status.publish(|s| s.warnings.push("could not start the test thread".into()));
        }
        let worker = Worker { stop, thread };
        TestPatternSource {
            inner: ExchangeSource::new(label, exchange, status, Some(Box::new(worker))),
        }
    }
}

impl FrameSource for TestPatternSource {
    fn label(&self) -> &str {
        self.inner.label()
    }
    fn set_waker(&mut self, waker: Waker) {
        self.inner.set_waker(waker);
    }
    fn poll(&mut self, staging: &mut UploadStaging) -> Option<UpdateInfo> {
        self.inner.poll(staging)
    }
    fn status(&mut self, out: &mut SourceStatus) -> bool {
        self.inner.status(out)
    }
    fn set_wake_lines(&mut self, lines: u32) {
        self.inner.set_wake_lines(lines);
    }
}

/// Field schedule on the raster: when each field starts and when its
/// k-th visible line is complete, in ns from the stream start.
#[derive(Clone, Debug)]
pub struct RasterClock {
    line_ns: f64,
    params: &'static StandardParams,
}

impl RasterClock {
    pub fn new(standard: VideoStandard) -> RasterClock {
        let params = standard.params();
        RasterClock {
            line_ns: 1e9 / params.line_hz,
            params,
        }
    }

    /// Parity of field number `n` (field 0 is the standard's first field).
    pub fn parity(&self, n: u64) -> FieldParity {
        if n.is_multiple_of(2) {
            FieldParity::First
        } else {
            FieldParity::Second
        }
    }

    /// Start of field `n`.
    pub fn field_start_ns(&self, n: u64) -> f64 {
        let frame_lines = self.params.lines_per_frame as f64;
        let first = self.params.lines_per_field[0] as f64;
        let frames = (n / 2) as f64;
        let extra = if n % 2 == 1 { first } else { 0.0 };
        (frames * frame_lines + extra) * self.line_ns
    }

    /// Time the k-th visible line (0-based) of field `n` is complete.
    pub fn line_done_ns(&self, n: u64, k: usize) -> f64 {
        let first = self.params.first_active_field_line(self.parity(n));
        self.field_start_ns(n) + (first + k + 1) as f64 * self.line_ns
    }
}

fn sleep_until(target_ns: u64, stop: &AtomicBool) -> bool {
    loop {
        if stop.load(Ordering::Relaxed) {
            return false;
        }
        let now = now_ns();
        if now >= target_ns {
            return true;
        }
        let wait = target_ns - now;
        // Sleep in slices so a stop request is noticed quickly.
        std::thread::sleep(Duration::from_nanos(wait.min(20_000_000)));
    }
}

fn run(
    config: TestSourceConfig,
    exchange: Arc<FrameExchange>,
    status: Arc<StatusCell<SourceStatus>>,
    stop: Arc<AtomicBool>,
) {
    let p = config.standard.params();
    let (w, h) = frame_size(config.standard);
    let lines = p.active_lines_per_field as usize;
    let chunk = config.chunk_lines.clamp(1, lines);
    let clock = RasterClock::new(config.standard);
    let mut frame = Frame::new(w, h);
    frame.meta.standard = Some(config.standard);
    frame.meta.pipeline = "test bob".into();
    let mut producer = exchange.producer();
    let phase = config.rx_base as f32 * 0.9;
    let mut start = now_ns();
    let mut field: u64 = 0;
    let mut frame_seq: u64 = 0;
    let mut last_status_ns = 0u64;
    let mut rows_by_rx = [0u32; 3];
    let mut fields_in_window = 0u32;
    let mut window_start = start;
    let mut field_rate = p.field_hz as f32;

    while !stop.load(Ordering::Relaxed) {
        let parity = clock.parity(field);
        let t = (clock.field_start_ns(field) * 1e-9) as f32;
        let cn = simulated_cn(t, phase);
        let best_cn = cn[0].max(cn[1]);
        let prob = if config.clean { 0.0 } else { damage_probability(best_cn) };
        let mut k0 = 0;
        while k0 < lines {
            let k1 = (k0 + chunk).min(lines);
            let due = start + clock.line_done_ns(field, k1 - 1) as u64;
            if !sleep_until(due, &stop) {
                return;
            }
            for k in k0..k1 {
                let src = if config.clean {
                    config.rx_base
                } else {
                    simulated_row_source(cn, k, t, config.rx_base)
                };
                let slot = if src == COMBINED {
                    2
                } else {
                    (src - config.rx_base) as usize
                };
                for row in [2 * k, 2 * k + 1] {
                    let ny = (row as f32 + 0.5) / h as f32;
                    let s = row * w..(row + 1) * w;
                    render_row(
                        config.pattern,
                        t,
                        ny,
                        &mut frame.y[s.clone()],
                        &mut frame.u[s.clone()],
                        &mut frame.v[s.clone()],
                    );
                    let seed = hash((field as u32).wrapping_mul(7919) ^ row as u32);
                    fill_damage(&mut frame.damage[s], prob, seed);
                    frame.row_source[row] = src;
                    rows_by_rx[slot] += 1;
                }
            }
            let field_done = k1 == lines;
            let frame_done = field_done && parity == p.top_field.other();
            frame.meta.field_seq = field;
            frame.meta.parity = Some(parity);
            frame.meta.seq = frame_seq;
            frame.meta.newest_line_ns = now_ns();
            frame.meta.ready_ns = frame.meta.newest_line_ns;
            producer.publish(
                &frame,
                RowUpdate {
                    rows: 2 * k0..2 * k1,
                    lines: (k1 - k0) as u32,
                    field_done,
                    frame_done,
                    newest_line_ns: 0,
                },
            );
            if frame_done {
                frame_seq += 1;
            }
            k0 = k1;
        }
        field += 1;
        fields_in_window += 1;

        let now = now_ns();
        // Resynchronise after a stall (debugger, sleep) instead of
        // bursting to catch up.
        let next_due = start + clock.line_done_ns(field, 0) as u64;
        if now > next_due + 100_000_000 {
            start = now - clock.field_start_ns(field) as u64;
        }
        if now - last_status_ns >= 100_000_000 {
            let secs = (now - window_start) as f32 * 1e-9;
            if secs > 0.5 {
                field_rate = fields_in_window as f32 / secs;
                fields_in_window = 0;
                window_start = now;
            }
            let total = rows_by_rx.iter().sum::<u32>().max(1) as f32;
            let freq = config.freq_mhz.load(Ordering::Relaxed);
            let mk = |i: usize, c: f32| RxStatus {
                rx: config.rx_base + i as RxId,
                cn_db: c,
                power_db: 20.0 + c,
                gain: 64,
                gain_on_c5: false,
                sync_locked: c > 6.0,
                freq_mhz: freq,
                sample_rate_hz: config.sample_rate_hz,
                usb_bytes_per_s: config.sample_rate_hz * 2.0,
                dropped_samples: 0,
                dropped_transfers: 0,
                share: (rows_by_rx[i] as f32 + 0.5 * rows_by_rx[2] as f32) / total,
            };
            let lock = if best_cn > 8.0 {
                LockState::Locked
            } else {
                LockState::Flywheel
            };
            status.publish(|s| {
                s.lock = lock;
                s.freq_mhz = Some(freq);
                s.field_rate_hz = field_rate;
                s.pipeline = "test bob".into();
                s.receivers.clear();
                s.receivers.push(mk(0, cn[0]));
                s.receivers.push(mk(1, cn[1]));
            });
            rows_by_rx = [0; 3];
            last_status_ns = now;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bars_are_75_percent() {
        let [r, g, b] = pattern_rgb(TestPattern::Bars, 0.01, 0.1, 0.0);
        assert_eq!([r, g, b], [0.75, 0.75, 0.75]);
        let [r, g, b] = pattern_rgb(TestPattern::Bars, 6.5 / 7.0, 0.1, 0.0);
        assert_eq!([r, g, b], [0.0, 0.0, 0.75]);
        // Bars scroll with time.
        assert_ne!(
            pattern_rgb(TestPattern::Bars, 0.01, 0.1, 0.0),
            pattern_rgb(TestPattern::Bars, 0.01, 0.1, 2.0)
        );
    }

    #[test]
    fn bench_card_matches_the_esp32() {
        // Points of ntsc_pattern_ire() in tools/bench/att_ctl/ntsc_frame.c.
        let c = |x, row| bench_ire(TestPattern::BenchCard, x, row);
        assert_eq!(c(0, 0), 7.5); // grey steps
        assert_eq!(c(105, 0), 100.0);
        assert_eq!(c(0, 60), 7.5); // 1 MHz stripes alternate every sample
        assert_eq!(c(1, 60), 100.0);
        assert_eq!(c(80, 60), 7.5); // 0.13 MHz zone: 8 black, 8 white
        assert_eq!(c(88, 60), 100.0);
        assert_eq!(c(0, 120), 7.5); // ramp
        assert_eq!(c(105, 120), 100.0);
        assert_eq!(c(0, 180), 7.5); // checkerboard, cells 8 x 20
        assert_eq!(c(8, 180), 100.0);
        assert_eq!(c(0, 200), 100.0);
        assert_eq!(bench_ire(TestPattern::BenchGrey, 50, 100), 50.0);
        assert_eq!(TestPattern::bench("card"), Some(TestPattern::BenchCard));
        assert_eq!(TestPattern::bench("nope"), None);
    }

    #[test]
    fn colour_card_matches_the_ladder() {
        // Layout of band_of()/picture_ire() in tools/bench/att_ctl/ladder.cpp.
        let c = |nx, row: f32| pattern_rgb(TestPattern::BenchColourCard, nx, (row + 0.5) / 240.0, 0.0);
        assert_eq!(c(0.01, 0.0), [0.75, 0.75, 0.75]); // white bar
        assert_eq!(c(0.99, 119.0), [0.0, 0.0, 0.75]); // blue bar
        let m = c(0.3, 140.0)[0]; // multiburst stays within 0.1..0.9
        assert!((0.1..=0.9).contains(&m));
        assert!(c(0.0, 180.0)[0] < 0.01 && c(0.999, 180.0)[0] > 0.99); // ramp
        assert_eq!(c(0.5, 220.0), [0.5, 0.5, 0.5]); // grey
        // Colour bars fill the whole field.
        assert_eq!(pattern_rgb(TestPattern::BenchColourBars, 0.78, 0.9, 0.0), [0.75, 0.0, 0.0]); // red, bar 5
        assert_eq!(TestPattern::bench_on("card", "ladder"), Some(TestPattern::BenchColourCard));
        assert_eq!(TestPattern::bench_on("card", "dac"), Some(TestPattern::BenchCard));
        assert_eq!(TestPattern::bench_on("ramp", "ladder"), Some(TestPattern::BenchRamp));
    }

    #[test]
    fn patterns_stay_in_range() {
        for p in TestPattern::ALL {
            for i in 0..50 {
                for j in 0..50 {
                    let c = pattern_rgb(p, i as f32 / 50.0, j as f32 / 50.0, 1.234);
                    assert!(c.iter().all(|x| (0.0..=1.0).contains(x)), "{p:?} {c:?}");
                }
            }
        }
    }

    #[test]
    fn frame_render_matches_rows() {
        let f = render_frame(TestPattern::Bars, VideoStandard::Ntsc, 0.0);
        assert_eq!((f.width, f.height), (754, 480));
        // Top-left is the white bar: Y = 0.75, no chroma.
        assert!((f.y[0] - 0.75).abs() < 1e-5);
        assert!(f.u[0].abs() < 1e-5 && f.v[0].abs() < 1e-5);
        let (w, h) = frame_size(VideoStandard::Pal);
        assert_eq!((w, h), (922, 576));
    }

    #[test]
    fn raster_clock_field_rates() {
        for std in VideoStandard::ALL {
            let c = RasterClock::new(std);
            let p = std.params();
            let two_fields = c.field_start_ns(2) * 1e-9;
            assert!((two_fields - 2.0 / p.field_hz).abs() < 1e-9, "{std:?}");
            assert!(c.line_done_ns(0, 0) < c.line_done_ns(0, 1));
            assert!(c.line_done_ns(0, 239) < c.field_start_ns(1));
            assert!(c.line_done_ns(1, 0) > c.field_start_ns(1));
        }
    }

    #[test]
    fn row_source_follows_cn() {
        assert_eq!(simulated_row_source([25.0, 5.0], 0, 0.0, 4), 4);
        assert_eq!(simulated_row_source([5.0, 25.0], 0, 0.0, 4), 5);
        assert_eq!(simulated_row_source([10.0, 10.0], 0, 0.0, 4), COMBINED);
        assert_eq!(damage_probability(20.0), 0.0);
        assert!(damage_probability(3.0) > 0.1);
    }

    #[test]
    fn damage_density() {
        let mut row = vec![0u8; 10_000];
        fill_damage(&mut row, 0.1, 42);
        let n = row.iter().filter(|&&d| d == damage::CLICK).count();
        assert!((800..1200).contains(&n), "{n}");
        fill_damage(&mut row, 0.0, 42);
        assert!(row.iter().all(|&d| d == 0));
    }

    #[test]
    fn source_streams_rows() {
        let cfg = TestSourceConfig {
            chunk_lines: 16,
            ..TestSourceConfig::new(TestPattern::Mix, VideoStandard::Pal, 0)
        };
        let mut src = TestPatternSource::start(cfg);
        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let tx = std::sync::Mutex::new(tx);
        src.set_waker(Arc::new(move || {
            let _ = tx.lock().unwrap().send(());
        }));
        let mut st = UploadStaging::default();
        let mut fields = 0;
        let mut rows = 0;
        let mut first = true;
        let t0 = std::time::Instant::now();
        while fields < 3 && t0.elapsed() < Duration::from_secs(5) {
            let _ = rx.recv_timeout(Duration::from_millis(100));
            while let Some(info) = src.poll(&mut st) {
                if first {
                    assert!(info.resized);
                    first = false;
                }
                assert!(info.newest_line_ns > 0);
                fields += info.fields_completed;
                rows += info.rows;
            }
        }
        assert!(fields >= 3, "only {fields} fields");
        assert!(rows > 0);
        assert_eq!((st.width, st.height), (922, 576));
        let mut status = SourceStatus::default();
        assert!(src.status(&mut status));
        assert!(status.simulated);
    }
}
