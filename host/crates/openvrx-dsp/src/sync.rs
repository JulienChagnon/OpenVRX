// SPDX-License-Identifier: GPL-3.0-or-later
//! Sync separation, line flywheel, field detection and time-base
//! correction onto the standard's line-locked grid.
//!
//! Input: demodulated video in Hz at the input sample rate (delay
//! compensated, so history index `j` is input sample `j`), with per-sample
//! damage, raw |z|^2 and clip flags.
//!
//! 1. A ~1 us box filter feeds a hysteresis slicer at the midpoint of the
//!    tracked sync-tip and blanking levels. Low intervals become pulses,
//!    classified by width (equalising, horizontal, broad). While searching,
//!    the slicer level comes from percentiles of the signal.
//! 2. Acquisition: a train of horizontal pulses spaced by one (or two)
//!    nominal line periods of NTSC or PAL locks the flywheel and picks the
//!    standard (the periods differ by 0.7 %; the match tolerance is 0.25 %).
//! 3. Flywheel: a two-state Kalman filter (0H position, line period in
//!    input samples) predicts each line start. The pulse found near the
//!    prediction is timed to sub-sample precision by Newton iteration on
//!    the band-limited (windowed-sinc interpolated) video at the 50 % level
//!    between tip and blanking; the measurement variance comes from the
//!    noise and the edge slope, so at high C/N the flywheel follows each
//!    edge and at low C/N it averages many lines. Missing or rejected
//!    pulses are coasted (FLYWHEEL). Consistent off-grid pulses for a few
//!    lines force a relock (RELOCK); after `coast_lines` without pulses the
//!    acquisition search runs again while the flywheel keeps the picture
//!    running.
//! 4. Vertical: the pulse kinds at 0H and at mid-line of each line form a
//!    half-line pattern; the broad-pulse block with its equalising pulses is
//!    matched against the standard's template. The block starting at 0H
//!    means field 1 (NTSC frame line 4, PAL line 1), at mid-line field 2
//!    (NTSC line 266, PAL line 313).
//! 5. Levels: blanking (after the burst, or the flat part of equalising
//!    lines) and sync tip are measured per line and tracked by scalar
//!    Kalman filters (random walk), so the clamp follows quickly at high
//!    C/N and averages at low C/N. IRE = (f - blank) * (-sync_ire) /
//!    (blank - tip): any VTX deviation normalises to standard levels.
//! 6. Each line [0H(n), 0H(n+1)) is resampled onto the grid with a
//!    polyphase windowed-sinc kernel; damage is resampled by maximum.

use crate::cn::EnvAcc;
use crate::config::SyncConfig;
use crate::filters::InterpTable;
use openvrx_core::{
    damage, FieldParity, LineFlags, LineQuality, LineRecord, RxId, StandardParams, VideoStandard,
};
use std::collections::VecDeque;

/// Pulse classes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PulseKind {
    None,
    Eq,
    H,
    Broad,
    Other,
}

#[derive(Copy, Clone, Debug)]
struct Pulse {
    start: f64,
    end: f64,
    kind: PulseKind,
}

/// Lock state for statistics.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LockState {
    /// Never locked, or searching with no flywheel.
    Searching,
    /// Flywheel locked to sync pulses.
    Locked,
    /// Flywheel running without pulses (lost or coasting).
    Coasting,
}

#[derive(Copy, Clone, Debug)]
struct Scalar {
    x: f64,
    p: f64,
}

impl Scalar {
    fn update(&mut self, z: f64, r: f64, q: f64, gate: f64) -> bool {
        let pp = self.p + q;
        let s = pp + r;
        let nu = z - self.x;
        if nu.abs() > gate.max(5.0 * s.sqrt()) {
            self.p = pp;
            return false;
        }
        let k = pp / s;
        self.x += k * nu;
        self.p = (1.0 - k) * pp;
        true
    }
}

#[derive(Clone, Debug)]
struct Levels {
    tip: Scalar,
    blank: Scalar,
    /// Noise RMS (Hz) of the video on flat regions, smoothed.
    noise: f64,
    valid: bool,
}

/// Per-line context supplied by the receiver chain.
#[derive(Copy, Clone, Debug)]
pub struct LineCtx {
    pub rx: RxId,
    pub host_ns: u64,
}

/// Measurements exposed for AFC and statistics.
#[derive(Copy, Clone, Debug, Default)]
pub struct SyncReport {
    pub locked: bool,
    pub tip_hz: f64,
    pub blank_hz: f64,
    pub noise_hz: f64,
    pub line_period: f64,
    pub field_lines: u32,
    pub fields: u64,
    pub relocks: u64,
    pub coasted_lines: u64,
    pub lines_out: u64,
}

#[derive(Copy, Clone, Debug)]
struct Slots {
    line: u64,
    s0: PulseKind,
    s1: PulseKind,
}

/// The time-base corrector.
pub struct Tbc {
    cfg: SyncConfig,
    fs: f64,
    std_override: Option<VideoStandard>,
    invert: bool,
    interp: InterpTable,
    hz_lpf_frac: f64,
    // History.
    base: u64,
    v: Vec<f32>,
    s: Vec<f32>,
    dmg: Vec<u8>,
    env: Vec<f32>,
    clip: Vec<u8>,
    box_half: usize,
    // Detector.
    scan: u64,
    low: bool,
    cross_down: f64,
    cross_up: f64,
    pulse_start: f64,
    pending: Option<Pulse>,
    pulses: VecDeque<Pulse>,
    thr: f32,
    hyst: f32,
    acq_next: u64,
    // Levels.
    lv: Levels,
    // Flywheel.
    std: Option<VideoStandard>,
    ever_locked: bool,
    searching: bool,
    t: f64,
    p: f64,
    cov: [f64; 3],
    misses: u32,
    slot0: PulseKind,
    relock_hist: VecDeque<Option<f64>>,
    expect_jump: bool,
    relock_flag: bool,
    /// Start of the next line after a relock found by acquisition.
    t_jump: Option<f64>,
    line_n: u64,
    // Field.
    frame_line: u16,
    field_known: bool,
    lines_since_lock: u64,
    field_seq: u64,
    slots: VecDeque<Slots>,
    last_vsync: Option<u64>,
    vsync_bad: u32,
    vsync_checked_upto: u64,
    // Events (aligned input indices).
    pub clicks: VecDeque<u64>,
    pub steps: VecDeque<u64>,
    pub lost: VecDeque<(u64, u64)>,
    // Stats.
    pub report: SyncReport,
    scratch: Vec<f32>,
    /// Standard deviation of the last timing innovation (input samples).
    innov_sd: f64,
    has_mag: bool,
    /// Context of the latest `run` (for lines coasted through a gap).
    last_ctx: Option<LineCtx>,
    /// Lines coasted through a gap, emitted with the next `run`.
    pending_out: Vec<LineRecord>,
}

const MAX_PULSES: usize = 256;

dispatch! {
    /// Box filter of `2h + 1` samples: `out[k]` is the mean of
    /// `v[from + k - h ..= from + k + h]`. Summed directly, 32 outputs at a
    /// time in registers: no running-sum dependency chain and no rounding
    /// drift, a few vector adds per output.
    fn box_filter(v: &[f32], h: usize, from: usize, out: &mut [f32]) {
        const W: usize = 8;
        const R: usize = 4;
        let n = out.len();
        let len = 2 * h + 1;
        let w = 1.0 / len as f32;
        let x = &v[from - h..from + n + h];
        let full = n / (W * R) * (W * R);
        let mut k = 0;
        while k < full {
            let mut acc = [[0.0f32; W]; R];
            for d in 0..len {
                let s: &[f32; W * R] = x[k + d..k + d + W * R].try_into().expect("block");
                for (r, a) in acc.iter_mut().enumerate() {
                    for q in 0..W {
                        a[q] += s[r * W + q];
                    }
                }
            }
            let o: &mut [f32; W * R] = (&mut out[k..k + W * R]).try_into().expect("block");
            for (r, a) in acc.iter().enumerate() {
                for q in 0..W {
                    o[r * W + q] = a[q] * w;
                }
            }
            k += W * R;
        }
        for k in full..n {
            out[k] = x[k..k + len].iter().sum::<f32>() * w;
        }
    }
}

/// First index with `s[k] < t` (`below`) or `s[k] > t`, scanning in
/// vectorisable chunks.
fn first_cross(s: &[f32], t: f32, below: bool) -> Option<usize> {
    const C: usize = 32;
    let mut base = 0;
    for c in s.chunks(C) {
        let hit = if below {
            c.iter().fold(false, |a, &v| a | (v < t))
        } else {
            c.iter().fold(false, |a, &v| a | (v > t))
        };
        if hit {
            let o = if below { c.iter().position(|&v| v < t) } else { c.iter().position(|&v| v > t) };
            return o.map(|o| base + o);
        }
        base += c.len();
    }
    None
}

/// Damage at grid positions `t0 + k * step` (local indices): the worse of
/// the two neighbouring input samples, LOST outside the history. Positions
/// step in 32.32 fixed point (no float to integer conversion per sample).
fn resample_damage(src: &[u8], t0: f64, step: f64, out: &mut [u8]) {
    const FX: f64 = 4_294_967_296.0;
    let mut pos = (t0 * FX).round() as i64;
    let st = (step * FX).round() as i64;
    let last = src.len() as i64 - 1;
    for d in out.iter_mut() {
        let i = pos >> 32;
        pos += st;
        *d = if i < 0 || i >= last {
            damage::LOST
        } else {
            let iu = i as usize;
            src[iu].max(src[iu + 1])
        };
    }
}

impl Tbc {
    pub fn new(
        cfg: &SyncConfig,
        fs: f64,
        std_override: Option<VideoStandard>,
        invert: bool,
        taps: usize,
        phases: usize,
        lpf_cutoff_hz: f64,
    ) -> Tbc {
        let box_len = ((1.0e-6 * fs).round() as usize).max(3) | 1;
        Tbc {
            cfg: cfg.clone(),
            fs,
            std_override,
            invert,
            interp: InterpTable::new(taps.max(8), phases.max(64), 0.47, 7.0),
            hz_lpf_frac: (2.0 * lpf_cutoff_hz / fs).clamp(0.05, 1.0),
            base: 0,
            v: Vec::new(),
            s: Vec::new(),
            dmg: Vec::new(),
            env: Vec::new(),
            clip: Vec::new(),
            box_half: box_len / 2,
            scan: 1,
            low: false,
            cross_down: 0.0,
            cross_up: 0.0,
            pulse_start: 0.0,
            pending: None,
            pulses: VecDeque::new(),
            thr: f32::NEG_INFINITY,
            hyst: 0.0,
            acq_next: 0,
            lv: Levels {
                tip: Scalar { x: 0.0, p: 1e18 },
                blank: Scalar { x: 0.0, p: 1e18 },
                noise: 0.0,
                valid: false,
            },
            std: std_override,
            ever_locked: false,
            searching: true,
            t: 0.0,
            p: 0.0,
            cov: [0.0; 3],
            misses: 0,
            slot0: PulseKind::H,
            relock_hist: VecDeque::new(),
            expect_jump: false,
            relock_flag: false,
            t_jump: None,
            line_n: 0,
            frame_line: 1,
            field_known: false,
            lines_since_lock: 0,
            field_seq: 0,
            slots: VecDeque::new(),
            last_vsync: None,
            vsync_bad: 0,
            vsync_checked_upto: 0,
            clicks: VecDeque::new(),
            steps: VecDeque::new(),
            lost: VecDeque::new(),
            report: SyncReport::default(),
            scratch: Vec::new(),
            innov_sd: 1.0,
            has_mag: true,
            last_ctx: None,
            pending_out: Vec::new(),
        }
    }

    /// Angle-only lanes carry no envelope: C/N is unknown and the
    /// no-carrier test is skipped.
    pub fn set_has_magnitude(&mut self, has_mag: bool) {
        self.has_mag = has_mag;
    }

    pub fn standard(&self) -> Option<VideoStandard> {
        self.std
    }

    pub fn lock_state(&self) -> LockState {
        if !self.ever_locked {
            LockState::Searching
        } else if self.searching || self.misses > 8 {
            LockState::Coasting
        } else {
            LockState::Locked
        }
    }

    /// Sign applied to stored video (polarity).
    fn sign(&self) -> f32 {
        if self.invert {
            -1.0
        } else {
            1.0
        }
    }

    /// Tracked levels in absolute Hz (None before the first lock).
    pub fn levels_hz(&self) -> Option<(f64, f64)> {
        if !self.lv.valid {
            return None;
        }
        let sg = self.sign() as f64;
        Some((self.lv.tip.x * sg, self.lv.blank.x * sg))
    }

    /// Shifts the tracked levels after the carrier moved by `hz` (retune).
    pub fn shift_levels(&mut self, hz: f64) {
        let d = hz * self.sign() as f64;
        self.lv.tip.x += d;
        self.lv.blank.x += d;
    }

    /// The next pulses may be off-grid (sampling clock slip): relock on the
    /// first consistent pulse.
    pub fn expect_jump(&mut self) {
        self.expect_jump = true;
    }

    /// End of the history (absolute index of the next sample expected).
    pub fn end(&self) -> u64 {
        self.base + self.v.len() as u64
    }

    /// Appends samples starting at absolute index `start`. Samples before
    /// the current end are dropped (overlap after a pipeline restart).
    pub fn append(&mut self, start: u64, video: &[f32], dmg: &[u8], env: &[f32], clip: &[u8]) {
        let end = self.end();
        if self.v.is_empty() && start > end {
            self.base = start;
            self.scan = start + 1;
            self.acq_next = start;
        }
        let end = self.end();
        let skip = end.saturating_sub(start) as usize;
        if skip >= video.len() {
            return;
        }
        if start + (skip as u64) > end {
            // Hole: fill with lost samples.
            let hole = (start + skip as u64 - end) as usize;
            self.fill_lost(hole);
        }
        let sg = self.sign();
        self.v.extend(video[skip..].iter().map(|&x| x * sg));
        self.dmg.extend_from_slice(&dmg[skip..]);
        self.env.extend_from_slice(&env[skip..]);
        self.clip.extend_from_slice(&clip[skip..]);
    }

    fn fill_lost(&mut self, n: usize) {
        let fill = if self.lv.valid {
            self.lv.blank.x as f32
        } else {
            self.v.last().copied().unwrap_or(0.0)
        };
        let at = self.end();
        self.v.extend(std::iter::repeat_n(fill, n));
        self.dmg.extend(std::iter::repeat_n(damage::LOST, n));
        self.env.extend(std::iter::repeat_n(0.0, n));
        self.clip.extend(std::iter::repeat_n(0, n));
        match self.lost.back_mut() {
            Some(last) if last.1 == at => last.1 += n as u64,
            _ => self.lost.push_back((at, at + n as u64)),
        }
    }

    /// USB loss of `lost` samples at the current end. Short gaps are
    /// filled and coasted through; long ones restart acquisition.
    pub fn gap(&mut self, lost: u64) {
        let field = self.fs / 50.0;
        if lost as f64 > 2.0 * field || !self.ever_locked {
            let at = self.end() + lost;
            self.lost.push_back((self.end(), at));
            self.v.clear();
            self.s.clear();
            self.dmg.clear();
            self.env.clear();
            self.clip.clear();
            self.base = at;
            self.scan = at + 1;
            self.acq_next = at;
            self.low = false;
            self.pending = None;
            self.pulses.clear();
            self.ever_locked = false;
            self.searching = true;
            self.field_known = false;
            self.slots.clear();
            self.last_vsync = None;
            self.std = self.std_override;
        } else if let Some(ctx) = self.last_ctx {
            // Fill and coast through the gap in pieces, so the fill (often
            // many times the real data when USB blocks are dropped) stays
            // in cache; the lines go out with the next `run`.
            const PIECE: u64 = 16384;
            let mut out = std::mem::take(&mut self.pending_out);
            let mut left = lost;
            while left > 0 {
                let m = left.min(PIECE);
                self.fill_lost(m as usize);
                left -= m;
                if left > 0 {
                    self.run(ctx, &mut out);
                }
            }
            self.pending_out = out;
        } else {
            self.fill_lost(lost as usize);
        }
    }

    fn us(&self, us: f64) -> f64 {
        us * 1e-6 * self.fs
    }

    /// Updates the box-filtered sync signal.
    fn update_box(&mut self) {
        let h = self.box_half;
        let n = self.v.len();
        if n < 2 * h + 1 {
            return;
        }
        // s[j] valid for j in [h, n - h); positions before h copy the first.
        let from = self.s.len();
        let to = n - h;
        if from >= to {
            return;
        }
        if from < h {
            let v0 = self.v[0];
            self.s.resize(h, v0);
        }
        let from = self.s.len();
        self.s.resize(to, 0.0);
        // Inside a filled USB gap the signal is constant: copy it instead
        // of filtering (gaps can be far longer than the real data).
        let base = self.base;
        let hh = h as u64;
        let mut j = from;
        while j < to {
            let abs = base + j as u64;
            let inside = self.lost.iter().find(|&&(ls, le)| ls + hh <= abs && abs + hh < le);
            if let Some(&(_, le)) = inside {
                let end = ((le - hh - base) as usize).min(to);
                self.s[j..end].copy_from_slice(&self.v[j..end]);
                j = end;
            } else {
                let next = self
                    .lost
                    .iter()
                    .filter(|&&(ls, le)| ls + hh > abs && ls + 2 * hh < le)
                    .map(|&(ls, _)| ls + hh)
                    .min();
                let end = next.map_or(to, |a| ((a - base) as usize).min(to));
                box_filter(&self.v, h, j, &mut self.s[j..end]);
                j = end;
            }
        }
    }

    fn classify(&self, width: f64) -> PulseKind {
        let w = width / self.fs * 1e6;
        if (1.2..3.4).contains(&w) {
            PulseKind::Eq
        } else if (3.4..7.5).contains(&w) {
            PulseKind::H
        } else if (18.0..31.0).contains(&w) {
            PulseKind::Broad
        } else {
            PulseKind::Other
        }
    }

    /// Runs the slicer over new box-filtered samples.
    fn detect(&mut self) {
        let s_end = self.base + self.s.len() as u64;
        if self.scan < self.base + 1 {
            self.scan = self.base + 1;
        }
        let merge_gap = self.us(0.35);
        let base = self.base as f64;
        while self.scan < s_end {
            // Threshold for this stretch: tracked levels when locked, else
            // the percentile estimate, refreshed every half window.
            let mut seg_end = s_end;
            if !self.searching && self.lv.valid {
                let mid = 0.5 * (self.lv.tip.x + self.lv.blank.x);
                self.thr = mid as f32;
                self.hyst = (0.12 * (self.lv.blank.x - self.lv.tip.x)) as f32;
            } else {
                if self.scan >= self.acq_next {
                    self.acquire_threshold();
                }
                seg_end = seg_end.min(self.acq_next.max(self.scan + 1));
            }
            let (thr, hy) = (self.thr, self.hyst);
            let lo = (self.scan - self.base) as usize;
            let hi = (seg_end - self.base) as usize;
            if !self.low {
                let target = thr - hy;
                let Some(o) = first_cross(&self.s[lo..hi], target, true) else {
                    self.scan = seg_end;
                    if let Some(p) = self.pending {
                        if seg_end as f64 - p.end > merge_gap {
                            self.push_pulse(p);
                            self.pending = None;
                        }
                    }
                    continue;
                };
                let j = lo + o;
                // Walk back to the crossing of the threshold itself.
                let mut i = j;
                while i > 1 && self.s[i - 1] < thr {
                    i -= 1;
                }
                let (a, b) = (self.s[i - 1], self.s[i]);
                let cross = if a >= thr && a > b {
                    (i - 1) as f64 + (a - thr) as f64 / (a - b) as f64
                } else {
                    i as f64
                } + base;
                self.cross_down = cross;
                match self.pending {
                    Some(p) if cross - p.end < merge_gap => {
                        self.pulse_start = p.start;
                        self.pending = None;
                    }
                    Some(p) => {
                        self.push_pulse(p);
                        self.pending = None;
                        self.pulse_start = cross;
                    }
                    None => self.pulse_start = cross,
                }
                self.low = true;
                self.scan = self.base + j as u64 + 1;
            } else {
                let target = thr + hy;
                let Some(o) = first_cross(&self.s[lo..hi], target, false) else {
                    self.scan = seg_end;
                    continue;
                };
                let j = lo + o;
                let mut i = j;
                while i > 1 && self.s[i - 1] >= thr {
                    i -= 1;
                }
                let (a, b) = (self.s[i - 1], self.s[i]);
                let cross = if a < thr && b > a {
                    (i - 1) as f64 + (thr - a) as f64 / (b - a) as f64
                } else {
                    i as f64
                } + base;
                self.cross_up = cross;
                self.low = false;
                self.pending = Some(Pulse {
                    start: self.pulse_start,
                    end: cross,
                    kind: self.classify(cross - self.pulse_start),
                });
                self.scan = self.base + j as u64 + 1;
            }
        }
    }

    fn push_pulse(&mut self, p: Pulse) {
        let p = Pulse {
            kind: self.classify(p.end - p.start),
            ..p
        };
        if p.kind == PulseKind::Other {
            return;
        }
        self.pulses.push_back(p);
        while self.pulses.len() > MAX_PULSES {
            self.pulses.pop_front();
        }
        if self.searching && p.kind == PulseKind::H {
            self.try_acquire();
        }
    }

    /// Percentile-based slicer level while searching.
    fn acquire_threshold(&mut self) {
        let win = self.us(4.0 * 64.0) as usize;
        let j = (self.scan - self.base) as usize;
        let lo = j.saturating_sub(win);
        self.acq_next = self.scan + (win as u64 / 2).max(1);
        if j - lo < win / 2 {
            return;
        }
        self.scratch.clear();
        self.scratch.extend_from_slice(&self.s[lo..j]);
        let n = self.scratch.len();
        let k2 = n / 50;
        let p02 = *self.scratch.select_nth_unstable_by(k2, |a, b| a.total_cmp(b)).1;
        let p50 = *self.scratch.select_nth_unstable_by(n / 2, |a, b| a.total_cmp(b)).1;
        let depth = p50 - p02;
        if depth > 2.0e5 {
            self.thr = p02 + 0.3 * depth;
            self.hyst = 0.05 * depth;
        } else {
            self.thr = f32::NEG_INFINITY;
        }
    }

    fn nominal_period(&self, std: VideoStandard) -> f64 {
        self.fs / std.params().line_hz
    }

    /// Looks for a train of horizontal pulses ending at the newest one.
    fn try_acquire(&mut self) {
        let hs: Vec<f64> = self
            .pulses
            .iter()
            .filter(|p| p.kind == PulseKind::H)
            .map(|p| p.start)
            .collect();
        if hs.len() < 7 {
            return;
        }
        let stds: Vec<VideoStandard> = match self.std_override {
            Some(s) => vec![s],
            None => VideoStandard::ALL.to_vec(),
        };
        let need_lines = if self.ever_locked { 6 } else { 8 };
        for std in stds {
            let pn = self.nominal_period(std);
            let mut cur = *hs.last().unwrap();
            let mut lines = 0u32;
            let mut count = 1u32;
            let mut idx = hs.len() - 1;
            while idx > 0 && lines < need_lines + 4 {
                let mut matched = false;
                for j in (0..idx).rev() {
                    let d = cur - hs[j];
                    if d > 2.2 * pn {
                        break;
                    }
                    for k in 1..=2u32 {
                        if (d - k as f64 * pn).abs() < 0.0025 * pn * k as f64 + 1.0 {
                            lines += k;
                            count += 1;
                            cur = hs[j];
                            idx = j;
                            matched = true;
                            break;
                        }
                    }
                    if matched {
                        break;
                    }
                }
                if !matched {
                    break;
                }
            }
            if lines >= need_lines && count >= 6 {
                let newest = *hs.last().unwrap();
                let period = (newest - cur) / lines as f64;
                if (period / pn - 1.0).abs() < 0.0025 {
                    self.lock(std, newest, period);
                    return;
                }
            }
        }
    }

