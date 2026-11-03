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

    fn lock(&mut self, std: VideoStandard, newest: f64, period: f64) {
        let p = std.params();
        // Levels from the newest pulses.
        let starts: Vec<f64> = self
            .pulses
            .iter()
            .rev()
            .filter(|q| q.kind == PulseKind::H)
            .take(8)
            .map(|q| q.start)
            .collect();
        let (mut tip, mut blank, mut nt, mut nb) = (0.0, 0.0, 0usize, 0usize);
        let (bw0, bw1) = self.blank_window(p);
        for &st in &starts {
            if let Some(m) = self.mean_range(st + self.us(0.25 * p.hsync_us), st + self.us(0.75 * p.hsync_us)) {
                tip += m.0;
                nt += 1;
            }
            if let Some(m) = self.mean_range(st + bw0, st + bw1) {
                blank += m.0;
                nb += 1;
            }
        }
        if nt == 0 || nb == 0 {
            return;
        }
        let (tip, blank) = (tip / nt as f64, blank / nb as f64);
        if blank - tip < 2.0e5 {
            return;
        }
        let relock = self.ever_locked;
        self.lv.tip = Scalar { x: tip, p: 1e8 };
        self.lv.blank = Scalar { x: blank, p: 1e8 };
        self.lv.valid = true;
        if self.lv.noise == 0.0 {
            self.lv.noise = 0.02 * (blank - tip);
        }
        let t = self.fine_edge(newest).map(|x| x.0).unwrap_or(newest);
        if relock {
            // Jump the running flywheel to the new train: the next line
            // starts at the train's grid point after the current line start.
            let k = ((self.t + 0.5 * self.p - t) / period).ceil();
            self.t_jump = Some(t + k * period);
            self.relock_flag = true;
            self.report.relocks += 1;
        } else {
            self.t = t;
            self.line_n = 0;
            self.vsync_checked_upto = 0;
            self.vsync_bad = 0;
            self.field_known = false;
            self.lines_since_lock = 0;
            self.slots.clear();
            self.last_vsync = None;
        }
        if self.std != Some(std) {
            self.field_known = false;
            self.slots.clear();
            self.last_vsync = None;
        }
        self.std = Some(std);
        self.p = period;
        self.cov = [0.05, 0.0, 1e-4];
        self.misses = 0;
        self.slot0 = PulseKind::H;
        self.searching = false;
        self.ever_locked = true;
        self.relock_hist.clear();
    }

    /// Blanking window after the burst, relative to 0H (input samples).
    fn blank_window(&self, p: &StandardParams) -> (f64, f64) {
        let burst_end = p.burst_start_us + p.burst_cycles / p.fsc_hz * 1e6;
        (self.us(burst_end + 0.3), self.us(p.active_start_us - 0.3))
    }

    /// Mean and variance of v over absolute [a, b).
    fn mean_range(&self, a: f64, b: f64) -> Option<(f64, f64, usize)> {
        let lo = a.ceil() as i64 - self.base as i64;
        let hi = b.floor() as i64 - self.base as i64;
        if lo < 0 || hi as usize > self.v.len() || hi - lo < 2 {
            return None;
        }
        let x = &self.v[lo as usize..hi as usize];
        let n = x.len() as f64;
        let m = x.iter().map(|&v| v as f64).sum::<f64>() / n;
        let var = x.iter().map(|&v| (v as f64 - m).powi(2)).sum::<f64>() / n;
        Some((m, var, x.len()))
    }

    /// Sub-sample 50 % crossing of a falling edge near `coarse`. Returns
    /// (absolute position, standard deviation in samples).
    fn fine_edge(&self, coarse: f64) -> Option<(f64, f64)> {
        let l50 = (0.5 * (self.lv.tip.x + self.lv.blank.x)) as f32;
        let depth = (self.lv.blank.x - self.lv.tip.x) as f32;
        let (rl, rh) = self.interp.reach();
        let c = coarse - self.base as f64;
        let span = self.us(0.6).max(2.0);
        let lo = ((c - span).floor().max(rl as f64 + 1.0)) as usize;
        let hi = ((c + span).ceil() as usize).min(self.v.len().saturating_sub(rh + 2));
        if hi <= lo {
            return None;
        }
        let mut best: Option<f64> = None;
        for j in lo..hi {
            let (a, b) = (self.v[j], self.v[j + 1]);
            if a >= l50 && b < l50 {
                let x = j as f64 + (a - l50) as f64 / (a - b) as f64;
                if best.is_none_or(|bx| (x - c).abs() < (bx - c).abs()) {
                    best = Some(x);
                }
            }
        }
        let mut t = best?;
        let j0 = t.floor();
        let mut slope = 0.0f64;
        for _ in 0..5 {
            let i = t.floor() as usize;
            let fr = t - i as f64;
            let val = (self.interp.eval(&self.v, i, fr) - l50) as f64;
            let der = self.interp.eval_deriv(&self.v, i, fr) as f64;
            slope = der;
            if der > -0.02 * depth as f64 {
                break;
            }
            let step = (val / der).clamp(-1.0, 1.0);
            t = (t - step).clamp(j0 - 1.0, j0 + 2.0);
            if step.abs() < 1e-4 {
                break;
            }
        }
        if slope > -0.02 * depth as f64 {
            return None;
        }
        let sigma = (self.lv.noise / slope.abs()).max(1e-3);
        Some((t + self.base as f64, sigma))
    }

    /// Pulse with a start inside [a, b] closest to `x`.
    fn pulse_near(&self, x: f64, w: f64) -> Option<Pulse> {
        self.pulses
            .iter()
            .filter(|p| (p.start - x).abs() <= w)
            .min_by(|a, b| (a.start - x).abs().total_cmp(&(b.start - x).abs()))
            .copied()
    }

    /// Main loop: detection, acquisition and line output.
    pub fn run(&mut self, ctx: LineCtx, out: &mut Vec<LineRecord>) {
        self.last_ctx = Some(ctx);
        out.append(&mut self.pending_out);
        self.update_box();
        self.detect();
        if self.ever_locked {
            while let Some(std) = self.std {
                let need = self.t + self.p + self.us(self.cfg.window_us as f64 + 9.0);
                if (self.scan as f64) < need || (self.end() as f64) < need {
                    break;
                }
                self.step_line(std, ctx, out);
            }
        }
        self.trim();
    }

    fn trim(&mut self) {
        let keep_from = if self.ever_locked {
            (self.t - self.us(70.0)).max(0.0) as u64
        } else {
            self.scan.saturating_sub(self.us(300.0) as u64)
        };
        let keep_from = keep_from.min(self.scan.saturating_sub(self.us(300.0) as u64 + 2));
        if keep_from > self.base + 65536 {
            let cut = (keep_from - self.base) as usize;
            let cut = cut.min(self.s.len());
            self.v.drain(..cut);
            self.s.drain(..cut);
            self.dmg.drain(..cut);
            self.env.drain(..cut);
            self.clip.drain(..cut);
            self.base += cut as u64;
        }
        let horizon = self.base as f64;
        while self.clicks.front().is_some_and(|&c| (c as f64) < horizon) {
            self.clicks.pop_front();
        }
        while self.steps.front().is_some_and(|&c| (c as f64) < horizon) {
            self.steps.pop_front();
        }
        while self.lost.front().is_some_and(|&(_, e)| (e as f64) < horizon) {
            self.lost.pop_front();
        }
    }

    /// Processes one line boundary and emits the line that ends there.
    fn step_line(&mut self, std: VideoStandard, ctx: LineCtx, out: &mut Vec<LineRecord>) {
        let par = std.params();
        let w = self.us(self.cfg.window_us as f64);
        let (qt, qp) = (
            (self.cfg.phase_noise as f64).powi(2),
            (self.cfg.period_noise as f64).powi(2),
        );
        // Predict.
        let [a, b, c] = self.cov;
        let pa = a + 2.0 * b + c + qt;
        let pb = b + c;
        let pc = c + qp;
        let t_start = self.t; // start of the line being emitted
        let pred = self.t + self.p;
        let found = self.pulse_near(pred, w).filter(|p| p.kind != PulseKind::Other);
        let mut measured = None;
        let mut innovation = f64::INFINITY;
        let mut outlier = false;
        if let Some(pu) = found {
            if let Some((m, sig)) = self.fine_edge(pu.start) {
                let r = sig * sig;
                let s = pa + r;
                let nu = m - pred;
                self.innov_sd = s.sqrt();
                let gate = (4.0 * s.sqrt()).max(self.us(0.08));
                if nu.abs() <= gate {
                    let k0 = pa / s;
                    let k1 = pb / s;
                    self.cov = [(1.0 - k0) * pa, (1.0 - k0) * pb, pc - k1 * pb];
                    self.t = pred + k0 * nu;
                    self.p += k1 * nu;
                    measured = Some(m);
                    innovation = nu;
                } else {
                    outlier = true;
                }
            }
        }
        if measured.is_none() {
            self.t = pred;
            self.cov = [pa, pb, pc];
            self.misses += 1;
            self.report.coasted_lines += 1;
        } else {
            self.misses = 0;
            if self.searching {
                self.searching = false;
            }
        }
        let pn = self.nominal_period(std);
        self.p = self.p.clamp(pn * 0.995, pn * 1.005);
        // Half-line slot of the line being emitted.
        let s1 = self
            .pulse_near(t_start + 0.5 * self.p, w)
            .map(|p| p.kind)
            .unwrap_or(PulseKind::None);
        let slot0_this = self.slot0;
        self.slot0 = found.map(|p| p.kind).unwrap_or(PulseKind::None);

        // Off-grid horizontal pulses inside the emitted line (relock).
        let off = self
            .pulses
            .iter()
            .filter(|p| p.kind == PulseKind::H && p.start > t_start + w && p.start < t_start + self.p - w)
            .map(|p| p.start - t_start)
            .next_back();
        let missing = measured.is_none() || outlier;
        self.relock_hist.push_back(if missing { off } else { None });
        let need = if self.expect_jump { 1 } else { self.cfg.relock_pulses.max(1) as usize };
        while self.relock_hist.len() > need {
            self.relock_hist.pop_front();
        }
        let mut jump_to = None;
        if self.relock_hist.len() == need && self.relock_hist.iter().all(|o| o.is_some()) {
            let os: Vec<f64> = self.relock_hist.iter().map(|o| o.unwrap()).collect();
            let spread = os.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
                - os.iter().cloned().fold(f64::INFINITY, f64::min);
            if spread < self.us(0.5) {
                let o = *os.last().unwrap();
                let base_t = t_start + o;
                jump_to = Some(if o < 0.5 * self.p { base_t + self.p } else { base_t });
            }
        }
        if let Some(jt) = self.t_jump.take() {
            jump_to = Some(jt);
        }
        let mut relock = std::mem::take(&mut self.relock_flag);
        let mut line_end = self.t;
        if let Some(jt) = jump_to {
            line_end = t_start + self.p;
            self.t = jt;
            self.cov = [0.05, 0.0, self.cov[2].max(1e-4)];
            self.relock_hist.clear();
            self.expect_jump = false;
            self.misses = 0;
            if !relock {
                self.report.relocks += 1;
            }
            relock = true;
            // The half-line pattern restarts after a jump.
            self.slots.clear();
        }
        if self.misses > self.cfg.coast_lines {
            self.searching = true;
        }
        // Emit the line [t_start, line_end).
        let lpf_frame = par.lines_per_frame;
        self.slots.push_back(Slots {
            line: self.line_n,
            s0: slot0_this,
            s1,
        });
        while self.slots.len() > 24 {
            self.slots.pop_front();
        }
        self.field_detect(std);
        let fl = self.frame_line;
        let emit = self.field_known
            || self.lines_since_lock
                > self.cfg.provisional_fields as u64 * par.lines_per_field[0] as u64;
        if emit {
            if !self.field_known {
                self.field_known = true;
            }
            let rec = self.make_line(
                std,
                ctx,
                t_start,
                line_end,
                fl,
                slot0_this,
                measured.is_some(),
                innovation,
                relock,
            );
            self.report.lines_out += 1;
            out.push(rec);
        }
        self.lines_since_lock += 1;
        self.line_n += 1;
        self.frame_line = if fl >= lpf_frame { 1 } else { fl + 1 };
        if self.frame_line == 1 || self.frame_line == par.lines_per_field[0] + 1 {
            self.field_seq += 1;
        }
        self.report.locked = !self.searching;
        self.report.line_period = self.p;
    }

    /// Matches the half-line pulse pattern around the broad pulses.
    fn field_detect(&mut self, std: VideoStandard) {
        let par = std.params();
        let (nb, ne) = match std {
            VideoStandard::Ntsc => (6i64, 6i64),
            VideoStandard::Pal => (5, 5),
        };
        // Half-line sequence of the stored lines.
        let n = self.slots.len();
        if n < 12 {
            return;
        }
        let first_line = self.slots[0].line;
        let mut seq = Vec::with_capacity(2 * n);
        for s in &self.slots {
            seq.push(s.s0);
            seq.push(s.s1);
        }
        let broad: Vec<i64> = seq
            .iter()
            .enumerate()
            .filter(|(_, k)| **k == PulseKind::Broad)
            .map(|(i, _)| i as i64)
            .collect();
        if broad.len() < 2 {
            return;
        }
        let (b0, b1) = (broad[0], *broad.last().unwrap());
        // Need the post-equalising pulses seen and not re-checked.
        if (seq.len() as i64) < b1 + ne + 2 || b1 - b0 > nb + 2 {
            return;
        }
        let vs_line_guess = first_line + (b0 / 2) as u64;
        if vs_line_guess < self.vsync_checked_upto {
            return;
        }
        let mut best = (i64::MIN, 0i64);
        for s in (b1 - nb + 1)..=b0 {
            let mut score = 0i64;
            for j in (s - ne).max(0)..(s + nb + ne).min(seq.len() as i64) {
                let expect = if j < s || j >= s + nb { PulseKind::Eq } else { PulseKind::Broad };
                let got = seq[j as usize];
                if got == PulseKind::None {
                    continue;
                }
                score += if got == expect { 1 } else { -1 };
            }
            if score > best.0 {
                best = (score, s);
            }
        }
        if best.0 < (nb + ne) / 2 {
            return;
        }
        let s = best.1;
        let parity = if s % 2 == 0 { FieldParity::First } else { FieldParity::Second };
        let vline = first_line + (s / 2) as u64;
        self.vsync_checked_upto = vline + 4;
        let vframe: u16 = match (std, parity) {
            (VideoStandard::Ntsc, FieldParity::First) => 4,
            (VideoStandard::Ntsc, FieldParity::Second) => 266,
            (VideoStandard::Pal, FieldParity::First) => 1,
            (VideoStandard::Pal, FieldParity::Second) => 313,
        };
        // Frame line of the line currently being emitted (self.line_n).
        let lpf = par.lines_per_frame as u64;
        let fl = ((vframe as u64 - 1 + (self.line_n - vline)) % lpf) as u16 + 1;
        if let Some(prev) = self.last_vsync {
            let d = vline - prev;
            let ok = d == par.lines_per_field[0] as u64 || d == par.lines_per_field[1] as u64;
            if ok {
                self.vsync_bad = 0;
                self.report.field_lines = d as u32;
            } else {
                self.vsync_bad += 1;
            }
            self.report.fields += 1;
        }
        self.last_vsync = Some(vline);
        if fl != self.frame_line || !self.field_known {
            self.frame_line = fl;
            self.field_known = true;
        }
        if self.vsync_bad >= 3 && self.std_override.is_none() {
            // Field length disagrees with the standard: search again.
            self.vsync_bad = 0;
            self.searching = true;
            self.std = None;
            self.ever_locked = false;
        }
    }

    /// Grades the damage of each segment of the line by its envelope C/N:
    /// 0 at or above `segment_clean_db`, rising linearly to `NOISY` at 0 dB.
    /// Returns the envelope statistics of the whole line.
    fn grade_segments(&self, dmg: &mut [u8], lo: usize, hi: usize, t0_local: f64, step: f64) -> EnvAcc {
        let grade = self.has_mag && self.cfg.segment_us > 0.0;
        let seg = if grade { (self.us(self.cfg.segment_us as f64) as usize).max(16) } else { hi - lo };
        let clean = self.cfg.segment_clean_db.max(1.0);
        let mut total = EnvAcc::default();
        let mut a = lo;
        while a < hi {
            let b = (a + seg).min(hi);
            let mut acc = EnvAcc::default();
            acc.add_slice(&self.env[a..b]);
            total.merge(&acc);
            if !grade || b - a < seg / 2 {
                a = b;
                continue;
            }
            let cn = acc.cn_db();
            if cn < clean {
                let g = ((clean - cn.max(0.0)) / clean * damage::NOISY as f32) as u8;
                // Grid samples whose input position falls in [a, b).
                let k0 = (((a as f64 - t0_local) / step).ceil().max(0.0)) as usize;
                let k1 = ((((b as f64) - t0_local) / step).ceil().max(0.0) as usize).min(dmg.len());
                for d in &mut dmg[k0.min(k1)..k1] {
                    *d = (*d).max(g);
                }
            }
            a = b;
        }
        total
    }

    #[allow(clippy::too_many_arguments)]
    fn make_line(
        &mut self,
        std: VideoStandard,
        ctx: LineCtx,
        t0: f64,
        t1: f64,
        frame_line: u16,
        kind0: PulseKind,
        measured: bool,
        innovation: f64,
        relock: bool,
    ) -> LineRecord {
        let par = std.params();
        let spl = par.samples_per_line;
        let step = (t1 - t0) / spl as f64;
        let mut flags = LineFlags::default();
        // Levels from this line.
        let sync_ire = par.sync_ire as f64;
        let hzpi = (self.lv.blank.x - self.lv.tip.x) / (-sync_ire);
        let q_b = (self.cfg.level_noise_ire as f64 * hzpi).powi(2);
        let neff = |n: usize| (n as f64 * self.hz_lpf_frac).max(1.0);
        let (bw0, bw1) = match kind0 {
            PulseKind::Eq => (self.us(par.eq_pulse_us + 0.6), self.us(0.5 * 1e6 / par.line_hz - 1.6)),
            _ => self.blank_window(par),
        };
        let mut noise_meas = Vec::new();
        if matches!(kind0, PulseKind::H | PulseKind::Eq) {
            if let Some((m, var, n)) = self.mean_range(t0 + bw0, t0 + bw1) {
                let r = (self.lv.noise.powi(2)).max(var) / neff(n);
                let gate = 0.15 * (self.lv.blank.x - self.lv.tip.x);
                if self.lv.blank.update(m, r, q_b, gate) {
                    noise_meas.push(var);
                }
            }
        }
        if matches!(kind0, PulseKind::H | PulseKind::Broad) {
            let w = if kind0 == PulseKind::H { par.hsync_us } else { par.broad_pulse_us };
            if let Some((m, var, n)) = self.mean_range(t0 + self.us(0.2 * w), t0 + self.us(0.8 * w)) {
                let r = (self.lv.noise.powi(2)).max(var) / neff(n);
                let gate = 0.15 * (self.lv.blank.x - self.lv.tip.x);
                if self.lv.tip.update(m, r, 0.25 * q_b, gate) {
                    noise_meas.push(var);
                }
            }
        }
        if !noise_meas.is_empty() {
            let v = noise_meas.iter().sum::<f64>() / noise_meas.len() as f64;
            self.lv.noise += 0.1 * (v.sqrt() - self.lv.noise);
        }
        let blank = self.lv.blank.x;
        let depth = (self.lv.blank.x - self.lv.tip.x).max(1.0);
        let gain = (-sync_ire / depth) as f32;
        let blank32 = blank as f32;

        // Resample.
        let mut samples = vec![0.0f32; spl];
        let mut dmg = vec![0u8; spl];
        let base = self.base as f64;
        let vlen = self.v.len();
        let tl = t0 - base;
        let (a, b) = (t0 as u64, t1.ceil() as u64);
        // Lines entirely inside a filled USB gap are blanking with every
        // sample lost: no need to resample the fill.
        let reach = self.interp.taps() as u64;
        let all_lost = self.lost.iter().any(|&(s, e)| s + reach <= a && e >= b + reach);
        if all_lost {
            dmg.fill(damage::LOST);
        } else {
            self.interp.resample(&self.v, tl, step, &mut samples, blank32);
            for s in samples.iter_mut() {
                *s = (*s - blank32) * gain;
            }
            resample_damage(&self.dmg, tl, step, &mut dmg);
        }

        // Envelope and clip statistics over the line.
        let lo = ((t0 - base).max(0.0) as usize).min(vlen);
        let hi = ((t1 - base).max(0.0) as usize).min(vlen);
        // Segment grading and the line statistics share one pass.
        let (acc, clipped, silent) = if all_lost {
            // The fill has no envelope and no clipping.
            let acc = EnvAcc {
                n: (hi - lo) as u64,
                ..EnvAcc::default()
            };
            (acc, false, true)
        } else {
            let acc = self.grade_segments(&mut dmg, lo, hi, t0 - base, step);
            let clipped = self.clip[lo..hi].iter().fold(0u8, |a, &c| a | c) != 0;
            let silent = !self.env[lo..hi].iter().fold(false, |a, &e| a | (e != 0.0));
            (acc, clipped, silent)
        };
        let ratio = acc.ratio();
        let cn_db = if !self.has_mag || silent { f32::NAN } else { acc.cn_db() };
        let no_carrier = self.has_mag && ratio.is_finite() && ratio < 1.1;
        if clipped {
            flags.insert(LineFlags::CLIPPED);
        }
        if no_carrier {
            flags.insert(LineFlags::NO_CARRIER);
            for d in &mut dmg {
                *d = (*d).max(damage::SEVERE);
            }
        }
        if !measured {
            flags.insert(LineFlags::FLYWHEEL);
        }
        if relock {
            flags.insert(LineFlags::RELOCK);
        }
        let clicks = self.clicks.iter().filter(|&&c| c >= a && c < b).count();
        if self.steps.iter().any(|&s| s >= a && s < b) {
            flags.insert(LineFlags::GAIN_STEP);
        }
        if self.lost.iter().any(|&(s, e)| s < b && e > a) || dmg.contains(&damage::LOST) {
            flags.insert(LineFlags::SAMPLES_LOST);
        }
        let parity = if frame_line <= par.lines_per_field[0] {
            FieldParity::First
        } else {
            FieldParity::Second
        };
        let field_line = (frame_line - par.field_first_frame_line(parity)) as usize;
        let first_act = par.first_active_field_line(parity);
        if field_line < first_act || field_line >= first_act + par.active_lines_per_field as usize {
            flags.insert(LineFlags::VBI);
        }

        // Noise on flat grid regions.
        let mut nsum = 0.0f64;
        let mut ncount = 0usize;
        let mut add_region = |r: std::ops::Range<usize>| {
            let x = &samples[r];
            if x.len() < 3 {
                return;
            }
            let m = x.iter().map(|&v| v as f64).sum::<f64>() / x.len() as f64;
            nsum += x.iter().map(|&v| (v as f64 - m).powi(2)).sum::<f64>();
            ncount += x.len();
        };
        add_region(par.back_porch_range());
        if kind0 != PulseKind::Eq {
            add_region(par.sync_tip_range());
        }
        let noise_ire = if ncount > 0 { (nsum / ncount as f64).sqrt() as f32 } else { f32::INFINITY };
        let timing_err = if measured { (innovation / step) as f32 } else { f32::INFINITY };
        let sync_score = if measured {
            let tip_line = self.mean_range(t0 + self.us(0.2 * par.hsync_us), t0 + self.us(0.8 * par.hsync_us));
            let depth_ok = tip_line
                .map(|(m, _, _)| ((blank - m) / depth).clamp(0.0, 1.0))
                .unwrap_or(0.0) as f32;
            let nf = 1.0 / (1.0 + (noise_ire / 10.0).powi(2));
            let sd = self.innov_sd.max(1e-3);
            let tf = (-0.5 * (innovation / (3.0 * sd)).powi(2)).exp() as f32;
            (depth_ok * nf * tf.max(0.2)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let weight = if no_carrier || !noise_ire.is_finite() {
            0.0
        } else {
            LineQuality::weight_from_noise(noise_ire) / (1.0 + 0.5 * clicks as f32)
        };
        let quality = LineQuality {
            cn_db,
            noise_ire,
            sync_score,
            clicks: clicks.min(u16::MAX as usize) as u16,
            timing_err,
            power_db: (10.0 * acc.mean().max(1e-12).log10()) as f32,
            weight,
            flags,
        };
        let field_seq = self.field_seq;
        LineRecord {
            rx: ctx.rx,
            standard: std,
            field_seq,
            parity,
            field_line: field_line as u16,
            frame_line,
            samples,
            damage: dmg,
            quality,
            // Video sample j represents input time j - 0.5 (phase difference).
            src_position: t0 - 0.5,
            src_line_len: self.p,
            host_ns: ctx.host_ns,
        }
    }
}
