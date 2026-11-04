# SPDX-License-Identifier: GPL-3.0-or-later
"""Reference decoder: lane words to time-base-corrected lines, fields, frames.

Chain (one receiver)::

    unpack (lane LUT) -> DC removal (moving mean, I/Q) -> wrap centre
    -> optional digital channel filter -> exact polar discriminator
       angle(z[n] conj z[n-1]) with the wrap point placed in the empty part
       of the spectrum -> wrap/click repair -> video low-pass
    -> sync slicer, sub-sample 0H edges -> line flywheel + local robust fit
    -> field detection from broad pulses -> frame line numbers
    -> resample every line onto the grid (910 / 1135 per line, 0 at 0H)
    -> IRE from tracked sync tip and blanking levels -> line records
    -> fields, frames, colour (colour.py), PNG / MP4 through ffmpeg

Correctness over speed; inner loops use numba.
"""
from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from numba import njit
from scipy.ndimage import median_filter, uniform_filter1d

from . import dsp
from .standards import FIRST, NTSC, PAL, SECOND, StandardParams, get_standard

# Damage values (openvrx_core::quality::damage)
D_NONE, D_NOISY, D_REPAIRED, D_CLICK, D_SEVERE, D_LOST = 0, 64, 96, 160, 224, 255
# Line flags (openvrx_core::quality::LineFlags)
F_FLYWHEEL, F_NO_CARRIER, F_VBI = 1 << 0, 1 << 2, 1 << 6


@dataclass
class DecoderConfig:
    standard: Optional[str] = None          # None: sidecar hint, else line period
    dc_window_lines: float = 16.0
    channel_filter_hz: Optional[float] = None  # complex half-bandwidth; 0 off; None auto
    click_threshold_hz: float = 4.5e6       # 0 disables click replacement
    wrap_repair: bool = True
    video_lpf_hz: Optional[float] = None    # None: 4.2 MHz NTSC, 5.0 MHz PAL
    sync_lpf_hz: float = 1.0e6
    timing_lpf_hz: float = 2.5e6
    tbc_half_window: int = 8
    level_smooth_lines: int = 31
    centre_cycles: Optional[float] = None   # wrap centre override (cycles/sample)
    demod: str = "polar"                    # polar | pll
    pll_bw_hz: float = 3.0e6                # PLL natural frequency (zeta 0.707)

    def channel_filter_for(self, fs: float) -> float:
        if self.channel_filter_hz is not None:
            return float(self.channel_filter_hz)
        # The digital channel filter only helps when the sample rate exceeds
        # the signal bandwidth (decided by experiment S0).
        return 7.5e6 if fs >= 19e6 else 0.0


# ---------------------------------------------------------------------------
# Demodulation
# ---------------------------------------------------------------------------

@njit(cache=True)
def _discriminate(zr, zi, centre):
    n = len(zr)
    d = np.empty(n)
    d[0] = centre
    inv = 1.0 / (2.0 * np.pi)
    for k in range(1, n):
        re = zr[k] * zr[k - 1] + zi[k] * zi[k - 1]
        im = zi[k] * zr[k - 1] - zr[k] * zi[k - 1]
        a = np.arctan2(im, re) * inv - centre
        a -= np.floor(a + 0.5)
        d[k] = a + centre
    return d


@njit(cache=True)
def _pll(zr, zi, wn_t, zeta, f0):
    """Second-order PLL, arctangent phase detector, one update per sample.

    Returns the loop's frequency estimate plus the proportional term in
    cycles/sample (the demodulated FM). ``f0`` is the start frequency."""
    n = len(zr)
    out = np.empty(n)
    kp = 2.0 * zeta * wn_t
    ki = wn_t * wn_t
    th = 0.0
    w = 2.0 * np.pi * f0
    inv = 1.0 / (2.0 * np.pi)
    for k in range(n):
        c, s_ = np.cos(th), np.sin(th)
        re = zr[k] * c + zi[k] * s_
        im = zi[k] * c - zr[k] * s_
        e = np.arctan2(im, re)
        w += ki * e
        step = w + kp * e
        out[k] = step * inv
        th += step
        if th > np.pi:
            th -= 2.0 * np.pi
        elif th < -np.pi:
            th += 2.0 * np.pi
    return out


@njit(cache=True)
def _repair(d, click_thr, wrap, span):
    """Wrap and click repair on discriminator output (cycles/sample).

    A noise click adds a full turn of phase within about one symbol of the
    analog channel (~75 ns). ``span`` samples cover that time at this rate.
    The residual against a local median (7 spans wide, the tested group
    excluded) is summed over each group of ``span`` samples; a group whose
    sum is half a turn or more is a wrap or click and takes the median
    values. At 13.33 MS/s (span 1) this is the classic one-sample wrap
    repair. Single samples further than ``click_thr`` from the median are
    replaced as well."""
    n = len(d)
    out = d.copy()
    dmg = np.zeros(n, np.uint8)
    h = 3 * span
    buf = np.empty(2 * h)
    med = np.empty(n)
    for k in range(n):
        med[k] = d[k]
    for k in range(h, n - h - span):
        q = 0
        for o in range(-h, 0):
            buf[q] = d[k + o]
            q += 1
        for o in range(span, span + h):
            buf[q] = d[k + o]
            q += 1
        med[k] = np.median(buf[:q])
    k = h
    while k < n - h - span:
        if wrap:
            s = 0.0
            for j in range(span):
                s += d[k + j] - med[k]
            if abs(s) >= 0.5:
                for j in range(span):
                    out[k + j] = med[k]
                    dmg[k + j] = 96
                k += span
                continue
        if click_thr > 0.0 and abs(d[k] - med[k]) > click_thr:
            out[k] = med[k]
            dmg[k] = 96
        k += 1
    return out, dmg


def wrap_centre(d0: np.ndarray, bins: int = 256, smooth: int = 21) -> float:
    """Centre of the occupied arc: the wrap point goes where the circular
    histogram of the instantaneous frequency is emptiest."""
    x = dsp.wrap_cycles(d0[::2])
    h, _ = np.histogram(x, bins=bins, range=(-0.5, 0.5))
    k = np.ones(smooth)
    hs = np.convolve(np.r_[h[-smooth:], h, h[:smooth]], k, mode="same")[smooth:-smooth]
    gap = (np.argmin(hs) + 0.5) / bins - 0.5
    return float(dsp.wrap_cycles(gap + 0.5))


@dataclass
class Demod:
    fs: float
    z: np.ndarray            # DC-removed I/Q (complex64)
    dc: np.ndarray           # per-sample DC estimate (complex64)
    centre: float            # wrap centre, cycles/sample
    raw_cycles: np.ndarray   # discriminator before repair, cycles/sample
    freq: np.ndarray         # repaired instantaneous frequency, Hz
    video: np.ndarray        # video low-passed frequency, Hz
    damage: np.ndarray
    power: np.ndarray        # |z|^2
    has_magnitude: bool


def demodulate(z: np.ndarray, fs: float, cfg: DecoderConfig, std: StandardParams,
               has_magnitude: bool = True) -> Demod:
    z = np.asarray(z, np.complex64)
    if has_magnitude:
        w = max(64, int(cfg.dc_window_lines * fs / std.line_hz))
        dc = (uniform_filter1d(z.real.astype(np.float64), w, mode="nearest")
              + 1j * uniform_filter1d(z.imag.astype(np.float64), w, mode="nearest")).astype(np.complex64)
        z0 = z - dc
    else:
        dc = np.zeros_like(z)
        z0 = z
    power = (z0.real.astype(np.float64) ** 2 + z0.imag.astype(np.float64) ** 2)
    zr, zi = z0.real.astype(np.float64), z0.imag.astype(np.float64)
    d0 = _discriminate(zr, zi, 0.0)
    centre = cfg.centre_cycles if cfg.centre_cycles is not None else wrap_centre(d0)
    cf = cfg.channel_filter_for(fs)
    if cf > 0 and cf < 0.5 * fs:
        n = np.arange(len(z0), dtype=np.float64)
        rot = np.exp(-2j * np.pi * np.mod(centre * n, 1.0))
        zs = dsp.filt(z0 * rot, dsp.lowpass_taps(cf, fs, 1.5e6))
        raw = _discriminate(zs.real.astype(np.float64), zs.imag.astype(np.float64), 0.0) + centre
    else:
        raw = _discriminate(zr, zi, centre)
    if cfg.demod == "pll":
        # The channel filter above (if any) also feeds the loop.
        src = zs if (cf > 0 and cf < 0.5 * fs) else z0
        if cf > 0 and cf < 0.5 * fs:
            src = zs * np.conj(rot)
        raw = _pll(src.real.astype(np.float64), src.imag.astype(np.float64),
                   2 * np.pi * cfg.pll_bw_hz / fs, 0.707, centre)
    thr = cfg.click_threshold_hz / fs if cfg.click_threshold_hz > 0 else 0.0
    span = max(1, int(round(fs / 13.333e6)))
    rep, dmg = _repair(raw, thr, cfg.wrap_repair, span)
    freq = rep * fs
    vl = cfg.video_lpf_hz or (4.2e6 if std.key == "ntsc" else 5.0e6)
    video = dsp.filt(freq, dsp.lowpass_taps(min(vl, 0.45 * fs), fs))
    if has_magnitude:
        mp = uniform_filter1d(power, max(16, int(2e-6 * fs)), mode="nearest")
        low = power < 0.05 * mp
        dmg = np.where(low & (dmg < D_NOISY), D_NOISY, dmg).astype(np.uint8)
    return Demod(fs, z0, dc, centre, raw, freq.astype(np.float64), video.astype(np.float64), dmg, power,
                 has_magnitude)


# ---------------------------------------------------------------------------
# Sync and time-base correction
# ---------------------------------------------------------------------------

def _rolling_median(x, n):
    if len(x) == 0:
        return x
    return median_filter(x, size=min(n, len(x)) | 1, mode="nearest")


def find_edges(video: np.ndarray, fs: float, cfg: DecoderConfig):
    """Falling sync edges (0H and H/2 candidates) at sub-sample precision.

    Returns (edges, tip_level, blank_level), levels in Hz per edge."""
    ys = dsp.filt(video, dsp.lowpass_taps(cfg.sync_lpf_hz, fs))
    ve = dsp.filt(video, dsp.lowpass_taps(cfg.timing_lpf_hz, fs, 2.0 * cfg.timing_lpf_hz, 40.0))
    tip0 = np.percentile(ys, 3.5)
    med = np.percentile(ys, 50)
    thr0 = tip0 + 0.35 * (med - tip0)
    below = ys < thr0
    W = max(4, int(1.5e-6 * fs))
    cs = np.concatenate([[0], np.cumsum(below)])
    fall = np.flatnonzero(below[1:] & ~below[:-1]) + 1
    cands = []
    last = -10 ** 9
    n = len(ys)
    for f in fall:
        if f + W >= n or f - last < W:
            continue
        if cs[f + W] - cs[f] >= 0.75 * W:
            cands.append(f)
            last = f
    cands = np.array(cands, np.int64)
    if len(cands) < 8:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    us = fs * 1e-6
    tips = np.array([np.median(ys[c + int(0.6 * us): c + int(1.9 * us)]) for c in cands])
    pre0 = np.maximum(cands - int(1.3 * us), 0)
    blanks = np.array([np.median(ys[a: max(a + 1, c - int(0.4 * us))]) for a, c in zip(pre0, cands)])
    tip = _rolling_median(tips, 51)
    blank = _rolling_median(blanks, 51)
    thr = 0.5 * (tip + blank)
    edges = np.full(len(cands), np.nan)
    lo_off, hi_off = int(1.2 * us), int(0.6 * us)
    for j, c in enumerate(cands):
        a, b = max(c - lo_off, 0), min(c + hi_off, n - 1)
        seg = ve[a:b + 1]
        idx = np.flatnonzero((seg[:-1] >= thr[j]) & (seg[1:] < thr[j]))
        if len(idx) == 0:
            continue
        target = c - a - 0.2 * us
        i = idx[np.argmin(np.abs(idx - target))]
        edges[j] = a + i + (seg[i] - thr[j]) / (seg[i] - seg[i + 1])
    ok = np.isfinite(edges)
    edges, tip, blank, thr = edges[ok], tip[ok], blank[ok], thr[ok]
    # Refine on the band-limited signal by bisection.
    lo, hi = edges - 0.6, edges + 0.6
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        v = dsp.interp(ve, mid)
        above = v >= thr
        lo = np.where(above, mid, lo)
        hi = np.where(above, hi, mid)
    return 0.5 * (lo + hi), tip, blank


@njit(cache=True)
def _local_fit(mn, me, n_lo, n_hi, K):
    """Robust local linear fit of 0H position against line index."""
    out = np.empty(n_hi - n_lo + 1)
    m = len(mn)
    j0 = 0
    for n in range(n_lo, n_hi + 1):
        k = K
        while True:
            while j0 < m and mn[j0] < n - k:
                j0 += 1
            j1 = j0
            while j1 < m and mn[j1] <= n + k:
                j1 += 1
            if j1 - j0 >= 4 or k > 64 * K:
                break
            k *= 2
        if j1 - j0 < 2:
            out[n - n_lo] = np.nan
            continue
        x = mn[j0:j1].astype(np.float64) - n
        y = me[j0:j1]
        wts = np.ones(len(x))
        a, b = 0.0, 0.0
        for it in range(3):
            sw = wts.sum()
            if sw < 2:
                break
            mx = (wts * x).sum() / sw
            my = (wts * y).sum() / sw
            sxx = (wts * (x - mx) ** 2).sum()
            b = (wts * (x - mx) * (y - my)).sum() / sxx if sxx > 0 else 0.0
            a = my - b * mx
            r = np.abs(y - (a + b * x))
            mad = np.median(r) * 1.4826
            lim = max(3.0 * mad, 0.15)
            for q in range(len(x)):
                wts[q] = 1.0 if r[q] <= lim else 0.0
        out[n - n_lo] = a
    return out


@dataclass
class LineGrid:
    n: np.ndarray               # line indices (consecutive)
    pos: np.ndarray             # 0H per line plus one extra at the end (len n + 1)
    measured: np.ndarray        # matched edge per line or NaN
    period: float


def line_grid(edges: np.ndarray, fs: float, std: StandardParams, length: int, cfg: DecoderConfig) -> LineGrid:
    L = fs / std.line_hz
    if len(edges) < 8:
        raise ValueError("no sync found")
    e = np.sort(edges)
    tol_acq = 0.01 * L
    tol = max(1.0e-6 * fs, 0.01 * L)

    def near(x, t):
        i = np.searchsorted(e, x)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(e) and abs(e[j] - x) <= t and (best is None or abs(e[j] - x) < abs(e[best] - x)):
                best = j
        return best

    def acquire(start):
        for j in range(np.searchsorted(e, start), len(e)):
            if near(e[j] + L, tol_acq) is not None and near(e[j] + 2 * L, tol_acq) is not None:
                return j
        return None

    j = acquire(e[0] - 1)
    if j is None:
        raise ValueError("no line lock")
    ref = e[j]
    pred, n, misses = ref, 0, 0
    mn, me = [], []
    while pred < length + L:
        k = near(pred, tol)
        if k is not None:
            mn.append(n)
            me.append(e[k])
            pred = e[k]
            misses = 0
            if len(mn) >= 16:
                L = 0.9 * L + 0.1 * (me[-1] - me[-16]) / (mn[-1] - mn[-16])
        else:
            misses += 1
            if misses > 40:
                jj = acquire(pred - L / 2)
                if jj is None:
                    break
                n += int(round((e[jj] - pred) / L))
                pred = e[jj]
                misses = 0
                continue
        pred += L
        n += 1
    mn = np.array(mn, np.int64)
    me = np.array(me, np.float64)
    if len(mn) < 2:
        raise ValueError("no line lock")
    # Global period from a robust fit.
    coef = np.array([L, me[0] - L * mn[0]])
    if len(mn) > 4:
        coef = np.polyfit(mn, me, 1)
        r = me - np.polyval(coef, mn)
        okm = np.abs(r) < max(4 * np.median(np.abs(r)) * 1.4826, 0.3)
        coef = np.polyfit(mn[okm], me[okm], 1)
        L = coef[0]
    n_lo = mn[0] - int(np.floor((me[0] - 2) / L))
    n_hi = mn[-1] + int(np.floor((length - 2 - me[-1]) / L))
    pos = _local_fit(mn, me, n_lo, n_hi, cfg.tbc_half_window)
    bad = ~np.isfinite(pos)
    if bad.any():
        idx = np.arange(n_lo, n_hi + 1)
        pos[bad] = np.polyval(coef, idx[bad])
    measured = np.full(n_hi - n_lo + 1, np.nan)
    measured[mn - n_lo] = me
    ns = np.arange(n_lo, n_hi)          # complete lines need the next 0H
    return LineGrid(ns, pos, measured[:-1], float(L))


def detect_fields(video_sync: np.ndarray, thr: float, grid: LineGrid, std: StandardParams):
    """Frame line per grid line from broad pulses; 0 where unknown."""
    below = video_sync < thr
    cs = np.concatenate([[0], np.cumsum(below)])
    p = grid.pos
    L = np.diff(p)
    nl = len(L)
    halves = np.zeros(2 * nl, bool)
    for h, (fa, fb) in enumerate(((0.12, 0.42), (0.62, 0.92))):
        a = np.clip((p[:-1] + fa * L).astype(np.int64), 0, len(below))
        b = np.clip((p[:-1] + fb * L).astype(np.int64), 0, len(below))
        frac = (cs[b] - cs[a]) / np.maximum(b - a, 1)
        halves[h::2] = frac > 0.5
    expected = std.vbi_slots[1]
    d = np.diff(np.concatenate([[0], halves.astype(np.int8), [0]]))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    votes = []
    lpf = std.lines_per_frame
    for s, e in zip(starts, ends):
        ln = e - s
        if ln < 3 or ln > expected + 2:
            continue
        hs = s if ln >= expected - 1 else e - expected
        parity = hs % 2
        line_idx = hs // 2
        anchor = std.broad_anchor_lines[parity]
        votes.append((line_idx - (anchor - 1)) % lpf)
    frame_line = np.zeros(nl, np.int64)
    if not votes:
        return frame_line, 0
    vals, counts = np.unique(votes, return_counts=True)
    off = int(vals[np.argmax(counts)])
    frame_line[:] = (np.arange(nl) - off) % lpf + 1
    return frame_line, len(votes)


# ---------------------------------------------------------------------------
# Line records
# ---------------------------------------------------------------------------

@njit(cache=True)
def _env_cn(power, starts, ends):
    out = np.empty(len(starts))
    for j in range(len(starts)):
        a, b = starts[j], ends[j]
        if b - a < 8:
            out[j] = np.nan
            continue
        seg = power[a:b]
        m = seg.mean()
        v = seg.var()
        if v <= 0:
            out[j] = 1e6
            continue
        r = m * m / v
        out[j] = r - 1.0 + np.sqrt(r * r - r) if r > 1.0 else 0.0
    return out


@njit(cache=True)
def _grid_damage(dmg, pos):
    out = np.zeros(len(pos), np.uint8)
    n = len(dmg)
    for j in range(len(pos)):
        i0 = int(np.floor(pos[j]))
        mx = 0
        for i in range(i0 - 1, i0 + 3):
            if 0 <= i < n and dmg[i] > mx:
                mx = dmg[i]
        out[j] = mx
    return out


@dataclass
class Lines:
    std: StandardParams
    n: np.ndarray               # line index (consecutive)
    src_position: np.ndarray    # 0H in input samples
    src_line_len: np.ndarray
    measured: np.ndarray        # matched edge or NaN
    frame_line: np.ndarray      # 1-based, 0 unknown
    parity: np.ndarray          # 0 first, 1 second, -1 unknown
    field_seq: np.ndarray       # -1 unknown
    hz: np.ndarray              # grid samples in Hz (lines x spl)
    ire: np.ndarray             # grid samples in IRE
    damage: np.ndarray          # lines x spl
    tip_hz: np.ndarray
    blank_hz: np.ndarray
    noise_ire: np.ndarray
    clicks: np.ndarray
    cn_db: np.ndarray
    power_db: np.ndarray
    timing_err: np.ndarray
    flags: np.ndarray
    anchors: int = 0

    @property
    def weight(self):
        n = np.maximum(np.nan_to_num(self.noise_ire, nan=1e3), 0.05)
        return 1.0 / (n * n)

    def __len__(self):
        return len(self.n)


def build_lines(demod: Demod, std: StandardParams, cfg: DecoderConfig, phase_ref=None) -> Lines:
    """``phase_ref`` (origin, period) in this signal's samples: keep only
    edges within 0.15 line of a whole line of that grid (used to stop a
    short segment from locking onto the half-line equalising pulses)."""
    fs = demod.fs
    edges, tip_e, blank_e = find_edges(demod.video, fs, cfg)
    if phase_ref is not None and len(edges):
        o, per = phase_ref
        fr = (edges - o) / per
        ok = np.abs(fr - np.round(fr)) < 0.15
        edges, tip_e, blank_e = edges[ok], tip_e[ok], blank_e[ok]
    grid = line_grid(edges, fs, std, len(demod.video), cfg)
    thr_g = float(np.median(0.5 * (tip_e + blank_e)))
    ys = dsp.filt(demod.video, dsp.lowpass_taps(cfg.sync_lpf_hz, fs))
    frame_line, anchors = detect_fields(ys, thr_g, grid, std)
    spl = std.samples_per_line
    p = grid.pos
    nl = len(grid.n)
    start, nxt = p[:-1], p[1:]
    k = np.arange(spl)
    pos = (start[:, None] + (nxt - start)[:, None] * k[None, :] / spl).ravel()
    hz = dsp.interp(demod.video, pos).reshape(nl, spl)
    gdmg = _grid_damage(demod.damage, pos).reshape(nl, spl)
    tr, bp = std.sync_tip_range, std.back_porch_range
    tip = hz[:, tr.start:tr.stop].mean(1)
    blank = hz[:, bp.start:bp.stop].mean(1)
    normal = np.ones(nl, bool)
    known = frame_line > 0
    if known.any():
        normal = np.array([std.slot_kind(2 * (fl - 1)) == "normal" if fl > 0 else True for fl in frame_line])
    idx = np.flatnonzero(normal)
    w = cfg.level_smooth_lines
    tip_s = np.interp(np.arange(nl), idx, _rolling_median(tip[idx], w))
    blank_s = np.interp(np.arange(nl), idx, _rolling_median(blank[idx], w))
    span = blank_s - tip_s
    span = np.maximum(span, 0.5 * np.median(span) if np.median(span) > 0 else 1.0)
    ire = (hz - blank_s[:, None]) * (-std.sync_ire) / span[:, None]
    vt = ire[:, tr.start:tr.stop].var(1)
    vb = ire[:, bp.start:bp.stop].var(1)
    noise = np.sqrt(0.5 * (vt + vb))
    noise[~normal] = np.nan
    a = np.clip(np.floor(start).astype(np.int64), 0, len(demod.damage))
    b = np.clip(np.floor(nxt).astype(np.int64), 0, len(demod.damage))
    rep = np.concatenate([[0], np.cumsum(demod.damage >= D_REPAIRED)])
    clicks = rep[b] - rep[a]
    if demod.has_magnitude:
        c = _env_cn(demod.power, a, b)
        cn_db = 10 * np.log10(np.maximum(c, 1e-3))
        pc = np.concatenate([[0.0], np.cumsum(demod.power)])
        power_db = 10 * np.log10(np.maximum((pc[b] - pc[a]) / np.maximum(b - a, 1), 1e-12))
    else:
        cn_db = np.full(nl, np.nan)
        power_db = np.full(nl, np.nan)
    timing_err = grid.measured - start
    flags = np.where(np.isfinite(grid.measured), 0, F_FLYWHEEL).astype(np.uint32)
    flags = np.where(normal, flags, flags | F_VBI).astype(np.uint32)
    parity = np.where(known, np.where(frame_line <= std.lines_per_field[0], FIRST, SECOND), -1)
    field_seq = np.full(nl, -1, np.int64)
    if known.any():
        lpf = std.lines_per_frame
        frame_no = np.floor((grid.n - grid.n[0] - (frame_line - 1)) / lpf + 1e-9).astype(np.int64)
        # frame_no counts frames since the start; normalise to the first field seen.
        seq = 2 * (frame_no - frame_no[0]) + parity
        field_seq = np.where(known, seq - seq[known].min(), -1)
    return Lines(std, grid.n, start, nxt - start, grid.measured, frame_line, parity, field_seq, hz,
                 ire.astype(np.float32), gdmg, tip_s, blank_s, noise, clicks, cn_db, power_db, timing_err, flags,
                 anchors)


def detect_standard(demod_video: np.ndarray, fs: float, cfg: DecoderConfig) -> StandardParams:
    edges, _, _ = find_edges(demod_video, fs, cfg)
    d = np.diff(np.sort(edges))
    best, score = NTSC, -1
    for std in (NTSC, PAL):
        L = fs / std.line_hz
        s = np.mean(np.abs(d - L) < 0.002 * L)
        if s > score:
            best, score = std, s
    return best


@dataclass
class DecodeResult:
    demod: Demod
    lines: Lines
    std: StandardParams


def decode_iq(z, fs: float, cfg: Optional[DecoderConfig] = None, std=None, has_magnitude: bool = True) -> DecodeResult:
    cfg = cfg or DecoderConfig()
    if std is None and cfg.standard:
        std = get_standard(cfg.standard)
    if isinstance(std, str):
        std = get_standard(std)
    if std is None:
        probe = demodulate(z, fs, cfg, NTSC, has_magnitude)
        std = detect_standard(probe.video, fs, cfg)
    demod = demodulate(z, fs, cfg, std, has_magnitude)
    return DecodeResult(demod, build_lines(demod, std, cfg), std)


def decode_recording(rec, rx: int = 0, cfg: Optional[DecoderConfig] = None) -> DecodeResult:
    cfg = cfg or DecoderConfig()
    info = rec.receivers[rx]
    std = get_standard(cfg.standard or rec.standard_hint) if (cfg.standard or rec.standard_hint) else None
    return decode_iq(rec.iq(rx), info.sample_rate_hz, cfg, std, info.sample_format.lanes.has_magnitude)


# ---------------------------------------------------------------------------
# Fields and frames
# ---------------------------------------------------------------------------

def fields(lines: Lines) -> dict:
    """{field_seq: (parity, array lines_per_field x spl IRE, NaN missing)}."""
    std = lines.std
    out = {}
    for j in np.flatnonzero(lines.field_seq >= 0):
        seq, par = int(lines.field_seq[j]), int(lines.parity[j])
        if seq not in out:
            out[seq] = (par, np.full((std.lines_per_field[par], std.samples_per_line), np.nan, np.float32))
        row = int(lines.frame_line[j]) - std.field_first_frame_line(par)
        out[seq][1][row] = lines.ire[j]
    return out


def frames(lines: Lines, require_complete: bool = True) -> list:
    """Woven frames (frame_height x spl IRE) from field pairs (field 1 then 2)."""
    std = lines.std
    fl = fields(lines)
    out = []
    for seq in sorted(fl):
        par, f1 = fl[seq]
        if par != FIRST or (seq + 1) not in fl:
            continue
        f2 = fl[seq + 1][1]
        frame = np.full((std.frame_height, std.samples_per_line), np.nan, np.float32)
        for parity, arr in ((FIRST, f1), (SECOND, f2)):
            first = std.field_first_frame_line(parity)
            for r in range(arr.shape[0]):
                row = std.output_row(first + r)
                if row is not None:
                    frame[row] = arr[r]
        if require_complete and np.isnan(frame).any():
            continue
        out.append(frame)
    return out


def write_images(rgb_frames: list, out: Path, fps: float):
    """PNG (single frame or %04d pattern) or MP4/other through ffmpeg."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames8 = [np.clip(np.nan_to_num(f) * 255 + 0.5, 0, 255).astype(np.uint8) for f in rgb_frames]
    if not frames8:
        raise ValueError("no complete frame decoded")
    h, w = frames8[0].shape[:2]
    if out.suffix.lower() == ".png":
        import imageio.v3 as iio
        if len(frames8) == 1 or "%" not in out.name:
            iio.imwrite(out, frames8[0])
        else:
            for i, f in enumerate(frames8):
                iio.imwrite(str(out) % i, f)
        return
    cmd = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
           "-r", f"{fps:.6f}", "-i", "-", "-vf", f"scale={w & ~1}:{h & ~1},setdar=4/3",
           "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "16", str(out)]
    subprocess.run(cmd, input=b"".join(f.tobytes() for f in frames8), check=True)


# ---------------------------------------------------------------------------
# Recordings with gaps (usb_gap events)
# ---------------------------------------------------------------------------

@dataclass
class SegmentLines:
    """Lines of one contiguous segment, numbered on the global line count."""
    start: int                  # absolute sample index of the segment's first sample
    length: int
    lines: Lines
    keep: np.ndarray            # rows fully inside the segment (filters settled)
    abs_n: np.ndarray           # global line number per row (keep rows only)


@dataclass
class GapDecode:
    std: StandardParams
    fs: float
    segments: list              # [SegmentLines]
    period: float               # global line period (samples)
    origin: float               # absolute 0H of global line 0
    fit_resid: np.ndarray       # measured 0H minus global fit, per measured line
    frame_offset: Optional[int]  # global line n has frame line (n - offset) % lpf + 1
    frame_votes: np.ndarray     # (n - frame_line + 1) % lpf per segment with a field anchor
    failed: int                 # segments without line lock
    relocked: int = 0           # segments re-locked onto the global grid

    def rows(self):
        """All kept lines: (absolute 0H, global n, Lines, row) in time order."""
        for s in self.segments:
            for j, n in zip(s.keep, s.abs_n):
                yield s.start + s.lines.src_position[j], int(n), s.lines, int(j)


def _subset_ok(lines: Lines, seg_len: int, margin: float) -> np.ndarray:
    a = lines.src_position
    b = a + lines.src_line_len
    return np.flatnonzero((a >= margin) & (b <= seg_len - margin))


def decode_gapped(rec, rx: int = 0, cfg: Optional[DecoderConfig] = None, std=None,
                  margin_us: float = 4.0) -> GapDecode:
    """Decodes a recording whose stream has holes (``usb_gap`` events).

    Every contiguous segment is demodulated and line-locked on its own (the
    DC window is shortened to fit a 32 KB block). The segments' absolute
    sample positions (file offset plus the bytes lost before it) then give
    one global line count: a line period from the within-segment line
    lengths, a sequential count across each gap, a robust linear fit of all
    measured 0H against that count. Segments that hold broad pulses vote for
    the frame phase, so every line gets its frame line and the field
    structure survives the gaps. The fit residual is a direct check of the
    gap accounting: a wrong ``lost_bytes`` shows as a jump of a line or more.
    """
    cfg = cfg or DecoderConfig(dc_window_lines=4.0)
    info = rec.receivers[rx]
    fs = float(info.sample_rate_hz)
    has_mag = info.sample_format.lanes.has_magnitude
    segs = rec.segments(rx, min_words=int(12 * fs / 15e3))
    if std is None:
        std = get_standard(cfg.standard or rec.standard_hint) if (cfg.standard or rec.standard_hint) else None
    if std is None:
        z = segs[0][1]
        std = detect_standard(demodulate(z, fs, cfg, NTSC, has_mag).video, fs, cfg)
    margin = margin_us * 1e-6 * fs
    out, failed = [], 0
    for a, z in segs:
        try:
            dm = demodulate(z, fs, cfg, std, has_mag)
            L = build_lines(dm, std, cfg)
        except ValueError:
            failed += 1
            continue
        keep = _subset_ok(L, len(z), margin)
        if len(keep) < 2:
            failed += 1
            continue
        out.append(SegmentLines(int(a), len(z), L, keep, np.zeros(len(keep), np.int64)))
    if not out:
        raise ValueError("no segment locked")
    def number(segs, period):
        prev_pos, prev_n = None, 0
        for s in segs:
            p = s.start + s.lines.src_position[s.keep]
            local = s.lines.n[s.keep] - s.lines.n[s.keep][0]
            n0 = 0 if prev_pos is None else prev_n + int(round((p[0] - prev_pos) / period))
            s.abs_n = n0 + local
            prev_pos, prev_n = p[-1], int(s.abs_n[-1])

    def fit(segs):
        mp, mn = [], []
        for s in segs:
            meas = s.lines.measured[s.keep]
            ok = np.isfinite(meas)
            mp.append(s.start + meas[ok])
            mn.append(s.abs_n[ok])
        mp, mn = np.concatenate(mp), np.concatenate(mn)
        coef = np.polyfit(mn, mp, 1)
        for _ in range(3):
            r = mp - np.polyval(coef, mn)
            okm = np.abs(r) < max(5 * 1.4826 * np.median(np.abs(r - np.median(r))), 1.0)
            coef = np.polyfit(mn[okm], mp[okm], 1)
        return coef, mp - np.polyval(coef, mn)

    # Pass 1: the line period and phase from segments with active video
    # (nearly every line measured). Counting across a gap of G lines errs by
    # G * (period error) / period; the median of thousands of line lengths
    # keeps that far below half a line.
    def well(s):
        return np.isfinite(s.lines.measured[s.keep]).mean() > 0.8
    good = [s for s in out if well(s)] or out
    lens = np.concatenate([s.lines.src_line_len[s.keep][np.isfinite(s.lines.measured[s.keep])] for s in good])
    number(good, float(np.median(lens)))
    coef, _ = fit(good)
    # Pass 2: every segment off the global grid by more than 0.1 line (the
    # vertical-interval segments that locked onto H/2 pulses) is re-locked
    # with the global phase as reference.
    relocked = 0
    for k, s in enumerate(out):
        p = s.start + s.lines.src_position[s.keep]
        fr = (p - coef[1]) / coef[0]
        if np.median(np.abs(fr - np.round(fr))) <= 0.1:
            continue
        a, z = s.start, None
        for aa, zz in segs:
            if aa == a:
                z = zz
                break
        try:
            dm = demodulate(z, fs, cfg, std, has_mag)
            L = build_lines(dm, std, cfg, (coef[1] - a, coef[0]))
        except ValueError:
            L = None
        keep = _subset_ok(L, len(z), margin) if L is not None else np.zeros(0, np.int64)
        out[k] = SegmentLines(a, len(z), L, keep, np.zeros(len(keep), np.int64)) if len(keep) >= 2 else None
        relocked += 1
    failed += sum(1 for s in out if s is None)
    out = [s for s in out if s is not None]
    # Final numbering straight from the global fit, then refit on everything.
    for s in out:
        p = s.start + s.lines.src_position[s.keep]
        s.abs_n = np.round((p - coef[1]) / coef[0]).astype(np.int64)
    coef, resid = fit(out)
    period = float(coef[0])
    # Frame phase from the segments that saw broad pulses.
    lpf = std.lines_per_frame
    votes = []
    for s in out:
        if s.lines.anchors <= 0:
            continue
        fl = s.lines.frame_line[s.keep]
        if (fl > 0).any():
            votes.append(int(np.bincount((s.abs_n[fl > 0] - (fl[fl > 0] - 1)) % lpf).argmax()))
    votes = np.array(votes, np.int64)
    off = int(np.bincount(votes).argmax()) if len(votes) else None
    return GapDecode(std, fs, out, float(coef[0]), float(coef[1]), resid, off, votes, failed, relocked)


def mosaic_frame(gd: GapDecode, t0: Optional[float] = None, t1: Optional[float] = None):
    """Woven frame (frame_height x spl IRE) from the newest copy of every
    frame line seen in [t0, t1) seconds; second array: lines per row."""
    std = gd.std
    if gd.frame_offset is None:
        raise ValueError("no vertical sync seen")
    lpf = std.lines_per_frame
    frame = np.full((std.frame_height, std.samples_per_line), np.nan, np.float32)
    count = np.zeros(std.frame_height, np.int64)
    for pos, n, L, j in gd.rows():
        t = pos / gd.fs
        if (t0 is not None and t < t0) or (t1 is not None and t >= t1):
            continue
        fl = (n - gd.frame_offset) % lpf + 1
        row = std.output_row(fl)
        if row is None:
            continue
        frame[row] = L.ire[j]
        count[row] += 1
    return frame, count
