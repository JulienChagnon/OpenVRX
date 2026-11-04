# SPDX-License-Identifier: GPL-3.0-or-later
"""Diversity combining.

Post-detection (on time-base-corrected line records, any receiver clocks):

- D1 select: per line, the receiver with the best envelope C/N (or the
  lowest flat-region noise for angle-only formats), with 1 dB hysteresis.
- D2 blend: per line, weights 1 / noise^2 (inverse noise variance after
  detection), noise from the line's flat regions.
- D3 repair: D2 weights, and per grid sample a receiver whose damage mask
  flags the sample (click, wrap, envelope dip) gets 2 % of its weight.

Pre-detection:

- D4 maximal-ratio combining of the complex I/Q before the discriminator:
  B is resampled onto A's time base from both receivers' line timing,
  derotated by the carrier offset measured from the two blanking levels,
  phase-aligned by a smoothed z_A conj(z_B), and summed with weights
  a_k / sigma_k^2 from a running envelope C/N estimate.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import uniform_filter1d

from . import dsp
from .decoder import D_NOISY, D_REPAIRED
from .metrics import cn_from_ratio


@dataclass
class Aligned:
    """Lines of several receivers that show the same transmitted line."""
    keys: np.ndarray            # common line key (e.g. absolute line number)
    rows: list                  # per receiver: row index into its Lines
    lines: list                 # per receiver Lines


def align(lines_list, keys_list) -> Aligned:
    common = keys_list[0]
    for k in keys_list[1:]:
        common = np.intersect1d(common, k)
    rows = []
    for k in keys_list:
        pos = {int(v): i for i, v in enumerate(k)}
        rows.append(np.array([pos[int(v)] for v in common], np.int64))
    return Aligned(common, rows, list(lines_list))


def _stack(al: Aligned, attr):
    return np.stack([getattr(L, attr)[r] for L, r in zip(al.lines, al.rows)])


def noise_of(al: Aligned):
    n = _stack(al, "noise_ire").astype(np.float64)
    # VBI lines carry no flat-region noise: borrow the neighbours' value.
    for k in range(len(n)):
        bad = ~np.isfinite(n[k])
        if bad.all():
            n[k] = 1e3
        elif bad.any():
            idx = np.arange(n.shape[1])
            n[k, bad] = np.interp(idx[bad], idx[~bad], n[k, ~bad])
    return np.maximum(n, 0.05)


def d1_select(al: Aligned, hysteresis_db: float = 1.0):
    cn = _stack(al, "cn_db").astype(np.float64)
    score = cn if np.isfinite(cn).all() else -20 * np.log10(noise_of(al))
    ire = _stack(al, "ire")
    cur = int(np.argmax(score[:, 0]))
    out = np.empty(ire.shape[1:], np.float32)
    src = np.empty(ire.shape[1], np.int64)
    for j in range(ire.shape[1]):
        best = int(np.argmax(score[:, j]))
        if best != cur and score[best, j] > score[cur, j] + hysteresis_db:
            cur = best
        out[j] = ire[cur, j]
        src[j] = cur
    return out, src


def d2_blend(al: Aligned):
    ire = _stack(al, "ire").astype(np.float64)
    w = 1.0 / noise_of(al) ** 2
    return (np.einsum("kl,kls->ls", w, ire) / w.sum(0)[:, None]).astype(np.float32), w


def d3_repair(al: Aligned, spread: int = 3, floor: float = 0.02):
    ire = _stack(al, "ire").astype(np.float64)
    dmg = _stack(al, "damage")
    w = (1.0 / noise_of(al) ** 2)[:, :, None] * np.ones(ire.shape)
    bad = dmg >= min(D_NOISY, D_REPAIRED)
    if spread:
        bad = uniform_filter1d(bad.astype(np.float32), 2 * spread + 1, axis=2) > 0
    w = np.where(bad, w * floor, w)
    return ((w * ire).sum(0) / w.sum(0)).astype(np.float32)


# ---------------------------------------------------------------------------
# D4: pre-detection maximal-ratio combining
# ---------------------------------------------------------------------------

def _running_cn(z, win):
    p = z.real.astype(np.float64) ** 2 + z.imag.astype(np.float64) ** 2
    m = uniform_filter1d(p, win, mode="nearest")
    m2 = uniform_filter1d(p * p, win, mode="nearest")
    var = np.maximum(m2 - m * m, 1e-12)
    c = cn_from_ratio(m * m / var)
    return m, c


def mrc(za: np.ndarray, zb: np.ndarray, lines_a, lines_b, rows_a, rows_b, fs: float,
        phase_win_us: float = 4.0, cn_win_us: float = 16.0) -> np.ndarray:
    """Combines DC-removed I/Q of two receivers onto A's sample grid."""
    pa = lines_a.src_position[rows_a]
    pb = lines_b.src_position[rows_b]
    t = np.arange(len(za), dtype=np.float64)
    # Piecewise-linear map of A sample index to B sample index (line timing).
    slope = np.polyfit(pa, pb, 1)
    tb = np.interp(t, pa, pb, left=np.nan, right=np.nan)
    lo, hi = np.isnan(tb) & (t < pa[0]), np.isnan(tb) & (t > pa[-1])
    tb[lo | hi] = np.polyval(slope, t[lo | hi])
    zb_a = dsp.interp(zb, tb).astype(np.complex64)
    valid = (tb >= 0) & (tb < len(zb) - 1)
    # Carrier offset of B against A from the blanking levels (Hz).
    df = float(np.nanmedian(lines_b.blank_hz[rows_b]) - np.nanmedian(lines_a.blank_hz[rows_a]))
    zb_a = zb_a * np.exp(-2j * np.pi * np.mod(df / fs * t, 1.0)).astype(np.complex64)
    # Smoothed relative phase.
    w1 = max(4, int(phase_win_us * 1e-6 * fs))
    x = za.astype(np.complex128) * np.conj(zb_a.astype(np.complex128))
    xs = uniform_filter1d(x.real, w1, mode="nearest") + 1j * uniform_filter1d(x.imag, w1, mode="nearest")
    rot = xs / np.maximum(np.abs(xs), 1e-12)
    # Weights a_k / sigma_k^2.
    w2 = max(16, int(cn_win_us * 1e-6 * fs))
    ma, ca = _running_cn(za, w2)
    mb, cb = _running_cn(zb_a, w2)
    sa2, sb2 = ma / (1 + ca), mb / (1 + cb)
    aa, ab = np.sqrt(ma * ca / (1 + ca)), np.sqrt(mb * cb / (1 + cb))
    wa, wb = aa / np.maximum(sa2, 1e-9), ab / np.maximum(sb2, 1e-9)
    wb = np.where(valid, wb, 0.0)
    y = wa * za + wb * rot * zb_a
    # Keep a level the decoder's DC/envelope logic is used to.
    scale = np.sqrt(np.mean(np.abs(za) ** 2) / max(np.mean(np.abs(y) ** 2), 1e-12))
    return (y * scale).astype(np.complex64)
