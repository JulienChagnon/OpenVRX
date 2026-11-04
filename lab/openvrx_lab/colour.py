# SPDX-License-Identifier: GPL-3.0-or-later
"""Simple NTSC/PAL colour decoder on the line-locked grid (1D, burst locked).

Per line: chroma band-pass around fsc, product detector against the grid
subcarrier, 1.3 MHz low-pass, phase (and amplitude, ACC) corrected by that
line's burst. PAL: the V switch is found from the +-135 degree burst swing
and consecutive lines are averaged (delay-line PAL). Luma is the composite
minus the band-passed chroma. Output BT.601 RGB in [0, 1].
"""
from __future__ import annotations

import numpy as np

from . import dsp
from .standards import StandardParams

CHROMA_HALF_BAND = 1.3e6


def _demod_lines(x: np.ndarray, std: StandardParams):
    fs = std.grid_rate_hz
    bp = dsp.bandpass_taps(std.fsc_hz, CHROMA_HALF_BAND, fs)
    c = dsp.filt(x, bp, axis=1)
    k = np.arange(x.shape[1])
    lo = np.exp(-2j * np.pi * std.fsc_per_sample * k)[None, :]
    q = dsp.filt(2 * c * lo, dsp.lowpass_taps(CHROMA_HALF_BAND, fs), axis=1)
    br = std.burst_range
    burst = q[:, br.start + 4: br.stop - 4].mean(1)
    return c, q, burst


def decode_frame(frame: np.ndarray, std: StandardParams, colour: bool = True) -> np.ndarray:
    """Woven composite frame (rows x samples_per_line, IRE) to RGB of the
    active picture (rows x active_width x 3)."""
    x = np.nan_to_num(np.asarray(frame, np.float64))
    a = std.active_range
    scale = 92.5 if std.key == "ntsc" else 100.0
    if not colour:
        y = (x - std.black_ire) / (std.white_ire - std.black_ire)
        y = y[:, a.start:a.stop]
        return np.repeat(np.clip(y, 0, 1)[:, :, None], 3, axis=2)
    c, q, burst = _demod_lines(x, std)
    mag = np.maximum(np.abs(burst), 1e-6)
    acc = np.clip(std.burst_ire / mag, 0.5, 2.0)
    if std.key == "ntsc":
        # Burst demodulates to j * A e^{j phi}: remove phi.
        corr = 1j * np.conj(burst) / mag
        qc = q * (corr * acc)[:, None]
        u, v = -qc.imag, qc.real
    else:
        # Burst at 45 or 135 degrees (s = +1 / -1) plus a phase error phi.
        ang = np.angle(burst)
        pp, pm = ang - np.pi / 4, ang - 3 * np.pi / 4
        grid = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        score = np.maximum(np.cos(pp[None, :] - grid[:, None]), np.cos(pm[None, :] - grid[:, None])).sum(1)
        phi0 = grid[np.argmax(score)]
        plus = np.cos(pp - phi0) >= np.cos(pm - phi0)
        s = np.where(plus, 1.0, -1.0)
        phi = np.where(plus, pp, pm)
        qc = q * (np.exp(-1j * phi) * acc)[:, None]
        u, v = -qc.imag, s[:, None] * qc.real
        # Delay-line PAL: average with the previous line of the same field.
        u2 = u.copy()
        v2 = v.copy()
        u2[2:] = 0.5 * (u[2:] + u[:-2])
        v2[2:] = 0.5 * (v[2:] + v[:-2])
        u, v = u2, v2
    luma = x - c
    setup = std.black_ire
    y = (luma - setup) / (std.white_ire - setup)
    u, v = u / scale, v / scale
    y, u, v = y[:, a.start:a.stop], u[:, a.start:a.stop], v[:, a.start:a.stop]
    r = y + 1.140 * v
    g = y - 0.395 * u - 0.581 * v
    b = y + 2.032 * u
    return np.clip(np.stack([r, g, b], axis=-1), 0, 1)
