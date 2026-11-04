# SPDX-License-Identifier: GPL-3.0-or-later
"""Filters and resamplers shared by the simulator, decoder and metrics.

All video-domain filters are linear phase FIRs applied with zero delay
(odd length, centred), so a symmetric edge keeps its 50 % point. The
fractional resampler is a Kaiser-windowed sinc evaluated from a dense table.
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
from numba import njit
from scipy import signal as sg

BUS_HZ = 80e6
# C5 analog channel (C5VRX research/sim.py): 5th-order Butterworth, 10 MHz
# cut-off on the complex baseband at 80 MS/s, ~20 MHz channel.
CHAN_SOS = sg.butter(5, 10e6, fs=BUS_HZ, output="sos")
# 802.11p setting: receive filter about +-6 MHz instead of +-9.5 MHz. Modelled
# as an extra 5th-order Butterworth at 6 MHz after the noise is added.
P11_SOS = sg.butter(5, 6e6, fs=BUS_HZ, output="sos")


@lru_cache(maxsize=None)
def sos_power_gain(key: str) -> float:
    """Sum of squared impulse response (white-noise power gain)."""
    sos = {"chan": CHAN_SOS, "p11": P11_SOS}[key]
    imp = np.zeros(8192)
    imp[0] = 1.0
    h = sg.sosfilt(sos, imp)
    return float(np.sum(h * h))


def sos_group_delay(sos, freqs_hz, fs=BUS_HZ) -> np.ndarray:
    """Group delay in samples at the given frequencies (complex baseband)."""
    b, a = sg.sos2tf(sos)
    _, gd = sg.group_delay((b, a), w=np.asarray(freqs_hz, float), fs=fs)
    return gd


@lru_cache(maxsize=256)
def lowpass_taps(cutoff_hz: float, fs: float, transition_hz: float | None = None, atten_db: float = 60.0):
    """Linear-phase low-pass FIR (Kaiser), odd length, unity DC gain.

    ``cutoff_hz`` is the -6 dB point. Returns a read-only array.
    """
    cutoff_hz = float(min(cutoff_hz, 0.49 * fs))
    if transition_hz is None:
        transition_hz = max(0.25 * cutoff_hz, 0.6e6)
    transition_hz = min(transition_hz, 2 * (0.5 * fs - cutoff_hz) if cutoff_hz < 0.5 * fs else transition_hz)
    transition_hz = max(transition_hz, 0.02 * fs)
    numtaps, beta = sg.kaiserord(atten_db, transition_hz / (0.5 * fs))
    numtaps |= 1
    taps = sg.firwin(numtaps, cutoff_hz, window=("kaiser", beta), fs=fs)
    taps /= taps.sum()
    taps.setflags(write=False)
    return taps


@lru_cache(maxsize=64)
def bandpass_taps(center_hz: float, half_width_hz: float, fs: float, transition_hz: float = 0.6e6):
    """Real linear-phase band-pass FIR (modulated low-pass), odd length."""
    lp = np.array(lowpass_taps(half_width_hz, fs, transition_hz))
    n = np.arange(len(lp)) - len(lp) // 2
    taps = 2.0 * lp * np.cos(2 * np.pi * center_hz / fs * n)
    taps.setflags(write=False)
    return taps


def filt(x: np.ndarray, taps: np.ndarray, axis: int = -1) -> np.ndarray:
    """Zero-delay FIR filtering (odd symmetric taps), edges by reflection."""
    taps = np.asarray(taps)
    half = len(taps) // 2
    x = np.asarray(x)
    if x.shape[axis] <= half + 1:
        return x.copy()
    pad = [(0, 0)] * x.ndim
    pad[axis] = (half, half)
    xp = np.pad(x, pad, mode="reflect")
    shape = [1] * x.ndim
    shape[axis] = len(taps)
    y = sg.oaconvolve(xp, taps.reshape(shape), mode="valid", axes=axis)
    return y.astype(np.result_type(x.dtype, np.float32), copy=False)


# ---------------------------------------------------------------------------
# Windowed-sinc fractional resampler
# ---------------------------------------------------------------------------

SINC_HALF = 16          # taps each side
SINC_OVERSAMPLE = 512   # table points per input sample
SINC_BETA = 8.0


@lru_cache(maxsize=8)
def _sinc_table(cutoff: float) -> np.ndarray:
    """Kernel table over [-SINC_HALF, SINC_HALF], ``cutoff`` relative to the
    input Nyquist (1.0 = full band)."""
    t = np.arange(-SINC_HALF * SINC_OVERSAMPLE, SINC_HALF * SINC_OVERSAMPLE + 2) / SINC_OVERSAMPLE
    w = np.kaiser(len(t), SINC_BETA)
    k = cutoff * np.sinc(cutoff * t) * w
    return k.astype(np.float64)


@njit(cache=True, fastmath=False)
def _interp_real(x, pos, table, half, over):
    n = len(x)
    out = np.empty(len(pos))
    for j in range(len(pos)):
        p = pos[j]
        i0 = int(np.floor(p))
        frac = p - i0
        acc = 0.0
        wsum = 0.0
        for k in range(-half + 1, half + 1):
            idx = i0 + k
            t = (k - frac) * over + half * over
            ti = int(t)
            tf = t - ti
            w = table[ti] * (1.0 - tf) + table[ti + 1] * tf
            if idx < 0:
                idx = 0
            elif idx >= n:
                idx = n - 1
            acc += x[idx] * w
            wsum += w
        out[j] = acc / wsum if wsum != 0.0 else 0.0
    return out


def interp(x: np.ndarray, positions: np.ndarray, cutoff: float = 1.0) -> np.ndarray:
    """Band-limited value of ``x`` at fractional sample ``positions``.

    The kernel is normalised per output point (unity DC gain). Use
    ``cutoff < 1`` when decimating a signal that is not already band-limited.
    """
    table = _sinc_table(float(cutoff))
    pos = np.ascontiguousarray(positions, dtype=np.float64)
    if np.iscomplexobj(x):
        re = _interp_real(np.ascontiguousarray(x.real, np.float64), pos, table, SINC_HALF, SINC_OVERSAMPLE)
        im = _interp_real(np.ascontiguousarray(x.imag, np.float64), pos, table, SINC_HALF, SINC_OVERSAMPLE)
        return re + 1j * im
    return _interp_real(np.ascontiguousarray(x, np.float64), pos, table, SINC_HALF, SINC_OVERSAMPLE)


# ---------------------------------------------------------------------------
# Weighting network for picture SNR
# ---------------------------------------------------------------------------

WEIGHT_TAU_S = 245e-9   # unified-weighting-network time constant


def weighting_filter(x: np.ndarray, fs: float, band_hz: float, axis: int = -1) -> np.ndarray:
    """Noise weighting: first-order low-pass with tau = 245 ns (power response
    1/(1+(2 pi f tau)^2)), then band limit at ``band_hz``. Zero-phase."""
    fc = 1.0 / (2 * np.pi * WEIGHT_TAU_S)
    b, a = sg.butter(1, fc, fs=fs)
    # Single pass in each direction would square the response; apply the
    # magnitude once by filtering forward only (phase is irrelevant for noise
    # power), then the band limit.
    y = sg.lfilter(b, a, x, axis=axis)
    return filt(y, lowpass_taps(band_hz, fs), axis=axis)


def wrap_cycles(x):
    """Wraps to [-0.5, 0.5)."""
    return x - np.floor(x + 0.5)
