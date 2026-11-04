# SPDX-License-Identifier: GPL-3.0-or-later
#
# The envelope C/N estimate is adapted from C5VRX main/cn_estimate.h and
# research/cn_envelope.py (commit 6bc9e4bbb5bef5948d42a065da27eae39315ad2d),
# C5VRX by Twotoz and the C5VRX contributors, GPL-3.0: with the window DC
# removed, p = |z|^2 of an FM carrier plus complex Gaussian noise has
# mean^2/var R = (c+1)^2/(2c+1). Changed: floating point on any lane format
# instead of x100 integers on 4-bit nibbles, and the exact inverse
# c = R - 1 + sqrt(R^2 - R) instead of the firmware's lookup tables. The
# SINAD follows C5VRX research/sim.py measure()/calibrate(): picture-region
# error against the band-limited truth after a gain/offset fit made on the
# noise-free decode; here on the line-locked grid instead of a lag search.
"""Picture and signal quality metrics.

- ``sinad_db``: video SINAD in the active picture, var(truth) / mean(err^2),
  both through the decoder's video low-pass (C5VRX style, including chroma).
- ``wsnr_db``: weighted picture SNR, 20 log10(100 IRE / weighted RMS error),
  weighting 1/(1 + (2 pi f 245 ns)^2) in a 5 MHz (PAL) / 4.2 MHz (NTSC) band.
- ``psnr_ssim``: on 8-bit luma frames.
- ``envelope_cn``: C/N per capture bandwidth from |z|^2 constancy.
- ``phase_probe``: sampling-phase lottery score (fourth-difference power).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import dsp
from .standards import StandardParams


def cn_from_ratio(r):
    """Invert R = (c+1)^2/(2c+1) for c >= 0 (R <= 1 gives 0)."""
    r = np.asarray(r, np.float64)
    return np.where(r > 1, r - 1 + np.sqrt(np.maximum(r * r - r, 0)), 0.0)


def ratio_from_cn(c):
    c = np.asarray(c, np.float64)
    return (c + 1) ** 2 / (2 * c + 1)


def envelope_cn(z: np.ndarray, remove_dc: bool = True) -> float:
    """Linear C/N from one window of complex samples."""
    z = np.asarray(z, np.complex128)
    if remove_dc:
        z = z - z.mean()
    p = z.real ** 2 + z.imag ** 2
    v = p.var()
    if v <= 0:
        return float("inf")
    return float(cn_from_ratio(p.mean() ** 2 / v))


def envelope_cn_db(z, remove_dc=True) -> float:
    return float(10 * np.log10(max(envelope_cn(z, remove_dc), 1e-6)))


def phase_probe(z: np.ndarray, step: float) -> float:
    """Sampling-phase probe on a short burst at 40 MS/s: power passed by the
    fourth difference (1 -4 6 -4 1) over total power, each less its rounding
    share (quantiser step ``step``). Clean reads near 0; mixed lanes 19-80;
    threshold 12."""
    z = np.asarray(z, np.complex128)
    z = z - z.mean()
    q = 2 * step * step / 12.0
    d4 = z[4:] - 4 * z[3:-1] + 6 * z[2:-2] - 4 * z[1:-3] + z[:-4]
    p4 = np.mean(np.abs(d4) ** 2) - 70 * q
    p = np.mean(np.abs(z) ** 2) - q
    return float(p4 / p) if p > 0 else float("inf")


def video_band(std: StandardParams) -> float:
    return 4.2e6 if std.key == "ntsc" else 5.0e6


def active_cols(std: StandardParams, margin: int = 8) -> slice:
    a = std.active_range
    return slice(a.start + margin, a.stop - margin)


def active_line_mask(std: StandardParams, frame_lines: np.ndarray) -> np.ndarray:
    return np.array([fl > 0 and std.output_row(int(fl)) is not None
                     and std.slot_kind(2 * (int(fl) - 1) + 1) == "normal" for fl in frame_lines])


def band_limit_lines(lines: np.ndarray, std: StandardParams) -> np.ndarray:
    return dsp.filt(np.asarray(lines, np.float64), dsp.lowpass_taps(video_band(std), std.grid_rate_hz), axis=1)


@dataclass
class Comparison:
    """Decoded lines matched to truth lines (both on the grid, IRE)."""
    std: StandardParams
    dec: np.ndarray          # n x spl
    truth: np.ndarray        # n x spl (band-limited like the decoder)
    truth_index: np.ndarray  # truth line index per row
    label_ok: float          # share of matched lines with the right frame line
    coverage: float          # matched active lines / truth active lines in span


def match_lines(lines, truth_lines: np.ndarray, truth_frame_line: np.ndarray, line_a: float, line_b: float,
                delay: float = 0.0, offset: Optional[float] = None) -> Comparison:
    """Matches decoded lines to truth lines by their 0H position.

    ``offset`` (rx samples) is a constant timing offset to remove before
    rounding (the channel delay plus integration offsets); None estimates
    it as the median residual."""
    std = lines.std
    m = (lines.src_position - line_a - delay) / line_b
    if offset is None:
        offset = float(np.median(m - np.round(m)))
    mi = np.round(m - offset).astype(np.int64)
    ok = (mi >= 0) & (mi < len(truth_lines)) & (np.abs(m - offset - mi) < 0.25)
    fl_true = truth_frame_line[np.clip(mi, 0, len(truth_lines) - 1)]
    act = ok & active_line_mask(std, fl_true)
    label_ok = float(np.mean(lines.frame_line[act] == fl_true[act])) if act.any() else 0.0
    want = active_line_mask(std, truth_frame_line)
    span = (np.arange(len(truth_lines)) >= mi[ok].min()) & (np.arange(len(truth_lines)) <= mi[ok].max()) \
        if ok.any() else np.zeros(len(truth_lines), bool)
    coverage = act.sum() / max((want & span).sum(), 1)
    dec = np.asarray(lines.ire[act], np.float64)
    tru = band_limit_lines(truth_lines[mi[act]], std)
    return Comparison(std, dec, tru, mi[act], label_ok, float(coverage))


def calibrate(cmp: Comparison):
    """Gain/offset mapping decoded IRE onto truth (fit on a clean decode)."""
    c = active_cols(cmp.std)
    x = cmp.dec[:, c].ravel()
    y = cmp.truth[:, c].ravel()
    A = np.c_[x, np.ones_like(x)]
    g, o = np.linalg.lstsq(A, y, rcond=None)[0]
    return float(g), float(o)


def _err(cmp: Comparison, cal=(1.0, 0.0)):
    c = active_cols(cmp.std)
    g, o = cal
    return (g * cmp.dec[:, c] + o) - cmp.truth[:, c], cmp.truth[:, c]


def sinad_db(cmp: Comparison, cal=(1.0, 0.0)) -> float:
    e, t = _err(cmp, cal)
    return float(10 * np.log10(np.var(t) / max(np.mean(e * e), 1e-12)))


def wsnr_db(cmp: Comparison, cal=(1.0, 0.0)) -> float:
    e, _ = _err(cmp, cal)
    std = cmp.std
    ew = dsp.weighting_filter(e, std.grid_rate_hz, video_band(std), axis=1)
    return float(20 * np.log10(100.0 / max(np.sqrt(np.mean(ew * ew)), 1e-9)))


def line_sinad_db(cmp: Comparison, cal=(1.0, 0.0)) -> np.ndarray:
    """Per-line SINAD (for breakup statistics)."""
    e, t = _err(cmp, cal)
    return 10 * np.log10(np.var(t) / np.maximum(np.mean(e * e, axis=1), 1e-12))


def luma_frame(ire_frame: np.ndarray, std: StandardParams) -> np.ndarray:
    """Composite frame to 8-bit luma (chroma notched by a 3 MHz low-pass)."""
    a = std.active_range
    x = dsp.filt(np.nan_to_num(ire_frame[:, a.start:a.stop]), dsp.lowpass_taps(3.0e6, std.grid_rate_hz), axis=1)
    lo = std.black_ire
    y = (x - lo) / (std.white_ire - lo)
    return np.clip(y * 255 + 0.5, 0, 255).astype(np.uint8)


def psnr_ssim(a8: np.ndarray, b8: np.ndarray):
    from skimage.metrics import peak_signal_noise_ratio, structural_similarity
    return (float(peak_signal_noise_ratio(a8, b8, data_range=255)),
            float(structural_similarity(a8, b8, data_range=255)))


def comparison_luma_rows(cmp: Comparison):
    """Truth and decoded active rows as 8-bit luma images (rows in decode order)."""
    std = cmp.std
    def to8(x):
        y = dsp.filt(x, dsp.lowpass_taps(3.0e6, std.grid_rate_hz), axis=1)[:, active_cols(std)]
        return np.clip((y - std.black_ire) / (std.white_ire - std.black_ire) * 255 + 0.5, 0, 255).astype(np.uint8)
    return to8(cmp.dec), to8(cmp.truth)
