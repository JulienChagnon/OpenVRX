# SPDX-License-Identifier: GPL-3.0-or-later
"""Composite video (CVBS) raster on the line-locked grid.

Builds NTSC-M or PAL-B/G composite video in IRE directly on the grid of
``standards`` (910 or 1135 samples per line, sample 0 at 0H), so the
simulator's truth and the decoder's output share one coordinate system.

Structure follows BT.470 field-sync geometry (as C5VRX research/sim.py
``raster()``: pre-equalising, broad and post-equalising half-line slots;
NTSC 6/6/6 starting at frame line 1, PAL 5/5/5 with the broad pulses
starting at frame line 1). Every pulse has a raised-cosine edge (0.25 us)
centred on its nominal time, so the 50 % point of each falling edge sits
exactly on 0H or H/2. Colour: NTSC Y + U sin + V cos with burst at 180
degrees; PAL with the V switch and +-135 degree burst. The picture comes
from a ``PictureSource`` (test scenes, image files, video files).
"""
from __future__ import annotations

import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np

from . import dsp
from .standards import StandardParams

EDGE_US = 0.25
PICTURE_EDGE_US = 0.15
FRONT_PORCH_US = 1.5
SOURCE_BAND_HZ = 5.75e6     # grid raster band limit (VTX video bandwidth)
CHROMA_BAND_HZ = 1.3e6


def _rc_step(x, width):
    """0 below -width/2, 1 above +width/2, raised cosine in between."""
    y = np.clip(x / width, -0.5, 0.5)
    return 0.5 + 0.5 * np.sin(np.pi * y)


# ---------------------------------------------------------------------------
# Picture sources (RGB in [0, 1], frame_height x active_width)
# ---------------------------------------------------------------------------

class PictureSource:
    """Base class: ``frame(k)`` returns float RGB (H, W, 3) for frame k."""

    def __init__(self, height: int, width: int):
        self.height, self.width = height, width

    def frame(self, k: int) -> np.ndarray:
        raise NotImplementedError

    def describe(self) -> str:
        return type(self).__name__


def _resize(img: np.ndarray, h: int, w: int) -> np.ndarray:
    from skimage.transform import resize
    return np.clip(resize(img, (h, w), order=3, anti_aliasing=True, mode="reflect"), 0, 1)


class StillSource(PictureSource):
    def __init__(self, img: np.ndarray, height: int, width: int, pan_px_per_frame: float = 0.0, label="image"):
        super().__init__(height, width)
        img = np.asarray(img, dtype=np.float64)
        if img.max() > 1.5:
            img = img / 255.0
        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        self.base = _resize(img[:, :, :3], height, width)
        self.pan = float(pan_px_per_frame)
        self.label = label

    def frame(self, k: int) -> np.ndarray:
        if self.pan == 0.0:
            return self.base
        shift = self.pan * k
        i = int(np.floor(shift))
        f = shift - i
        a = np.roll(self.base, -i, axis=1)
        b = np.roll(self.base, -i - 1, axis=1)
        return (1 - f) * a + f * b

    def describe(self) -> str:
        return f"{self.label} pan={self.pan}"


def colour_bars(height: int, width: int) -> np.ndarray:
    """75 % colour bars (upper 2/3), luma ramp and a pluge row (lower third)."""
    img = np.zeros((height, width, 3))
    bars = [(0.75, 0.75, 0.75), (0.75, 0.75, 0), (0, 0.75, 0.75), (0, 0.75, 0),
            (0.75, 0, 0.75), (0.75, 0, 0), (0, 0, 0.75)]
    h1 = 2 * height // 3
    for j, c in enumerate(bars):
        a, b = j * width // 7, (j + 1) * width // 7
        img[:h1, a:b] = c
    ramp = np.linspace(0, 1, width)
    img[h1:h1 + (height - h1) // 2] = ramp[None, :, None]
    img[h1 + (height - h1) // 2:, : width // 3] = 0.0
    img[h1 + (height - h1) // 2:, width // 3: 2 * width // 3] = 1.0
    img[h1 + (height - h1) // 2:, 2 * width // 3:] = 0.04
    return img


def synthetic_scene(height: int, width: int, seed: int = 0) -> np.ndarray:
    """Textured test scene: sky/ground gradient, discs, stripes, fine detail
    and C5VRX-style random luma texture."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:height, 0:width] / np.array([height, width])[:, None, None]
    img = np.zeros((height, width, 3))
    sky = y < 0.45
    img[sky] = np.stack([0.35 + 0.2 * y[sky], 0.55 + 0.2 * y[sky], 0.9 - 0.1 * y[sky]], axis=-1)
    ground = ~sky
    tex = np.interp(np.arange(width), np.arange(0, width + 8, 8), rng.uniform(0.2, 0.8, width // 8 + 2))
    g = 0.3 + 0.4 * tex[None, :] * np.ones((height, 1))
    img[ground] = np.stack([0.25 + 0.3 * g[ground], 0.45 + 0.25 * g[ground], 0.2 + 0.1 * g[ground]], axis=-1)
    for _ in range(6):
        cx, cy, r = rng.uniform(0.1, 0.9), rng.uniform(0.15, 0.85), rng.uniform(0.04, 0.12)
        col = rng.uniform(0.05, 0.95, 3)
        m = (x - cx) ** 2 * (width / height) ** 2 + (y - cy) ** 2 < r * r
        img[m] = col
    stripes = (y > 0.8) & (x > 0.55)
    img[stripes] = (0.5 + 0.45 * np.sign(np.sin(2 * np.pi * x[stripes] * 60)))[:, None]
    return np.clip(img, 0, 1)


def _load_named_image(name: str) -> np.ndarray:
    import skimage.data
    if not hasattr(skimage.data, name):
        raise ValueError(f"no image named {name!r}")
    return getattr(skimage.data, name)()


class VideoSource(PictureSource):
    """Frames from a video file through ffmpeg (rgb24), looped."""

    def __init__(self, path: str, height: int, width: int, max_frames: int = 300):
        super().__init__(height, width)
        cmd = ["ffmpeg", "-v", "error", "-i", str(path), "-frames:v", str(max_frames),
               "-vf", f"scale={width}:{height}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
        data = subprocess.run(cmd, check=True, capture_output=True).stdout
        n = len(data) // (height * width * 3)
        if n == 0:
            raise ValueError(f"{path}: no frames decoded")
        self.frames = np.frombuffer(data[: n * height * width * 3], np.uint8).reshape(n, height, width, 3)
        self.path = path

    def frame(self, k: int) -> np.ndarray:
        return self.frames[k % len(self.frames)].astype(np.float64) / 255.0

    def describe(self) -> str:
        return f"video:{self.path}"


def make_picture_source(spec: str, std: StandardParams, seed: int = 0) -> PictureSource:
    """``bars``, ``scene``, ``image:<path or skimage name>[@pan]``,
    ``video:<path>``. ``@pan`` is a horizontal pan in pixels per frame."""
    h, w = std.frame_height, std.active_width
    pan = 0.0
    if "@" in spec and not spec.startswith("video:"):
        spec, p = spec.rsplit("@", 1)
        pan = float(p)
    if spec == "bars":
        return StillSource(colour_bars(h, w), h, w, pan, "bars")
    if spec == "scene":
        return StillSource(synthetic_scene(h, w, seed), h, w, pan, f"scene seed={seed}")
    if spec.startswith("image:"):
        ref = spec[len("image:"):]
        if Path(ref).exists():
            import imageio.v3 as iio
            img = iio.imread(ref)
        else:
            img = _load_named_image(ref)
        return StillSource(img, h, w, pan, spec)
    if spec.startswith("video:"):
        return VideoSource(spec[len("video:"):], h, w)
    raise ValueError(f"unknown picture source {spec!r}")


# ---------------------------------------------------------------------------
# Raster
# ---------------------------------------------------------------------------

def rgb_to_yuv(rgb: np.ndarray):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, 0.492 * (b - y), 0.877 * (r - y)


class Raster:
    """Line generator for one standard and picture source.

    ``lines(first_frame_line, count)`` returns ``count`` consecutive lines
    (IRE, shape count x samples_per_line); absolute line m = 0 is the line
    with frame line ``first_frame_line`` of frame 0.
    """

    def __init__(self, std: StandardParams, source: PictureSource, colour: bool = True,
                 band_limit: bool = True):
        self.std = std
        self.source = source
        self.colour = colour
        self.band_limit = band_limit
        self.spl = std.samples_per_line
        self._skeleton, self._burst_env, self._pic_win = self._build_skeleton()
        self._picture_cache = {}

    # -- static per-frame structure -----------------------------------------

    def _build_skeleton(self):
        std, spl = self.std, self.spl
        lpf = std.lines_per_frame
        total = lpf * spl
        sk = np.zeros(total)
        edge = std.us_to_samples(EDGE_US)
        widths = {"normal": std.us_to_samples(std.hsync_us), "eq": std.us_to_samples(std.eq_pulse_us),
                  "broad": std.us_to_samples(std.broad_pulse_us)}
        pulses = []
        for hl in range(2 * lpf):
            kind = std.slot_kind(hl)
            if kind == "normal" and hl % 2 == 1:
                continue
            start = hl * spl / 2.0
            pulses.append((start, start + widths[kind]))
        # Pulses as products of two shaped steps, evaluated circularly.
        for a, b in pulses:
            lo = int(np.floor(a - edge)) - 1
            hi = int(np.ceil(b + edge)) + 2
            idx = np.arange(lo, hi)
            shape = _rc_step(idx - a, edge) * (1.0 - _rc_step(idx - b, edge))
            sk[idx % total] += std.sync_ire * shape
        sk = sk.reshape(lpf, spl)

        # Burst envelope and picture window per frame line.
        burst = np.zeros((lpf, spl))
        win = np.zeros((lpf, spl))
        k = np.arange(spl)
        b0 = std.us_to_samples(std.burst_start_us)
        b1 = b0 + std.burst_cycles / std.fsc_per_sample
        ramp = 1.0 / std.fsc_per_sample
        benv = _rc_step(k - b0 - ramp / 2, ramp) * (1 - _rc_step(k - b1 + ramp / 2, ramp))
        a0 = std.active_range.start - 0.5
        a1 = std.active_range.stop - 0.5
        pe = std.us_to_samples(PICTURE_EDGE_US)
        half_end = spl / 2.0 - std.us_to_samples(FRONT_PORCH_US)
        for line in range(1, lpf + 1):
            first_normal = std.slot_kind(2 * (line - 1)) == "normal"
            second_normal = std.slot_kind(2 * (line - 1) + 1) == "normal"
            if first_normal:
                burst[line - 1] = benv
            if std.output_row(line) is not None and first_normal:
                end = a1 if second_normal else min(a1, half_end)
                win[line - 1] = _rc_step(k - a0 - pe / 2, pe) * (1 - _rc_step(k - end + pe / 2, pe))
        return sk, burst, win

    # -- picture -------------------------------------------------------------

    def _picture(self, frame_index: int):
        """Band-limited (Y_ire, U_ire, V_ire) rows for a frame, on grid columns."""
        if frame_index in self._picture_cache:
            return self._picture_cache[frame_index]
        std = self.std
        rgb = self.source.frame(frame_index)
        y, u, v = rgb_to_yuv(rgb)
        grid = std.grid_rate_hz
        luma_band = 4.2e6 if std.key == "ntsc" else 5.0e6
        y = dsp.filt(y, dsp.lowpass_taps(luma_band, grid), axis=1)
        u = dsp.filt(u, dsp.lowpass_taps(CHROMA_BAND_HZ, grid), axis=1)
        v = dsp.filt(v, dsp.lowpass_taps(CHROMA_BAND_HZ, grid), axis=1)
        if std.key == "ntsc":
            scale, setup = 92.5, 7.5
        else:
            scale, setup = 100.0, 0.0
        out = (setup + scale * y, scale * u, scale * v)
        if len(self._picture_cache) > 8:
            self._picture_cache.clear()
        self._picture_cache[frame_index] = out
        return out

    def subcarrier_phase(self, abs_line: np.ndarray) -> np.ndarray:
        """Subcarrier phase (radians) at every grid sample of absolute lines."""
        std = self.std
        cyc_per_line = std.fsc_hz / std.line_hz
        k = np.arange(self.spl)
        frac_line = np.mod(np.asarray(abs_line, np.float64) * cyc_per_line, 1.0)
        return 2 * np.pi * np.mod(frac_line[:, None] + k[None, :] * std.fsc_per_sample, 1.0)

    def lines(self, first_frame_line: int, count: int) -> np.ndarray:
        std, spl = self.std, self.spl
        lpf = std.lines_per_frame
        m = np.arange(count)
        frame_idx0 = (first_frame_line - 1 + m)
        frame_line = frame_idx0 % lpf + 1
        frame_no = frame_idx0 // lpf
        out = self._skeleton[frame_line - 1].copy()
        theta = self.subcarrier_phase(m)
        vsw = np.where(m % 2 == 0, 1.0, -1.0)[:, None]
        if self.colour:
            if std.key == "ntsc":
                burst = -std.burst_ire * np.sin(theta)
            else:
                burst = std.burst_ire * (-np.sin(theta) + vsw * np.cos(theta)) / np.sqrt(2.0)
            out += self._burst_env[frame_line - 1] * burst
        a = std.active_range.start
        w = std.active_width
        for j in range(count):
            row = std.output_row(int(frame_line[j]))
            win = self._pic_win[frame_line[j] - 1]
            if row is None or not win.any():
                continue
            yi, ui, vi = self._picture(int(frame_no[j]))
            pic = np.zeros(spl)
            pic[a:a + w] = yi[row]
            if self.colour:
                th = theta[j, a:a + w]
                if std.key == "ntsc":
                    pic[a:a + w] += ui[row] * np.sin(th) + vi[row] * np.cos(th)
                else:
                    pic[a:a + w] += ui[row] * np.sin(th) + vsw[j, 0] * vi[row] * np.cos(th)
            out[j] += win * pic
        if self.band_limit:
            flat = dsp.filt(out.ravel(), dsp.lowpass_taps(SOURCE_BAND_HZ, std.grid_rate_hz, 0.9e6))
            out = flat.reshape(count, spl)
        return out


@lru_cache(maxsize=4)
def _cached_source(spec: str, std_key: str, seed: int):
    from .standards import get_standard
    return make_picture_source(spec, get_standard(std_key), seed)


def make_raster(std: StandardParams, picture: str = "scene", seed: int = 0, colour: bool = True) -> Raster:
    return Raster(std, _cached_source(picture, std.key, seed), colour=colour)
