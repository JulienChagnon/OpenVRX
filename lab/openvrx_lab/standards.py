# SPDX-License-Identifier: GPL-3.0-or-later
"""Analog video raster geometry.

Exact mirror of ``host/crates/openvrx-core/src/standard.rs``.

Every line is resampled (time-base corrected) onto a line-locked grid of
``samples_per_line`` samples, sample 0 at 0H (the 50 % point of the falling
edge of horizontal sync).

- NTSC-M: 910 samples per line = exactly 4 x fsc (14.318181 MHz).
- PAL-B/G: 1135 samples per line (17.734375 MHz); the subcarrier slips by
  0.0064 samples a line on this grid.

Frame lines are 1-based (NTSC 1..=525, PAL 1..=625). Field 1 is frame lines
1..=263 (NTSC) or 1..=313 (PAL), field 2 the rest.

Rounding: Rust's ``f64::round`` rounds half away from zero; ``_round`` does
the same (Python's ``round`` would round half to even).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

FIRST = 0   # FieldParity::First
SECOND = 1  # FieldParity::Second


def _round(x: float) -> int:
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


@dataclass(frozen=True)
class StandardParams:
    name: str
    line_hz: float
    fsc_hz: float
    lines_per_frame: int
    lines_per_field: tuple
    samples_per_line: int
    hsync_us: float
    eq_pulse_us: float
    broad_pulse_us: float
    burst_start_us: float
    burst_cycles: float
    active_start_us: float
    active_us: float
    sync_ire: float
    black_ire: float
    white_ire: float
    burst_ire: float
    active_lines_per_field: int
    first_active_line: tuple
    top_field: int
    field_hz: float

    @property
    def key(self) -> str:
        return self.name.lower()

    @property
    def grid_rate_hz(self) -> float:
        return self.samples_per_line * self.line_hz

    @property
    def line_period_s(self) -> float:
        return 1.0 / self.line_hz

    def us_to_samples(self, us: float) -> float:
        return us * 1e-6 * self.grid_rate_hz

    @property
    def fsc_per_sample(self) -> float:
        return self.fsc_hz / self.grid_rate_hz

    @property
    def hsync_samples(self) -> int:
        return _round(self.us_to_samples(self.hsync_us))

    @property
    def burst_range(self) -> range:
        start = _round(self.us_to_samples(self.burst_start_us))
        n = _round(self.burst_cycles / self.fsc_per_sample)
        return range(start, start + n)

    @property
    def active_range(self) -> range:
        start = _round(self.us_to_samples(self.active_start_us))
        n = _round(self.us_to_samples(self.active_us))
        return range(start, min(start + n, self.samples_per_line))

    @property
    def active_width(self) -> int:
        return len(self.active_range)

    @property
    def back_porch_range(self) -> range:
        b = self.burst_range
        a = self.active_range
        return range(b.stop + 2, max(max(a.start - 2, 0), b.stop + 3))

    @property
    def sync_tip_range(self) -> range:
        w = self.hsync_samples
        return range(w // 6, w - w // 6)

    @property
    def frame_height(self) -> int:
        return 2 * self.active_lines_per_field

    @property
    def display_aspect(self) -> float:
        return 4.0 / 3.0

    def field_lines(self, parity: int) -> int:
        return self.lines_per_field[parity]

    def field_first_frame_line(self, parity: int) -> int:
        return 1 if parity == FIRST else self.lines_per_field[0] + 1

    def frame_line(self, parity: int, field_line: int) -> int:
        return self.field_first_frame_line(parity) + field_line

    def parity_of(self, frame_line: int) -> int:
        return FIRST if frame_line <= self.lines_per_field[0] else SECOND

    def output_row(self, frame_line: int) -> Optional[int]:
        parity = self.parity_of(frame_line)
        first = self.first_active_line[parity]
        if frame_line < first:
            return None
        k = frame_line - first
        if k >= self.active_lines_per_field:
            return None
        offset = 0 if parity == self.top_field else 1
        return 2 * k + offset

    def field_row_offset(self, parity: int) -> int:
        return 0 if parity == self.top_field else 1

    def first_active_field_line(self, parity: int) -> int:
        return self.first_active_line[parity] - self.field_first_frame_line(parity)

    # --- Raster structure helpers (not in the Rust core; used by the
    # simulator and the decoder's field detection). ------------------------

    @property
    def vbi_slots(self) -> tuple:
        """(pre-equalising, broad, post-equalising) half-line slot counts."""
        return (6, 6, 6) if self.key == "ntsc" else (5, 5, 5)

    @property
    def vbi_phase(self) -> int:
        """Half-line offset so that slot ``(hl + phase) % half_lines_per_field``
        counts from the first pre-equalising slot (hl = 2*(line-1)+half)."""
        return 0 if self.key == "ntsc" else 5

    @property
    def half_lines_per_field(self) -> int:
        return self.lines_per_frame  # 525 or 625 half lines per field

    def slot_kind(self, half_line: int) -> str:
        """'eq', 'broad' or 'normal' for a frame half-line slot (0-based)."""
        pos = (half_line + self.vbi_phase) % self.half_lines_per_field
        pre, broad, post = self.vbi_slots
        if pos < pre:
            return "eq"
        if pos < pre + broad:
            return "broad"
        if pos < pre + broad + post:
            return "eq"
        return "normal"

    @property
    def broad_anchor_lines(self) -> tuple:
        """Frame line whose first (field 1) or second (field 2) half holds
        the first broad pulse of each vertical interval."""
        return (4, 266) if self.key == "ntsc" else (1, 313)


NTSC = StandardParams(
    name="NTSC",
    line_hz=4.5e6 / 286.0,
    fsc_hz=315.0e6 / 88.0,
    lines_per_frame=525,
    lines_per_field=(263, 262),
    samples_per_line=910,
    hsync_us=4.7,
    eq_pulse_us=2.3,
    broad_pulse_us=27.1,
    burst_start_us=5.3,
    burst_cycles=9.0,
    active_start_us=9.4,
    active_us=52.66,
    sync_ire=-40.0,
    black_ire=7.5,
    white_ire=100.0,
    burst_ire=20.0,
    active_lines_per_field=240,
    first_active_line=(21, 283),
    top_field=SECOND,
    field_hz=60000.0 / 1001.0,
)

PAL = StandardParams(
    name="PAL",
    line_hz=15625.0,
    fsc_hz=4_433_618.75,
    lines_per_frame=625,
    lines_per_field=(313, 312),
    samples_per_line=1135,
    hsync_us=4.7,
    eq_pulse_us=2.35,
    broad_pulse_us=27.3,
    burst_start_us=5.6,
    burst_cycles=10.0,
    active_start_us=10.4,
    active_us=52.0,
    sync_ire=-300.0 / 7.0,
    black_ire=0.0,
    white_ire=100.0,
    burst_ire=150.0 / 7.0,
    active_lines_per_field=288,
    first_active_line=(23, 336),
    top_field=FIRST,
    field_hz=50.0,
)

STANDARDS = {"ntsc": NTSC, "pal": PAL}


def get_standard(name: str) -> StandardParams:
    try:
        return STANDARDS[name.lower()]
    except KeyError:
        raise ValueError(f"unknown video standard `{name}` (ntsc|pal)") from None
