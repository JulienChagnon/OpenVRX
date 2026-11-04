# SPDX-License-Identifier: GPL-3.0-or-later
"""Lane formats: which C5 ADC bits travel on which FX2 port bits.

Exact mirror of ``host/crates/openvrx-core/src/lanes.rs``. Every function
here must give bit-identical words and float32-identical decoded values to
the Rust implementation; ``tests/test_lanes.py`` checks the values used by
the Rust unit tests.

The C5's MODEM_DIAG bus carries Q on bits 0..9 and I on bits 10..19, each a
10-bit two's complement ADC value. A component is described by the ADC bits
it carries (most significant first) and the word bit holding the first of
them. The carried bits are read back as an n-bit two's complement number
``t`` and scaled to 10-bit ADC units as ``t * 2^lsb + 2^lsb / 2``. For bits
contiguous from the sign down this is truncation; for a gap after the sign
bit (the folded {9,6,5,4} slice used by C5VRX, "ultrafine") it is a folded
window of +-2^(lsb+n-1) ADC units.

16-bit words: Q on the low byte (FX2 PB0..PB7), I on the high byte
(PD0..PD7), MSB aligned, unused low lanes tied to GND. 8-bit words with
4+4 bits: Q on FD0..FD3, I on FD4..FD7.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

import numpy as np

C5_PLL_HZ = 240e6
C5_BUS_HZ = 80e6

_TAU32 = np.float32(2.0 * np.pi)


class LaneFormatError(ValueError):
    """Invalid lane format (mirrors CoreError::LaneFormat)."""


@dataclass(frozen=True)
class ComponentLanes:
    word_msb: int
    adc_bits: tuple

    def __post_init__(self):
        object.__setattr__(self, "adc_bits", tuple(int(b) for b in self.adc_bits))

    @property
    def width(self) -> int:
        return len(self.adc_bits)

    @property
    def word_lsb(self) -> int:
        return self.word_msb + 1 - len(self.adc_bits)

    @property
    def word_mask(self) -> int:
        return ((1 << self.width) - 1) << self.word_lsb

    @property
    def adc_lsb(self) -> int:
        return self.adc_bits[-1]

    def decode(self, word):
        """Carried bits as two's complement, scaled to ADC units (float32)."""
        n = self.width
        w = np.asarray(word, dtype=np.int64)
        raw = (w >> self.word_lsb) & ((1 << n) - 1)
        t = np.where(raw >= (1 << (n - 1)), raw - (1 << n), raw)
        step = np.float32(1 << self.adc_lsb)
        out = t.astype(np.float32) * step + step * np.float32(0.5)
        return out if out.ndim else np.float32(out)

    def encode(self, adc10):
        """Places the carried bits of 10-bit ADC values into the word."""
        v = np.clip(np.asarray(adc10, dtype=np.int64), -512, 511) & 0x3FF
        out = np.zeros(v.shape, dtype=np.int64)
        wb = self.word_msb
        for b in self.adc_bits:
            out |= ((v >> b) & 1) << wb
            wb -= 1
        return out if out.ndim else int(out)

    def to_json(self) -> dict:
        return {"word_msb": self.word_msb, "adc_bits": list(self.adc_bits)}

    @staticmethod
    def from_json(d: dict) -> "ComponentLanes":
        return ComponentLanes(int(d["word_msb"]), tuple(d["adc_bits"]))


@dataclass(frozen=True)
class IqEncoding:
    i: ComponentLanes
    q: ComponentLanes
    kind: str = "iq"


@dataclass(frozen=True)
class AngleEncoding:
    word_msb: int
    bits: int
    kind: str = "angle"


Encoding = Union[IqEncoding, AngleEncoding]


@dataclass(frozen=True)
class LaneFormat:
    word_bytes: int
    encoding: Encoding

    ANGLE_MAGNITUDE = np.float32(64.0)

    def validate(self) -> None:
        if self.word_bytes not in (1, 2):
            raise LaneFormatError("word_bytes must be 1 or 2")
        word_bits = 8 * self.word_bytes

        def check(c: ComponentLanes, name: str) -> int:
            if not c.adc_bits or len(c.adc_bits) > 10:
                raise LaneFormatError(f"{name}: 1..=10 bits")
            if any(a <= b for a, b in zip(c.adc_bits, c.adc_bits[1:])) or c.adc_bits[0] > 9:
                raise LaneFormatError(f"{name}: ADC bits must be strictly descending and <= 9")
            if c.word_msb >= word_bits or c.word_msb + 1 < len(c.adc_bits):
                raise LaneFormatError(f"{name}: does not fit the word")
            return c.word_mask

        enc = self.encoding
        if isinstance(enc, IqEncoding):
            if check(enc.i, "I") & check(enc.q, "Q"):
                raise LaneFormatError("I and Q lanes overlap")
        else:
            if enc.bits == 0 or enc.bits > word_bits or enc.word_msb >= word_bits:
                raise LaneFormatError("angle does not fit the word")

    @property
    def has_magnitude(self) -> bool:
        return isinstance(self.encoding, IqEncoding)

    def decode(self, word) -> np.ndarray:
        """Words to complex64 samples in 10-bit ADC units."""
        enc = self.encoding
        if isinstance(enc, IqEncoding):
            re = enc.i.decode(word)
            im = enc.q.decode(word)
            out = np.empty(np.shape(re), dtype=np.complex64)
            out.real = re
            out.imag = im
            return out if out.ndim else np.complex64(out)
        n = enc.bits
        lsb = enc.word_msb + 1 - n
        w = np.asarray(word, dtype=np.int64)
        raw = (w >> lsb) & ((1 << n) - 1)
        a = (raw.astype(np.float32) + np.float32(0.5)) * _TAU32 / np.float32(1 << n)
        out = np.empty(np.shape(a), dtype=np.complex64)
        out.real = self.ANGLE_MAGNITUDE * np.cos(a)
        out.imag = self.ANGLE_MAGNITUDE * np.sin(a)
        return out if out.ndim else np.complex64(out)

    def encode(self, i_adc10, q_adc10):
        """10-bit ADC values (or, for angle formats, their angle) to words."""
        enc = self.encoding
        if isinstance(enc, IqEncoding):
            return enc.i.encode(i_adc10) | enc.q.encode(q_adc10)
        n = enc.bits
        lsb = enc.word_msb + 1 - n
        i32 = np.asarray(i_adc10, dtype=np.int64).astype(np.float32)
        q32 = np.asarray(q_adc10, dtype=np.int64).astype(np.float32)
        a = np.arctan2(q32, i32).astype(np.float32)
        # f32::rem_euclid(TAU)
        r = np.fmod(a, _TAU32)
        r = np.where(r < 0, r + _TAU32, r).astype(np.float32)
        scaled = (r / _TAU32) * np.float32(1 << n)
        code = scaled.astype(np.float32).astype(np.int64) & ((1 << n) - 1)
        out = code << lsb
        return out if np.ndim(out) else int(out)

    def build_lut(self) -> np.ndarray:
        n = 1 << (8 * self.word_bytes)
        return self.decode(np.arange(n, dtype=np.int64))

    @property
    def used_word_mask(self) -> int:
        enc = self.encoding
        if isinstance(enc, IqEncoding):
            return enc.i.word_mask | enc.q.word_mask
        return ((1 << enc.bits) - 1) << (enc.word_msb + 1 - enc.bits)

    @property
    def component_full_scale(self) -> float:
        enc = self.encoding
        if isinstance(enc, IqEncoding):
            return float(1 << (enc.i.adc_lsb + enc.i.width - 1))
        return float(self.ANGLE_MAGNITUDE)

    # serde form of openvrx_core::LaneFormat
    def to_json(self) -> dict:
        enc = self.encoding
        if isinstance(enc, IqEncoding):
            e = {"kind": "iq", "i": enc.i.to_json(), "q": enc.q.to_json()}
        else:
            e = {"kind": "angle", "word_msb": enc.word_msb, "bits": enc.bits}
        return {"word_bytes": self.word_bytes, "encoding": e}

    @staticmethod
    def from_json(d: dict) -> "LaneFormat":
        e = d["encoding"]
        if e["kind"] == "iq":
            enc = IqEncoding(ComponentLanes.from_json(e["i"]), ComponentLanes.from_json(e["q"]))
        elif e["kind"] == "angle":
            enc = AngleEncoding(int(e["word_msb"]), int(e["bits"]))
        else:
            raise LaneFormatError(f"unknown encoding kind {e['kind']!r}")
        f = LaneFormat(int(d["word_bytes"]), enc)
        f.validate()
        return f


def _iq16(n: int) -> LaneFormat:
    bits = tuple(9 - k for k in range(n))
    return LaneFormat(2, IqEncoding(ComponentLanes(15, bits), ComponentLanes(7, bits)))


def _iq8(bits) -> LaneFormat:
    return LaneFormat(1, IqEncoding(ComponentLanes(7, tuple(bits)), ComponentLanes(3, tuple(bits))))


PRESETS = {
    "8x8": _iq16(8),
    "7x7": _iq16(7),
    "6x6": _iq16(6),
    "5x5": _iq16(5),
    "4x4top": _iq8((9, 8, 7, 6)),
    "4x4fine": _iq8((9, 7, 6, 5)),
    "4x4ultra": _iq8((9, 6, 5, 4)),
    "angle8": LaneFormat(1, AngleEncoding(7, 8)),
}
PRESET_NAMES = tuple(PRESETS)


def preset_format(name: str) -> LaneFormat:
    for k, v in PRESETS.items():
        if k.lower() == name.lower():
            return v
    raise LaneFormatError(f"unknown preset `{name}`")


def canonical_preset(name: str) -> str:
    for k in PRESETS:
        if k.lower() == name.lower():
            return k
    raise LaneFormatError(f"unknown preset `{name}`")


def preset_data_pins(name: str) -> int:
    enc = preset_format(name).encoding
    if isinstance(enc, IqEncoding):
        return enc.i.width + enc.q.width
    return enc.bits


def preset_for_format(fmt: LaneFormat) -> Optional[str]:
    for k, v in PRESETS.items():
        if v == fmt:
            return k
    return None


@dataclass(frozen=True)
class SampleFormat:
    preset: Optional[str]
    lanes: LaneFormat
    divider: int

    @staticmethod
    def from_preset(preset: str, divider: int) -> "SampleFormat":
        name = canonical_preset(preset)
        return SampleFormat(name, PRESETS[name], int(divider))

    @property
    def sample_rate_hz(self) -> float:
        return C5_PLL_HZ / self.divider

    @property
    def byte_rate(self) -> float:
        return self.sample_rate_hz * self.lanes.word_bytes

    @property
    def bus_locked(self) -> bool:
        return self.divider % 3 == 0

    @property
    def bus_decimation(self) -> float:
        return C5_BUS_HZ / self.sample_rate_hz

    @property
    def data_pins(self) -> int:
        enc = self.lanes.encoding
        if isinstance(enc, IqEncoding):
            return enc.i.width + enc.q.width
        return enc.bits

    def to_json(self) -> dict:
        return {"preset": self.preset, "lanes": self.lanes.to_json(), "divider": self.divider}

    @staticmethod
    def from_json(d: dict) -> "SampleFormat":
        lanes = LaneFormat.from_json(d["lanes"])
        preset = d.get("preset")
        if preset is not None:
            preset = canonical_preset(preset)
        return SampleFormat(preset, lanes, int(d["divider"]))


def words_from_bytes(raw: bytes | np.ndarray, word_bytes: int) -> np.ndarray:
    """Raw USB bytes to words (16-bit words are little endian: FD0..7 first)."""
    b = np.frombuffer(raw, dtype=np.uint8) if not isinstance(raw, np.ndarray) else raw.view(np.uint8)
    if word_bytes == 1:
        return b
    n = len(b) // 2
    return b[: 2 * n].view("<u2")


def words_to_bytes(words: np.ndarray, word_bytes: int) -> bytes:
    if word_bytes == 1:
        return np.asarray(words, dtype=np.uint8).tobytes()
    return np.asarray(words, dtype="<u2").tobytes()


def unpack(raw, fmt: LaneFormat, lut: Optional[np.ndarray] = None) -> np.ndarray:
    """Raw bytes to complex64 I/Q through the decode table."""
    words = words_from_bytes(raw, fmt.word_bytes)
    if lut is None:
        lut = fmt.build_lut()
    return lut[words]
