# SPDX-License-Identifier: GPL-3.0-or-later
"""Reader and writer for ``.openvrx`` recordings (version 1).

Implements ``docs/RECORDING_FORMAT.md``:

- ``NAME.openvrx.json``    sidecar (everything needed to decode)
- ``NAME.rx<k>.bin``     raw bytes per receiver, exactly as received
- ``NAME.events.jsonl``  one JSON object per line
- ``NAME.truth.f32``     simulator only: clean composite video, little-endian
                         f32 IRE, one value per input sample of the truth rx

Opening any of the files opens the recording.
"""
from __future__ import annotations

import datetime as _dt
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from . import __version__
from .lanes import SampleFormat, unpack, words_from_bytes

FORMAT_NAME = "openvrx-recording"
FORMAT_VERSION = 1
EVENT_KINDS = ("gain", "tune", "clk", "slip", "probe", "fx2_status", "usb_gap", "transfer", "marker", "c5")


def recording_base(path) -> Path:
    """``dir/NAME`` for any of a recording's files."""
    p = Path(path)
    name = p.name
    for suffix in (".openvrx.json", ".events.jsonl", ".truth.f32"):
        if name.endswith(suffix):
            return p.with_name(name[: -len(suffix)])
    if name.endswith(".bin") and ".rx" in name:
        return p.with_name(name[: name.rindex(".rx")])
    if name.endswith(".openvrx"):
        return p.with_name(name[: -len(".openvrx")])
    return p


@dataclass
class ReceiverInfo:
    rx: int
    file: str
    sample_format: SampleFormat
    bytes: int = 0
    sample_rate_hz: Optional[float] = None
    channel_mhz: float = 5732
    offset_khz: float = 0
    gain: int = 64
    p11: bool = False
    board: str = "sim"
    c5_fw: str = ""
    fx2_fw: str = ""
    fx2_serial: str = ""
    ifclk_inverted: bool = False
    watch_n: int = 1
    start_host_ns: int = 0
    antenna: str = ""
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.sample_rate_hz is None:
            self.sample_rate_hz = self.sample_format.sample_rate_hz

    def to_json(self) -> dict:
        d = {
            "rx": self.rx,
            "file": self.file,
            "bytes": int(self.bytes),
            "sample_format": self.sample_format.to_json(),
            "sample_rate_hz": float(self.sample_rate_hz),
            "channel_mhz": self.channel_mhz,
            "offset_khz": self.offset_khz,
            "gain": int(self.gain),
            "p11": bool(self.p11),
            "board": self.board,
            "c5_fw": self.c5_fw,
            "fx2_fw": self.fx2_fw,
            "fx2_serial": self.fx2_serial,
            "ifclk_inverted": bool(self.ifclk_inverted),
            "watch_n": int(self.watch_n),
            "start_host_ns": int(self.start_host_ns),
            "antenna": self.antenna,
        }
        d.update(self.extra)
        return d

    @staticmethod
    def from_json(d: dict) -> "ReceiverInfo":
        known = {"rx", "file", "bytes", "sample_format", "sample_rate_hz", "channel_mhz", "offset_khz", "gain",
                 "p11", "board", "c5_fw", "fx2_fw", "fx2_serial", "ifclk_inverted", "watch_n", "start_host_ns",
                 "antenna"}
        return ReceiverInfo(
            rx=int(d["rx"]), file=d["file"], sample_format=SampleFormat.from_json(d["sample_format"]),
            bytes=int(d.get("bytes", 0)), sample_rate_hz=d.get("sample_rate_hz"),
            channel_mhz=d.get("channel_mhz", 0), offset_khz=d.get("offset_khz", 0), gain=d.get("gain", 0),
            p11=bool(d.get("p11", False)), board=d.get("board", ""), c5_fw=d.get("c5_fw", ""),
            fx2_fw=d.get("fx2_fw", ""), fx2_serial=d.get("fx2_serial", ""),
            ifclk_inverted=bool(d.get("ifclk_inverted", False)), watch_n=int(d.get("watch_n", 1)),
            start_host_ns=int(d.get("start_host_ns", 0)), antenna=d.get("antenna", ""),
            extra={k: v for k, v in d.items() if k not in known})


@dataclass
class Recording:
    """An opened recording. Raw bytes are loaded lazily per receiver."""
    base: Path
    sidecar: dict
    receivers: list

    @property
    def name(self) -> str:
        return self.base.name

    @property
    def standard_hint(self) -> Optional[str]:
        return self.sidecar.get("standard_hint")

    @property
    def sim(self) -> Optional[dict]:
        return self.sidecar.get("sim")

    def path_of(self, file: str) -> Path:
        return self.base.parent / file

    def raw(self, rx: int = 0) -> np.ndarray:
        info = self.receivers[rx]
        return np.fromfile(self.path_of(info.file), dtype=np.uint8)

    def words(self, rx: int = 0) -> np.ndarray:
        return words_from_bytes(self.raw(rx), self.receivers[rx].sample_format.lanes.word_bytes)

    def iq(self, rx: int = 0) -> np.ndarray:
        return unpack(self.raw(rx), self.receivers[rx].sample_format.lanes)

    def gaps(self, rx: int = 0) -> list:
        """``usb_gap`` events of one receiver as (file_byte, lost_bytes), sorted.

        ``file_byte`` is the offset in the .bin where the lost bytes would have
        been; ``lost_bytes`` were dropped by the host before that offset."""
        out = [(int(e["byte"]), int(e.get("lost_bytes", 0))) for e in self.events()
               if e.get("kind") == "usb_gap" and int(e.get("rx", 0)) == rx]
        return sorted(out)

    def segments(self, rx: int = 0, min_words: int = 0) -> list:
        """Contiguous runs of samples between ``usb_gap`` events.

        Returns [(absolute_word_index, iq complex64)], where the absolute index
        counts the dropped words too, so sample times stay exact across gaps.
        Runs shorter than ``min_words`` are left out."""
        info = self.receivers[rx]
        wb = info.sample_format.lanes.word_bytes
        raw = self.raw(rx)
        lut = info.sample_format.lanes.build_lut()
        cuts, lost_before = [0], [0]
        total = 0
        for byte, lost in self.gaps(rx):
            if byte % wb or lost % wb:
                raise ValueError(f"usb_gap at {byte} (+{lost}) is not word aligned")
            total += lost
            if byte == cuts[-1]:
                lost_before[-1] = total
                continue
            cuts.append(byte)
            lost_before.append(total)
        cuts.append(len(raw))
        out = []
        for a, b, lost in zip(cuts[:-1], cuts[1:], lost_before):
            if b - a < max(min_words, 1) * wb:
                continue
            words = words_from_bytes(raw[a:b], wb)
            out.append(((a + lost) // wb, lut[words]))
        return out

    def events(self) -> list:
        p = self.base.parent / f"{self.base.name}.events.jsonl"
        if not p.exists():
            return []
        out = []
        for line in p.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                out.append(json.loads(line))
        return out

    def truth(self) -> Optional[np.ndarray]:
        sim = self.sim
        if not sim or not sim.get("truth"):
            return None
        return np.fromfile(self.path_of(sim["truth"]["file"]), dtype="<f4")

    @property
    def truth_info(self) -> Optional[dict]:
        sim = self.sim
        return sim.get("truth") if sim else None


def open_recording(path) -> Recording:
    base = recording_base(path)
    side = base.parent / f"{base.name}.openvrx.json"
    sidecar = json.loads(side.read_text(encoding="utf-8"))
    if sidecar.get("format") != FORMAT_NAME:
        raise ValueError(f"{side}: not a {FORMAT_NAME} sidecar")
    if int(sidecar.get("version", 0)) != FORMAT_VERSION:
        raise ValueError(f"{side}: unsupported version {sidecar.get('version')}")
    receivers = [ReceiverInfo.from_json(r) for r in sidecar["receivers"]]
    receivers.sort(key=lambda r: r.rx)
    return Recording(base, sidecar, receivers)


def utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class RecordingWriter:
    """Writes a recording: sidecar at start, raw per receiver, events, truth.

    Usage::

        w = RecordingWriter("out/name", receivers=[info0, info1], source="sim", sim={...})
        w.write_raw(0, bytes0); w.event(0, "gain", index=64)
        w.write_truth(truth_f32); w.close()
    """

    def __init__(self, base, receivers, source="sim", notes="", standard_hint=None, sim=None,
                 software=None, created_utc=None):
        self.base = recording_base(base)
        self.base.parent.mkdir(parents=True, exist_ok=True)
        self.receivers = list(receivers)
        for r in self.receivers:
            if not r.file:
                r.file = f"{self.base.name}.rx{r.rx}.bin"
            r.bytes = 0
        self.sidecar = {
            "format": FORMAT_NAME,
            "version": FORMAT_VERSION,
            "created_utc": created_utc or utc_now(),
            "software": software or f"openvrx-lab {__version__}",
            "source": source,
            "notes": notes,
            "closed_cleanly": False,
            "receivers": [],
            "standard_hint": standard_hint,
            "sim": sim,
        }
        self._raw = {r.rx: open(self.base.parent / r.file, "wb") for r in self.receivers}
        self._events = open(self.base.parent / f"{self.base.name}.events.jsonl", "w", encoding="utf-8")
        self._write_sidecar()

    @property
    def sidecar_path(self) -> Path:
        return self.base.parent / f"{self.base.name}.openvrx.json"

    def _write_sidecar(self):
        self.sidecar["receivers"] = [r.to_json() for r in self.receivers]
        self.sidecar_path.write_text(json.dumps(self.sidecar, indent=2), encoding="utf-8")

    def write_raw(self, rx: int, data) -> None:
        b = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
        self._raw[rx].write(b)
        for r in self.receivers:
            if r.rx == rx:
                r.bytes += len(b)

    def event(self, rx, kind: str, host_ns: int = 0, byte: int = 0, **fields) -> None:
        if kind not in EVENT_KINDS:
            raise ValueError(f"unknown event kind {kind!r}")
        ev = {"rx": rx, "host_ns": int(host_ns), "kind": kind, "byte": int(byte)}
        ev.update(fields)
        self._events.write(json.dumps(ev) + "\n")
        self._events.flush()

    def write_truth(self, truth: np.ndarray, rx: int = 0, delay_samples: float = 0.0) -> None:
        if self.sidecar["sim"] is None:
            self.sidecar["sim"] = {}
        name = f"{self.base.name}.truth.f32"
        np.asarray(truth, dtype="<f4").tofile(self.base.parent / name)
        self.sidecar["sim"]["truth"] = {"file": name, "rx": rx, "unit": "ire",
                                        "delay_samples": float(delay_samples)}

    def close(self) -> Recording:
        for f in self._raw.values():
            f.close()
        self._events.close()
        self.sidecar["closed_cleanly"] = True
        self._write_sidecar()
        return open_recording(self.sidecar_path)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if not self._events.closed:
            self.close()
