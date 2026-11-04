# SPDX-License-Identifier: GPL-3.0-or-later
"""S0: decoder settings check (so no format is judged with a handicapped
decoder). Digital channel filter half-bandwidth and click threshold, per rate.
"""
from __future__ import annotations

import numpy as np

from .. import decoder, simgen
from . import common as C

CNS = (2, 4, 6, 8, 10, 14)
CASES = [("8x8", 18), ("4x4top", 12), ("4x4top", 9), ("4x4top", 6)]
VARIANTS = {
    "default": {},
    "no channel filter": {"channel_filter_hz": 0.0},
    "channel 6.5 MHz": {"channel_filter_hz": 6.5e6},
    "channel 9 MHz": {"channel_filter_hz": 9.0e6},
    "no click repair": {"click_threshold_hz": 0.0, "wrap_repair": False},
    "click 3.5 MHz": {"click_threshold_hz": 3.5e6},
    "click 6 MHz": {"click_threshold_hz": 6.0e6},
}


def _job(args):
    dev, seed, preset, div = args
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", C.PICTURE, dev, seed=seed), 0.05)
    cap = simgen.generate(tx, simgen.RxConfig(seed=100 + seed), [div], 0.05)[div]
    out = []
    for name, kw in VARIANTS.items():
        if "channel" in name and cap.fs < 19e6:
            continue
        dcfg = decoder.DecoderConfig(**kw)
        _, cal = C.evaluate(tx, cap, simgen.ReceiverRun(preset, div, 60.0), dcfg)
        for cn in CNS:
            d, _ = C.evaluate(tx, cap, simgen.ReceiverRun(preset, div, cn), dcfg, cal)
            out.append(dict(dev=dev, seed=seed, preset=preset, div=div, variant=name, cn=cn, **d))
    return out


def run(seeds=(0, 1), devs=(1.0, 1.4)):
    jobs = [(dev, s, p, d) for dev in devs for s in seeds for p, d in CASES]
    rows = [r for part in C.pmap(_job, jobs) for r in part]
    C.save_json("S0_decoder.json", rows)
    lines = ["# S0: decoder settings [sim]", "",
             "Luma SINAD (dB, median over seeds) after the full reference decode, NTSC, astronaut picture.",
             "Default decoder: channel filter 7.5 MHz half-band at >= 20 MS/s (off below), "
             "wrap/click repair over one 75 ns group, single-sample click threshold 4.5 MHz.", ""]
    for dev in devs:
        for p, d in CASES:
            hdr = ["variant"] + [f"{c} dB" for c in CNS]
            tab = []
            for name in VARIANTS:
                sel = [r for r in rows if r["dev"] == dev and r["preset"] == p and r["div"] == d and r["variant"] == name]
                if not sel:
                    continue
                tab.append([name] + [C.fmt_db(np.nanmedian([r["sinad"] for r in sel if r["cn"] == c])) for c in CNS])
            lines += [f"## {p} /{d} ({240 / d:.2f} MS/s), deviation {dev}", "", C.md_table(hdr, tab), ""]
    (C.RESULTS / "S0_decoder.md").write_text("\n".join(lines), encoding="utf-8")
    return rows
