# SPDX-License-Identifier: GPL-3.0-or-later
"""S1c: per-sample PLL demodulator against the polar discriminator, per rate.

Checks the community FPGA result (same 4-bit I/Q, about 3 dB from rate plus
a PLL demod running on every sample) inside our chain. The PLL is a
second-order loop with an arctangent phase detector (decoder._pll), natural
frequency swept; everything after the demod (repair, video filter, sync,
TBC) is the same as for the polar discriminator.
"""
from __future__ import annotations

import numpy as np

from .. import decoder, simgen
from . import common as C

CNS = tuple(range(4, 17))
# 80 MS/s (divider 3, every bus value) is out of reach for one FX2; it is
# here as the FPGA's operating point.
CASES = [("4x4top", 9), ("4x4top", 6), ("4x4top", 3)]
VARIANTS = {"polar": {}, "PLL 6 MHz": {"demod": "pll", "pll_bw_hz": 6.0e6},
            "PLL 8 MHz": {"demod": "pll", "pll_bw_hz": 8.0e6}}
DURATION = 0.05


def _job(args):
    dev, seed, div, presets = args
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", C.PICTURE, dev, seed=seed), DURATION)
    cap = simgen.generate(tx, simgen.RxConfig(seed=600 + seed), [div], DURATION)[div]
    out = []
    for p in presets:
        _, cal = C.evaluate(tx, cap, simgen.ReceiverRun(p, div, 60.0))
        for name, kw in VARIANTS.items():
            dcfg = decoder.DecoderConfig(**kw)
            for cn in CNS + (60.0,):
                d, _ = C.evaluate(tx, cap, simgen.ReceiverRun(p, div, float(cn)), dcfg, cal)
                out.append(dict(dev=dev, seed=seed, preset=p, div=div, variant=name, cn=float(cn), **d))
    return out


def run(seeds=(0, 1), devs=(1.0,)):
    jobs = []
    for dev in devs:
        for s in seeds:
            for d in sorted({d for _, d in CASES}):
                jobs.append((dev, s, d, tuple(p for p, dd in CASES if dd == d)))
    rows = [r for part in C.pmap_cached(_job, jobs, "s1c") for r in part]
    C.save_json("S1c_rows.json", rows)
    write(rows)
    return rows


def write(rows):
    L = ["# S1c: PLL demod against the polar discriminator [sim]", "",
         f"NTSC, astronaut, deviation 1.0, {len({r['seed'] for r in rows})} seeds x {DURATION} s. Cells: C/N at 1 % "
         "breakup / C/N at SINAD 10 dB / noise-free SINAD (medians over seeds). Lower C/N is better.", ""]
    hdr = ["format", "rate"] + list(VARIANTS)
    tab = []
    for p, d in CASES:
        row = [p, f"{240 / d:.2f} MS/s"]
        for name in VARIANTS:
            tb, t10, ce = [], [], []
            for s in sorted({r["seed"] for r in rows}):
                rs = sorted([r for r in rows if r["preset"] == p and r["div"] == d and r["variant"] == name
                             and r["seed"] == s and r["cn"] < 59], key=lambda r: r["cn"])
                if not rs:
                    continue
                cn = [r["cn"] for r in rs]
                tb.append(C.threshold_down(cn, [r["breakup"] for r in rs], 0.01))
                t10.append(C.threshold(cn, [r["sinad"] for r in rs], 10.0))
                ce += [r["sinad"] for r in rows if r["preset"] == p and r["div"] == d and r["variant"] == name
                       and r["seed"] == s and r["cn"] >= 59]
            row.append(f"{C.fmt_db(np.nanmedian(tb) if tb else np.nan)} / {C.fmt_db(np.nanmedian(t10) if t10 else np.nan)}"
                       f" / {C.fmt_db(np.nanmedian(ce) if ce else np.nan)}")
        tab.append(row)
    L += [C.md_table(hdr, tab), ""]
    (C.RESULTS / "S1c_pll.md").write_text("\n".join(L), encoding="utf-8")


if __name__ == "__main__":
    run()
