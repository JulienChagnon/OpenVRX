# SPDX-License-Identifier: GPL-3.0-or-later
"""S1: sample format. Picture quality after the full reference decode versus
C/N for every lane preset at the dividers the FX2 can carry, three VTX
deviations, several seeds. S1b: the 802.11p filter (and the unverified
"bus at 40 MS/s" behaviour) at each rate.
"""
from __future__ import annotations

import numpy as np

from .. import simgen
from . import common as C

CNS = tuple(range(2, 17)) + (18, 20)
WIDE = ("8x8", "7x7", "6x6", "5x5", "4x4top")
NARROW = ("4x4top", "4x4fine", "4x4ultra")
# The layouts the bench can try: 16-bit words at 13.33 MS/s, 7x7 at 20 MS/s
# (40 MB/s), 8-bit 4+4 words at 26.67 and 40 MS/s. 4x4top at 13.33 MS/s is the
# rate-only reference.
FORMATS = {18: WIDE, 12: ("7x7",), 9: NARROW, 6: NARROW}
DEVS = (0.7, 1.0, 1.4)
DURATION = 0.05


def _job(args):
    dev, seed, div, presets, rxkw, tag = args
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", C.PICTURE, dev, seed=seed), DURATION)
    rx = simgen.RxConfig(seed=500 + seed, **rxkw)
    cap = simgen.generate(tx, rx, [div], DURATION)[div]
    out = []
    for p in presets:
        clean, cal = C.evaluate(tx, cap, simgen.ReceiverRun(p, div, 60.0))
        out.append(dict(tag=tag, dev=dev, seed=seed, div=div, preset=p, cn=60.0, **clean))
        for cn in CNS:
            d, _ = C.evaluate(tx, cap, simgen.ReceiverRun(p, div, float(cn)), cal=cal, psnr=True)
            out.append(dict(tag=tag, dev=dev, seed=seed, div=div, preset=p, cn=float(cn), **d))
    return out


def summarise(rows, tag):
    """Per (preset, div, dev): thresholds per seed, then median and range."""
    out = {}
    keys = sorted({(r["preset"], r["div"]) for r in rows if r["tag"] == tag},
                  key=lambda k: (-240 / k[1], k[0]))
    for p, d in keys:
        for dev in DEVS:
            sel = [r for r in rows if r["tag"] == tag and r["preset"] == p and r["div"] == d and r["dev"] == dev]
            if not sel:
                continue
            seeds = sorted({r["seed"] for r in sel})
            t10, t20, tb, ceil, s8 = [], [], [], [], []
            for s in seeds:
                rs = sorted([r for r in sel if r["seed"] == s and r["cn"] < 59], key=lambda r: r["cn"])
                cn = [r["cn"] for r in rs]
                si = [r["sinad"] for r in rs]
                t10.append(C.threshold(cn, si, 10.0))
                t20.append(C.threshold(cn, si, 20.0))
                tb.append(C.threshold_down(cn, [r["breakup"] for r in rs], 0.01))
                ceil += [r["sinad"] for r in sel if r["seed"] == s and r["cn"] >= 59]
                s8 += [r["sinad"] for r in rs if r["cn"] == 8.0]
            out[(p, d, dev)] = dict(t10=C.spread(t10), t20=C.spread(t20), tb=C.spread(tb), ceil=C.spread(ceil),
                                    s8=C.spread(s8))
    return out


def curves(rows, tag, p, d, dev, key="sinad"):
    sel = [r for r in rows if r["tag"] == tag and r["preset"] == p and r["div"] == d and r["dev"] == dev
           and r["cn"] < 59]
    cns = sorted({r["cn"] for r in sel})
    med = [np.nanmedian([r[key] for r in sel if r["cn"] == c]) for c in cns]
    lo = [np.nanmin([r[key] for r in sel if r["cn"] == c]) for c in cns]
    hi = [np.nanmax([r[key] for r in sel if r["cn"] == c]) for c in cns]
    return np.array(cns), np.array(med), np.array(lo), np.array(hi)


def plot(rows, tag, keys, fname, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(DEVS), figsize=(5.2 * len(DEVS), 4.4), sharey=True)
    cmap = plt.get_cmap("tab10")
    for ax, dev in zip(axes, DEVS):
        for j, (p, d) in enumerate(keys):
            cn, med, lo, hi = curves(rows, tag, p, d, dev)
            if not len(cn):
                continue
            ls = "-" if d in (6, 9) else "--"
            ax.plot(cn, med, ls, color=cmap(j % 10), lw=1.6, label=f"{p} @ {240 / d:.2f} MS/s")
            ax.fill_between(cn, lo, hi, color=cmap(j % 10), alpha=0.12, lw=0)
        ax.axhline(20, color="0.6", lw=0.8)
        ax.axhline(10, color="0.6", lw=0.8)
        ax.set_title(f"deviation {dev}")
        ax.set_xlabel("C/N per 20 MHz channel (dB)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("luma SINAD after decode (dB)")
    axes[0].set_ylim(-5, 35)
    axes[-1].legend(fontsize=7, loc="lower right")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(C.RESULTS / fname, dpi=110)
    plt.close(fig)


def run_s1(seeds=(0, 1)):
    jobs = [(dev, s, d, FORMATS[d], {}, "s1") for dev in DEVS for s in seeds for d in FORMATS]
    rows = [r for part in C.pmap_cached(_job, jobs, "s1") for r in part]
    C.save_json("S1_rows.json", rows)
    write_s1(rows)
    return rows


def write_s1(rows):
    summ = summarise(rows, "s1")
    keys = sorted({(p, d) for p, d, _ in summ}, key=lambda k: (k[1], k[0]))
    lines = ["# S1: sample format [sim]", "",
             f"NTSC, astronaut picture, 3 fields per run, {len({r['seed'] for r in rows})} seeds per point (independent noise, start line, "
             "carrier phase). C/N per 20 MHz channel (C5VRX definition). Full reference decode: DC removal, "
             "exact polar discriminator with the wrap point in the empty part of the spectrum, wrap/click "
             "repair, digital channel filter at >= 20 MS/s, video low-pass, sync, TBC to the 910-sample grid. "
             "Gain policy: contiguous lanes and angle8 at RMS 0.45 of the +-512 window; folded lanes (4x4fine, "
             "4x4ultra) at 4.2 cells (C5VRX ultrafine policy). I DC offset 20 ADC units.", "",
             "Columns: C/N where luma SINAD reaches 10 dB (usable) and 20 dB (good), C/N where line breakup "
             f"(line RMS error > {C.BREAK_IRE:.0f} IRE or line missing) falls to 1 %, the noise-free ceiling "
             "(SINAD at 60 dB C/N). Median over seeds, (min..max) in brackets. Lower C/N is better.", ""]
    for dev in DEVS:
        hdr = ["format", "rate", "pins", "MB/s", "C/N @ SINAD 10", "C/N @ SINAD 20", "C/N @ 1 % breakup",
               "SINAD @ 8 dB", "ceiling"]
        tab = []
        for p, d in keys:
            s = summ.get((p, d, dev))
            if s is None:
                continue
            rate, mbs, pins = C.rate_label(p, d)
            tab.append([p, rate, pins + 1, f"{mbs:.1f}", s["t10"], s["t20"], s["tb"], s["s8"], s["ceil"]])
        lines += [f"## Deviation {dev}", "", C.md_table(hdr, tab), ""]
    # Ranking by the worst deviation (a receiver must work with any VTX).
    hdr = ["rank", "format", "rate", "pins (incl. clock)", "MB/s", "worst-dev C/N @ 1 % breakup",
           "worst-dev C/N @ SINAD 10", "mean C/N @ SINAD 20", "worst ceiling"]
    rank = []
    for p, d in keys:
        tb = [summ[(p, d, dev)]["tb"].median for dev in DEVS if (p, d, dev) in summ]
        t10 = [summ[(p, d, dev)]["t10"].median for dev in DEVS if (p, d, dev) in summ]
        t20 = [summ[(p, d, dev)]["t20"].median for dev in DEVS if (p, d, dev) in summ]
        ce = [summ[(p, d, dev)]["ceil"].median for dev in DEVS if (p, d, dev) in summ]
        rate, mbs, pins = C.rate_label(p, d)
        rank.append((np.nanmax(tb) if np.isfinite(tb).any() else 99, p, rate, pins + 1, mbs,
                     np.nanmax(t10) if np.isfinite(t10).any() else np.nan,
                     np.nanmean(t20) if np.isfinite(t20).all() else np.nan, np.nanmin(ce)))
    rank.sort(key=lambda r: (r[0], r[5]))
    tab = [[i + 1, r[1], r[2], r[3], f"{r[4]:.1f}", C.fmt_db(r[0]), C.fmt_db(r[5]), C.fmt_db(r[6]),
            C.fmt_db(r[7])] for i, r in enumerate(rank)]
    lines += ["## Ranking (worst deviation, 1 % breakup)", "", C.md_table(hdr, tab), "",
              "Plots: `S1_sinad_16bit.png` (16-bit word formats), `S1_sinad_8bit.png` (8-bit word formats); "
              "shaded band = min..max over seeds.", ""]
    (C.RESULTS / "S1_sample_format.md").write_text("\n".join(lines), encoding="utf-8")
    C.save_json("S1_ranking.json", [dict(zip(["breakup1", "preset", "rate", "pins", "mbs", "t10", "t20", "ceiling"], r))
                                    for r in rank])
    plot(rows, "s1", [(p, d) for p, d in keys if p in ("8x8", "7x7", "6x6", "5x5")]
         + [("4x4top", 6), ("4x4top", 9)], "S1_sinad_16bit.png", "S1: 16-bit formats (dashed) vs 4x4top at 26.67/40 MS/s")
    plot(rows, "s1", [(p, d) for p, d in keys if p not in ("8x8", "7x7", "6x6", "5x5") and d in (6, 9, 12)],
         "S1_sinad_8bit.png", "S1: 8-bit formats")


S1B_CASES = [("8x8", 18), ("7x7", 18), ("7x7", 12), ("4x4top", 12), ("4x4top", 9), ("4x4ultra", 9),
             ("4x4top", 6), ("4x4ultra", 6)]
S1B_VARIANTS = {"20 MHz channel": {}, "802.11p filter": {"p11": True},
                "802.11p + bus 40 MS/s": {"p11": True, "bus40": True}}


def run_s1b(seeds=(0, 1, 2), devs=(1.0, 1.4)):
    jobs = []
    for name, kw in S1B_VARIANTS.items():
        for dev in devs:
            for s in seeds:
                for d in sorted({d for _, d in S1B_CASES}):
                    jobs.append((dev, s, d, tuple(p for p, dd in S1B_CASES if dd == d), kw, name))
    rows = [r for part in C.pmap(_job, jobs) for r in part]
    C.save_json("S1b_rows.json", rows)
    write_s1b(rows, devs)
    return rows


def write_s1b(rows, devs=(1.0, 1.4)):
    lines = ["# S1b: does the 802.11p narrow filter help? [sim]", "",
             "Same chain as S1. The 802.11p setting is modelled as an extra 5th-order Butterworth at 6 MHz "
             "after the noise (C/N still per 20 MHz channel, so the narrower noise shows up as a gain). "
             "'bus 40 MS/s' adds the unverified hypothesis that MODEM_DIAG then changes value every 25 ns "
             "(sample-and-hold of every second bus value; at /9 this makes the sampling non-uniform).", "",
             "Cells: C/N at 1 % breakup / C/N at SINAD 10 dB / noise-free ceiling (median over 3 seeds).", ""]
    for dev in devs:
        hdr = ["format", "rate"] + list(S1B_VARIANTS)
        tab = []
        for p, d in S1B_CASES:
            row = [p, f"{240 / d:.2f} MS/s"]
            for name in S1B_VARIANTS:
                summ = summarise([r for r in rows if r["dev"] == dev], name).get((p, d, dev))
                row.append("n/a" if summ is None else
                           f"{C.fmt_db(summ['tb'].median)} / {C.fmt_db(summ['t10'].median)} / {C.fmt_db(summ['ceil'].median)}")
            tab.append(row)
        lines += [f"## Deviation {dev}", "", C.md_table(hdr, tab), ""]
    (C.RESULTS / "S1b_p11_filter.md").write_text("\n".join(lines), encoding="utf-8")
