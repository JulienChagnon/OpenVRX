# SPDX-License-Identifier: GPL-3.0-or-later
"""S3: diversity combining. Two receivers of one VTX, D1 select, D2 blend,
D3 per-sample repair, D4 pre-detection MRC, with and without fading.

Scenario (matches the community Monte Carlo it is checked against): Rician
K = 6 dB, complex correlation 0.5 between the antennas' scattered parts,
Doppler 100 Hz, carrier offsets 120 to 170 kHz between the receivers'
crystals, sample clocks 20 to 30 ppm apart, independent block AGC per
receiver. Lane format 4x4top at 26.67 MS/s (the first live layout).
"""
from __future__ import annotations

import numpy as np

from .. import decoder, diversity, metrics as M, simgen
from ..lanes import PRESETS
from . import common as C

PRESET, DIV = "4x4top", 9
DURATION = 0.25
CNS = (0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 23, 26)
METHODS = ("single", "D1 select", "D2 blend", "D3 repair", "D4 MRC")


def rician_pair(t_list, k_db=6.0, rho=0.5, doppler=100.0, seed=0, n=2):
    """Complex gains per receiver: own LOS phase, scattered part correlated
    rho with a common process. E|g|^2 = 1."""
    k = 10 ** (k_db / 10)
    common = simgen.FadingConfig("rayleigh", doppler, seed=seed * 31 + 1)
    out = []
    rng = np.random.default_rng(seed * 31 + 7)
    th0 = rng.uniform(-np.pi, np.pi)
    for j, t in enumerate(t_list):
        own = simgen.FadingConfig("rayleigh", doppler, seed=seed * 31 + 11 + j)
        diff = rho * simgen.fading_gain(t, common) + np.sqrt(1 - rho * rho) * simgen.fading_gain(t, own)
        los = np.exp(1j * (2 * np.pi * doppler * np.cos(th0) * t + rng.uniform(-np.pi, np.pi)))
        out.append((np.sqrt(k / (k + 1)) * los + np.sqrt(1 / (k + 1)) * diff).astype(np.complex64))
    return out


def quantise_gain(cap, preset, div, cn, g, agc_mode):
    fmt = PRESETS[preset]
    y = simgen.mix(cap, cn, g)
    gains, _ = simgen.apply_agc(y, cap.fs, fmt, simgen.AgcConfig(mode=agc_mode))
    i, q = simgen.adc10(y * gains, 20.0, 0.0)
    return simgen.encode_words(i, q, fmt)


def keys_for(lines, cap):
    m = (lines.src_position - cap.line_a - cap.delay_samples) / cap.line_b
    off = float(np.median(m - np.round(m)))
    return np.round(m - off).astype(np.int64)


def cmp_rows(std, ire_rows, keys, tx, coverage):
    fl = C.truth_frame_lines(tx)
    ok = (keys >= 0) & (keys < tx.n_lines)
    act = ok & M.active_line_mask(std, fl[np.clip(keys, 0, tx.n_lines - 1)])
    return C.trim_ends(M.Comparison(std, np.asarray(ire_rows[act], np.float64),
                                    M.band_limit_lines(tx.lines[keys[act]], std), keys[act], 1.0, coverage), tx)


def decode_keep(words, preset, div, std):
    fmt = PRESETS[preset]
    res = decoder.decode_iq(fmt.build_lut()[words], 240e6 / div, decoder.DecoderConfig(), std, fmt.has_magnitude)
    z = res.demod.z
    res.demod = None   # free the large intermediate arrays
    return res.lines, z


def combine_scores(lines_list, caps, tx, cal, zs=None, methods=METHODS[1:]):
    """Scores of D1..D4 for receivers already decoded. Missing receivers are None."""
    std = tx.std
    keys = [keys_for(L, c) for L, c in zip(lines_list, caps)]
    al = diversity.align(lines_list, keys)
    cov = min(1.0, len(al.keys) / max(1, len(keys[0])))
    out = {}
    if "D1 select" in methods:
        out["D1 select"] = C.scores(cmp_rows(std, diversity.d1_select(al)[0], al.keys, tx, cov), cal)
    if "D2 blend" in methods:
        out["D2 blend"] = C.scores(cmp_rows(std, diversity.d2_blend(al)[0], al.keys, tx, cov), cal)
    if "D3 repair" in methods:
        out["D3 repair"] = C.scores(cmp_rows(std, diversity.d3_repair(al), al.keys, tx, cov), cal)
    if "D4 MRC" in methods and zs is not None:
        try:
            y = diversity.mrc(zs[0], zs[1], lines_list[0], lines_list[1], al.rows[0], al.rows[1], caps[0].fs)
            fmt = PRESETS[PRESET]
            res = decoder.decode_iq(y, caps[0].fs, decoder.DecoderConfig(), std, True)
            cmp = C.compare(res, tx, caps[0])
            out["D4 MRC"] = C.scores(cmp, cal)
        except Exception:
            out["D4 MRC"] = {"sinad": np.nan, "wsnr": np.nan, "breakup": 1.0, "coverage": 0.0}
    return out


def _job(seed):
    rng = np.random.default_rng(9000 + seed)
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", C.PICTURE, 1.0, seed=seed), DURATION)
    rxa = simgen.RxConfig(seed=7000 + seed)
    rxb = simgen.RxConfig(seed=8000 + seed, lo_offset_hz=rng.choice([-1, 1]) * rng.uniform(120e3, 170e3),
                          rx_ppm=rng.choice([-1, 1]) * rng.uniform(20, 30), bus_delay=rng.uniform(0, 1))
    caps = [simgen.generate(tx, r, [DIV], DURATION)[DIV] for r in (rxa, rxb)]
    gains = rician_pair([c.t for c in caps], seed=seed)
    env_corr = float(np.corrcoef(np.abs(gains[0][::50]), np.abs(gains[1][::50]))[0, 1])
    rows = []
    cal = None
    for scen in ("no fading", "Rician K=6 dB"):
        for cn in (60.0,) + CNS:
            g = [None, None] if scen == "no fading" else gains
            mode = "static" if scen == "no fading" else "tracking"
            dec = []
            for cap, gk in zip(caps, g):
                w = quantise_gain(cap, PRESET, DIV, cn, gk, mode)
                try:
                    dec.append(decode_keep(w, PRESET, DIV, tx.std))
                except Exception:
                    dec.append(None)
            singles = []
            for (pair, cap) in zip(dec, caps):
                if pair is None:
                    singles.append({"sinad": np.nan, "breakup": 1.0, "coverage": 0.0})
                    continue
                cmp = C.trim_ends(M.match_lines(pair[0], tx.lines, C.truth_frame_lines(tx), cap.line_a,
                                                cap.line_b, cap.delay_samples), tx)
                if cal is None:
                    cal = M.calibrate(cmp)
                singles.append(C.scores(cmp, cal))
            if cn >= 59:
                continue
            res = {"single": singles[0], "single B": singles[1]}
            if all(d is not None for d in dec):
                res.update(combine_scores([d[0] for d in dec], caps, tx, cal, [d[1] for d in dec]))
            for name, d in res.items():
                rows.append(dict(seed=seed, scenario=scen, cn=cn, method=name, env_corr=env_corr,
                                 lo_khz=rxb.lo_offset_hz / 1e3, ppm=rxb.rx_ppm,
                                 **{k: v for k, v in d.items() if k in ("sinad", "wsnr", "breakup", "coverage")}))
    return rows


def pooled(rows, scen, method, key):
    """Per C/N: breakup pooled over seeds (mean), SINAD median."""
    sel = [r for r in rows if r["scenario"] == scen and r["method"] == method]
    cns = sorted({r["cn"] for r in sel})
    if key == "breakup":
        y = [np.mean([r[key] for r in sel if r["cn"] == c]) for c in cns]
    else:
        y = [np.nanmedian([r[key] for r in sel if r["cn"] == c]) for c in cns]
    return np.array(cns), np.array(y)


def run(seeds=tuple(range(8))):
    rows = [r for part in C.pmap(_job, list(seeds), n=min(4, C.workers())) for r in part]
    C.save_json("S3_rows.json", rows)
    write(rows)
    return rows


def write(rows):
    methods = ("single", "single B") + METHODS[1:]
    lines = ["# S3: diversity combining [sim]", "",
             f"Two receivers, {PRESET} at {240 / DIV:.2f} MS/s each, NTSC, deviation 1.0, "
             f"{len({r['seed'] for r in rows})} seeds x {DURATION} s. Receiver B: carrier offset "
             f"{min(abs(r['lo_khz']) for r in rows):.0f} to {max(abs(r['lo_khz']) for r in rows):.0f} kHz, "
             f"clock {min(abs(r['ppm']) for r in rows):.0f} to {max(abs(r['ppm']) for r in rows):.0f} ppm off "
             "receiver A, random bus phase. Fading: Rician K = 6 dB, Doppler 100 Hz, scattered parts "
             f"correlated 0.5 (measured envelope correlation {np.mean([r['env_corr'] for r in rows]):.2f}), "
             "independent tracking AGC per receiver. C/N is the mean C/N per 20 MHz channel, equal on both "
             f"antennas. Breakup = share of active lines with RMS error > {C.BREAK_IRE:.0f} IRE or missing, "
             "pooled over seeds.", ""]
    for scen in ("no fading", "Rician K=6 dB"):
        hdr = ["method", "C/N @ 1 % breakup", "gain vs single", "C/N @ 10 % breakup", "C/N @ SINAD 10",
               "gain @ SINAD 10", "SINAD @ 10 dB C/N"]
        tab = []
        base = {}
        for m in methods:
            cn, br = pooled(rows, scen, m, "breakup")
            _, si = pooled(rows, scen, m, "sinad")
            if not len(cn):
                continue
            t1 = C.threshold_down(cn, br, 0.01)
            t10 = C.threshold_down(cn, br, 0.10)
            ts = C.threshold(cn, si, 10.0)
            if m == "single":
                base = {"t1": t1, "ts": ts}
            s10 = si[list(cn).index(10)] if 10 in cn else np.nan
            tab.append([m, C.fmt_db(t1), C.fmt_db(base["t1"] - t1) if m != "single" else "",
                        C.fmt_db(t10), C.fmt_db(ts), C.fmt_db(base["ts"] - ts) if m != "single" else "",
                        C.fmt_db(s10)])
        lines += [f"## {scen}", "", C.md_table(hdr, tab), ""]
    a1 = {m: C.threshold_down(*pooled(rows, "Rician K=6 dB", m, "breakup"), 0.01) for m in methods}
    awgn = {m: C.threshold_down(*pooled(rows, "no fading", m, "breakup"), 0.01) for m in methods}
    lines += ["## Against the community Monte Carlo", "",
              C.md_table(["quantity", "community", "this lab"],
                         [["selection, fade margin saved at 1 % breakup", "~6 dB",
                           C.fmt_db(a1["single"] - a1["D1 select"]) + " dB (D1), "
                           + C.fmt_db(a1["single"] - a1["D3 repair"]) + " dB (D3)"],
                          ["pre-detection MRC, fade margin saved", "~8 dB", C.fmt_db(a1["single"] - a1["D4 MRC"]) + " dB"],
                          ["MRC, no fading", "+3 dB", C.fmt_db(awgn["single"] - awgn["D4 MRC"]) + " dB"]]), "",
              "Plot: `S3_breakup.png`.", ""]
    (C.RESULTS / "S3_combining.md").write_text("\n".join(lines), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for ax, scen in zip(axes, ("no fading", "Rician K=6 dB")):
        for m in methods:
            cn, br = pooled(rows, scen, m, "breakup")
            ax.semilogy(cn, np.maximum(br, 1e-4), "-o", ms=3, label=m)
        ax.axhline(0.01, color="0.5", lw=0.8)
        ax.set_title(scen)
        ax.set_xlabel("mean C/N per 20 MHz channel (dB)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("share of broken lines")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(C.RESULTS / "S3_breakup.png", dpi=110)
    plt.close(fig)
