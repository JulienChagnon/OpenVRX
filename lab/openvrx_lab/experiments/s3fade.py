# SPDX-License-Identifier: GPL-3.0-or-later
"""S3f: selection against pre-detection MRC under Rician fading.

Two stages, so that 1 % breakup is measured on millions of lines in minutes:

1. Calibration, full chain, no fading. Two receivers of one VTX, 4x4top at
   26.67 MS/s each; receiver B has a carrier offset of 120 to 170 kHz and a
   sample clock 20 to 30 ppm off receiver A, own noise, own static gain. Both
   are decoded by the reference decoder and combined by D1 (per-line
   selection), D3 (per-sample repair) and D4 (pre-detection MRC, resampling,
   derotation, phase tracking and weighting included). The share of broken
   lines versus C/N gives p_single(c), p_D1(c) and p_mrc(c) at equal C/N on
   both antennas. The MRC curve therefore carries every implementation loss
   of the real combiner (offset and drift tracking, 4-bit quantisation).

2. Fading Monte Carlo at line rate. Correlated Rician gains (K = 6 dB, LOS
   phases independent, scattered parts with complex correlation rho, Doppler
   100 Hz) for 120 s of lines. Fading is flat over a line (coherence time
   ~ms against 63.6 us lines), so a line breaks with the probability the
   calibration gives for its instantaneous C/N:

   - single: p_single(c |g_A|^2)
   - selection (ideal per-line choice, what D1/D3 approach): p_single(c max(|g_A|^2, |g_B|^2))
   - MRC: p_mrc((c |g_A|^2 + c |g_B|^2) / 2), since the calibration was taken
     at equal branch C/N and MRC output C/N is the sum of the branch C/Ns.

   The C/N where the breakup share falls to 1 % per method, against the same
   for one receiver, is the fade margin the combiner saves.
"""
from __future__ import annotations

import numpy as np

from .. import simgen
from . import common as C
from . import s3

DURATION = 0.1
CAL_CNS = tuple(range(0, 21))
METHODS = ("single", "single B", "D1 select", "D3 repair", "D4 MRC")


def _cal_job(seed):
    rng = np.random.default_rng(9100 + seed)
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", C.PICTURE, 1.0, seed=seed), DURATION)
    rxa = simgen.RxConfig(seed=7100 + seed)
    rxb = simgen.RxConfig(seed=8100 + seed, lo_offset_hz=rng.choice([-1, 1]) * rng.uniform(120e3, 170e3),
                          rx_ppm=rng.choice([-1, 1]) * rng.uniform(20, 30), bus_delay=rng.uniform(0, 1))
    caps = [simgen.generate(tx, r, [s3.DIV], DURATION)[s3.DIV] for r in (rxa, rxb)]
    rows, cal = [], None
    for cn in (60.0,) + CAL_CNS:
        dec = []
        for cap in caps:
            w = s3.quantise_gain(cap, s3.PRESET, s3.DIV, cn, None, "static")
            try:
                dec.append(s3.decode_keep(w, s3.PRESET, s3.DIV, tx.std))
            except Exception:
                dec.append(None)
        singles = []
        for pair, cap in zip(dec, caps):
            if pair is None:
                singles.append({"sinad": np.nan, "breakup": 1.0})
                continue
            cmp = C.trim_ends(C.M.match_lines(pair[0], tx.lines, C.truth_frame_lines(tx), cap.line_a, cap.line_b,
                                              cap.delay_samples), tx)
            if cal is None:
                cal = C.M.calibrate(cmp)
            singles.append(C.scores(cmp, cal))
        if cn >= 59:
            continue
        res = {"single": singles[0], "single B": singles[1]}
        if all(d is not None for d in dec):
            res.update(s3.combine_scores([d[0] for d in dec], caps, tx, cal, [d[1] for d in dec],
                                         methods=("D1 select", "D3 repair", "D4 MRC")))
        for m in METHODS:
            d = res.get(m, {"sinad": np.nan, "breakup": 1.0})
            rows.append(dict(seed=seed, cn=cn, method=m, breakup=float(d["breakup"]), sinad=float(d["sinad"]),
                             lo_khz=rxb.lo_offset_hz / 1e3, ppm=rxb.rx_ppm))
    return rows


def curve(rows, method):
    cns = np.array(sorted({r["cn"] for r in rows}), float)
    names = ("single", "single B") if method == "single" else (method,)
    p = np.array([np.mean([r["breakup"] for r in rows if r["cn"] == c and r["method"] in names]) for c in cns])
    # Monotone (more C/N never breaks more lines), as the physical curve is.
    p = np.minimum.accumulate(p)
    return cns, p


def p_of(cns, p, cn_db):
    """Breakup probability at C/N (dB), log-interpolated, 1 below the
    curve, 0 above its last non-zero point."""
    lp = np.log10(np.maximum(p, 1e-6))
    out = 10 ** np.interp(cn_db, cns, lp, left=0.0, right=-6.0)
    out = np.where(cn_db < cns[0], 1.0, out)
    out = np.where(out <= 1.5e-6, 0.0, out)
    return out


def rician_pair_lines(n, k_db, rho, doppler, line_hz, seed, chunk=200_000, los_same=False):
    """|g_A|^2, |g_B|^2 at line rate (E = 1), sum of 16 sinusoids per process."""
    rng = np.random.default_rng(seed)
    k = 10 ** (k_db / 10)
    M = 16

    def proc():
        theta = rng.uniform(-np.pi, np.pi)
        phi, psi = rng.uniform(-np.pi, np.pi, M), rng.uniform(-np.pi, np.pi, M)
        alpha = (2 * np.pi * np.arange(1, M + 1) - np.pi + theta) / (4 * M)
        return np.cos(alpha), np.sin(alpha), phi, psi

    common, own_a, own_b = proc(), proc(), proc()
    # One LOS direction (same Doppler) for both antennas, own phase each.
    th_los = rng.uniform(-np.pi, np.pi)
    los = [(th_los, rng.uniform(-np.pi, np.pi)) for _ in range(2)]
    if los_same:   # antennas close together: same LOS phase, envelopes correlate
        los[1] = los[0]
    wd = 2 * np.pi * doppler
    ga, gb = np.empty(n), np.empty(n)
    for a in range(0, n, chunk):
        t = np.arange(a, min(n, a + chunk)) / line_hz

        def ray(p):
            ca, sa, phi, psi = p
            tt = t[:, None]
            return (np.cos(wd * tt * ca + phi).sum(1) + 1j * np.cos(wd * tt * sa + psi).sum(1)) / np.sqrt(M)

        rc = ray(common)
        out = []
        for own, (th0, ph0) in zip((own_a, own_b), los):
            diff = rho * rc + np.sqrt(1 - rho * rho) * ray(own)
            l = np.exp(1j * (wd * np.cos(th0) * t + ph0))
            g = np.sqrt(k / (k + 1)) * l + np.sqrt(1 / (k + 1)) * diff
            out.append(np.abs(g) ** 2)
        ga[a:a + len(t)], gb[a:a + len(t)] = out
    return ga, gb


def fading_mc(cal_rows, k_db=6.0, rho=0.5, doppler=100.0, seconds=120.0, seed=1):
    line_hz = 15734.26
    n = int(seconds * line_hz)
    ga, gb = rician_pair_lines(n, k_db, rho, doppler, line_hz, seed)
    env_corr = float(np.corrcoef(np.sqrt(ga[::7]), np.sqrt(gb[::7]))[0, 1])
    cs, ps = curve(cal_rows, "single")
    cm, pm = curve(cal_rows, "D4 MRC")
    grid = np.arange(-2.0, 45.0, 0.25)
    out = {"single": [], "selection": [], "MRC": [], "single, no fading": [], "MRC, no fading": []}
    sub = slice(None, None, 3)   # every third line: independent enough, 3x faster
    a, b = ga[sub], gb[sub]
    for c in grid:
        ca, cb = c + 10 * np.log10(a), c + 10 * np.log10(b)
        out["single"].append(float(np.mean(p_of(cs, ps, ca))))
        out["selection"].append(float(np.mean(p_of(cs, ps, np.maximum(ca, cb)))))
        out["MRC"].append(float(np.mean(p_of(cm, pm, 10 * np.log10(10 ** (ca / 10) + 10 ** (cb / 10)) - 10 * np.log10(2)))))
        out["single, no fading"].append(float(p_of(cs, ps, np.array([c]))[0]))
        out["MRC, no fading"].append(float(p_of(cm, pm, np.array([c]))[0]))
    thr = {m: C.threshold_down(grid, np.array(v), 0.01) for m, v in out.items()}
    return dict(k_db=k_db, rho=rho, doppler=doppler, env_corr=env_corr, grid=grid.tolist(),
                curves=out, thr=thr)


def run(seeds=(0, 1, 2, 3)):
    cal = [r for part in C.pmap_cached(_cal_job, list(seeds), "s3f", n=min(4, C.workers())) for r in part]
    C.save_json("S3f_calibration.json", cal)
    cases = [fading_mc(cal, rho=0.5), fading_mc(cal, rho=0.71), fading_mc(cal, k_db=3.0, rho=0.5),
             fading_mc(cal, k_db=10.0, rho=0.5)]
    C.save_json("S3f_fading.json", cases)
    write(cal, cases)
    return cal, cases


def write(cal, cases):
    L = ["# S3f: selection against pre-detection MRC under Rician fading [sim + Monte Carlo]", "",
         f"Calibration: full chain, two receivers, {s3.PRESET} at {240 / s3.DIV:.2f} MS/s, NTSC, deviation 1.0, "
         f"{len({r['seed'] for r in cal})} seeds x {DURATION} s, receiver B {min(abs(r['lo_khz']) for r in cal):.0f} to "
         f"{max(abs(r['lo_khz']) for r in cal):.0f} kHz and {min(abs(r['ppm']) for r in cal):.0f} to "
         f"{max(abs(r['ppm']) for r in cal):.0f} ppm off receiver A, no fading. Broken line = RMS error > "
         f"{C.BREAK_IRE:.0f} IRE or missing.", ""]
    cns = sorted({r["cn"] for r in cal})
    hdr = ["C/N (dB)"] + list(METHODS)
    tab = []
    for c in cns:
        tab.append([f"{c:.0f}"] + [f"{100 * np.mean([r['breakup'] for r in cal if r['cn'] == c and r['method'] == m]):.2f} %"
                                   for m in METHODS])
    L += ["Share of broken lines, no fading (equal C/N on both antennas):", "", C.md_table(hdr, tab), ""]
    awgn = {m: C.threshold_down(*curve(cal, m), 0.01) for m in ("single", "D1 select", "D3 repair", "D4 MRC")}
    L += ["C/N at 1 % breakup without fading: " + ", ".join(f"{m} {C.fmt_db(v)} dB" for m, v in awgn.items())
          + f". MRC gain without fading: {C.fmt_db(awgn['single'] - awgn['D4 MRC'])} dB (ideal 3.0).", ""]
    hdr = ["fading case", "envelope corr.", "single @ 1 %", "selection @ 1 %", "MRC @ 1 %", "fade margin (single)",
           "saved by selection", "saved by MRC"]
    tab = []
    for cs in cases:
        t = cs["thr"]
        tab.append([f"Rician K {cs['k_db']:.0f} dB, rho {cs['rho']:.2f}", f"{cs['env_corr']:.2f}",
                    C.fmt_db(t["single"]), C.fmt_db(t["selection"]), C.fmt_db(t["MRC"]),
                    C.fmt_db(t["single"] - t["single, no fading"]),
                    C.fmt_db(t["single"] - t["selection"]), C.fmt_db(t["single"] - t["MRC"])])
    L += ["Fading Monte Carlo, 120 s of lines per case, Doppler 100 Hz, mean C/N equal on both antennas "
          "(dB per 20 MHz channel). rho is the complex correlation of the scattered parts.", "",
          C.md_table(hdr, tab), "",
          "Community reference (K 6 dB, correlation 0.5, 1 % breakup): selection about 6 dB, MRC about 8 dB, "
          "MRC without fading +3 dB.", "", "Plot: `S3f_fading.png`.", ""]
    (C.RESULTS / "S3f_diversity.md").write_text("\n".join(L), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 4.4))
    cs = cases[0]
    for m, v in cs["curves"].items():
        ax.semilogy(cs["grid"], np.maximum(v, 1e-5), "--" if "no fading" in m else "-", label=m)
    ax.axhline(0.01, color="0.5", lw=0.8)
    ax.set_xlim(0, 40)
    ax.set_ylim(1e-4, 1.1)
    ax.set_xlabel("mean C/N per antenna, dB per 20 MHz")
    ax.set_ylabel("share of broken lines")
    ax.set_title("4x4top @ 26.67 MS/s, Rician K 6 dB, rho 0.5")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(C.RESULTS / "S3f_fading.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    run()


# ---------------------------------------------------------------------------
# Light variant: single-receiver curve from S1, MRC as ideal C/N addition
# ---------------------------------------------------------------------------

def s1_curve(preset="4x4top", div=9, dev=1.0):
    import json
    rows = json.loads((C.RESULTS / "S1_rows.json").read_text(encoding="utf-8"))
    sel = [r for r in rows if r["preset"] == preset and r["div"] == div and r["dev"] == dev and r["cn"] < 59]
    cns = np.array(sorted({r["cn"] for r in sel}), float)
    p = np.array([np.mean([r["breakup"] for r in sel if r["cn"] == c]) for c in cns])
    return cns, np.minimum.accumulate(p)


def fading_light(cns, ps, k_db=6.0, rho=0.5, doppler=100.0, seconds=60.0, seed=1, mrc_loss_db=(0.0, 1.0),
                 los_same=False):
    line_hz = 15734.26
    n = int(seconds * line_hz)
    ga, gb = rician_pair_lines(n, k_db, rho, doppler, line_hz, seed, chunk=50_000, los_same=los_same)
    a, b = ga[::3], gb[::3]
    del ga, gb
    env_corr = float(np.corrcoef(np.sqrt(a), np.sqrt(b))[0, 1])
    la, lb = 10 * np.log10(a), 10 * np.log10(b)
    lsel = np.maximum(la, lb)
    lmrc = 10 * np.log10(a + b)
    grid = np.arange(0.0, 40.0, 0.1)
    curves = {"single": [], "selection": []}
    for L in mrc_loss_db:
        curves[f"MRC (loss {L:.1f} dB)"] = []
    for c in grid:
        curves["single"].append(float(np.mean(p_of(cns, ps, c + la))))
        curves["selection"].append(float(np.mean(p_of(cns, ps, c + lsel))))
        for L in mrc_loss_db:
            curves[f"MRC (loss {L:.1f} dB)"].append(float(np.mean(p_of(cns, ps, c + lmrc - L))))
    thr = {m: C.threshold_down(grid, np.array(v), 0.01) for m, v in curves.items()}
    awgn = C.threshold_down(cns, ps, 0.01)
    return dict(k_db=k_db, rho=rho, los_same=los_same, env_corr=env_corr, thr=thr, awgn=awgn,
                grid=grid.tolist(), curves=curves)


def run_light():
    cns, ps = s1_curve()
    cases = [fading_light(cns, ps, rho=0.5), fading_light(cns, ps, rho=0.5, los_same=True),
             fading_light(cns, ps, rho=0.71, los_same=True), fading_light(cns, ps, rho=0.5, seed=2),
             fading_light(cns, ps, k_db=3.0), fading_light(cns, ps, k_db=10.0)]
    C.save_json("S3f_fading_light.json", cases)
    L = ["# S3f: selection against pre-detection MRC under Rician fading [sim curve + Monte Carlo]", "",
         "Per-line breakup probability versus C/N for one receiver from S1 (4x4top at 26.67 MS/s, deviation 1.0, "
         "full reference decode, 2 seeds). Fading: two correlated Rician gains at line rate (16-sinusoid "
         "processes, Doppler 100 Hz, 60 s per case, one LOS direction with an own LOS phase per antenna unless "
         "marked 'same LOS phase', scattered parts with complex correlation rho). With K 6 dB the envelope "
         "follows the LOS/scatter interference, so the envelope correlation depends on the antennas' LOS "
         "phase difference as much as on rho; the cases bracket envelope correlations 0.03 to 0.5. Fading is flat over a 63.6 us line, so a line breaks with the probability the S1 "
         "curve gives at its instantaneous C/N. Selection = per-line choice of the stronger antenna (what D1/D3 "
         "approach; D3 works per sample and does slightly better). MRC = sum of the branch C/Ns minus an "
         "implementation loss (0 dB ideal; 1 dB covers carrier offset 120 to 170 kHz, 20 to 30 ppm clock drift "
         "and phase tracking on 4-bit samples).", "",
         "Not yet run: the full-chain calibration (`run()` in this module), which decodes two receivers with "
         "those offsets and drifts through D1, D3 and the real D4 combiner to measure its loss; it was stopped "
         "for lack of memory on the lab machine. Run `python -m openvrx_lab.experiments.s3fade` on an idle "
         "machine to replace the assumed 0 to 1 dB loss.", "",
         f"No fading, one receiver: 1 % breakup at {C.fmt_db(cases[0]['awgn'])} dB C/N.", ""]
    keys = list(cases[0]["thr"])
    hdr = ["fading case", "envelope corr."] + [f"{k} @ 1 %" for k in keys] + ["fade margin (single)"] + \
          [f"saved by {k}" for k in keys[1:]]
    tab = []
    for cs in cases:
        t = cs["thr"]
        tab.append([f"K {cs['k_db']:.0f} dB, rho {cs['rho']:.2f}" + (", same LOS phase" if cs["los_same"] else ""),
                    f"{cs['env_corr']:.2f}"]
                   + [C.fmt_db(t[k]) for k in keys] + [C.fmt_db(t["single"] - cs["awgn"])]
                   + [C.fmt_db(t["single"] - t[k]) for k in keys[1:]])
    L += [C.md_table(hdr, tab), "",
          "Community Monte Carlo (K 6 dB, correlation 0.5, 1 % breakup): selection about 6 dB, MRC about 8 dB, "
          "MRC without fading +3 dB. Without fading, ideal MRC here saves exactly 3 dB and selection about 0.", ""]
    (C.RESULTS / "S3f_diversity.md").write_text("\n".join(L), encoding="utf-8")
    return cases
