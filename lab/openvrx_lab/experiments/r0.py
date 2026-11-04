# SPDX-License-Identifier: GPL-3.0-or-later
"""R0: first hardware capture through the reference decoder.

Decodes a recording with ``usb_gap`` holes segment by segment
(decoder.decode_gapped), checks the gap accounting against the line timing,
measures levels, line timing jitter and flat-region noise, writes mosaic
frames (newest copy of every frame line), and estimates the equivalent C/N
by running the simulator with the same layout, rate and clipping and
matching the flat-region noise.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from .. import colour, decoder, metrics as M, simgen
from ..recording import open_recording
from ..standards import NTSC
from . import common as C


def _gather(gd, attr):
    return np.concatenate([getattr(s.lines, attr)[s.keep] for s in gd.segments])


def analyse(path):
    rec = open_recording(path)
    info = rec.receivers[0]
    raw = rec.raw(0)
    z = info.sample_format.lanes.build_lut()[raw[: 4 << 20]]
    fsc = info.sample_format.lanes.component_full_scale
    clip_i = float(np.mean(np.abs(z.real) >= fsc - 33))
    clip_q = float(np.mean(np.abs(z.imag) >= fsc - 33))
    gaps = rec.gaps(0)
    lost = sum(l for _, l in gaps)
    gd = decoder.decode_gapped(rec, 0, std=NTSC)
    fs = gd.fs
    tip, blank = _gather(gd, "tip_hz"), _gather(gd, "blank_hz")
    noise = _gather(gd, "noise_ire")
    terr = _gather(gd, "timing_err")
    clicks = _gather(gd, "clicks")
    meas = np.isfinite(terr)
    steps = []
    for s in gd.segments:
        m = s.lines.measured[s.keep]
        ok = np.isfinite(m)
        steps.append(np.median(s.start + m[ok] - (gd.origin + gd.period * s.abs_n[ok])))
    steps = np.abs(np.diff(steps))
    out = dict(
        name=rec.name, bytes=len(raw), lost=lost, gaps=len(gaps), span_s=(len(raw) + lost) / fs,
        kept_share=len(raw) / (len(raw) + lost), segments=len(gd.segments), failed=gd.failed,
        relocked=gd.relocked, clip_i=clip_i, clip_q=clip_q, fs=fs,
        line_hz=fs / gd.period, period=gd.period, resid_rms=float(np.sqrt(np.mean(gd.fit_resid ** 2))),
        step_p99=float(np.percentile(steps, 99)), step_max=float(steps.max()),
        votes=int(np.sum(gd.frame_votes == gd.frame_offset)), votes_all=len(gd.frame_votes),
        tip_mhz=float(np.median(tip) / 1e6), blank_mhz=float(np.median(blank) / 1e6),
        jitter_ns=float(1.4826 * np.median(np.abs(terr[meas] - np.median(terr[meas]))) / fs * 1e9),
        noise_med=float(np.nanmedian(noise)), noise_p90=float(np.nanpercentile(noise, 90)),
        clicks=float(np.mean(clicks)), lines=int(len(noise)),
    )
    out["sync_mhz"] = out["blank_mhz"] - out["tip_mhz"]
    out["dev"] = out["sync_mhz"] / (simgen.DEV_PP * 40 / 140 / 1e6)
    out["snr_db"] = 20 * np.log10(100 / out["noise_med"])
    return rec, gd, out


def write_mosaics(gd, stem):
    C.RESULTS.mkdir(parents=True, exist_ok=True)
    std = gd.std
    t_end = max(s.start + s.length for s in gd.segments) / gd.fs
    files = []
    for tag, t0, t1 in (("all", None, None), ("last1s", t_end - 1.0, None)):
        frame, count = decoder.mosaic_frame(gd, t0, t1)
        filled = float(np.mean(count > 0))
        frame = np.where(np.isnan(frame), std.black_ire, frame)
        for col in (False, True):
            rgb = colour.decode_frame(frame, std, col)
            f = C.RESULTS / f"{stem}_{tag}{'_colour' if col else ''}.png"
            decoder.write_images([rgb], f, 29.97)
            files.append((f.name, filled))
    return files


def sim_noise_curve(dev, clip_target, cns=(14, 17, 20, 23, 26, 30, 35, 40, 60), seed=0, porch_hz=simgen.PORCH_HZ):
    """Median flat-region noise (IRE) of the simulated chain at the capture's
    layout, rate, deviation, carrier frequency and clipping, per C/N.

    With heavy clipping the I/Q pair is close to hard limited and the
    limiter's odd harmonics alias around the 26.67 MS/s sampling, so the
    noise depends strongly on where the carrier sits: the simulation must use
    the measured blanking frequency, not the -1.0 MHz AFC target."""
    tx = simgen.Transmitter(simgen.TxConfig("ntsc", "scene", dev, porch_hz=porch_hz, seed=seed), 0.04)
    cap = simgen.generate(tx, simgen.RxConfig(seed=40 + seed), [9], 0.04)[9]
    out = []
    for cn in cns:
        run = simgen.ReceiverRun("4x4top", 9, float(cn), agc=simgen.AgcConfig(target=clip_target))
        words, _, _, _ = simgen.quantise(cap, run)
        res = C.decode_words(words, "4x4top", 9, NTSC)
        L = res.lines
        fsc = np.mean(np.abs(PRE_LUT[words].real) >= 480)
        out.append((cn, float(np.nanmedian(L.noise_ire)), float(fsc), float(np.mean(L.clicks))))
    return out


PRE_LUT = None


def run(paths):
    global PRE_LUT
    from ..lanes import PRESETS
    PRE_LUT = PRESETS["4x4top"].build_lut()
    rows = []
    for p in paths:
        rec, gd, o = analyse(p)
        o["mosaics"] = write_mosaics(gd, "R0_" + rec.name)
        rows.append(o)
        print(o, flush=True)
    # Clipping matched: find the AGC target that gives the capture's full-scale share.
    want = np.mean([0.5 * (r["clip_i"] + r["clip_q"]) for r in rows])
    dev = float(np.median([r["dev"] for r in rows]))
    porch = float(np.median([r["blank_mhz"] for r in rows])) * 1e6
    best = None
    for tgt in (900, 1200, 1500, 2000, 2600, 3200):
        c = sim_noise_curve(dev, tgt, cns=(30,), porch_hz=porch)[0][2]
        if best is None or abs(c - want) < abs(best[1] - want):
            best = (tgt, c)
    curve = sim_noise_curve(dev, best[0], porch_hz=porch)
    curve_lin = sim_noise_curve(dev, None, porch_hz=porch)
    # Gain sweep at a strong signal: what clipping costs or gains, at the
    # measured carrier frequency and at the -1.0 MHz AFC target.
    sweep = []
    for ph in (porch, simgen.PORCH_HZ):
        for tgt in (None, 500, 700, 1000, 1400, 2000):
            n, fsc, ck = sim_noise_curve(dev, tgt, cns=(30,), porch_hz=ph)[0][1:]
            sweep.append((ph / 1e6, tgt, n, fsc, ck))
    C.save_json("R0_capture.json", dict(rows=rows, clip_target=best, curve=curve, curve_unclipped=curve_lin,
                                        porch_hz=porch, gain_sweep=sweep))
    write(rows, best, curve, curve_lin, dev, porch, sweep)
    return rows


def equiv_cn(curve, noise):
    cn = np.array([c[0] for c in curve], float)
    nz = np.array([c[1] for c in curve], float)
    o = np.argsort(cn)
    cn, nz = cn[o], nz[o]
    if noise <= nz.min():
        return f"> {cn[np.argmin(nz)]:.0f}"
    if noise >= nz.max():
        return f"< {cn[0]:.0f}"
    # noise decreases with C/N: interpolate on log noise
    return f"{np.interp(-np.log(noise), -np.log(nz), cn):.1f}"


def write(rows, best, curve, curve_lin, dev, porch, sweep):
    L = ["# R0: first hardware capture [hardware]", "",
         "Reference decoder (`decoder.decode_gapped`) on the 2026-10-31 recordings: 4x4top at 26.67 MS/s "
         "(divider 9), 5880 MHz, gain 64, NTSC drone video. The live app dropped most blocks, so every 32 KB "
         "block is demodulated and line-locked on its own, placed by its absolute sample index (file offset "
         "plus the bytes reported lost before it) and numbered on one global line count. Blocks that lie in "
         "the vertical interval lock onto the half-line equalising pulses at first; a second pass re-locks "
         "them on the global grid.", ""]
    hdr = ["quantity"] + [r["name"] for r in rows]
    def row(label, key, fmt):
        return [label] + [fmt.format(r[key]) if not callable(fmt) else fmt(r) for r in rows]
    tab = [
        row("bytes kept / span", "kept_share", lambda r: f"{r['bytes'] / 1e6:.1f} MB of {r['span_s']:.2f} s ({100 * r['kept_share']:.1f} %)"),
        row("usb_gap events / segments", "gaps", lambda r: f"{r['gaps']} / {r['segments']} (failed {r['failed']}, re-locked {r['relocked']})"),
        row("I, Q at full scale", "clip_i", lambda r: f"{100 * r['clip_i']:.0f} %, {100 * r['clip_q']:.0f} %"),
        row("line rate", "line_hz", "{:.2f} Hz"),
        row("global fit residual (RMS)", "resid_rms", lambda r: f"{r['resid_rms']:.1f} samples (smooth clock drift)"),
        row("step between neighbour blocks (p99 / max)", "step_p99", lambda r: f"{r['step_p99']:.2f} / {r['step_max']:.2f} samples"),
        row("frame phase votes agreeing", "votes", lambda r: f"{r['votes']} of {r['votes_all']}"),
        row("blanking / sync tip", "blank_mhz", lambda r: f"{r['blank_mhz']:.2f} / {r['tip_mhz']:.2f} MHz"),
        row("sync depth, implied deviation", "sync_mhz", lambda r: f"{r['sync_mhz']:.2f} MHz, {r['dev']:.2f}"),
        row("0H jitter (robust RMS)", "jitter_ns", "{:.1f} ns"),
        row("flat-region noise (median / p90)", "noise_med", lambda r: f"{r['noise_med']:.2f} / {r['noise_p90']:.2f} IRE"),
        row("unweighted flat-region SNR", "snr_db", "{:.1f} dB"),
        row("repaired samples per line", "clicks", "{:.3f}"),
        row("equivalent C/N (sim, same clipping)", "noise_med", lambda r: equiv_cn(curve, r["noise_med"]) + " dB"),
        row("equivalent C/N (sim, no clipping)", "noise_med", lambda r: equiv_cn(curve_lin, r["noise_med"]) + " dB"),
    ]
    L += [C.md_table(hdr, tab), ""]
    L += [f"Simulator check, 4x4top at 26.67 MS/s, deviation {dev:.2f}, blanking at {porch / 1e6:.2f} MHz (measured), "
          f"AGC target {best[0]} ADC units RMS (full-scale share {100 * best[1]:.0f} %, the capture's clipping) and "
          "the normal 0.45 FS target (230 units RMS):", "",
          C.md_table(["C/N (dB)", "noise, clipped (IRE)", "full scale share", "repaired/line, clipped",
                      "noise, normal gain (IRE)"],
                     [[c[0], f"{c[1]:.2f}", f"{100 * c[2]:.0f} %", f"{c[3]:.2f}", f"{d[1]:.2f}"]
                      for c, d in zip(curve, curve_lin)]), "",
          "Gain sweep at 30 dB C/N (signal RMS in ADC units; the window is +-512 per component):", "",
          C.md_table(["blanking (MHz)", "RMS target", "full scale share", "noise (IRE)", "repaired/line"],
                     [[f"{s[0]:.2f}", "0.45 FS" if s[1] is None else s[1], f"{100 * s[3]:.0f} %", f"{s[2]:.2f}",
                       f"{s[4]:.2f}"] for s in sweep]), "",
          "With the I/Q pair close to hard limited, the limiter's harmonics alias around the sample rate and "
          "the result depends on where the carrier sits. At the measured -1.6 MHz blanking the simulated "
          "clipped chain reproduces the hardware (noise and repaired samples per line); at the -1.0 MHz AFC "
          "target the same clipping is several times noisier.", ""]
    L += ["Mosaic frames (newest copy of each frame line; black where no block covered the line):", ""]
    for r in rows:
        for f, filled in r["mosaics"]:
            L.append(f"- `{f}` ({100 * filled:.0f} % of rows filled)")
    L.append("")
    (C.RESULTS / "R0_first_capture.md").write_text("\n".join(L), encoding="utf-8")


if __name__ == "__main__":
    run(sys.argv[1:])
