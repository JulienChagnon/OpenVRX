# SPDX-License-Identifier: GPL-3.0-or-later
"""Shared helpers for the experiments: evaluation, thresholds, parallel map,
Markdown tables and plots."""
from __future__ import annotations

import json
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .. import decoder, metrics as M, simgen
from ..lanes import PRESETS, SampleFormat

RESULTS = Path(__file__).resolve().parents[2] / "results"
PICTURE = "image:astronaut"
BREAK_IRE = 10.0     # a line is "broken" when its RMS picture error exceeds this


def workers(n=None):
    return n or int(os.environ.get("OPENVRX_WORKERS", "0")) or max(1, min(6, (os.cpu_count() or 4) - 2))


def pmap(fn, jobs, n=None):
    t = time.time()
    with ProcessPoolExecutor(max_workers=workers(n)) as ex:
        out = list(ex.map(fn, jobs))
    print(f"  {len(jobs)} jobs in {time.time() - t:.0f} s", flush=True)
    return out


def pmap_cached(fn, jobs, tag, n=None, tries=4):
    """pmap with one JSON file per finished job under results/cache/, so a
    run that dies (out of memory on a busy machine) resumes where it stopped."""
    import hashlib
    from concurrent.futures import as_completed
    cache = RESULTS / "cache"
    cache.mkdir(parents=True, exist_ok=True)

    def path(job):
        return cache / f"{tag}_{hashlib.sha1(repr(job).encode()).hexdigest()[:16]}.json"

    t = time.time()
    for attempt in range(tries):
        todo = [j for j in jobs if not path(j).exists()]
        if not todo:
            break
        try:
            with ProcessPoolExecutor(max_workers=workers(n)) as ex:
                futs = {ex.submit(fn, j): j for j in todo}
                for f in as_completed(futs):
                    path(futs[f]).write_text(json.dumps(f.result(), default=float), encoding="utf-8")
        except Exception as exc:  # broken pool: retry what is missing
            print(f"  attempt {attempt + 1}: {type(exc).__name__}, retrying missing jobs", flush=True)
    missing = [j for j in jobs if not path(j).exists()]
    if missing:
        raise RuntimeError(f"{len(missing)} jobs failed")
    print(f"  {len(jobs)} jobs in {time.time() - t:.0f} s", flush=True)
    return [json.loads(path(j).read_text(encoding="utf-8")) for j in jobs]


def truth_frame_lines(tx):
    return tx.frame_line_of(np.arange(tx.n_lines))


def decode_words(words, preset, divider, std, dcfg=None):
    fmt = PRESETS[preset]
    z = fmt.build_lut()[words]
    return decoder.decode_iq(z, 240e6 / divider, dcfg or decoder.DecoderConfig(), std, fmt.has_magnitude)


END_TRIM = 4         # lines dropped at each end of a capture (filter tails, not the format)


def compare(res, tx, cap, offset=None):
    cmp = M.match_lines(res.lines, tx.lines, truth_frame_lines(tx), cap.line_a, cap.line_b,
                        cap.delay_samples, offset)
    return trim_ends(cmp, tx)


def trim_ends(cmp, tx, n=END_TRIM):
    # Truth lines whose 0H falls in the first or last n lines of the capture.
    first_ok, last_ok = n, tx.n_lines - 1 - n
    ti = cmp.truth_index
    if len(ti) == 0:
        return cmp
    keep = (ti >= ti.min() + n) & (ti <= ti.max() - n) & (ti >= first_ok) & (ti <= last_ok)
    return M.Comparison(cmp.std, cmp.dec[keep], cmp.truth[keep], ti[keep], cmp.label_ok, cmp.coverage)


def line_rms(cmp, cal):
    e, _ = M._err(cmp, cal)
    return np.sqrt(np.mean(e * e, axis=1))


def scores(cmp, cal, extra=None):
    d = {"sinad": M.sinad_db(cmp, cal), "wsnr": M.wsnr_db(cmp, cal), "coverage": cmp.coverage,
         "label_ok": cmp.label_ok}
    lr = line_rms(cmp, cal)
    d["breakup"] = float(np.mean(lr > BREAK_IRE)) if len(lr) else 1.0
    # Missing lines count as broken.
    d["breakup"] = 1 - (1 - d["breakup"]) * min(1.0, cmp.coverage)
    if extra:
        d.update(extra)
    return d


def evaluate(tx, cap, run, dcfg=None, cal=None, psnr=False):
    """Quantise, decode, score one receiver run. Failure scores as no picture."""
    words, _, _, _ = simgen.quantise(cap, run)
    try:
        res = decode_words(words, run.preset, run.divider, tx.std, dcfg)
        cmp = compare(res, tx, cap)
    except Exception as exc:  # no sync at all: no picture
        return {"sinad": np.nan, "wsnr": np.nan, "coverage": 0.0, "label_ok": 0.0, "breakup": 1.0,
                "error": str(exc)[:80]}, None
    cal = cal or M.calibrate(cmp)
    d = scores(cmp, cal, {"clicks_per_line": float(np.mean(res.lines.clicks))})
    if psnr:
        a, b = M.comparison_luma_rows(cmp)
        d["psnr"], d["ssim"] = M.psnr_ssim(b, a)
    return d, cal


def threshold(cn, y, level):
    """Lowest C/N above which the curve stays at or above ``level``
    (linear interpolation); NaN if never reached."""
    cn, y = np.asarray(cn, float), np.asarray(y, float)
    o = np.argsort(cn)
    cn, y = cn[o], y[o]
    ok = np.nan_to_num(y, nan=-99) >= level
    if not ok[-1]:
        return np.nan
    k = len(ok) - 1
    while k > 0 and ok[k - 1]:
        k -= 1
    if k == 0:
        return float(cn[0]) if ok[0] else np.nan
    y0, y1 = np.nan_to_num(y[k - 1], nan=-99), y[k]
    return float(cn[k - 1] + (level - y0) * (cn[k] - cn[k - 1]) / (y1 - y0))


def threshold_down(cn, y, level):
    """Lowest C/N above which ``y`` (e.g. breakup share) stays <= level."""
    return threshold(cn, -np.asarray(y, float), -level)


def fmt_db(x, nd=1):
    return "n/a" if x is None or not np.isfinite(x) else f"{x:.{nd}f}"


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---:" if i else "---" for i in range(len(header))) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def save_json(name, obj):
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / name).write_text(json.dumps(obj, indent=1, default=float), encoding="utf-8")


def rate_label(preset, divider):
    sf = SampleFormat.from_preset(preset, divider)
    return f"{sf.sample_rate_hz / 1e6:.2f} MS/s", sf.byte_rate / 1e6, sf.data_pins


@dataclass
class Spread:
    median: float
    lo: float
    hi: float

    def __str__(self):
        if not np.isfinite(self.median):
            return "n/a"
        return f"{self.median:.1f} ({self.lo:.1f}..{self.hi:.1f})"


def spread(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    if len(v) == 0:
        return Spread(np.nan, np.nan, np.nan)
    return Spread(float(np.median(v)), float(v.min()), float(v.max()))
