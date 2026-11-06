#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare the host FM demodulators and video presets on an attenuation ladder.

Offline only: scores existing recordings with `openvrx-cli score`, one run per
(step, demod, preset), then summarises PSNR per step and the attenuation where
each variant crosses a PSNR level.

    demod_compare.py run LADDER_DIR --ref REF_BASE [--jobs 4] [--demods ...] [--presets ...]
    demod_compare.py summarize LADDER_DIR [--watchable 20] [--lost 10]
    demod_compare.py speed LADDER_DIR --steps a00,a10 [--repeat 3]

LADDER_DIR holds recordings aNN (NN = dB of extra attenuation). Results go to
LADDER_DIR/demod-compare/: runs/*.json (raw score lines), results.csv,
thresholds.csv, speed.csv and summary.md.

Thresholds: the attenuation where PSNR median first falls below the level,
linearly interpolated between the two steps around the crossing (a step below
the level followed by better steps still counts as the crossing, so noisy
ladders give the pessimistic answer). "Lost" uses a lower PSNR level (a step
with no frames at all counts as below it). The lock state is not used: it is
a snapshot at the end of each run and reads Coasting on clean steps.
"""
import argparse
import csv
import json
import re
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

DEMODS = ["polar", "lut", "ls", "pll", "ekf"]
PRESETS = ["raw", "low_latency", "balanced"]
DEFAULT_CLI = Path(__file__).resolve().parents[2] / "host/target/main/release/openvrx-cli"
STEP_RE = re.compile(r"^a(\d+)$")


def steps_of(ladder: Path):
    """(attenuation dB, base path) of every aNN recording, sorted."""
    out = []
    for side in ladder.glob("a*.openvrx.json"):
        name = side.name[: -len(".openvrx.json")]
        m = STEP_RE.match(name)
        if m:
            out.append((int(m.group(1)), ladder / name))
    return sorted(out)


def score(cli, rec, ref, demod, preset, frames, skip, ref_demod=None):
    cmd = [str(cli), "score", str(rec), "--ref", str(ref), "--frames", str(frames),
           "--skip", str(skip), "--demod", demod, "--preset", preset]
    if ref_demod:
        cmd += ["--ref-demod", ref_demod]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}: {r.stderr.strip()}")
    return json.loads(r.stdout.strip().splitlines()[-1])


def cmd_run(a):
    out = a.ladder / "demod-compare" / "runs"
    out.mkdir(parents=True, exist_ok=True)
    jobs = []
    for _, base in steps_of(a.ladder):
        for d in a.demods:
            for p in a.presets:
                jobs.append((base, d, p, None))
    # The reference against itself per chain, and against the polar chain
    # (the latter exposes systematic distortion of a demodulator).
    for d in a.demods:
        for p in a.presets:
            jobs.append((a.ref, d, p, None))
            jobs.append((a.ref, d, p, "polar"))

    def one(job):
        base, d, p, rd = job
        tag = f"-ref{rd}" if rd else ""
        dst = out / f"{Path(base).name}-{d}-{p}{tag}.json"
        if dst.exists() and not a.force:
            return
        s = score(a.cli, base, a.ref, d, p, a.frames, a.skip, rd)
        dst.write_text(json.dumps(s) + "\n")

    with ThreadPoolExecutor(a.jobs) as ex:
        for k, _ in enumerate(ex.map(one, jobs), 1):
            if k % 25 == 0:
                print(f"{k}/{len(jobs)}", file=sys.stderr)
    print(f"{len(jobs)} runs in {out}")


def crossing(points, level):
    """First attenuation where y < level, interpolated; None if never."""
    prev = None
    for x, y in points:
        if y is None or y < level:
            if prev is None or y is None:
                return float(x)
            px, py = prev
            return px + (py - level) / (py - y) * (x - px)
        prev = (x, y)
    return None


def load_runs(ladder: Path):
    rows = []
    for f in sorted((ladder / "demod-compare" / "runs").glob("*.json")):
        d = json.loads(f.read_text())
        m = re.match(r"^(.+?)-(polar|lut|ls|pll|ekf)-(raw|low_latency|balanced|quality)(-ref(\w+))?\.json$", f.name)
        if not m:
            continue
        d["asked_demod"], d["file_preset"], d["ref_kind"] = m.group(2), m.group(3), m.group(5) or "same"
        rows.append(d)
    return rows


def cmd_summarize(a):
    rows = load_runs(a.ladder)
    dc = a.ladder / "demod-compare"
    step_rows = [r for r in rows if STEP_RE.match(r["rec"])]
    ref_rows = [r for r in rows if not STEP_RE.match(r["rec"])]
    with open(dc / "results.csv", "w", newline="") as fh:
        cols = ["rec", "att_db", "asked_demod", "demod", "preset", "ref_kind", "lock", "cn_db", "clip_pct",
                "frames", "psnr_median", "psnr_min", "load", "demod_ns"]
        w = csv.writer(fh)
        w.writerow(cols)
        for r in sorted(rows, key=lambda r: (r["rec"], r["asked_demod"], r["preset"], r["ref_kind"])):
            m = STEP_RE.match(r["rec"])
            w.writerow([r["rec"], int(m.group(1)) if m else "", r["asked_demod"], r["demod"], r["preset"],
                        r["ref_kind"], r["lock"], r["cn_db"], r["clip_pct"], r["frames"], r["psnr_median"],
                        r["psnr_min"], r["load"], r["stage_ns"].get("demod")])

    variants = sorted({(r["asked_demod"], r["preset"]) for r in step_rows},
                      key=lambda v: (PRESETS.index(v[1]) if v[1] in PRESETS else 9, DEMODS.index(v[0])))
    atts = sorted({int(STEP_RE.match(r["rec"]).group(1)) for r in step_rows})
    by = {(r["asked_demod"], r["preset"], int(STEP_RE.match(r["rec"]).group(1))): r for r in step_rows}

    def pts(v):
        # psnr_median is None when no frame came out (picture gone). The
        # lock state is not used: it is a snapshot at the end of the run and
        # reads Coasting on steps with a clean picture.
        return [(x, by[(v[0], v[1], x)]["psnr_median"]) for x in atts if (v[0], v[1], x) in by]

    th = {}
    for v in variants:
        th[v] = (crossing(pts(v), a.watchable), crossing(pts(v), a.lost))
    with open(dc / "thresholds.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["demod", "preset", f"att_below_{a.watchable:g}dB", f"gain_vs_polar_db",
                    f"att_lost_below_{a.lost:g}dB", "lost_gain_vs_polar_db"])
        for v in variants:
            pw, pl = th.get(("polar", v[1]), (None, None))
            vw, vl = th[v]
            g = lambda x, p: "" if x is None or p is None else f"{x - p:.2f}"
            w.writerow([v[0], v[1], "" if vw is None else f"{vw:.2f}", g(vw, pw),
                        "" if vl is None else f"{vl:.2f}", g(vl, pl)])

    # Markdown summary.
    md = [f"# Demodulator comparison: {a.ladder.name}", ""]
    md.append(f"PSNR median (dB) per step; watchable = {a.watchable:g} dB, lost = {a.lost:g} dB or no frames.")
    md.append("")
    for preset in [p for p in PRESETS if any(v[1] == p for v in variants)]:
        vs = [v for v in variants if v[1] == preset]
        md.append(f"## preset {preset}")
        md.append("")
        md.append("| att dB | C/N | " + " | ".join(v[0] for v in vs) + " |")
        md.append("|---" * (len(vs) + 2) + "|")
        for x in atts:
            cn = by.get(("polar", preset, x), {}).get("cn_db", "")
            cells = []
            for v in vs:
                r = by.get((v[0], v[1], x))
                cells.append("" if r is None or r["psnr_median"] is None else f"{r['psnr_median']:.1f}")
            md.append(f"| {x} | {cn} | " + " | ".join(cells) + " |")
        md.append("")
        md.append(f"| demod | below {a.watchable:g} dB at | vs polar | lost at | vs polar |")
        md.append("|---|---|---|---|---|")
        pw, pl = th[("polar", preset)]
        for v in vs:
            vw, vl = th[v]
            f = lambda x: "never" if x is None else f"{x:.1f}"
            g = lambda x, p: "" if x is None or p is None else f"{x - p:+.1f}"
            md.append(f"| {v[0]} | {f(vw)} | {g(vw, pw)} | {f(vl)} | {g(vl, pl)} |")
        md.append("")
    if ref_rows:
        md.append("## reference (strong signal) per chain")
        md.append("")
        md.append("| demod | preset | vs own chain | vs polar chain |")
        md.append("|---|---|---|---|")
        rr = {(r["asked_demod"], r["preset"], r["ref_kind"]): r["psnr_median"] for r in ref_rows}
        for v in variants:
            md.append(f"| {v[0]} | {v[1]} | {rr.get((v[0], v[1], 'same'))} | {rr.get((v[0], v[1], 'polar'))} |")
        md.append("")
    sp = dc / "speed.csv"
    if sp.exists():
        md.append("## speed (one job at a time)")
        md.append("")
        md.append("| demod | demod ns/sample | DSP total ns/sample | load |")
        md.append("|---|---|---|---|")
        for row in csv.DictReader(open(sp)):
            md.append(f"| {row['demod']} | {row['demod_ns']} | {row['total_ns']} | {row['load']} |")
        md.append("")
    (dc / "summary.md").write_text("\n".join(md))
    print("\n".join(md))


def cmd_speed(a):
    """DSP cost per demod, one job at a time (parallel runs skew `load`)."""
    dc = a.ladder / "demod-compare"
    dc.mkdir(parents=True, exist_ok=True)
    rows = []
    for d in a.demods:
        demod_ns, total_ns, load = [], [], []
        for step in a.steps.split(","):
            for _ in range(a.repeat):
                s = score(a.cli, a.ladder / step, a.ladder / step, d, "raw", a.frames, a.skip)
                demod_ns.append(s["stage_ns"]["demod"])
                total_ns.append(sum(s["stage_ns"].values()))
                load.append(s["load"])
        rows.append({"demod": d, "demod_ns": f"{statistics.median(demod_ns):.2f}",
                     "total_ns": f"{statistics.median(total_ns):.2f}", "load": f"{statistics.median(load):.3f}"})
        print(rows[-1])
    with open(dc / "speed.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cli", type=Path, default=DEFAULT_CLI, help="openvrx-cli binary")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="score every step for every demod and preset")
    r.add_argument("ladder", type=Path)
    r.add_argument("--ref", type=Path, required=True, help="reference recording base path")
    r.add_argument("--jobs", type=int, default=4)
    r.add_argument("--demods", nargs="+", default=DEMODS)
    r.add_argument("--presets", nargs="+", default=PRESETS)
    r.add_argument("--frames", type=int, default=30)
    r.add_argument("--skip", type=int, default=10)
    r.add_argument("--force", action="store_true", help="rerun runs that already have a result")
    s = sub.add_parser("summarize", help="results.csv, thresholds.csv, summary.md")
    s.add_argument("ladder", type=Path)
    s.add_argument("--watchable", type=float, default=20.0, help="PSNR level called watchable (dB)")
    s.add_argument("--lost", type=float, default=10.0, help="PSNR level called lost (dB)")
    p = sub.add_parser("speed", help="DSP cost per demod, serial")
    p.add_argument("ladder", type=Path)
    p.add_argument("--steps", default="a00,a10", help="comma list of steps to time")
    p.add_argument("--demods", nargs="+", default=DEMODS)
    p.add_argument("--repeat", type=int, default=3)
    p.add_argument("--frames", type=int, default=30)
    p.add_argument("--skip", type=int, default=10)
    a = ap.parse_args()
    {"run": cmd_run, "summarize": cmd_summarize, "speed": cmd_speed}[a.cmd](a)


if __name__ == "__main__":
    main()
