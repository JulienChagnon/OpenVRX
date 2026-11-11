#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Noisy/clean frame pairs from a bench ladder, for the denoise models.

A `bench.py --patterns ...` run records every ESP32 test pattern at every
attenuator step. The patterns are static, so the clean picture of a pattern
is the mean of many decoded fields of its strongest steps: the noise
averages out, while everything the receiver and the decoder do to a clean
signal (filters, sampling, the 4-bit quantisation) stays in. The result is
what this receiver chain should show; the MS2107 truth (truth-NNN.mkv) is
kept for checking that reference, not used as the target, because its
geometry and levels differ from the decode.

Fields of the two parities sit half a line apart, so the mean is taken per
parity, and the reference sequence alternates the two like the decode does
(`denoise_filter.align` then finds the parity phase as a frame offset).

Layout written (the one denoise_filter.py and denoise_cnn.py read):

    DATA/bench-<pattern>/ref/f0000.png ...      clean, alternating parity
    DATA/bench-<pattern>/cnXX.X-attYY.YY/...    decoded fields of one step
    DATA/bench-<pattern>/damage/cnXX.X-attYY.YY/...  their decoder damage masks

    bench_dataset.py build --run RUN_DIR[,RUN_DIR2] --data DATA [--fields 200]
                           [--ref-margin 2] [--min-cn 3] [--jobs 3]

--ref-margin: steps whose C/N is within this many dB of the pattern's best
(and picture ok, clip under 1 %) go into the reference. --min-cn: weaker
steps are left out (no lock, nothing to learn from). Writes DATA/bench-
<pattern>/build.json with the steps used and their scores.

Moving pictures (a clip from `prep_clips.py` looping as the transmitted
picture, `bench.py` without --patterns):

    bench_dataset.py video --run RUN_DIR[,RUN_DIR2] --data DATA --clips CLIPS_DIR --clip NAME
                           [--ref-steps 3] [--ref-margin 2] [--min-cn 3] [--jobs 3]

A mean over fields cannot be the reference of a moving picture, so each
step's recording is decoded whole and the clip's slate (black, white,
black) found from the mean brightness per field: it fixes the loop phase,
so every field of the recording has a loop position ((field - slate end)
mod loop length). Record each step at least one loop plus a second. The
reference loop is the strongest step's fields by position, averaged with
the next strongest steps (up to --ref-steps within --ref-margin dB) where
their fields match exactly (best of positions -2..+2). Every weaker step
becomes its own scene DATA/video-NAME-attYY.YY: its fields in recorded
order (real consecutive fields) and a ref sequence hard linked from the
reference loop in DATA/video-NAME/pool (no extra disk). Left out and
reported: a step whose loop sits on the other field parity (half a line
off: keep mpv playing through the whole ladder), and a step where under
90 % of sampled fields match their position (dropped or repeated pictures:
play the loop at 59.94 Hz). DATA/video-NAME/build.json lists the
scenes (for prepare and overnight.py --extra-train).
`picture` in results.jsonl is not used for video steps: bench.py grades it
against the strongest step frame by frame, which does not line up in time
for a moving clip.

Needs numpy and Pillow; openvrx-cli from host/target/main/release (or --cli).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SAVED = re.compile(r"saved (\S+\.png) \(field \d+ parity Some\((First|Second)\)\)")


def decode(cli, rec, out, fields):
    """Decodes `fields` fields of a recording into out/; returns [(png, parity)]."""
    shutil.rmtree(out, ignore_errors=True)
    shutil.rmtree(out + "-damage", ignore_errors=True)
    os.makedirs(out)
    # Damage masks (same names) in OUT-damage, for the mask-aware models.
    p = subprocess.run([cli, "decode", rec, out, str(fields), "--damage", out + "-damage"],
                       capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"decode {rec}: {p.stderr.strip()[-300:]}")
    got = [(m.group(1), m.group(2)) for m in SAVED.finditer(p.stdout + p.stderr)]
    with open(os.path.join(out, "parity.json"), "w") as f:
        json.dump([[os.path.basename(a), b] for a, b in got], f)
    return got


def read_rgb(png, tries=3):
    """A decoded field as float RGB, or None when it cannot be read. A read
    once failed on an intact file ("unrecognized data stream", 2026-11-07,
    disk busy with other jobs) and ended an unattended run: retry, then skip
    that one field rather than the whole build."""
    for i in range(tries):
        try:
            return np.asarray(Image.open(png).convert("RGB"), dtype=np.float64)
        except OSError as e:
            if i == tries - 1:
                print(f"  WARNING: {png} unreadable ({e}), field left out of the reference")
                return None
            time.sleep(1.0)


def cmd_build(a):
    rows = []
    for run in a.run:  # one ladder may be split over folders (a resumed sweep)
        for l in open(os.path.join(run, "results.jsonl")):
            if l.strip():
                r = json.loads(l)
                if r.get("pattern"):
                    r["_dir"] = run
                    rows.append(r)
    if not rows:
        raise SystemExit("no pattern rows in results.jsonl (run bench.py with --patterns)")
    work = os.path.join(a.data, "_decode")
    os.makedirs(work, exist_ok=True)

    def job(r):
        name = os.path.basename(r["rec"]) if "rec" in r else f"g{r['gain']}-att{r['att_db']:05.2f}-{r['pattern']}"
        return r, decode(a.cli, os.path.join(r["_dir"], name), os.path.join(work, name), a.fields)

    # Recordings with more host clip than --max-clip are left out.
    keep = [r for r in rows if r["cn_db"] is not None and r["cn_db"] >= a.min_cn and r["frames"] > 0
            and r["clip_pct"] <= a.max_clip]
    print(f"{sum(1 for r in rows if r['clip_pct'] > a.max_clip)} recordings over {a.max_clip} % host clip left out")
    print(f"decoding {len(keep)} of {len(rows)} recordings ({a.fields} fields each)")
    with ThreadPoolExecutor(a.jobs) as ex:
        done = list(ex.map(job, keep))

    for pattern in sorted({r["pattern"] for r, _ in done}):
        mine = [(r, d) for r, d in done if r["pattern"] == pattern]
        good = [(r, d) for r, d in mine if r["picture"] == "ok" and r["clip_pct"] < 1.0]
        if not good:
            print(f"{pattern}: no clean step, skipped")
            continue
        best = max(r["cn_db"] for r, _ in good)
        ref_runs = [(r, d) for r, d in good if r["cn_db"] >= best - a.ref_margin]
        sums = {"First": None, "Second": None}
        counts = {"First": 0, "Second": 0}
        for _, fields in ref_runs:
            for png, par in fields:
                x = read_rgb(png)
                if x is None:
                    continue
                sums[par] = x if sums[par] is None else sums[par] + x
                counts[par] += 1
        if not counts["First"] or not counts["Second"]:
            print(f"{pattern}: one parity missing in the reference runs, skipped")
            continue
        means = {p: np.clip(sums[p] / counts[p] + 0.5, 0, 255).astype(np.uint8) for p in sums}
        scene = os.path.join(a.data, f"bench-{pattern}")
        shutil.rmtree(scene, ignore_errors=True)
        os.makedirs(os.path.join(scene, "ref"))
        n = max(len(f) for _, f in mine)
        first = mine[0][1][0][1] if mine[0][1] else "First"
        order = [first, "Second" if first == "First" else "First"]
        for k in range(n):
            Image.fromarray(means[order[k % 2]]).save(os.path.join(scene, "ref", f"f{k:04d}.png"))
        runs = []
        for r, fields in mine:
            if (r, fields) in ref_runs:
                continue  # never score a run against a mean it is part of
            dst = os.path.join(scene, f"cn{r['cn_db']:04.1f}-att{r['att_db']:05.2f}")
            os.makedirs(dst, exist_ok=True)
            for i, (png, par) in enumerate(fields):
                shutil.copy(png, os.path.join(dst, f"f{i:04d}.png"))
                mask = os.path.join(os.path.dirname(png) + "-damage", os.path.basename(png))
                if os.path.exists(mask):
                    mdst = os.path.join(scene, "damage", os.path.basename(dst))
                    os.makedirs(mdst, exist_ok=True)
                    shutil.copy(mask, os.path.join(mdst, f"f{i:04d}.png"))
            runs.append({"dir": os.path.basename(dst), "att_db": r["att_db"], "cn_db": r["cn_db"],
                         "psnr_median": r["psnr_median"], "lock": r["lock"], "fields": len(fields)})
        info = {"runs": [os.path.abspath(r) for r in a.run], "pattern": pattern,
                "ref_steps": [r["att_db"] for r, _ in ref_runs], "ref_fields": counts, "best_cn": best,
                "noisy_runs": runs}
        with open(os.path.join(scene, "build.json"), "w") as f:
            json.dump(info, f, indent=2)
        print(f"bench-{pattern}: reference from steps {info['ref_steps']} ({counts} fields, best C/N {best}), "
              f"{len(runs)} noisy runs")
    if not a.keep_decodes:
        shutil.rmtree(work, ignore_errors=True)


def small_luma(png, step=4):
    """Luma of a field at 1/step resolution (for slate finding and matching)."""
    a = np.asarray(Image.open(png).convert("L"), dtype=np.float32)
    return a[::step, ::step]


def find_pass(means, slate):
    """First field after the first complete slate (black, white, black of
    `slate` fields each, from the per-field mean brightness), or None.

    Black and white are judged against the darkest and brightest fields
    (1st and 99.9th percentile: a dark clip can fill far more than 5 % of the
    fields with near-black). Fields at the slate's edges can be part one
    picture, part the next (the source is not frame-locked to the fields),
    so the white part is a bright stretch with dips of up to 3 fields closed,
    about `slate` long, mostly dark on both sides; its middle places the pass
    start (white start + 2 slate parts), so an edge glitch cannot move it."""
    m = np.asarray(means, dtype=float)
    lo, hi = np.percentile(m, 1), np.percentile(m, 99.9)
    if hi - lo < 40:  # no slate visible (lost picture)
        return None
    white = m > lo + 0.6 * (hi - lo)
    dark = m < lo + 0.3 * (hi - lo)
    n = len(m)
    # Close short dips inside bright stretches.
    idx = np.flatnonzero(white)
    for a, b in zip(idx[:-1], idx[1:]):
        if 1 < b - a <= 4:
            white[a:b] = True
    i = 0
    while i < n:
        if not white[i]:
            i += 1
            continue
        j = i
        while j < n and white[j]:
            j += 1
        length = j - i
        before = dark[max(0, i - slate):i]
        after = dark[j:j + slate]
        if (abs(length - slate) <= max(2, slate // 5) and len(before) >= slate // 2 and before.mean() >= 0.6
                and len(after) == slate and after.mean() >= 0.6):
            start = int(round((i + j - 1) / 2 - (slate - 1) / 2)) + 2 * slate
            return start if start < n else None
        i = j
    return None


def cmd_video(a):
    clips = json.load(open(os.path.join(a.clips, "clips.json")))
    if a.clip not in clips:
        raise SystemExit(f"{a.clip} not in {a.clips}/clips.json (have: {', '.join(clips)})")
    info = clips[a.clip]
    slate = info["slate_frames"] // 3
    length = info["frames"] - info["slate_frames"]  # fields of one pass after the slate
    rows = []
    for run in a.run:
        for l in open(os.path.join(run, "results.jsonl")):
            if l.strip():
                r = json.loads(l)
                if not r.get("pattern"):
                    r["_dir"] = run
                    rows.append(r)
    keep = [r for r in rows if r["cn_db"] is not None and r["cn_db"] >= a.min_cn and r["frames"] > 0
            and r["clip_pct"] <= a.max_clip]
    if not keep:
        raise SystemExit("no usable video steps in results.jsonl (rows without a pattern, C/N over --min-cn)")
    work = os.path.join(a.data, "_decode")
    os.makedirs(work, exist_ok=True)
    print(f"decoding {len(keep)} recordings whole (clip {a.clip}: {length} fields after a {3 * slate}-field slate)")

    def job(r):
        name = f"g{r['gain']}-att{r['att_db']:05.2f}"
        fields = decode(a.cli, os.path.join(r["_dir"], name), os.path.join(work, name), 10 ** 6)
        means = [float(small_luma(p).mean()) for p, _ in fields]
        start = find_pass(means, slate)
        return r, fields, start

    with ThreadPoolExecutor(a.jobs) as ex:
        done = list(ex.map(job, keep))
    loop = info["frames"]  # fields per loop, slate included
    usable = []
    for r, fields, p0 in done:
        if p0 is None:
            print(f"  att {r['att_db']:5.2f} (C/N {r['cn_db']}): no slate found, left out")
        else:
            usable.append((r, fields, p0))
    if not usable:
        raise SystemExit("no recording shows the slate (black, white, black): is the clip looping?")
    usable.sort(key=lambda u: -u[0]["cn_db"])

    def pos(t, p0):
        """Loop position of recording field t (0 = first clip field after the slate)."""
        return (t - p0) % loop

    def phase(fields, p0):
        """Parity of the field at loop position 0 (the same for every pass of an
        even-length loop): steps with another phase are half a line off."""
        t = p0 if p0 < len(fields) else p0 - loop
        return fields[t][1]

    # Reference loop: the best step's fields by loop position, averaged with
    # the next strongest steps' fields at positions where they match exactly.
    best_r, best_f, best_p = usable[0]
    ref_phase = phase(best_f, best_p)
    ref_t = {}
    for t in range(len(best_f)):
        ref_t.setdefault(pos(t, best_p), t)
    missing = loop - len(ref_t)
    if missing:
        raise SystemExit(f"the strongest step covers only {len(ref_t)} of the {loop} loop fields: "
                         "record each step at least one loop (plus a second) long")
    ref_small = {i: small_luma(best_f[t][0]) for i, t in ref_t.items()}
    sums = {i: read_rgb(best_f[t][0]) for i, t in ref_t.items()}
    counts = {i: 1 for i in ref_t}

    def matches(y, i):
        """Field luma y shows loop position i: its error against position i is
        the smallest of i-2..i+2, within 2 % (flat pictures such as the slate
        tie with their neighbours; a moving picture one field off does not)."""
        err = {j: float(np.mean((y - ref_small[j % loop]) ** 2)) for j in range(i - 2, i + 3)}
        return err[i] <= 1.02 * min(err.values()) + 0.5

    ref_steps = [best_r["att_db"]]
    for r, fields, p0 in usable[1:]:
        if len(ref_steps) >= a.ref_steps or r["cn_db"] < best_r["cn_db"] - a.ref_margin:
            break
        if phase(fields, p0) != ref_phase:
            continue
        used = 0
        for t, (png, _) in enumerate(fields):
            i = pos(t, p0)
            if matches(small_luma(png), i):
                x = read_rgb(png)
                if x is not None and sums[i] is not None:
                    sums[i] += x
                    counts[i] += 1
                    used += 1
        if used:
            ref_steps.append(r["att_db"])
    scene = os.path.join(a.data, f"video-{a.clip}")
    shutil.rmtree(scene, ignore_errors=True)
    pool = os.path.join(scene, "pool")
    os.makedirs(pool)
    for i in range(loop):
        if sums[i] is None:
            raise SystemExit(f"reference field {i} unreadable")
        Image.fromarray(np.clip(sums[i] / counts[i] + 0.5, 0, 255).astype(np.uint8)).save(
            os.path.join(pool, f"r{i:05d}.png"))

    # Every weaker step: its own scene, its fields in recorded order (real
    # consecutive fields for the temporal models), each paired with the
    # reference field of its loop position (hard links into the pool).
    scenes, runs, skipped = [], [], []
    for r, fields, p0 in usable:
        if r["att_db"] in ref_steps:
            continue  # never score a run against a reference it is part of
        if phase(fields, p0) != ref_phase:
            skipped.append({"att_db": r["att_db"], "why": "other field parity"})
            print(f"  att {r['att_db']:5.2f}: loop on the other field parity (half a line off), left out")
            continue
        probe = range(0, len(fields), max(1, len(fields) // 40))
        good = sum(matches(small_luma(fields[t][0]), pos(t, p0)) for t in probe) / len(probe)
        if good < 0.9:
            skipped.append({"att_db": r["att_db"], "why": f"only {good:.0%} of fields at their loop position"})
            print(f"  att {r['att_db']:5.2f}: only {good:.0%} of fields where the slate puts them "
                  "(dropped or repeated pictures?), left out")
            continue
        name = f"video-{a.clip}-att{r['att_db']:05.2f}"
        dst_scene = os.path.join(a.data, name)
        shutil.rmtree(dst_scene, ignore_errors=True)
        run_name = f"cn{r['cn_db']:04.1f}-att{r['att_db']:05.2f}"
        dst, rdst = os.path.join(dst_scene, run_name), os.path.join(dst_scene, "ref")
        mdst = os.path.join(dst_scene, "damage", run_name)
        os.makedirs(dst)
        os.makedirs(rdst)
        for t, (png, _) in enumerate(fields):
            os.link(os.path.join(pool, f"r{pos(t, p0):05d}.png"), os.path.join(rdst, f"f{t:04d}.png"))
            shutil.copy(png, os.path.join(dst, f"f{t:04d}.png"))
            mask = os.path.join(os.path.dirname(png) + "-damage", os.path.basename(png))
            if os.path.exists(mask):
                os.makedirs(mdst, exist_ok=True)
                shutil.copy(mask, os.path.join(mdst, f"f{t:04d}.png"))
        with open(os.path.join(dst_scene, "build.json"), "w") as f:
            json.dump({"video": a.clip, "ref_pool": os.path.abspath(pool), "slate_end_field": p0}, f)
        scenes.append(name)
        runs.append({"dir": run_name, "scene": name, "att_db": r["att_db"], "cn_db": r["cn_db"],
                     "lock": r.get("lock"), "fields": len(fields), "at_loop_position": round(good, 3)})
    with open(os.path.join(scene, "build.json"), "w") as f:
        json.dump({"video": a.clip, "clip": info, "runs_dirs": [os.path.abspath(x) for x in a.run],
                   "ref_steps": ref_steps, "ref_fields_averaged": [counts[i] for i in range(loop)],
                   "scenes": scenes, "noisy_runs": runs, "skipped": skipped}, f, indent=2)
    print(f"video-{a.clip}: reference loop from steps {ref_steps}, {len(runs)} noisy runs, scenes: "
          f"{','.join(scenes)}" + (f"; {len(skipped)} left out" if skipped else ""))
    if not a.keep_decodes:
        shutil.rmtree(work, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("video")
    v.add_argument("--run", required=True, type=lambda s: [x for x in s.split(",") if x],
                   help="bench.py run folder(s) of a video ladder")
    v.add_argument("--data", required=True)
    v.add_argument("--clips", required=True, help="prep_clips.py output folder (clips.json)")
    v.add_argument("--clip", required=True, help="the clip that was looping (name in clips.json)")
    v.add_argument("--ref-steps", type=int, default=3, help="strongest steps averaged into the reference")
    v.add_argument("--ref-margin", type=float, default=2.0, help="dB under the best C/N still used for the reference")
    v.add_argument("--min-cn", type=float, default=3.0)
    v.add_argument("--max-clip", type=float, default=1.0)
    v.add_argument("--jobs", type=int, default=3)
    v.add_argument("--keep-decodes", action="store_true")
    v.add_argument("--cli", default=os.path.join(REPO, "host", "target", "main", "release", "openvrx-cli"))
    b = sub.add_parser("build")
    b.add_argument("--run", required=True, type=lambda v: [x for x in v.split(",") if x],
                   help="bench.py run folder(s), comma separated (results.jsonl and recordings)")
    b.add_argument("--data", required=True, help="dataset folder (scenes are added or replaced)")
    b.add_argument("--fields", type=int, default=200, help="fields decoded per recording")
    b.add_argument("--ref-margin", type=float, default=2.0, help="dB under the best C/N still used for the reference")
    b.add_argument("--min-cn", type=float, default=3.0, help="steps under this C/N are left out")
    b.add_argument("--max-clip", type=float, default=1.0,
                   help="recordings with more host clip (%%) are left out (FX2 lane faults)")
    b.add_argument("--jobs", type=int, default=3)
    b.add_argument("--keep-decodes", action="store_true", help="keep DATA/_decode")
    b.add_argument("--cli", default=os.path.join(REPO, "host", "target", "main", "release", "openvrx-cli"))
    a = ap.parse_args()
    {"build": cmd_build, "video": cmd_video}[a.cmd](a)


if __name__ == "__main__":
    main()
