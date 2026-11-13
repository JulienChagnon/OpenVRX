#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Unattended denoise pipeline: data, a long CPU training, eval, whole-flight videos.

One command for a night (or any time budget) with nobody watching. Each step
writes its outputs to files and is marked done in OUT/state.json, so running
the same command again (after a crash, a reboot, Ctrl+C) skips finished steps
and the training continues from its last checkpoint. Re-executes itself under
systemd-inhibit so Atlas does not suspend while it runs.

    overnight.py --name v3 [--hours 8] [--data ~/openvrx-data/denoise-data]
                 [--width 32] [--depth 10] [--sat-weight 4] [--colour-start 0.4] [--no-mask]
                 [--cn 2,4,6,8,10,12,14,16,20] [--crops 64] [--size 96]
                 [--render REC[,REC2]] [--panels raw,model] [--compare A.pt,B.pt]
                 [--steps simulate,bench,prepare,train,eval,gallery,render] [--smoke]
                 [--kaggle USER [--iters N] [--no-amp]] [--extra-train A,B] [--extra-val C]

Steps (all outputs under DATA; OUT = DATA/overnight-NAME):
  simulate  room0..room5 through the simulator at every --cn, with decoder
            damage masks (the references are kept; existing runs are redone
            with the same seed, adding their masks)
  bench     the bench pattern scenes rebuilt with masks from --bench-runs
  prepare   crops: train = room0..4 + bench card/bars/check, val = room5 +
            bench ramp/grey (the v1/v2 split), to OUT/train.npz, OUT/val.npz
  train     DATA/cnn-NAME.pt; the iteration count is set once from --hours
            and a short speed test, then kept in state.json (checkpoints
            need it fixed)
  eval      room5, bench-ramp, bench-grey: raw, linear, --compare models and
            the new one, to OUT/eval-*.json
  gallery   media.py: room5 before/after and the flight cuts (incl. the keep
            cut), one panel per model, in ~/openvrx-data/media
  render    every --render recording, whole, to DATA/renders/NAME/*.mp4

--kaggle USER: the train step runs on a Kaggle T4 instead (kaggle_train.py):
the crops go up as the private dataset USER/openvrx-denoise-NAME, the job
USER/openvrx-train-NAME trains with float16 convolutions (--no-amp: off),
and the model comes back to DATA/cnn-NAME.pt; the other steps stay on this
machine. --iters sets the length (no speed test there). Free jobs stop at 12 h: if one
does not finish, rerun the command; the checkpoint fetched from the job goes
up with the crops and the training continues from it.

OUT/log.txt has everything printed; OUT/SUMMARY.md the eval tables and the
video paths at the end. --smoke runs every step tiny (minutes, not hours) to
check the pipeline. Needs torch, numpy, Pillow, ffmpeg, the release
openvrx-cli and openvrx-sim (host/target/main/release).
"""
import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
BIN = os.path.join(REPO, "host", "target", "main", "release")
LINEAR = os.path.join(REPO, "host", "apps", "openvrx-cli", "filters", "denoise-4x4top.json")
PY = sys.executable
ROOMS_TRAIN = ["room0", "room1", "room2", "room3", "room4"]
ROOMS_VAL = ["room5"]
BENCH_TRAIN = ["bench-card", "bench-bars", "bench-check"]
BENCH_VAL = ["bench-ramp", "bench-grey"]
FLIGHT_CUTS = ["~/openvrx-data/flights/keep/f1106-022948-t4/f1106-022948-t4", "~/openvrx-data/flights/cuts/f1106-022457-t13",
               "~/openvrx-data/flights/cuts/f1031-214743-t36", "~/openvrx-data/flights/cuts/f1102-235032-t14"]
STEPS = ["simulate", "bench", "prepare", "train", "eval", "gallery", "render"]


class Run:
    def __init__(self, out):
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.state_path = os.path.join(out, "state.json")
        self.state = json.load(open(self.state_path)) if os.path.exists(self.state_path) else {"done": []}
        self.logf = open(os.path.join(out, "log.txt"), "a")

    def save(self):
        with open(self.state_path + ".tmp", "w") as f:
            json.dump(self.state, f, indent=1)
        os.replace(self.state_path + ".tmp", self.state_path)

    def log(self, msg):
        line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
        print(line, flush=True)
        self.logf.write(line + "\n")
        self.logf.flush()

    def sh(self, cmd):
        """Runs a command, its output to the terminal and log.txt; stops the
        pipeline on failure (rerun the same command to continue)."""
        self.log("$ " + " ".join(cmd))
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            sys.stdout.write(line)
            self.logf.write(line)
            self.logf.flush()
        if p.wait() != 0:
            self.log(f"FAILED (exit {p.returncode}): rerun the same command to continue from here")
            sys.exit(p.returncode)

    def step(self, name, fn):
        if name in self.state["done"]:
            self.log(f"== {name}: done before, skipped")
            return
        self.log(f"== {name}")
        t0 = time.time()
        fn()
        self.state["done"].append(name)
        self.save()
        self.log(f"== {name}: done in {(time.time() - t0) / 60:.1f} min")


def speed_test(width, depth, mask, size, batch, threads):
    """Seconds per training iteration of this model on this CPU (20 timed
    iterations after 3 warm-up ones, random data)."""
    sys.path.insert(0, HERE)
    import torch
    import denoise_cnn as cnn
    torch.set_num_threads(threads)
    m = cnn.Denoiser(width, depth, mask)
    opt = torch.optim.Adam(m.parameters(), 1e-3)
    x = torch.rand(batch, 9 + 3 * mask, size, size)
    y = torch.rand(batch, 3, size, size)
    for i in range(23):
        if i == 3:
            t0 = time.time()
        loss = (m(x) - y).abs().mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
    return (time.time() - t0) / 20


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    lst = lambda v: [x for x in v.split(",") if x]  # noqa: E731
    ap.add_argument("--name", required=True, help="model name: DATA/cnn-NAME.pt, DATA/overnight-NAME/")
    ap.add_argument("--data", default="~/openvrx-data/denoise-data")
    ap.add_argument("--hours", type=float, default=8.0, help="training time budget (the other steps add about 1 to 2 h)")
    ap.add_argument("--width", type=int, default=32)
    ap.add_argument("--depth", type=int, default=10)
    ap.add_argument("--chroma-weight", type=float, default=0.0,
                    help="low-pass Cb/Cr term weight (off by default)")
    ap.add_argument("--sat-weight", type=float, default=4.0,
                    help="hue-blind colour strength term (v2b/v2c), against the pull toward grey")
    ap.add_argument("--colour-start", type=float, default=0.4,
                    help="share of the training before the colour terms start (pixel loss alone first)")
    ap.add_argument("--no-mask", action="store_true", help="train without the damage mask planes")
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--cn", default="2,4,6,8,10,12,14,16,20", help="simulated C/N values (dB)")
    ap.add_argument("--crops", type=int, default=64, help="crops per noisy run")
    ap.add_argument("--size", type=int, default=96, help="crop size (px)")
    ap.add_argument("--bench-runs", default="~/openvrx-data/bench-runs/ml-ladder-164756,~/openvrx-data/bench-runs/ml-ladder-164756-b")
    ap.add_argument("--render", type=lst, default=["~/openvrx-data/flights/20261107/openvrx-20261107-022457",
                                                   "~/openvrx-data/flights/20261107/openvrx-20261107-022948"],
                    help="whole recordings to render after training")
    ap.add_argument("--panels", default="raw,model", help="render panels, from raw, linear, model "
                    "(linear is slow in Python: about doubles the render time)")
    ap.add_argument("--compare", type=lst, default=None,
                    help="older models for eval and the gallery (default: DATA/cnn-v1.pt and the best cnn-v2*.pt there)")
    ap.add_argument("--gallery", default="~/openvrx-data/media")
    ap.add_argument("--steps", type=lst, default=STEPS, help="only these steps (default: all, in order)")
    ap.add_argument("--smoke", action="store_true", help="tiny version of every step, to test the pipeline")
    ap.add_argument("--extra-train", type=lst, default=[],
                    help="more scenes for training (folders in DATA; one with a clip DATA/clips/NAME.mkv "
                         "is simulated first, like the rooms)")
    ap.add_argument("--extra-val", type=lst, default=[], help="more held-out scenes (also scored in eval)")
    ap.add_argument("--kaggle", metavar="USER", help="train on a Kaggle GPU as USER (kaggle_train.py)")
    ap.add_argument("--iters", type=int, help="training iterations (default: from --hours and a CPU speed "
                    "test; with --kaggle: 60000)")
    ap.add_argument("--no-amp", action="store_true", help="with --kaggle: train in float32 only")
    a = ap.parse_args()

    if (platform.system() == "Linux" and not os.environ.get("OPENVRX_INHIBITED")
            and shutil.which("systemd-inhibit")):
        os.environ["OPENVRX_INHIBITED"] = "1"
        os.execvp("systemd-inhibit", ["systemd-inhibit", "--what=sleep:idle", "--who=openvrx",
                                      f"--why=openvrx denoise overnight {a.name}", PY] + sys.argv)

    data = os.path.expanduser(a.data)
    name = a.name + ("-smoke" if a.smoke else "")
    run = Run(os.path.join(data, f"overnight-{name}"))
    model = os.path.join(data, f"cnn-{name}.pt")
    mask = not a.no_mask
    compare = a.compare
    if compare is None:
        compare = [p for p in [os.path.join(data, "cnn-v1.pt"), os.path.join(data, "cnn-v2.pt")] if os.path.exists(p)]
    compare = [os.path.expanduser(p) for p in compare]
    run.log(f"overnight {name}: {vars(a)}")
    cn = "2,6,12" if a.smoke else a.cn
    rooms = ["room0", "room5"] if a.smoke else ROOMS_TRAIN + ROOMS_VAL
    rooms += [s for s in a.extra_train + a.extra_val if os.path.exists(os.path.join(data, "clips", s + ".mkv"))]

    def simulate():
        for r in rooms:
            run.sh([PY, os.path.join(HERE, "denoise_filter.py"), "simulate", "--clip", os.path.join(data, "clips", r + ".mkv"),
                    "--scene", r, "--data", data, "--cn", cn, "--damage", "--keep-ref", "--jobs", "4",
                    "--bin", BIN])

    def bench():
        runs = ",".join(os.path.expanduser(x) for x in a.bench_runs.split(","))
        run.sh([PY, os.path.join(HERE, "bench_dataset.py"), "build", "--run", runs, "--data", data])

    crops = 4 if a.smoke else a.crops
    tr = ["room0"] + BENCH_TRAIN[:1] if a.smoke else ROOMS_TRAIN + BENCH_TRAIN + a.extra_train
    va = ["room5"] if a.smoke else ROOMS_VAL + BENCH_VAL + a.extra_val

    def prepare():
        for out, scenes in (("train.npz", tr), ("val.npz", va)):
            run.sh([PY, os.path.join(HERE, "denoise_cnn.py"), "prepare", "--data", data, "--scenes", ",".join(scenes),
                    "--out", os.path.join(run.out, out), "--crops", str(crops), "--size", str(a.size), "--seed", "1"]
                   + (["--mask", "--skip-unmasked"] if mask and a.smoke else ["--mask"] if mask else []))

    def train_args(train, val, iters):
        return (["--train", train, "--val", val, "--iters", str(iters),
                 "--batch", str(a.batch), "--width", str(a.width), "--depth", str(a.depth),
                 "--chroma-weight", str(a.chroma_weight), "--sat-weight", str(a.sat_weight),
                 "--colour-start", str(a.colour_start), "--threads", str(a.threads), "--resume",
                 "--log-every", "20" if a.smoke else "250", "--checkpoint-every", "20" if a.smoke else "500"]
                + (["--mask"] if mask else []))

    def train_kaggle():
        if "iters" not in run.state:
            run.state["iters"] = 300 if a.smoke else (a.iters or 60000)
            run.save()
        iters = run.state["iters"]
        kt = os.path.join(HERE, "kaggle_train.py")
        dataset, kernel = f"{a.kaggle}/openvrx-denoise-{name}", f"{a.kaggle}/openvrx-train-{name}"
        up, got = os.path.join(run.out, "kaggle-upload"), os.path.join(run.out, "kaggle-out")
        os.makedirs(up, exist_ok=True)
        for f in ("train.npz", "val.npz"):  # hard links: no second copy on disk
            dst = os.path.join(up, f)
            if os.path.exists(dst):
                os.remove(dst)
            os.link(os.path.join(run.out, f), dst)
        ckpt = model + ".ckpt"
        if os.path.exists(ckpt):  # an unfinished job's checkpoint: continue from it
            shutil.copy(ckpt, os.path.join(up, os.path.basename(ckpt)))
            run.log(f"continuing from {ckpt}")
        run.sh([PY, kt, "upload", "--dataset", dataset, "--dir", up, "--message", f"crops for {name}"])
        run.sh([PY, kt, "push", "--kernel", kernel, "--dataset", dataset, "--out", os.path.basename(model), "--"]
               + train_args("{data}/train.npz", "{data}/val.npz", iters) + ([] if a.no_amp else ["--amp"]))
        run.log(f"training on Kaggle: {kernel}, {iters} iterations; waiting (status every minute)")
        shutil.rmtree(got, ignore_errors=True)
        p = subprocess.run([PY, kt, "fetch", "--kernel", kernel, "--to", got, "--wait"], capture_output=True, text=True)
        run.log((p.stdout + p.stderr).strip()[-1500:])
        if os.path.exists(os.path.join(got, "train.log")):
            with open(os.path.join(got, "train.log")) as f:
                run.logf.write(f.read())
        if os.path.exists(os.path.join(got, os.path.basename(ckpt))):
            shutil.copy(os.path.join(got, os.path.basename(ckpt)), ckpt)  # for a rerun to continue
        if p.returncode != 0 or not os.path.exists(os.path.join(got, os.path.basename(model))):
            run.log("FAILED: the Kaggle job did not finish (log above); rerun the same command to continue")
            sys.exit(1)
        shutil.copy(os.path.join(got, os.path.basename(model)), model)
        run.log(f"model -> {model}")

    def train():
        if a.kaggle:
            return train_kaggle()
        if "iters" not in run.state:
            s = speed_test(a.width, a.depth, mask, a.size, a.batch, a.threads)
            # Validation passes cost about 10 % on top of the iterations.
            iters = 60 if a.smoke else (a.iters or max(1000, int(a.hours * 3600 / (s * 1.1) / 500) * 500))
            run.state["iters"] = iters
            run.save()
            run.log(f"speed test: {s:.3f} s/iteration -> {iters} iterations for {a.hours} h")
        iters = run.state["iters"]
        run.sh([PY, os.path.join(HERE, "denoise_cnn.py"), "train", "--out", model]
               + train_args(os.path.join(run.out, "train.npz"), os.path.join(run.out, "val.npz"), iters))

    def evaluate():
        for scene in (["room5"] if a.smoke else ["room5"] + BENCH_VAL + a.extra_val):
            run.sh([PY, os.path.join(HERE, "denoise_cnn.py"), "eval", "--data", data, "--scene", scene,
                    "--model", ",".join(compare + [model]), "--filter", LINEAR, "--every", "24" if a.smoke else "8",
                    "--threads", str(a.threads), "--out", os.path.join(run.out, f"eval-{scene}.json")]
                   + (["--runs", "cn2,cn6,cn12"] if a.smoke else []))

    def gallery():
        # The smoke test keeps its videos out of the real gallery.
        g = os.path.join(run.out, "gallery") if a.smoke else os.path.expanduser(a.gallery)
        models = ",".join(compare + [model])
        run.sh([PY, os.path.join(REPO, "tools", "bench", "media.py"), "denoise", "--gallery", g, "--data", data,
                "--scene", "room5", "--runs", "cn2,cn12" if a.smoke else "cn4,cn8,cn12", "--model", models,
                "--filter", LINEAR, "--frames", "20" if a.smoke else "90"])
        cuts = [os.path.expanduser(c) for c in FLIGHT_CUTS]
        cuts = [c for c in cuts if os.path.exists(c + ".rx0.bin")]
        run.sh([PY, os.path.join(REPO, "tools", "bench", "media.py"), "flight", "--gallery", g, ",".join(cuts),
                "--model", models, "--filter", LINEAR, "--fields", "20" if a.smoke else "180"])

    def render():
        dst = os.path.join(data, "renders", name)
        os.makedirs(dst, exist_ok=True)
        for rec in a.render:
            rec = os.path.expanduser(rec)
            out = os.path.join(dst, f"{os.path.basename(rec)}-{a.panels.replace(',', '-')}.mp4")
            run.sh([PY, os.path.join(HERE, "denoise_cnn.py"), "render", "--rec", rec, "--model", model, "--out", out,
                    "--panels", a.panels, "--filter", LINEAR, "--threads", str(a.threads)]
                   + (["--seconds", "1"] if a.smoke else []))
            run.state.setdefault("videos", [])
            if out not in run.state["videos"]:
                run.state["videos"].append(out)
            run.save()

    fns = {"simulate": simulate, "bench": bench, "prepare": prepare, "train": train, "eval": evaluate,
           "gallery": gallery, "render": render}
    for s in STEPS:
        if s in a.steps:
            run.step(s, fns[s])
    summary(run, name, model)


def summary(run, name, model):
    lines = [f"# Overnight denoise run {name}", "", f"Model: {model}", ""]
    for f in sorted(os.listdir(run.out)):
        if f.startswith("eval-") and f.endswith(".json"):
            e = json.load(open(os.path.join(run.out, f)))
            res = e["psnr_db"]
            kinds = [k for k in next(iter(res.values())) if isinstance(next(iter(res.values()))[k], dict)]
            lines += [f"## {e['scene']}: RGB PSNR dB (sat = colour vs reference, 1.00 = same)", "",
                      "| run | " + " | ".join(kinds) + " |", "|---" * (len(kinds) + 1) + "|"]
            for r in sorted(res, key=lambda n: float("".join(c for c in n.split("-")[0] if c.isdigit() or c == ".") or 0)):
                lines.append(f"| {r} | " + " | ".join(f"{res[r][k]['rgb']:.2f} ({res[r][k]['sat_ratio']:.2f})"
                                                      for k in kinds) + " |")
            lines.append("")
    vids = run.state.get("videos", [])
    if vids:
        lines += ["## Videos", ""] + [f"- {v}" for v in vids] + [""]
    lines += [f"Steps done: {', '.join(run.state['done'])}", ""]
    with open(os.path.join(run.out, "SUMMARY.md"), "w") as f:
        f.write("\n".join(lines))
    run.log(f"summary -> {os.path.join(run.out, 'SUMMARY.md')}")


if __name__ == "__main__":
    main()
