#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Runs a denoise_cnn.py training on a Kaggle GPU, from this machine.

The crops (prepare's .npz files) go up once as a private Kaggle dataset;
each training is a private Kaggle script job ("kernel") that gets the
current denoise_cnn.py and denoise_filter.py embedded (a script job is one
file), trains on a T4, and leaves MODEL.pt, its .ckpt and train.log in the
job's output, which `fetch` downloads.

    kaggle_train.py upload --dataset USER/SLUG --dir DIR
    kaggle_train.py push   --kernel USER/SLUG --dataset USER/SLUG --out MODEL.pt \\
                           [--accelerator NvidiaTeslaT4] [--script enhance_cnn.py] -- TRAIN ARGS...
    kaggle_train.py status --kernel USER/SLUG
    kaggle_train.py fetch  --kernel USER/SLUG --to DIR [--wait]

upload: DIR holds the .npz files (dataset-metadata.json is written if
missing); a dataset that exists gets a new version. A MODEL.pt.ckpt in DIR
goes up too and the job copies it next to its output, so `-- ... --resume`
continues a training that a job did not finish. push: TRAIN ARGS are
train arguments of --script (denoise_cnn.py, the default, or
enhance_cnn.py) except --out; {data} in them is replaced by
the dataset's folder on Kaggle (for example --train {data}/train.npz). The
job stops at once if Kaggle gave it no GPU (an account without phone
verification, or a missing --accelerator: with the Kaggle CLI 2.x,
enable_gpu alone gives a CPU session). fetch --wait polls the status every
minute until the job ends.

Free tier limits (Nov 2026): 30 GPU hours a week, 12 h per job, 20 GB of
output. Needs the kaggle CLI (pip install --user kaggle) and a token in
~/.kaggle/access_token.
"""
import argparse
import base64
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCES = ("denoise_cnn.py", "denoise_filter.py", "enhance_cnn.py")
# Any *_cnn.py model script beside this file can be trained; the chosen one is embedded with SOURCES.
SCRIPTS = tuple(sorted(n for n in os.listdir(HERE) if n.endswith("_cnn.py")))

RUN = r'''
import base64, glob, os, subprocess, sys, time
SRC = __SRC__
ARGS = __ARGS__
OUT = __OUT__
SCRIPT = __SCRIPT__
os.makedirs("/kaggle/working/src", exist_ok=True)
for name, b in SRC.items():
    open(f"/kaggle/working/src/{name}", "wb").write(base64.b64decode(b))
import torch
if not torch.cuda.is_available():
    sys.exit("no GPU in this session: push with --accelerator NvidiaTeslaT4 from a phone-verified account")
print("GPU:", torch.cuda.get_device_name(0), "| torch", torch.__version__, flush=True)
hits = glob.glob("/kaggle/input/**/*.npz", recursive=True)
if not hits:
    sys.exit("no .npz under /kaggle/input: is the dataset attached?")
data = os.path.dirname(hits[0])
args = [x.replace("{data}", data) for x in ARGS]
# A checkpoint uploaded with the crops (a job cut off at the 12 h limit, or
# stopped) is where --resume continues from.
import shutil
for c in glob.glob(f"/kaggle/input/**/{OUT}.ckpt", recursive=True):
    shutil.copy(c, f"/kaggle/working/{OUT}.ckpt")
    print("checkpoint from the dataset:", c, flush=True)
    break
cmd = [sys.executable, f"/kaggle/working/src/{SCRIPT}", "train", *args, "--out", f"/kaggle/working/{OUT}"]
print("$", " ".join(cmd), flush=True)
with open("/kaggle/working/train.log", "w") as log:
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in p.stdout:
        log.write(line)
        log.flush()
        print(line, end="", flush=True)
sys.exit(p.wait())
'''


def kaggle(*args, check=True):
    p = subprocess.run(["kaggle", *args], capture_output=True, text=True)
    if check and p.returncode != 0:
        sys.exit(f"kaggle {' '.join(args)}: {(p.stdout + p.stderr).strip()[-500:]}")
    return (p.stdout + p.stderr).strip()


def cmd_upload(a):
    meta = os.path.join(a.dir, "dataset-metadata.json")
    if not os.path.exists(meta):
        with open(meta, "w") as f:
            json.dump({"title": a.dataset.split("/")[1], "id": a.dataset, "licenses": [{"name": "other"}]}, f)
    exists = "ready" in kaggle("datasets", "status", a.dataset, check=False).lower()
    if exists:
        print(kaggle("datasets", "version", "-p", a.dir, "-m", a.message or "new crops"))
    else:
        print(kaggle("datasets", "create", "-p", a.dir))  # private unless --public is passed
    # Right after `datasets version`, `datasets status` still reports the previous version as ready,
    # so "ready" alone let a push start on the old files. Wait until the published file list holds
    # every local data file at its local size.
    want = {n: os.path.getsize(os.path.join(a.dir, n)) for n in os.listdir(a.dir)
            if n != "dataset-metadata.json" and os.path.isfile(os.path.join(a.dir, n))}
    for _ in range(90):
        if "ready" in kaggle("datasets", "status", a.dataset, check=False).lower():
            listing = kaggle("datasets", "files", a.dataset, check=False)
            have = {}
            for line in listing.splitlines():
                parts = line.split()
                if len(parts) >= 2 and parts[1].isdigit():
                    have[parts[0]] = int(parts[1])
            missing = [n for n, size in want.items() if have.get(n) != size]
            if not missing:
                print(f"{a.dataset} ready (private), all {len(want)} files published")
                return
        time.sleep(20)
    print(f"{a.dataset} still processing or files missing: check `kaggle datasets files {a.dataset}` before push")
    sys.exit(1)


def cmd_push(a):
    if any(x == "--out" for x in a.train):
        sys.exit("leave --out out of the train arguments: use push --out")
    src = {n: base64.b64encode(open(os.path.join(HERE, n), "rb").read()).decode()
           for n in sorted(set(SOURCES) | {a.script})}
    with tempfile.TemporaryDirectory() as d:
        code = (RUN.replace("__SRC__", repr(src)).replace("__ARGS__", repr(a.train)).replace("__OUT__", repr(a.out))
                .replace("__SCRIPT__", repr(a.script)))
        open(os.path.join(d, "run.py"), "w").write(code)
        meta = {"id": a.kernel, "title": a.kernel.split("/")[1], "code_file": "run.py", "language": "python",
                "kernel_type": "script", "is_private": True, "enable_gpu": True, "enable_internet": False,
                "machine_shape": a.accelerator, "dataset_sources": [a.dataset], "competition_sources": [],
                "kernel_sources": [], "model_sources": []}
        with open(os.path.join(d, "kernel-metadata.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(kaggle("kernels", "push", "-p", d, "--accelerator", a.accelerator))
    print(f"then: kaggle_train.py fetch --kernel {a.kernel} --to DIR --wait")


def status(kernel):
    s = kaggle("kernels", "status", kernel, check=False)
    for word in ("COMPLETE", "ERROR", "CANCEL", "RUNNING", "QUEUED"):
        if word in s.upper():
            return word.lower(), s
    return "unknown", s


def cmd_status(a):
    print(status(a.kernel)[1])


def cmd_fetch(a):
    while a.wait:
        st, _ = status(a.kernel)
        if st not in ("running", "queued"):
            break
        time.sleep(60)
    st, text = status(a.kernel)
    print(text)
    os.makedirs(a.to, exist_ok=True)
    print(kaggle("kernels", "output", a.kernel, "-p", a.to, check=False))
    if st != "complete":
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    u = sub.add_parser("upload")
    u.add_argument("--dataset", required=True, help="USER/SLUG")
    u.add_argument("--dir", required=True, help="folder with the .npz crops")
    u.add_argument("--message", help="version note when the dataset exists")
    p = sub.add_parser("push")
    p.add_argument("--kernel", required=True, help="USER/SLUG of the job")
    p.add_argument("--dataset", required=True, help="USER/SLUG of the crops")
    p.add_argument("--out", required=True, help="model file name in the job output")
    p.add_argument("--accelerator", default="NvidiaTeslaT4")
    p.add_argument("--script", default="denoise_cnn.py", choices=SCRIPTS, help="which model's train runs")
    p.add_argument("train", nargs=argparse.REMAINDER, help="-- then denoise_cnn.py train arguments")
    s = sub.add_parser("status")
    s.add_argument("--kernel", required=True)
    f = sub.add_parser("fetch")
    f.add_argument("--kernel", required=True)
    f.add_argument("--to", required=True, help="local folder for the output")
    f.add_argument("--wait", action="store_true", help="poll every minute until the job ends")
    a = ap.parse_args()
    if a.cmd == "push":
        a.train = a.train[1:] if a.train[:1] == ["--"] else a.train  # only the separator
    {"upload": cmd_upload, "push": cmd_push, "status": cmd_status, "fetch": cmd_fetch}[a.cmd](a)


if __name__ == "__main__":
    main()
