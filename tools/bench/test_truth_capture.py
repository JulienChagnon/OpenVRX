#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline test of bench.py TruthCapture: no capture card, no bench.

A fake ffmpeg (a Python script written into a temp folder) stands in for the
real one, and the "device" is a plain file the test deletes and recreates.
Each fake ffmpeg start takes the next behaviour from a list:

    ok     appends to its output file every 0.1 s until SIGINT, exits 0
    stall  writes a little, then stops writing but keeps running (the MS2107
           stall); exits 255 on SIGINT like ffmpeg
    hang   as stall, but ignores SIGINT (needs the SIGKILL fallback)
    exit   writes a little, deletes the device file and exits 251 (the card
           dropping off USB; ffmpeg ends on "No such device")
    slow   waits 0.8 s before its first write, then as ok (inside the start
           grace: must not count as a stall)

    test_truth_capture.py [--keep]

Prints one line per case and exits non-zero on a failure. --keep leaves the
temp folders (truth.jsonl, fake logs) for a look.
"""
import argparse
import json
import os
import shutil
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import TruthCapture  # noqa: E402

FAKE = r'''#!/usr/bin/env python3
import os, signal, sys, time
d = os.path.dirname(os.path.abspath(__file__))
plan = open(os.path.join(d, "plan")).read().split(",")
n = len(os.listdir(os.path.join(d, "starts")))
open(os.path.join(d, "starts", str(n)), "w").close()
mode = plan[min(n, len(plan) - 1)]
dev = sys.argv[sys.argv.index("-i") + 1]
out = open(sys.argv[-1], "ab")
if mode == "hang":
    signal.signal(signal.SIGINT, signal.SIG_IGN)
else:
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0 if mode in ("ok", "slow") else 255))
if mode == "slow":
    time.sleep(0.8)
for i in range(10**6):
    if mode in ("stall", "hang", "exit") and i >= 3:
        if mode == "exit":
            os.remove(dev)
            sys.exit(251)
        time.sleep(3600)
    out.write(b"x" * 1024)
    out.flush()
    time.sleep(0.1)
'''


def run_case(name, plan, seconds, check, keep, on_start=None):
    tmp = tempfile.mkdtemp(prefix=f"truthtest-{name}-")
    fake_dir = os.path.join(tmp, "fake")
    os.makedirs(os.path.join(fake_dir, "starts"))
    ff = os.path.join(fake_dir, "ffmpeg")
    with open(ff, "w") as f:
        f.write(FAKE)
    os.chmod(ff, 0o755)
    with open(os.path.join(fake_dir, "plan"), "w") as f:
        f.write(",".join(plan))
    dev = os.path.join(tmp, "video0")
    open(dev, "w").close()
    out = os.path.join(tmp, "run")
    os.makedirs(out)
    tc = TruthCapture(dev, out, stall_s=1.0, start_grace_s=1.0, ffmpeg=ff)
    if on_start:
        threading.Thread(target=on_start, args=(dev,), daemon=True).start()
    time.sleep(seconds)
    counts = tc.close()
    segs = [json.loads(l) for l in open(os.path.join(out, "truth.jsonl")) if l.strip()]
    err = check(counts, segs)
    print(f"{'FAIL' if err else 'ok  '} {name}: {counts}, ends {[s['end'] for s in segs]}"
          + (f"  <- {err}" if err else ""))
    if keep:
        print(f"     kept {tmp}")
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return not err


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()

    def back_after(s):
        def f(dev):
            # Recreate the device s seconds after the fake removed it.
            while os.path.exists(dev):
                time.sleep(0.05)
            time.sleep(s)
            open(dev, "w").close()
        return f

    cases = [
        ("steady", ["ok"], 3.0,
         lambda c, s: None if c == {"segments": 1, "ffmpeg_exits": 0, "stalls": 0} else "expected one clean segment"),
        ("slow start inside grace", ["slow"], 3.0,
         lambda c, s: None if c["stalls"] == 0 else "a slow first write counted as a stall"),
        ("stall, then ok", ["stall", "ok"], 5.0,
         lambda c, s: None if c["stalls"] == 1 and s[0]["end"] == "stall" and s[0]["exit"] == 255
         and s[-1]["end"] == "stop" else "expected one stall (exit 255 on SIGINT), then a running segment"),
        ("stall ignoring SIGINT", ["hang", "ok"], 11.0,
         lambda c, s: None if c["stalls"] == 1 and s[0]["exit"] == -9 else "expected SIGKILL (exit -9)"),
        ("device drops, comes back after 2 s", ["exit", "ok"], 5.0,
         lambda c, s: None if c["ffmpeg_exits"] == 1 and c["segments"] == 2 and s[1]["t_start"] - s[0]["t_end"] > 1.9
         else "expected a wait for the device, then a second segment"),
    ]
    ok = True
    for name, plan, secs, check in cases:
        ok &= run_case(name, plan, secs, check, a.keep,
                       on_start=back_after(2.0) if plan[0] == "exit" else None)
    print("all passed" if ok else "FAILURES")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
