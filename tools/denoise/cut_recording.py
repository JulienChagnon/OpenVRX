#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Cuts a time window out of an OpenVRX recording into a new recording.

`openvrx-cli decode` reads from the start of a recording; to decode, score
or denoise a stretch in the middle of a flight (a fade at 13 s, say), cut it
out first. The raw file has no header: words of `word_bytes` bytes at
`sample_rate_hz`, so the window is a byte range rounded to whole words.
The sidecar is copied with the new file name and byte count, the start time
moved, and a note of where the cut came from; events (usb_gap) inside the
window are kept with their byte offsets shifted, the others dropped.
Stream time ignores usb_gap losses before the cut (a few 32 KB blocks at
most), which is fine for picking a stretch.

    cut_recording.py --rec BASE --start S --seconds N --out NEWBASE

BASE and NEWBASE are recording base paths (without .openvrx.json).
"""
import argparse
import json
import os


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rec", required=True)
    ap.add_argument("--start", type=float, required=True, help="seconds from the start")
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    side = json.load(open(a.rec + ".openvrx.json"))
    if len(side["receivers"]) != 1:
        raise SystemExit("one receiver only")
    rx = side["receivers"][0]
    wb = rx["sample_format"]["lanes"]["word_bytes"]
    rate = rx["sample_rate_hz"]
    b0 = int(a.start * rate) * wb
    b1 = min(int((a.start + a.seconds) * rate) * wb, rx["bytes"])
    if b0 >= b1:
        raise SystemExit(f"window outside the recording ({rx['bytes'] / wb / rate:.1f} s long)")
    src = os.path.join(os.path.dirname(a.rec + ".x"), rx["file"])
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    name = os.path.basename(a.out) + ".rx0.bin"
    with open(src, "rb") as fi, open(a.out + ".rx0.bin", "wb") as fo:
        fi.seek(b0)
        left = b1 - b0
        while left:
            chunk = fi.read(min(left, 8 << 20))
            if not chunk:
                break
            fo.write(chunk)
            left -= len(chunk)
    rx = dict(rx, file=name, bytes=b1 - b0)
    rx["start_host_ns"] = rx.get("start_host_ns", 0) + int(b0 / wb / rate * 1e9)
    side = dict(side, receivers=[rx], notes=(side.get("notes", "") + f" cut {a.start:g}+{a.seconds:g} s from "
                                             f"{os.path.basename(a.rec)}").strip())
    with open(a.out + ".openvrx.json", "w") as f:
        json.dump(side, f, indent=2)
    kept = 0
    with open(a.out + ".events.jsonl", "w") as fo:
        ev = a.rec + ".events.jsonl"
        if os.path.exists(ev):
            for l in open(ev):
                if not l.strip():
                    continue
                e = json.loads(l)
                if "byte" in e and b0 <= e["byte"] < b1:
                    e["byte"] -= b0
                    fo.write(json.dumps(e) + "\n")
                    kept += 1
    print(f"{a.out}: {(b1 - b0) / wb / rate:.2f} s from {a.start:g} s, {kept} events")


if __name__ == "__main__":
    main()
