#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Turns video clips into looping bench sources with a slate.

Each clip is cut to a short stretch, cropped to 4:3 (analog FPV), scaled to
720x480 and set to 59.94 frames/s (one picture per NTSC field), and gets a
slate in front: black, white, black (--slate seconds each). Looped as the
transmitted picture, every attenuator step's recording then holds whole
passes of the clip, and the slate marks where each pass starts
(`bench_dataset.py video` finds it even at weak signal from the mean
brightness alone).

    prep_clips.py --in CLIP.mp4[,CLIP2.mkv] | --in DIR --out DIR
                  [--seconds 12] [--start 0] [--slate 0.25] [--license TEXT]

Writes OUT/NAME.mkv (FFV1, lossless: no new compression artefacts) and
OUT/clips.json (source, start, length, frames, slate frames, licence note
per clip). --start skips into each source (intros); --license records where
the footage comes from and its terms (the clips and the recordings stay
local; only models trained on them could ever be shared). Play one with:

    mpv --fs --loop-file=inf --no-audio --hwdec=no OUT/NAME.mkv

Play it at 59.94 Hz so every picture is shown once (the builder leaves out
steps with dropped or repeated pictures). Needs ffmpeg.
"""
import argparse
import json
import os
import subprocess
import sys

FPS = "60000/1001"
EXTS = (".mp4", ".mkv", ".mov", ".webm", ".avi", ".m4v")


def probe_seconds(path):
    p = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                       capture_output=True, text=True)
    try:
        return float(p.stdout.strip())
    except ValueError:
        return None


def prep(src, out, seconds, start, slate, w, h):
    """One loop file: slate (black, white, black) then the clip."""
    # Crop the middle to 4:3 (wider sources lose the sides, taller lose top
    # and bottom), then scale; lanczos keeps the detail the chain can carry.
    picture = (f"crop='min(iw,ih*4/3)':'min(ih,iw*3/4)',scale={w}:{h}:flags=lanczos,"
               f"fps={FPS},setsar=1,format=yuv420p")
    color = lambda c: f"color=c={c}:s={w}x{h}:r={FPS}:d={slate},format=yuv420p"  # noqa: E731
    graph = (f"[0:v]{picture}[clip];{color('black')}[s0];{color('white')}[s1];{color('black')}[s2];"
             f"[s0][s1][s2][clip]concat=n=4:v=1:a=0[v]")
    # An even number of pictures per loop: each pass then starts on the same
    # field parity, so every pass lines up with the reference.
    total = 3 * round(slate * 60000 / 1001) + round(seconds * 60000 / 1001)
    total -= total % 2
    cmd = ["ffmpeg", "-y", "-v", "error", "-ss", str(start), "-t", str(seconds + 1), "-i", src,
           "-filter_complex", graph, "-map", "[v]", "-an", "-r", FPS, "-frames:v", str(total),
           "-c:v", "ffv1", "-level", "3", out]
    subprocess.run(cmd, check=True)


def frames_of(path):
    p = subprocess.run(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0", "-show_entries",
                        "stream=nb_read_frames", "-of", "csv=p=0", path], capture_output=True, text=True)
    return int(p.stdout.strip() or 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="inputs", required=True,
                    help="video files (comma separated) or a folder of them")
    ap.add_argument("--out", required=True, help="folder for the loops and clips.json")
    ap.add_argument("--seconds", type=float, default=12.0, help="clip length after the slate")
    ap.add_argument("--start", type=float, default=0.0, help="seconds skipped at the start of each source")
    ap.add_argument("--slate", type=float, default=0.25, help="length of each slate part (black, white, black)")
    ap.add_argument("--size", default="720x480", help="output size (NTSC: 720x480)")
    ap.add_argument("--license", default="", help="where the footage is from and its terms (kept in clips.json)")
    a = ap.parse_args()

    if os.path.isdir(a.inputs):
        srcs = sorted(os.path.join(a.inputs, f) for f in os.listdir(a.inputs) if f.lower().endswith(EXTS))
    else:
        srcs = [os.path.expanduser(s) for s in a.inputs.split(",") if s]
    if not srcs:
        sys.exit("no video files found")
    w, h = (int(x) for x in a.size.split("x"))
    os.makedirs(a.out, exist_ok=True)
    manifest_path = os.path.join(a.out, "clips.json")
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    for src in srcs:
        name = os.path.splitext(os.path.basename(src))[0].replace(" ", "_")
        total = probe_seconds(src)
        start = a.start
        if total is not None and start + a.seconds > total:
            start = max(0.0, total - a.seconds)  # short source: take its end
        dst = os.path.join(a.out, name + ".mkv")
        prep(src, dst, a.seconds, start, a.slate, w, h)
        n = frames_of(dst)
        slate_frames = 3 * round(a.slate * 60000 / 1001)
        manifest[name] = {"source": os.path.abspath(src), "start_s": start, "seconds": a.seconds,
                          "frames": n, "slate_frames": slate_frames, "size": a.size, "fps": FPS,
                          "license": a.license}
        print(f"{name}: {n} frames ({slate_frames} slate) -> {dst}")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"-> {manifest_path}\nplay: mpv --fs --loop-file=inf --no-audio --hwdec=no {os.path.join(a.out, 'NAME.mkv')}")


if __name__ == "__main__":
    main()
