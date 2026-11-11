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

--hd N (default 2) also writes OUT/NAME.hd.mkv: the same pictures (same
crop, same slate, same picture count, frame for frame) at N times the size,
as lossless RGB (FFV1 gbrp, BT.709 matrix as tagged on the sources), for the
models that restore detail lost in the analog chain. --hd 0 skips it.
--hd-only writes just the HD files for clips already in clips.json, with
their recorded start, length and slate (the loops that were played are left
untouched).
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


def graph_for(seconds, slate, w, h, rgb=False):
    """Filter graph and picture count of one loop: slate (black, white,
    black) then the clip. rgb=True: the same pictures as lossless RGB."""
    # Crop the middle to 4:3 (wider sources lose the sides, taller lose top
    # and bottom), then scale; lanczos keeps the detail the chain can carry.
    # The RGB version converts with the sources' BT.709 matrix (limited
    # range), as the loop is shown.
    fmt = "format=yuv420p" if not rgb else "scale=in_color_matrix=bt709:in_range=tv:out_range=full,format=gbrp"
    picture = (f"crop='min(iw,ih*4/3)':'min(ih,iw*3/4)',scale={w}:{h}:flags=lanczos,"
               f"fps={FPS},setsar=1,{fmt}")
    cfmt = "format=yuv420p" if not rgb else "format=gbrp"
    color = lambda c: f"color=c={c}:s={w}x{h}:r={FPS}:d={slate},{cfmt}"  # noqa: E731
    graph = (f"[0:v]{picture}[clip];{color('black')}[s0];{color('white')}[s1];{color('black')}[s2];"
             f"[s0][s1][s2][clip]concat=n=4:v=1:a=0[v]")
    # An even number of pictures per loop: each pass then starts on the same
    # field parity, so every pass lines up with the reference.
    total = 3 * round(slate * 60000 / 1001) + round(seconds * 60000 / 1001)
    total -= total % 2
    return graph, total


def prep(src, out, seconds, start, slate, w, h, rgb=False):
    """One loop file (rgb=True: its lossless RGB version at w x h)."""
    graph, total = graph_for(seconds, slate, w, h, rgb)
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
    ap.add_argument("--hd", type=int, default=2,
                    help="also write NAME.hd.mkv at this many times --size, lossless RGB (0: no)")
    ap.add_argument("--hd-only", action="store_true",
                    help="only the HD files, for clips already in OUT/clips.json (their start, length, slate)")
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
    if a.hd_only and a.hd < 1:
        sys.exit("--hd-only needs --hd 2 (or another factor)")
    for src in srcs:
        name = os.path.splitext(os.path.basename(src))[0].replace(" ", "_")
        if a.hd_only:
            if name not in manifest:
                print(f"{name}: not in {manifest_path}, skipped (make its loop first)")
                continue
            m = manifest[name]
            start, seconds = m["start_s"], m["seconds"]
            slate = m["slate_frames"] / 3 * 1001 / 60000
            w, h = (int(x) for x in m["size"].split("x"))
        else:
            total = probe_seconds(src)
            start, seconds, slate = a.start, a.seconds, a.slate
            if total is not None and start + seconds > total:
                start = max(0.0, total - seconds)  # short source: take its end
            dst = os.path.join(a.out, name + ".mkv")
            prep(src, dst, seconds, start, slate, w, h)
            n = frames_of(dst)
            slate_frames = 3 * round(slate * 60000 / 1001)
            manifest[name] = {"source": os.path.abspath(src), "start_s": start, "seconds": seconds,
                              "frames": n, "slate_frames": slate_frames, "size": a.size, "fps": FPS,
                              "license": a.license}
            print(f"{name}: {n} frames ({slate_frames} slate) -> {dst}")
        if a.hd > 0:
            hw, hh = w * a.hd, h * a.hd
            hdst = os.path.join(a.out, name + ".hd.mkv")
            prep(src, hdst, seconds, start, slate, hw, hh, rgb=True)
            hn = frames_of(hdst)
            if hn != manifest[name]["frames"]:
                sys.exit(f"{hdst}: {hn} pictures, the loop has {manifest[name]['frames']}: not frame for frame")
            manifest[name]["hd"] = {"file": os.path.basename(hdst), "size": f"{hw}x{hh}", "factor": a.hd,
                                    "pix_fmt": "gbrp", "matrix": "bt709"}
            print(f"{name}: HD {hw}x{hh}, {hn} frames -> {hdst}")
        with open(manifest_path, "w") as f:  # after every clip: a long batch keeps what it made
            json.dump(manifest, f, indent=2)
    print(f"-> {manifest_path}\nplay: mpv --fs --loop-file=inf --no-audio --hwdec=no {os.path.join(a.out, 'NAME.mkv')}")


if __name__ == "__main__":
    main()
