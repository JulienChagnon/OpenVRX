#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Contact sheets of bench runs and denoise results, opened in a viewer.

Never runs during a recording: it decodes and draws after the fact, so it
cannot cost a sweep any USB time. Sheets are PNG files written next to the
data (RUN/show/, DATA/SCENE/show/), so any agent or person can open them
later; --no-open only writes them.

    show.py run RUN_DIR[,RUN_DIR2] [--field 20] [--no-open]
        one sheet per pattern (or one for the run): a decoded field of every
        step, labelled with attenuation, C/N, PSNR and picture ok/lost, and
        the MS2107 truth (first segment) as the first tile when there is one
    show.py denoise --data DIR --scene NAME [--runs cn10,cn14] [--model A.pt[,B.pt]]
                    [--filter LIN.json] [--frame 40] [--no-open]
        before/after: reference | raw | linear filter | CNN, one row per run,
        with each panel's PSNR against the reference
    show.py open FILE...
        opens files (PNG sheets, truth .mkv) in mpv, detached

Viewer: mpv (on Atlas the only one; xdg-open sends PNG to rnote). In mpv:
left/right arrows step through the sheets, q closes. Needs numpy, Pillow,
and openvrx-cli for `run`; torch only for --model.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLI = os.path.join(ROOT, "host", "target", "main", "release", "openvrx-cli")
sys.path.insert(0, os.path.join(ROOT, "tools", "denoise"))
from denoise_filter import frame_key  # noqa: E402  (numeric field order, see there)
TILE_W = 360  # tiles are decoded fields scaled to this width
FONTS = ["/usr/share/fonts/google-noto-vf/NotoSansMono[wght].ttf",
         "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf"]


def font(size):
    for f in FONTS:
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def tile(img, label, w=TILE_W, sub=None):
    """img scaled to width w with a label bar on top (and a second line)."""
    img = img.convert("RGB")
    h = int(img.height * w / img.width)
    img = img.resize((w, h), Image.BILINEAR)
    bar = 40 if sub else 22
    out = Image.new("RGB", (w, h + bar), (24, 24, 24))
    out.paste(img, (0, bar))
    d = ImageDraw.Draw(out)
    d.text((6, 3), label, fill=(240, 240, 240), font=font(14))
    if sub:
        d.text((6, 21), sub, fill=(170, 200, 255), font=font(13))
    return out


def grid(tiles, cols, title):
    w = max(t.width for t in tiles)
    h = max(t.height for t in tiles)
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * (w + 4) + 4, rows * (h + 4) + 34), (10, 10, 10))
    ImageDraw.Draw(sheet).text((8, 6), title, fill=(255, 220, 120), font=font(18))
    for i, t in enumerate(tiles):
        sheet.paste(t, (4 + (i % cols) * (w + 4), 34 + (i // cols) * (h + 4)))
    return sheet


def open_files(paths):
    paths = [p for p in paths if p]
    if not paths:
        return
    if not shutil.which("mpv"):
        print("no mpv: open these by hand:\n  " + "\n  ".join(paths))
        return
    # Detached (own session), so closing the terminal or this script leaves
    # the window alone and the window never blocks the caller.
    subprocess.Popen(["mpv", "--force-window=yes", "--keep-open=yes", "--image-display-duration=inf",
                      "--title=openvrx: ${filename}", "--really-quiet"] + paths,
                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    print(f"opened {len(paths)} file(s) in mpv (arrows: next/previous, q: close)")


def decode_field(rec, work, field):
    """One decoded field of a recording (the decoder needs a few fields to lock)."""
    out = os.path.join(work, os.path.basename(rec))
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    subprocess.run([CLI, "decode", rec, out, str(field + 1)], capture_output=True)
    pngs = sorted((f for f in os.listdir(out) if f.endswith(".png")), key=frame_key)
    return Image.open(os.path.join(out, pngs[-1])) if pngs else None


def truth_frame(run_dir):
    """A frame from the first non-empty MS2107 truth segment, or None."""
    for f in sorted(os.listdir(run_dir)):
        if f.startswith("truth") and f.endswith(".mkv"):
            png = os.path.join(run_dir, "show", "_truth.png")
            p = subprocess.run(["ffmpeg", "-v", "error", "-ss", "1", "-i", os.path.join(run_dir, f),
                                "-frames:v", "1", "-y", png], capture_output=True)
            if p.returncode == 0 and os.path.exists(png):
                return Image.open(png)
    return None


def cmd_run(a):
    rows = []
    for d in a.run_dirs:
        for l in open(os.path.join(d, "results.jsonl")):
            if l.strip():
                r = json.loads(l)
                r["_dir"] = d
                rows.append(r)
    out = os.path.join(a.run_dirs[-1], "show")
    work = os.path.join(out, "_decode")
    os.makedirs(work, exist_ok=True)
    truth = None
    for d in a.run_dirs:
        truth = truth or truth_frame(d)
    sheets = []
    for pat in sorted({r.get("pattern") or "" for r in rows}):
        mine = sorted((r for r in rows if (r.get("pattern") or "") == pat), key=lambda r: r["att_db"])
        tiles = [tile(truth, "MS2107 truth (VTX input)")] if truth is not None else []
        for r in mine:
            img = decode_field(os.path.join(r["_dir"], r["rec"]), work, a.field)
            if img is None:
                img = Image.new("RGB", (720, 480), (0, 0, 0))
            rx = f"{r['rx_dbm']:.0f} dBm" if r.get("rx_dbm") is not None else "pit"
            psnr = f"{r['psnr_median']:.1f}" if r.get("psnr_median") is not None else "-"
            sub = (f"C/N {r['cn_db']:.1f}  PSNR {psnr}  {r['picture']}"
                   + (f"  hostclip {r['clip_pct']:.0f}%" if r.get("clip_pct", 0) > 1 else ""))
            tiles.append(tile(img, f"att {r['att_db']:g} dB  {rx}  {r['lock']}", sub=sub))
        name = os.path.basename(a.run_dirs[0].rstrip("/"))
        sheet = grid(tiles, a.cols, f"{name}  {pat or 'run'}  field {a.field}  ({len(mine)} steps)")
        path = os.path.join(out, f"sheet-{pat or 'run'}.png")
        sheet.save(path)
        sheets.append(path)
        print(f"wrote {path}")
    shutil.rmtree(work, ignore_errors=True)
    if not a.no_open:
        open_files(sheets)


def cmd_denoise(a):
    sys.path.insert(0, os.path.join(ROOT, "tools", "denoise"))
    import denoise_filter as lin
    sd = os.path.join(a.data, a.scene)
    ref_dir, runs = lin.pairs(sd)
    if a.runs:
        # Exact names, or a bench run's C/N part (cn15.2 for cn15.2-att06.00).
        runs = [r for r in runs if any(os.path.basename(r) == x or os.path.basename(r).startswith(x + "-")
                                       for x in a.runs)]
    ref_paths = lin.frames(ref_dir)
    ref = [lin.to_ycc(p) for p in ref_paths]
    models, coef = {}, None
    if a.model:
        import denoise_cnn as cnn
        models = {cnn.model_name(p): cnn.load_model(p) for p in a.model}
    if a.filter:
        coef = lin.load_filter(a.filter, symmetric=True)
    tiles = []
    for run in runs:
        paths = lin.frames(run)
        seq = [lin.to_ycc(p) for p in paths]
        off = lin.align(ref, seq)
        k = min(max(a.frame, 1), len(ref) - 1)
        while not (1 <= k + off < len(seq) - 1) and k > 1:
            k -= 1
        j = k + off
        target = ref[k]
        panels = [("reference", target), ("raw decode", seq[j])]
        if coef is not None:
            panels.append(("linear filter", lin.filter_seq(seq[j - 1:j + 2], coef)[1]))
        if models:
            rgb = [np.asarray(Image.open(p).convert("RGB")) for p in paths[j - 1:j + 2]]
            mdir = cnn.mask_dir_of(run)
            masks = ([cnn.load_mask(os.path.join(mdir, os.path.basename(p))) for p in paths[j - 1:j + 2]]
                     if any(m.mask for m in models.values()) else None)
            for mname, m in models.items():
                panels.append((mname, cnn.run_model(m, rgb, 1, masks if m.mask else None)))
        rs = cnn.saturation([target])[0] if models else None
        for name, y in panels:
            sub = "" if name == "reference" else f"PSNR {lin.psnr(lin.to_rgb(target).astype(np.float32), lin.to_rgb(y).astype(np.float32)):.2f} dB RGB"
            if sub and rs:
                sub += f"  sat {cnn.saturation([y])[0] / rs:.2f}"
            tiles.append(tile(Image.fromarray(lin.to_rgb(y)), f"{os.path.basename(run)}  {name}", sub=sub or "target"))
    cols = 2 + (coef is not None) + len(models)
    out = os.path.join(sd, "show")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, f"before-after-f{a.frame}.png")
    grid(tiles, cols, f"{a.scene}: before / after, frame {a.frame}").save(path)
    print(f"wrote {path}")
    if not a.no_open:
        open_files([path])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("run_dirs", type=lambda v: [os.path.expanduser(x) for x in v.split(",") if x])
    r.add_argument("--field", type=int, default=20, help="decoded field shown per recording")
    r.add_argument("--cols", type=int, default=5)
    r.add_argument("--no-open", action="store_true")
    d = sub.add_parser("denoise")
    d.add_argument("--data", required=True)
    d.add_argument("--scene", required=True)
    d.add_argument("--runs", type=lambda v: [x for x in v.split(",") if x], default=None,
                   help="run name prefixes (default: all)")
    d.add_argument("--model", type=lambda v: [x for x in v.split(",") if x], default=None,
                   help="one or more models, comma separated (a panel each)")
    d.add_argument("--filter", default=None)
    d.add_argument("--frame", type=int, default=40)
    d.add_argument("--no-open", action="store_true")
    o = sub.add_parser("open")
    o.add_argument("files", nargs="+")
    a = ap.parse_args()
    if a.cmd == "run":
        cmd_run(a)
    elif a.cmd == "denoise":
        cmd_denoise(a)
    else:
        open_files([os.path.abspath(f) for f in a.files])


if __name__ == "__main__":
    main()
