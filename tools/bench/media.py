#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Video gallery of bench runs and denoise results, with a menu.

Builds short videos after the fact (never during a sweep) and a local web
page (GALLERY/index.html) that plays them side by side, in sync, with a
menu to pick which groups and which panel kinds to show. Plain files: open
index.html in any browser later, no server.

    media.py denoise --gallery G --data DIR --scene NAME [--runs cn10,cn14]
                     [--model A.pt[,B.pt]] [--filter LIN.json] [--frames 120]
        one group per run: reference | raw | linear | one video per model
        (panel named after the model file, e.g. cnn-v1, cnn-v2)
    media.py run --gallery G RUN_DIR[,RUN_DIR2] [--fields 120] [--patterns card,grey]
                 [--steps 0,4,8,12]
        one group per pattern: the decode of every chosen step, plus the
        MS2107 truth (first segment) when there is one
    media.py flight --gallery G REC[,REC2] [--model A.pt[,B.pt]] [--filter LIN.json] [--fields 180]
        real recordings with no reference (fades): raw | linear | one video per
        model, one group per recording (cut stretches out with
        tools/denoise/cut_recording.py; whole flights: denoise_cnn.py render).
        Models that take damage masks get them from the decode.
    media.py page --gallery G [--open]
        (re)writes index.html from G/gallery.json and opens it in Firefox

Videos are VP9 WebM at CRF 12 (quality high enough that the codec does not
hide the noise being compared), decoded fields at 59.94 per second. Each
build adds or replaces its groups in gallery.json and rewrites the page.
Needs ffmpeg, numpy, Pillow, openvrx-cli; torch for --model.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLI = os.path.join(ROOT, "host", "target", "main", "release", "openvrx-cli")
sys.path.insert(0, os.path.join(ROOT, "tools", "denoise"))
from denoise_filter import frame_key  # noqa: E402  (numeric field order, see there)
FIELD_RATE = "60000/1001"
# Keyframe every 15 fields: a seek (field step, loop, resync) decodes at most
# 15 fields. With the encoder default (hundreds) every seek stalled the video,
# and the page's resync kept seeking it: the panel showed about 1 frame/s.
GOP = 15


def encode(png_dir, out, rate=FIELD_RATE):
    """PNG sequence (f0000.png ...) -> VP9 WebM."""
    os.makedirs(os.path.dirname(out), exist_ok=True)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", rate, "-i", os.path.join(png_dir, "f%04d.png"),
                    "-c:v", "libvpx-vp9", "-crf", "12", "-b:v", "0", "-deadline", "good", "-cpu-used", "4",
                    "-row-mt", "1", "-g", str(GOP), "-pix_fmt", "yuv420p", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", out],
                   check=True)


def load_gallery(g):
    p = os.path.join(g, "gallery.json")
    return json.load(open(p)) if os.path.exists(p) else {"groups": []}


def save_group(g, group):
    gal = load_gallery(g)
    gal["groups"] = [x for x in gal["groups"] if x["id"] != group["id"]] + [group]
    with open(os.path.join(g, "gallery.json"), "w") as f:
        json.dump(gal, f, indent=1)
    write_page(g)


def load_models(paths):
    """{name: model} for --model a.pt,b.pt (name = file stem, e.g. cnn-v2);
    each becomes its own panel kind."""
    if not paths:
        return {}
    import denoise_cnn as cnn
    import torch
    torch.set_num_threads(4)
    return {cnn.model_name(p): cnn.load_model(p) for p in paths}


def run_models(models, paths, j, cache, mask_dir=None):
    """Every model on field j of paths; cache holds the RGB fields (and masks)
    of the sliding 3-field window."""
    import denoise_cnn as cnn
    for q in (j - 1, j, j + 1):
        if q not in cache:
            img = np.asarray(Image.open(paths[q]).convert("RGB"))
            m = cnn.load_mask(os.path.join(mask_dir, os.path.basename(paths[q]))) if mask_dir else None
            cache[q] = (img, m)
    cache.pop(j - 2, None)
    rgb = [cache[q][0] for q in (j - 1, j, j + 1)]
    masks = [cache[q][1] for q in (j - 1, j, j + 1)]
    return {n: cnn.run_model(m, rgb, 1, masks if m.mask else None) for n, m in models.items()}


def cmd_denoise(a):
    import denoise_filter as lin
    sd = os.path.join(a.data, a.scene)
    ref_dir, runs = lin.pairs(sd)
    if a.runs:
        # Exact names, or a bench run's C/N part (cn15.2 for cn15.2-att06.00);
        # a bare prefix made cn2 pick cn20 too.
        runs = [r for r in runs if any(os.path.basename(r) == x or os.path.basename(r).startswith(x + "-")
                                       for x in a.runs)]
    ref_paths = lin.frames(ref_dir)
    ref = [lin.to_ycc(p) for p in ref_paths]
    models = load_models(a.model)
    coef = lin.load_filter(a.filter, symmetric=True) if a.filter else None
    for run in runs:
        name = os.path.basename(run)
        paths = lin.frames(run)
        if len(paths) < 3:
            print(f"{a.scene}/{name}: only {len(paths)} fields decoded, skipped")
            continue
        seq = [lin.to_ycc(p) for p in paths]
        off = lin.align(ref, seq)
        ks = [k for k in range(len(ref)) if 1 <= k + off < len(seq) - 1][:a.frames]
        tmp = tempfile.mkdtemp(prefix="media-")
        panels = ["reference", "raw"] + (["linear"] if coef is not None else []) + list(models)
        mask_dir = os.path.join(sd, "damage", name) if any(m.mask for m in models.values()) else None
        cache = {}
        for i, k in enumerate(ks):
            j = k + off
            frames = {"reference": ref[k], "raw": seq[j]}
            if coef is not None:
                frames["linear"] = lin.filter_seq(seq[j - 1:j + 2], coef)[1]
            frames.update(run_models(models, paths, j, cache, mask_dir))
            for kind, y in frames.items():
                d = os.path.join(tmp, kind)
                os.makedirs(d, exist_ok=True)
                Image.fromarray(lin.to_rgb(y)).save(os.path.join(d, f"f{i:04d}.png"))
            if i % 20 == 0:
                print(f"  {a.scene}/{name}: frame {i}/{len(ks)}", flush=True)
        items = []
        for kind in panels:
            rel = f"media/{a.scene}/{name}-{kind}.webm"
            encode(os.path.join(tmp, kind), os.path.join(a.gallery, rel))
            items.append({"kind": kind, "label": kind, "src": rel})
        shutil.rmtree(tmp, ignore_errors=True)
        save_group(a.gallery, {"id": f"denoise/{a.scene}/{name}", "title": f"{a.scene} {name}",
                               "section": "denoise: before / after", "items": items,
                               "info": f"{len(ks)} fields; models {', '.join(models) or '-'}"})
        print(f"{a.scene}/{name}: {len(items)} videos")


def cmd_flight(a):
    """Real recordings (fades, no clean reference): raw | linear | CNN."""
    import denoise_filter as lin
    models = load_models(a.model)
    coef = lin.load_filter(a.filter, symmetric=True) if a.filter else None
    for rec in a.recs:
        name = os.path.basename(rec)
        tmp = tempfile.mkdtemp(prefix="media-")
        dec = os.path.join(tmp, "dec")
        os.makedirs(dec)
        mask_dir = os.path.join(tmp, "damage") if any(m.mask for m in models.values()) else None
        subprocess.run([CLI, "decode", rec, dec, str(a.fields + 2)] + (["--damage", mask_dir] if mask_dir else []),
                       capture_output=True)
        paths = lin.frames(dec)
        if len(paths) < 3:
            print(f"{name}: only {len(paths)} fields decoded, skipped")
            shutil.rmtree(tmp, ignore_errors=True)
            continue
        kinds = ["raw"] + (["linear"] if coef is not None else []) + list(models)
        for kind in kinds:
            os.makedirs(os.path.join(tmp, kind))
        cache = {}
        for i, j in enumerate(range(1, len(paths) - 1)):
            y = lin.to_ycc(paths[j])
            out = {"raw": y}
            if coef is not None:
                out["linear"] = lin.filter_seq([lin.to_ycc(p) for p in paths[j - 1:j + 2]], coef)[1]
            out.update(run_models(models, paths, j, cache, mask_dir))
            for kind, v in out.items():
                Image.fromarray(lin.to_rgb(v)).save(os.path.join(tmp, kind, f"f{i:04d}.png"))
            if i % 30 == 0:
                print(f"  {name}: field {i}/{len(paths) - 2}", flush=True)
        items = []
        for kind in kinds:
            rel = f"media/flights/{name}-{kind}.webm"
            encode(os.path.join(tmp, kind), os.path.join(a.gallery, rel))
            items.append({"kind": kind, "label": kind, "src": rel})
        shutil.rmtree(tmp, ignore_errors=True)
        side = json.load(open(rec + ".openvrx.json"))
        save_group(a.gallery, {"id": f"flight/{name}", "title": name, "section": "flights: real fades (no reference)",
                               "items": items, "info": f"{len(paths) - 2} fields; {side.get('notes', '')}"})
        print(f"{name}: {len(items)} videos")


def cmd_run(a):
    rows = []
    for d in a.run_dirs:
        for l in open(os.path.join(d, "results.jsonl")):
            if l.strip():
                r = json.loads(l)
                r["_dir"] = d
                rows.append(r)
    if a.steps:
        rows = [r for r in rows if r["att_db"] in a.steps]
    run_name = os.path.basename(a.run_dirs[0].rstrip("/"))
    truth = None
    for d in a.run_dirs:
        segs = sorted(f for f in os.listdir(d) if f.startswith("truth") and f.endswith(".mkv"))
        if segs and truth is None:
            rel = f"media/{run_name}/truth.webm"
            dst = os.path.join(a.gallery, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", os.path.join(d, segs[0]), "-t", "4", "-c:v",
                            "libvpx-vp9", "-crf", "12", "-b:v", "0", "-deadline", "good", "-cpu-used", "4",
                            "-g", str(GOP), "-pix_fmt", "yuv420p", dst], check=True)
            truth = {"kind": "truth", "label": "MS2107 truth (VTX input)", "src": rel}
    pats = sorted({r.get("pattern") or "run" for r in rows})
    if a.patterns:
        pats = [p for p in pats if p in a.patterns]
    for pat in pats:
        mine = sorted((r for r in rows if (r.get("pattern") or "run") == pat), key=lambda r: r["att_db"])
        items = [truth] if truth else []
        for r in mine:
            tmp = tempfile.mkdtemp(prefix="media-")
            subprocess.run([CLI, "decode", os.path.join(r["_dir"], r["rec"]), tmp, str(a.fields)],
                           capture_output=True)
            pngs = sorted((f for f in os.listdir(tmp) if f.endswith(".png")), key=frame_key)
            seqd = os.path.join(tmp, "seq")
            os.makedirs(seqd)
            for i, p in enumerate(pngs):
                os.rename(os.path.join(tmp, p), os.path.join(seqd, f"f{i:04d}.png"))
            rel = f"media/{run_name}/{r['rec']}.webm"
            if pngs:
                encode(seqd, os.path.join(a.gallery, rel))
                rx = f"{r['rx_dbm']:.0f} dBm" if r.get("rx_dbm") is not None else "pit"
                items.append({"kind": "decode", "label": f"att {r['att_db']:g} dB, {rx}",
                              "info": f"C/N {r['cn_db']:.1f}, PSNR {r.get('psnr_median')}, {r['lock']}, "
                                      f"{r['picture']}, host clip {r['clip_pct']:.1f}%", "src": rel})
            shutil.rmtree(tmp, ignore_errors=True)
            print(f"  {run_name}/{pat}: att {r['att_db']:g} ({len(pngs)} fields)", flush=True)
        save_group(a.gallery, {"id": f"run/{run_name}/{pat}", "title": f"{run_name} {pat}",
                               "section": "bench runs: decoded steps", "items": items,
                               "info": f"{len(mine)} steps, {a.fields} fields each"})


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>OpenVRX media</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#111;--panel:#1b1b1b;--line:#2c2c2c;--text:#e6e6e6;--dim:#9a9a9a;--accent:#ffc861}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,sans-serif;display:flex;height:100vh}
aside{width:300px;min-width:300px;overflow:auto;border-right:1px solid var(--line);padding:12px;background:var(--panel)}
main{flex:1;display:flex;flex-direction:column;min-width:0}
h1{font-size:16px;margin:0 0 10px;color:var(--accent)}h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--dim);margin:16px 0 6px}
label{display:flex;gap:6px;align-items:flex-start;padding:2px 0;cursor:pointer}.sub{margin-left:20px}.info{color:var(--dim);font-size:12px}
.bar{display:flex;gap:8px;align-items:center;padding:8px 12px;border-bottom:1px solid var(--line);flex-wrap:wrap}
button,select{background:#262626;color:var(--text);border:1px solid #3a3a3a;border-radius:4px;padding:4px 10px;font:inherit;cursor:pointer}
#grid{flex:1;overflow:auto;padding:10px;display:grid;gap:8px;grid-template-columns:repeat(var(--cols,3),minmax(0,1fr));align-content:start}
figure{margin:0;background:var(--panel);border:1px solid var(--line);border-radius:4px;overflow:hidden}
figcaption{padding:4px 8px;font-size:12px}figcaption b{color:var(--accent);font-weight:600}
video{width:100%;display:block;background:#000;image-rendering:pixelated}
.empty{color:var(--dim);padding:40px}
</style></head><body>
<aside><h1>OpenVRX media</h1>
<h2>Panels</h2><div id="kinds"></div>
<h2>Groups</h2><div id="groups"></div></aside>
<main><div class="bar">
<button id="play">Play</button><button id="back">&#9664; field</button><button id="fwd">field &#9654;</button>
<button id="restart">Restart</button>
<label>Speed <select id="speed"><option>0.1</option><option>0.25</option><option>0.5</option><option selected>1</option></select></label>
<label><input type="checkbox" id="loop" checked> loop</label>
<label>Columns <select id="cols"><option>1</option><option>2</option><option selected>3</option><option>4</option><option>5</option></select></label>
<span class="info" id="time"></span></div>
<div id="grid"></div></main>
<script>
const G = __GALLERY__;
const FIELD = 1001/60000;
const state = (() => { try { return JSON.parse(localStorage.getItem("openvrx-media")) || {}; } catch (e) { return {}; } })();
state.groups = state.groups || {}; state.kinds = state.kinds || {}; state.items = state.items || {};
const save = () => { try { localStorage.setItem("openvrx-media", JSON.stringify(state)); } catch (e) {} };
const kinds = [...new Set(G.groups.flatMap(g => g.items.map(i => i.kind)))];
const kdiv = document.getElementById("kinds");
kinds.forEach(k => {
  const l = document.createElement("label");
  l.innerHTML = `<input type="checkbox"> ${k}`;
  const c = l.querySelector("input"); c.checked = state.kinds[k] !== false;
  c.onchange = () => { state.kinds[k] = c.checked; save(); render(); };
  kdiv.appendChild(l);
});
const gdiv = document.getElementById("groups");
const sections = [...new Set(G.groups.map(g => g.section))];
sections.forEach(s => {
  const h = document.createElement("div"); h.className = "info"; h.style.margin = "8px 0 2px"; h.textContent = s; gdiv.appendChild(h);
  G.groups.filter(g => g.section === s).forEach(g => {
    const l = document.createElement("label");
    l.innerHTML = `<input type="checkbox"> <span>${g.title}<br><span class="info">${g.info || ""}</span></span>`;
    const c = l.querySelector("input"); c.checked = !!state.groups[g.id];
    c.onchange = () => { state.groups[g.id] = c.checked; save(); render(); };
    gdiv.appendChild(l);
    const sub = document.createElement("div"); sub.className = "sub";
    g.items.forEach(it => {
      const key = g.id + "|" + it.src;
      const li = document.createElement("label");
      li.innerHTML = `<input type="checkbox"> <span class="info">${it.label}</span>`;
      const ci = li.querySelector("input"); ci.checked = state.items[key] !== false;
      ci.onchange = () => { state.items[key] = ci.checked; save(); render(); };
      sub.appendChild(li);
    });
    gdiv.appendChild(sub);
  });
});
const grid = document.getElementById("grid");
let vids = [];
function render() {
  grid.innerHTML = ""; vids = [];
  G.groups.filter(g => state.groups[g.id]).forEach(g => g.items.forEach(it => {
    if (state.kinds[it.kind] === false || state.items[g.id + "|" + it.src] === false) return;
    const f = document.createElement("figure");
    f.innerHTML = `<video src="${it.src}" muted playsinline preload="auto"></video>
      <figcaption><b>${it.label}</b> &middot; ${g.title}${it.info ? `<br><span class="info">${it.info}</span>` : ""}</figcaption>`;
    const v = f.querySelector("video"); v.loop = document.getElementById("loop").checked;
    v.playbackRate = +document.getElementById("speed").value;
    grid.appendChild(f); vids.push(v);
  }));
  if (!vids.length) grid.innerHTML = '<div class="empty">Tick one or more groups in the menu on the left.</div>';
  document.getElementById("play").textContent = "Play";
}
const lead = () => vids[0];
function seek(t) { vids.forEach(v => { v.currentTime = Math.max(0, Math.min(t, (v.duration || t))); }); }
document.getElementById("play").onclick = e => {
  if (!vids.length) return;
  const playing = !lead().paused;
  if (playing) { vids.forEach(v => v.pause()); e.target.textContent = "Play"; }
  else { seek(lead().currentTime); vids.forEach(v => v.play()); e.target.textContent = "Pause"; }
};
document.getElementById("fwd").onclick = () => { vids.forEach(v => v.pause()); if (lead()) seek(lead().currentTime + FIELD); };
document.getElementById("back").onclick = () => { vids.forEach(v => v.pause()); if (lead()) seek(lead().currentTime - FIELD); };
document.getElementById("restart").onclick = () => seek(0);
document.getElementById("speed").onchange = e => vids.forEach(v => v.playbackRate = +e.target.value);
document.getElementById("loop").onchange = e => vids.forEach(v => v.loop = e.target.checked);
const cols = document.getElementById("cols");
cols.value = state.cols || "3"; grid.style.setProperty("--cols", cols.value);
cols.onchange = () => { grid.style.setProperty("--cols", cols.value); state.cols = cols.value; save(); };
setInterval(() => {
  // Keep followers on the leader by speed, not by seeking: a seek stalls a
  // video while it decodes from a keyframe, and seeking every tick froze the
  // panels that drift most. Behind: play up to 10 % faster; ahead: slower.
  // Only a big jump (pause, step, a loop restart) is fixed with a seek.
  const l = lead(); if (!l) return;
  document.getElementById("time").textContent = `field ${Math.round(l.currentTime / FIELD)}  (${l.currentTime.toFixed(2)} s)`;
  const base = +document.getElementById("speed").value;
  if (l.paused) return;
  vids.slice(1).forEach(v => {
    const d = l.currentTime - v.currentTime;
    if (Math.abs(d) > 0.5) { v.currentTime = l.currentTime; v.playbackRate = base; }
    else v.playbackRate = base * (1 + Math.max(-0.1, Math.min(0.1, d * 2)));
  });
}, 250);
render();
</script></body></html>
"""


def write_page(g):
    gal = load_gallery(g)
    with open(os.path.join(g, "index.html"), "w") as f:
        f.write(PAGE.replace("__GALLERY__", json.dumps(gal)))


def open_page(g):
    page = "file://" + os.path.abspath(os.path.join(g, "index.html"))
    subprocess.Popen(["firefox", "--new-window", page], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, start_new_session=True)
    print(f"opened {page}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    lst = lambda v: [x for x in v.split(",") if x]  # noqa: E731
    d = sub.add_parser("denoise")
    d.add_argument("--gallery", required=True)
    d.add_argument("--data", required=True)
    d.add_argument("--scene", required=True)
    d.add_argument("--runs", type=lst, default=None)
    d.add_argument("--model", type=lst, default=None, help="one or more models, comma separated (a panel each)")
    d.add_argument("--filter", default=None)
    d.add_argument("--frames", type=int, default=120)
    r = sub.add_parser("run")
    r.add_argument("--gallery", required=True)
    r.add_argument("run_dirs", type=lambda v: [os.path.expanduser(x) for x in v.split(",") if x])
    r.add_argument("--fields", type=int, default=120)
    r.add_argument("--patterns", type=lst, default=None)
    r.add_argument("--steps", type=lambda v: [float(x) for x in v.split(",") if x], default=None)
    fl = sub.add_parser("flight")
    fl.add_argument("--gallery", required=True)
    fl.add_argument("recs", type=lambda v: [os.path.expanduser(x) for x in v.split(",") if x],
                    help="recording base paths, comma separated (cut long ones with cut_recording.py)")
    fl.add_argument("--model", type=lst, default=None, help="one or more models, comma separated (a panel each)")
    fl.add_argument("--filter", default=None)
    fl.add_argument("--fields", type=int, default=180)
    p = sub.add_parser("page")
    p.add_argument("--gallery", required=True)
    p.add_argument("--open", action="store_true")
    a = ap.parse_args()
    a.gallery = os.path.expanduser(a.gallery)
    os.makedirs(a.gallery, exist_ok=True)
    if a.cmd == "denoise":
        cmd_denoise(a)
    elif a.cmd == "run":
        cmd_run(a)
    elif a.cmd == "flight":
        cmd_flight(a)
    else:
        write_page(a.gallery)
        if a.open:
            open_page(a.gallery)


if __name__ == "__main__":
    main()
