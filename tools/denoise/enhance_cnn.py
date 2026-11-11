#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Post-flight picture enhancement: a CNN that restores detail and colour
lost in the analog chain, at twice the decode size.

Three decoded fields in (the middle one and its neighbours, Y/Cb/Cr each,
as denoise_cnn.py), a picture at 2x the field size out: convolutions at the
field size, then a pixel shuffle to 2x, added to the bilinear 2x of the
middle field (a zero net is the plain upscale).

Scenes are folders with a pairs.json (format openvrx-enhance-pairs):
decoded fields (SCENE/fields/REC/*.png, damage masks in SCENE/damage/REC/),
per field its parity, the picture it pairs with (HD_POOL/pNNNNN.png) and
whether it is usable, and one geometric transform per scene from the
picture's pixel grid to the decode's (see warp_target).

    enhance_cnn.py prepare --data DIR --scenes A,B --out CROPS.npz [--recs R1,R2] [--crops 400]
                           [--size 64] [--mask] [--max-damage 0.02]
    enhance_cnn.py train   --train A.npz,B.npz --val V.npz --out MODEL.pt
                           [--iters 20000] [--batch 16] [--width 48] [--depth 10]
                           [--chroma-weight 1.0] [--chroma-pool 16] [--mask]
                           [--checkpoint-every 500] [--resume] [--device auto] [--amp]
    enhance_cnn.py eval    --data DIR --scene NAME --model A.pt[,B.pt] [--recs R1] [--every 8]
                           [--out EVAL.json] [--save DIR]
    enhance_cnn.py apply   --model MODEL.pt --frames DIR --out DIR [--damage DIR]
    enhance_cnn.py smoke   [--threads 4]

prepare: random crops (3 fields of SIZE x SIZE and the 2*SIZE target, uint8
RGB; with --mask the 3 damage masks) from the usable fields of the scenes'
recordings (--recs: only those). Crops lie inside the moving picture and
inside the target picture; --max-damage leaves out crops whose middle
field has more than that share of damaged pixels. Hold whole recordings or
scenes out for --val.

train: Charbonnier (smooth L1) on Y/Cb/Cr, plus --chroma-weight x L1 of the
PxP average-pooled Cb/Cr (P = --chroma-pool, output pixels) against the
pull toward grey; the colour term can start late (--colour-start, share of
the iterations, then a 10 % ramp). Checkpoints, --resume, Ctrl+C/SIGTERM
handling, --device and --amp work as in denoise_cnn.py (kaggle_train.py
--script enhance_cnn.py runs it on a Kaggle GPU). The best model by
validation PSNR is OUT.

eval: per recording, every --every-th usable field (odd step: both
parities), RGB PSNR over the moving picture of the bicubic 2x of the raw
field and of each model against the target, and the colour ratio (mean
|Cb|+|Cr| of 16x16 block means, output over target). --save writes
side-by-side PNGs (bicubic | model | target) of the scored fields.

smoke: a CPU self-test on synthetic data (no files needed): warp identity,
model shapes, zero-init = bilinear, a short training that must lower the
loss, save/load and checkpoint/resume. Exit status 0 when all pass.

Needs torch, numpy, Pillow; prepare and eval also scipy.
"""
import argparse
import json
import os
import signal
import sys
import tempfile
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import denoise_cnn as dc  # noqa: E402  (colour space, PSNR, file helpers)

import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
import torch.nn.functional as F  # noqa: E402

PARITIES = ("First", "Second")


class Enhancer(nn.Module):
    """depth 3x3 convolutions (ReLU between), width channels, over the three
    fields' Y/Cb/Cr (9 planes, plus 3 damage masks / 255 with mask=True), at
    the field size; the last one makes 3 * scale^2 planes that a pixel
    shuffle turns into Y/Cb/Cr at scale x the size, added to the bilinear
    upscale of the middle field. Receptive field 2*depth+1 field pixels."""

    def __init__(self, width=48, depth=10, mask=False, scale=2):
        super().__init__()
        layers = [nn.Conv2d(9 + 3 * mask, width, 3, padding=1), nn.ReLU(inplace=True)]
        for _ in range(depth - 2):
            layers += [nn.Conv2d(width, width, 3, padding=1), nn.ReLU(inplace=True)]
        layers += [nn.Conv2d(width, 3 * scale * scale, 3, padding=1), nn.PixelShuffle(scale)]
        self.net = nn.Sequential(*layers)
        nn.init.zeros_(self.net[-2].weight)
        nn.init.zeros_(self.net[-2].bias)
        self.width, self.depth, self.mask, self.scale = width, depth, bool(mask), scale

    def forward(self, x):  # x: (B, 9[+3], H, W) Y/Cb/Cr / 255 of fields k-1, k, k+1 [, masks / 255]
        base = F.interpolate(x[:, 3:6], scale_factor=self.scale, mode="bilinear", align_corners=False)
        return base + self.net(x)


# --------------------------------------------------------------------------- geometry


def target_coords(tf, parity, r0, c0, h, w, hd_factor=2, out_factor=2):
    """Target picture pixel coordinates (rows, cols, each (h, w)) of output
    pixels r0..r0+h, c0..c0+w at out_factor x the decode grid. tf maps the
    picture's base grid (HD / hd_factor) to decode pixels:
    x_dec = sx * x + ox, y_dec = sy * y + oy[parity] (pixel centres)."""
    u = np.arange(c0, c0 + w, dtype=np.float64)
    v = np.arange(r0, r0 + h, dtype=np.float64)
    xd = (u + 0.5) / out_factor - 0.5
    yd = (v + 0.5) / out_factor - 0.5
    xs = (xd - tf["ox"]) / tf["sx"]
    ys = (yd - tf["oy"][parity]) / tf["sy"]
    xh = (xs + 0.5) * hd_factor - 0.5
    yh = (ys + 0.5) * hd_factor - 0.5
    return np.broadcast_to(yh[:, None], (h, w)), np.broadcast_to(xh[None, :], (h, w))


def target_inside(tf, parity, r0, c0, h, w, hd_shape, hd_factor=2, out_factor=2, margin=3):
    rows, cols = target_coords(tf, parity, r0, c0, h, w, hd_factor, out_factor)
    return (rows.min() >= margin and cols.min() >= margin and rows.max() <= hd_shape[0] - 1 - margin
            and cols.max() <= hd_shape[1] - 1 - margin)


def warp_target(hd, tf, parity, r0, c0, h, w, hd_factor=2, out_factor=2):
    """The target picture (HxWx3 uint8) resampled (cubic spline) onto output
    pixels r0..r0+h, c0..c0+w at out_factor x the decode grid: float (h, w, 3).
    Outside the picture: edge pixels repeated."""
    from scipy.ndimage import map_coordinates
    rows, cols = target_coords(tf, parity, r0, c0, h, w, hd_factor, out_factor)
    # Only the window the coordinates touch (plus the spline's support).
    a0 = int(max(0, np.floor(rows.min()) - 4))
    a1 = int(min(hd.shape[0], np.ceil(rows.max()) + 5))
    b0 = int(max(0, np.floor(cols.min()) - 4))
    b1 = int(min(hd.shape[1], np.ceil(cols.max()) + 5))
    sub = np.asarray(hd[a0:a1, b0:b1], dtype=np.float64)
    out = np.empty((h, w, 3))
    for ch in range(3):
        out[..., ch] = map_coordinates(sub[..., ch], [rows - a0, cols - b0], order=3, mode="nearest")
    return np.clip(out, 0, 255)


# --------------------------------------------------------------------------- data


def load_scene(data, scene):
    sd = os.path.join(data, scene)
    p = json.load(open(os.path.join(sd, "pairs.json")))
    if p.get("format") != "openvrx-enhance-pairs":
        sys.exit(f"{sd}/pairs.json is not an openvrx-enhance-pairs file")
    return sd, p


def usable(rows):
    """Field indexes with both neighbours decoded and the field itself usable."""
    return [t for t in range(1, len(rows) - 1) if rows[t]["ok"]]


class HdCache:
    def __init__(self, pool, keep=24):
        self.pool, self.keep, self.c = pool, keep, {}

    def get(self, pos):
        if pos not in self.c:
            if len(self.c) >= self.keep:
                self.c.pop(next(iter(self.c)))
            self.c[pos] = dc.load_rgb(os.path.join(self.pool, f"p{pos:05d}.png"))
        return self.c[pos]


def cmd_prepare(a):
    rng = np.random.default_rng(a.seed)
    X, Y, M, meta = [], [], [], []
    s, S = a.size, 2 * a.size
    for scene in a.scenes:
        sd, p = load_scene(a.data, scene)
        tf, hf, of = p["transform"], p["hd_factor"], p["out_factor"]
        if of != 2:
            sys.exit(f"{scene}: out_factor {of}, this model makes 2x")
        r0, r1, c0, c1 = p["moving_region"]
        hd = HdCache(p["hd_pool"])
        for rec, info in p["recordings"].items():
            if a.recs and rec not in a.recs:
                continue
            rows = info["fields"]
            ts = usable(rows)
            if not ts:
                print(f"  {scene}/{rec}: no usable fields, skipped")
                continue
            fd = os.path.join(sd, "fields", rec)
            md = os.path.join(sd, "damage", rec)
            if (a.mask or a.max_damage < 1) and not os.path.isdir(md):
                sys.exit(f"{scene}/{rec}: no damage masks in {md}")
            # Fields in random order, several crops each (the HD picture is read once).
            per = max(1, a.crops // len(ts))
            got, tries = 0, 0
            order = list(rng.permutation(ts))
            while got < a.crops and tries < 20 * a.crops:
                t = int(order[tries % len(order)])
                tries += 1
                fr = [dc.load_rgb(os.path.join(fd, rows[t + d]["f"])) for d in (-1, 0, 1)]
                ms = [dc.load_mask(os.path.join(md, rows[t + d]["f"])) for d in (-1, 0, 1)] \
                    if (a.mask or a.max_damage < 1) else None
                target = hd.get(rows[t]["pos"])
                for _ in range(per):
                    if got >= a.crops:
                        break
                    rr = int(rng.integers(r0, r1 - s))
                    cc = int(rng.integers(c0, c1 - s))
                    if not target_inside(tf, rows[t]["parity"], 2 * rr, 2 * cc, S, S, target.shape, hf, of):
                        continue
                    if ms is not None and (ms[1][rr:rr + s, cc:cc + s] > 0).mean() > a.max_damage:
                        continue
                    win = (slice(rr, rr + s), slice(cc, cc + s))
                    X.append(np.concatenate([f[win] for f in fr], -1).transpose(2, 0, 1))
                    y = warp_target(target, tf, rows[t]["parity"], 2 * rr, 2 * cc, S, S, hf, of)
                    Y.append((y + 0.5).astype(np.uint8).transpose(2, 0, 1))
                    if a.mask:
                        M.append(np.stack([m[win] for m in ms]))
                    got += 1
            meta.append({"scene": scene, "rec": rec, "crops": got, "cn_db": info.get("cn_db"),
                         "usable_fields": len(ts)})
            print(f"  {scene}/{rec}: {got} crops from {len(ts)} usable fields", flush=True)
    if not X:
        sys.exit("no crops")
    extra = {"m": np.stack(M)} if a.mask else {}
    np.savez_compressed(a.out, x=np.stack(X), y=np.stack(Y), meta=json.dumps(meta), size=s, **extra)
    print(f"{len(X)} crops of {s}x{s} -> {S}x{S}" + (" with masks" if a.mask else "") + f" -> {a.out}")


def load_crops(paths, mask=False):
    xs, ys, ms = [], [], []
    for p in paths:
        d = np.load(p)
        if d["y"].shape[-1] != 2 * d["x"].shape[-1]:
            sys.exit(f"{p}: targets are not 2x the inputs (a denoise_cnn.py crop file?)")
        xs.append(d["x"])
        ys.append(d["y"])
        if mask:
            if "m" not in d:
                sys.exit(f"{p} has no damage masks: make it with `prepare --mask`")
            ms.append(d["m"])
    m = torch.from_numpy(np.concatenate(ms)) if mask else None
    return torch.from_numpy(np.concatenate(xs)), torch.from_numpy(np.concatenate(ys)), m


def augment(x, y, m, g):
    """Flips only (no transpose: the scan-line axis is special)."""
    if torch.rand(1, generator=g).item() < 0.5:
        x, y = x.flip(-1), y.flip(-1)
    if torch.rand(1, generator=g).item() < 0.5:
        x, y = x.flip(-2), y.flip(-2)
    return x, y


def bicubic_up(x):
    """Middle field's Y/Cb/Cr upscaled 2x (bicubic): the baseline."""
    return F.interpolate(x[:, 3:6], scale_factor=2, mode="bicubic", align_corners=False)


# --------------------------------------------------------------------------- train


def cmd_train(a):
    torch.set_num_threads(a.threads)
    torch.manual_seed(a.seed)
    g = torch.Generator().manual_seed(a.seed)
    xt, yt, mt = load_crops(a.train, a.mask)
    xv, yv, mv = load_crops(a.val, a.mask) if a.val else (None, None, None)
    print(f"train crops {len(xt)} ({xt.shape[-1]} -> {yt.shape[-1]})"
          + (f", val crops {len(xv)}" if xv is not None else "") + (" (with damage masks)" if a.mask else ""))
    dev = torch.device("cuda" if a.device == "auto" and torch.cuda.is_available()
                       else ("cpu" if a.device == "auto" else a.device))
    to = (lambda t: t.to(dev) if t is not None else None)
    xt, yt, mt, xv, yv, mv = map(to, (xt, yt, mt, xv, yv, mv))
    model = Enhancer(a.width, a.depth, a.mask).to(dev)
    params = sum(p.numel() for p in model.parameters())
    print(f"model: {a.depth} layers x {a.width} channels, 2x out, {params} parameters, on {dev}"
          + (f" ({torch.cuda.get_device_name(dev)})" if dev.type == "cuda" else ""), flush=True)
    amp = bool(a.amp and dev.type == "cuda")
    scaler = torch.amp.GradScaler("cuda", enabled=amp)
    opt = torch.optim.Adam(model.parameters(), lr=a.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.iters)
    ckpt = a.out + ".ckpt"
    log, best, start = [], None, 1
    shape = (a.width, a.depth, a.mask, a.iters)
    if a.resume and os.path.exists(ckpt):
        st = torch.load(ckpt, map_location="cpu", weights_only=False)
        if (st["width"], st["depth"], st["mask"], st["iters"]) != shape:
            sys.exit(f"{ckpt} was made with width/depth/mask/iters {st['width']}/{st['depth']}/{st['mask']}/"
                     f"{st['iters']}: resume with the same flags, or delete it to start over")
        model.load_state_dict(st["state"])
        opt.load_state_dict(st["opt"])
        sched.load_state_dict(st["sched"])
        g.set_state(st["gen"])
        torch.set_rng_state(st["torch_rng"])
        if amp and st.get("scaler"):
            scaler.load_state_dict(st["scaler"])
        log, best, start = st["log"], st["best"], st["it"] + 1
        print(f"resumed from {ckpt} at iteration {st['it']} (best val {best})")
    elif a.resume:
        print(f"no {ckpt} yet: starting from iteration 1")
    if a.init and start == 1:
        model.load_state_dict(torch.load(os.path.expanduser(a.init), map_location="cpu", weights_only=False)["state"])
        print(f"initialised from {a.init}")

    stop = {"sig": None}

    def on_signal(sig, _frame):
        stop["sig"] = sig

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def save_ckpt(it):
        tmp = ckpt + ".tmp"
        torch.save({"it": it, "width": a.width, "depth": a.depth, "mask": a.mask, "iters": a.iters,
                    "state": model.state_dict(), "opt": opt.state_dict(), "sched": sched.state_dict(),
                    "gen": g.get_state(), "torch_rng": torch.get_rng_state(), "log": log, "best": best,
                    "scaler": scaler.state_dict() if amp else None}, tmp)
        os.replace(tmp, ckpt)

    def val_score():
        if xv is None:
            return None
        model.eval()
        raw, out = [], []
        with torch.no_grad():
            for i in range(0, len(xv), 32):
                x = dc.make_input(xv[i:i + 32], mv[i:i + 32] if mv is not None else None)
                y = dc.rgb_to_ycc(yv[i:i + 32].float())
                raw.append(dc.psnr_t(bicubic_up(x), y))
                out.append(dc.psnr_t(model(x), y))
        model.train()
        return float(np.mean(raw)), float(np.mean(out))

    t0, it0 = time.time(), start
    it = start - 1
    colour = a.chroma_weight > 0
    for it in range(start, a.iters + 1):
        idx = torch.randint(0, len(xt), (a.batch,), generator=g).to(dev)
        x = dc.make_input(xt[idx], mt[idx] if mt is not None else None)
        y = dc.rgb_to_ycc(yt[idx].float())
        x, y = augment(x, y, None, g)
        with torch.autocast(device_type=dev.type, dtype=torch.float16, enabled=amp):
            pred = model(x)
        pred = pred.float()
        pix = torch.sqrt((pred - y).pow(2) + 1e-6).mean()
        loss = pix
        s0 = a.colour_start * a.iters
        ramp = 0.0 if it < s0 else min(1.0, (it - s0) / max(0.1 * a.iters, 1))
        if colour:
            col = (F.avg_pool2d(pred[:, 1:3], a.chroma_pool) - F.avg_pool2d(y[:, 1:3], a.chroma_pool)).abs().mean()
            loss = loss + ramp * a.chroma_weight * col
        opt.zero_grad()
        scaler.scale(loss).backward()
        scaler.step(opt)
        scaler.update()
        sched.step()
        if it % a.log_every == 0 or it == a.iters:
            v = val_score()
            el = time.time() - t0
            per = el / max(it - it0 + 1, 1)
            entry = {"iter": it, "loss": round(loss.item(), 5), "pix": round(pix.item(), 5),
                     "s": round(el, 1), "s_per_iter": round(per, 3),
                     "eta_min": round(per * (a.iters - it) / 60, 1), "lr": round(sched.get_last_lr()[0], 7)}
            if colour:
                entry["chroma"] = round(col.item(), 5)
            if v:
                entry.update(val_bicubic_db=round(v[0], 2), val_model_db=round(v[1], 2))
            log.append(entry)
            print(json.dumps(entry), flush=True)
            if v and (ramp >= 1.0 or not colour) and (best is None or v[1] > best):
                best = v[1]
                save(model, a, log, a.out)
        if a.max_minutes and time.time() - t0 > 60 * a.max_minutes:
            print(f"--max-minutes {a.max_minutes} reached at iteration {it}", flush=True)
            stop["sig"] = "time"
        if it % a.checkpoint_every == 0 or stop["sig"] is not None:
            save_ckpt(it)
        if stop["sig"] is not None:
            if xv is None or best is None:
                save(model, a, log, a.out)
            print(f"stopped ({stop['sig']}) at iteration {it}: checkpoint {ckpt}; continue with --resume",
                  flush=True)
            sys.exit(130 if stop["sig"] != "time" else 0)
    save_ckpt(it)
    if xv is None or best is None:
        save(model, a, log, a.out)
    print(f"done in {time.time() - t0:.0f} s -> {a.out}" + (f" (best val {best:.2f} dB)" if best else ""))


def save(model, a, log, path):
    torch.save({"format": "openvrx-enhance-cnn", "version": 1, "width": model.width, "depth": model.depth,
                "mask": model.mask, "scale": model.scale,
                "planes": "3 fields x Y/Cb/Cr (denoise_filter.to_ycc) / 255"
                          + (", then the 3 fields' damage masks / 255" if model.mask else "")
                          + "; output Y/Cb/Cr / 255 at 2x, added to the bilinear 2x of the middle field",
                "state": model.state_dict(), "train": [os.path.basename(p) for p in a.train],
                "loss": {"pixel": "charbonnier", "chroma_weight": a.chroma_weight, "chroma_pool": a.chroma_pool},
                "iters": a.iters, "seed": a.seed, "val": [os.path.basename(p) for p in (a.val or [])],
                "log": log}, path)


def load_model(path):
    ck = torch.load(os.path.expanduser(path), map_location="cpu", weights_only=False)
    if ck.get("format") != "openvrx-enhance-cnn":
        sys.exit(f"{path} is not an enhance model")
    m = Enhancer(ck["width"], ck["depth"], ck.get("mask", False), ck.get("scale", 2))
    m.load_state_dict(ck["state"])
    m.eval()
    return m


def run_model(model, rgb_frames, k, masks=None):
    """Enhanced field k (needs k-1, k+1) as Y/Cb/Cr (3, 2H, 2W), 0..255 scale."""
    x = np.concatenate([rgb_frames[k + d] for d in (-1, 0, 1)], -1).transpose(2, 0, 1)
    m = None
    if model.mask:
        if masks is None:
            raise ValueError("this model needs the decoder's damage masks (decode with --damage)")
        m = torch.from_numpy(np.stack([masks[k + d] for d in (-1, 0, 1)]))[None]
    with torch.no_grad():
        y = model(dc.make_input(torch.from_numpy(x)[None], m))[0]
    return (y * 255.0).numpy()


def ycc_np(rgb):
    rgb = np.asarray(rgb, dtype=np.float64)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return np.stack([y, 0.564 * (b - y), 0.713 * (r - y)])


def colour(ycc, block=16):
    c = ycc[1:]
    hh, ww = (c.shape[1] // block) * block, (c.shape[2] // block) * block
    b = c[:, :hh, :ww].reshape(2, hh // block, block, ww // block, block).mean(axis=(2, 4))
    return float(np.abs(b).sum(0).mean())


def cmd_eval(a):
    import denoise_filter as lin
    torch.set_num_threads(a.threads)
    models = {dc.model_name(p): load_model(p) for p in a.model}
    need_mask = any(m.mask for m in models.values())
    sd, p = load_scene(a.data, a.scene)
    tf, hf = p["transform"], p["hd_factor"]
    r0, r1, c0, c1 = (2 * v for v in p["moving_region"])
    hd = HdCache(p["hd_pool"], keep=4)
    res = {}
    step = a.every if a.every % 2 else a.every + 1
    if a.save:
        os.makedirs(a.save, exist_ok=True)
    for rec, info in p["recordings"].items():
        if a.recs and rec not in a.recs:
            continue
        rows = info["fields"]
        ts = usable(rows)[::step]
        fd, md = os.path.join(sd, "fields", rec), os.path.join(sd, "damage", rec)
        scores = {"bicubic": [], **{n: [] for n in models}}
        sat = {"target": [], "bicubic": [], **{n: [] for n in models}}
        for t in ts:
            idx = (t - 1, t, t + 1)
            rgb = {q: dc.load_rgb(os.path.join(fd, rows[q]["f"])) for q in idx}
            masks = {q: dc.load_mask(os.path.join(md, rows[q]["f"])) for q in idx} if need_mask else None
            h, w = rgb[t].shape[:2]
            tgt = warp_target(hd.get(rows[t]["pos"]), tf, rows[t]["parity"], 0, 0, 2 * h, 2 * w, hf, 2)
            ty = ycc_np(tgt)[:, r0:r1, c0:c1]
            x = dc.make_input(torch.from_numpy(np.concatenate([rgb[q] for q in idx], -1).transpose(2, 0, 1))[None])
            with torch.no_grad():
                outs = {"bicubic": (bicubic_up(x)[0] * 255.0).numpy()}
            for n, m in models.items():
                outs[n] = run_model(m, rgb, t, masks)
            sat["target"].append(colour(ty))
            for n, o in outs.items():
                o_rgb = lin.to_rgb(o).astype(np.float64)
                scores[n].append(lin.psnr(o_rgb[r0:r1, c0:c1], tgt[r0:r1, c0:c1]))
                sat[n].append(colour(ycc_np(o_rgb)[:, r0:r1, c0:c1]))
            if a.save:
                tiles = [lin.to_rgb(outs["bicubic"])] + [lin.to_rgb(outs[n]) for n in models] \
                    + [(tgt + 0.5).astype(np.uint8)]
                Image.fromarray(np.concatenate(tiles, 1)).save(os.path.join(a.save, f"{rec}-{rows[t]['f']}"))
        if not ts:
            continue
        ts_ = np.mean(sat["target"])
        r = {n: {"rgb_db": round(float(np.mean(v)), 3), "colour_ratio": round(float(np.mean(sat[n]) / ts_), 3)}
             for n, v in scores.items()}
        r["fields"] = len(ts)
        r["cn_db"] = info.get("cn_db")
        res[rec] = r
        print(f"{rec:>10} ({len(ts)} fields)  " + "   ".join(
            f"{n} {r[n]['rgb_db']:6.2f} dB colour {r[n]['colour_ratio']:.2f}" for n in scores), flush=True)
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"scene": a.scene, "models": a.model, "every": step, "psnr": res}, f, indent=1)
        print(f"-> {a.out}")


def cmd_apply(a):
    import denoise_filter as lin
    torch.set_num_threads(a.threads)
    model = load_model(a.model)
    paths = lin.frames(a.frames)
    rgb = [dc.load_rgb(p) for p in paths]
    masks = [dc.load_mask(os.path.join(a.damage, os.path.basename(p))) for p in paths] if model.mask else None
    os.makedirs(a.out, exist_ok=True)
    for k, p in enumerate(paths):
        if 1 <= k < len(paths) - 1:
            y = run_model(model, rgb, k, masks)
        else:  # no neighbour: plain upscale
            x = torch.from_numpy(np.concatenate([rgb[k]] * 3, -1).transpose(2, 0, 1))[None]
            y = (bicubic_up(dc.make_input(x))[0] * 255.0).numpy()
        Image.fromarray(lin.to_rgb(y)).save(os.path.join(a.out, os.path.basename(p)))
    print(f"{len(paths)} fields -> {a.out} (2x)")


# --------------------------------------------------------------------------- smoke test


def cmd_smoke(a):
    torch.set_num_threads(a.threads)
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"{'PASS' if cond else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""), flush=True)
        ok = ok and bool(cond)

    rng = np.random.default_rng(0)
    # 1. Warp identity: sx = sy = 1, no offset, HD factor 2, 2x out: the HD picture itself.
    hd = rng.integers(0, 256, (96, 128, 3)).astype(np.uint8)
    tf = {"sx": 1.0, "sy": 1.0, "ox": 0.0, "oy": {"First": 0.0, "Second": 0.0}}
    try:
        wt = warp_target(hd, tf, "First", 10, 12, 40, 50)
        err = float(np.abs(wt - hd[10:50, 12:62]).max())
        check("warp identity", err < 1e-6, f"max error {err:.2e}")
        # A shift: ox = 1 decode px -> the target moves by 2 HD px.
        tf2 = dict(tf, ox=1.0)
        wt2 = warp_target(hd, tf2, "First", 10, 12, 40, 50)
        err2 = float(np.abs(wt2 - hd[10:50, 10:60]).max())
        check("warp offset convention", err2 < 1e-6, f"max error {err2:.2e}")
    except ImportError:
        check("warp (scipy)", False, "scipy missing")

    # 2. Shapes and zero init.
    m = Enhancer(16, 4)
    x = torch.rand(2, 9, 24, 32)
    with torch.no_grad():
        y = m(x)
    check("output shape", tuple(y.shape) == (2, 3, 48, 64), str(tuple(y.shape)))
    base = F.interpolate(x[:, 3:6], scale_factor=2, mode="bilinear", align_corners=False)
    check("zero init = bilinear 2x", float((y - base).abs().max()) == 0.0)
    mm = Enhancer(16, 4, mask=True)
    check("mask model shape", tuple(mm(torch.rand(1, 12, 16, 16)).shape) == (1, 3, 32, 32))

    # 3. Short training on synthetic pairs: smooth colour pictures with
    #    edges at 2x; inputs are their blurred, noisy half-size versions.
    with tempfile.TemporaryDirectory() as d:
        def pictures(n, s):
            out = []
            for _ in range(n):
                yy, xx = np.mgrid[0:2 * s, 0:2 * s] / (2 * s)
                p = np.zeros((2 * s, 2 * s, 3))
                for c in range(3):
                    f = rng.uniform(2, 9, 2)
                    p[..., c] = 128 + 90 * np.sin(2 * np.pi * (f[0] * xx + rng.uniform()) ) * np.cos(2 * np.pi * f[1] * yy)
                p[(xx > rng.uniform(0.2, 0.8))] *= 0.6
                out.append(np.clip(p, 0, 255).astype(np.uint8))
            return out

        def degrade(p):
            t = torch.from_numpy(p.transpose(2, 0, 1)).float()[None]
            lo = F.avg_pool2d(t, 2)
            lo = F.avg_pool2d(F.pad(lo, (1, 1, 0, 0), mode="replicate"), (1, 3), stride=1)  # horizontal blur
            lo = lo.clone()
            lo[:, 1:] = 0.7 * lo[:, 1:] + 0.3 * lo.mean(1, keepdim=True)  # weaker colour
            lo = lo + torch.randn_like(lo) * 3
            return lo.clamp(0, 255)[0].numpy().astype(np.uint8)

        s = 24
        for name, n in (("train", 96), ("val", 24)):
            P = pictures(n, s)
            lo = [degrade(p) for p in P]
            X = np.stack([np.concatenate([q] * 3, 0) for q in lo])
            Y = np.stack([p.transpose(2, 0, 1) for p in P])
            np.savez_compressed(os.path.join(d, f"{name}.npz"), x=X, y=Y, meta="[]", size=s)
        out = os.path.join(d, "m.pt")
        base_args = ["train", "--train", os.path.join(d, "train.npz"), "--val", os.path.join(d, "val.npz"),
                     "--out", out, "--width", "16", "--depth", "4", "--batch", "8", "--log-every", "20",
                     "--threads", str(a.threads), "--chroma-weight", "1.0", "--chroma-pool", "8",
                     "--device", "cpu"]
        import subprocess
        t0 = time.time()
        p1 = subprocess.run([sys.executable, os.path.abspath(__file__), *base_args, "--iters", "120",
                             "--checkpoint-every", "60"], capture_output=True, text=True)
        logs = [json.loads(line) for line in p1.stdout.splitlines() if line.startswith("{")]
        check("train runs", p1.returncode == 0, p1.stderr.strip()[-300:] if p1.returncode else f"{time.time() - t0:.0f} s")
        if logs:
            first, last = logs[0], logs[-1]
            check("loss falls", last["pix"] < first["pix"], f"pix {first['pix']} -> {last['pix']}")
            check("beats bicubic on val", last["val_model_db"] > last["val_bicubic_db"],
                  f"{last['val_bicubic_db']} -> {last['val_model_db']} dB")
        check("model saved", os.path.exists(out))
        if os.path.exists(out):
            mdl = load_model(out)
            xs = torch.from_numpy(np.load(os.path.join(d, "val.npz"))["x"][:2])
            yo = mdl(dc.make_input(xs))
            check("load + run", tuple(yo.shape) == (2, 3, 2 * s, 2 * s))
        # Resume: extend nothing, but the checkpoint must load and continue to the end.
        p2 = subprocess.run([sys.executable, os.path.abspath(__file__), *base_args, "--iters", "120",
                             "--checkpoint-every", "60", "--resume"], capture_output=True, text=True)
        check("resume from checkpoint", p2.returncode == 0 and "resumed from" in p2.stdout,
              p2.stderr.strip()[-300:])
    print("smoke test " + ("passed" if ok else "FAILED"))
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    csv = lambda v: [x for x in v.split(",") if x]  # noqa: E731
    p = sub.add_parser("prepare")
    p.add_argument("--data", required=True)
    p.add_argument("--scenes", type=csv, required=True)
    p.add_argument("--recs", type=csv, default=None, help="only these recordings (names in pairs.json)")
    p.add_argument("--out", required=True)
    p.add_argument("--crops", type=int, default=400, help="crops per recording")
    p.add_argument("--size", type=int, default=64, help="input crop size (target: twice)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--mask", action="store_true", help="also store the damage masks")
    p.add_argument("--max-damage", type=float, default=1.0,
                   help="leave out crops whose middle field has more damaged pixels than this share (1: off)")
    t = sub.add_parser("train")
    t.add_argument("--train", type=csv, required=True)
    t.add_argument("--val", type=csv, default=None)
    t.add_argument("--out", required=True)
    t.add_argument("--iters", type=int, default=20000)
    t.add_argument("--batch", type=int, default=16)
    t.add_argument("--width", type=int, default=48)
    t.add_argument("--depth", type=int, default=10)
    t.add_argument("--lr", type=float, default=5e-4)
    t.add_argument("--chroma-weight", type=float, default=1.0, help="weight of the pooled Cb/Cr L1 term (0: off)")
    t.add_argument("--chroma-pool", type=int, default=16, help="block size (output pixels) of the colour term")
    t.add_argument("--colour-start", type=float, default=0.0,
                   help="share of the iterations before the colour term starts (then ramped in over 10 %%)")
    t.add_argument("--mask", action="store_true", help="damage masks as 3 more input planes")
    t.add_argument("--log-every", type=int, default=200)
    t.add_argument("--device", default="auto")
    t.add_argument("--amp", action="store_true")
    t.add_argument("--checkpoint-every", type=int, default=500)
    t.add_argument("--resume", action="store_true")
    t.add_argument("--init", default=None, help="start from this model's weights (same shape)")
    t.add_argument("--max-minutes", type=float, default=0,
                   help="stop cleanly (model + checkpoint) after this many minutes (0: no limit)")
    t.add_argument("--threads", type=int, default=4)
    t.add_argument("--seed", type=int, default=1)
    e = sub.add_parser("eval")
    e.add_argument("--data", required=True)
    e.add_argument("--scene", required=True)
    e.add_argument("--model", type=csv, required=True)
    e.add_argument("--recs", type=csv, default=None)
    e.add_argument("--every", type=int, default=8)
    e.add_argument("--save", default=None, help="folder for side-by-side PNGs of the scored fields")
    e.add_argument("--threads", type=int, default=4)
    e.add_argument("--out")
    x = sub.add_parser("apply")
    x.add_argument("--model", required=True)
    x.add_argument("--frames", required=True)
    x.add_argument("--out", required=True)
    x.add_argument("--damage", default=None)
    x.add_argument("--threads", type=int, default=4)
    s = sub.add_parser("smoke")
    s.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    {"prepare": cmd_prepare, "train": cmd_train, "eval": cmd_eval, "apply": cmd_apply, "smoke": cmd_smoke}[a.cmd](a)


if __name__ == "__main__":
    main()
