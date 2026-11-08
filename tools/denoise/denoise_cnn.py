#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Small residual CNN for the post-flight denoise.

Same job and data as denoise_filter.py (the linear 5x5x3 filter): three
decoded frames in (the middle one and its neighbours, Y/Cb/Cr each), a
correction to the middle frame out. Same colour space, alignment and PSNR
as the linear filter, so `eval` compares raw, linear and CNN on one scale.

Scenes are folders in the denoise_filter.py layout: SCENE/ref/*.png (clean)
and SCENE/cn*/ (noisy decodes), from `denoise_filter.py simulate` (real
footage through the simulator) or `bench_dataset.py build` (bench ladders).
Decoder damage masks, when made (`simulate --damage`, bench_dataset), sit in
SCENE/damage/RUN/ under the same file names.

    denoise_cnn.py prepare --data DIR --scenes A,B --out CROPS.npz [--crops 48] [--size 64] [--mask]
    denoise_cnn.py train   --train A.npz,B.npz --val V.npz --out MODEL.pt
                           [--iters 3000] [--batch 16] [--width 32] [--depth 8] [--threads 4]
                           [--chroma-weight W] [--chroma-pool 8] [--mask]
                           [--checkpoint-every 500] [--resume] [--device auto] [--amp]
    denoise_cnn.py eval    --data DIR --scene NAME --model A.pt[,B.pt] [--filter LINEAR.json]
                           [--runs cn8,cn10] [--every 8] [--out EVAL.json]
    denoise_cnn.py apply   --model MODEL.pt --frames DIR --out DIR [--damage DIR]
    denoise_cnn.py render  --rec REC --model MODEL.pt --out OUT.mp4 [--panels raw,model]
                           [--filter LINEAR.json] [--seconds N] [--crf 18] [--work DIR]

prepare: random aligned crops (3 noisy frames + the clean middle frame) from
every noisy run of the scenes, stored as uint8 RGB (and, with --mask, the
3 frames' damage masks). Hold whole scenes out for --val: a held-out pattern
or clip shows whether the model learned the noise or memorised the pictures
(bench patterns are static).

train: --chroma-weight (v2) adds W x L1 between the PxP average-pooled Cb/Cr
of the output and of the reference to the per-pixel loss, against the pull
toward grey. --mask (v3) gives the model the 3 damage masks as 3 more input
planes (crops must come from `prepare --mask`). Every --checkpoint-every
iterations the full training state (model, optimiser, LR schedule, RNG,
log) goes to OUT.ckpt; --resume continues from it (and starts fresh when
there is none, so a script can always pass it). Ctrl+C or SIGTERM saves a
checkpoint and stops cleanly. The best model by validation PSNR is OUT.
--device auto trains on a GPU when torch has one (kaggle_train.py runs it on
a Kaggle T4); --amp adds float16 convolutions there.

eval scores every model on the same frames and also prints a saturation
ratio: mean |Cb|+|Cr| of 8x8 block means, output over reference.

render: a whole recording through a model into an MP4 (59.94 fields/s; x264
when ffmpeg has it, else VP9 as on Fedora's ffmpeg-free):
decodes every field (or the first --seconds), runs the model field by field
and pipes the result to ffmpeg. --panels picks what is shown side by side,
left to right, from raw, linear (needs --filter) and model. On a 4-core CPU
budget about 1 s per field for an 8x32 model (a 30 s flight: about 30 min).

The OSD rows (bottom 30 %) are left out of crops and scores for simulated
scenes, as in denoise_filter.py; bench scenes (with a build.json) have no OSD
and keep them. Needs torch (CPU is enough), numpy and Pillow; render needs
ffmpeg and openvrx-cli. The models train on CPU or GPU; eval, apply and
render run on the CPU.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import denoise_filter as lin  # noqa: E402  (same folder: colour space, align, score)

import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
import torch.nn.functional as F  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLI = os.path.join(REPO, "host", "target", "main", "release", "openvrx-cli")

# RGB -> Y/Cb/Cr as denoise_filter.to_ycc (BT.601, no offset), and back.
YCC = torch.tensor([[0.299, 0.587, 0.114],
                    [-0.564 * 0.299, -0.564 * 0.587, 0.564 * 0.886],
                    [0.713 * 0.701, -0.713 * 0.587, -0.713 * 0.114]])
RGB = torch.linalg.inv(YCC)


def rgb_to_ycc(x):
    """x: (..., 3k, H, W) RGB in 0..255 -> Y/Cb/Cr scaled by 1/255."""
    s = x.shape
    x = x.reshape(s[0], -1, 3, s[-2], s[-1]) / 255.0
    return torch.einsum("oc,bkchw->bkohw", YCC.to(x.device), x).reshape(s)


def ycc_to_rgb(x):
    return torch.einsum("oc,bchw->bohw", RGB.to(x.device), x) * 255.0


class Denoiser(nn.Module):
    """depth 3x3 convolutions (ReLU between), width channels, over the three
    frames' Y/Cb/Cr stacked (9 planes, plus their 3 damage masks / 255 with
    mask=True); the output is added to the middle frame, so a zero net is
    the raw decode. Receptive field 2*depth+1."""

    def __init__(self, width=32, depth=8, mask=False):
        super().__init__()
        layers = [nn.Conv2d(9 + 3 * mask, width, 3, padding=1), nn.ReLU(inplace=True)]
        for _ in range(depth - 2):
            layers += [nn.Conv2d(width, width, 3, padding=1), nn.ReLU(inplace=True)]
        layers += [nn.Conv2d(width, 3, 3, padding=1)]
        self.net = nn.Sequential(*layers)
        nn.init.zeros_(self.net[-1].weight)
        nn.init.zeros_(self.net[-1].bias)
        self.width, self.depth, self.mask = width, depth, bool(mask)

    def forward(self, x):  # x: (B, 9[+3], H, W) Y/Cb/Cr / 255 of frames k-1, k, k+1 [, masks / 255]
        return x[:, 3:6] + self.net(x)


def is_bench(scene_dir):
    return os.path.exists(os.path.join(scene_dir, "build.json"))


def load_rgb(path):
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)


def mask_dir_of(run_dir):
    """SCENE/RUN -> SCENE/damage/RUN (where simulate --damage and bench_dataset put masks)."""
    return os.path.join(os.path.dirname(run_dir), "damage", os.path.basename(run_dir))


def load_mask(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.uint8)


# ---------------------------------------------------------------------------


def cmd_prepare(a):
    rng = np.random.default_rng(a.seed)
    X, Y, M, meta = [], [], [], []
    for scene in a.scenes:
        sd = os.path.join(a.data, scene)
        ref_dir, runs = lin.pairs(sd)
        ref_paths = lin.frames(ref_dir)
        ref_y = [lin.to_ycc(p) for p in ref_paths]  # for alignment only
        bottom = None if is_bench(sd) else lin.OSD_SHARE
        for run in runs:
            paths = lin.frames(run)
            if a.mask:
                mdir = mask_dir_of(run)
                missing = [p for p in paths if not os.path.exists(os.path.join(mdir, os.path.basename(p)))]
                if missing and a.skip_unmasked:
                    print(f"  {scene}/{os.path.basename(run)}: no damage masks, skipped")
                    continue
                if missing:
                    sys.exit(f"{scene}/{os.path.basename(run)}: {len(missing)} damage masks missing in {mdir} "
                             "(redo it with `denoise_filter.py simulate --damage` or bench_dataset.py)")
            seq_y = [lin.to_ycc(p) for p in paths]
            if len(seq_y) < 3:
                print(f"  {scene}/{os.path.basename(run)}: only {len(seq_y)} frames decoded, skipped")
                continue
            off = lin.align(ref_y, seq_y)
            ks = [k for k in range(len(ref_paths)) if 1 <= k + off < len(paths) - 1]
            if not ks:
                continue
            h, w = seq_y[0].shape[1:]
            hmax = int(h * bottom) if bottom else h
            cache = {}

            def rgb(p):
                if p not in cache:
                    cache[p] = load_rgb(p)
                return cache[p]

            def msk(p):
                q = os.path.join(mask_dir_of(run), os.path.basename(p))
                if q not in cache:
                    cache[q] = load_mask(q)
                return cache[q]

            for _ in range(a.crops):
                k = int(rng.choice(ks))
                r0 = int(rng.integers(0, hmax - a.size))
                c0 = int(rng.integers(0, w - a.size))
                win = (slice(r0, r0 + a.size), slice(c0, c0 + a.size))
                x = np.concatenate([rgb(paths[k + off + d])[win] for d in (-1, 0, 1)], -1)
                X.append(x.transpose(2, 0, 1))
                Y.append(rgb(ref_paths[k])[win].transpose(2, 0, 1))
                if a.mask:
                    M.append(np.stack([msk(paths[k + off + d])[win] for d in (-1, 0, 1)]))
            meta.append({"scene": scene, "run": os.path.basename(run), "offset": off, "crops": a.crops})
            print(f"  {scene}/{os.path.basename(run)}: offset {off}, {a.crops} crops", flush=True)
    extra = {"m": np.stack(M)} if a.mask else {}
    np.savez_compressed(a.out, x=np.stack(X), y=np.stack(Y), meta=json.dumps(meta), **extra)
    print(f"{len(X)} crops of {a.size}x{a.size} from {len(meta)} runs" + (" with masks" if a.mask else "")
          + f" -> {a.out}")


def load_crops(paths, mask=False):
    xs, ys, ms = [], [], []
    for p in paths:
        d = np.load(p)
        xs.append(d["x"])
        ys.append(d["y"])
        if mask:
            if "m" not in d:
                sys.exit(f"{p} has no damage masks: make it with `prepare --mask`")
            ms.append(d["m"])
    m = torch.from_numpy(np.concatenate(ms)) if mask else None
    return torch.from_numpy(np.concatenate(xs)), torch.from_numpy(np.concatenate(ys)), m


def make_input(x_rgb, m=None):
    """Model input from uint8 crops: Y/Cb/Cr / 255 of the 3 frames, then the
    3 masks / 255 when the model takes them."""
    x = rgb_to_ycc(x_rgb.float())
    if m is not None:
        x = torch.cat([x, m.float() / 255.0], 1)
    return x


def psnr_t(a, b):
    """Mean PSNR (dB, 0..255 scale) of Y/Cb/Cr/255 tensors."""
    mse = ((a - b) * 255.0).pow(2).mean(dim=(1, 2, 3)).clamp_min(1e-9)
    return (10 * torch.log10(255.0 ** 2 / mse)).mean().item()


def augment(x, y, g):
    """Flips only, no transpose: FM noise and clicks run along the scan line,
    so the picture's two axes are not interchangeable."""
    if torch.rand(1, generator=g).item() < 0.5:
        x, y = x.flip(-1), y.flip(-1)
    if torch.rand(1, generator=g).item() < 0.5:
        x, y = x.flip(-2), y.flip(-2)
    return x, y


def cmd_train(a):
    torch.set_num_threads(a.threads)
    torch.manual_seed(a.seed)
    g = torch.Generator().manual_seed(a.seed)
    xt, yt, mt = load_crops(a.train, a.mask)
    xv, yv, mv = load_crops(a.val, a.mask) if a.val else (None, None, None)
    print(f"train crops {len(xt)}" + (f", val crops {len(xv)}" if xv is not None else "")
          + (" (with damage masks)" if a.mask else ""))
    # --device auto: the GPU when torch sees one. The crops (uint8, a few
    # hundred MB) go to the device once, so no batch crosses the bus.
    dev = torch.device("cuda" if a.device == "auto" and torch.cuda.is_available()
                       else ("cpu" if a.device == "auto" else a.device))
    to = (lambda t: t.to(dev) if t is not None else None)
    xt, yt, mt, xv, yv, mv = map(to, (xt, yt, mt, xv, yv, mv))
    model = Denoiser(a.width, a.depth, a.mask).to(dev)
    params = sum(p.numel() for p in model.parameters())
    print(f"model: {a.depth} layers x {a.width} channels, {params} parameters, on {dev}"
          + (f" ({torch.cuda.get_device_name(dev)})" if dev.type == "cuda" else ""))
    # --amp: the convolutions in float16 on the GPU (tensor cores), the loss
    # and the weights in float32; the scaler keeps small gradients from
    # underflowing in float16. Off on the CPU (no gain there).
    amp = bool(a.amp and dev.type == "cuda")
    scaler = torch.amp.GradScaler("cuda", enabled=amp)
    opt = torch.optim.Adam(model.parameters(), lr=a.lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.iters)
    ckpt = a.out + ".ckpt"
    log, best, start = [], None, 1
    if a.resume and os.path.exists(ckpt):
        st = torch.load(ckpt, map_location="cpu", weights_only=False)
        if (st["width"], st["depth"], st["mask"], st["iters"]) != (a.width, a.depth, a.mask, a.iters):
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
        # Fine-tune: start from a trained model's weights (same shape; a mask
        # model can start from a maskless one: the mask planes' input weights
        # start at zero, so it begins as exactly that model).
        src = torch.load(os.path.expanduser(a.init), map_location="cpu", weights_only=False)["state"]
        own = model.state_dict()
        w0 = src["net.0.weight"]
        if own["net.0.weight"].shape != w0.shape:
            pad = torch.zeros_like(own["net.0.weight"])
            pad[:, :w0.shape[1]] = w0
            src["net.0.weight"] = pad
        model.load_state_dict(src)
        print(f"initialised from {a.init}")

    stop = {"sig": None}

    def on_signal(sig, _frame):
        stop["sig"] = sig  # finish this iteration, checkpoint, exit

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def save_ckpt(it):
        tmp = ckpt + ".tmp"
        torch.save({"it": it, "width": a.width, "depth": a.depth, "mask": a.mask, "iters": a.iters,
                    "state": model.state_dict(), "opt": opt.state_dict(), "sched": sched.state_dict(),
                    "gen": g.get_state(), "torch_rng": torch.get_rng_state(), "log": log, "best": best,
                    "scaler": scaler.state_dict() if amp else None}, tmp)
        os.replace(tmp, ckpt)  # never a half-written checkpoint, even if killed while saving

    def val_score():
        if xv is None:
            return None
        model.eval()
        raw, out = [], []
        with torch.no_grad():
            for i in range(0, len(xv), 64):
                x = make_input(xv[i:i + 64], mv[i:i + 64] if mv is not None else None)
                y = rgb_to_ycc(yv[i:i + 64].float())
                raw.append(psnr_t(x[:, 3:6], y))
                out.append(psnr_t(model(x), y))
        model.train()
        return float(np.mean(raw)), float(np.mean(out))

    t0, it0 = time.time(), start
    it = start - 1
    for it in range(start, a.iters + 1):
        idx = torch.randint(0, len(xt), (a.batch,), generator=g)
        x = make_input(xt[idx], mt[idx] if mt is not None else None)
        y = rgb_to_ycc(yt[idx].float())
        x, y = augment(x, y, g)
        with torch.autocast(device_type=dev.type, dtype=torch.float16, enabled=amp):
            pred = model(x)
        pred = pred.float()  # losses in float32 (the 1e-6 terms are near float16's floor)
        # Charbonnier (smooth L1): robust to the clicks' outliers in the targets.
        pix = torch.sqrt((pred - y).pow(2) + 1e-6).mean()
        loss = pix
        # Colour terms off for the first --colour-start share of the run (pixel
        # loss alone), then ramped in over 10 % of it.
        s0 = a.colour_start * a.iters
        ramp = 0.0 if it < s0 else min(1.0, (it - s0) / max(0.1 * a.iters, 1))
        if a.chroma_weight > 0:
            # L1 between the PxP average-pooled Cb/Cr of output and reference.
            col = (F.avg_pool2d(pred[:, 1:3], a.chroma_pool) - F.avg_pool2d(y[:, 1:3], a.chroma_pool)).abs().mean()
            loss = loss + ramp * a.chroma_weight * col
        if a.sat_weight > 0:
            # L1 between the pooled colour strength |C| of output and
            # reference (hue-blind).
            def strength(c):
                p = F.avg_pool2d(c[:, 1:3], a.chroma_pool)
                return torch.sqrt(p.pow(2).sum(1) + 1e-6)
            sat = (strength(pred) - strength(y)).abs().mean()
            loss = loss + ramp * a.sat_weight * sat
        opt.zero_grad()
        scaler.scale(loss).backward()  # plain backward/step when amp is off
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
            if a.chroma_weight > 0:
                entry["chroma"] = round(col.item(), 5)
            if a.sat_weight > 0:
                entry["sat"] = round(sat.item(), 5)
            if v:
                entry.update(val_raw_db=round(v[0], 2), val_cnn_db=round(v[1], 2))
            log.append(entry)
            print(json.dumps(entry), flush=True)
            # With colour terms, only fully ramped models count: they cost a
            # little PSNR, so earlier (greyer) ones would otherwise win.
            colour = a.chroma_weight > 0 or a.sat_weight > 0
            if v and (ramp >= 1.0 or not colour) and (best is None or v[1] > best):
                best = v[1]
                save(model, a, log, a.out)
        if it % a.checkpoint_every == 0 or stop["sig"] is not None:
            save_ckpt(it)
        if stop["sig"] is not None:
            print(f"signal {stop['sig']}: checkpoint at iteration {it} in {ckpt}; continue with --resume",
                  flush=True)
            sys.exit(130)
    save_ckpt(it)
    if xv is None:
        save(model, a, log, a.out)
    print(f"done in {time.time() - t0:.0f} s -> {a.out}" + (f" (best val {best:.2f} dB)" if best else ""))


def save(model, a, log, path):
    torch.save({"format": "openvrx-denoise-cnn", "version": 1, "width": model.width, "depth": model.depth,
                "mask": model.mask,
                "planes": "3 frames x Y/Cb/Cr (denoise_filter.to_ycc) / 255"
                          + (", then the 3 frames' damage masks / 255" if model.mask else "")
                          + "; output added to the middle frame",
                "state": model.state_dict(), "train": [os.path.basename(p) for p in a.train],
                "loss": {"pixel": "charbonnier", "chroma_weight": a.chroma_weight, "sat_weight": a.sat_weight,
                         "chroma_pool": a.chroma_pool},
                "iters": a.iters, "seed": a.seed,
                "val": [os.path.basename(p) for p in (a.val or [])], "log": log}, path)


def load_model(path):
    ck = torch.load(os.path.expanduser(path), map_location="cpu", weights_only=False)
    m = Denoiser(ck["width"], ck["depth"], ck.get("mask", False))
    m.load_state_dict(ck["state"])
    m.eval()
    return m


def run_model(model, rgb_frames, k, masks=None):
    """Denoised frame k (needs k-1 and k+1) as Y/Cb/Cr (3, H, W) numpy, the
    scale of denoise_filter.to_ycc. masks: the frames' damage masks (same
    indexing), needed when the model takes them."""
    x = np.concatenate([rgb_frames[k + d] for d in (-1, 0, 1)], -1).transpose(2, 0, 1)
    m = None
    if model.mask:
        if masks is None:
            raise ValueError("this model needs the decoder's damage masks (decode with --damage)")
        m = torch.from_numpy(np.stack([masks[k + d] for d in (-1, 0, 1)]))[None]
    with torch.no_grad():
        y = model(make_input(torch.from_numpy(x)[None], m))[0]
    return (y * 255.0).numpy()


def saturation(frames_ycc, block=8):
    """Colour strength of frames (Y/Cb/Cr, 0..255 scale) over the scored
    area of lin.score: mean |Cb|+|Cr| of BLOCKxBLOCK block means ("sat",
    the picture's colour; per-pixel noise mostly averages out, so a noisy
    decode does not count as colourful) and of single pixels ("sat_px",
    colour plus chroma noise). Compare with the reference's: a ratio under 1
    is a pull toward grey."""
    s, sp = [], []
    for x in frames_ycc:
        h = x.shape[1]
        c = x[1:, 8:int(h * lin.OSD_SHARE), 8:-8]
        hh, ww = (c.shape[1] // block) * block, (c.shape[2] // block) * block
        b = c[:, :hh, :ww].reshape(2, hh // block, block, ww // block, block).mean(axis=(2, 4))
        s.append(np.abs(b).sum(0).mean())
        sp.append(np.abs(c).sum(0).mean())
    return float(np.mean(s)), float(np.mean(sp))


def model_name(path):
    return os.path.splitext(os.path.basename(path))[0]


def cmd_eval(a):
    torch.set_num_threads(a.threads)
    models = {model_name(p): load_model(p) for p in a.model}
    need_mask = any(m.mask for m in models.values())
    coef = lin.load_filter(a.filter, symmetric=True) if a.filter else None
    sd = os.path.join(a.data, a.scene)
    if is_bench(sd):
        lin.OSD_SHARE = 1.0  # no OSD in bench patterns: score the whole picture
    ref_dir, runs = lin.pairs(sd)
    if a.runs:
        runs = [r for r in runs if os.path.basename(r) in a.runs]
    ref_all = [lin.to_ycc(p) for p in lin.frames(ref_dir)]
    res = {}
    for run in runs:
        paths = lin.frames(run)
        if len(paths) < 3:
            print(f"{os.path.basename(run):>8}  only {len(paths)} frames decoded, skipped")
            continue
        rgb = [load_rgb(p) for p in paths]
        masks = [load_mask(os.path.join(mask_dir_of(run), os.path.basename(p))) for p in paths] if need_mask else None
        seq = [lin.to_ycc(p) for p in paths]
        off = lin.align(ref_all, seq)
        # Odd step: the scored fields alternate parity (an even step, as up
        # to 2026-11-08, scored only top or only bottom fields).
        step = a.every if a.every % 2 else a.every + 1
        ks = [k for k in range(len(ref_all)) if 1 <= k + off < len(seq) - 1][::step]
        refs = [ref_all[k] for k in ks]
        rs, rsp = saturation(refs)
        outs = {"raw": [seq[k + off] for k in ks]}
        if coef is not None:
            # Only the scored frames, each from its 3-frame window (filtering
            # whole runs costs far more than the CNN itself).
            outs["linear"] = [lin.filter_seq(seq[k + off - 1:k + off + 2], coef)[1] for k in ks]
        for name, m in models.items():
            outs[name] = [run_model(m, rgb, k + off, masks) for k in ks]
        # Score through lin.score on short aligned sequences: [prev, frame, next]
        # around each picked frame, so its offset search lands on 0.
        r = {"ref_sat": round(rs, 3), "ref_sat_px": round(rsp, 3)}
        for name, fr in outs.items():
            sc = lin.score(refs, fr)
            s, sp = saturation(fr)
            sc.update(sat=round(s, 3), sat_ratio=round(s / max(rs, 1e-6), 3), sat_px=round(sp, 3))
            r[name] = sc
        res[os.path.basename(run)] = r
        print(f"{os.path.basename(run):>8}  " + "   ".join(
            f"{n} {r[n]['rgb']:5.2f} dB sat {r[n]['sat_ratio']:.2f}" for n in outs), flush=True)
    kinds = ["raw"] + (["linear"] if coef is not None else []) + list(models)
    cns = sorted(res, key=lambda n: float("".join(ch for ch in n.split("-")[0] if ch.isdigit() or ch == ".") or 0))
    print("\nRGB PSNR dB (sat = colour of 8x8 block means vs the reference, 1.00 = same)")
    print(f"{'run':>8} " + " ".join(f"{k:>16}" for k in kinds))
    for n in cns:
        print(f"{n:>8} " + " ".join(f"{res[n][k]['rgb']:7.2f} sat {res[n][k]['sat_ratio']:4.2f}" for k in kinds))
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"scene": a.scene, "models": a.model, "filter": a.filter, "every": a.every,
                       "psnr_db": res}, f, indent=1)
        print(f"-> {a.out}")


def cmd_apply(a):
    torch.set_num_threads(a.threads)
    model = load_model(a.model)
    paths = lin.frames(a.frames)
    rgb = [load_rgb(p) for p in paths]
    masks = [load_mask(os.path.join(a.damage, os.path.basename(p))) for p in paths] if model.mask else None
    os.makedirs(a.out, exist_ok=True)
    for k, p in enumerate(paths):
        y = run_model(model, rgb, k, masks) if 1 <= k < len(paths) - 1 else lin.to_ycc(p)
        Image.fromarray(lin.to_rgb(y)).save(os.path.join(a.out, os.path.basename(p)))
    print(f"{len(paths)} frames -> {a.out}")


def cmd_render(a):
    """Whole recording -> MP4 through the model, field by field. Fields are
    decoded to PNG first (openvrx-cli decode, the raw preset the models are
    trained on), then streamed: only 3 fields are in memory at a time."""
    torch.set_num_threads(a.threads)
    model = load_model(a.model)
    coef = lin.load_filter(a.filter, symmetric=True) if a.filter else None
    if "linear" in a.panels and coef is None:
        sys.exit("--panels linear needs --filter")
    work = a.work or (os.path.splitext(a.out)[0] + "-work")
    dec, dmg = os.path.join(work, "dec"), os.path.join(work, "damage")
    n = int(a.seconds * 60000 / 1001) + 2 if a.seconds else 10 ** 7
    t0 = time.time()
    if not (a.reuse_decode and os.path.isdir(dec) and lin.frames(dec)):
        shutil.rmtree(work, ignore_errors=True)
        os.makedirs(work)
        print(f"decoding {a.rec} ...", flush=True)
        subprocess.run([a.cli, "decode", a.rec, dec, str(n)] + (["--damage", dmg] if model.mask else []),
                       check=True, stdout=subprocess.DEVNULL)
    paths = lin.frames(dec)
    if len(paths) < 3:
        sys.exit(f"only {len(paths)} fields decoded")
    h, w = load_rgb(paths[0]).shape[:2]
    print(f"{len(paths)} fields ({len(paths) * 1001 / 60000:.1f} s) decoded in {time.time() - t0:.0f} s; "
          f"panels {','.join(a.panels)}", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    W = w * len(a.panels)
    # libx264 where ffmpeg has it (Windows builds); Fedora's ffmpeg-free does
    # not, so VP9 there (MP4 holds VP9 fine; mpv and Firefox play it). The
    # CRF numbers differ per codec: x264 18 and VP9 24 both keep the noise
    # being compared visible.
    encoders = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True).stdout
    if " libx264 " in encoders:
        codec = ["-c:v", "libx264", "-crf", str(a.crf if a.crf is not None else 18), "-preset", a.x264_preset]
    else:
        codec = ["-c:v", "libvpx-vp9", "-crf", str(a.crf if a.crf is not None else 24), "-b:v", "0",
                 "-deadline", "good", "-cpu-used", "4", "-row-mt", "1"]
    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{h}",
                           "-r", "60000/1001", "-i", "-", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2"] + codec
                          + ["-pix_fmt", "yuv420p", a.out], stdin=subprocess.PIPE)
    rgb, msk = {}, {}
    t1 = time.time()
    for j in range(len(paths)):
        for q in (j - 1, j, j + 1):
            if 0 <= q < len(paths) and q not in rgb:
                rgb[q] = load_rgb(paths[q])
                if model.mask:
                    msk[q] = load_mask(os.path.join(dmg, os.path.basename(paths[q])))
        rgb.pop(j - 2, None)
        msk.pop(j - 2, None)
        inner = 1 <= j < len(paths) - 1  # the first and last field lack a neighbour: shown raw
        tiles = []
        for p in a.panels:
            if p == "raw" or not inner:
                tiles.append(rgb[j])
            elif p == "linear":
                seq = [lin.to_ycc(paths[q]) for q in (j - 1, j, j + 1)]
                tiles.append(lin.to_rgb(lin.filter_seq(seq, coef)[1]))
            else:
                tiles.append(lin.to_rgb(run_model(model, rgb, j, msk if model.mask else None)))
        ff.stdin.write(np.concatenate(tiles, 1).tobytes())
        if j % 60 == 0 or j == len(paths) - 1:
            el = time.time() - t1
            eta = el / (j + 1) * (len(paths) - j - 1)
            print(f"  field {j + 1}/{len(paths)}  {el / (j + 1):.2f} s/field  eta {eta / 60:.1f} min", flush=True)
    ff.stdin.close()
    if ff.wait() != 0:
        sys.exit("ffmpeg failed")
    if not a.keep_work:
        shutil.rmtree(work, ignore_errors=True)
    print(f"-> {a.out} ({len(paths)} fields, {(time.time() - t0) / 60:.1f} min)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    csv = lambda v: [x for x in v.split(",") if x]  # noqa: E731
    p = sub.add_parser("prepare")
    p.add_argument("--data", required=True)
    p.add_argument("--scenes", type=csv, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--crops", type=int, default=48, help="crops per noisy run")
    p.add_argument("--size", type=int, default=64)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--mask", action="store_true", help="also store the damage masks (SCENE/damage/RUN)")
    p.add_argument("--skip-unmasked", action="store_true", help="with --mask: leave out runs that have no masks")
    t = sub.add_parser("train")
    t.add_argument("--train", type=csv, required=True, help="crop files from prepare")
    t.add_argument("--val", type=csv, default=None, help="crop files of held-out scenes")
    t.add_argument("--out", required=True)
    t.add_argument("--iters", type=int, default=3000)
    t.add_argument("--batch", type=int, default=16)
    t.add_argument("--width", type=int, default=32)
    t.add_argument("--depth", type=int, default=8)
    t.add_argument("--lr", type=float, default=1e-3)
    t.add_argument("--chroma-weight", type=float, default=0.0,
                   help="weight of the low-pass Cb/Cr L1 term (0 = v1, pixel loss only)")
    t.add_argument("--sat-weight", type=float, default=0.0,
                   help="weight of the hue-blind colour strength term: L1 of pooled |Cb,Cr| (v2b)")
    t.add_argument("--chroma-pool", type=int, default=8, help="block size of the colour terms' average pool")
    t.add_argument("--colour-start", type=float, default=0.0,
                   help="share of the iterations before the colour terms start (then ramped in over 10 %%)")
    t.add_argument("--mask", action="store_true", help="damage masks as 3 more input planes (v3)")
    t.add_argument("--log-every", type=int, default=100)
    t.add_argument("--device", default="auto", help="auto (GPU if any), cpu, cuda, cuda:1")
    t.add_argument("--amp", action="store_true", help="float16 convolutions on a GPU (ignored on the CPU)")
    t.add_argument("--checkpoint-every", type=int, default=500, help="iterations between OUT.ckpt saves")
    t.add_argument("--resume", action="store_true", help="continue from OUT.ckpt when it exists")
    t.add_argument("--init", default=None,
                   help="start from this model's weights (fine-tune; same width/depth, mask may be added)")
    t.add_argument("--threads", type=int, default=4)
    t.add_argument("--seed", type=int, default=1)
    e = sub.add_parser("eval")
    e.add_argument("--data", required=True)
    e.add_argument("--scene", required=True)
    e.add_argument("--model", type=csv, required=True, help="one or more models, comma separated")
    e.add_argument("--filter", default=None, help="linear filter JSON to compare with")
    e.add_argument("--runs", type=csv, default=None, help="run names to score (default: all)")
    e.add_argument("--every", type=int, default=8, help="score every Nth frame (the CNN is slow on CPU)")
    e.add_argument("--threads", type=int, default=4)
    e.add_argument("--out")
    x = sub.add_parser("apply")
    x.add_argument("--model", required=True)
    x.add_argument("--frames", required=True)
    x.add_argument("--out", required=True)
    x.add_argument("--damage", default=None, help="damage masks of the frames (models with masks)")
    x.add_argument("--threads", type=int, default=4)
    r = sub.add_parser("render")
    r.add_argument("--rec", required=True, help="recording base path (REC.rx0.bin, REC.openvrx.json)")
    r.add_argument("--model", required=True)
    r.add_argument("--out", required=True, help="output .mp4")
    r.add_argument("--panels", type=csv, default=["model"],
                   help="side by side, left to right, from raw, linear, model (default: model)")
    r.add_argument("--filter", default=None, help="linear filter JSON for the linear panel")
    r.add_argument("--seconds", type=float, default=None, help="only the first N seconds (default: all)")
    r.add_argument("--crf", type=int, default=None, help="quality (default: 18 with x264, 24 with VP9)")
    r.add_argument("--x264-preset", default="medium")
    r.add_argument("--work", default=None, help="folder for the decoded fields (default: OUT-work, removed after)")
    r.add_argument("--keep-work", action="store_true", help="keep the decoded fields")
    r.add_argument("--reuse-decode", action="store_true", help="use the fields already in --work")
    r.add_argument("--cli", default=CLI)
    r.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()
    {"prepare": cmd_prepare, "train": cmd_train, "eval": cmd_eval, "apply": cmd_apply,
     "render": cmd_render}[a.cmd](a)


if __name__ == "__main__":
    main()
