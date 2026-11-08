#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Learned post-flight denoise filter for `openvrx-cli render`.

A linear spatio-temporal filter (5x5 pixels x 3 frames over Y, Cb and Cr,
one output per plane) fitted by least squares on simulated recordings: the
simulator turns a clean clip into 4-bit I/Q at a given C/N, `openvrx-cli
decode` turns that back into frames, and the filter learns to map the noisy
frames onto a reference (the average of several 20 dB runs with different
noise seeds; a near-noiseless run is no reference with 4-bit lanes, whose
undithered quantisation draws rainbow stripes).

    denoise_filter.py simulate --clip VIDEO --scene NAME --data DIR
                               [--cn 9,12,16,20] [--ref-runs 8] [--seed 41] [--jobs 4]
                               [--damage] [--keep-ref] [--steady-osd]
    denoise_filter.py train    --data DIR --scenes A,B --out FILTER.json
    denoise_filter.py eval     --data DIR --scene NAME --filter FILTER.json [--out EVAL.json]
    denoise_filter.py apply    --filter FILTER.json --frames DIR --out DIR

VIDEO is any clip ffmpeg reads (about 2 s of real FPV footage works well;
decoded OpenVRX frames of a clean stretch are fine). DIR/NAME/ref holds the
reference frames, DIR/NAME/cnXX the decoded noisy runs. `eval` prints and
writes PSNR (luma, chroma, RGB) per C/N for the raw decode and the filter.
The OSD rows (bottom 30 %) are left out of training and scoring.

Simulator settings follow the 2026-11-06 flight: 4x4top lanes, clock divider
9, deviation 0.69, narrow channel filter, clean sampling phase, no I/Q DC
offset (the C5 DC servo), no fading. Tools: openvrx-sim and openvrx-cli from
host/target/release (or --bin DIR). Needs numpy and Pillow.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image

RADIUS = 2  # spatial radius (5x5)
TIME = 1  # temporal radius (3 frames) of the symmetric filter; also the frames scoring skips
# Frames the filter reads, relative to the one it outputs: the post-flight
# filter looks one frame ahead; a live filter must not (it would add a frame
# of latency), so `train --offsets -2,-1,0` fits a causal one.
OFFSETS = (-1, 0, 1)
OSD_SHARE = 0.7  # rows below this share of the height hold the OSD
EXE = '.exe' if os.name == 'nt' else ''
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def frame_key(name):
    """Sort key by the number in the name: `openvrx-cli decode` writes
    -frame00 .. -frame99, -frame100 ..., which a plain sort puts as frame10,
    frame100, frame101, ... frame11 (scrambling every decode over 100 fields)."""
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', name)]


def frames(d):
    return [os.path.join(d, f) for f in sorted(os.listdir(d), key=frame_key) if f.endswith('.png')]


def to_ycc(path):
    a = np.asarray(Image.open(path).convert('RGB'), dtype=np.float32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return np.stack([y, 0.564 * (b - y), 0.713 * (r - y)])


def to_rgb(x):
    y, cb, cr = x
    r = y + cr / 0.713
    b = y + cb / 0.564
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return np.clip(np.stack([r, g, b], -1) + 0.5, 0, 255).astype(np.uint8)


def window_ok(k, n, offs):
    """Frame k of n has every frame the filter reads."""
    return 0 <= k + min(offs) and k + max(offs) < n


def features(seq, k, rows, cols, offs=OFFSETS):
    """Neighbourhood of pixels (rows, cols) in frame k: for each frame
    offset, plane, dy and dx (in that order), then a constant 1. The Rust
    filter in openvrx-cli uses the same order."""
    f = []
    for dt in offs:
        fr = seq[k + dt]
        for c in range(3):
            pl = fr[c]
            for dy in range(-RADIUS, RADIUS + 1):
                for dx in range(-RADIUS, RADIUS + 1):
                    f.append(pl[rows + dy, cols + dx])
    f.append(np.ones(len(rows), dtype=np.float32))
    return np.stack(f, 1)


def align(ref, seq):
    """Frame offset of seq against ref (decodes lock a frame or so apart)."""
    best = None
    for off in range(-8, 9):
        ks = [k for k in range(len(ref) // 4, 3 * len(ref) // 4, 7) if TIME <= k + off < len(seq) - TIME]
        if not ks:
            continue
        e = np.mean([np.mean((ref[k][0] - seq[k + off][0]) ** 2) for k in ks])
        if best is None or e < best[0]:
            best = (e, off)
    return best[1] if best else 0  # too few frames to compare: no shift


def psnr(a, b):
    return 10 * np.log10(255.0 ** 2 / max(float(np.mean((a - b) ** 2)), 1e-9))


# ---------------------------------------------------------------------------


def run_sim(bin_dir, cfg, clip, out_base, cn, seed, damage=False, steady=False):
    sim = os.path.join(bin_dir, 'openvrx-sim' + EXE)
    cli = os.path.join(bin_dir, 'openvrx-cli' + EXE)
    subprocess.run([sim, '--config', cfg, '-o', out_base, '--seconds', '1.6', '--standard', 'ntsc',
                    '--preset', '4x4top', '--divider', '9', '--cn', str(cn), '--lottery', 'clean',
                    '--picture', 'video:' + clip, '--deviation', '0.69', '--p11', '--fading', 'none',
                    '--overwrite', '--seed', str(seed)], check=True, capture_output=True)
    dec = out_base + '-dec'
    shutil.rmtree(dec, ignore_errors=True)
    shutil.rmtree(dec + '-damage', ignore_errors=True)
    extra = (['--damage', dec + '-damage'] if damage else []) + (['--steady-osd'] if steady else [])
    subprocess.run([cli, 'decode', out_base, dec, '95'] + extra, check=True, capture_output=True)
    for ext in ('.rx0.bin', '.truth.f32'):
        if os.path.exists(out_base + ext):
            os.remove(out_base + ext)
    return dec


def cmd_simulate(a):
    bin_dir = a.bin or os.path.join(REPO, 'host', 'target', 'release')
    scene_dir = os.path.join(a.data, a.scene)
    work = os.path.join(scene_dir, 'work')
    os.makedirs(work, exist_ok=True)
    # Simulator config without the I/Q DC offset model.
    sim = os.path.join(bin_dir, 'openvrx-sim' + EXE)
    base = subprocess.run([sim, '--preset', '4x4top', '--dump-config'], check=True, capture_output=True, text=True).stdout
    conf = json.loads(base)
    for rx in conf.get('receivers', []):
        rx['dc_scale'] = 0.0
    cfg = os.path.join(work, 'config.json')
    with open(cfg, 'w') as f:
        json.dump(conf, f)
    clip = os.path.abspath(a.clip)
    out = os.path.join(scene_dir, 'ref')
    keep_ref = a.keep_ref and os.path.isdir(out) and frames(out)
    jobs = [] if keep_ref else [('ref', 20, 100 + i) for i in range(a.ref_runs)]
    jobs += [(f'cn{cn}', cn, a.seed) for cn in a.cn]
    with ThreadPoolExecutor(a.jobs) as ex:
        decs = list(ex.map(lambda j: (j[0], run_sim(bin_dir, cfg, clip, os.path.join(work, f'{j[0]}-{j[2]}'), j[1], j[2],
                                                    damage=a.damage and j[0] != 'ref', steady=a.steady_osd)), jobs))
    refs = [[to_ycc(p) for p in frames(d)] for name, d in decs if name == 'ref']
    if refs:
        # Reference: the aligned mean of the ref runs.
        offs = [0] + [align(refs[0], s) for s in refs[1:]]
        shutil.rmtree(out, ignore_errors=True)
        os.makedirs(out)
        lo = max(0, -min(offs)) + TIME
        hi = min(len(s) - o for s, o in zip(refs, offs)) - TIME
        for k in range(lo, hi):
            m = sum(s[k + o] for s, o in zip(refs, offs)) / len(refs)
            Image.fromarray(to_rgb(m)).save(os.path.join(out, f'f{k:04d}.png'))
        for name, d in decs:
            if name == 'ref':
                shutil.rmtree(d, ignore_errors=True)
    for name, d in decs:
        if name != 'ref':
            dst = os.path.join(scene_dir, name)
            shutil.rmtree(dst, ignore_errors=True)
            shutil.move(d, dst)
            # Damage masks (same file names) in SCENE/damage/RUN, outside the
            # cn* pattern that pairs() lists as runs.
            mdst = os.path.join(scene_dir, 'damage', name)
            shutil.rmtree(mdst, ignore_errors=True)
            if os.path.isdir(d + '-damage'):
                os.makedirs(os.path.dirname(mdst), exist_ok=True)
                shutil.move(d + '-damage', mdst)
    print(f'{a.scene}: ' + (f'reference from {len(refs)} runs (offsets {offs}), ' if refs else 'reference kept, ')
          + f'noisy runs {[n for n, _ in decs if n != "ref"]}' + (' with damage masks' if a.damage else ''))


def pairs(scene_dir):
    """(ref frames, [noisy run dirs]) of a scene."""
    runs = sorted(d for d in os.listdir(scene_dir) if d.startswith('cn'))
    return os.path.join(scene_dir, 'ref'), [os.path.join(scene_dir, d) for d in runs]


def cmd_train(a):
    rng = np.random.default_rng(0)
    offs = a.offsets
    if list(offs) != list(range(offs[0], offs[-1] + 1)) or not offs[0] <= 0 <= offs[-1]:
        raise SystemExit('--offsets: consecutive frames that include 0, for example -1,0,1 or -2,-1,0')
    A, B = [], []
    for scene in a.scenes:
        ref_dir, runs = pairs(os.path.join(a.data, scene))
        if a.runs:
            runs = [r for r in runs if os.path.basename(r) in a.runs]
        ref = [to_ycc(p) for p in frames(ref_dir)]
        h, w = ref[0].shape[1:]
        for run in runs:
            seq = [to_ycc(p) for p in frames(run)]
            off = align(ref, seq)
            # Every frame: fields alternate parity, so a step of 2 (as up to
            # 2026-11-08) fitted on one parity only.
            for k in range(len(ref)):
                if not window_ok(k + off, len(seq), offs):
                    continue
                rows = rng.integers(RADIUS + 8, int(h * OSD_SHARE), a.samples)
                cols = rng.integers(RADIUS + 8, w - RADIUS - 8, a.samples)
                A.append(features(seq, k + off, rows, cols, offs))
                B.append(ref[k][:, rows, cols].T)
    A = np.concatenate(A).astype(np.float64)
    B = np.concatenate(B).astype(np.float64)
    # A little ridge keeps the taps tame where planes are nearly collinear.
    n = A.shape[1]
    G = A.T @ A
    G += 1e-3 * np.trace(G) / n * np.eye(n)
    # Flat colours pass unchanged: per output plane the taps on that plane
    # sum to 1, the taps on the other planes to 0, and there is no bias.
    # Without this the fit pulls noisy chroma towards grey (MSE shrinkage),
    # which shows as washed-out colour.
    per_plane = (2 * RADIUS + 1) ** 2
    plane_of = np.array([(t // per_plane) % 3 for t in range(n - 1)] + [-1])
    C = np.zeros((4, n))
    for c in range(3):
        C[c, plane_of == c] = 1.0
    C[3, n - 1] = 1.0
    coef = np.zeros((n, 3))
    for o in range(3):
        d = np.zeros(4)
        d[o] = 1.0
        kkt = np.block([[G, C.T], [C, np.zeros((4, 4))]])
        coef[:, o] = np.linalg.solve(kkt, np.concatenate([A.T @ B[:, o], d]))[:n]
    # Version 1 (symmetric, 'time') is what every openvrx-cli reads; version
    # 2 lists the frame offsets instead, for the causal live filter.
    sym = list(offs) == list(range(-offs[-1], offs[-1] + 1))
    out = {
        'format': 'openvrx-denoise-filter',
        'version': 1 if sym else 2,
        'radius': RADIUS,
        **({'time': offs[-1]} if sym else {'offsets': list(offs)}),
        'planes': 'ycbcr (BT.601 from RGB8: y = .299 r + .587 g + .114 b, cb = .564 (b - y), cr = .713 (r - y))',
        'order': 'per output plane: for dt (each frame offset, oldest first), plane, dy, dx (each from -r to r), '
                 'then bias',
        'trained_on': {'scenes': a.scenes, 'runs': a.runs or 'all', 'pixels': int(A.shape[0])},
        'coef': [coef[:, j].astype(float).round(7).tolist() for j in range(3)],
    }
    with open(a.out, 'w') as f:
        json.dump(out, f)
    print(f'trained on {A.shape[0]} pixels of {", ".join(a.scenes)} -> {a.out}')


def load_filter(path, symmetric=False):
    """(coef as taps x 3, frame offsets) of a filter file (version 1 or 2).
    symmetric=True: only the coef, for callers that pass [k-1, k, k+1]
    windows (the comparison panels); a causal filter is refused there."""
    with open(path) as f:
        flt = json.load(f)
    assert flt['radius'] == RADIUS
    offs = tuple(flt['offsets']) if 'offsets' in flt else tuple(range(-flt['time'], flt['time'] + 1))
    coef = np.array(flt['coef'], dtype=np.float32).T
    if symmetric:
        if offs != OFFSETS:
            raise SystemExit(f'{path}: frames {offs}, this tool needs a {OFFSETS} filter')
        return coef
    return coef, offs


def filter_seq(seq, coef, offs=OFFSETS):
    """Every frame through the filter; frames without the whole window are
    kept as they are."""
    h, w = seq[0].shape[1:]
    rr, cc = np.mgrid[RADIUS:h - RADIUS, RADIUS:w - RADIUS]
    rr, cc = rr.ravel(), cc.ravel()
    out = []
    for k in range(len(seq)):
        y = seq[k].copy()
        if window_ok(k, len(seq), offs):
            y[:, rr, cc] = (features(seq, k, rr, cc, offs) @ coef).T
        out.append(y)
    return out


def score(ref, seq, margin=TIME):
    """PSNR of seq against ref over the scored area, skipping `margin` frames
    at both ends of seq (filters leave those unfiltered; compare filters with
    different windows at the same margin)."""
    off = align(ref, seq)
    h = ref[0].shape[1]
    cut = (slice(None), slice(8, int(h * OSD_SHARE)), slice(8, -8))
    ks = [k for k in range(len(ref)) if margin <= k + off < len(seq) - margin]
    r = lambda x: x[cut]
    s = {'y': np.mean([psnr(r(ref[k])[0], r(seq[k + off])[0]) for k in ks]),
         'c': np.mean([psnr(r(ref[k])[1:], r(seq[k + off])[1:]) for k in ks]),
         'rgb': np.mean([psnr(to_rgb(r(ref[k])).astype(np.float32), to_rgb(r(seq[k + off])).astype(np.float32)) for k in ks])}
    return {k: round(float(v), 2) for k, v in s.items()}


def filter_frame(seq, k, coef, offs):
    """Frame k of seq through the filter (border pixels kept)."""
    h, w = seq[k].shape[1:]
    rr, cc = np.mgrid[RADIUS:h - RADIUS, RADIUS:w - RADIUS]
    rr, cc = rr.ravel(), cc.ravel()
    y = seq[k].copy()
    y[:, rr, cc] = (features(seq, k, rr, cc, offs) @ coef).T
    return y


def score_pairs(refs, outs):
    """score() on frames already paired (no alignment search)."""
    h = refs[0].shape[1]
    cut = (slice(None), slice(8, int(h * OSD_SHARE)), slice(8, -8))
    s = {'y': np.mean([psnr(r[cut][0], o[cut][0]) for r, o in zip(refs, outs)]),
         'c': np.mean([psnr(r[cut][1:], o[cut][1:]) for r, o in zip(refs, outs)]),
         'rgb': np.mean([psnr(to_rgb(r[cut]).astype(np.float32), to_rgb(o[cut]).astype(np.float32))
                         for r, o in zip(refs, outs)])}
    return {k: round(float(v), 2) for k, v in s.items()}


def cmd_eval(a):
    """Every filter of --filter (comma separated) on every noisy run of the
    scene, all scored on the same frames: every --every-th frame with the
    whole window of the widest filter."""
    flts = [(os.path.splitext(os.path.basename(p))[0], *load_filter(p)) for p in a.filter.split(',')]
    margin = max(max(abs(o) for o in offs) for _, _, offs in flts)
    sd = os.path.join(a.data, a.scene)
    global OSD_SHARE
    if os.path.exists(os.path.join(sd, 'build.json')):
        OSD_SHARE = 1.0  # bench patterns have no OSD: score the whole picture
    ref_dir, runs = pairs(sd)
    if a.runs:
        runs = [r for r in runs if os.path.basename(r) in a.runs]
    ref = [to_ycc(p) for p in frames(ref_dir)]
    res = {}
    for run in runs:
        seq = [to_ycc(p) for p in frames(run)]
        if len(seq) < 2 * margin + 1:
            continue
        off = align(ref, seq)
        # An odd step alternates the scored fields' parity (an even one
        # scores only top or only bottom fields).
        step = a.every if a.every % 2 else a.every + 1
        ks = [k for k in range(len(ref)) if margin <= k + off < len(seq) - margin][::step]
        refs = [ref[k] for k in ks]
        r = {'raw': score_pairs(refs, [seq[k + off] for k in ks])}
        for name, coef, offs in flts:
            r[name] = score_pairs(refs, [filter_frame(seq, k + off, coef, offs) for k in ks])
        res[os.path.basename(run)] = r
        print(f"{os.path.basename(run):>15}  raw RGB {r['raw']['rgb']:5.2f}"
              + "".join(f"   {n} RGB {r[n]['rgb']:5.2f} ({r[n]['rgb'] - r['raw']['rgb']:+.2f})" for n, _, _ in flts),
              flush=True)
    if a.out:
        with open(a.out, 'w') as f:
            json.dump({'scene': a.scene, 'filter': a.filter, 'margin': margin, 'psnr_db': res}, f, indent=1)


def cmd_apply(a):
    coef, offs = load_filter(a.filter)
    paths = frames(a.frames)
    seq = [to_ycc(p) for p in paths]
    os.makedirs(a.out, exist_ok=True)
    for p, y in zip(paths, filter_seq(seq, coef, offs)):
        Image.fromarray(to_rgb(y)).save(os.path.join(a.out, os.path.basename(p)))
    print(f'{len(paths)} frames -> {a.out}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('simulate')
    s.add_argument('--clip', required=True)
    s.add_argument('--scene', required=True)
    s.add_argument('--data', required=True)
    s.add_argument('--cn', type=lambda v: [int(x) for x in v.split(',')], default=[9, 12, 16, 20])
    s.add_argument('--ref-runs', type=int, default=8)
    s.add_argument('--seed', type=int, default=41)
    s.add_argument('--jobs', type=int, default=4)
    s.add_argument('--bin')
    s.add_argument('--damage', action='store_true',
                   help='also write each noisy run\'s decoder damage masks to SCENE/damage/RUN')
    s.add_argument('--keep-ref', action='store_true',
                   help='keep an existing SCENE/ref (only add or redo the noisy runs)')
    s.add_argument('--steady-osd', action='store_true',
                   help="decode as the live app's default view (line-doubled fields on the same rows); "
                        'use a separate --scene name: these frames sit half a line from the normal ones')
    t = sub.add_parser('train')
    t.add_argument('--data', required=True)
    t.add_argument('--scenes', type=lambda v: v.split(','), required=True)
    t.add_argument('--out', required=True)
    t.add_argument('--samples', type=int, default=2000, help='pixels per frame pair (every frame is used)')
    t.add_argument('--runs', type=lambda v: v.split(','),
                   help='only these noisy runs of each scene (for example cn8,cn10,cn12,cn14,cn16,cn20)')
    t.add_argument('--offsets', type=lambda v: tuple(int(x) for x in v.split(',')), default=OFFSETS,
                   help='frames read, relative to the output one (default -1,0,1; live, no added latency: -2,-1,0)')
    e = sub.add_parser('eval')
    e.add_argument('--data', required=True)
    e.add_argument('--scene', required=True)
    e.add_argument('--filter', required=True, help='one or more filter files, comma separated')
    e.add_argument('--every', type=int, default=5,
                   help='score every N-th frame (1: all; an even N is raised by one so both field parities count)')
    e.add_argument('--runs', type=lambda v: v.split(','), help='only these runs (default: all)')
    e.add_argument('--out')
    ap = sub.add_parser('apply')
    ap.add_argument('--filter', required=True)
    ap.add_argument('--frames', required=True)
    ap.add_argument('--out', required=True)
    a = p.parse_args()
    {'simulate': cmd_simulate, 'train': cmd_train, 'eval': cmd_eval, 'apply': cmd_apply}[a.cmd](a)


if __name__ == '__main__':
    main()
