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
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image

RADIUS = 2  # spatial radius (5x5)
TIME = 1  # temporal radius (3 frames)
OSD_SHARE = 0.7  # rows below this share of the height hold the OSD
EXE = '.exe' if os.name == 'nt' else ''
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def frames(d):
    return [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.endswith('.png')]


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


def features(seq, k, rows, cols):
    """Neighbourhood of pixels (rows, cols) in frame k: for each frame
    offset, plane, dy and dx (in that order), then a constant 1. The Rust
    filter in openvrx-cli uses the same order."""
    f = []
    for dt in range(-TIME, TIME + 1):
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
    return best[1]


def psnr(a, b):
    return 10 * np.log10(255.0 ** 2 / max(float(np.mean((a - b) ** 2)), 1e-9))


# ---------------------------------------------------------------------------


def run_sim(bin_dir, cfg, clip, out_base, cn, seed):
    sim = os.path.join(bin_dir, 'openvrx-sim' + EXE)
    cli = os.path.join(bin_dir, 'openvrx-cli' + EXE)
    subprocess.run([sim, '--config', cfg, '-o', out_base, '--seconds', '1.6', '--standard', 'ntsc',
                    '--preset', '4x4top', '--divider', '9', '--cn', str(cn), '--lottery', 'clean',
                    '--picture', 'video:' + clip, '--deviation', '0.69', '--p11', '--fading', 'none',
                    '--overwrite', '--seed', str(seed)], check=True, capture_output=True)
    dec = out_base + '-dec'
    shutil.rmtree(dec, ignore_errors=True)
    subprocess.run([cli, 'decode', out_base, dec, '95'], check=True, capture_output=True)
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
    jobs = [('ref', 20, 100 + i) for i in range(a.ref_runs)]
    jobs += [(f'cn{cn}', cn, a.seed) for cn in a.cn]
    with ThreadPoolExecutor(a.jobs) as ex:
        decs = list(ex.map(lambda j: (j[0], run_sim(bin_dir, cfg, clip, os.path.join(work, f'{j[0]}-{j[2]}'), j[1], j[2])), jobs))
    # Reference: the aligned mean of the ref runs.
    refs = [[to_ycc(p) for p in frames(d)] for name, d in decs if name == 'ref']
    offs = [0] + [align(refs[0], s) for s in refs[1:]]
    out = os.path.join(scene_dir, 'ref')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    lo = max(0, -min(offs)) + TIME
    hi = min(len(s) - o for s, o in zip(refs, offs)) - TIME
    for k in range(lo, hi):
        m = sum(s[k + o] for s, o in zip(refs, offs)) / len(refs)
        Image.fromarray(to_rgb(m)).save(os.path.join(out, f'f{k:04d}.png'))
    for name, d in decs:
        if name != 'ref':
            dst = os.path.join(scene_dir, name)
            shutil.rmtree(dst, ignore_errors=True)
            shutil.move(d, dst)
    print(f'{a.scene}: reference from {len(refs)} runs (offsets {offs}), noisy runs {[n for n, _ in decs if n != "ref"]}')


def pairs(scene_dir):
    """(ref frames, [noisy run dirs]) of a scene."""
    runs = sorted(d for d in os.listdir(scene_dir) if d.startswith('cn'))
    return os.path.join(scene_dir, 'ref'), [os.path.join(scene_dir, d) for d in runs]


def cmd_train(a):
    rng = np.random.default_rng(0)
    A, B = [], []
    for scene in a.scenes:
        ref_dir, runs = pairs(os.path.join(a.data, scene))
        ref = [to_ycc(p) for p in frames(ref_dir)]
        h, w = ref[0].shape[1:]
        for run in runs:
            seq = [to_ycc(p) for p in frames(run)]
            off = align(ref, seq)
            for k in range(0, len(ref), 2):
                if not (TIME <= k + off < len(seq) - TIME):
                    continue
                rows = rng.integers(RADIUS + 8, int(h * OSD_SHARE), a.samples)
                cols = rng.integers(RADIUS + 8, w - RADIUS - 8, a.samples)
                A.append(features(seq, k + off, rows, cols))
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
    out = {
        'format': 'openvrx-denoise-filter',
        'version': 1,
        'radius': RADIUS,
        'time': TIME,
        'planes': 'ycbcr (BT.601 from RGB8: y = .299 r + .587 g + .114 b, cb = .564 (b - y), cr = .713 (r - y))',
        'order': 'per output plane: for dt, plane, dy, dx (each from -r to r), then bias',
        'trained_on': {'scenes': a.scenes, 'pixels': int(A.shape[0])},
        'coef': [coef[:, j].astype(float).round(7).tolist() for j in range(3)],
    }
    with open(a.out, 'w') as f:
        json.dump(out, f)
    print(f'trained on {A.shape[0]} pixels of {", ".join(a.scenes)} -> {a.out}')


def load_filter(path):
    with open(path) as f:
        flt = json.load(f)
    assert flt['radius'] == RADIUS and flt['time'] == TIME
    return np.array(flt['coef'], dtype=np.float32).T


def filter_seq(seq, coef):
    h, w = seq[0].shape[1:]
    rr, cc = np.mgrid[RADIUS:h - RADIUS, RADIUS:w - RADIUS]
    rr, cc = rr.ravel(), cc.ravel()
    out = []
    for k in range(len(seq)):
        y = seq[k].copy()
        if TIME <= k < len(seq) - TIME:
            y[:, rr, cc] = (features(seq, k, rr, cc) @ coef).T
        out.append(y)
    return out


def score(ref, seq):
    off = align(ref, seq)
    h = ref[0].shape[1]
    cut = (slice(None), slice(8, int(h * OSD_SHARE)), slice(8, -8))
    ks = [k for k in range(len(ref)) if TIME <= k + off < len(seq) - TIME]
    r = lambda x: x[cut]
    s = {'y': np.mean([psnr(r(ref[k])[0], r(seq[k + off])[0]) for k in ks]),
         'c': np.mean([psnr(r(ref[k])[1:], r(seq[k + off])[1:]) for k in ks]),
         'rgb': np.mean([psnr(to_rgb(r(ref[k])).astype(np.float32), to_rgb(r(seq[k + off])).astype(np.float32)) for k in ks])}
    return {k: round(float(v), 2) for k, v in s.items()}


def cmd_eval(a):
    coef = load_filter(a.filter)
    ref_dir, runs = pairs(os.path.join(a.data, a.scene))
    ref = [to_ycc(p) for p in frames(ref_dir)]
    res = {}
    for run in runs:
        seq = [to_ycc(p) for p in frames(run)]
        raw, filt = score(ref, seq), score(ref, filter_seq(seq, coef))
        res[os.path.basename(run)] = {'raw': raw, 'filtered': filt}
        print(f"{os.path.basename(run):>5}  raw Y {raw['y']:5.2f} C {raw['c']:5.2f} RGB {raw['rgb']:5.2f}"
              f"   filtered Y {filt['y']:5.2f} C {filt['c']:5.2f} RGB {filt['rgb']:5.2f}"
              f"   ({filt['rgb'] - raw['rgb']:+.2f} dB RGB)")
    if a.out:
        with open(a.out, 'w') as f:
            json.dump({'scene': a.scene, 'filter': a.filter, 'psnr_db': res}, f, indent=1)


def cmd_apply(a):
    coef = load_filter(a.filter)
    paths = frames(a.frames)
    seq = [to_ycc(p) for p in paths]
    os.makedirs(a.out, exist_ok=True)
    for p, y in zip(paths, filter_seq(seq, coef)):
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
    t = sub.add_parser('train')
    t.add_argument('--data', required=True)
    t.add_argument('--scenes', type=lambda v: v.split(','), required=True)
    t.add_argument('--out', required=True)
    t.add_argument('--samples', type=int, default=4000, help='pixels per frame pair')
    e = sub.add_parser('eval')
    e.add_argument('--data', required=True)
    e.add_argument('--scene', required=True)
    e.add_argument('--filter', required=True)
    e.add_argument('--out')
    ap = sub.add_parser('apply')
    ap.add_argument('--filter', required=True)
    ap.add_argument('--frames', required=True)
    ap.add_argument('--out', required=True)
    a = p.parse_args()
    {'simulate': cmd_simulate, 'train': cmd_train, 'eval': cmd_eval, 'apply': cmd_apply}[a.cmd](a)


if __name__ == '__main__':
    main()
