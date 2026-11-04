#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Attenuation sweep for the wired bench (docs/BENCH.md).

For each C5 gain and each step attenuator setting: set the attenuation,
record a few seconds with `openvrx-cli record`, score it with
`openvrx-cli score` against the first (strongest) step of the same gain,
and append one line to results.jsonl and results.csv in the run folder.

Attenuator control:
  --att manual          the script asks you to set each value and press Enter
  --att esp32:PORT      tools/bench/att_ctl firmware presses the buttons

C5 gain:
  --gains 56            nothing is sent to the C5: it must already run gain 56
                        (FX2-only, works with one USB port)
  --c5 PORT --gains 48,56,64
                        the script sets each gain over the C5 console (needs the
                        C5 USB and the FX2 on the PC at the same time)

Example (manual attenuator, 40 + 30 dB fixed, 25 mW VTX):
  python tools/bench/bench.py --fixed-db 70 --vtx-dbm 14 --att manual \\
      --steps 0:31:2 --lanes 4x4top --clk 9 --mhz 5880 --gains 56

Every argument is listed by --help. Needs pyserial only for esp32/c5 control.
"""
import argparse
import csv
import json
import os
import platform
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXE = ".exe" if platform.system() == "Windows" else ""
DEFAULT_CLI = os.path.join(ROOT, "host", "target", "main", "release", "openvrx-cli" + EXE)

# Picture counts as lost below this share of the reference's frames, or this
# many dB of PSNR below the reference's own frame-to-mean PSNR.
MIN_FRAME_SHARE = 0.5
MAX_PSNR_DROP_DB = 3.0


def open_serial(port):
    import serial  # only needed for esp32/c5 control

    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.1, 2
    # DTR/RTS low before the open: a line reset can hang a running C5.
    s.dtr = False
    s.rts = False
    s.open()
    s.reset_input_buffer()
    return s


def command(s, line, word, timeout):
    """Sends one console line and waits for its OK/ERR reply."""
    s.write((line + "\n").encode())
    deadline = time.time() + timeout
    out = b""
    while time.time() < deadline:
        out += s.read(4096)
        for l in out.decode(errors="replace").splitlines():
            if l.startswith("OK " + word):
                return l
            if l.startswith("ERR " + word):
                raise RuntimeError(l)
    raise RuntimeError(f"no reply to '{line}'")


class ManualAttenuator:
    def set(self, db):
        input(f"\n  Set the step attenuator to {db:g} dB (add the lit LEDs), then press Enter... ")


class Esp32Attenuator:
    def __init__(self, port):
        self.s = open_serial(port)
        time.sleep(0.3)
        print("  homing the attenuator to 0 dB (about 15 s)...")
        command(self.s, "home", "home", 30.0)

    def set(self, db):
        command(self.s, f"set {db:g}", "set", 30.0)


def parse_steps(text):
    """'0:31:2' (start:stop:step, stop included when on the grid) or '0,4,10.5'."""
    if ":" in text:
        a, b, c = (float(x) for x in text.split(":"))
        out, v = [], a
        while v <= b + 1e-9:
            out.append(round(v * 4) / 4)  # attenuator grid is 0.25 dB
            v += c
        return out
    return [round(float(x) * 4) / 4 for x in text.split(",")]


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{p.stderr.strip()}")
    return p.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixed-db", type=float, required=True, help="sum of the fixed attenuators, e.g. 70")
    ap.add_argument("--vtx-dbm", type=float, default=14.0, help="VTX output power in dBm (25 mW = 14)")
    ap.add_argument("--extra-loss-db", type=float, default=3.0,
                    help="cables + step attenuator insertion loss estimate (1 to 4.5 dB for the AW8637)")
    ap.add_argument("--steps", default="0:31:2", help="step attenuator values, start:stop:step or a list")
    ap.add_argument("--att", default="manual", help="manual or esp32:PORT")
    ap.add_argument("--c5", help="C5 console port, to set each gain (else gains are labels only)")
    ap.add_argument("--gains", default="56", help="comma list of C5 gain indexes")
    ap.add_argument("--lanes", default="4x4top")
    ap.add_argument("--clk", type=int, default=9)
    ap.add_argument("--mhz", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=3.0, help="recording length per step")
    ap.add_argument("--frames", type=int, default=30, help="frames scored per step")
    ap.add_argument("--out", default=None, help="run folder (default: bench-runs/<time> beside the repo)")
    ap.add_argument("--keep", action="store_true", help="keep every recording (about 27 MB/s each)")
    ap.add_argument("--cli", default=DEFAULT_CLI, help="path to openvrx-cli")
    args = ap.parse_args()

    steps = parse_steps(args.steps)
    gains = [int(g) for g in args.gains.split(",")]
    out = args.out or os.path.join(os.path.dirname(ROOT), "bench-runs", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out, exist_ok=True)
    if not os.path.exists(args.cli):
        sys.exit(f"openvrx-cli not found at {args.cli}: cargo build --release -p openvrx-cli")

    att = Esp32Attenuator(args.att.split(":", 1)[1]) if args.att.startswith("esp32:") else ManualAttenuator()
    c5 = open_serial(args.c5) if args.c5 else None
    meta = {k: v for k, v in vars(args).items()}
    meta["started"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(os.path.join(out, "run.json"), "w") as f:
        json.dump(meta, f, indent=2)

    jl = open(os.path.join(out, "results.jsonl"), "a")
    cf = open(os.path.join(out, "results.csv"), "a", newline="")
    fields = ["gain", "att_db", "rx_dbm", "frames", "psnr_median", "psnr_min", "damaged", "lock",
              "cn_db", "clip_pct", "power_db", "picture"]
    cw = csv.DictWriter(cf, fieldnames=fields, extrasaction="ignore")
    if cf.tell() == 0:
        cw.writeheader()

    summary = {}
    for gain in gains:
        if c5:
            command(c5, f"gain {gain}", "gain", 2.0)
            time.sleep(0.5)
        print(f"\n== gain {gain}: {len(steps)} steps, fixed {args.fixed_db:g} dB")
        ref, ref_psnr, ref_frames, prev = None, None, None, None
        for db in steps:
            att.set(db)
            time.sleep(1.0)
            base = os.path.join(out, f"g{gain}-att{db:05.2f}")
            run([args.cli, "record", base, "--lanes", args.lanes, "--clk", str(args.clk),
                 "--seconds", str(args.seconds), "--mhz", str(args.mhz), "--gain", str(gain)])
            score_cmd = [args.cli, "score", base, "--frames", str(args.frames), "--skip", "10"]
            if ref is None:
                ref = base
            score_cmd += ["--ref", ref]
            r = json.loads(run(score_cmd).strip().splitlines()[-1])
            if ref == base:
                ref_psnr, ref_frames = r["psnr_median"], max(r["frames"], 1)
            r.update(gain=gain, att_db=db,
                     rx_dbm=round(args.vtx_dbm - args.fixed_db - args.extra_loss_db - db, 2))
            ok = (r["frames"] >= MIN_FRAME_SHARE * ref_frames and r["psnr_median"] is not None
                  and ref_psnr is not None and r["psnr_median"] >= ref_psnr - MAX_PSNR_DROP_DB)
            r["picture"] = "ok" if ok else "lost"
            # Leakage check: 10 dB more attenuation should cost about 10 dB of power
            # while the C5 is in its linear range (no clipping, above the noise).
            if prev and db - prev["att_db"] >= 8 and prev["clip_pct"] < 1 and r["clip_pct"] < 1:
                drop = prev["power_db"] - r["power_db"]
                expect = db - prev["att_db"]
                if r["picture"] == "ok" and drop < 0.5 * expect:
                    r["leak_warning"] = f"power fell {drop:.1f} dB for {expect:g} dB more attenuation"
            jl.write(json.dumps(r) + "\n")
            jl.flush()
            cw.writerow(r)
            cf.flush()
            print(f"  att {db:5.2f} dB  rx ~{r['rx_dbm']:7.2f} dBm  frames {r['frames']:3d}  "
                  f"psnr {r['psnr_median']}  C/N {r['cn_db']:.1f}  clip {r['clip_pct']:.1f}%  {r['picture']}"
                  + (f"  LEAK? {r['leak_warning']}" if "leak_warning" in r else ""))
            if r["picture"] == "ok":
                summary[gain] = r["rx_dbm"]
            if not args.keep and base != ref:
                for ext in (".rx0.bin", ".openvrx.json", ".events.jsonl"):
                    try:
                        os.remove(base + ext)
                    except OSError:
                        pass
            prev = r
        if not args.keep:
            for ext in (".rx0.bin",):
                try:
                    os.remove(ref + ext)
                except OSError:
                    pass

    print("\nweakest level with a picture, per gain:")
    for g, dbm in summary.items():
        print(f"  gain {g}: {dbm:.1f} dBm (estimated)")
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump({str(g): v for g, v in summary.items()}, f, indent=2)
    print(f"results: {out}")


if __name__ == "__main__":
    main()
