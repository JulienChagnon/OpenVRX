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
      --att-step 0.25   dB per click (default 0.25: 1 dB is 4 clicks, a 2 dB
                        sweep step 8 clicks). If + and - together selects a
                        coarser step on your unit, set it and pass it here.
      only + has a relay (the firmware default, detected automatically):
                        sweeps go upward on their own, and the script asks you
                        to set lower values by hand (the start, and between gains)

C5 gain (the C5 firmware has its own gain loop, `agc on|off`):
  --gains 56            nothing is sent to the C5: it must already run gain 56
                        with `agc off` saved (FX2-only, works with one USB port)
  --c5 PORT --gains 48,56,64
                        the script sends `agc off` and sets each gain over the C5
                        console (needs the C5 USB and the FX2 on the PC at once)
  --c5 PORT --gains auto,56
                        `auto` runs a sweep with the C5's gain loop on: the whole
                        receiver as it flies

Test pattern (the same ESP32 runs it: att_ctl puts NTSC on GPIO25 for the
standalone VTX's video input, docs/BENCH.md):
  --pattern card        sets the pattern before the sweep: card, bars, ramp,
                        check, grey, black or white (default: leave it as is)
  --video-amp 1000      sync tip to white in mV (default: leave it as is)
  --video-port PORT     the ESP32's port when --att is manual
  Whenever an ESP32 is reachable, its pattern and measured line rate go into
  run.json under "video". With the quad's camera as the source, leave these out.

Example (manual attenuator, 40 + 30 dB fixed, 25 mW VTX):
  python tools/bench/bench.py --fixed-db 70 --vtx-dbm 14 --att manual \\
      --steps 0:31:2 --lanes 4x4top --clk 9 --mhz 5880 --gains 56

Example (ESP32 attenuator and test card, standalone VTX):
  python3 tools/bench/bench.py --fixed-db 70 --vtx-dbm 14 --att esp32:/dev/ttyUSB0 \\
      --pattern card --steps 0:31:2 --lanes 4x4top --clk 9 --mhz 5880 --gains 56

Every argument is listed by --help. Needs pyserial only for esp32/c5 control.
"""
import argparse
import csv
import json
import os
import math
import platform
import queue
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXE = ".exe" if platform.system() == "Windows" else ""
DEFAULT_CLI = os.path.join(ROOT, "host", "target", "main", "release", "openvrx-cli" + EXE)
DEFAULT_APP = os.path.join(ROOT, "host", "target", "main", "release", "openvrx" + EXE)

# Picture counts as lost below this share of the reference's frames, or this
# many dB of PSNR below the reference's own frame-to-mean PSNR.
MIN_FRAME_SHARE = 0.5
MAX_PSNR_DROP_DB = 3.0


def open_serial(port):
    import serial  # only needed for esp32/c5 control

    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.1, 2
    # A line reset can hang a running C5 (and reboots the attenuator ESP32).
    if sys.platform == "win32":
        # DTR/RTS low before the open: Windows never raises them.
        s.dtr = False
        s.rts = False
        s.open()
    else:
        # Linux raises DTR and RTS together on open; pySerial would then drop
        # DTR first (DTR low + RTS high = reset). Drop RTS first instead.
        s.open()
        s.rts = False
        s.dtr = False
    s.reset_input_buffer()
    return s


def command(s, line, word, timeout):
    """Sends one console line and waits for its OK/ERR reply."""
    s.write((line + "\n").encode())
    deadline = time.time() + timeout
    out = b""
    while time.time() < deadline:
        out += s.read(4096)
        # Whole lines only: a reply still arriving would match "OK word"
        # half read (e.g. a get without its minus= field).
        for l in out.decode(errors="replace").split("\n")[:-1]:
            l = l.rstrip("\r")
            if l.startswith("OK " + word):
                return l
            if l.startswith("ERR " + word):
                raise RuntimeError(l)
    raise RuntimeError(f"no reply to '{line}'")


class ManualAttenuator:
    def set(self, db):
        input(f"\n  Set the step attenuator to {db:g} dB (add the lit LEDs), then press Enter... ")


class Esp32Attenuator:
    """att_ctl firmware. `step`: the step size chosen on the attenuator by
    hand. `at`: the value it shows now (skips homing). `plus_only`: only the
    + button is wired; going down is done by hand when the script asks. The
    firmware reports `minus=no` when it has no - relay, which also sets it."""

    def __init__(self, port, step, at=None, plus_only=False):
        self.s = open_serial(port)
        time.sleep(0.3)
        command(self.s, f"step {step:g}", "step", 2.0)
        if "minus=no" in command(self.s, "get", "get", 2.0):
            plus_only = True
        self.plus_only = plus_only
        if plus_only:
            print("  attenuator: + relay only, lower values are set by hand when asked")
        if at is None and plus_only:
            at = 0.0
            input("\n  Set the step attenuator to 0 dB by hand (hold -), then press Enter... ")
        if at is None:
            print("  homing the attenuator to 0 dB...")
            command(self.s, "home", "home", 60.0)
            self.pos = 0.0
        else:
            command(self.s, f"at {at:g}", "at", 2.0)
            self.pos = at

    def set(self, db):
        if self.plus_only and db < self.pos:
            input(f"\n  Set the step attenuator to {db:g} dB by hand (hold -), then press Enter... ")
            command(self.s, f"at {db:g}", "at", 2.0)
        else:
            command(self.s, f"set {db:g}", "set", 60.0)
        self.pos = db


class LiveApp:
    """The OpenVRX app with its window open (--live): the picture shows on this
    PC during the sweep, and each step is recorded by the app itself through
    its --bench-control stdin (the FX2 has one owner, so openvrx-cli record
    cannot run beside it). Key X in the window toggles the side-by-side view."""

    SAVED = re.compile(r"^MSG recording saved: (.+?)\.openvrx\.json \(")

    def __init__(self, exe, args, log_path):
        if not os.path.exists(exe):
            sys.exit(f"openvrx app not found at {exe}: cargo build --release -p openvrx")
        cmd = [exe, "--source", "live", "--control", "fx2-only", "--lanes", args.lanes,
               "--clk", str(args.clk), "--bench-control", "--compare-pattern", args.pattern or "card"]
        if args.mhz:
            cmd += ["--mhz", str(args.mhz)]
        if args.pattern:
            cmd.append("--compare")
        self.log = open(log_path, "w")
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
                                  text=True, bufsize=1)
        self.lines = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()
        self.wait(lambda l: l.startswith("MSG opened rx"), 30, "the app did not open the FX2")
        print("  live view open (key X: side by side with the test card)")

    def _read(self):
        for line in self.p.stdout:
            self.lines.put(line.rstrip("\n"))
        self.lines.put(None)

    def wait(self, ok, timeout, what):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self.lines.get(timeout=max(0.05, deadline - time.time()))
            except queue.Empty:
                break
            if line is None:
                raise RuntimeError(f"{what}: the app exited (log: {self.log.name})")
            if line.startswith("MSG error:"):
                raise RuntimeError(f"{what}: {line[4:]}")
            if ok(line):
                return line
        raise RuntimeError(f"{what} (log: {self.log.name})")

    def send(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()

    def record(self, base, seconds):
        """Records `seconds` into a folder next to `base`; returns the base path
        of the recording the app wrote (it names its files itself)."""
        folder = base + "-rec"
        os.makedirs(folder, exist_ok=True)
        self.send(f"record {folder}")
        time.sleep(seconds)
        self.send("stop")
        line = self.wait(lambda l: self.SAVED.match(l) is not None, 20, "recording not saved")
        return self.SAVED.match(line).group(1)

    def close(self):
        try:
            self.send("quit")
            self.p.wait(timeout=10)
        except Exception:
            self.p.kill()
        self.log.close()


class VtxGuard:
    """Holds att_ctl's pit-mode lease for the run: the VTX leaves pit mode only
    while a bench run renews `vtx active` (each step), and goes back when the
    run ends or dies (the lease runs out)."""

    LEASE_S = 120

    def __init__(self, s):
        self.s = s
        self.last = None

    def renew(self):
        stale = self.last is None or time.time() - self.last > self.LEASE_S - 5
        reply = command(self.s, f"vtx active {self.LEASE_S}", "vtx", 8.0)
        if stale:
            time.sleep(2.0)  # out of pit mode: let the VTX and the lock settle
        self.last = time.time()
        return reply

    def release(self):
        try:
            command(self.s, "vtx active 0", "vtx", 8.0)
            print("  VTX back in pit mode")
        except Exception as e:
            print(f"  WARNING: could not put the VTX in pit mode ({e})")


def setup_vtx(args, att):
    """Sets the VTX over Tramp (att_ctl `vtx`) when asked and returns its state
    and the pit-mode guard ((None, None) without an ESP32, or without any
    --vtx-* flag and no Tramp answer)."""
    s = att.s if isinstance(att, Esp32Attenuator) else (open_serial(args.video_port) if args.video_port else None)
    wants = [(f"vtx ch {args.vtx_ch}", "vtx") if args.vtx_ch else None,
             (f"vtx mhz {args.vtx_mhz}", "vtx") if args.vtx_mhz else None,
             (f"vtx mw {args.vtx_mw}" + (" force" if args.vtx_mw > 200 else ""), "vtx") if args.vtx_mw else None]
    wants = [w for w in wants if w]
    if s is None:
        if wants:
            sys.exit("--vtx-* need the ESP32: --att esp32:PORT or --video-port PORT")
        return None, None
    for line, word in wants:
        command(s, line, word, 8.0)
    try:
        first = command(s, "vtx", "vtx", 5.0)  # the full query: limits and temperature too
    except RuntimeError as e:
        if wants:
            raise
        print(f"  no VTX over Tramp ({e}): no pit-mode guard, the VTX stays as it is")
        return None, None
    guard = VtxGuard(s)
    reply = guard.renew()  # out of pit mode for the run
    state = dict(kv.split("=", 1) for kv in first.split()[2:] if "=" in kv)
    state.update(kv.split("=", 1) for kv in reply.split()[2:] if "=" in kv)
    print(f"  VTX: {state.get('mhz')} MHz, {state.get('conf_mw')} mW, pit {state.get('pit')}, "
          f"temp {state.get('temp')} C; pit mode again when the run ends")
    return state, guard


PATTERNS = ("card", "bars", "ramp", "check", "grey", "black", "white")


def setup_video(args, att):
    """Sets the ESP32 test pattern when asked, and returns its state for
    run.json (None when no ESP32 is reachable). A change restarts the
    picture, so this waits for the receiver to relock."""
    if isinstance(att, Esp32Attenuator):
        s = att.s
    elif args.video_port:
        s = open_serial(args.video_port)
        time.sleep(0.3)
    else:
        if args.pattern or args.video_amp:
            sys.exit("--pattern/--video-amp need the ESP32: --att esp32:PORT or --video-port PORT")
        return None
    changed = False
    if args.video_amp is not None:
        command(s, f"amp {args.video_amp}", "amp", 3.0)
        changed = True
    if args.pattern:
        command(s, f"pattern {args.pattern}", "pattern", 3.0)
        changed = True
    try:
        reply = command(s, "vget", "vget", 3.0)
    except RuntimeError as e:
        if changed:
            raise
        print(f"  no test pattern state from the ESP32 ({e}); older att_ctl?")
        return None
    state = dict(kv.split("=", 1) for kv in reply.split()[2:] if "=" in kv)
    print(f"  test pattern: {state.get('pattern')} {state.get('amp_mv')} mV, video {state.get('video')}, "
          f"line rate {state.get('line_hz')} Hz")
    if state.get("video") != "on":
        print("  WARNING: the ESP32 video output is off (send 'video on')")
    if changed:
        time.sleep(2.0)
    return state


def parse_steps(text, grid=0.25):
    """'0:31:2' (start:stop:step, stop included when on the grid) or '0,4,10.5'.
    Every value must sit on the attenuator's step grid."""
    if ":" in text:
        a, b, c = (float(x) for x in text.split(":"))
        out, v = [], a
        while v <= b + 1e-9:
            out.append(round(v * 4) / 4)  # attenuator resolution is 0.25 dB
            v += c
    else:
        out = [round(float(x) * 4) / 4 for x in text.split(",")]
    off = [v for v in out if abs(v / grid - round(v / grid)) > 1e-6 or not 0 <= v <= 31.75]
    if off:
        sys.exit(f"steps {off} are not on the {grid:g} dB attenuator grid (0 to 31.75)")
    return out


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{p.stderr.strip()}")
    return p.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixed-db", type=float, required=True, help="sum of the fixed attenuators, e.g. 70")
    ap.add_argument("--vtx-dbm", type=float, default=None,
                    help="VTX output power in dBm (25 mW = 14; default: from --vtx-mw or Tramp, else 14)")
    ap.add_argument("--extra-loss-db", type=float, default=3.0,
                    help="cables + step attenuator insertion loss estimate (1 to 4.5 dB for the AW8637)")
    ap.add_argument("--steps", default="0:31:2", help="step attenuator values, start:stop:step or a list")
    ap.add_argument("--att", default="manual", help="manual or esp32:PORT")
    ap.add_argument("--att-step", type=float, default=0.25,
                    help="step size set on the attenuator by hand: 0.25, 0.5, 1 or 10 dB (esp32: one press per step)")
    ap.add_argument("--att-at", type=float, default=None,
                    help="esp32: the value the attenuator shows now, instead of homing it")
    ap.add_argument("--att-plus-only", action="store_true",
                    help="esp32: treat it as + only even if the firmware has a - relay (+ only is detected anyway)")
    ap.add_argument("--c5", help="C5 console port, to set each gain (else gains are labels only)")
    ap.add_argument("--gains", default="56", help="comma list of C5 gain indexes, or auto (C5 gain loop)")
    ap.add_argument("--lanes", default="4x4top")
    ap.add_argument("--clk", type=int, default=9)
    ap.add_argument("--mhz", type=int, default=5880,
                    help="channel label in MHz (default 5880 = R7, the bench channel; the C5 must already be there)")
    ap.add_argument("--seconds", type=float, default=3.0, help="recording length per step")
    ap.add_argument("--frames", type=int, default=30, help="frames scored per step")
    ap.add_argument("--out", default=None, help="run folder (default: bench-runs/<time> beside the repo)")
    ap.add_argument("--keep", action="store_true", help="keep every recording (about 27 MB/s each)")
    ap.add_argument("--cli", default=DEFAULT_CLI, help="path to openvrx-cli")
    ap.add_argument("--pattern", choices=PATTERNS, default=None,
                    help="ESP32 test pattern for the VTX (default: leave as is)")
    ap.add_argument("--video-amp", type=int, default=None,
                    help="test pattern level, sync tip to white in mV (200..3200)")
    ap.add_argument("--video-port", default=None,
                    help="pattern ESP32 port when --att is manual (else the --att esp32 port)")
    ap.add_argument("--live", action="store_true",
                    help="show the picture on this PC during the sweep (records through the OpenVRX app)")
    ap.add_argument("--app", default=DEFAULT_APP, help="path to the openvrx app (for --live)")
    ap.add_argument("--vtx-ch", default=None, help="set the VTX channel over Tramp first, e.g. R8")
    ap.add_argument("--vtx-mhz", type=int, default=None, help="set the VTX frequency over Tramp first")
    ap.add_argument("--vtx-mw", type=int, default=None,
                    help="set the VTX power (mW) over Tramp first; also sets --vtx-dbm if that is not given")
    args = ap.parse_args()

    if args.att_step not in (0.25, 0.5, 1.0, 10.0):
        sys.exit("--att-step must be 0.25, 0.5, 1 or 10")
    steps = parse_steps(args.steps, args.att_step if args.att.startswith("esp32:") else 0.25)
    gains = [g.strip() for g in args.gains.split(",")]
    for g in gains:
        if g != "auto" and not g.isdigit():
            sys.exit(f"bad gain {g!r}: an index or auto")
    out = args.out or os.path.join(os.path.dirname(ROOT), "bench-runs", time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out, exist_ok=True)
    if not os.path.exists(args.cli):
        sys.exit(f"openvrx-cli not found at {args.cli}: cargo build --release -p openvrx-cli")

    if args.att.startswith("esp32:"):
        att = Esp32Attenuator(args.att.split(":", 1)[1], args.att_step, args.att_at, args.att_plus_only)
    else:
        att = ManualAttenuator()
    c5 = open_serial(args.c5) if args.c5 else None
    meta = {k: v for k, v in vars(args).items()}
    meta["started"] = time.strftime("%Y-%m-%d %H:%M:%S")
    video = setup_video(args, att)
    if video:
        meta["video"] = video
    vtx, guard = setup_vtx(args, att)
    if vtx:
        meta["vtx"] = vtx
    if args.vtx_dbm is None:
        # The power the VTX reports over Tramp, else what was asked, else 25 mW.
        mw = int(vtx.get("conf_mw", "0") or 0) if vtx else 0
        mw = mw or args.vtx_mw or 25
        args.vtx_dbm = round(10 * math.log10(mw), 2)
        meta["vtx_dbm"] = args.vtx_dbm
        print(f"  VTX power used for levels: {mw} mW = {args.vtx_dbm} dBm")
    with open(os.path.join(out, "run.json"), "w") as f:
        json.dump(meta, f, indent=2)
    live = None
    try:
        live = LiveApp(args.app, args, os.path.join(out, "app.log")) if args.live else None
        sweep(args, att, c5, live, guard, steps, gains, out)
    finally:
        if live:
            live.close()
        if guard:
            guard.release()


def sweep(args, att, c5, live, guard, steps, gains, out):
    """Runs every gain over every attenuator step (see the module doc)."""

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
            if gain == "auto":
                command(c5, "agc on", "agc", 2.0)
            else:
                command(c5, "agc off", "agc", 2.0)
                command(c5, f"gain {gain}", "gain", 2.0)
            time.sleep(0.5)
        print(f"\n== gain {gain}: {len(steps)} steps, fixed {args.fixed_db:g} dB")
        ref, ref_psnr, ref_frames, prev = None, None, None, None
        for db in steps:
            att.set(db)
            if guard:
                guard.renew()  # keeps the VTX out of pit mode for this step
            time.sleep(1.0)
            base = os.path.join(out, f"g{gain}-att{db:05.2f}")
            if live:
                base = live.record(base, args.seconds)
            else:
                run([args.cli, "record", base, "--lanes", args.lanes, "--clk", str(args.clk),
                     "--seconds", str(args.seconds), "--mhz", str(args.mhz),
                     "--gain", "0" if gain == "auto" else gain])
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
            # Not with the gain loop on: it makes up the difference.
            if gain != "auto" and prev and db - prev["att_db"] >= 8 and prev["clip_pct"] < 1 and r["clip_pct"] < 1:
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
                if live:
                    try:
                        os.rmdir(os.path.dirname(base))  # the step's -rec folder, now empty
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
