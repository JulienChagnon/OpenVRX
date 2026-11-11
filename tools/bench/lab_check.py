#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""One read-only look at the whole bench before a test.

Queries every device it can reach and prints its state plus warnings for
the usual faults: USB devices (C5, FX2, capture card), the C5 console
(gain, AGC, clip, slips), and over the ESP32's network console the
attenuator position, the video output and the VTX through Tramp (pit
mode, power, temperature); on Hyprland also the video output monitor.
Nothing is changed and nothing is reset: the C5 is opened with DTR/RTS low
(as c5cmd.py does) and the ESP32 only over the network (its USB port resets
it when opened).

    lab_check.py [--c5 /dev/ttyACM0] [--esp32 socket://openvrx-bench.local:2323]
                 [--monitor HDMI-A-2] [--json OUT.json]

Exit status 1 when a warning is printed. Needs pyserial.
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import time

import serial


def lsusb():
    p = subprocess.run(["lsusb"], capture_output=True, text=True)
    want = {"04b4:": "FX2 (no firmware yet)", "1209:0001": "FX2 (OpenVRX firmware)",
            "303a:": "C5 (Espressif USB)", "534d:": "capture card (MacroSilicon)",
            "10c4:": "ESP32 USB (CP210x)", "1a86:": "ESP32 USB (CH34x)"}
    found = []
    for line in p.stdout.splitlines():
        for vid, name in want.items():
            if f"ID {vid}" in line:
                found.append(f"{name}: {line.split('ID ')[1]}")
    return found


def kv(text):
    """key=value pairs of a console reply."""
    return dict(re.findall(r"(\w+)=([^\s]+)", text))


def c5_status(port):
    # Exactly as tools/c5cmd.py: on Linux open with the defaults, then drop
    # RTS before DTR. Setting dtr/rts False before open makes pySerial apply
    # DTR low with RTS high, the USB Serial/JTAG reset, which hangs a running
    # C5 until it is power-cycled. Windows applies them before the lines change.
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.2, 2
    if sys.platform == "win32":
        s.dtr = False
        s.rts = False
        s.open()
    else:
        s.open()
        s.rts = False
        s.dtr = False
    try:
        s.reset_input_buffer()
        out = ""
        for _ in range(3):  # a freshly booted C5 may not answer the first time
            s.write(b"status\n")
            t0 = time.time()
            while time.time() - t0 < 1.5:
                out += s.read(4096).decode(errors="replace")
                if "OK status" in out:
                    break
            if "OK status" in out:
                break
        line = next((l for l in out.splitlines() if l.startswith("OK status")), "")
        return kv(line)
    finally:
        s.close()


def esp32(url, cmds):
    s = serial.serial_for_url(url, timeout=0.3)
    try:
        time.sleep(0.3)
        s.read(8192)
        out = {}
        for c in cmds:
            s.write((c + "\n").encode())
            text, t0 = "", time.time()
            while time.time() - t0 < 3.0:
                text += s.read(4096).decode(errors="replace")
                if re.search(r"^(OK|ERR) " + c.split()[0], text, re.M):
                    break
            out[c] = next((l for l in text.splitlines() if l.startswith(("OK", "ERR"))), "no reply")
        return out
    finally:
        s.close()


def monitor(name):
    if not shutil.which("hyprctl"):
        return None
    p = subprocess.run(["hyprctl", "-j", "monitors"], capture_output=True, text=True)
    for m in json.loads(p.stdout or "[]"):
        if m["name"] == name:
            return f"{m['width']}x{m['height']}@{m['refreshRate']:.3f}"
    return "not connected"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--c5", default="/dev/ttyACM0")
    ap.add_argument("--esp32", default="socket://openvrx-bench.local:2323")
    ap.add_argument("--monitor", default="HDMI-A-2", help="video output feeding the CVBS converter")
    ap.add_argument("--json", help="also write everything to this file")
    a = ap.parse_args()
    report, warn = {}, []

    report["usb"] = lsusb()
    print("USB:", *report["usb"], sep="\n  ")
    if not any(u.startswith("FX2") for u in report["usb"]):
        warn.append("no FX2 on USB")

    try:
        c5 = c5_status(a.c5)
        report["c5"] = c5
        print(f"C5: {' '.join(f'{k}={c5.get(k)}' for k in ('mhz', 'gain', 'agc', 'clip', 'rms', 'dc', 'slips', 'phy'))}")
        if c5.get("agc") != "1":
            warn.append("C5 AGC off")
        if c5.get("slips", "0") != "0":
            warn.append(f"C5 sample slips {c5['slips']}")
        if c5.get("gain") == "83" and float(c5.get("rms", 9)) < 2.5:
            warn.append("C5 at full gain with a weak input: VTX off, in pit mode, or the RF path is open")
        if float(c5.get("clip", 0)) > 2:
            warn.append(f"C5 clipping {c5['clip']} %")
    except Exception as e:
        report["c5"] = {"error": str(e)}
        warn.append(f"C5 console not reachable on {a.c5}: {e}")
        print("C5: not reachable:", e)

    try:
        r = esp32(a.esp32, ["get", "vget", "vtx"])
        report["esp32"] = r
        for k, v in r.items():
            print(f"ESP32 {k}: {v}")
        vtx = kv(r.get("vtx", ""))
        if "no answer" in r.get("vtx", ""):
            warn.append("VTX not answering on Tramp (wire off or VTX unpowered): its pit/power state is unknown")
        elif vtx.get("pit") == "1":
            warn.append("VTX in PIT MODE (well under 1 mW): 'vtx active 600' over the ESP32 console for a test")
        if vtx.get("temp") and int(vtx["temp"]) >= 85:
            warn.append(f"VTX temperature {vtx['temp']} C")
        if "db=unknown" in r.get("get", ""):
            warn.append("attenuator position unknown: confirm 0 dB on the LEDs, then 'at 0'")
        if "video=on" in r.get("vget", ""):
            warn.append("ESP32 video output is ON: make sure it is not wired to the VTX pad while the converter feeds it")
    except Exception as e:
        report["esp32"] = {"error": str(e)}
        warn.append(f"ESP32 not reachable at {a.esp32}: {e}")
        print("ESP32: not reachable:", e)

    mon = monitor(a.monitor)
    if mon is not None:
        report["monitor"] = {a.monitor: mon}
        print(f"Video output {a.monitor}: {mon}")
        if mon not in ("not connected",) and not mon.startswith("720x480@59.94"):
            warn.append(f"{a.monitor} is {mon}, not 720x480@59.94 (NTSC loops expect that)")

    report["warnings"] = warn
    if a.json:
        with open(a.json, "w") as f:
            json.dump(report, f, indent=1)
    print("\nWARNINGS:" if warn else "\nno warnings", *warn, sep="\n  - " if warn else "")
    sys.exit(1 if warn else 0)


if __name__ == "__main__":
    main()
