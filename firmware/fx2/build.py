#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Builds the OpenVRX FX2 firmware with SDCC, without make.

    python build.py            build/openvrx-fx2.ihx, .iic, copies to prebuilt/
    python build.py --test     ... and runs the unit tests
    python build.py --clean    removes build/

SDCC is looked up in $SDCC_HOME/bin, on PATH, then in the default install
directories. Warnings in OpenVRX sources fail the build.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "tools"))
import fx2image  # noqa: E402

SRC = HERE / "src"
LIBFX2 = HERE / "third_party" / "libfx2" / "library"
BUILD = HERE / "build"
PREBUILT = HERE / "prebuilt"
TARGET = "openvrx-fx2"

# Memory map (16 KB unified program/data RAM at 0x0000-0x3FFF):
#   0x0000-0x37FF  code and constants
#   0x3800-0x3FEF  xdata variables
#   0x3FF0-0x3FF7  board ID preset written by the host (never linked)
CODE_SIZE = 0x3800
XRAM_LOC = 0x3800
XRAM_SIZE = 0x07F0
ID_PRESET = 0x3FF0

USB_VID, USB_PID = 0x1209, 0x0001

COMMON = ["-mmcs51", "--model-small", "-DSYNCDELAYLEN=16", f"-I{LIBFX2 / 'include'}"]

OPENVRX_C = ["main.c", "usb_desc.c", "requests.c", "stream.c", "board.c"]
OPENVRX_ASM = ["hotloops.asm"]

DEFISRS = ["IE0", "TF0", "IE1", "TF1", "RI_TI_0", "TF2", "RESUME", "RI_TI_1", "I2C", "IE5", "IE6"]
DEFAUTOISRS = [
    "SUDAV", "SOF", "SUTOK", "SUSPEND", "USBRESET", "HISPEED",
    "EP0ACK", "EP0IN", "EP0OUT", "EP1IN", "EP1OUT", "EP2", "EP4", "EP6", "EP8",
    "IBN", "EP0PING", "EP1PING", "EP2PING", "EP4PING", "EP6PING", "EP8PING",
    "ERRLIMIT", "EP2ISOERR", "EP4ISOERR", "EP6ISOERR", "EP8ISOERR",
    "EP2PF", "EP4PF", "EP6PF", "EP8PF", "EP2EF", "EP4EF", "EP6EF", "EP8EF",
    "EP2FF", "EP4FF", "EP6FF", "EP8FF", "GPIFDONE", "GPIFWF",
]


def find_sdcc_bin():
    candidates = []
    if os.environ.get("SDCC_HOME"):
        candidates.append(Path(os.environ["SDCC_HOME"]) / "bin")
    found = shutil.which("sdcc")
    if found:
        candidates.append(Path(found).parent)
    candidates += [Path(r"C:\Program Files\SDCC\bin"), Path(r"C:\Program Files (x86)\SDCC\bin"),
                   Path("/usr/bin"), Path("/usr/local/bin")]
    for c in candidates:
        if (c / "sdcc").exists() or (c / "sdcc.exe").exists():
            return c
    sys.exit("error: SDCC not found (set SDCC_HOME or put sdcc on PATH)")


class Toolchain:
    def __init__(self):
        self.bin = find_sdcc_bin()
        self.env = dict(os.environ)
        # sdcpp starts cc1 from PATH on Windows.
        self.env["PATH"] = str(self.bin) + os.pathsep + self.env.get("PATH", "")

    def run(self, args, check_warnings=False):
        tool = str(self.bin / args[0])
        proc = subprocess.run([tool] + [str(a) for a in args[1:]], env=self.env,
                              capture_output=True, text=True)
        out = (proc.stdout + proc.stderr).strip()
        if proc.returncode != 0:
            print(" ".join(str(a) for a in args))
            print(out)
            sys.exit(f"error: {args[0]} failed")
        if check_warnings and re.search(r"warning", out, re.IGNORECASE):
            print(" ".join(str(a) for a in args))
            print(out)
            sys.exit("error: warnings in OpenVRX sources")
        return out

    def cc(self, src, obj, extra=(), std="--std-sdcc99", check_warnings=False):
        obj.parent.mkdir(parents=True, exist_ok=True)
        self.run(["sdcc", *COMMON, std, *extra, "-c", "-o", obj, src],
                 check_warnings=check_warnings)

    def asm(self, src, obj):
        obj.parent.mkdir(parents=True, exist_ok=True)
        # sdas writes the object next to the name given by -o style first arg.
        self.run(["sdas8051", "-plosgff", obj, src])

    def lib(self, path, objs):
        if path.exists():
            path.unlink()
        self.run(["sdar", "rc", path, *objs])


def build_libfx2(tc):
    out = BUILD / "libfx2"
    objs = {"fx2": [], "fx2isrs": [], "fx2usb": []}
    for name in ["xmemcpy", "xmemclr", "bswap", "delay", "i2c", "eeprom"]:
        o = out / f"{name}.rel"
        tc.cc(LIBFX2 / f"{name}.c", o)
        objs["fx2"].append(o)
    o = out / "syncdelay.rel"
    tc.asm(LIBFX2 / "syncdelay.asm", o)
    objs["fx2"].append(o)

    o = out / "autovec.rel"
    tc.asm(LIBFX2 / "autovec.asm", o)
    objs["fx2isrs"].append(o)
    for n in DEFISRS:
        o = out / f"defisr_{n}.rel"
        tc.cc(LIBFX2 / "defisr.c", o, [f"-DISRNAME=isr_{n}", f"-DINTNAME=_INT_{n}"])
        objs["fx2isrs"].append(o)
    for n in DEFAUTOISRS:
        o = out / f"defautoisr_{n}.rel"
        tc.cc(LIBFX2 / "defautoisr.c", o, [f"-DISRNAME=isr_{n}"])
        objs["fx2isrs"].append(o)

    for name in ["usb", "defusbdesc", "defusbsetup", "defusbsetconfig", "defusbgetconfig",
                 "defusbsetiface", "defusbgetiface", "defusbhalt"]:
        o = out / f"{name}.rel"
        tc.cc(LIBFX2 / f"{name}.c", o)
        objs["fx2usb"].append(o)

    libs = []
    for name in ["fx2usb", "fx2isrs", "fx2"]:
        path = out / f"{name}.lib"
        tc.lib(path, objs[name])
        libs.append(path)
    return libs


def build_firmware(tc, libs):
    objs = []
    for name in OPENVRX_C:
        o = BUILD / "obj" / name.replace(".c", ".rel")
        tc.cc(SRC / name, o, [f"-I{SRC}"], std="--std-sdcc11", check_warnings=True)
        objs.append(o)
    for name in OPENVRX_ASM:
        o = BUILD / "obj" / name.replace(".asm", ".rel")
        tc.asm(SRC / name, o)
        objs.append(o)

    linked = BUILD / f"{TARGET}-linked.ihx"
    tc.run(["sdcc", "-mmcs51", "--model-small",
            "--code-size", hex(CODE_SIZE), "--xram-loc", hex(XRAM_LOC),
            "--xram-size", hex(XRAM_SIZE), "--iram-size", "0x100",
            "-o", linked, *objs, *libs], check_warnings=True)
    return linked


def check_layout(segments):
    end = max(a + len(d) for a, d in segments)
    if end > CODE_SIZE:
        sys.exit(f"error: image ends at 0x{end:04X}, beyond the code area")
    for a, d in segments:
        if a < ID_PRESET + 8 and a + len(d) > ID_PRESET:
            sys.exit("error: image overlaps the board ID preset block")
    return end


def report_memory(linked):
    mem = linked.with_suffix(".mem")
    text = mem.read_text(errors="replace") if mem.exists() else ""
    lines = [l for l in text.splitlines() if l.strip()]
    for l in lines:
        if re.search(r"(ROM/EPROM/FLASH|EXTERNAL RAM|Stack starts|Other memory|Internal RAM)", l):
            print("  " + l.strip())


def build(run_tests):
    tc = Toolchain()
    print(f"SDCC: {tc.bin}")
    print(tc.run(["sdcc", "--version"]).splitlines()[0])
    BUILD.mkdir(exist_ok=True)

    libs = build_libfx2(tc)
    linked = build_firmware(tc, libs)

    segments = fx2image.read_ihx(linked)
    end = check_layout(segments)
    code_bytes = sum(len(d) for _, d in segments)

    ihx = BUILD / f"{TARGET}.ihx"
    fx2image.write_ihx(ihx, segments)
    iic = BUILD / f"{TARGET}.iic"
    did = (0 << 8) | (1 << 4) | 0
    iic.write_bytes(fx2image.make_c2(segments, USB_VID, USB_PID, did, config=0x00))
    # The map of the linked image is used by the descriptor tests.
    shutil.copyfile(linked.with_suffix(".map"), BUILD / f"{TARGET}.map")

    PREBUILT.mkdir(exist_ok=True)
    shutil.copyfile(ihx, PREBUILT / f"{TARGET}.ihx")
    shutil.copyfile(iic, PREBUILT / f"{TARGET}.iic")

    print(f"image: {code_bytes} bytes loaded, highest address 0x{end - 1:04X}"
          f" (code limit 0x{CODE_SIZE:04X})")
    report_memory(linked)
    print(f"EEPROM image: {iic.stat().st_size} bytes (C2 load)")
    print(f"wrote {ihx.relative_to(HERE)}, {iic.relative_to(HERE)}, prebuilt/{TARGET}.ihx, "
          f"prebuilt/{TARGET}.iic")

    subprocess.run([sys.executable, str(HERE / "tools" / "cycles.py"),
                    str(BUILD / "obj" / "hotloops.lst")], check=True)

    if run_tests:
        r = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s",
                            str(HERE / "tests"), "-v"], cwd=HERE)
        if r.returncode != 0:
            sys.exit("error: tests failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clean", action="store_true", help="remove build/ and exit")
    ap.add_argument("--test", action="store_true", help="run the unit tests after building")
    args = ap.parse_args()
    if args.clean:
        shutil.rmtree(BUILD, ignore_errors=True)
        return
    build(args.test)


if __name__ == "__main__":
    main()
