#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Flash the OpenVRX C5 firmware without hanging the board.

Adapted from C5VRX tools/flash.py and tools/enter_download.py (C5VRX by
Twotoz and the C5VRX contributors, GPL-3.0). Changes: OpenVRX console command
`dl` instead of the C5VRX download key, no health check or IQ logging.

`idf.py flash` (and a plain pySerial open/close) resets the chip through the
USB DTR/RTS lines. On a running image that reset leaves the modem domain
(forced clocks, the MODEM_DIAG writer) running into the ROM loader, which
then stops answering: esptool fails with "Write timeout" and the board stays
hung until it is unplugged. This takes the route that works:

1. send `dl` on the console (opened with DTR and RTS low): the firmware
   stops the writer and Wi-Fi and reboots into the ROM loader itself;
2. esptool --before no-reset write-flash @flash_args;
3. --after watchdog-reset starts the new image (an RTS reset would leave a
   C5 in the loader).

    idf.py build                    (in firmware/c5)
    python tools/flash.py COM9
    python tools/flash.py COM9 --app-only
    python tools/flash.py COM9 --no-dl     # board already in the ROM loader

Run it in the ESP-IDF environment (esptool and pyserial).
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_BUILD = HERE.parent / "firmware" / "c5" / "build"


def open_quiet(port):
    """Open the console with DTR and RTS low before the open: never resets the chip."""
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0.05, 0.5
    s.dtr = False
    s.rts = False
    s.open()
    return s


def enter_download(port, attempts=5):
    """Send `dl` until the firmware acknowledges it (or the port vanishes)."""
    import serial
    for attempt in range(1, attempts + 1):
        try:
            s = open_quiet(port)
        except serial.SerialException as e:
            print(f"flash: open {port} failed ({e}); retrying")
            time.sleep(0.5)
            continue
        seen = b""
        try:
            s.reset_input_buffer()
            s.write(b"\ndl\n")
            deadline = time.time() + 1.0
            while time.time() < deadline and b"OK dl" not in seen:
                seen = (seen + s.read(512))[-4096:]
        except serial.SerialTimeoutException:
            print(f"flash: attempt {attempt}: console not reading (write timeout)")
        except serial.SerialException:
            seen += b"OK dl"   # port vanished: the restart already happened
        finally:
            try:
                s.close()
            except serial.SerialException:
                pass
        if b"OK dl" in seen:
            print("flash: firmware acknowledged, waiting for the ROM loader")
            return True
        time.sleep(0.3)
    print("flash: no acknowledgement. Checking for the ROM loader;\n"
          "if it is hung, unplug it (all power) and plug it back in, then retry.")
    return False


def in_loader(port):
    """True when the ROM loader answers (the `dl` reply can be lost when USB
    re-enumerates, so a missing acknowledgement is checked this way)."""
    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32c5", "-p", port,
           "--before", "no-reset", "--after", "no-reset", "read-mac"]
    return subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def wait_for_port(port, timeout_s=6.0, settle_s=1.0):
    """The ROM loader re-enumerates on USB after the restart."""
    from serial.tools import list_ports
    deadline = time.monotonic() + timeout_s
    time.sleep(0.3)
    while time.monotonic() < deadline:
        if any(p.device.lower() == port.lower() for p in list_ports.comports()):
            break
        time.sleep(0.2)
    time.sleep(settle_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--build", type=Path, default=DEFAULT_BUILD)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--app-only", action="store_true", help="app image only (flash_app_args)")
    ap.add_argument("--no-dl", action="store_true", help="board is already in the ROM loader")
    a = ap.parse_args()

    args_file = "flash_app_args" if a.app_only else "flash_args"
    if not (a.build / args_file).exists():
        sys.exit(f"flash: {a.build / args_file} missing; run idf.py build in firmware/c5 first")
    if not a.no_dl and not enter_download(a.port):
        wait_for_port(a.port)
        if not in_loader(a.port):
            sys.exit(1)
        print("flash: the ROM loader answers; flashing")
    wait_for_port(a.port)
    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32c5", "-p", a.port, "-b", str(a.baud),
           "--before", "no-reset", "--after", "watchdog-reset", "write-flash", "@" + args_file]
    print("flash:", " ".join(cmd))
    sys.exit(subprocess.call(cmd, cwd=a.build))


if __name__ == "__main__":
    main()
