#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Send console commands to an OpenVRX C5 and print the replies.

The port is opened with DTR and RTS low before the open, so neither opening
nor closing it resets the chip (a line reset on a running image can leave
the C5 hung in the ROM loader; see tools/flash.py).

    python tools/c5cmd.py COM9 ver status pins
    python tools/c5cmd.py COM9 "tune 5880" "lanes 4x4top" "clk 9" save status

Needs pyserial (the ESP-IDF Python environment has it).
"""
import sys
import time

import serial

SLOW = {"selftest": 3.0, "save": 1.5, "tune": 1.5}


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = sys.argv[1], 115200, 0.1, 2
    if sys.platform == "win32":
        # Windows applies these before the lines change: they never go high.
        s.dtr = False
        s.rts = False
        s.open()
    else:
        # Linux raises DTR and RTS together on open (idle on the C5), and
        # pySerial then applies dtr=False first: DTR low with RTS high is the
        # USB Serial/JTAG reset, which hangs a running C5. So open with the
        # defaults (no change after open) and drop RTS before DTR.
        s.open()
        s.rts = False
        s.dtr = False
    try:
        s.reset_input_buffer()
        for c in sys.argv[2:]:
            s.write((c + "\n").encode())
            word = c.split()[0]
            deadline = time.time() + SLOW.get(word, 1.0)
            out = b""
            # A command gets exactly one OK/ERR reply; stop once it arrives.
            while time.time() < deadline:
                out += s.read(4096)
                # Whole lines only, so a reply still arriving is not cut off.
                lines = out.decode(errors="replace").split("\n")[:-1]
                if any(l.startswith(("OK " + word, "ERR " + word)) for l in lines):
                    break
            print(f">>> {c}\n{out.decode(errors='replace').rstrip()}")
    finally:
        s.close()


if __name__ == "__main__":
    main()
