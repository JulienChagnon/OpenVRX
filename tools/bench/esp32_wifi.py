#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Puts the bench ESP32 (att_ctl) on Wi-Fi, over its USB console.

Asks for the network name (unless --ssid) and the password (hidden), sends
them to the ESP32, which keeps them in its own flash only, then waits for it
to connect and prints the address to use instead of the USB port:

    python3 tools/bench/esp32_wifi.py --ssid MyNetwork
    python3 tools/bench/bench.py ... --att esp32:socket://openvrx-bench.local:2323

Run it in your own terminal: the password is read with a hidden prompt and
is never printed. The ESP32 must be on USB for this step; afterwards it can
run from a power bank. 2.4 GHz networks only (the ESP32 has no 5 GHz).

    --port /dev/ttyUSB0   USB console of the ESP32 (default)
    --status              only show the current Wi-Fi state
    --forget              erase the saved network and turn Wi-Fi off
"""
import argparse
import getpass
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import command, open_serial  # noqa: E402  (same folder)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--ssid", default=None, help="network name (asked for if not given)")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--forget", action="store_true")
    args = ap.parse_args()

    s = open_serial(args.port)
    time.sleep(0.3)
    s.write(b"\n")  # end any half line left in the ESP32's buffer
    time.sleep(0.1)
    s.reset_input_buffer()

    if args.status:
        print(command(s, "wifi", "wifi", 3.0))
        return
    if args.forget:
        print(command(s, "wifi forget", "wifi", 5.0))
        return

    ssid = args.ssid or input("Wi-Fi name (2.4 GHz): ").strip()
    password = getpass.getpass(f"Password for {ssid} (hidden): ")
    if not ssid or len(ssid) > 32:
        sys.exit("the name must be 1 to 32 characters")
    if len(password) > 63:
        sys.exit("the password must be at most 63 characters")
    if password != password.strip():
        sys.exit("passwords with leading or trailing spaces are not supported")

    command(s, f"wifissid {ssid}", "wifissid", 3.0)
    command(s, f"wifipass {password}", "wifipass", 3.0)
    del password
    command(s, "wifi on", "wifi", 5.0)
    print(f"saved on the ESP32; connecting to {ssid}...")

    deadline = time.time() + 30
    while time.time() < deadline:
        state = command(s, "wifi", "wifi", 3.0)
        if "status=connected" in state:
            fields = dict(kv.split("=", 1) for kv in state.split()[2:] if "=" in kv)
            print(f"connected: ip {fields.get('ip')}, signal {fields.get('rssi')} dBm")
            print(f"use: --att esp32:socket://{fields.get('host')}:{fields.get('port')}")
            print(f" or: --att esp32:socket://{fields.get('ip')}:{fields.get('port')}")
            return
        time.sleep(1.0)
    sys.exit("not connected after 30 s: check the name and password (a 2.4 GHz network), "
             "then run this again")


if __name__ == "__main__":
    main()
