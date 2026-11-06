// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: the att_ctl console over Wi-Fi as well as USB, so the ESP32
// can run from a power bank with no cable to the PC (a USB cable joins the
// VTX ground to the PC and the receiver: a leak path, docs/bench-logs BENCH-7).
//
// TCP port 2323, one client at a time, the same line commands as USB
// (pyserial: socket://openvrx-bench.local:2323). Wi-Fi settings live only in
// the ESP32's flash (set over USB with tools/bench/esp32_wifi.py):
//   wifissid <name> | wifipass <password> | wifi [on|off|forget]
#pragma once
#include <Arduino.h>

// Replies and events: to USB and, when connected, the TCP client.
extern Print &OUT;

// Starts Wi-Fi from the saved settings (if any) and the TCP server.
void net_begin();
// Tracks the connection, accepts a client, feeds its lines to handle().
void net_poll(void (*handle)(String));
// True while a TCP line is half received (do not block the loop then).
bool net_line_pending();
// Handles the wifi commands; false if cmd is not one of them.
bool net_command(const String &cmd, const String &arg);
