// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: USB Serial/JTAG console (docs/PROTOCOL.md section 1).
#pragma once

#include <stdarg.h>
#include "esp_err.h"

// Installs the USB Serial/JTAG driver, routes stdout and ESP-IDF logging
// through it (log lines get a "LOG " prefix) and starts the line reader.
// handler runs in the console task for every complete line and must emit
// exactly one reply with console_reply().
esp_err_t console_start(void (*handler)(char *line));

// Writes one reply line ("OK ..." / "ERR ..."); a newline is appended.
void console_reply(const char *line);
// Unsolicited lines.
void console_evt(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void console_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
// Blocks until queued output has left (bounded), e.g. before a reset.
void console_flush(void);
