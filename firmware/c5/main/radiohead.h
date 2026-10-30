// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: device state and console command execution.
#pragma once

#include "esp_err.h"
#include "openvrx_settings_logic.h"

// Applies a state at boot: pin map, sample clock, lanes, radio.
void radiohead_boot(const openvrx_settings_t *s);
// Default state for a fresh board (Kconfig default board, largest preset).
void radiohead_defaults(openvrx_settings_t *s);
// Console line handler: exactly one reply line per call.
void radiohead_command(char *line);
