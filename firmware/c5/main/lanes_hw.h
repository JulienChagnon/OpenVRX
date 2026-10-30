// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane pads (MODEM_DIAG routing and the self-test).
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "openvrx_boards.h"
#include "openvrx_cmd.h"

esp_err_t lanes_hw_init(void);
// Applies a route: lanes carry their MODEM_DIAG bus bits, idle pins are
// driven low, spare pins become inputs with pull-down, and pins of the
// previous route that the new one does not use are released. A running
// self-test stops. drive: 0..3 for the lane pads.
esp_err_t lanes_hw_apply(const openvrx_route_t *r, unsigned drive);
const openvrx_route_t *lanes_hw_route(void);
esp_err_t lanes_hw_set_drive(unsigned drive);
// Self-test: lanes become plain GPIO outputs (detached from MODEM_DIAG).
// OPENVRX_ST_OFF re-attaches MODEM_DIAG.
esp_err_t lanes_hw_selftest(openvrx_selftest_mode_t mode, uint32_t value, uint32_t phase_us);
openvrx_selftest_mode_t lanes_hw_selftest_mode(void);
uint32_t lanes_hw_selftest_phase_us(void);
const char *lanes_hw_selftest_name(openvrx_selftest_mode_t mode);
