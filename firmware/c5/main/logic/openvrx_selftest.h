// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane self-test patterns (pure logic).
//
// docs/PROTOCOL.md: lane j is the j-th lane of the preset sorted by FX2 word
// bit ascending. walk: phase 0 all low, phase 1 all high, phase 2+j only
// lane j high, repeating. count: lane j carries bit j of a counter that
// increments every phase. const: bit j of the value drives lane j.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "openvrx_boards.h"
#include "openvrx_cmd.h"

#ifdef __cplusplus
extern "C" {
#endif

// Phases in one walk cycle for n lanes (2 + n).
uint32_t openvrx_walk_phases(unsigned n_lanes);
// Lane bits for a phase. For walk, phase is taken modulo the cycle; for
// count, phase is the counter value; for const, value is used.
uint32_t openvrx_selftest_bits(openvrx_selftest_mode_t mode, uint32_t phase, unsigned n_lanes, uint32_t value);
// True when a const value only uses lanes that exist.
bool openvrx_selftest_const_fits(uint32_t value, unsigned n_lanes);
// GPIO output levels (bit g = level of GPIO g) for lane bits on a route.
uint32_t openvrx_lane_bits_to_gpio(const openvrx_route_t *r, uint32_t bits);
// The FX2 word the host should read for lane bits on a route.
uint32_t openvrx_lane_bits_to_word(const openvrx_route_t *r, uint32_t bits);

#ifdef __cplusplus
}
#endif
