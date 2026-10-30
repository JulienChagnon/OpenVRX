// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: sample clock arithmetic and the slip plan (pure logic).
//
// The sample clock is the PARLIO TX core clock, PLL_F240M divided by an
// integer N (PCR_PARL_CLK_TX_DIV_NUM = N - 1; the C5 has no fractional
// part). The modem's MODEM_DIAG bus changes every 3 PLL ticks (80 MHz), so
// with N a multiple of 3 every clock edge keeps the same position (phase)
// against the bus: one of three, set by the sum of all past periods mod 3.
//
// Slip: the divider counter latches a new threshold only when it wraps
// (ESP-IDF parlio_tx.c: "The update of the threshold relies on the moment
// when the counter reaches the threshold each time"). One period of L + 1
// ticks among periods that are multiples of 3 moves every later edge by
// exactly one tick against the bus. Doing that with the real divider N
// would need a write window of one period (25 ns at N = 6), which software
// cannot hit. So the slip first switches to a long divider L (a multiple of
// 3, 5 us at L = 1200), finds a rising edge of the clock pad, and makes its
// two writes a quarter period away from any possible wrap:
//
//   t0         write L            (latched at the next wrap of N)
//   t0+settle  look for a rising edge r (all periods are now L)
//   r + L/4    write L + 1        (latched at the next wrap W1)
//   r + 5L/4   write N            (latched at W2 = W1 + L + 1)
//
// A divider's rising edge sits either on the wrap or half a period after it,
// so W1 is r + L/2 or r + L; both writes land at least L/4 (300 ticks,
// 1.25 us) from a wrap, far more than the polling latency. Exactly one
// period is L + 1; all others are N or L. Net shift: +1 tick per slip, so
// three slips walk through all three phases and return to the start.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OPENVRX_PLL_HZ 240000000u
#define OPENVRX_BUS_TICKS 3u            // PLL ticks per MODEM_DIAG bus sample
#define OPENVRX_SLIP_LONG_DIV 1200u     // L: multiple of 3, above every user divider
#define OPENVRX_SLIP_MARGIN_TICKS 64u

typedef struct {
    uint32_t long_div;            // L
    uint32_t settle_ticks;        // after writing L, before looking for an edge
    uint32_t edge_timeout_ticks;  // per level wait while looking for the edge
    uint32_t stretch_at_ticks;    // after the edge: write L + 1
    uint32_t restore_at_ticks;    // after the edge: write the user divider
} openvrx_slip_plan_t;

// Sample clock frequency for a divider (integer Hz, rounded down).
uint32_t openvrx_clock_hz(uint32_t div);
// True when the divider keeps a whole number of bus samples per period.
bool openvrx_div_bus_locked(uint32_t div);
// Builds the slip plan for the divider currently in use (1..65536).
bool openvrx_slip_plan(uint32_t cur_div, openvrx_slip_plan_t *plan);
// Converts PLL ticks to CPU cycles (rounded up).
uint32_t openvrx_ticks_to_cycles(uint32_t ticks, uint32_t cpu_hz);

#ifdef __cplusplus
}
#endif
