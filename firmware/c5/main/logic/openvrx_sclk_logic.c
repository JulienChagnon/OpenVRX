// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: sample clock arithmetic and the slip plan (pure logic).
#include "openvrx_sclk_logic.h"

uint32_t openvrx_clock_hz(uint32_t div)
{
    return div ? OPENVRX_PLL_HZ / div : 0u;
}

bool openvrx_div_bus_locked(uint32_t div)
{
    return div != 0u && div % OPENVRX_BUS_TICKS == 0u;
}

bool openvrx_slip_plan(uint32_t cur_div, openvrx_slip_plan_t *plan)
{
    if (cur_div == 0u || cur_div > 65536u || cur_div >= OPENVRX_SLIP_LONG_DIV) return false;
    const uint32_t l = OPENVRX_SLIP_LONG_DIV;
    plan->long_div = l;
    // The period running when L is written is N, or L + 1 when a previous
    // slip ended moments ago. Waiting longer than either guarantees that
    // the running period has wrapped and latched L.
    plan->settle_ticks = (cur_div > l + 1u ? cur_div : l + 1u) + OPENVRX_SLIP_MARGIN_TICKS;
    plan->edge_timeout_ticks = 3u * l;
    plan->stretch_at_ticks = l / 4u;
    plan->restore_at_ticks = l + l / 4u;
    return true;
}

uint32_t openvrx_ticks_to_cycles(uint32_t ticks, uint32_t cpu_hz)
{
    const uint64_t c = (uint64_t)ticks * cpu_hz + (OPENVRX_PLL_HZ - 1u);
    return (uint32_t)(c / OPENVRX_PLL_HZ);
}
