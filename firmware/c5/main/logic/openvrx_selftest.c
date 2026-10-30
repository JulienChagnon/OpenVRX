// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane self-test patterns (pure logic).
#include "openvrx_selftest.h"

static uint32_t lane_mask(unsigned n)
{
    return n >= 32u ? UINT32_MAX : (1u << n) - 1u;
}

uint32_t openvrx_walk_phases(unsigned n_lanes)
{
    return 2u + n_lanes;
}

uint32_t openvrx_selftest_bits(openvrx_selftest_mode_t mode, uint32_t phase, unsigned n_lanes, uint32_t value)
{
    const uint32_t mask = lane_mask(n_lanes);
    switch (mode) {
    case OPENVRX_ST_WALK: {
        const uint32_t p = phase % openvrx_walk_phases(n_lanes);
        if (p == 0) return 0;
        if (p == 1) return mask;
        return 1u << (p - 2u);
    }
    case OPENVRX_ST_COUNT:
        return phase & mask;
    case OPENVRX_ST_CONST:
        return value & mask;
    default:
        return 0;
    }
}

bool openvrx_selftest_const_fits(uint32_t value, unsigned n_lanes)
{
    return (value & ~lane_mask(n_lanes)) == 0;
}

uint32_t openvrx_lane_bits_to_gpio(const openvrx_route_t *r, uint32_t bits)
{
    uint32_t out = 0;
    for (unsigned k = 0; k < r->n; ++k)
        if ((bits >> k) & 1u) out |= 1u << (unsigned)r->gpio[k];
    return out;
}

uint32_t openvrx_lane_bits_to_word(const openvrx_route_t *r, uint32_t bits)
{
    uint32_t out = 0;
    for (unsigned k = 0; k < r->n; ++k)
        if ((bits >> k) & 1u) out |= 1u << r->lane[k].word_bit;
    return out;
}
