// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane presets (pure logic, also built on the host).
#include "openvrx_lanes.h"

#include <stddef.h>

typedef struct {
    const char *name;
    uint8_t word_bytes;
    uint8_t bits;          // ADC bits per component (0 = not an I/Q preset)
    uint8_t adc[8];        // ADC bits, most significant first
} preset_def_t;

static const preset_def_t s_presets[OPENVRX_PRESET_COUNT] = {
    [OPENVRX_PRESET_8X8] = {"8x8", 2, 8, {9, 8, 7, 6, 5, 4, 3, 2}},
    [OPENVRX_PRESET_7X7] = {"7x7", 2, 7, {9, 8, 7, 6, 5, 4, 3}},
    [OPENVRX_PRESET_6X6] = {"6x6", 2, 6, {9, 8, 7, 6, 5, 4}},
    [OPENVRX_PRESET_5X5] = {"5x5", 2, 5, {9, 8, 7, 6, 5}},
    [OPENVRX_PRESET_4X4TOP] = {"4x4top", 1, 4, {9, 8, 7, 6}},
    [OPENVRX_PRESET_4X4FINE] = {"4x4fine", 1, 4, {9, 7, 6, 5}},
    [OPENVRX_PRESET_4X4ULTRA] = {"4x4ultra", 1, 4, {9, 6, 5, 4}},
    [OPENVRX_PRESET_ANGLE8] = {"angle8", 1, 0, {0}},
};

static char lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

const char *openvrx_preset_name(openvrx_preset_t p)
{
    if (p == OPENVRX_PRESET_NONE) return "none";
    return (unsigned)p < OPENVRX_PRESET_COUNT ? s_presets[p].name : "?";
}

bool openvrx_preset_parse(const char *name, openvrx_preset_t *out)
{
    if (!name) return false;
    for (unsigned k = 0; k < OPENVRX_PRESET_COUNT; ++k) {
        const char *a = s_presets[k].name, *b = name;
        while (*a && *b && lower(*a) == lower(*b)) { ++a; ++b; }
        if (!*a && !*b) {
            *out = (openvrx_preset_t)k;
            return true;
        }
    }
    return false;
}

unsigned openvrx_preset_word_bytes(openvrx_preset_t p)
{
    return (unsigned)p < OPENVRX_PRESET_COUNT ? s_presets[p].word_bytes : 0u;
}

unsigned openvrx_preset_data_pins(openvrx_preset_t p)
{
    if ((unsigned)p >= OPENVRX_PRESET_COUNT) return 0u;
    if (p == OPENVRX_PRESET_ANGLE8) return 8u;
    return 2u * s_presets[p].bits;
}

bool openvrx_preset_is_iq(openvrx_preset_t p)
{
    return (unsigned)p < OPENVRX_PRESET_COUNT && s_presets[p].bits != 0u;
}

unsigned openvrx_preset_lanes(openvrx_preset_t p, openvrx_lane_t out[OPENVRX_MAX_LANES])
{
    if (!openvrx_preset_is_iq(p)) return 0u;
    const preset_def_t *d = &s_presets[p];
    // Word MSB of each component (lanes.rs: iq16 -> Q 7, I 15; iq8 -> Q 3, I 7).
    const unsigned q_msb = d->word_bytes == 2u ? 7u : 3u;
    const unsigned i_msb = d->word_bytes == 2u ? 15u : 7u;
    unsigned n = 0;
    // Q first (lower word bits), each component from its LSB upward, so the
    // result is ascending by word bit.
    for (unsigned c = 0; c < 2u; ++c) {
        const unsigned msb = c == 0 ? q_msb : i_msb;
        for (unsigned k = d->bits; k-- > 0;) {
            openvrx_lane_t *l = &out[n++];
            l->word_bit = (uint8_t)(msb - k);
            l->adc_bit = d->adc[k];
            l->comp = c == 0 ? 'Q' : 'I';
            l->bus_bit = (uint8_t)(c == 0 ? d->adc[k] : 10u + d->adc[k]);
        }
    }
    return n;
}

uint32_t openvrx_preset_word_mask(openvrx_preset_t p)
{
    openvrx_lane_t lanes[OPENVRX_MAX_LANES];
    const unsigned n = openvrx_preset_lanes(p, lanes);
    uint32_t mask = 0;
    for (unsigned k = 0; k < n; ++k) mask |= 1u << lanes[k].word_bit;
    if (p == OPENVRX_PRESET_ANGLE8) mask = 0xffu;
    return mask;
}
