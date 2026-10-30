// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: board pin maps and lane routing (pure logic).
#include "openvrx_boards.h"

#include <stdio.h>
#include <string.h>

// Seeed XIAO ESP32-C5. Front header, USB at the top: left side D0..D6 =
// GPIO1, 0, 25, 7, 23, 24, 11; right side 5V, GND, 3V3, D10..D7 = GPIO10, 9,
// 8, 12. Back pads MTMS/MTDI/MTCK/MTDO = GPIO2/3/4/5. GPIO27 (user LED),
// GPIO28 (BOOT), GPIO6/26 (battery sense) and GPIO13/14 (USB) stay free.
// Clock on D10 (GPIO10): the header pin closest to GND, not a strapping pin.
// 16-bit: Q bits 9..5 on D0..D4, I bits 9..5 on D5..D9 (all 5x5 lanes on
// front pins), bits 4 and 3 on the back pads (6x6, 7x7).
// 8-bit (4x4, front pins only): D0..D3 keep PB7..PB4 (now I), D4 keeps PB3
// (now Q MSB) and D5..D7 move to PB2..PB0.
static const openvrx_board_map_t s_xiao = {
    .name = "xiao",
    .clk = 10,
    .w16 = {-1, 3, 2, 23, 7, 25, 0, 1,     // PB0..PB7: Q (PB0 tied to GND)
            -1, 5, 4, 9, 8, 12, 11, 24},   // PD0..PD7: I (PD0 tied to GND)
    .w8 = {12, 11, 24, 23, 7, 25, 0, 1},   // PB0..PB3 Q, PB4..PB7 I
};

// Waveshare ESP32-C5-Zero. Edge, USB at the top: left side 5V, GND, 3V3,
// GPIO0..5; right side GPIO11, 12, 13 (USB D-), 14 (USB D+), 10, 9, 8, 7, 6.
// Back test pads GPIO23, 24, 25, 28. GPIO26 is the antenna switch and
// GPIO27 the RGB LED: both stay untouched. GPIO11 reaches its pad through a
// 499 ohm resistor (R9 on the schematic), so it carries the least
// significant I lane (8x8 only). GPIO28 has the BOOT button and a 10k
// pull-up: least significant Q lane.
// Clock on GPIO0: the GPIO next to the 3V3/GND group, not a strapping pin.
// One wiring serves every preset: the 8-bit table is the PB half of the
// 16-bit table, so 8x8 and the 4x4 presets need no rewiring.
static const openvrx_board_map_t s_zero = {
    .name = "zero",
    .clk = 0,
    .w16 = {28, 24, 12, 5, 4, 3, 2, 1,     // PB0..PB7: Q bits 2..9
            11, 25, 23, 6, 7, 8, 9, 10},   // PD0..PD7: I bits 2..9
    .w8 = {28, 24, 12, 5, 4, 3, 2, 1},
};

static const char *const s_board_names[OPENVRX_BOARD_COUNT] = {"xiao", "zero", "custom"};

const openvrx_board_map_t *openvrx_board_builtin(openvrx_board_id_t id)
{
    switch (id) {
    case OPENVRX_BOARD_XIAO: return &s_xiao;
    case OPENVRX_BOARD_ZERO: return &s_zero;
    default: return NULL;
    }
}

const char *openvrx_board_name(openvrx_board_id_t id)
{
    return (unsigned)id < OPENVRX_BOARD_COUNT ? s_board_names[id] : "?";
}

bool openvrx_board_parse(const char *name, openvrx_board_id_t *out)
{
    if (!name) return false;
    for (unsigned k = 0; k < OPENVRX_BOARD_COUNT; ++k)
        if (strcmp(name, s_board_names[k]) == 0) {
            *out = (openvrx_board_id_t)k;
            return true;
        }
    return false;
}

bool openvrx_gpio_usable(int gpio)
{
    return (gpio >= 0 && gpio <= 12) || (gpio >= 23 && gpio <= 28);
}

static bool check_table(const int8_t *t, unsigned n, int clk, char *why, size_t why_len, const char *name)
{
    uint32_t seen = 0;
    for (unsigned k = 0; k < n; ++k) {
        const int g = t[k];
        if (g < 0) continue;
        if (!openvrx_gpio_usable(g)) {
            snprintf(why, why_len, "%s_bit%u_gpio%d_not_usable", name, k, g);
            return false;
        }
        if (g == clk) {
            snprintf(why, why_len, "%s_bit%u_gpio%d_is_clock", name, k, g);
            return false;
        }
        if (seen & (1u << g)) {
            snprintf(why, why_len, "%s_gpio%d_used_twice", name, g);
            return false;
        }
        seen |= 1u << g;
    }
    return true;
}

bool openvrx_board_validate(const openvrx_board_map_t *m, char *why, size_t why_len)
{
    char scratch[8];
    if (!why || !why_len) { why = scratch; why_len = sizeof(scratch); }
    why[0] = 0;
    if (!m) {
        snprintf(why, why_len, "no_map");
        return false;
    }
    if (!openvrx_gpio_usable(m->clk)) {
        snprintf(why, why_len, "clock_gpio%d_not_usable", m->clk);
        return false;
    }
    if (!check_table(m->w16, 16, m->clk, why, why_len, "w16")) return false;
    if (!check_table(m->w8, 8, m->clk, why, why_len, "w8")) return false;
    bool any = false;
    for (unsigned k = 0; k < 16; ++k) any |= m->w16[k] >= 0;
    for (unsigned k = 0; k < 8; ++k) any |= m->w8[k] >= 0;
    if (!any) {
        snprintf(why, why_len, "no_lane_pins");
        return false;
    }
    return true;
}

uint32_t openvrx_board_gpio_mask(const openvrx_board_map_t *m)
{
    uint32_t mask = 0;
    if (!m) return 0;
    if (m->clk >= 0) mask |= 1u << m->clk;
    for (unsigned k = 0; k < 16; ++k) if (m->w16[k] >= 0) mask |= 1u << m->w16[k];
    for (unsigned k = 0; k < 8; ++k) if (m->w8[k] >= 0) mask |= 1u << m->w8[k];
    return mask;
}

unsigned openvrx_board_pin_count(const openvrx_board_map_t *m)
{
    uint32_t v = openvrx_board_gpio_mask(m);
    unsigned n = 0;
    while (v) { v &= v - 1u; ++n; }
    return n;
}

openvrx_route_result_t openvrx_route(const openvrx_board_map_t *m, openvrx_preset_t p, openvrx_route_t *r,
                                 int *missing_word_bit)
{
    if (missing_word_bit) *missing_word_bit = -1;
    memset(r, 0, sizeof(*r));
    r->preset = OPENVRX_PRESET_NONE;
    r->clk = -1;
    if (!openvrx_board_validate(m, NULL, 0)) return OPENVRX_ROUTE_BAD_MAP;
    r->clk = m->clk;
    if (p == OPENVRX_PRESET_NONE) {
        // Nothing routed: every lane pin of the board is a spare.
        r->spare_mask = openvrx_board_gpio_mask(m) & ~(1u << m->clk);
        return OPENVRX_ROUTE_OK;
    }
    if (!openvrx_preset_is_iq(p)) return OPENVRX_ROUTE_UNSUPPORTED;
    openvrx_lane_t lanes[OPENVRX_MAX_LANES];
    const unsigned n = openvrx_preset_lanes(p, lanes);
    const bool wide = openvrx_preset_word_bytes(p) == 2u;
    const int8_t *table = wide ? m->w16 : m->w8;
    const unsigned table_n = wide ? 16u : 8u;
    for (unsigned k = 0; k < n; ++k) {
        const int g = table[lanes[k].word_bit];
        if (g < 0) {
            if (missing_word_bit) *missing_word_bit = lanes[k].word_bit;
            memset(r, 0, sizeof(*r));
            r->preset = OPENVRX_PRESET_NONE;
            r->clk = m->clk;
            return OPENVRX_ROUTE_NO_FIT;
        }
        r->lane[k] = lanes[k];
        r->gpio[k] = (int8_t)g;
        r->used_mask |= 1u << g;
    }
    r->n = n;
    r->preset = p;
    uint32_t active = 0;
    for (unsigned k = 0; k < table_n; ++k) if (table[k] >= 0) active |= 1u << table[k];
    r->idle_mask = active & ~r->used_mask;
    r->spare_mask = openvrx_board_gpio_mask(m) & ~active & ~(1u << m->clk);
    return OPENVRX_ROUTE_OK;
}

bool openvrx_board_fits(const openvrx_board_map_t *m, openvrx_preset_t p)
{
    openvrx_route_t r;
    return openvrx_route(m, p, &r, NULL) == OPENVRX_ROUTE_OK && p != OPENVRX_PRESET_NONE;
}

openvrx_preset_t openvrx_board_default_preset(const openvrx_board_map_t *m)
{
    // Presets are ordered from most to fewest data pins.
    for (unsigned k = 0; k < OPENVRX_PRESET_COUNT; ++k)
        if (openvrx_board_fits(m, (openvrx_preset_t)k)) return (openvrx_preset_t)k;
    return OPENVRX_PRESET_NONE;
}

const char *openvrx_fx2_pin_name(unsigned word_bit)
{
    static const char *const names[16] = {
        "PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7",
        "PD0", "PD1", "PD2", "PD3", "PD4", "PD5", "PD6", "PD7",
    };
    return word_bit < 16u ? names[word_bit] : "?";
}
