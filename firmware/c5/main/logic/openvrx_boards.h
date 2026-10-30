// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: board pin maps and lane routing (pure logic).
//
// A board map fixes, per FX2 word bit, the C5 GPIO wired to that FX2 port
// pin. There is one table for 16-bit presets (word bits 0..15 = PB0..PB7,
// PD0..PD7) and one for 8-bit presets (word bits 0..7 = PB0..PB7). -1 means
// "no C5 pin": that FX2 pin is tied to GND on the board. A preset fits a
// board when every word bit it uses has a pin. The firmware then routes the
// ADC bit that belongs on each word bit to that pin. Pins of the active
// table that the preset does not use are driven low (their FX2 pin has no
// other driver in that wiring, so the FX2 reads 0 as with a GND tie). Pins
// that only appear in the other table stay high-impedance with a weak
// pull-down, so they never fight a wire left over from the other wiring.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "openvrx_lanes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OPENVRX_BOARD_XIAO = 0,
    OPENVRX_BOARD_ZERO,
    OPENVRX_BOARD_CUSTOM,
    OPENVRX_BOARD_COUNT,
} openvrx_board_id_t;

typedef struct {
    const char *name;
    int8_t clk;      // sample clock (FX2 IFCLK)
    int8_t w16[16];  // 16-bit presets: GPIO per FX2 word bit, -1 = tied to GND
    int8_t w8[8];    // 8-bit presets: GPIO per FX2 word bit, -1 = tied to GND
} openvrx_board_map_t;

#define OPENVRX_GPIO_COUNT 29u

typedef struct {
    openvrx_preset_t preset;
    unsigned n;                            // routed lanes
    openvrx_lane_t lane[OPENVRX_MAX_LANES];    // sorted by word bit ascending
    int8_t gpio[OPENVRX_MAX_LANES];          // GPIO for lane[k]
    int8_t clk;
    uint32_t used_mask;   // GPIOs carrying lanes
    uint32_t idle_mask;   // GPIOs of the active table not used by this preset: driven low
    uint32_t spare_mask;  // GPIOs only in the other table: high-impedance, pull-down
} openvrx_route_t;

typedef enum {
    OPENVRX_ROUTE_OK = 0,
    OPENVRX_ROUTE_UNSUPPORTED = -1, // preset cannot be routed from MODEM_DIAG (angle8)
    OPENVRX_ROUTE_NO_FIT = -2,      // the board has no pin for a needed word bit
    OPENVRX_ROUTE_BAD_MAP = -3,     // the board map is invalid
} openvrx_route_result_t;

// Built-in maps (XIAO, C5-Zero). OPENVRX_BOARD_CUSTOM returns NULL here; the
// firmware supplies the custom map from Kconfig.
const openvrx_board_map_t *openvrx_board_builtin(openvrx_board_id_t id);
const char *openvrx_board_name(openvrx_board_id_t id);
bool openvrx_board_parse(const char *name, openvrx_board_id_t *out);

// GPIOs the radio head may drive on any C5 board: 0..12 and 23..28.
// 13/14 are USB, 15..22 are flash/PSRAM.
bool openvrx_gpio_usable(int gpio);
// Checks GPIO numbers, duplicates within each table and against the clock.
// On failure writes a short reason (no spaces) into why.
bool openvrx_board_validate(const openvrx_board_map_t *m, char *why, size_t why_len);
// Distinct GPIOs the map uses (clock plus both tables).
unsigned openvrx_board_pin_count(const openvrx_board_map_t *m);
// Mask of every GPIO the map uses, clock included.
uint32_t openvrx_board_gpio_mask(const openvrx_board_map_t *m);
// Routes a preset on a board. On OPENVRX_ROUTE_NO_FIT, *missing_word_bit (if
// not NULL) receives the first word bit without a pin.
openvrx_route_result_t openvrx_route(const openvrx_board_map_t *m, openvrx_preset_t p, openvrx_route_t *r,
                                 int *missing_word_bit);
bool openvrx_board_fits(const openvrx_board_map_t *m, openvrx_preset_t p);
// Largest preset (most data pins) that fits, or OPENVRX_PRESET_NONE.
openvrx_preset_t openvrx_board_default_preset(const openvrx_board_map_t *m);
// FX2 pin name of a word bit for a word width: "PB3", "PD7".
const char *openvrx_fx2_pin_name(unsigned word_bit);

#ifdef __cplusplus
}
#endif
