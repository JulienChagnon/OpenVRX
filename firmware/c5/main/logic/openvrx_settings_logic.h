// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: the persisted state blob (pure logic).
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "openvrx_boards.h"
#include "openvrx_lanes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPENVRX_SETTINGS_VERSION 2u
// Size of the version 1 blob (no agc byte), upgraded on load.
#define OPENVRX_SETTINGS_V1_SIZE 16u

// Stored as one NVS blob by `save`. Fixed layout, no pointers.
typedef struct {
    uint8_t version;
    uint8_t board;        // openvrx_board_id_t
    uint8_t preset;       // openvrx_preset_t or OPENVRX_PRESET_NONE
    uint8_t p11;          // 0/1
    uint16_t div;         // sample clock divider
    uint16_t mhz;         // tuned frequency
    int16_t off_khz;      // fine offset
    uint8_t gain;         // gain table index
    int8_t filter_code;   // OPENVRX_FILTER_CAL (-1) or 0..63
    uint8_t skirt;        // 0..60
    uint8_t bw_mhz;       // 20 or 40
    uint8_t drive_lanes;  // 0..3
    uint8_t drive_clk;    // 0..3
    uint8_t agc;          // 0/1, version 2
} openvrx_settings_t;

void openvrx_settings_default(openvrx_settings_t *s, openvrx_board_id_t board, openvrx_preset_t preset);
// Every field in range (the board fit of the preset is checked at boot).
bool openvrx_settings_valid(const openvrx_settings_t *s);
// Reads a stored blob of `len` bytes: the current layout, or a version 1
// blob upgraded with the AGC on. False when it is neither or invalid.
bool openvrx_settings_from_blob(const void *blob, unsigned len, openvrx_settings_t *out);

#ifdef __cplusplus
}
#endif
