// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane presets (pure logic, also built on the host).
//
// Mirrors openvrx_core::LanePreset (host/crates/openvrx-core/src/lanes.rs):
// MODEM_DIAG carries Q on bus bits 0..9 and I on bus bits 10..19 (10-bit
// two's complement ADC values). Each preset lists which ADC bits travel and
// the FX2 word bit each one lands on.
//
//   16-bit words: Q MSB-aligned on word bit 7 downward (FX2 PB7..PB0),
//                 I MSB-aligned on word bit 15 downward (FX2 PD7..PD0).
//   8-bit words (4x4): Q on word bits 3..0, I on word bits 7..4.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OPENVRX_PRESET_8X8 = 0,
    OPENVRX_PRESET_7X7,
    OPENVRX_PRESET_6X6,
    OPENVRX_PRESET_5X5,
    OPENVRX_PRESET_4X4TOP,
    OPENVRX_PRESET_4X4FINE,
    OPENVRX_PRESET_4X4ULTRA,
    OPENVRX_PRESET_ANGLE8,
    OPENVRX_PRESET_COUNT,
    OPENVRX_PRESET_NONE = 0xff, // no lanes routed (board does not fit)
} openvrx_preset_t;

#define OPENVRX_MAX_LANES 16u

typedef struct {
    uint8_t word_bit; // FX2 word bit (0..15); 0..7 = PB0..PB7, 8..15 = PD0..PD7
    uint8_t adc_bit;  // ADC bit 0..9 of the component
    uint8_t bus_bit;  // MODEM_DIAG bus bit: adc_bit for Q, 10 + adc_bit for I
    char comp;        // 'I' or 'Q'
} openvrx_lane_t;

// Preset name as used by the console and lanes.rs ("8x8", "4x4ultra", ...).
const char *openvrx_preset_name(openvrx_preset_t p);
// Case-insensitive name lookup. Returns false for an unknown name.
bool openvrx_preset_parse(const char *name, openvrx_preset_t *out);
// 1 (8-bit FX2 port) or 2 (16-bit), 0 for NONE.
unsigned openvrx_preset_word_bytes(openvrx_preset_t p);
// Data pins the preset needs (the sample clock needs one more).
unsigned openvrx_preset_data_pins(openvrx_preset_t p);
// True when the preset is an I/Q preset the radio head can route directly
// from MODEM_DIAG (false for angle8 and NONE).
bool openvrx_preset_is_iq(openvrx_preset_t p);
// Fills out[] with the preset's lanes sorted by FX2 word bit ascending (the
// self-test lane order of PROTOCOL.md). Returns the lane count (0 for
// angle8 and NONE).
unsigned openvrx_preset_lanes(openvrx_preset_t p, openvrx_lane_t out[OPENVRX_MAX_LANES]);
// Mask of FX2 word bits the preset uses.
uint32_t openvrx_preset_word_mask(openvrx_preset_t p);

#ifdef __cplusplus
}
#endif
