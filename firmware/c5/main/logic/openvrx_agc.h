// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain from the lane pads (pure logic).
//
// The vendor AGC stays off (it hunts on the analog FM carrier). Instead the
// firmware reads the lane pads back through GPIO_IN at random moments: each
// read is one sample word as the FX2 sees it. From a few thousand reads the
// loop knows how often I and Q sit at full scale and their RMS in steps of
// the lowest carried bit, and moves the forced gain index towards a level
// that uses the carried bits without clipping. Works in every mode, also
// FX2-only where the PC has no link to the C5.
//
// Only presets that carry the top ADC bits (8x8 to 4x4top) have a monotonic
// value on the pads. The folded presets (4x4fine, 4x4ultra) wrap around when
// the signal is too big, and angle8 has no level: the loop holds the gain
// for those.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "openvrx_lanes.h"

#ifdef __cplusplus
extern "C" {
#endif

// Loop period and pad reads per update.
#define OPENVRX_AGC_PERIOD_MS 100u
#define OPENVRX_AGC_READS 4096u
// Target RMS per component as a share of full scale (2^(bits-1) steps):
// 0.55 puts the FM envelope just under the clip level (4x4top: 4.4 steps).
#define OPENVRX_AGC_TARGET_SHARE 0.55f
// No change while the level is within this many dB of the target.
#define OPENVRX_AGC_DEADBAND_DB 1.5f
// Clipped share of component samples that counts as overload whatever the
// RMS says (clipping caps the RMS near full scale).
#define OPENVRX_AGC_OVERLOAD_SHARE 0.25f
// Gain is only raised while fewer component samples than this clip.
#define OPENVRX_AGC_RAISE_MAX_CLIP 0.05f
// Assumed dB per gain index until per-board tables are measured.
#define OPENVRX_AGC_DB_PER_INDEX 0.5f
// Largest index change per update (2 dB per 100 ms = 20 dB/s).
#define OPENVRX_AGC_MAX_STEP 4

// How to turn a pad word into I and Q: one entry per routed lane.
typedef struct {
    unsigned n;                         // lanes
    uint8_t gpio[OPENVRX_MAX_LANES];
    int16_t weight[OPENVRX_MAX_LANES];  // signed value of the bit in steps (sign bit negative)
    uint8_t is_i[OPENVRX_MAX_LANES];
    unsigned bits;                      // carried bits per component
} openvrx_agc_plan_t;

// Builds the plan for a routed preset (lanes and their GPIOs). False when
// the preset cannot be measured (folded, angle8, NONE, or I and Q of
// different widths).
bool openvrx_agc_plan(openvrx_preset_t preset, const openvrx_lane_t *lanes, const int8_t *gpio, unsigned n,
                      openvrx_agc_plan_t *plan);

typedef struct {
    uint32_t values;   // component values seen (2 per read)
    uint32_t clips;    // at the most negative or most positive code
    uint64_t sumsq4;   // sum of (2 v + 1)^2: four times the half-step centred square
} openvrx_agc_acc_t;

void openvrx_agc_acc_reset(openvrx_agc_acc_t *a);
// Adds one pad read (GPIO_IN value).
void openvrx_agc_acc_add(openvrx_agc_acc_t *a, const openvrx_agc_plan_t *plan, uint32_t gpio_in);
// Clipped share and RMS per component in steps.
float openvrx_agc_clip_share(const openvrx_agc_acc_t *a);
float openvrx_agc_rms_steps(const openvrx_agc_acc_t *a);

// Next gain index from the measured level, within [gain_min, gain_max].
unsigned openvrx_agc_next(unsigned gain, unsigned gain_min, unsigned gain_max, unsigned bits, float clip_share,
                          float rms_steps);

#ifdef __cplusplus
}
#endif
