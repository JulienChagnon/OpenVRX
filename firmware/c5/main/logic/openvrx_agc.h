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

// Fast attack (C5-4). Flight fades swing 20 to 30 dB within a fraction of
// a second and the 100 ms loop, at most 4 indexes a step, left 30 to 95 %
// of the values clipped for seconds after a fade (2026-11-04 flights).
// Every OPENVRX_AGC_FAST_MS a short batch of pad reads checks clipping;
// at OPENVRX_AGC_ATTACK_CLIP or more the gain drops OPENVRX_AGC_ATTACK_STEP
// indexes, again after OPENVRX_AGC_ATTACK_GAP_MS while it still clips. The
// dB per index is not known (C5-3: one bench case moved 28 dB in 5
// indexes), so the attack is closed-loop: it measures after every step
// instead of computing one. Raising stays with the 100 ms loop, held off
// for OPENVRX_AGC_RAISE_HOLD_MS after an attack so it does not pump back
// into the clipping. C5VRX found the picture follows how often the gain
// is written (about 7 ms of picture per write), so attacks are few and
// large rather than many and small.
#define OPENVRX_AGC_FAST_MS 5u
#define OPENVRX_AGC_FAST_READS 256u
// 10 %: at the loop's target the FM envelope sits near 0.78 of full scale
// and clips almost nothing; interference bursts on the board read 2-6 %.
// At 20 % the attack stopped with 5-20 % left for the next 100 ms update.
#define OPENVRX_AGC_ATTACK_CLIP 0.10f
// Consecutive hot batches before the first attack. Board 2026-11-04 with a
// single hot batch: about 2 attacks a second with no VTX at all (gain
// dragged 83 -> 65 and back): short interference bursts (most likely
// 5 GHz Wi-Fi packets, 2-6 % of a 4096-read measurement) fill a 256-read
// batch of about 0.1 ms. Three batches are 10 ms of clipping, longer than
// a packet; a fade recovery clips for hundreds of ms. Once confirmed, the
// attack repeats every OPENVRX_AGC_ATTACK_GAP_MS while the clipping lasts.
#define OPENVRX_AGC_ATTACK_CONFIRM 3u
#define OPENVRX_AGC_ATTACK_STEP 6
#define OPENVRX_AGC_ATTACK_GAP_MS 10u
#define OPENVRX_AGC_RAISE_HOLD_MS 500u

// Loop state across the fast checks and the 100 ms updates.
typedef struct {
    bool attacked;          // an attack happened (the times below are valid)
    uint32_t last_attack_ms;
    unsigned hot;           // consecutive fast batches at or above OPENVRX_AGC_ATTACK_CLIP
    uint32_t attacks;       // count, for the status line
} openvrx_agc_ctl_t;

void openvrx_agc_ctl_init(openvrx_agc_ctl_t *c);
// Fast check at now_ms on a short batch: the gain to set (== gain: none).
unsigned openvrx_agc_fast(openvrx_agc_ctl_t *c, uint32_t now_ms, unsigned gain, unsigned gain_min, float clip_share);
// 100 ms update: openvrx_agc_next with raises held off after an attack.
unsigned openvrx_agc_slow(const openvrx_agc_ctl_t *c, uint32_t now_ms, unsigned gain, unsigned gain_min,
                          unsigned gain_max, unsigned bits, float clip_share, float rms_steps);

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
    int32_t sum2[2];   // per axis (0 = I, 1 = Q): sum of 2 v + 1, twice the half-step centred value
    uint32_t neg[2];   // per axis: values with the sign bit set
} openvrx_agc_acc_t;

void openvrx_agc_acc_reset(openvrx_agc_acc_t *a);
// Adds one pad read (GPIO_IN value).
void openvrx_agc_acc_add(openvrx_agc_acc_t *a, const openvrx_agc_plan_t *plan, uint32_t gpio_in);
// Clipped share and RMS per component in steps. rms_steps includes the DC
// (what the gain loop used before DC correction existed); rms_ac_steps is
// taken around each axis' mean, so an offset does not count as signal.
float openvrx_agc_clip_share(const openvrx_agc_acc_t *a);
float openvrx_agc_rms_steps(const openvrx_agc_acc_t *a);
float openvrx_agc_rms_ac_steps(const openvrx_agc_acc_t *a);
// Mean of one axis (0 = I, 1 = Q) in steps, half-step centred (a value v
// counts as v + 0.5, the middle of its quantiser cell). Clipping pulls it
// towards zero but never across.
float openvrx_agc_mean_steps(const openvrx_agc_acc_t *a, unsigned axis);
// Share of one axis' values below zero (sign bit set). The sign bit is the
// ADC's own: on a top-bit layout it follows the true DC at any level.
float openvrx_agc_neg_share(const openvrx_agc_acc_t *a, unsigned axis);

// Next gain index from the measured level, within [gain_min, gain_max].
unsigned openvrx_agc_next(unsigned gain, unsigned gain_min, unsigned gain_max, unsigned bits, float clip_share,
                          float rms_steps);

#ifdef __cplusplus
}
#endif
