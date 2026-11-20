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
// Target RMS per component as a share of full scale (2^(bits-1) steps), by bits per component
// (openvrx_agc_target_share). With 4 bits, 0.55 puts the FM envelope just under the clip level
// (4x4top: 4.4 steps): the picture loses fast on both sides of it, so the target sits at the top.
// More bits make the low side cheap (each bit is 6 dB more range below the clip), so the target
// moves down to keep room for fade-ups: 7 and 8 bits at 0.20 (about 9 dB under the 4-bit target,
// 12.8 steps of 64 for 7x7), 5 and 6 bits in between.
#define OPENVRX_AGC_TARGET_SHARE_4 0.55f
#define OPENVRX_AGC_TARGET_SHARE_5 0.40f
#define OPENVRX_AGC_TARGET_SHARE_6 0.28f
#define OPENVRX_AGC_TARGET_SHARE_7 0.20f

// Target share of full scale for a layout with `bits` per component (see above).
float openvrx_agc_target_share(unsigned bits);
// No change while the level is within this many dB of the target.
#define OPENVRX_AGC_DEADBAND_DB 1.5f
// Clipped share of component samples that counts as overload whatever the
// RMS says (clipping caps the RMS near full scale). The level is then read
// from the clipped share (openvrx_agc_clip_over_db).
#define OPENVRX_AGC_OVERLOAD_SHARE 0.25f
// Gain is only raised while fewer component samples than this clip.
#define OPENVRX_AGC_RAISE_MAX_CLIP 0.05f
// dB per gain index, measured on the bench board: receiver noise against
// gain in the top RF stage (from index OPENVRX_AGC_TOP_STAGE_START up) and
// the loop holding a stepped signal below it. One board so far, not a
// per-board table yet.
#define OPENVRX_AGC_TOP_STAGE_START 54u
#define OPENVRX_AGC_DB_PER_INDEX_TOP 0.93f
#define OPENVRX_AGC_DB_PER_INDEX_LOW 1.1f
// Largest index change per update (about 4 dB per 100 ms).
#define OPENVRX_AGC_MAX_STEP 4

// Fast attack (C5-4). Flight fades swing 20 to 30 dB within a fraction of
// a second and the 100 ms loop, at most 4 indexes a step, left 30 to 95 %
// of the values clipped for seconds after a fade (2026-11-04 flights).
// Every OPENVRX_AGC_FAST_MS a short batch of pad reads checks clipping;
// at OPENVRX_AGC_ATTACK_CLIP or more the gain drops by part of the
// overload the batch's clipped share implies (OPENVRX_AGC_ATTACK_FRACTION,
// OPENVRX_AGC_ATTACK_STEP_MIN to _MAX indexes), again after
// OPENVRX_AGC_ATTACK_GAP_MS while it still clips: the dB per index is
// measured on one board only, so each step is checked by the next batch. Raising stays with the 100 ms loop, held off for
// OPENVRX_AGC_RAISE_HOLD_MS after an attack so it does not pump back into
// the clipping. A gain write blanks the lanes for a few microseconds (the
// radio output drops to zero while the row is rewritten).
#define OPENVRX_AGC_FAST_MS 5u
#define OPENVRX_AGC_FAST_READS 256u
// The attack is for overload that costs picture. An FM carrier keeps its
// phase while only one axis clips: the clipped axis is wrong but the angle
// moves little, and the demodulated video barely changes. The phase is lost
// once both axes clip together (the constellation's corners), which starts
// at a carrier radius of sqrt(2) times the clip level: about +4 dB over the
// 4-bit target, where each component clips half the time. Lighter clipping
// is cheaper than the under-range a large drop leaves behind (and the
// raise hold-off keeps it there). So the attack starts at 40 % (a carrier
// about +2.9 dB over the target on 4 bits) and drops the gain by the
// overload the clipped share implies (openvrx_agc_clip_over_db), landing
// near the target rather than well under it. Interference bursts on the
// board read 2-6 % of a 4096-read measurement.
#define OPENVRX_AGC_ATTACK_CLIP 0.40f
// Consecutive hot batches before the first attack. Board 2026-11-04 with a
// single hot batch: about 2 attacks a second with no VTX at all (gain
// dragged 83 -> 65 and back): short interference bursts (most likely
// 5 GHz Wi-Fi packets, 2-6 % of a 4096-read measurement) fill a 256-read
// batch of about 0.1 ms. Three batches are 10 ms of clipping, longer than
// a packet; a fade recovery clips for hundreds of ms. Once confirmed, the
// attack repeats every OPENVRX_AGC_ATTACK_GAP_MS while the clipping lasts.
#define OPENVRX_AGC_ATTACK_CONFIRM 3u
// Share of the implied overload one attack removes. The next batch measures
// again _GAP_MS later, so an attack need not get there in one step, and a
// computed step scales any error in the dB per index: at a half, 20 dB of
// overload takes about three attacks at the measured slope, and a board
// twice as steep as the table would land on the target in one.
#define OPENVRX_AGC_ATTACK_FRACTION 0.5f
// Attack size bounds, indexes. The clipped share stops telling levels apart
// far above the clip level (it tends to 1), so one attack is at most
// _MAX and a deeper fade-up takes a few attacks _GAP_MS apart.
#define OPENVRX_AGC_ATTACK_STEP_MIN 2
#define OPENVRX_AGC_ATTACK_STEP_MAX 12
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
// Fast check at now_ms on a short batch of a `bits`-per-component layout:
// the gain to set (== gain: none).
unsigned openvrx_agc_fast(openvrx_agc_ctl_t *c, uint32_t now_ms, unsigned gain, unsigned gain_min, unsigned bits,
                          float clip_share);
// Level in dB against the gain target (openvrx_agc_target_share) of a
// constant-envelope carrier whose components sit at an end code a share
// `clip_share` of the time. A component of a circle of radius A passes the
// end code (2^(bits-1) - 1 steps) a share (2/pi) acos(clip / A) of the
// time, so A = clip / cos(pi share / 2). The share is clamped to 0.97
// (about +27 dB over the clip level); at 0 it gives the level where
// clipping starts. Noise on the carrier makes it read a little high.
float openvrx_agc_clip_over_db(unsigned bits, float clip_share);
// Index step that removes `over_db` at `gain` (the measured dB per index).
int openvrx_agc_db_to_step(unsigned gain, float over_db);
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
