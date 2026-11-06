// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: receiver DC correction before the lane cut (pure logic).
//
// The C5's I and Q leave the ADC with an offset that the boot calibration
// does not remove (crash flight: I at +216 of 512, 36 % clipped). Gain
// cannot fix an offset, and once the samples are cut to 4 bits the clipped
// side is lost. The offset has to come off in the analog front end: the
// PBUS DC DACs ahead of the ADC (bank 2 of PBUS blocks 2 and 3, one code
// pair per gain row, C5VRX's recovered libphy facts; phy.c holds them).
//
// Approach after C5VRX's board work on the same chip and the same pinned
// libphy (Twotoz and the C5VRX contributors; their live DC servo took the
// board DC from about 134 to about 11 ADC units): every gain row keeps its
// own trim, in codes added to the vendor's bank-2 code of that row, and a
// slow servo moves the trim of the row in use. The DC differs per boot and
// per channel setup and wanders by itself, so the servo runs all the time.
//
// Measurement: the gain loop's pad reads (openvrx_agc.h), on layouts that
// carry the top ADC bits. On those the mean never folds; clipping pulls it
// towards zero but never across, so the loop still converges to the true
// zero, where the clipping bias vanishes. Two zones per axis:
//   linear     the cloud spans zero (sign share inside 3..97 %): one
//              proportional step, 0.7 of the error over the learned slope
//              (ADC units per code), at most 8 codes.
//   saturated  the cloud sits on one side (a large DC, or receiver noise
//              under one step, where the IQ is one fixed point and the mean
//              moves only in whole steps): a bracketing search on the sign
//              bit. Still on the same side after a step: the next step
//              doubles (8, 16); once zero has been crossed every step
//              halves (bisection); a one-code crossing is held (zero within
//              one code) until the mean moves.
// The DAC polarity is a hardware constant, taken as positive (C5VRX boards:
// +1 code raises the DC); it flips only on evidence: three net linear-zone
// steps that moved the DC the wrong way, or a saturated walk of 24 codes
// that pushed the cloud further out, or a trim stuck at its limit while
// saturated, each only while the polarity is not yet proven by three
// agreeing steps.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Gain rows (>= ARC_VENDOR_GAIN_MAX + 1).
#define OPENVRX_DC_ROWS 96u
// DAC code range and how far a trim may move from the vendor code (C5VRX
// DCO_RANGE: board trims of the edge rows were 13 to 48 codes).
#define OPENVRX_DC_CODE_MAX 511
#define OPENVRX_DC_TRIM_MAX 96
// Sign share beyond which the cloud is on one side of zero.
#define OPENVRX_DC_SAT_SHARE 0.03f
// Linear zone: hold while |mean| is under the largest of this many ADC
// units, 0.6 of a code, or half a code plus two standard errors of the
// mean (the DAC cannot do better than half a code, and a step on noise
// would only dither it). The standard error is the AC RMS over the root of
// the reads, times OPENVRX_DC_SE_FACTOR: the reads come in one burst of a
// few ms and are not independent, so the plain formula would understate it.
#define OPENVRX_DC_DEADBAND_ADC 6.0f
#define OPENVRX_DC_DEADBAND_CODES 0.6f
#define OPENVRX_DC_SE_FACTOR 2.0f
// Linear zone: an error under this many deadbands moves only when the
// previous measurement (no step since) was out on the same side too. The
// mean of 4096 reads carries a few ADC units of noise with a full-scale
// carrier; one confirmation keeps that from dithering the code.
#define OPENVRX_DC_CONFIRM_DEADBANDS 4.0f
#define OPENVRX_DC_LOOP_GAIN 0.7f
#define OPENVRX_DC_STEP_MAX 8
// Saturated zone: first and largest bracket step in codes.
#define OPENVRX_DC_BRACKET_START 8
#define OPENVRX_DC_BRACKET_MAX 16
// A held one-code crossing is released when the mean moves this many steps.
#define OPENVRX_DC_HOLD_MOVE_STEPS 0.8f
// Slope in ADC units (full scale 512) per DAC code. C5VRX learned about
// 0.4 of a 4-bit step per code; the default leans high, which makes the
// first steps short rather than overshooting.
#define OPENVRX_DC_SLOPE_DEFAULT 16.0f
#define OPENVRX_DC_SLOPE_MIN 1.0f
#define OPENVRX_DC_SLOPE_MAX 80.0f
// A slope is learned from a linear-zone step of at least this many codes
// that moved the mean by at least this many ADC units (the DC wanders by
// itself; smaller moves drown in it).
#define OPENVRX_DC_LEARN_STEP 2
#define OPENVRX_DC_LEARN_MIN_ADC 6.0f
// Polarity evidence: starts at +1 for the default; flips at -2; proven at 3.
#define OPENVRX_DC_VOTES_START 1
#define OPENVRX_DC_VOTES_FLIP (-2)
#define OPENVRX_DC_VOTES_PROVEN 3
#define OPENVRX_DC_VOTES_MAX 6
// Saturated walk in one direction after which a growing |mean| means the
// polarity is wrong.
#define OPENVRX_DC_WRONG_WALK 24

// ---------------------------------------------------------------- per-row trims

typedef struct {
    int16_t trim[OPENVRX_DC_ROWS][2];   // codes added to the vendor code, I then Q
    uint8_t have[OPENVRX_DC_ROWS];      // row claimed (its trim is its own)
} openvrx_dc_rows_t;

void openvrx_dc_rows_reset(openvrx_dc_rows_t *rows);
// The trim of a row. A row seen for the first time starts from the nearest
// claimed row (the DC shift after a channel setup is common to the rows;
// C5VRX board 2026-10-19), or 0, and is claimed.
void openvrx_dc_rows_claim(openvrx_dc_rows_t *rows, unsigned row, int trim[2]);
// The DAC code for a vendor base code and a trim, within 0..511.
int openvrx_dc_code(int base, int trim);
// Adds step to a row's trim, within +-OPENVRX_DC_TRIM_MAX and the DAC range
// around base. applied: the change made; limited: an axis whose request
// was cut short. The row is claimed first.
void openvrx_dc_rows_step(openvrx_dc_rows_t *rows, unsigned row, const int base[2], const int step[2], int applied[2],
                          bool limited[2]);

// ---------------------------------------------------------------- servo

typedef struct {
    float mean_adc[2];    // I, Q mean in ADC units (full scale 512)
    float neg_share[2];   // share below zero
    float step_adc;       // ADC units per pad step: 2^(10 - bits)
    float rms_ac_adc;     // RMS around the mean per axis, ADC units (0: unknown)
    unsigned reads;       // pad reads behind the means
} openvrx_dc_meas_t;

typedef struct {
    // Hardware model, learned.
    float slope[OPENVRX_DC_ROWS][2];        // ADC units per code, magnitude
    uint8_t slope_known[OPENVRX_DC_ROWS];   // bit 0: I learned, bit 1: Q learned
    int8_t pol[2];                          // +1: a code up raises the DC
    int8_t votes[2];                        // evidence for pol
    // Context of the row in use: forgotten on any change of row or epoch.
    bool ctx_valid;
    unsigned ctx_row;
    uint32_t ctx_epoch;
    float last_mean[2];
    bool last_sat[2];
    int last_step[2];      // codes applied after the last measurement
    int br_prev[2];        // last bracket step (0: none)
    bool br_bisect[2];     // zero crossed once: every further step halves
    bool br_hold[2];       // one-code crossing held
    float br_hold_mean[2];
    int walk[2];           // codes walked in one direction while saturated
    float walk_abs0[2];    // |mean| when that walk began
    int last_dir[2];       // direction of the last decided step (for the stuck-limit rule)
    // Telemetry.
    uint32_t steps, learned, flips, limits;
} openvrx_dc_servo_t;

void openvrx_dc_servo_init(openvrx_dc_servo_t *s);
// One decision for gain row `row` from a measurement taken at the current
// codes. epoch: changes whenever something other than the servo moved the
// DC (retune, gain write, hold); a new epoch or row forgets the context.
// step: codes to add to the row's trim, I then Q.
void openvrx_dc_servo_decide(openvrx_dc_servo_t *s, unsigned row, uint32_t epoch, const openvrx_dc_meas_t *m,
                             int step[2]);
// What was actually applied after the decision (openvrx_dc_rows_step).
void openvrx_dc_servo_applied(openvrx_dc_servo_t *s, const int applied[2], const bool limited[2]);
// Slope used for a row and axis (nearest learned row, or the default).
float openvrx_dc_servo_slope(const openvrx_dc_servo_t *s, unsigned row, unsigned axis);

#ifdef __cplusplus
}
#endif
