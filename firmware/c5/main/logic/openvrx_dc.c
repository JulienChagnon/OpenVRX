// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: receiver DC correction before the lane cut (pure logic).
#include "openvrx_dc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static int sgnf(float v) { return v > 0.0f ? 1 : v < 0.0f ? -1 : 0; }

// Nearest row with the given bit set in flags (ties: the lower row), or -1.
static int nearest(const uint8_t *flags, uint8_t bit, unsigned row)
{
    for (unsigned d = 0; d < OPENVRX_DC_ROWS; ++d) {
        if (row >= d && (flags[row - d] & bit)) return (int)(row - d);
        if (row + d < OPENVRX_DC_ROWS && (flags[row + d] & bit)) return (int)(row + d);
    }
    return -1;
}

// ---------------------------------------------------------------- per-row trims

void openvrx_dc_rows_reset(openvrx_dc_rows_t *rows) { memset(rows, 0, sizeof(*rows)); }

void openvrx_dc_rows_claim(openvrx_dc_rows_t *rows, unsigned row, int trim[2])
{
    if (row >= OPENVRX_DC_ROWS) {
        trim[0] = trim[1] = 0;
        return;
    }
    if (!rows->have[row]) {
        const int from = nearest(rows->have, 1u, row);
        rows->trim[row][0] = from >= 0 ? rows->trim[from][0] : 0;
        rows->trim[row][1] = from >= 0 ? rows->trim[from][1] : 0;
        rows->have[row] = 1;
    }
    trim[0] = rows->trim[row][0];
    trim[1] = rows->trim[row][1];
}

int openvrx_dc_code(int base, int trim) { return clampi(base + trim, 0, OPENVRX_DC_CODE_MAX); }

void openvrx_dc_rows_step(openvrx_dc_rows_t *rows, unsigned row, const int base[2], const int step[2], int applied[2],
                          bool limited[2])
{
    int trim[2];
    openvrx_dc_rows_claim(rows, row, trim);
    for (unsigned a = 0; a < 2; ++a) {
        applied[a] = 0;
        limited[a] = false;
        if (row >= OPENVRX_DC_ROWS) continue;
        // Trim window and DAC range, both around the vendor code.
        const int lo = clampi(-OPENVRX_DC_TRIM_MAX, -base[a], OPENVRX_DC_CODE_MAX - base[a]);
        const int hi = clampi(OPENVRX_DC_TRIM_MAX, -base[a], OPENVRX_DC_CODE_MAX - base[a]);
        const int next = clampi(trim[a] + step[a], lo, hi);
        applied[a] = next - trim[a];
        limited[a] = applied[a] != step[a];
        rows->trim[row][a] = (int16_t)next;
    }
}

// ---------------------------------------------------------------- servo

void openvrx_dc_servo_init(openvrx_dc_servo_t *s)
{
    memset(s, 0, sizeof(*s));
    for (unsigned a = 0; a < 2; ++a) {
        s->pol[a] = 1;
        s->votes[a] = OPENVRX_DC_VOTES_START;
    }
}

float openvrx_dc_servo_slope(const openvrx_dc_servo_t *s, unsigned row, unsigned axis)
{
    if (row >= OPENVRX_DC_ROWS || axis > 1u) return OPENVRX_DC_SLOPE_DEFAULT;
    const int from = nearest(s->slope_known, (uint8_t)(1u << axis), row);
    return from >= 0 ? s->slope[from][axis] : OPENVRX_DC_SLOPE_DEFAULT;
}

static bool proven(const openvrx_dc_servo_t *s, unsigned a) { return s->votes[a] >= OPENVRX_DC_VOTES_PROVEN; }

static void flip(openvrx_dc_servo_t *s, unsigned a)
{
    s->pol[a] = (int8_t)-s->pol[a];
    s->votes[a] = 0;
    s->br_prev[a] = 0;
    s->br_bisect[a] = false;
    s->br_hold[a] = false;
    s->walk[a] = 0;
    ++s->flips;
}

static void forget_bracket(openvrx_dc_servo_t *s, unsigned a)
{
    s->br_prev[a] = 0;
    s->br_bisect[a] = false;
    s->br_hold[a] = false;
    s->walk[a] = 0;
}

// Learns from the last step where both measurements were in the linear zone.
static void learn(openvrx_dc_servo_t *s, unsigned row, unsigned a, float mean, bool sat)
{
    const int st = s->last_step[a];
    if (!st || sat || s->last_sat[a] || abs(st) < OPENVRX_DC_LEARN_STEP) return;
    const float moved = mean - s->last_mean[a];
    if (fabsf(moved) < OPENVRX_DC_LEARN_MIN_ADC) return;
    const float slope = moved / (float)st;   // signed ADC units per code
    const int agree = sgnf(slope) == s->pol[a] ? 1 : -1;
    s->votes[a] = (int8_t)clampi(s->votes[a] + agree, -OPENVRX_DC_VOTES_MAX, OPENVRX_DC_VOTES_MAX);
    if (s->votes[a] <= OPENVRX_DC_VOTES_FLIP) {
        flip(s, a);
        return;
    }
    if (agree < 0) return;   // a wrong-way move says nothing about the size
    float mag = fabsf(slope);
    if (mag < OPENVRX_DC_SLOPE_MIN) mag = OPENVRX_DC_SLOPE_MIN;
    if (mag > OPENVRX_DC_SLOPE_MAX) mag = OPENVRX_DC_SLOPE_MAX;
    const uint8_t bit = (uint8_t)(1u << a);
    // First measurement of a row as is; later ones smoothed (the DC wanders).
    s->slope[row][a] = (s->slope_known[row] & bit) ? 0.7f * s->slope[row][a] + 0.3f * mag : mag;
    s->slope_known[row] |= bit;
    ++s->learned;
}

void openvrx_dc_servo_decide(openvrx_dc_servo_t *s, unsigned row, uint32_t epoch, const openvrx_dc_meas_t *m,
                             int step[2])
{
    step[0] = step[1] = 0;
    if (row >= OPENVRX_DC_ROWS) return;
    if (!s->ctx_valid || s->ctx_row != row || s->ctx_epoch != epoch) {
        // Another row or another analog state: the last step and the
        // bracket say nothing here. A row without its own slope starts from
        // the nearest learned one.
        s->ctx_valid = true;
        s->ctx_row = row;
        s->ctx_epoch = epoch;
        for (unsigned a = 0; a < 2; ++a) {
            s->last_step[a] = 0;
            s->last_dir[a] = 0;
            forget_bracket(s, a);
            const uint8_t bit = (uint8_t)(1u << a);
            if (!(s->slope_known[row] & bit)) s->slope[row][a] = openvrx_dc_servo_slope(s, row, a);
        }
    }
    for (unsigned a = 0; a < 2; ++a) {
        const float mean = m->mean_adc[a];
        const float p = m->neg_share[a];
        const bool sat = p < OPENVRX_DC_SAT_SHARE || p > 1.0f - OPENVRX_DC_SAT_SHARE;
        // Confirmation for a small error: the previous measurement, with no
        // step since, was out on the same side (fewer steps on noise).
        const bool confirmed = !s->last_step[a] && !s->last_sat[a] && sgnf(s->last_mean[a]) == sgnf(mean);
        const float last_abs = fabsf(s->last_mean[a]);
        learn(s, row, a, mean, sat);
        s->last_mean[a] = mean;
        s->last_sat[a] = sat;
        s->last_step[a] = 0;
        s->last_dir[a] = 0;
        int st = 0;
        if (!sat) {
            forget_bracket(s, a);
            const float slope = s->slope[row][a] > 0.0f ? s->slope[row][a] : OPENVRX_DC_SLOPE_DEFAULT;
            const float se = m->reads ? OPENVRX_DC_SE_FACTOR * m->rms_ac_adc / sqrtf((float)m->reads) : 0.0f;
            const float deadband = fmaxf(fmaxf(OPENVRX_DC_DEADBAND_ADC, OPENVRX_DC_DEADBAND_CODES * slope),
                                         0.5f * slope + 2.0f * se);
            if (fabsf(mean) <= deadband) continue;
            if (fabsf(mean) <= OPENVRX_DC_CONFIRM_DEADBANDS * deadband && !(confirmed && last_abs > deadband))
                continue;
            st = (int)lroundf(-(float)s->pol[a] * OPENVRX_DC_LOOP_GAIN * mean / slope);
            if (!st) st = -s->pol[a] * sgnf(mean);   // outside the deadband: at least one code
            st = clampi(st, -OPENVRX_DC_STEP_MAX, OPENVRX_DC_STEP_MAX);
        } else {
            // The cloud is on one side: fewer negatives than half means a positive DC.
            const int dc_sign = p < 0.5f ? 1 : -1;
            if (s->br_hold[a]) {
                if (fabsf(mean - s->br_hold_mean[a]) < OPENVRX_DC_HOLD_MOVE_STEPS * m->step_adc) continue;
                forget_bracket(s, a);
            }
            // A walk this long that pushed the cloud further out: wrong polarity.
            if (s->walk[a] >= OPENVRX_DC_WRONG_WALK && fabsf(mean) > s->walk_abs0[a] + 0.5f * m->step_adc &&
                !proven(s, a))
                flip(s, a);
            const int dir = -s->pol[a] * dc_sign;
            const int prev = s->br_prev[a];
            // First step from the evidence: a saturated mean is at least the
            // size of the DC minus half a step, so near zero (a fixed point
            // at -1 or 0) this is one code and far out the full start size.
            const float slope = s->slope[row][a] > 0.0f ? s->slope[row][a] : OPENVRX_DC_SLOPE_DEFAULT;
            int mag = clampi((int)lroundf(OPENVRX_DC_LOOP_GAIN * fabsf(mean) / slope), 1, OPENVRX_DC_BRACKET_START);
            if (prev && sgnf((float)prev) != dir) s->br_bisect[a] = true;   // crossed zero
            if (prev && s->br_bisect[a]) {
                mag = abs(prev) / 2;   // bisection: halve whichever side
                if (!mag) {            // a one-code crossing: zero within one code, hold
                    s->br_hold[a] = true;
                    s->br_hold_mean[a] = mean;
                    s->br_prev[a] = 0;
                    s->br_bisect[a] = false;
                    s->walk[a] = 0;
                    continue;
                }
            } else if (prev) {
                mag = 2 * abs(prev);   // same side, no crossing yet: expand
                if (mag > OPENVRX_DC_BRACKET_MAX) mag = OPENVRX_DC_BRACKET_MAX;
            }
            st = dir * mag;
            if (!s->walk[a] || !prev || sgnf((float)prev) != dir) {
                s->walk[a] = 0;
                s->walk_abs0[a] = fabsf(mean);
            }
            s->walk[a] += mag;
            s->br_prev[a] = st;
        }
        step[a] = st;
        s->last_dir[a] = sgnf((float)st);
    }
    if (step[0] || step[1]) ++s->steps;
}

void openvrx_dc_servo_applied(openvrx_dc_servo_t *s, const int applied[2], const bool limited[2])
{
    for (unsigned a = 0; a < 2; ++a) {
        s->last_step[a] = applied[a];
        if (!limited[a]) continue;
        ++s->limits;
        // Stuck at the trim limit while the cloud stays on one side. If the
        // walk there did not bring |mean| in at all and the polarity is not
        // proven, the walk went the wrong way; otherwise the DC is beyond
        // the DAC's reach and the row stays at its limit (no ping-pong
        // between the two limits).
        const bool no_gain = fabsf(s->last_mean[a]) >= s->walk_abs0[a] - 1.0f;
        if (applied[a] == 0 && s->last_sat[a] && s->last_dir[a] && no_gain && !proven(s, a)) flip(s, a);
        else if (applied[a]) s->br_prev[a] = applied[a];   // a step cut to nothing keeps the walk's history
    }
}
