// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain from the lane pads (pure logic).
#include "openvrx_agc.h"

#include <math.h>
#include <string.h>

bool openvrx_agc_plan(openvrx_preset_t preset, const openvrx_lane_t *lanes, const int8_t *gpio, unsigned n,
                      openvrx_agc_plan_t *plan)
{
    memset(plan, 0, sizeof(*plan));
    switch (preset) {
    case OPENVRX_PRESET_8X8:
    case OPENVRX_PRESET_7X7:
    case OPENVRX_PRESET_6X6:
    case OPENVRX_PRESET_5X5:
    case OPENVRX_PRESET_4X4TOP: break;
    default: return false;
    }
    if (n == 0 || n > OPENVRX_MAX_LANES || n % 2) return false;
    unsigned ni = 0, nq = 0;
    for (unsigned k = 0; k < n; ++k) {
        if (lanes[k].comp == 'I') ++ni;
        else ++nq;
    }
    if (ni != nq) return false;
    const unsigned bits = ni;
    const unsigned lsb = 10u - bits;   // lowest carried ADC bit (top bits only)
    for (unsigned k = 0; k < n; ++k) {
        const unsigned b = lanes[k].adc_bit;
        if (b < lsb || b > 9 || gpio[k] < 0) return false;
        const int w = 1 << (b - lsb);
        plan->gpio[k] = (uint8_t)gpio[k];
        plan->weight[k] = (int16_t)(b == 9 ? -w : w);
        plan->is_i[k] = lanes[k].comp == 'I';
    }
    plan->n = n;
    plan->bits = bits;
    return true;
}

void openvrx_agc_acc_reset(openvrx_agc_acc_t *a) { memset(a, 0, sizeof(*a)); }

void openvrx_agc_acc_add(openvrx_agc_acc_t *a, const openvrx_agc_plan_t *plan, uint32_t gpio_in)
{
    int vi = 0, vq = 0;
    for (unsigned k = 0; k < plan->n; ++k) {
        if (!((gpio_in >> plan->gpio[k]) & 1u)) continue;
        if (plan->is_i[k]) vi += plan->weight[k];
        else vq += plan->weight[k];
    }
    const int hi = (1 << (plan->bits - 1)) - 1;
    const int lo = -(1 << (plan->bits - 1));
    a->clips += (vi == hi || vi == lo) + (vq == hi || vq == lo);
    a->sumsq4 += (uint64_t)((2 * vi + 1) * (2 * vi + 1)) + (uint64_t)((2 * vq + 1) * (2 * vq + 1));
    a->values += 2;
}

float openvrx_agc_clip_share(const openvrx_agc_acc_t *a)
{
    return a->values ? (float)a->clips / (float)a->values : 0.0f;
}

float openvrx_agc_rms_steps(const openvrx_agc_acc_t *a)
{
    return a->values ? sqrtf((float)a->sumsq4 / (4.0f * (float)a->values)) : 0.0f;
}

unsigned openvrx_agc_next(unsigned gain, unsigned gain_min, unsigned gain_max, unsigned bits, float clip_share,
                          float rms_steps)
{
    if (bits < 2 || gain_min > gain_max) return gain;
    const float target = OPENVRX_AGC_TARGET_SHARE * (float)(1u << (bits - 1));
    float err_db = 20.0f * log10f(fmaxf(rms_steps, 0.05f) / target);
    if (clip_share >= OPENVRX_AGC_OVERLOAD_SHARE && err_db < 6.0f) err_db = 6.0f;
    if (fabsf(err_db) < OPENVRX_AGC_DEADBAND_DB) return gain;
    if (err_db < 0.0f && clip_share >= OPENVRX_AGC_RAISE_MAX_CLIP) return gain;
    int step = (int)lroundf(err_db / OPENVRX_AGC_DB_PER_INDEX);
    if (step > OPENVRX_AGC_MAX_STEP) step = OPENVRX_AGC_MAX_STEP;
    if (step < -OPENVRX_AGC_MAX_STEP) step = -OPENVRX_AGC_MAX_STEP;
    int next = (int)gain - step;   // too loud: lower index
    if (next < (int)gain_min) next = (int)gain_min;
    if (next > (int)gain_max) next = (int)gain_max;
    return (unsigned)next;
}
