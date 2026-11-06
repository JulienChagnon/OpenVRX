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
    a->sum2[0] += 2 * vi + 1;
    a->sum2[1] += 2 * vq + 1;
    a->neg[0] += vi < 0;
    a->neg[1] += vq < 0;
    a->values += 2;
}

float openvrx_agc_mean_steps(const openvrx_agc_acc_t *a, unsigned axis)
{
    const uint32_t per_axis = a->values / 2u;
    return axis < 2u && per_axis ? (float)a->sum2[axis] / (2.0f * (float)per_axis) : 0.0f;
}

float openvrx_agc_neg_share(const openvrx_agc_acc_t *a, unsigned axis)
{
    const uint32_t per_axis = a->values / 2u;
    return axis < 2u && per_axis ? (float)a->neg[axis] / (float)per_axis : 0.5f;
}

float openvrx_agc_rms_ac_steps(const openvrx_agc_acc_t *a)
{
    if (!a->values) return 0.0f;
    // Mean square over both axes minus the mean of the two squared means.
    const float mi = openvrx_agc_mean_steps(a, 0), mq = openvrx_agc_mean_steps(a, 1);
    const float ms = (float)a->sumsq4 / (4.0f * (float)a->values) - 0.5f * (mi * mi + mq * mq);
    return ms > 0.0f ? sqrtf(ms) : 0.0f;
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

void openvrx_agc_ctl_init(openvrx_agc_ctl_t *c) { memset(c, 0, sizeof(*c)); }

// Milliseconds from a to b, wrap-safe (uint32 ms wraps after 49 days).
static int32_t ms_since(uint32_t now, uint32_t then) { return (int32_t)(now - then); }

unsigned openvrx_agc_fast(openvrx_agc_ctl_t *c, uint32_t now_ms, unsigned gain, unsigned gain_min, float clip_share)
{
    if (clip_share < OPENVRX_AGC_ATTACK_CLIP) {
        c->hot = 0;
        return gain;
    }
    if (c->hot < OPENVRX_AGC_ATTACK_CONFIRM) ++c->hot;
    if (c->hot < OPENVRX_AGC_ATTACK_CONFIRM || gain <= gain_min) return gain;
    if (c->attacked && ms_since(now_ms, c->last_attack_ms) < (int32_t)OPENVRX_AGC_ATTACK_GAP_MS) return gain;
    const unsigned next = gain > gain_min + OPENVRX_AGC_ATTACK_STEP ? gain - OPENVRX_AGC_ATTACK_STEP : gain_min;
    c->attacked = true;
    c->last_attack_ms = now_ms;
    ++c->attacks;
    return next;
}

unsigned openvrx_agc_slow(const openvrx_agc_ctl_t *c, uint32_t now_ms, unsigned gain, unsigned gain_min,
                          unsigned gain_max, unsigned bits, float clip_share, float rms_steps)
{
    const unsigned next = openvrx_agc_next(gain, gain_min, gain_max, bits, clip_share, rms_steps);
    if (next > gain && c->attacked && ms_since(now_ms, c->last_attack_ms) < (int32_t)OPENVRX_AGC_RAISE_HOLD_MS)
        return gain;
    return next;
}
