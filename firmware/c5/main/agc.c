// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain and DC servo task.
//
// Every OPENVRX_AGC_FAST_MS the task reads GPIO_IN OPENVRX_AGC_FAST_READS
// times (the lane pads have their input buffer on, see lanes_hw.c) into a
// batch. The batch is checked for heavy clipping, which drops the gain at
// once (fast attack, C5-4), and goes into a window (C5-21). Every
// OPENVRX_AGC_PERIOD_MS the window's batches are pooled, interference bursts
// left out (logic/openvrx_agc.h), and the pool moves the DC trim of the row
// in force while the DC is held (rf_dc_step, logic/openvrx_dc.h) and the
// forced gain index (rf_set_gain). A read can catch a lane mid-transition;
// at a few thousand reads that only blurs the statistics slightly. With
// `iq on` (C5-20) each batch is instead a 256-sample one-shot 10-bit
// capture from the dump engine (rf_iq_capture, logic/openvrx_iq.h).
#include "agc.h"

#include <math.h>
#include <string.h>
#include "console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lanes_hw.h"
#include "openvrx_agc.h"
#include "openvrx_dc.h"
#include "openvrx_iq.h"
#include "rf.h"
#include "soc/gpio_struct.h"

#define AGC_STACK 3072
#define AGC_PRIO 2
// With iq on, this many failed captures in a row turn it off again (EVT iq).
#define IQ_FAIL_LIMIT 50u

static volatile bool s_enabled;
static volatile unsigned s_clip_permille, s_rms_centi;
static volatile int s_dc_adc[2];
static volatile bool s_measurable;
static openvrx_dc_servo_t s_servo;   // agc task only
static openvrx_agc_ctl_t s_ctl;      // agc task writes; agc_attacks reads the count
static openvrx_agc_window_t s_win;   // agc task only (too big for its stack)
static volatile uint32_t s_bursts;   // batches left out as bursts since boot
// C5-20: batches from one-shot 10-bit captures instead of the pads.
static volatile bool s_iq, s_iq_strict;
static volatile uint32_t s_iq_ok, s_iq_fail, s_iq_us_max, s_iq_gain_bad;
static volatile unsigned s_iq_us, s_rail_permille;
static uint32_t s_iq_words[OPENVRX_AGC_FAST_READS];   // agc task only

static uint32_t now_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

// One DC servo decision for the row just measured (logic/openvrx_dc.h).
// It runs whenever the layout can be measured, also on pure receiver noise
// at the top rows (C5VRX 2026-10-28: a servo gated on other health checks
// froze there with 2.4 to 4.2 cells of DC).
static void dc_service(unsigned gain, uint32_t epoch, const openvrx_agc_acc_t *acc, unsigned bits)
{
    openvrx_dc_meas_t m;
    m.step_adc = (float)(1u << (10u - bits));
    for (unsigned a = 0; a < 2; ++a) {
        m.mean_adc[a] = openvrx_agc_mean_steps(acc, a) * m.step_adc;
        m.neg_share[a] = openvrx_agc_neg_share(acc, a);
    }
    m.rms_ac_adc = openvrx_agc_rms_ac_steps(acc) * m.step_adc;
    m.reads = acc->values / 2u;
    int step[2], applied[2];
    bool limited[2];
    openvrx_dc_servo_decide(&s_servo, gain, epoch, &m, step);
    if (!step[0] && !step[1]) return;
    if (rf_dc_step((uint8_t)gain, step, applied, limited) != ESP_OK) return;
    const uint32_t limits = s_servo.limits;
    openvrx_dc_servo_applied(&s_servo, applied, limited);
    if (s_servo.limits != limits && !(limits & 63u))   // the first and then every 64th
        console_evt("dc limit gain=%u i=%d q=%d", gain, (int)m.mean_adc[0], (int)m.mean_adc[1]);
}

static void agc_task(void *arg)
{
    (void)arg;
    openvrx_agc_plan_t plan, win_plan;
    openvrx_agc_acc_t batch, acc;
    openvrx_dc_servo_init(&s_servo);
    openvrx_agc_ctl_init(&s_ctl);
    openvrx_agc_window_reset(&s_win);
    memset(&win_plan, 0, sizeof(win_plan));
    uint32_t next_full = now_ms();
    unsigned iq_run = 0;   // failed captures in a row
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OPENVRX_AGC_FAST_MS));
        const uint32_t now = now_ms();
        const openvrx_route_t *r = lanes_hw_route();
        const bool ok = rf_started() && lanes_hw_selftest_mode() == OPENVRX_ST_OFF &&
                        openvrx_agc_plan(r->preset, r->lane, r->gpio, r->n, &plan);
        s_measurable = ok;
        if (!ok) {
            openvrx_agc_window_reset(&s_win);
            continue;
        }
        // Batches of another layout or pin map mean something else.
        if (memcmp(&plan, &win_plan, sizeof(plan)) != 0) {
            openvrx_agc_window_reset(&s_win);
            win_plan = plan;
        }
        // A gain write, retune or hold change during the reads mixes two
        // analog states: drop that batch (the window starts over at the
        // next one, which comes at the new gain or epoch).
        const unsigned gain = rf_config().gain;
        const uint32_t epoch = rf_dc_epoch();
        openvrx_agc_acc_reset(&batch);
        if (s_iq) {
            // One capture of as many samples as a pad batch has reads. A
            // failed one skips the batch (the window never mixes units).
            rf_iq_meta_t m;
            openvrx_iq_check_t chk;
            const esp_err_t e = rf_iq_capture(s_iq_words, OPENVRX_AGC_FAST_READS, &m);
            s_iq_us = m.us;
            if (m.us > s_iq_us_max) s_iq_us_max = m.us;
            const bool ok = e == ESP_OK && openvrx_iq_add(&batch, s_iq_words, OPENVRX_AGC_FAST_READS, plan.bits,
                                                          (int)gain, s_iq_strict, &chk);
            if (e == ESP_OK) s_iq_gain_bad += chk.gain_bad;
            if (!ok) {
                ++s_iq_fail;
                if (++iq_run >= IQ_FAIL_LIMIT) {
                    s_iq = false;
                    iq_run = 0;
                    console_evt("iq state=off cause=%s fails=%lu", e == ESP_OK ? "check" : esp_err_to_name(e),
                                (unsigned long)s_iq_fail);
                }
                continue;
            }
            iq_run = 0;
            ++s_iq_ok;
        } else {
            for (unsigned k = 0; k < OPENVRX_AGC_FAST_READS; ++k) openvrx_agc_acc_add(&batch, &plan, GPIO.in.val);
        }
        if (rf_config().gain != gain || rf_dc_epoch() != epoch) continue;
        openvrx_agc_window_push(&s_win, &batch, gain, epoch);
        if (s_enabled) {
            // Fast attack check (C5-4): this batch alone, clipping only.
            const unsigned next = openvrx_agc_fast(&s_ctl, now, gain, 0, plan.bits, openvrx_agc_clip_share(&batch));
            if (next != gain) {
                (void)rf_set_gain((uint8_t)next);
                continue;
            }
        }
        if ((int32_t)(now - next_full) < 0 || s_win.n < OPENVRX_AGC_WINDOW_MIN) continue;
        next_full = now + OPENVRX_AGC_PERIOD_MS;
        s_bursts += openvrx_agc_window_pool(&s_win, &acc);
        openvrx_agc_window_reset(&s_win);   // each batch is pooled once
        const float clip = openvrx_agc_clip_share(&acc);
        // While the DC is held and corrected it is not signal; without the
        // correction the total level is what has to fit (C5-1).
        const bool dc_held = rf_dc_held();
        const float rms = dc_held ? openvrx_agc_rms_ac_steps(&acc) : openvrx_agc_rms_steps(&acc);
        const float step_adc = (float)(1u << (10u - plan.bits));
        s_clip_permille = (unsigned)(clip * 1000.0f + 0.5f);
        s_rail_permille = acc.values ? (unsigned)(1000u * (uint64_t)acc.rails / acc.values) : 0u;
        s_rms_centi = (unsigned)(rms * 100.0f + 0.5f);
        s_dc_adc[0] = (int)lroundf(openvrx_agc_mean_steps(&acc, 0) * step_adc);
        s_dc_adc[1] = (int)lroundf(openvrx_agc_mean_steps(&acc, 1) * step_adc);
        if (dc_held) dc_service(gain, epoch, &acc, plan.bits);
        if (!s_enabled) continue;
        const unsigned next = openvrx_agc_slow(&s_ctl, now, gain, 0, rf_gain_max(), plan.bits, clip, rms);
        if (next != gain) (void)rf_set_gain((uint8_t)next);
    }
}

esp_err_t agc_start(bool enabled)
{
    s_enabled = enabled;
    return xTaskCreate(agc_task, "agc", AGC_STACK, NULL, AGC_PRIO, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void agc_set_enabled(bool on) { s_enabled = on; }
bool agc_enabled(void) { return s_enabled; }

void agc_last(unsigned *clip_permille, unsigned *rms_centi, bool *measurable)
{
    *clip_permille = s_clip_permille;
    *rms_centi = s_rms_centi;
    *measurable = s_measurable;
}

unsigned agc_attacks(void) { return s_ctl.attacks; }

unsigned agc_bursts(void) { return s_bursts; }

void agc_set_iq(bool on, bool strict)
{
    if (on && !s_iq) {
        s_iq_ok = s_iq_fail = s_iq_us_max = s_iq_gain_bad = 0;
        s_iq_us = 0;
    }
    s_iq_strict = strict;
    s_iq = on;
}

bool agc_iq(void) { return s_iq; }

void agc_iq_stats(agc_iq_stats_t *s)
{
    s->on = s_iq;
    s->strict = s_iq_strict;
    s->ok = s_iq_ok;
    s->fail = s_iq_fail;
    s->us = s_iq_us;
    s->us_max = s_iq_us_max;
    s->gain_bad = s_iq_gain_bad;
    s->rail_permille = s_rail_permille;
}

void agc_last_dc(int dc_adc[2])
{
    dc_adc[0] = s_dc_adc[0];
    dc_adc[1] = s_dc_adc[1];
}
