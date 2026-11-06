// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain and DC servo task.
//
// Every OPENVRX_AGC_PERIOD_MS the task reads GPIO_IN OPENVRX_AGC_READS
// times (the lane pads have their input buffer on, see lanes_hw.c), feeds
// the pure-logic accumulator, moves the DC trim of the row in force while
// the DC is held (rf_dc_step, logic/openvrx_dc.h) and moves the forced gain
// index with rf_set_gain. Between those updates, every OPENVRX_AGC_FAST_MS
// a batch of OPENVRX_AGC_FAST_READS reads checks for heavy clipping and
// drops the gain at once (fast attack, C5-4). A read can catch a lane
// mid-transition; at a few thousand reads that only blurs the statistics
// slightly.
#include "agc.h"

#include <math.h>
#include "console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lanes_hw.h"
#include "openvrx_agc.h"
#include "openvrx_dc.h"
#include "rf.h"
#include "soc/gpio_struct.h"

#define AGC_STACK 3072
#define AGC_PRIO 2

static volatile bool s_enabled;
static volatile unsigned s_clip_permille, s_rms_centi;
static volatile int s_dc_adc[2];
static volatile bool s_measurable;
static openvrx_dc_servo_t s_servo;   // agc task only
static openvrx_agc_ctl_t s_ctl;      // agc task writes; agc_attacks reads the count

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
    openvrx_agc_plan_t plan;
    openvrx_agc_acc_t acc;
    openvrx_dc_servo_init(&s_servo);
    openvrx_agc_ctl_init(&s_ctl);
    uint32_t next_full = now_ms();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OPENVRX_AGC_FAST_MS));
        const uint32_t now = now_ms();
        const openvrx_route_t *r = lanes_hw_route();
        const bool ok = rf_started() && lanes_hw_selftest_mode() == OPENVRX_ST_OFF &&
                        openvrx_agc_plan(r->preset, r->lane, r->gpio, r->n, &plan);
        s_measurable = ok;
        if (!ok) continue;
        // A gain write, retune or hold change during the reads mixes two
        // analog states: drop that measurement.
        const unsigned gain = rf_config().gain;
        const uint32_t epoch = rf_dc_epoch();
        const bool full = (int32_t)(now - next_full) >= 0;
        if (!full) {
            // Fast attack check (C5-4): a short batch, clipping only.
            if (!s_enabled) continue;
            openvrx_agc_acc_reset(&acc);
            for (unsigned k = 0; k < OPENVRX_AGC_FAST_READS; ++k) openvrx_agc_acc_add(&acc, &plan, GPIO.in.val);
            if (rf_config().gain != gain || rf_dc_epoch() != epoch) continue;
            const unsigned next = openvrx_agc_fast(&s_ctl, now, gain, 0, openvrx_agc_clip_share(&acc));
            if (next != gain) (void)rf_set_gain((uint8_t)next);
            continue;
        }
        next_full = now + OPENVRX_AGC_PERIOD_MS;
        openvrx_agc_acc_reset(&acc);
        for (unsigned k = 0; k < OPENVRX_AGC_READS; ++k) openvrx_agc_acc_add(&acc, &plan, GPIO.in.val);
        if (rf_config().gain != gain || rf_dc_epoch() != epoch) continue;
        const float clip = openvrx_agc_clip_share(&acc);
        // While the DC is held and corrected it is not signal; without the
        // correction the total level is what has to fit (C5-1).
        const bool dc_held = rf_dc_held();
        const float rms = dc_held ? openvrx_agc_rms_ac_steps(&acc) : openvrx_agc_rms_steps(&acc);
        const float step_adc = (float)(1u << (10u - plan.bits));
        s_clip_permille = (unsigned)(clip * 1000.0f + 0.5f);
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

void agc_last_dc(int dc_adc[2])
{
    dc_adc[0] = s_dc_adc[0];
    dc_adc[1] = s_dc_adc[1];
}
