// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain task.
//
// Every OPENVRX_AGC_PERIOD_MS the task reads GPIO_IN OPENVRX_AGC_READS
// times (the lane pads have their input buffer on, see lanes_hw.c), feeds
// the pure-logic accumulator and moves the forced gain index with
// rf_set_gain. A read can catch a lane mid-transition; at a few thousand
// reads that only blurs the statistics slightly.
#include "agc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lanes_hw.h"
#include "openvrx_agc.h"
#include "rf.h"
#include "soc/gpio_struct.h"

#define AGC_STACK 3072
#define AGC_PRIO 2

static volatile bool s_enabled;
static volatile unsigned s_clip_permille, s_rms_centi;
static volatile bool s_measurable;

static void agc_task(void *arg)
{
    (void)arg;
    openvrx_agc_plan_t plan;
    openvrx_agc_acc_t acc;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OPENVRX_AGC_PERIOD_MS));
        const openvrx_route_t *r = lanes_hw_route();
        const bool ok = rf_started() && lanes_hw_selftest_mode() == OPENVRX_ST_OFF &&
                        openvrx_agc_plan(r->preset, r->lane, r->gpio, r->n, &plan);
        s_measurable = ok;
        if (!ok) continue;
        openvrx_agc_acc_reset(&acc);
        for (unsigned k = 0; k < OPENVRX_AGC_READS; ++k) openvrx_agc_acc_add(&acc, &plan, GPIO.in.val);
        const float clip = openvrx_agc_clip_share(&acc);
        const float rms = openvrx_agc_rms_steps(&acc);
        s_clip_permille = (unsigned)(clip * 1000.0f + 0.5f);
        s_rms_centi = (unsigned)(rms * 100.0f + 0.5f);
        if (!s_enabled) continue;
        const unsigned gain = rf_config().gain;
        const unsigned next = openvrx_agc_next(gain, 0, rf_gain_max(), plan.bits, clip, rms);
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
