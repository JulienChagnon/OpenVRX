// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: lane pads (MODEM_DIAG routing and the self-test).
//
// MODEM_DIAG routing follows C5VRX (main/rf.c route_modem_iq, by Twotoz and
// the C5VRX contributors): pad as GPIO output, then the GPIO matrix connects
// MODEM_DIAG0_IDX + bus bit, no CPU or register in between. OpenVRX routes
// any preset's bits instead of C5VRX's fixed 4+4 lanes and forces the
// output enable from GPIO_ENABLE.
#include "lanes_hw.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_ll.h"
#include "openvrx_selftest.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"

static openvrx_route_t s_route = {.preset = OPENVRX_PRESET_NONE, .clk = -1};
static unsigned s_drive = 1;
static gptimer_handle_t s_timer;
static bool s_timer_running;
static openvrx_selftest_mode_t s_mode = OPENVRX_ST_OFF;
static uint32_t s_phase_us = OPENVRX_PHASE_US_DEFAULT;
static volatile uint32_t s_phase;
static portMUX_TYPE s_out_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t route_mask(const openvrx_route_t *r)
{
    return r->used_mask | r->idle_mask | r->spare_mask;
}

static esp_err_t config_pins(uint32_t mask, gpio_mode_t mode, bool pull_down)
{
    if (!mask) return ESP_OK;
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

static void for_each(uint32_t mask, void (*fn)(int gpio, void *ctx), void *ctx)
{
    for (int g = 0; g < (int)OPENVRX_GPIO_COUNT; ++g)
        if (mask & (1u << g)) fn(g, ctx);
}

static void to_gpio_signal(int g, void *ctx)
{
    (void)ctx;
    esp_rom_gpio_connect_out_signal((uint32_t)g, SIG_GPIO_OUT_IDX, false, false);
}

static void force_oe(int g, void *ctx)
{
    (void)ctx;
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)g, false, false);
}

static void set_drive(int g, void *ctx)
{
    (void)gpio_set_drive_capability((gpio_num_t)g, (gpio_drive_cap_t)*(const unsigned *)ctx);
}

// Writes all lane levels at once (one store to GPIO_OUT).
static IRAM_ATTR void write_lanes(uint32_t levels)
{
    portENTER_CRITICAL_SAFE(&s_out_mux);
    GPIO.out.val = (GPIO.out.val & ~s_route.used_mask) | (levels & s_route.used_mask);
    portEXIT_CRITICAL_SAFE(&s_out_mux);
}

static void attach_modem_diag(void)
{
    for (unsigned k = 0; k < s_route.n; ++k)
        esp_rom_gpio_connect_out_signal((uint32_t)s_route.gpio[k], MODEM_DIAG0_IDX + s_route.lane[k].bus_bit, false,
                                        false);
    for_each(s_route.used_mask, force_oe, NULL);
}

static IRAM_ATTR bool on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    (void)timer;
    (void)edata;
    (void)ctx;
    const uint32_t phase = s_phase + 1u;
    s_phase = phase;
    write_lanes(openvrx_lane_bits_to_gpio(&s_route, openvrx_selftest_bits(s_mode, phase, s_route.n, 0)));
    return false;
}

esp_err_t lanes_hw_init(void)
{
    const gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    esp_err_t err = gptimer_new_timer(&cfg, &s_timer);
    if (err != ESP_OK) return err;
    const gptimer_event_callbacks_t cbs = {.on_alarm = on_alarm};
    err = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (err != ESP_OK) return err;
    return gptimer_enable(s_timer);
}

static void stop_timer(void)
{
    if (s_timer_running) {
        (void)gptimer_stop(s_timer);
        s_timer_running = false;
    }
}

esp_err_t lanes_hw_apply(const openvrx_route_t *r, unsigned drive)
{
    if (drive > OPENVRX_DRIVE_MAX) return ESP_ERR_INVALID_ARG;
    stop_timer();
    s_mode = OPENVRX_ST_OFF;
    const uint32_t keep = route_mask(r) | (r->clk >= 0 ? 1u << r->clk : 0u);
    const uint32_t release = route_mask(&s_route) & ~keep;
    s_route = *r;
    s_drive = drive;
    esp_err_t err;
    // Released pins: no output, no pulls.
    for_each(release, to_gpio_signal, NULL);
    if ((err = config_pins(release, GPIO_MODE_DISABLE, false)) != ESP_OK) return err;
    // Spare pins: high-impedance with a weak pull-down.
    for_each(r->spare_mask, to_gpio_signal, NULL);
    if ((err = config_pins(r->spare_mask, GPIO_MODE_INPUT, true)) != ESP_OK) return err;
    // Idle pins: driven low.
    if ((err = config_pins(r->idle_mask, GPIO_MODE_OUTPUT, false)) != ESP_OK) return err;
    for_each(r->idle_mask, to_gpio_signal, NULL);
    GPIO.out_w1tc.val = r->idle_mask;
    for_each(r->idle_mask, force_oe, NULL);
    // Lanes: GPIO output, then MODEM_DIAG through the matrix.
    if ((err = config_pins(r->used_mask, GPIO_MODE_OUTPUT, false)) != ESP_OK) return err;
    unsigned d = drive;
    for_each(r->used_mask | r->idle_mask, set_drive, &d);
    attach_modem_diag();
    return ESP_OK;
}

const openvrx_route_t *lanes_hw_route(void) { return &s_route; }

esp_err_t lanes_hw_set_drive(unsigned drive)
{
    if (drive > OPENVRX_DRIVE_MAX) return ESP_ERR_INVALID_ARG;
    s_drive = drive;
    for_each(s_route.used_mask | s_route.idle_mask, set_drive, &drive);
    return ESP_OK;
}

esp_err_t lanes_hw_selftest(openvrx_selftest_mode_t mode, uint32_t value, uint32_t phase_us)
{
    stop_timer();
    if (mode == OPENVRX_ST_OFF) {
        s_mode = OPENVRX_ST_OFF;
        attach_modem_diag();
        return ESP_OK;
    }
    if (s_route.n == 0) return ESP_ERR_INVALID_STATE;
    if (mode == OPENVRX_ST_CONST && !openvrx_selftest_const_fits(value, s_route.n)) return ESP_ERR_INVALID_ARG;
    s_mode = mode;
    s_phase_us = mode == OPENVRX_ST_CONST ? 0u : phase_us;
    s_phase = 0;
    // Detach from MODEM_DIAG: the pads now follow GPIO_OUT.
    write_lanes(openvrx_lane_bits_to_gpio(&s_route, openvrx_selftest_bits(mode, 0, s_route.n, value)));
    for_each(s_route.used_mask, to_gpio_signal, NULL);
    for_each(s_route.used_mask, force_oe, NULL);
    if (mode == OPENVRX_ST_CONST) return ESP_OK;
    esp_err_t err = gptimer_set_raw_count(s_timer, 0);
    if (err != ESP_OK) return err;
    const gptimer_alarm_config_t alarm = {
        .alarm_count = phase_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    if ((err = gptimer_set_alarm_action(s_timer, &alarm)) != ESP_OK) return err;
    if ((err = gptimer_start(s_timer)) != ESP_OK) return err;
    s_timer_running = true;
    return ESP_OK;
}

openvrx_selftest_mode_t lanes_hw_selftest_mode(void) { return s_mode; }
uint32_t lanes_hw_selftest_phase_us(void) { return s_phase_us; }

const char *lanes_hw_selftest_name(openvrx_selftest_mode_t mode)
{
    switch (mode) {
    case OPENVRX_ST_WALK: return "walk";
    case OPENVRX_ST_CONST: return "const";
    case OPENVRX_ST_COUNT: return "count";
    default: return "off";
    }
}
