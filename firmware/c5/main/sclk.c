// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: the sample clock.
//
// PARLIO TX runs a loop transmission from a small zero buffer with no data
// pins; its output clock (signal PARL_TX_CLK_OUT_IDX) is the TX core clock,
// PLL_F240M divided by an integer (ESP-IDF v6.0.2 parlio_tx.c computes
// out_clk_freq_hz = src / div; esp32c5 parlio_ll.h: integer divider in
// PCR_PARL_CLK_TX_DIV_NUM, PARLIO_LL_TX_MAX_CLK_FRACT_DIV 0, no fractional
// part on the C5). Clock gating by data (tx_gating_en) stays off, so the
// clock never depends on the loop data. The modem's 80 MHz bus comes from
// the same PLL, so a divider that is a multiple of 3 is frequency locked to
// MODEM_DIAG.
//
// The pad is routed through the GPIO matrix by this module (the driver gets
// clk_out_gpio_num = -1), which allows gating at the pad and moving the
// clock to another pin without touching PARLIO. The pad keeps its input
// buffer enabled so the slip routine can watch the clock.
#include "sclk.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/parlio_tx.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_clk.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "hal/misc.h"
#include "openvrx_cmd.h"
#include "openvrx_sclk_logic.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/pcr_struct.h"

#define LOOP_BYTES 512u
#define LOOP_WIDTH 2u   // data lines (none routed); 2 keeps the driver's standard loop EOF path

static parlio_tx_unit_handle_t s_tx;
static uint8_t *s_loop_buf;
static uint32_t s_div;
static int s_pin = -1;
static bool s_on = true;
static unsigned s_drive = 2;
static uint32_t s_slips;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// PCR_PARL_CLK_TX_DIV_NUM holds divider - 1; the counter latches it at its
// next wrap.
static inline __attribute__((always_inline)) void write_div(uint32_t div)
{
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_tx_conf, parl_clk_tx_div_num, div - 1u);
}

static void pad_release(int gpio)
{
    if (gpio < 0) return;
    esp_rom_gpio_connect_out_signal((uint32_t)gpio, SIG_GPIO_OUT_IDX, false, false);
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ull << gpio,
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&cfg);
}

static esp_err_t pad_route(int gpio, bool on)
{
    if (gpio < 0) return ESP_OK;
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ull << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    err = gpio_set_drive_capability((gpio_num_t)gpio, (gpio_drive_cap_t)s_drive);
    if (err != ESP_OK) return err;
    if (on) {
        esp_rom_gpio_connect_out_signal((uint32_t)gpio, PARL_TX_CLK_OUT_IDX, false, false);
    } else {
        gpio_ll_set_level(&GPIO, (uint32_t)gpio, 0);
        esp_rom_gpio_connect_out_signal((uint32_t)gpio, SIG_GPIO_OUT_IDX, false, false);
    }
    // Output enable from GPIO_ENABLE (set by gpio_config), never from the
    // peripheral: the pad drives whatever the matrix selects.
    gpio_ll_set_output_enable_ctrl(&GPIO, (uint8_t)gpio, false, false);
    return ESP_OK;
}

esp_err_t sclk_start(uint32_t div, int gpio, unsigned drive)
{
    if (div < OPENVRX_DIV_MIN || div > OPENVRX_DIV_MAX || drive > OPENVRX_DRIVE_MAX) return ESP_ERR_INVALID_ARG;
    if (s_tx) return ESP_ERR_INVALID_STATE;
    s_drive = drive;
    s_loop_buf = heap_caps_aligned_calloc(64, 1, LOOP_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_loop_buf) return ESP_ERR_NO_MEM;
    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_PLL_F240M,
        .clk_in_gpio_num = GPIO_NUM_NC,
        .output_clk_freq_hz = openvrx_clock_hz(div),
        .data_width = LOOP_WIDTH,
        .clk_out_gpio_num = GPIO_NUM_NC,
        .valid_gpio_num = GPIO_NUM_NC,
        .trans_queue_depth = 1,
        .max_transfer_size = LOOP_BYTES,
        .dma_burst_size = 32,
        .shift_edge = PARLIO_SHIFT_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (unsigned k = 0; k < PARLIO_TX_UNIT_MAX_DATA_WIDTH; ++k) cfg.data_gpio_nums[k] = GPIO_NUM_NC;
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_tx);
    if (err != ESP_OK) return err;
    err = parlio_tx_unit_enable(s_tx);
    if (err != ESP_OK) return err;
    const parlio_transmit_config_t tcfg = {.idle_value = 0, .flags = {.loop_transmission = 1}};
    err = parlio_tx_unit_transmit(s_tx, s_loop_buf, LOOP_BYTES * 8u, &tcfg);
    if (err != ESP_OK) return err;
    // The driver rounds output_clk_freq_hz to the same integer divider; set
    // it explicitly so the register is exactly what the host asked for.
    write_div(div);
    s_div = div;
    s_pin = gpio;
    s_on = true;
    return pad_route(gpio, true);
}

esp_err_t sclk_set_div(uint32_t div)
{
    if (!s_tx) return ESP_ERR_INVALID_STATE;
    if (div < OPENVRX_DIV_MIN || div > OPENVRX_DIV_MAX) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_mux);
    write_div(div);
    s_div = div;
    portEXIT_CRITICAL(&s_mux);
    return ESP_OK;
}

uint32_t sclk_div(void) { return s_div; }
int sclk_pin(void) { return s_pin; }
bool sclk_on(void) { return s_on; }
uint32_t sclk_slips(void) { return s_slips; }

esp_err_t sclk_set_pin(int gpio)
{
    if (gpio == s_pin) return ESP_OK;
    pad_release(s_pin);
    s_pin = gpio;
    return pad_route(gpio, s_on);
}

esp_err_t sclk_gate(bool on)
{
    s_on = on;
    return pad_route(s_pin, on);
}

esp_err_t sclk_set_drive(unsigned drive)
{
    if (drive > OPENVRX_DRIVE_MAX) return ESP_ERR_INVALID_ARG;
    s_drive = drive;
    return s_pin >= 0 ? gpio_set_drive_capability((gpio_num_t)s_pin, (gpio_drive_cap_t)drive) : ESP_OK;
}

typedef struct {
    uint32_t settle, timeout, stretch_at, restore_at;
    uint32_t long_div, user_div;
    uint32_t pin;
} slip_cycles_t;

static inline __attribute__((always_inline)) void wait_cycles(uint32_t from, uint32_t n)
{
    while ((uint32_t)esp_cpu_get_cycle_count() - from < n) {
    }
}

static inline __attribute__((always_inline)) bool wait_level(uint32_t pin, int level, uint32_t timeout)
{
    const uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    while (gpio_ll_get_level(&GPIO, pin) != level)
        if ((uint32_t)esp_cpu_get_cycle_count() - t0 >= timeout) return false;
    return true;
}

// One slip (openvrx_sclk_logic.h explains the timing). Runs with interrupts
// off and from IRAM so cache misses cannot stretch the 1.25 us margins.
static IRAM_ATTR bool slip_once(const slip_cycles_t *c)
{
    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    write_div(c->long_div);
    wait_cycles((uint32_t)esp_cpu_get_cycle_count(), c->settle);
    if (wait_level(c->pin, 0, c->timeout) && wait_level(c->pin, 1, c->timeout)) {
        const uint32_t r = (uint32_t)esp_cpu_get_cycle_count();
        wait_cycles(r, c->stretch_at);
        write_div(c->long_div + 1u);
        wait_cycles(r, c->restore_at);
        ok = true;
    }
    write_div(c->user_div);
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

esp_err_t sclk_slip(unsigned n)
{
    if (!s_tx || s_pin < 0) return ESP_ERR_INVALID_STATE;
    if (!s_on) return ESP_ERR_INVALID_STATE;   // the slip watches the pad
    openvrx_slip_plan_t plan;
    if (!openvrx_slip_plan(s_div, &plan)) return ESP_ERR_INVALID_ARG;
    const uint32_t cpu_hz = (uint32_t)esp_clk_cpu_freq();
    const slip_cycles_t c = {
        .settle = openvrx_ticks_to_cycles(plan.settle_ticks, cpu_hz),
        .timeout = openvrx_ticks_to_cycles(plan.edge_timeout_ticks, cpu_hz),
        .stretch_at = openvrx_ticks_to_cycles(plan.stretch_at_ticks, cpu_hz),
        .restore_at = openvrx_ticks_to_cycles(plan.restore_at_ticks, cpu_hz),
        .long_div = plan.long_div,
        .user_div = s_div,
        .pin = (uint32_t)s_pin,
    };
    for (unsigned k = 0; k < n; ++k) {
        if (!slip_once(&c)) return ESP_ERR_TIMEOUT;
        ++s_slips;
        // Let the last long periods run out (under 10 us) and give other
        // tasks the CPU between slips.
        vTaskDelay(1);
    }
    return ESP_OK;
}
