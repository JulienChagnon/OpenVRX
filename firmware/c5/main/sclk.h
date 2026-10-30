// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: the sample clock (PARLIO TX, PLL_F240M / divider).
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Creates the PARLIO TX unit (no data pins), starts a loop transmission
// and routes the clock to gpio (when gpio >= 0). drive: 0..3.
esp_err_t sclk_start(uint32_t div, int gpio, unsigned drive);
// Divider 6..1023, latched by the hardware at the next clock period.
esp_err_t sclk_set_div(uint32_t div);
uint32_t sclk_div(void);
// Moves the clock to another pad (board change). The old pad is released.
esp_err_t sclk_set_pin(int gpio);
int sclk_pin(void);
// Pad-level gate: off holds the pad low while PARLIO keeps running, so the
// phase against the bus survives off/on.
esp_err_t sclk_gate(bool on);
bool sclk_on(void);
esp_err_t sclk_set_drive(unsigned drive);
// Moves the clock edge one PLL tick later against the bus, n times.
// ESP_ERR_INVALID_STATE while gated, ESP_ERR_TIMEOUT if the pad does not
// toggle (pin shorted or not driven).
esp_err_t sclk_slip(unsigned n);
uint32_t sclk_slips(void);
