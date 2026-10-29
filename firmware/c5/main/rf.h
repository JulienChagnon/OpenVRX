// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/rf.h at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d. Changes for OpenVRX: the radio
// is a pure receive head; channel tables, lane routing (now lanes_hw.c),
// DC hold and bank checks removed; tuning by raw MHz plus kHz offset; 11p
// switchable; PHY state reporting and a watchdog added.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t mhz;
    int16_t off_khz;
    uint8_t gain;
    bool p11;
    int8_t filter_code;   // OPENVRX_FILTER_CAL (-1) or 0..63
    uint8_t skirt;
    bool bw20;
} rf_config_t;

typedef enum {
    RF_PHY_OK = 0,        // every requested setting read back as set
    RF_PHY_RESTORED,      // the watchdog found drift and restored the state
    RF_PHY_ERROR,         // a setting did not read back
    RF_PHY_UNSUPPORTED,   // PHY not pinned: filter/11p/digital filter untouched
    RF_PHY_DOWN,          // radio not started
} rf_phy_state_t;

// Starts Wi-Fi as a receive-only front end (NVS must be initialised) and
// applies the configuration. Gain is clamped to the table.
esp_err_t rf_start(const rf_config_t *cfg);
bool rf_started(void);

esp_err_t rf_tune(uint16_t mhz, int off_khz);
// ESP_ERR_INVALID_ARG when the index is above the gain table maximum.
esp_err_t rf_set_gain(uint8_t gain);
// ESP_ERR_NOT_SUPPORTED without the pinned PHY.
esp_err_t rf_set_p11(bool on);
esp_err_t rf_set_filter(int code, int skirt);
esp_err_t rf_set_bw20(bool bw20);

rf_config_t rf_config(void);
rf_phy_state_t rf_phy_state(void);
const char *rf_phy_state_name(rf_phy_state_t s);
uint8_t rf_gain_max(void);
// First gain index of each RF stage; returns the count.
unsigned rf_gain_stage_starts(uint8_t *starts, unsigned max);

// Checks the receive lock (modem streaming, MAC TX queues off, forced gain,
// 11p state) and restores it when something drifted. Emits EVT phy lines.
void rf_watchdog_poll(void);

// Stops the modem writer and Wi-Fi and restarts into the app. A plain
// esp_restart() is a CPU reset that leaves the modem domain (forced clocks,
// dump writer) running, and the next boot stalls. Never returns.
void rf_reboot(void) __attribute__((noreturn));

// Stops the modem writer and Wi-Fi, requests a ROM download boot and
// restarts (esptool can then flash with --before no-reset). Never returns.
void rf_reboot_to_download(void) __attribute__((noreturn));
