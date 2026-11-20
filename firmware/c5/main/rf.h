// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from C5VRX by Twotoz and the C5VRX contributors
// (https://github.com/Twotoz/C5VRX), file main/rf.h at commit
// 6bc9e4bbb5bef5948d42a065da27eae39315ad2d. Changes for OpenVRX: the radio
// is a pure receive head; channel tables, lane routing (now lanes_hw.c)
// and bank checks removed; tuning by raw MHz plus kHz offset; 11p
// switchable; PHY state reporting and a watchdog added; DC hold rebuilt
// around a per-row trim table (rf_dc_*).
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
    bool dc;              // hold the DC DACs at the per-row trims (logic/openvrx_dc.h)
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

// DC correction ahead of the ADC (pinned PHY only). While on, the DC DAC
// pair of the gain row in force is held at the vendor code plus that row's
// trim; every gain write, retune and restore releases the hold first and
// takes it again after. The servo (agc.c) moves the trims.
esp_err_t rf_dc_set_enabled(bool on);
bool rf_dc_held(void);
// Changes whenever something other than rf_dc_step moved the DC (gain
// write, retune, restore, hold taken or dropped): a measurement that spans
// a change is stale.
uint32_t rf_dc_epoch(void);
// Adds step to the trim of gain row `gain` (applied at once when that row
// is held). applied/limited as openvrx_dc_rows_step.
esp_err_t rf_dc_step(uint8_t gain, const int step[2], int applied[2], bool limited[2]);
typedef struct {
    bool held;
    bool failed;       // a hold did not read back: off until `dc on` or a retune
    int code[2];       // held bank-2 pair, I then Q
    int base[2];       // the vendor's code of the row in force
    int trim[2];       // code - base as the trim table holds it
    int bank1[2];      // bank 1 forced with the hold (pinned per RF stage)
    uint32_t bank1_overrides;   // holds that found the vendor's bank 1 moved
} rf_dc_info_t;
void rf_dc_info(rf_dc_info_t *out);

// Timing of the last rf_tune (C5-22): the vendor part (channel setup or
// offset helper) and the whole call with the restore; whether it was a
// full channel setup and whether the restore wrote the gain force again.
typedef struct {
    uint32_t vendor_us, total_us;
    bool setup, reforced;
} rf_tune_time_t;
rf_tune_time_t rf_last_tune(void);

// One-shot 10-bit I/Q capture (C5-20, words decoded by logic/openvrx_iq.h).
// The dump engine that streams the lanes is stopped, writes n words into
// SRAM on a software trigger, and streams again: the lanes pause for
// meta->us. ESP_OK when the engine reported done and wrote exactly the n
// words; ESP_ERR_TIMEOUT without done; ESP_ERR_INVALID_RESPONSE when words
// are missing or it wrote past them; ESP_ERR_INVALID_STATE when the radio or
// the stream is down. words gets the n words in every case.
typedef struct {
    uint32_t us;        // from stopping the stream to restarting it
    uint32_t ptr;       // the engine's pointer after the capture (DUMP_PTR_MODE bits 15:0)
    unsigned written;   // of the n words
    bool done;          // the engine's done bit came within the timeout
    bool guard_ok;      // the word after the n was left alone
} rf_iq_meta_t;
esp_err_t rf_iq_capture(uint32_t *words, unsigned n, rf_iq_meta_t *meta);
// rf_iq_capture with the whole 64 KB capture RAM filled and scanned after:
// where the engine wrote (bench check of the capture layout).
typedef struct {
    rf_iq_meta_t meta;
    unsigned changed;   // words of the 16384 that changed
    int first, last;    // first and last changed index, -1 when none
    unsigned gain_forced;
} rf_iq_probe_t;
esp_err_t rf_iq_probe(uint32_t *words, unsigned n, rf_iq_probe_t *p);

// Checks the receive lock (modem streaming, MAC TX queues off, forced gain,
// 11p state, DC hold) and restores it when something drifted. Emits EVT phy lines.
void rf_watchdog_poll(void);

// Stops the modem writer and Wi-Fi and restarts into the app. A plain
// esp_restart() is a CPU reset that leaves the modem domain (forced clocks,
// dump writer) running, and the next boot stalls. Never returns.
void rf_reboot(void) __attribute__((noreturn));

// Stops the modem writer and Wi-Fi, requests a ROM download boot and
// restarts (esptool can then flash with --before no-reset). Never returns.
void rf_reboot_to_download(void) __attribute__((noreturn));
