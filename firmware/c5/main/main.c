// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head firmware ("openvrx-c5").
//
// The ESP32-C5 is a pure receive head: Wi-Fi/PHY tunes, MODEM_DIAG streams
// the ADC's I/Q, the GPIO matrix routes the chosen bits to pads, and PARLIO
// TX provides a sample clock locked to the bus. All demodulation happens on
// the PC behind an FX2LP bridge. The receive bring-up is adapted from C5VRX
// by Twotoz and the C5VRX contributors (https://github.com/Twotoz/C5VRX,
// https://twotoz.github.io/C5VRX/); see the headers of rf.c, phy.c and
// logic/arc_gain.c.
#include <stdbool.h>
#include <stdio.h>
#include "boot_guard.h"
#include "console.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radiohead.h"
#include "rf.h"
#include "settings.h"

static volatile bool s_ready;

static void on_line(char *line)
{
    if (!s_ready) {
        // Exactly one reply even while booting.
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        char word[25];
        unsigned n = 0;
        while (p[n] && p[n] != ' ' && p[n] != '\t' && n + 1 < sizeof(word)) { word[n] = p[n]; ++n; }
        word[n] = 0;
        char out[64];
        snprintf(out, sizeof(out), "ERR %s state booting", word);
        console_reply(out);
        return;
    }
    radiohead_command(line);
}

static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default: return "unknown";
    }
}

void app_main(void)
{
    // The console first: download mode and diagnostics must stay reachable
    // whatever a later boot step does.
    const esp_err_t cerr = console_start(on_line);
    console_evt("boot fw=%s reason=%s", esp_app_get_description()->version, reset_reason_name(esp_reset_reason()));
    if (cerr != ESP_OK) console_log("console: %s", esp_err_to_name(cerr));

    // A stalled previous boot (watchdog reset) starts on the defaults so the
    // radio head stays reachable; the saved state stays in flash.
    const bool safe = boot_guard_begin();
    if (safe) {
        console_evt("boot safe=1 stalled=%s", boot_stage_name(boot_guard_stalled()));
        console_log("previous boot stalled at %s; saved settings not applied (send save to overwrite them)",
                    boot_stage_name(boot_guard_stalled()));
    }

    openvrx_settings_t s;
    boot_guard_stage(BOOT_STAGE_SETTINGS);
    const esp_err_t nerr = settings_init();
    if (nerr != ESP_OK) console_log("nvs: %s", esp_err_to_name(nerr));
    radiohead_defaults(&s);
    if (!safe && nerr == ESP_OK && settings_load(&s)) console_log("saved state loaded");
    radiohead_boot(&s);
    boot_guard_done();
    s_ready = true;
    console_evt("phy state=%s", rf_phy_state_name(rf_phy_state()));

    // The receive lock is checked once a second; drift is restored and
    // reported as an EVT phy line.
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        rf_watchdog_poll();
    }
}
