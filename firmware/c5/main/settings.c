// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: NVS persistence of the `save` state.
#include "settings.h"

#include "nvs.h"
#include "nvs_flash.h"

#define NAMESPACE "openvrx"
#define KEY "state"

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    return err;
}

bool settings_load(openvrx_settings_t *s)
{
    nvs_handle_t h;
    if (nvs_open(NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t blob[sizeof(openvrx_settings_t) + 8];
    size_t len = sizeof(blob);
    const esp_err_t err = nvs_get_blob(h, KEY, blob, &len);
    nvs_close(h);
    return err == ESP_OK && openvrx_settings_from_blob(blob, (unsigned)len, s);
}

esp_err_t settings_store(const openvrx_settings_t *s)
{
    if (!openvrx_settings_valid(s)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, KEY, s, sizeof(*s));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}
