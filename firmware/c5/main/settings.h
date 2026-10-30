// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: NVS persistence of the `save` state.
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "openvrx_settings_logic.h"

// Initialises NVS (erasing it when its layout is outdated).
esp_err_t settings_init(void);
// True and *s filled when a valid saved state exists.
bool settings_load(openvrx_settings_t *s);
esp_err_t settings_store(const openvrx_settings_t *s);
