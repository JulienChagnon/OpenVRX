// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: the persisted state blob (pure logic).
#include "openvrx_settings_logic.h"

#include <string.h>
#include "openvrx_cmd.h"

void openvrx_settings_default(openvrx_settings_t *s, openvrx_board_id_t board, openvrx_preset_t preset)
{
    memset(s, 0, sizeof(*s));
    s->version = OPENVRX_SETTINGS_VERSION;
    s->board = (uint8_t)board;
    s->preset = (uint8_t)preset;
    s->p11 = 1;
    s->div = 18;            // 13.33 MS/s
    s->mhz = 5732;          // R3
    s->off_khz = 0;
    s->gain = 64;
    s->filter_code = OPENVRX_FILTER_CAL;
    s->skirt = 0;
    s->bw_mhz = 40;
    s->drive_lanes = 1;
    s->drive_clk = 2;
    s->agc = 1;
    s->dc = 1;
}

bool openvrx_settings_valid(const openvrx_settings_t *s)
{
    if (s->version != OPENVRX_SETTINGS_VERSION) return false;
    if (s->board >= OPENVRX_BOARD_COUNT) return false;
    if (s->preset >= OPENVRX_PRESET_COUNT && s->preset != OPENVRX_PRESET_NONE) return false;
    if (s->p11 > 1) return false;
    if (s->div < OPENVRX_DIV_MIN || s->div > OPENVRX_DIV_MAX) return false;
    if (s->mhz < OPENVRX_TUNE_MIN_MHZ || s->mhz > OPENVRX_TUNE_MAX_MHZ) return false;
    if (s->off_khz < -OPENVRX_OFFSET_MAX_KHZ || s->off_khz > OPENVRX_OFFSET_MAX_KHZ) return false;
    if (s->filter_code != OPENVRX_FILTER_CAL && (s->filter_code < 0 || s->filter_code > OPENVRX_FILTER_CODE_MAX))
        return false;
    if (s->skirt > OPENVRX_FILTER_SKIRT_MAX) return false;
    if (s->bw_mhz != 20 && s->bw_mhz != 40) return false;
    if (s->drive_lanes > OPENVRX_DRIVE_MAX || s->drive_clk > OPENVRX_DRIVE_MAX) return false;
    if (s->agc > 1) return false;
    if (s->dc > 1) return false;
    return true;
}

bool openvrx_settings_from_blob(const void *blob, unsigned len, openvrx_settings_t *out)
{
    openvrx_settings_t s;
    memset(&s, 0, sizeof(s));
    const uint8_t version = len ? ((const uint8_t *)blob)[0] : 0u;
    if (len == sizeof(s) && version == OPENVRX_SETTINGS_VERSION) {
        memcpy(&s, blob, sizeof(s));
    } else if (len >= OPENVRX_SETTINGS_V2_FIELDS && len <= sizeof(s) && version == 2u) {
        memcpy(&s, blob, OPENVRX_SETTINGS_V2_FIELDS);
        s.version = OPENVRX_SETTINGS_VERSION;
        s.dc = 1;
    } else if (len == OPENVRX_SETTINGS_V1_SIZE && version == 1u) {
        memcpy(&s, blob, OPENVRX_SETTINGS_V1_SIZE);
        s.version = OPENVRX_SETTINGS_VERSION;
        s.agc = 1;
        s.dc = 1;
    } else {
        return false;
    }
    if (!openvrx_settings_valid(&s)) return false;
    *out = s;
    return true;
}
