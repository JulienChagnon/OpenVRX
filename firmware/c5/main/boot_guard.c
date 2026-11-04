// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: boot guard (see boot_guard.h).
#include "boot_guard.h"

#include "esp_attr.h"
#include "hal/wdt_hal.h"

#define GUARD_MAGIC 0x50434247u   // "PCBG"
// RTC slow clock ticks: about 6 s at 136 kHz, longer on a 32 kHz clock.
// A good boot takes well under a second.
#define GUARD_TICKS 800000u

static RTC_NOINIT_ATTR uint32_t s_magic;
static RTC_NOINIT_ATTR uint32_t s_stage;
static boot_stage_t s_stalled;
static wdt_hal_context_t s_rwdt = RWDT_HAL_CONTEXT_DEFAULT();

bool boot_guard_begin(void)
{
    s_stalled = BOOT_STAGE_NONE;
    if (s_magic == GUARD_MAGIC && s_stage != BOOT_STAGE_READY && s_stage <= BOOT_STAGE_RADIO)
        s_stalled = (boot_stage_t)s_stage;
    s_magic = GUARD_MAGIC;
    s_stage = BOOT_STAGE_NONE;

    wdt_hal_write_protect_disable(&s_rwdt);
    wdt_hal_config_stage(&s_rwdt, WDT_STAGE0, GUARD_TICKS, WDT_STAGE_ACTION_RESET_SYSTEM);
    wdt_hal_enable(&s_rwdt);
    wdt_hal_write_protect_enable(&s_rwdt);
    return s_stalled != BOOT_STAGE_NONE;
}

void boot_guard_stage(boot_stage_t stage)
{
    s_stage = stage;
}

void boot_guard_done(void)
{
    s_stage = BOOT_STAGE_READY;
    wdt_hal_write_protect_disable(&s_rwdt);
    wdt_hal_disable(&s_rwdt);
    wdt_hal_write_protect_enable(&s_rwdt);
}

boot_stage_t boot_guard_stalled(void)
{
    return s_stalled;
}

const char *boot_stage_name(boot_stage_t stage)
{
    switch (stage) {
    case BOOT_STAGE_NONE: return "none";
    case BOOT_STAGE_SETTINGS: return "settings";
    case BOOT_STAGE_LANES_INIT: return "lanes_init";
    case BOOT_STAGE_CLOCK: return "clock";
    case BOOT_STAGE_ROUTE: return "route";
    case BOOT_STAGE_RADIO: return "radio";
    case BOOT_STAGE_READY: return "ready";
    default: return "?";
    }
}
