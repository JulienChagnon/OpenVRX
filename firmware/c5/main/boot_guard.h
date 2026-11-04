// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: boot guard. A hardware watchdog (RWDT) runs while
// the radio is brought up, and each step leaves a breadcrumb in RTC memory
// that survives the reset. A boot that stalls is reset by the watchdog; the
// next boot sees the breadcrumb, starts in safe mode with the default
// settings (the saved ones are kept, not applied) and reports the step.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOOT_STAGE_NONE = 0,
    BOOT_STAGE_SETTINGS = 1,
    BOOT_STAGE_LANES_INIT = 2,
    BOOT_STAGE_CLOCK = 3,
    BOOT_STAGE_ROUTE = 4,
    BOOT_STAGE_RADIO = 5,
    BOOT_STAGE_READY = 6,
} boot_stage_t;

// Reads the previous boot's breadcrumb and arms the watchdog. Returns true
// when the previous boot stalled (start in safe mode).
bool boot_guard_begin(void);

// Records the step about to run.
void boot_guard_stage(boot_stage_t stage);

// Boot finished: disarms the watchdog.
void boot_guard_done(void);

// Step the previous boot stalled at, or BOOT_STAGE_NONE.
boot_stage_t boot_guard_stalled(void);

const char *boot_stage_name(boot_stage_t stage);
