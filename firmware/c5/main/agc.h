// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX C5 radio head: automatic gain task (logic in logic/openvrx_agc.c).
#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Starts the task (idle until agc_set_enabled(true) and the radio runs).
esp_err_t agc_start(bool enabled);
void agc_set_enabled(bool on);
bool agc_enabled(void);
// Last measurement: clipped share in per mille, RMS per component in
// hundredths of a step, and whether the current preset can be measured.
void agc_last(unsigned *clip_permille, unsigned *rms_centi, bool *measurable);
// Fast-attack gain drops since boot (C5-4).
unsigned agc_attacks(void);
// 5 ms batches left out of the 100 ms measurements as interference bursts since boot (C5-21).
unsigned agc_bursts(void);
// Last measured I and Q mean in ADC units (full scale 512, half-step centred).
void agc_last_dc(int dc_adc[2]);

// C5-20: the loop's batches from one-shot 10-bit captures (rf_iq_capture)
// instead of the pads. Not saved: every boot starts on the pads. strict
// also drops a capture whose gain field differs from the index in force.
void agc_set_iq(bool on, bool strict);
bool agc_iq(void);
typedef struct {
    bool on, strict;
    uint32_t ok, fail;        // captures used and rejected since iq on
    unsigned us;              // the lanes' pause in the last capture
    uint32_t us_max;
    uint32_t gain_bad;        // sample words whose gain field differed from the index in force
    unsigned rail_permille;   // values at the ADC rails in the last update (0 on pad reads)
} agc_iq_stats_t;
void agc_iq_stats(agc_iq_stats_t *s);
