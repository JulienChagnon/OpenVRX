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
// Last measured I and Q mean in ADC units (full scale 512, half-step centred).
void agc_last_dc(int dc_adc[2]);
