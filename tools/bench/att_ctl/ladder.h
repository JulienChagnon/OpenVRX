// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: colour NTSC on a 6-bit resistor ladder (docs/BENCH.md).
//
// I2S0 in LCD (parallel) mode clocks 6 pins at 4 x fsc = 14.318182 MHz:
//   GPIO 18 19 21 22 23 26 = bits 0..5 through 8.2k 3.9k 2.0k 1.0k 470 240
//   ohm to the VIDEO node, 200 ohm and 470 pF from VIDEO to GND.
// 910 samples per line (4 per subcarrier cycle, so colour is four fixed
// values per cycle: V, U, -V, -U). The subcarrier flips phase every line and
// repeats every 2 frames, so a circular DMA chain of 1050 descriptors (2
// frames) points into about 20 unique line buffers.
#pragma once
#include "ntsc_frame.h"

// Starts (or restarts) the ladder output. Returns nullptr or why it failed.
const char *ladder_start(ntsc_pattern_t p, int amp_mv);
void ladder_stop();
bool ladder_running();
// Sample rate the APLL really gives (0 when stopped).
float ladder_sample_hz();
