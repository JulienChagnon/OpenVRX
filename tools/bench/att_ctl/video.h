// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: NTSC test pattern on GPIO25 (DAC channel 0) through the
// DAC continuous driver; DMA replays one frame forever, no CPU time.
#pragma once
#include "ntsc_frame.h"

// Starts or restarts the output. Returns nullptr, or why it failed.
const char *video_start(ntsc_pattern_t p, int amp_mv);
void video_stop();
bool video_running();
// Line rate counted from DMA buffer-done interrupts over ms milliseconds,
// 0 if off or no interrupts arrive. Timed on the same crystal as the APLL,
// so it checks the clock setup (dividers), not the crystal's ppm.
float video_measure_line_hz(int ms);
