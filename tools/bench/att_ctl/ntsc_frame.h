// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: one 525-line interlaced monochrome NTSC frame for the ESP32
// DAC, 128 samples per line. Plain C with no ESP-IDF calls, so
// test/ntsc_frame_test.c can build and check it on the PC.
//
// Timing follows host/crates/openvrx-sim/src/cvbs.rs (line_layout) and
// openvrx-core/src/standard.rs, rounded to whole samples at 2.013986 MS/s
// (one sample 0.4965 us):
//   line 128 (63.556 us), half line 64, hsync 9 (4.47 us), equalising
//   pulse 5 (2.48 us), broad pulse 55 (27.3 us, serration 4.47 us),
//   picture from sample 19 (9.43 us) for 106 samples (52.6 us).
// Picture: frame lines 21..260 (field 1) and 283..522 (field 2), 240 rows
// each; both fields carry the same rows, so nothing twitters.
// No colour: a colour burst needs 3.58 MHz, above this sample rate.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTSC_LINES 525
#define NTSC_SPL 128
#define NTSC_HALF 64
#define NTSC_FRAME_BYTES (NTSC_LINES * NTSC_SPL)
#define NTSC_SAMPLE_HZ 2013986u   // 128 * 4.5 MHz / 286
#define NTSC_HSYNC 9
#define NTSC_EQ 5
#define NTSC_BROAD 55
#define NTSC_ACT0 19
#define NTSC_ACTW 106
#define NTSC_ROWS 240
#define NTSC_FIRST_ACTIVE_1 21
#define NTSC_FIRST_ACTIVE_2 283

typedef enum {
  NTSC_PULSE_NONE,
  NTSC_PULSE_HSYNC,
  NTSC_PULSE_EQ,
  NTSC_PULSE_BROAD
} ntsc_pulse_t;

typedef enum {
  PAT_CARD,    // grey steps, stripes (1 MHz down to 0.13 MHz), ramp, checkerboard
  PAT_BARS,    // 8 grey steps, full height
  PAT_RAMP,    // black to white, full height
  PAT_CHECK,   // checkerboard, about square cells
  PAT_GREY,    // flat 50 IRE (noise and C/N on a flat field)
  PAT_BLACK,
  PAT_WHITE,
  PAT_COUNT
} ntsc_pattern_t;

// DAC code of the sync tip and DAC codes per IRE.
typedef struct {
  float base;
  float per_ire;
} ntsc_levels_t;

// amp_mv: sync tip to peak white (140 IRE), 1000 for a standard 1 Vpp
// signal. mv_per_code: DAC step, about 3300 / 256 = 12.9 mV.
ntsc_levels_t ntsc_levels(int amp_mv, float mv_per_code);
// DAC code of an IRE value (sync -40, blanking 0, black 7.5, white 100).
uint8_t ntsc_code(ntsc_levels_t lv, float ire);
// Pulses starting at 0H and at H/2 on a frame line (1..525).
void ntsc_line_pulses(int frame_line, ntsc_pulse_t out[2]);
// Picture row (0..239) of a frame line, or -1 outside the picture.
int ntsc_row(int frame_line);
// Pattern value in IRE at picture sample x (0..105) and row (0..239).
float ntsc_pattern_ire(ntsc_pattern_t p, int x, int row);
const char *ntsc_pattern_name(ntsc_pattern_t p);
// Pattern by name, or -1.
int ntsc_pattern_from_name(const char *name);
// Fills frame[NTSC_FRAME_BYTES], frame line 1 first.
void ntsc_build_frame(uint8_t *frame, ntsc_pattern_t p, ntsc_levels_t lv);

#ifdef __cplusplus
}
#endif
