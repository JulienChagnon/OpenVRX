// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: NTSC test frames for the ESP32 DAC (see ntsc_frame.h).
#include "ntsc_frame.h"

#include <math.h>
#include <string.h>

static const char *const NAMES[PAT_COUNT] = {"card", "bars", "ramp", "check", "grey", "black", "white"};

#define IRE_SYNC (-40.0f)
#define IRE_BLACK 7.5f
#define IRE_WHITE 100.0f

ntsc_levels_t ntsc_levels(int amp_mv, float mv_per_code)
{
  ntsc_levels_t lv;
  // Sync tip a few codes above 0, off the bottom of the DAC's range. The
  // VTX input is AC coupled, so only the swing matters.
  lv.base = 4.0f;
  lv.per_ire = (float)amp_mv / 140.0f / mv_per_code;
  return lv;
}

uint8_t ntsc_code(ntsc_levels_t lv, float ire)
{
  float c = lv.base + (ire - IRE_SYNC) * lv.per_ire;
  if (c < 0.0f) c = 0.0f;
  if (c > 255.0f) c = 255.0f;
  return (uint8_t)lroundf(c);
}

// Same table as line_layout() in openvrx-sim/src/cvbs.rs (NTSC).
void ntsc_line_pulses(int l, ntsc_pulse_t out[2])
{
  ntsc_pulse_t a = NTSC_PULSE_HSYNC, b = NTSC_PULSE_NONE;
  if ((l >= 1 && l <= 3) || (l >= 7 && l <= 9)) {
    a = NTSC_PULSE_EQ;
    b = NTSC_PULSE_EQ;
  } else if (l >= 4 && l <= 6) {
    a = NTSC_PULSE_BROAD;
    b = NTSC_PULSE_BROAD;
  } else if (l == 263) {
    b = NTSC_PULSE_EQ;
  } else if (l == 264 || l == 265 || l == 270 || l == 271) {
    a = NTSC_PULSE_EQ;
    b = NTSC_PULSE_EQ;
  } else if (l == 266) {
    a = NTSC_PULSE_EQ;
    b = NTSC_PULSE_BROAD;
  } else if (l == 267 || l == 268) {
    a = NTSC_PULSE_BROAD;
    b = NTSC_PULSE_BROAD;
  } else if (l == 269) {
    a = NTSC_PULSE_BROAD;
    b = NTSC_PULSE_EQ;
  } else if (l == 272) {
    a = NTSC_PULSE_EQ;
  }
  out[0] = a;
  out[1] = b;
}

int ntsc_row(int l)
{
  if (l >= NTSC_FIRST_ACTIVE_1 && l < NTSC_FIRST_ACTIVE_1 + NTSC_ROWS) return l - NTSC_FIRST_ACTIVE_1;
  if (l >= NTSC_FIRST_ACTIVE_2 && l < NTSC_FIRST_ACTIVE_2 + NTSC_ROWS) return l - NTSC_FIRST_ACTIVE_2;
  return -1;
}

static float grey_step(int x, int steps)
{
  int s = x * steps / NTSC_ACTW;
  return IRE_BLACK + (IRE_WHITE - IRE_BLACK) * (float)s / (float)(steps - 1);
}

static float ramp(int x)
{
  return IRE_BLACK + (IRE_WHITE - IRE_BLACK) * (float)x / (float)(NTSC_ACTW - 1);
}

// Cells of 8 samples by 20 rows, about square on a 4:3 screen (one sample
// is W/106 wide, one field row W/320 high).
static float check(int x, int row)
{
  return (((x / 8) + (row / 20)) & 1) ? IRE_WHITE : IRE_BLACK;
}

// Stripes in four zones, periods 2, 4, 8 and 16 samples
// (1.0, 0.5, 0.25, 0.13 MHz): how much detail survives the link.
static float stripes(int x)
{
  int zone = x * 4 / NTSC_ACTW;
  int half = 1 << zone;
  return ((x / half) & 1) ? IRE_WHITE : IRE_BLACK;
}

float ntsc_pattern_ire(ntsc_pattern_t p, int x, int row)
{
  switch (p) {
  case PAT_CARD:
    if (row < 60) return grey_step(x, 8);
    if (row < 120) return stripes(x);
    if (row < 180) return ramp(x);
    return check(x, row - 180);
  case PAT_BARS:
    return grey_step(x, 8);
  case PAT_RAMP:
    return ramp(x);
  case PAT_CHECK:
    return check(x, row);
  case PAT_GREY:
    return 50.0f;
  case PAT_BLACK:
    return IRE_BLACK;
  case PAT_WHITE:
  default:
    return IRE_WHITE;
  }
}

const char *ntsc_pattern_name(ntsc_pattern_t p)
{
  return (p >= 0 && p < PAT_COUNT) ? NAMES[p] : "?";
}

int ntsc_pattern_from_name(const char *name)
{
  for (int i = 0; i < PAT_COUNT; i++)
    if (strcmp(name, NAMES[i]) == 0) return i;
  return -1;
}

static int pulse_len(ntsc_pulse_t p)
{
  switch (p) {
  case NTSC_PULSE_HSYNC: return NTSC_HSYNC;
  case NTSC_PULSE_EQ: return NTSC_EQ;
  case NTSC_PULSE_BROAD: return NTSC_BROAD;
  default: return 0;
  }
}

void ntsc_build_frame(uint8_t *frame, ntsc_pattern_t p, ntsc_levels_t lv)
{
  const uint8_t sync = ntsc_code(lv, IRE_SYNC);
  const uint8_t blank = ntsc_code(lv, 0.0f);
  for (int l = 1; l <= NTSC_LINES; l++) {
    uint8_t *line = frame + (l - 1) * NTSC_SPL;
    memset(line, blank, NTSC_SPL);
    ntsc_pulse_t pulses[2];
    ntsc_line_pulses(l, pulses);
    memset(line, sync, pulse_len(pulses[0]));
    memset(line + NTSC_HALF, sync, pulse_len(pulses[1]));
    int row = ntsc_row(l);
    if (row < 0) continue;
    for (int x = 0; x < NTSC_ACTW; x++) line[NTSC_ACT0 + x] = ntsc_code(lv, ntsc_pattern_ire(p, x, row));
  }
}
