// SPDX-License-Identifier: GPL-3.0-or-later
// Host test for ../ntsc_frame.c: builds every pattern and checks the frame
// against NTSC described independently, as 1050 half-line slots (the
// builder uses a per-line table), plus the timings in microseconds.
//   gcc -O2 -Wall -Wextra -I.. ../ntsc_frame.c ntsc_frame_test.c -lm -o /tmp/ntsc_test && /tmp/ntsc_test
// Optional: /tmp/ntsc_test DIR also writes DIR/<pattern>.u8 (raw frames).
#include "ntsc_frame.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;
#define CHECK(cond, ...)                 \
  do {                                   \
    if (!(cond)) {                       \
      failures++;                        \
      if (failures <= 20) {              \
        printf("FAIL: " __VA_ARGS__);    \
        printf("\n");                    \
      }                                  \
    }                                    \
  } while (0)

// Expected pulse length on half-line slot h (0..1049) of the frame:
// field 1 starts with 6 equalising, 6 broad, 6 equalising half lines at
// line 1; field 2 the same, starting half a line later (middle of line 263).
static int expected_slot(int h)
{
  int v = -1;
  if (h < 18) v = h;
  else if (h >= 525 && h < 543) v = h - 525;
  if (v >= 0) return (v >= 6 && v < 12) ? NTSC_BROAD : NTSC_EQ;
  return (h % 2 == 0) ? NTSC_HSYNC : 0;
}

static double us(int samples) { return samples * 1e6 / NTSC_SAMPLE_HZ; }

static void near(const char *what, double got, double want, double tol)
{
  CHECK(fabs(got - want) <= tol, "%s %.3f, want %.3f +- %.3f", what, got, want, tol);
}

int main(int argc, char **argv)
{
  // Timing against RS-170A / standard.rs.
  double line_hz = (double)NTSC_SAMPLE_HZ / NTSC_SPL;
  near("line rate Hz", line_hz, 4.5e6 / 286.0, 0.02);
  near("hsync us", us(NTSC_HSYNC), 4.7, 0.3);
  near("equalising us", us(NTSC_EQ), 2.3, 0.2);
  near("broad us", us(NTSC_BROAD), 27.1, 0.5);
  near("serration us", us(NTSC_HALF - NTSC_BROAD), 4.7, 0.3);
  near("active start us", us(NTSC_ACT0), 9.4, 0.25);
  near("active width us", us(NTSC_ACTW), 52.66, 0.6);
  CHECK(NTSC_ACT0 + NTSC_ACTW <= NTSC_SPL - 3, "front porch under 1.5 us");

  // Levels at 1 Vpp: 140 IRE over about 77.5 DAC codes.
  ntsc_levels_t lv = ntsc_levels(1000, 3300.0f / 256.0f);
  int sync = ntsc_code(lv, -40), blank = ntsc_code(lv, 0), black = ntsc_code(lv, 7.5f), white = ntsc_code(lv, 100);
  near("white - sync codes", white - sync, 1000.0 / (3300.0 / 256.0), 1.0);
  CHECK(sync < blank && blank < black && black < white, "level order %d %d %d %d", sync, blank, black, white);
  printf("levels at 1000 mV: sync %d blank %d black %d white %d\n", sync, blank, black, white);

  static uint8_t f[NTSC_FRAME_BYTES];
  for (int p = 0; p < PAT_COUNT; p++) {
    ntsc_build_frame(f, (ntsc_pattern_t)p, lv);
    int picture_lines = 0;
    for (int l = 1; l <= NTSC_LINES; l++) {
      const uint8_t *line = f + (l - 1) * NTSC_SPL;
      int sync_samples = 0;
      for (int i = 0; i < NTSC_SPL; i++) sync_samples += line[i] == sync;
      int expect_sync = 0;
      for (int half = 0; half < 2; half++) {
        int h = 2 * (l - 1) + half, start = half * NTSC_HALF, want = expected_slot(h), run = 0;
        while (start + run < NTSC_SPL && line[start + run] == sync) run++;
        CHECK(run == want, "%s line %d half %d: pulse %d samples, want %d", ntsc_pattern_name(p), l, half, run, want);
        expect_sync += want;
      }
      // No sync level anywhere but the pulses (the picture never reaches it).
      CHECK(sync_samples == expect_sync, "%s line %d: %d sync samples, want %d", ntsc_pattern_name(p), l,
            sync_samples, expect_sync);
      int row = ntsc_row(l);
      if (row >= 0) {
        picture_lines++;
        CHECK(expected_slot(2 * (l - 1)) == NTSC_HSYNC && expected_slot(2 * (l - 1) + 1) == 0,
              "picture on a vertical-interval line %d", l);
        // Porches stay at blanking.
        for (int i = NTSC_HSYNC; i < NTSC_ACT0; i++) CHECK(line[i] == blank, "line %d back porch", l);
        for (int i = NTSC_ACT0 + NTSC_ACTW; i < NTSC_SPL; i++) CHECK(line[i] == blank, "line %d front porch", l);
        for (int x = 0; x < NTSC_ACTW; x++) {
          int v = line[NTSC_ACT0 + x];
          CHECK(v >= black && v <= white, "%s line %d x %d: code %d outside black..white",
                ntsc_pattern_name(p), l, x, v);
        }
      } else {
        for (int i = 0; i < NTSC_SPL; i++)
          CHECK(line[i] == sync || line[i] == blank, "line %d: picture outside the active lines", l);
      }
    }
    CHECK(picture_lines == 2 * NTSC_ROWS, "%s: %d picture lines, want 480", ntsc_pattern_name(p), picture_lines);
    // Both fields carry the same rows.
    for (int r = 0; r < NTSC_ROWS; r++)
      for (int i = 0; i < NTSC_SPL; i++)
        CHECK(f[(NTSC_FIRST_ACTIVE_1 - 1 + r) * NTSC_SPL + i] == f[(NTSC_FIRST_ACTIVE_2 - 1 + r) * NTSC_SPL + i],
              "row %d differs between fields", r);
    CHECK(ntsc_pattern_from_name(ntsc_pattern_name(p)) == p, "name round trip %d", p);
    if (argc > 1) {
      char path[512];
      snprintf(path, sizeof path, "%s/%s.u8", argv[1], ntsc_pattern_name(p));
      FILE *o = fopen(path, "wb");
      if (o) {
        fwrite(f, 1, sizeof f, o);
        fclose(o);
      }
    }
  }
  CHECK(ntsc_pattern_from_name("nope") == -1, "unknown name");
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("ntsc_frame: all checks passed (%d patterns, %u Hz, %.4f Hz lines)\n", PAT_COUNT, NTSC_SAMPLE_HZ, line_hz);
  return 0;
}
