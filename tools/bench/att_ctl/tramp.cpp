// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: IRC Tramp VTX control (see tramp.h).
//
// Frames are 16 bytes both ways: 0x0F, a command letter, a 16-bit value
// (little endian) in bytes 2..3, zeros, a checksum in byte 14 (sum of bytes
// 1..13, 8 bit) and 0 in byte 15. Queries: 'r' limits (min MHz 2..3, max MHz
// 4..5, max mW 6..7), 'v' status (MHz 2..3, configured mW 4..5, control
// flags 6 with race lock in bit 0, pit mode 7, actual mW 8..9), 's'
// temperature (6..7, signed). Sets: 'F' MHz, 'P' mW, 'I' pit (value 0 = pit
// on, 1 = pit off); the VTX does not answer sets, so each is confirmed with
// a fresh 'v'.
//
// One wire: the ESP32 hears its own request. A real answer always has a
// non-zero value field (a frequency, limits or a temperature); the echo of a
// query carries zeros there, so it is skipped by that test.
#include "tramp.h"

#include <Arduino.h>

static const int PIN_RX = 16;
static const int PIN_TX = 17;
static const int GAP_MS = 200;      // minimum time between requests
static const int REPLY_MS = 250;    // wait for an answer
static const int SET_TRIES = 4;

static uint32_t s_last_ms = 0;

static uint8_t checksum(const uint8_t *f)
{
  uint8_t c = 0;
  for (int i = 1; i < 14; i++) c += f[i];
  return c;
}

static void send(char cmd, uint16_t value)
{
  uint8_t f[16] = {0};
  f[0] = 0x0F;
  f[1] = (uint8_t)cmd;
  f[2] = value & 0xFF;
  f[3] = value >> 8;
  f[14] = checksum(f);
  int32_t wait = (int32_t)(s_last_ms + GAP_MS - millis());
  if (wait > 0) delay(wait);
  while (Serial2.available()) Serial2.read();
  Serial2.write(f, sizeof f);
  Serial2.flush();
  s_last_ms = millis();
}

static uint16_t u16(const uint8_t *f, int at)
{
  return (uint16_t)(f[at] | (f[at + 1] << 8));
}

// Waits for a valid answer to query `cmd` (echo skipped). True if found.
static bool receive(char cmd, uint8_t out[16])
{
  uint8_t w[16];
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)REPLY_MS) {
    if (!Serial2.available()) {
      delay(1);
      continue;
    }
    uint8_t b = (uint8_t)Serial2.read();
    if (n == 0 && b != 0x0F) continue;
    w[n++] = b;
    if (n == 2 && w[1] != (uint8_t)cmd) {
      n = (b == 0x0F) ? 1 : 0;  // resync
      if (n) w[0] = 0x0F;
      continue;
    }
    if (n < 16) continue;
    n = 0;
    if (w[14] != checksum(w) || w[15] != 0) continue;
    bool real = (cmd == 'r' || cmd == 'v') ? u16(w, 2) != 0 : u16(w, 6) != 0;
    if (!real) continue;  // our own request coming back
    memcpy(out, w, 16);
    return true;
  }
  return false;
}

static bool query(char cmd, uint8_t out[16])
{
  send(cmd, 0);
  return receive(cmd, out);
}

static bool query_status(TrampState *st)
{
  uint8_t f[16];
  if (!query('v', f)) return false;
  st->have_status = true;
  st->mhz = u16(f, 2);
  st->conf_mw = u16(f, 4);
  st->race_lock = (f[6] & 0x01) != 0;
  st->pit = f[7] != 0;
  st->actual_mw = u16(f, 8);
  return true;
}

void tramp_begin()
{
  Serial2.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
}

bool tramp_query_all(TrampState *st)
{
  *st = TrampState();
  uint8_t f[16];
  if (query('r', f)) {
    st->have_limits = true;
    st->min_mhz = u16(f, 2);
    st->max_mhz = u16(f, 4);
    st->max_mw = u16(f, 6);
  }
  query_status(st);
  if (query('s', f)) {
    st->have_temp = true;
    st->temp = (int16_t)u16(f, 6);
  }
  return st->have_limits || st->have_status || st->have_temp;
}

bool tramp_read_temp(int16_t *temp)
{
  uint8_t f[16];
  if (!query('s', f)) return false;
  *temp = (int16_t)u16(f, 6);
  return true;
}

// Sends one set until a status read shows `done`, or gives up.
template <typename Done>
static const char *set_and_confirm(char cmd, uint16_t value, TrampState *st, Done done)
{
  *st = TrampState();
  if (!query_status(st)) return "no answer from the VTX";
  if (cmd != 'I' && st->race_lock) return "race lock is on (the VTX ignores channel and power)";
  for (int i = 0; i < SET_TRIES; i++) {
    if (done(*st)) return nullptr;
    send(cmd, value);
    delay(GAP_MS);
    // The VTX sometimes misses the first read right after a change
    // (seen leaving pit mode, 2026-11-04): ask once more.
    if (!query_status(st) && !query_status(st)) return "no answer after the set";
  }
  return done(*st) ? nullptr : "the VTX did not take it";
}

const char *tramp_set_mhz(uint16_t mhz, TrampState *st)
{
  return set_and_confirm('F', mhz, st, [mhz](const TrampState &s) { return s.mhz == mhz; });
}

const char *tramp_set_mw(uint16_t mw, TrampState *st)
{
  return set_and_confirm('P', mw, st, [mw](const TrampState &s) { return s.conf_mw == mw; });
}

const char *tramp_set_pit(bool on, TrampState *st)
{
  return set_and_confirm('I', on ? 0 : 1, st, [on](const TrampState &s) { return s.pit == on; });
}

uint16_t tramp_band_mhz(char band, int ch)
{
  static const uint16_t T[5][8] = {
      {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725},  // A
      {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},  // B
      {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945},  // E
      {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880},  // F
      {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917},  // R
  };
  const char *bands = "ABEFR";
  const char *p = strchr(bands, toupper(band));
  if (!p || !*p || ch < 1 || ch > 8) return 0;
  return T[p - bands][ch - 1];
}
