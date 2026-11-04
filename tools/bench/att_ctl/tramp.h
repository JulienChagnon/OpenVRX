// SPDX-License-Identifier: GPL-3.0-or-later
// OpenVRX bench: IRC Tramp VTX control on one wire (docs/BENCH.md): UART2,
// TX on P17 through 1 kohm and RX on P16, both on the VTX's RX pad.
#pragma once
#include <stdint.h>

struct TrampState {
  bool have_limits;   // 'r' answered
  uint16_t min_mhz, max_mhz, max_mw;
  bool have_status;   // 'v' answered
  uint16_t mhz, conf_mw, actual_mw;
  bool pit, race_lock;
  bool have_temp;     // 's' answered
  int16_t temp;
};

void tramp_begin();
// Queries limits, status and temperature. False if the VTX never answered.
bool tramp_query_all(TrampState *st);
// Set and confirm by re-reading the status (a few tries). Returns nullptr
// or why it failed; st holds the last status read.
const char *tramp_set_mhz(uint16_t mhz, TrampState *st);
const char *tramp_set_mw(uint16_t mw, TrampState *st);
const char *tramp_set_pit(bool on, TrampState *st);
// Temperature only (one 's' query). False if no answer.
bool tramp_read_temp(int16_t *temp);
// Band letter (A B E F R) and channel 1..8 to MHz, 0 if unknown.
uint16_t tramp_band_mhz(char band, int ch);
