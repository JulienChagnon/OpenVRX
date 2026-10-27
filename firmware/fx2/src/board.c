// SPDX-License-Identifier: GPL-3.0-or-later
//
// Board services: 1 ms timer (uptime, status tick, IFCLK activity sampling,
// LED blink, I2C watchdog), I2C EEPROM detection and access, board ID.

#include <fx2regs.h>
#include <fx2ints.h>
#include <fx2i2c.h>
#include <fx2eeprom.h>
#include "openvrx.h"

volatile __data uint32_t uptime_ms;
volatile __bit fifo_moved;
volatile __bit led_blink;
volatile __data uint8_t status_interval = 8;
volatile __data uint8_t i2c_guard_ms;

__xdata uint8_t  eeprom_chip;
__xdata uint8_t  eeprom_2byte;
__xdata uint32_t board_id;
__xdata uint32_t board_id_source;

// Timer 2 runs from CLKOUT/12 = 4 MHz at the 48 MHz CPU clock.
#define T2_RELOAD (65536u - 4000u)

#define LED_HALF_PERIOD_MS 125

static __data uint8_t tick_left = 8;
static __data uint8_t led_left = LED_HALF_PERIOD_MS;
static __data uint8_t last_bcl;
static __data uint8_t last_bch;

void isr_TF2(void) __interrupt(_INT_TF2) {
  uint8_t l, h;

  // The compiler saves DPL/DPH of whichever pointer is selected; make sure
  // that is DPTR0 (same precaution as the libfx2 SETUP handler).
__asm
  push _DPS
  mov  _DPS, #0
__endasm;

  TF2 = 0;
  uptime_ms++;

  // Data moved into the FIFO (byte count changed) or out over USB (EP2
  // buffer-available request bit) since the previous millisecond.
  l = EP2FIFOBCL;
  h = EP2FIFOBCH;
  if(l != last_bcl || h != last_bch || (EPIRQ & _EPI_EP2)) {
    fifo_moved = 1;
    last_bcl = l;
    last_bch = h;
  }
  EPIRQ = _EPI_EP2;

  if(--tick_left == 0) {
    tick_left = status_interval;
    svc_tick = 1;
    svc = 1;
  }

  if(led_blink) {
    if(--led_left == 0) {
      led_left = LED_HALF_PERIOD_MS;
      IOA ^= 0x03;
    }
  }

  if(i2c_guard_ms != 0) {
    if(--i2c_guard_ms == 0)
      i2c_cancel = true;
  }

__asm
  pop  _DPS
__endasm;
}

void timer_init(void) {
  T2CON = 0;                    // 16-bit auto-reload timer, stopped
  CKCON &= ~_T2M;               // CLKOUT / 12
  RCAP2 = T2_RELOAD;
  T2 = T2_RELOAD;
  TF2 = 0;
  ET2 = 1;
  TR2 = 1;
}

uint32_t uptime_read(void) __reentrant {
  uint32_t v;
  ET2 = 0;
  v = uptime_ms;
  ET2 = 1;
  return v;
}

void led_set(bool on) __reentrant {
  if(on) {
    IOA |= 0x03;
    OEA |= 0x03;
    led_left = LED_HALF_PERIOD_MS;
    led_blink = 1;
  } else {
    led_blink = 0;
    OEA &= ~0x03;               // release PA0/PA1
  }
}

// ---------------------------------------------------------------------------
// I2C with a timeout
// ---------------------------------------------------------------------------

static void guard_begin(uint8_t ms) {
  i2c_cancel = false;
  i2c_guard_ms = ms;
}

static void guard_end(void) {
  i2c_guard_ms = 0;
  i2c_cancel = false;
}

static bool probe(uint8_t chip) __reentrant {
  bool ok;
  guard_begin(20);
  ok = i2c_start(chip << 1);
  i2c_stop();
  guard_end();
  return ok;
}

// Same rule as the FX2 boot ROM: a one-byte-address EEPROM is strapped to
// 0x50, a two-byte-address EEPROM to 0x51. The boot ROM leaves what it found
// in I2CS.ID[1:0]; trust that first, then probe.
void eeprom_detect(void) {
  uint8_t id = I2CS & (_ID1 | _ID0);

  eeprom_chip = 0;
  eeprom_2byte = 0;

  if(id == _ID1 && probe(0x51)) {
    eeprom_chip = 0x51;
    eeprom_2byte = 1;
  } else if(id == _ID0 && probe(0x50)) {
    eeprom_chip = 0x50;
  } else if(probe(0x50)) {
    eeprom_chip = 0x50;
  } else if(probe(0x51)) {
    eeprom_chip = 0x51;
    eeprom_2byte = 1;
  }
}

bool eeprom_read_guarded(uint16_t addr, __xdata uint8_t *buf, uint8_t len) __reentrant {
  bool ok;
  if(eeprom_chip == 0)
    return false;
  guard_begin(50);
  ok = eeprom_read(eeprom_chip, addr, (uint8_t *)buf, len, eeprom_2byte);
  if(!ok)
    i2c_stop();
  guard_end();
  return ok;
}

bool eeprom_write_guarded(uint16_t addr, __xdata uint8_t *buf, uint8_t len,
                          uint8_t page_log2) __reentrant {
  bool ok;
  if(eeprom_chip == 0)
    return false;
  if(page_log2 == 0)
    page_log2 = eeprom_2byte ? 5 : 3;   // 32-byte pages (24C32..24C512), 8-byte pages (24C01/02)
  guard_begin(250);
  ok = eeprom_write(eeprom_chip, addr, (uint8_t *)buf, len, eeprom_2byte, page_log2,
                    /*timeout=*/166);
  if(!ok)
    i2c_stop();
  guard_end();
  return ok;
}

// ---------------------------------------------------------------------------
// Board ID
// ---------------------------------------------------------------------------

// Host preset block, written with the 0xA0 request before the CPU starts.
// It lies outside the linker's code and xdata ranges (see build.py).
static __xdata __at(ID_PRESET_ADDR) volatile uint8_t id_preset[ID_RECORD_LEN];

static __xdata uint8_t id_buf[ID_RECORD_LEN];

static bool id_record_valid(__xdata volatile uint8_t *rec, __xdata uint32_t *out) __reentrant {
  uint32_t v;
  if(rec[0] != 'P' || rec[1] != 'C' || rec[2] != 'I' || rec[3] != 'D')
    return false;
  v = ((uint32_t)rec[7] << 24) | ((uint32_t)rec[6] << 16) |
      ((uint32_t)rec[5] << 8) | rec[4];
  if(v == 0 || v == 0xFFFFFFFFUL)
    return false;
  *out = v;
  return true;
}

static __xdata uint32_t id_value;

void board_id_load(void) __reentrant {

  board_id = 0;
  board_id_source = 0;

  if(eeprom_chip != 0 &&
     eeprom_read_guarded(eeprom_2byte ? ID_EEPROM_ADDR_2B : ID_EEPROM_ADDR_1B,
                         id_buf, ID_RECORD_LEN) &&
     id_record_valid(id_buf, &id_value)) {
    board_id = id_value;
    board_id_source = FEAT_ID_EEPROM;
  } else if(id_record_valid(id_preset, &id_value)) {
    board_id = id_value;
    board_id_source = FEAT_ID_PRESET;
  }

  // Consume the preset so that a later upload without one does not reuse it.
  id_preset[0] = 0;

  serial_string_update();
}

void board_init(void) {
  // PA0/PA1 (LED) released; port B and D never driven by the CPU.
  OEA &= ~0x03;
  OEB = 0;
  OED = 0;
  I2CTL = 0;                    // 100 kHz
  timer_init();
  EA = 1;                       // the I2C watchdog below needs the timer
  eeprom_detect();
  board_id_load();
}
