// SPDX-License-Identifier: GPL-3.0-or-later
//
// Slave FIFO configuration, streaming modes and the status block.
//
// Configuration rules followed here (EZ-USB TRM 001-13670 Rev *D):
//  - REVCTL = DYN_OUT | ENH_PKT, required for INPKTEND with SKIP (15.5.10).
//  - Every write to a register of TRM table 15-6 is followed by SYNCDELAY,
//    and so is every write to 0xE6xx that precedes one (15.15). SYNCDELAYLEN
//    is 16 instruction cycles (build flag), enough for IFCLK down to 5 MHz
//    even if the TRM formula is read as instruction cycles.
//  - Mode changes on EP2 use the TRM 9.3.13 sequence: NAKALL, AUTOIN off,
//    reset the FIFO, new mode, release NAKALL.
//  - FIFO registers are always programmed while a clock is certainly present:
//    first under the internal 48 MHz IFCLK with the pins disconnected (ports
//    mode), then repeated under the final clock (the TRM requires an external
//    IFCLK to be running before IFCLKSRC is cleared, 9.2.3), and only then are
//    the pins connected (IFCFG = 11). Stopping switches back to the internal
//    clock before the FIFO is flushed, so a stopped C5 clock cannot leave the
//    FIFO half reset.

#include <fx2regs.h>
#include <fx2delay.h>
#include <fx2usb.h>
#include "openvrx.h"

volatile __data uint32_t cnt_committed;
volatile __data uint32_t cnt_skipped;
volatile __data uint32_t cnt_ffull;
volatile __data uint16_t watch_left;
volatile __data uint16_t watch_reload;
volatile __bit watch_use_npak = 1;
volatile __data uint8_t pattern_low;
volatile __data uint8_t pattern_high;
volatile __xdata uint8_t pattern_warm;
volatile __data uint8_t pattern_groups;

enum {
  MODE_IDLE,
  MODE_AUTO,
  MODE_WATCH,
  MODE_PATTERN,
};

static __xdata uint8_t  mode;
static __xdata uint8_t  cfg_flags;        // STF_* bits describing the running stream
static __xdata uint8_t  cfg_ifconfig;     // IFCONFIG value of the running stream, ports mode
static __xdata uint8_t  cfg_wordwide;
static __xdata uint16_t watch_n;          // current ratio (0 = full stream)
static __xdata uint16_t status_seq;
static __bit    ifclk_active;
static __bit    opt_slwr_hi;
static __bit    opt_pattern_fast;

#define IFCONFIG_IDLE (_IFCLKSRC | _3048MHZ)          // internal 48 MHz, ports
#define IFCFG_SLAVE   (_IFCFG1 | _IFCFG0)

bool usb_high_speed(void) {
  return (USBCS & _HSM) != 0;
}

// NAKALL, manual mode, reset EP2, `fifocfg`, release NAKALL.
static void fifo_reset_ep2(uint8_t fifocfg) {
  FIFORESET = _NAKALL;           SYNCDELAY;
  EP2FIFOCFG = cfg_wordwide;     SYNCDELAY;   // AUTOIN off before the reset
  FIFORESET = _NAKALL | 2;       SYNCDELAY;
  EP2FIFOCFG = fifocfg;          SYNCDELAY;
  FIFORESET = 0;                 SYNCDELAY;
}

void stream_init(void) {
  mode = MODE_IDLE;
  cfg_flags = 0;
  cfg_wordwide = 0;
  watch_n = 0;

  IFCONFIG = IFCONFIG_IDLE;      SYNCDELAY;
  REVCTL = _DYN_OUT | _ENH_PKT;  SYNCDELAY;

  EP1OUTCFG = 0;                 SYNCDELAY;   // not used
  EP1INCFG = _VALID | 0x30;      SYNCDELAY;   // interrupt IN
  EP2CFG = 0xE0;                 SYNCDELAY;   // valid, IN, bulk, 512, quad
  EP4CFG = 0;                    SYNCDELAY;   // EP4 memory belongs to EP2 (quad)
  EP6CFG = 0;                    SYNCDELAY;
  EP8CFG = 0;                    SYNCDELAY;

  // Reset defaults set WORDWIDE on EP4/6/8; clear so only EP2 decides.
  EP4FIFOCFG = 0;                SYNCDELAY;
  EP6FIFOCFG = 0;                SYNCDELAY;
  EP8FIFOCFG = 0;                SYNCDELAY;

  PINFLAGSAB = 0;                SYNCDELAY;
  PINFLAGSCD = 0;                SYNCDELAY;
  PORTACFG = 0;                  SYNCDELAY;   // PA7 is a port pin, not FLAGD/SLCS#
  FIFOPINPOLAR = 0;              SYNCDELAY;

  EP2AUTOINLENH = 0x02;          SYNCDELAY;
  EP2AUTOINLENL = 0x00;          SYNCDELAY;

  FIFORESET = _NAKALL;           SYNCDELAY;
  FIFORESET = _NAKALL | 2;       SYNCDELAY;
  FIFORESET = _NAKALL | 4;       SYNCDELAY;
  FIFORESET = _NAKALL | 6;       SYNCDELAY;
  FIFORESET = _NAKALL | 8;       SYNCDELAY;
  fifo_reset_ep2(0);
}

void stream_stop(void) {
  mode = MODE_IDLE;
  cfg_flags = 0;
  IFCONFIG = IFCONFIG_IDLE;      SYNCDELAY;
  cfg_wordwide = 0;
  fifo_reset_ep2(0);
}

bool stream_start(uint16_t flags, uint16_t ratio) __reentrant {
  bool hs = usb_high_speed();
  uint16_t pkt = hs ? 512 : 64;
  bool pattern = (flags & START_TEST_PATTERN) != 0;
  bool internal = pattern || (flags & START_INTERNAL_CLK) != 0;
  bool watch = !pattern && (flags & START_WATCH) && ratio > 1;
  uint8_t ifc;
  uint8_t fifocfg;

  if(usb_config_value == 0)
    return false;

  stream_stop();

  ifc = internal ? (_IFCLKSRC | ((flags & START_INTERNAL_48) ? _3048MHZ : 0)) : 0;
  if(flags & START_IFCLK_INV)
    ifc |= _IFCLKPOL;
  cfg_wordwide = (flags & START_WIDTH16) ? _WORDWIDE : 0;
  fifocfg = cfg_wordwide | ((watch || pattern) ? 0 : _AUTOIN);

  // Phase A: internal 48 MHz, pins disconnected, NAK the host.
  FIFORESET = _NAKALL;                          SYNCDELAY;
  FIFOPINPOLAR = opt_slwr_hi ? _SLWR : 0;       SYNCDELAY;
  EP2FIFOCFG = cfg_wordwide;                    SYNCDELAY;
  FIFORESET = _NAKALL | 2;                      SYNCDELAY;
  EP2AUTOINLENH = (uint8_t)(pkt >> 8);          SYNCDELAY;
  EP2AUTOINLENL = (uint8_t)pkt;                 SYNCDELAY;
  EP2FIFOCFG = fifocfg;                         SYNCDELAY;

  // Phase B: final clock source and polarity, still in ports mode.
  IFCONFIG = ifc;                               SYNCDELAY;
  delay_us(20);

  // Phase C: repeat the reset under the final clock, release NAKALL.
  fifo_reset_ep2(fifocfg);

  // Stream state.
  watch_n = watch ? ratio : 0;
  watch_reload = ratio - 1;
  watch_left = 0;
  pattern_low = 0;
  pattern_high = 0;
  pattern_warm = 0;
  pattern_groups = hs ? 32 : 4;
  cfg_ifconfig = ifc;

  cfg_flags = STF_RUNNING;
  if(cfg_wordwide)
    cfg_flags |= STF_WIDTH16;
  if(flags & START_IFCLK_INV)
    cfg_flags |= STF_IFCLK_INV;
  if(internal)
    cfg_flags |= STF_INTERNAL_CLK;
  if(watch)
    cfg_flags |= STF_WATCH;
  if(pattern)
    cfg_flags |= STF_TEST_PATTERN;

  // Phase D: connect the pins (the test pattern stays in ports mode: the CPU
  // is the only source of EP2 data).
  if(pattern) {
    mode = MODE_PATTERN;
  } else {
    IFCONFIG = ifc | IFCFG_SLAVE;               SYNCDELAY;
    mode = watch ? MODE_WATCH : MODE_AUTO;
  }
  return true;
}

void stream_set_watch(uint16_t ratio) __reentrant {
  bool watch = ratio > 1;

  if(mode != MODE_AUTO && mode != MODE_WATCH) {
    // Not streaming from the pins: remembered only for the status block.
    watch_n = watch ? ratio : 0;
    return;
  }

  // Live switch between AUTOIN and manual commit. Disconnect the pins while
  // the FIFO is reset so that no partial packet is left behind.
  IFCONFIG = cfg_ifconfig;                      SYNCDELAY;
  watch_reload = ratio - 1;
  watch_left = 0;
  fifo_reset_ep2(cfg_wordwide | (watch ? 0 : _AUTOIN));
  watch_n = watch ? ratio : 0;
  if(watch)
    cfg_flags |= STF_WATCH;
  else
    cfg_flags &= ~STF_WATCH;
  mode = watch ? MODE_WATCH : MODE_AUTO;
  IFCONFIG = cfg_ifconfig | IFCFG_SLAVE;        SYNCDELAY;
}

// ---------------------------------------------------------------------------
// Loops that are not on the high-speed hot path
// ---------------------------------------------------------------------------

// Watch mode at full speed (64-byte packets, at most about 1 MB/s on the bus).
static void watch_loop_fs(void) __reentrant {
  uint8_t npak;
  uint8_t full;
  uint16_t bc;

  while(!svc) {
    if(EP24FIFOFLGS & _EP2FF)
      cnt_ffull++;
    // Committed packets first, then the byte count: a packet sent in between
    // can only make the estimate smaller (see loop_watch_hs).
    npak = watch_use_npak ? ((EP2CS >> 4) & 0x07) : 0;
    bc = ((uint16_t)EP2FIFOBCH << 8) | EP2FIFOBCL;
    full = (uint8_t)(bc >> 6);
    if(full <= npak)
      continue;
    if(watch_left != 0) {
      watch_left--;
      INPKTEND = _SKIP | 2;
      cnt_skipped++;
    } else {
      watch_left = watch_reload;
      INPKTEND = 2;
      cnt_committed++;
    }
    SYNCDELAY;
  }
}

// CPU-sourced test pattern: consecutive 16-bit little-endian words.
static void pattern_loop(void) __reentrant {
  bool hs = usb_high_speed();

  while(!svc) {
    if(EP2468STAT & _EP2F)
      continue;                 // all four buffers are with the USB engine
    if(hs && opt_pattern_fast && pattern_warm >= 4) {
      pattern_fill_high();
    } else {
      pattern_fill_full();
      if(pattern_warm < 4)
        pattern_warm++;
    }
    SYNCDELAY;                  // XAUTODAT2 writes precede EP2BCH (TRM 15.15)
    if(hs) {
      EP2BCH = 0x02;            SYNCDELAY;
      EP2BCL = 0x00;
    } else {
      EP2BCH = 0x00;            SYNCDELAY;
      EP2BCL = 64;
    }
    SYNCDELAY;
    cnt_committed++;
  }
}

void stream_run(void) {
  switch(mode) {
  case MODE_AUTO:
    loop_auto();
    break;
  case MODE_WATCH:
    if(usb_high_speed())
      loop_watch_hs();
    else
      watch_loop_fs();
    break;
  case MODE_PATTERN:
    pattern_loop();
    break;
  default:
    while(!svc)
      ;
    break;
  }
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

static void put16(__xdata uint8_t *p, uint16_t v) __reentrant {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void put32(__xdata uint8_t *p, uint32_t v) __reentrant {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

void status_fill(__xdata uint8_t *buf) __reentrant {
  uint8_t flags = cfg_flags;
  if(ifclk_active && mode != MODE_IDLE)
    flags |= STF_IFCLK_ACTIVE;

  buf[0] = 'P';
  buf[1] = 'C';
  buf[2] = 'V';
  buf[3] = 'S';
  buf[4] = PROTO_VERSION;
  buf[5] = flags;
  put16(buf + 6, watch_n);
  put32(buf + 8, cnt_committed);
  put32(buf + 12, cnt_skipped);
  put32(buf + 16, cnt_ffull);
  put32(buf + 20, uptime_read());
  put16(buf + 24, status_seq);
  put16(buf + 26, (FW_MAJOR << 8) | FW_MINOR);
  put32(buf + 28, 0);
}

void stream_tick(void) {
  ET2 = 0;
  ifclk_active = fifo_moved;
  fifo_moved = 0;
  ET2 = 1;

  if(usb_config_value != 0 && !(EP1INCS & _BUSY)) {
    status_seq++;
    status_fill(EP1INBUF);
    EP1INBC = 32;
  }
}

void stream_reset_counters(void) {
  cnt_committed = 0;
  cnt_skipped = 0;
  cnt_ffull = 0;
}

bool stream_set_option(uint16_t key, uint16_t value) __reentrant {
  switch(key) {
  case OPT_WATCH_DETECT:
    if(value > 1)
      return false;
    watch_use_npak = (value == 0);
    return true;
  case OPT_SLWR_ACTIVE_HI:
    if(value > 1)
      return false;
    opt_slwr_hi = (value != 0);
    return true;
  case OPT_STATUS_INTERVAL:
    if(value < 1 || value > 255)
      return false;
    status_interval = (uint8_t)value;
    return true;
  case OPT_PATTERN_FILL:
    if(value > 1)
      return false;
    opt_pattern_fast = (value != 0);
    return true;
  case OPT_IFCLK_OUT:
    // Finds the IFCLK header pin: the FX2 drives its internal clock out of
    // IFCLK and a jumper from that pin to a port pin shows it toggling in
    // GET_PINS. Never while streaming; the next START or STOP turns it off.
    if(value > 1 || mode != MODE_IDLE)
      return false;
    IFCONFIG = IFCONFIG_IDLE | (value ? _IFCLKOE : 0);  SYNCDELAY;
    return true;
  default:
    return false;
  }
}
