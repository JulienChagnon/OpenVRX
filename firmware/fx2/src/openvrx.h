// SPDX-License-Identifier: GPL-3.0-or-later
//
// OpenVRX FX2 streaming firmware: shared definitions.
// The wire protocol is specified in docs/PROTOCOL.md, section 2.

#ifndef OPENVRX_H
#define OPENVRX_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Versions and identity
// ---------------------------------------------------------------------------

#define FW_MAJOR        0
#define FW_MINOR        1
#define FW_PATCH        0
#define PROTO_VERSION   1

#define USB_VID         0x1209
#define USB_PID         0x0001
// bcdDevice: major.minor.patch packed as BCD digits 0x0MmP.
#define USB_BCD_DEVICE  ((FW_MAJOR << 8) | (FW_MINOR << 4) | FW_PATCH)

// Vendor code shared by the MS OS 1.0 and MS OS 2.0 descriptor requests.
#define MS_VENDOR_CODE  0xEE

// ---------------------------------------------------------------------------
// Vendor requests (bmRequestType 0x40 OUT / 0xC0 IN, device recipient)
// ---------------------------------------------------------------------------

#define REQ_GET_INFO        0xB0
#define REQ_START           0xB1
#define REQ_STOP            0xB2
#define REQ_GET_STATUS      0xB3
#define REQ_RESET_COUNTERS  0xB4
#define REQ_SET_WATCH       0xB5
#define REQ_LED             0xB6
#define REQ_SET_OPTION      0xB7
#define REQ_GET_PINS        0xBD
#define REQ_EEPROM_READ     0xBA
#define REQ_EEPROM_WRITE    0xBB
#define REQ_SET_BOARD_ID    0xBC

// START flags (wValue)
#define START_WIDTH16       0x01
#define START_IFCLK_INV     0x02
#define START_INTERNAL_CLK  0x04
#define START_INTERNAL_48   0x08
#define START_WATCH         0x10
#define START_TEST_PATTERN  0x20

// Status block flags (byte 5)
#define STF_RUNNING         0x01
#define STF_WIDTH16         0x02
#define STF_IFCLK_INV       0x04
#define STF_INTERNAL_CLK    0x08
#define STF_WATCH           0x10
#define STF_IFCLK_ACTIVE    0x20
#define STF_TEST_PATTERN    0x40

// SET_OPTION keys (wValue = key, wIndex = value)
#define OPT_WATCH_DETECT    0x01  // 0 = FIFO byte count minus committed packets, 1 = byte count only
#define OPT_SLWR_ACTIVE_HI  0x02  // 0 = SLWR active low (default), 1 = active high
#define OPT_STATUS_INTERVAL 0x03  // EP1 status period in ms, 1..255 (default 8)
#define OPT_PATTERN_FILL    0x04  // 0 = write whole packet (default), 1 = rewrite high bytes only
#define OPT_IFCLK_OUT       0x05  // wiring check, stopped only: 1 = drive 48 MHz out of IFCLK, 0 = input

// GET_INFO feature bits
#define FEAT_WATCH          0x00000001UL
#define FEAT_INTERNAL_CLK   0x00000002UL
#define FEAT_TEST_PATTERN   0x00000004UL
#define FEAT_SET_OPTION     0x00000008UL
#define FEAT_EEPROM         0x00000010UL  // an I2C EEPROM answered at 0x50 or 0x51
#define FEAT_EEPROM_2BYTE   0x00000020UL  // ... and it uses two-byte addressing (0x51)
#define FEAT_ID_EEPROM      0x00000040UL  // board ID came from the EEPROM ID record
#define FEAT_ID_PRESET      0x00000080UL  // board ID came from the RAM preset block
#define FEAT_ID_HOST        0x00000100UL  // board ID was set by SET_BOARD_ID
#define FEAT_HIGH_SPEED     0x00000200UL  // currently connected at high speed

// ---------------------------------------------------------------------------
// Board ID storage
// ---------------------------------------------------------------------------

// 8-byte ID record: 'P','C','I','D', u32 board_id little endian.
#define ID_RECORD_LEN       8
#define ID_EEPROM_ADDR_1B   0x00F8  // one-byte-address EEPROM at 0x50
#define ID_EEPROM_ADDR_2B   0x3FF8  // two-byte-address EEPROM at 0x51
// RAM preset written by the host with the 0xA0 request before releasing CPUCS.
#define ID_PRESET_ADDR      0x3FF0

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

// Service request: set by interrupts when the main loop has work to do.
// Every hot loop polls this single bit and returns when it is set.
extern volatile __bit svc;
extern volatile __bit svc_setup;   // a non-standard SETUP packet is waiting
extern volatile __bit svc_tick;    // status interval elapsed
extern volatile __bit svc_bus;     // bus reset, speed change or configuration change

// Counters (little endian in memory, copied verbatim into the status block).
extern volatile __data uint32_t cnt_committed;
extern volatile __data uint32_t cnt_skipped;
extern volatile __data uint32_t cnt_ffull;

// Watch-mode down counter and reload (packets to skip before the next commit).
extern volatile __data uint16_t watch_left;
extern volatile __data uint16_t watch_reload;
extern volatile __bit watch_use_npak;

// Test pattern state: high byte of the 16-bit word counter and fill mode.
extern volatile __data uint8_t pattern_low;
extern volatile __data uint8_t pattern_high;
extern volatile __xdata uint8_t pattern_warm;
extern volatile __data uint8_t pattern_groups;  // 8-word groups per packet: 32 (HS) or 4 (FS)

// Board identity
extern __xdata uint32_t board_id;
extern __xdata uint32_t board_id_source;   // FEAT_ID_* bit, 0 if no ID known

// EEPROM
extern __xdata uint8_t eeprom_chip;   // 0x50, 0x51 or 0 when absent
extern __xdata uint8_t eeprom_2byte;  // nonzero: two-byte addressing

// Timer and status
extern volatile __data uint32_t uptime_ms;
extern volatile __bit fifo_moved;
extern volatile __bit led_blink;
extern volatile __data uint8_t status_interval;
extern volatile __data uint8_t i2c_guard_ms;

// ---------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------

// board.c
void board_init(void);
void timer_init(void);
uint32_t uptime_read(void) __reentrant;
void led_set(bool on) __reentrant;
void eeprom_detect(void);
void board_id_load(void) __reentrant;
bool eeprom_read_guarded(uint16_t addr, __xdata uint8_t *buf, uint8_t len) __reentrant;
bool eeprom_write_guarded(uint16_t addr, __xdata uint8_t *buf, uint8_t len,
                          uint8_t page_log2) __reentrant;

// stream.c
void stream_init(void);
bool stream_start(uint16_t flags, uint16_t watch_n) __reentrant;
void stream_stop(void);
void stream_set_watch(uint16_t watch_n) __reentrant;
void stream_run(void);
void stream_tick(void);
void stream_reset_counters(void);
void status_fill(__xdata uint8_t *buf) __reentrant;
bool stream_set_option(uint16_t key, uint16_t value) __reentrant;
bool usb_high_speed(void);

// requests.c
void requests_service(void) __reentrant;

// usb_desc.c (main-loop context; rebuilds the serial number string)
void serial_string_update(void) __reentrant;

// hotloops.asm
void loop_auto(void);
void loop_watch_hs(void);
void pattern_fill_full(void);
void pattern_fill_high(void);

#endif
