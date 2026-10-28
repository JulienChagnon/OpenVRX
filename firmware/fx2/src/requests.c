// SPDX-License-Identifier: GPL-3.0-or-later
//
// OpenVRX vendor requests (docs/PROTOCOL.md section 2), handled in the main
// loop after the SETUP interrupt flagged them (svc_setup).

#include <fx2regs.h>
#include <fx2usb.h>
#include "openvrx.h"

#define VENDOR_IN   (USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN)
#define VENDOR_OUT  (USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT)

// Longest time a data stage may stay idle before the request is abandoned.
#define EP0_TIMEOUT_MS 1000

// Waits for EP0 to finish its current packet. Gives up (false) when a new
// SETUP packet arrived (the host abandoned this transfer) or on timeout.
static bool ep0_wait(void) __reentrant {
  uint32_t t0 = uptime_read();
  while(EP0CS & _BUSY) {
    if(svc_setup)
      return false;
    if(uptime_read() - t0 > EP0_TIMEOUT_MS)
      return false;
  }
  return true;
}

// Sends `len` bytes already placed in EP0BUF, clamped to wLength.
static void ep0_in_buf(uint8_t len, uint16_t wlength) __reentrant {
  if(wlength < len)
    len = (uint8_t)wlength;
  if(len == 0) {
    ACK_EP0();
    return;
  }
  SETUP_EP0_IN_BUF(len);
}

static void put32(__xdata uint8_t *p, uint32_t v) __reentrant {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static void get_info(uint16_t wlength) __reentrant {
  uint32_t features = FEAT_WATCH | FEAT_INTERNAL_CLK | FEAT_TEST_PATTERN | FEAT_SET_OPTION |
                      board_id_source;
  if(eeprom_chip != 0)
    features |= FEAT_EEPROM;
  if(eeprom_2byte)
    features |= FEAT_EEPROM_2BYTE;
  if(usb_high_speed())
    features |= FEAT_HIGH_SPEED;

  EP0BUF[0] = 'P';
  EP0BUF[1] = 'C';
  EP0BUF[2] = 'V';
  EP0BUF[3] = 'X';
  EP0BUF[4] = PROTO_VERSION;
  EP0BUF[5] = FW_MAJOR;
  EP0BUF[6] = FW_MINOR;
  EP0BUF[7] = FW_PATCH;
  put32(EP0BUF + 8, features);
  put32(EP0BUF + 12, board_id);
  ep0_in_buf(16, wlength);
}

// Wiring check: live pin levels and FIFO state (docs/PROTOCOL.md, 0xBD).
// Ports are sampled 256 times; the OR and AND of the samples show which
// pins toggle (data lanes carrying noise) and which sit high or low. Only
// meaningful while stopped: in slave FIFO mode PB/PD belong to the FIFO.
static void get_pins(uint16_t wlength) __reentrant {
  uint8_t a_or = 0, a_and = 0xFF, b_or = 0, b_and = 0xFF, d_or = 0, d_and = 0xFF;
  uint8_t r_or = 0, r_and = 0xFF;
  uint8_t v;
  uint16_t k;

  for(k = 0; k < 256; k++) {
    v = IOA; a_or |= v; a_and &= v;
    v = IOB; b_or |= v; b_and &= v;
    v = IOD; d_or |= v; d_and &= v;
    v = GPIFREADYSTAT; r_or |= v; r_and &= v;
  }
  EP0BUF[0] = a_or;  EP0BUF[1] = a_and;
  EP0BUF[2] = b_or;  EP0BUF[3] = b_and;
  EP0BUF[4] = d_or;  EP0BUF[5] = d_and;
  EP0BUF[6] = r_or;  EP0BUF[7] = r_and;
  EP0BUF[8] = IFCONFIG;
  EP0BUF[9] = EP2FIFOCFG;
  EP0BUF[10] = EP24FIFOFLGS;
  EP0BUF[11] = EP2FIFOBCH;
  EP0BUF[12] = EP2FIFOBCL;
  EP0BUF[13] = EP2CS;
  EP0BUF[14] = FIFOPINPOLAR;
  EP0BUF[15] = OEA;
  ep0_in_buf(16, wlength);
}

static void eeprom_in(uint16_t addr, uint16_t len) __reentrant {
  uint8_t n;

  if(len == 0) {
    ACK_EP0();
    return;
  }
  if(eeprom_chip == 0) {
    STALL_EP0();
    return;
  }
  while(len != 0) {
    n = len > 64 ? 64 : (uint8_t)len;
    if(!ep0_wait())
      return;
    if(!eeprom_read_guarded(addr, EP0BUF, n)) {
      STALL_EP0();
      return;
    }
    SUDPTRCTL = _SDPAUTO;
    EP0BCH = 0;
    EP0BCL = n;
    addr += n;
    len -= n;
  }
  ACK_EP0();
}

static void eeprom_out(uint16_t addr, uint16_t len, uint8_t page_log2) __reentrant {
  uint8_t n;

  if(len == 0) {
    ACK_EP0();
    return;
  }
  if(eeprom_chip == 0 || page_log2 > 7) {
    STALL_EP0();
    return;
  }
  while(len != 0) {
    SETUP_EP0_OUT_BUF();        // arm EP0 for the next OUT packet
    if(!ep0_wait())
      return;
    n = EP0BCL;
    if(n == 0 || n > len) {
      STALL_EP0();
      return;
    }
    if(!eeprom_write_guarded(addr, EP0BUF, n, page_log2)) {
      STALL_EP0();
      return;
    }
    addr += n;
    len -= n;
  }
  ACK_EP0();
}

void requests_service(void) __reentrant {
  __xdata struct usb_req_setup *req = (__xdata struct usb_req_setup *)SETUPDAT;
  uint8_t  type    = req->bmRequestType;
  uint8_t  request = req->bRequest;
  uint16_t value   = req->wValue;
  uint16_t index   = req->wIndex;
  uint16_t length  = req->wLength;
  bool     ok      = true;

  // The SETUP packet is copied: a new one may now be accepted.
  svc_setup = 0;

  if(type == VENDOR_IN) {
    switch(request) {
    case REQ_GET_INFO:
      get_info(length);
      return;
    case REQ_GET_STATUS:
      status_fill(EP0BUF);
      ep0_in_buf(32, length);
      return;
    case REQ_EEPROM_READ:
      eeprom_in(value, length);
      return;
    case REQ_GET_PINS:
      get_pins(length);
      return;
    default:
      break;
    }
  } else if(type == VENDOR_OUT) {
    if(request == REQ_EEPROM_WRITE) {
      eeprom_out(value, length, (uint8_t)index);
      return;
    }
    if(length != 0) {
      STALL_EP0();
      return;
    }
    switch(request) {
    case REQ_START:
      ok = stream_start(value, index);
      break;
    case REQ_STOP:
      stream_stop();
      break;
    case REQ_RESET_COUNTERS:
      stream_reset_counters();
      break;
    case REQ_SET_WATCH:
      stream_set_watch(value);
      break;
    case REQ_LED:
      led_set(value != 0);
      break;
    case REQ_SET_OPTION:
      ok = stream_set_option(value, index);
      break;
    case REQ_SET_BOARD_ID:
      board_id = ((uint32_t)index << 16) | value;
      board_id_source = FEAT_ID_HOST;
      serial_string_update();
      break;
    default:
      ok = false;
      break;
    }
    if(ok)
      ACK_EP0();
    else
      STALL_EP0();
    return;
  }

  STALL_EP0();
}
