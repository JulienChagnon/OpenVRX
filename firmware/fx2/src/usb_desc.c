// SPDX-License-Identifier: GPL-3.0-or-later
//
// USB descriptors and the parts of control-request handling that run inside
// the SETUP (SUDAV) interrupt: GET_DESCRIPTOR, the Microsoft OS descriptor
// vendor requests, SET_CONFIGURATION / SET_INTERFACE, bus reset and
// high-speed detection. Everything else is deferred to the main loop
// (requests.c) through the svc_setup flag.
//
// All descriptor tables are kept in code memory as plain byte layouts. Their
// sizes are exported in openvrx_desc_sizes[] so that tests/test_descriptors.py
// can extract and check every byte of the compiled image.

// Functions in this file run in interrupt context; never overlay their locals
// with those of main-loop functions.
#pragma nooverlay

#include <stddef.h>
#include <fx2regs.h>
#include <fx2ints.h>
#include <fx2usb.h>
#include "openvrx.h"

#define W(x)   ((uint8_t)((x) & 0xFF)), ((uint8_t)(((x) >> 8) & 0xFF))
#define DW(x)  W((x) & 0xFFFF), W(((uint32_t)(x) >> 16) & 0xFFFF)

#define CONFIG_TOTAL_LENGTH (9 + 9 + 7 + 7)

// ---------------------------------------------------------------------------
// Device, qualifier, configurations
// ---------------------------------------------------------------------------

__code const uint8_t openvrx_desc_device[18] = {
  18,                     // bLength
  0x01,                   // bDescriptorType: DEVICE
  W(0x0201),              // bcdUSB 2.01: makes Windows ask for the BOS
  0xFF, 0xFF, 0xFF,       // vendor-specific class / subclass / protocol
  64,                     // bMaxPacketSize0
  W(USB_VID),
  W(USB_PID),
  W(USB_BCD_DEVICE),
  1,                      // iManufacturer
  2,                      // iProduct
  3,                      // iSerialNumber
  1,                      // bNumConfigurations
};

__code const uint8_t openvrx_desc_qualifier[10] = {
  10,                     // bLength
  0x06,                   // bDescriptorType: DEVICE_QUALIFIER
  W(0x0200),              // bcdUSB
  0xFF, 0xFF, 0xFF,
  64,                     // bMaxPacketSize0 at the other speed
  1,                      // bNumConfigurations
  0,                      // bReserved
};

// High speed: EP2 bulk 512, EP1 interrupt every 2^(7-1) microframes = 8 ms.
__code const uint8_t openvrx_desc_config_hs[CONFIG_TOTAL_LENGTH] = {
  9, 0x02, W(CONFIG_TOTAL_LENGTH), 1, 1, 0, 0x80, 125,    // bus powered, 250 mA
  9, 0x04, 0, 0, 2, 0xFF, 0xFF, 0xFF, 0,                  // interface 0, alt 0, 2 EPs
  7, 0x05, 0x82, 0x02, W(512), 0,                         // EP2 IN bulk 512
  7, 0x05, 0x81, 0x03, W(64), 7,                          // EP1 IN interrupt 64, 8 ms
};

// Full speed: EP2 bulk 64, EP1 interrupt every 8 frames = 8 ms.
__code const uint8_t openvrx_desc_config_fs[CONFIG_TOTAL_LENGTH] = {
  9, 0x02, W(CONFIG_TOTAL_LENGTH), 1, 1, 0, 0x80, 125,
  9, 0x04, 0, 0, 2, 0xFF, 0xFF, 0xFF, 0,
  7, 0x05, 0x82, 0x02, W(64), 0,
  7, 0x05, 0x81, 0x03, W(64), 8,
};

// ---------------------------------------------------------------------------
// Strings (UTF-16LE, built by the compiler from u"" literals)
// ---------------------------------------------------------------------------

__code const uint8_t openvrx_desc_lang[4] = { 4, 0x03, W(0x0409) };

// ASCII characters listed one by one are already UTF-16LE code units.
#define STRING_DESC(name, nchars, ...) \
  __code const struct { uint8_t bLength; uint8_t bDescriptorType; uint16_t s[nchars]; } \
  name = { 2 + 2 * (nchars), 0x03, { __VA_ARGS__ } }

STRING_DESC(openvrx_desc_manufacturer, 7, 'O', 'p', 'e', 'n', 'V', 'R', 'X');
STRING_DESC(openvrx_desc_product, 11, 'O', 'p', 'e', 'n', 'V', 'R', 'X', ' ', 'F', 'X', '2');

// Serial number template: eight hex digits, patched from the board ID.
STRING_DESC(openvrx_desc_serial_template, 8, '0', '0', '0', '0', '0', '0', '0', '0');

// MS OS 1.0 string descriptor (string index 0xEE).
__code const struct {
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t qwSignature[7];
  uint8_t  bMS_VendorCode;
  uint8_t  bPad;
} openvrx_desc_msos10_string = {
  18, 0x03, { 'M', 'S', 'F', 'T', '1', '0', '0' }, MS_VENDOR_CODE, 0
};

// ---------------------------------------------------------------------------
// Microsoft OS 2.0 descriptor set (non-composite device: header, compatible
// ID, registry property; no configuration or function subsets)
// ---------------------------------------------------------------------------

struct ms_os20_set {
  // MS_OS_20_SET_HEADER_DESCRIPTOR
  uint16_t wLength;
  uint16_t wDescriptorType;
  uint32_t dwWindowsVersion;
  uint16_t wTotalLength;
  // MS_OS_20_FEATURE_COMPATBLE_ID
  uint16_t cid_wLength;
  uint16_t cid_wDescriptorType;
  uint8_t  CompatibleID[8];
  uint8_t  SubCompatibleID[8];
  // MS_OS_20_FEATURE_REG_PROPERTY
  uint16_t reg_wLength;
  uint16_t reg_wDescriptorType;
  uint16_t wPropertyDataType;
  uint16_t wPropertyNameLength;
  uint16_t PropertyName[21];          // "DeviceInterfaceGUIDs" + NUL
  uint16_t wPropertyDataLength;
  uint16_t PropertyData[40];          // "{GUID}" + NUL + NUL (REG_MULTI_SZ)
};

__code const struct ms_os20_set openvrx_desc_msos20_set = {
  10, 0x0000, 0x06030000UL, sizeof(struct ms_os20_set),
  20, 0x0003, { 'W', 'I', 'N', 'U', 'S', 'B', 0, 0 }, { 0 },
  sizeof(struct ms_os20_set) - offsetof(struct ms_os20_set, reg_wLength),
  0x0004,
  7,                                   // REG_MULTI_SZ
  sizeof(((struct ms_os20_set *)0)->PropertyName),
  u"DeviceInterfaceGUIDs",
  sizeof(((struct ms_os20_set *)0)->PropertyData),
  u"{5C2B2B1E-7D5A-4C1B-9E0B-50435652580A}\0",
};

// ---------------------------------------------------------------------------
// BOS: USB 2.0 extension (no LPM) + MS OS 2.0 platform capability
// ---------------------------------------------------------------------------

#define BOS_TOTAL_LENGTH (5 + 7 + 28)

__code const uint8_t openvrx_desc_bos[BOS_TOTAL_LENGTH] = {
  5, 0x0F, W(BOS_TOTAL_LENGTH), 2,
  // USB 2.0 Extension: bmAttributes 0 (LPM not supported)
  7, 0x10, 0x02, DW(0),
  // Platform capability, MS OS 2.0 descriptor set information
  28, 0x10, 0x05, 0x00,
  0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,   // {D8DD60DF-4589-4CC7-
  0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,   //  9CD2-659D9E648A9F}
  DW(0x06030000UL),                                  // Windows 8.1 and later
  W(sizeof(struct ms_os20_set)),
  MS_VENDOR_CODE,
  0,                                                 // bAltEnumCode
};

// ---------------------------------------------------------------------------
// MS OS 1.0 fallback: extended compat ID and extended properties
// ---------------------------------------------------------------------------

__code const uint8_t openvrx_desc_msos10_compat[40] = {
  DW(40), W(0x0100), W(0x0004), 1, 0, 0, 0, 0, 0, 0, 0,
  0,                                                 // bFirstInterfaceNumber
  1,                                                 // reserved, must be 1
  'W', 'I', 'N', 'U', 'S', 'B', 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
};

struct ms_os10_props {
  uint32_t dwLength;
  uint16_t bcdVersion;
  uint16_t wIndex;
  uint16_t wCount;
  uint32_t dwSize;
  uint32_t dwPropertyDataType;
  uint16_t wPropertyNameLength;
  uint16_t bPropertyName[21];
  uint32_t dwPropertyDataLength;
  uint16_t bPropertyData[40];
};

__code const struct ms_os10_props openvrx_desc_msos10_props = {
  sizeof(struct ms_os10_props), 0x0100, 0x0005, 1,
  sizeof(struct ms_os10_props) - offsetof(struct ms_os10_props, dwSize),
  7,
  sizeof(((struct ms_os10_props *)0)->bPropertyName),
  u"DeviceInterfaceGUIDs",
  sizeof(((struct ms_os10_props *)0)->bPropertyData),
  u"{5C2B2B1E-7D5A-4C1B-9E0B-50435652580A}\0",
};

// Sizes in a fixed order, read by tests/test_descriptors.py.
__code const uint16_t openvrx_desc_sizes[13] = {
  sizeof(openvrx_desc_device),
  sizeof(openvrx_desc_qualifier),
  sizeof(openvrx_desc_config_hs),
  sizeof(openvrx_desc_config_fs),
  sizeof(openvrx_desc_lang),
  sizeof(openvrx_desc_manufacturer),
  sizeof(openvrx_desc_product),
  sizeof(openvrx_desc_serial_template),
  sizeof(openvrx_desc_msos10_string),
  sizeof(openvrx_desc_bos),
  sizeof(openvrx_desc_msos20_set),
  sizeof(openvrx_desc_msos10_compat),
  sizeof(openvrx_desc_msos10_props),
};

// ---------------------------------------------------------------------------
// Runtime serial number string
// ---------------------------------------------------------------------------

// Written by the main loop (serial_string_update), read by the SETUP interrupt.
static __xdata uint8_t serial_desc[sizeof(openvrx_desc_serial_template)];

static __code const char hex_digits[] = "0123456789ABCDEF";

void serial_string_update(void) __reentrant {
  uint32_t id = board_id;
  uint8_t i;
  uint8_t digits[8];

  for(i = 0; i < 8; i++) {
    digits[7 - i] = hex_digits[(uint8_t)id & 0x0F];
    id >>= 4;
  }

  EA = 0;
  serial_desc[0] = sizeof(serial_desc);
  serial_desc[1] = 0x03;
  for(i = 0; i < 8; i++) {
    serial_desc[2 + 2 * i] = digits[i];
    serial_desc[3 + 2 * i] = 0;
  }
  EA = 1;
}

// ---------------------------------------------------------------------------
// EP0 IN through the setup data pointer
// ---------------------------------------------------------------------------

// Word-aligned staging buffer in the 512-byte scratch RAM (0xE000-0xE0FF).
// SUDPTR can only source word-aligned on-chip RAM, and the code-space tables
// have no alignment guarantee, so descriptors are copied here first.
static __xdata __at(0xE000) uint8_t ep0_stage[256];

static uint16_t setup_wlength(void) __reentrant {
  return ((uint16_t)SETUPDAT[7] << 8) | SETUPDAT[6];
}

// Arms the data stage with `len` bytes from ep0_stage, clamped to wLength.
static void ep0_send_stage(uint16_t len) __reentrant {
  uint16_t wlength = setup_wlength();
  if(len > wlength)
    len = wlength;
  if(len == 0) {
    ACK_EP0();
    return;
  }
  SUDPTRCTL = 0;                // manual mode: length from EP0BCH:L
  EP0BCH = (uint8_t)(len >> 8);
  EP0BCL = (uint8_t)len;
  SUDPTRH = (uint8_t)((uint16_t)ep0_stage >> 8);
  SUDPTRL = (uint8_t)((uint16_t)ep0_stage);   // starts the transfer
  ACK_EP0();
}

static void ep0_send_code(__code const uint8_t *src, uint16_t len) __reentrant {
  uint16_t i;
  for(i = 0; i < len; i++)
    ep0_stage[i] = src[i];
  ep0_send_stage(len);
}

void handle_usb_get_descriptor(enum usb_descriptor type, uint8_t index) {
  bool hs = (USBCS & _HSM) != 0;
  uint8_t i;

  switch((uint8_t)type) {
  case USB_DESC_DEVICE:
    if(index != 0)
      break;
    ep0_send_code(openvrx_desc_device, sizeof(openvrx_desc_device));
    return;

  case USB_DESC_DEVICE_QUALIFIER:
    if(index != 0)
      break;
    ep0_send_code(openvrx_desc_qualifier, sizeof(openvrx_desc_qualifier));
    return;

  case USB_DESC_CONFIGURATION:
    if(index != 0)
      break;
    ep0_send_code(hs ? openvrx_desc_config_hs : openvrx_desc_config_fs, CONFIG_TOTAL_LENGTH);
    return;

  case USB_DESC_OTHER_SPEED_CONFIGURATION:
    if(index != 0)
      break;
    for(i = 0; i < CONFIG_TOTAL_LENGTH; i++)
      ep0_stage[i] = hs ? openvrx_desc_config_fs[i] : openvrx_desc_config_hs[i];
    ep0_stage[1] = USB_DESC_OTHER_SPEED_CONFIGURATION;
    ep0_send_stage(CONFIG_TOTAL_LENGTH);
    return;

  case USB_DESC_STRING:
    switch(index) {
    case 0:
      ep0_send_code(openvrx_desc_lang, sizeof(openvrx_desc_lang));
      return;
    case 1:
      ep0_send_code((__code const uint8_t *)&openvrx_desc_manufacturer,
                    sizeof(openvrx_desc_manufacturer));
      return;
    case 2:
      ep0_send_code((__code const uint8_t *)&openvrx_desc_product,
                    sizeof(openvrx_desc_product));
      return;
    case 3:
      for(i = 0; i < sizeof(serial_desc); i++)
        ep0_stage[i] = serial_desc[i];
      ep0_send_stage(sizeof(serial_desc));
      return;
    case 0xEE:
      ep0_send_code((__code const uint8_t *)&openvrx_desc_msos10_string,
                    sizeof(openvrx_desc_msos10_string));
      return;
    default:
      break;
    }
    break;

  case USB_DESC_BINARY_OBJECT_STORE:
    if(index != 0)
      break;
    ep0_send_code(openvrx_desc_bos, sizeof(openvrx_desc_bos));
    return;

  default:
    break;
  }

  STALL_EP0();
}

// ---------------------------------------------------------------------------
// Non-standard SETUP requests
// ---------------------------------------------------------------------------

void handle_usb_setup(__xdata struct usb_req_setup *req) {
  // Microsoft OS descriptor requests are pure table reads; answer them here.
  if(req->bRequest == MS_VENDOR_CODE &&
     (req->bmRequestType == (USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN) ||
      req->bmRequestType == (USB_RECIP_IFACE | USB_TYPE_VENDOR | USB_DIR_IN))) {
    switch(req->wIndex) {
    case 0x0007:   // MS_OS_20_DESCRIPTOR_INDEX
      ep0_send_code((__code const uint8_t *)&openvrx_desc_msos20_set,
                    sizeof(openvrx_desc_msos20_set));
      return;
    case 0x0004:   // MS OS 1.0 extended compat ID
      ep0_send_code(openvrx_desc_msos10_compat, sizeof(openvrx_desc_msos10_compat));
      return;
    case 0x0005:   // MS OS 1.0 extended properties (interface 0 only)
      if((req->wValue & 0xFF) == 0) {
        ep0_send_code((__code const uint8_t *)&openvrx_desc_msos10_props,
                      sizeof(openvrx_desc_msos10_props));
        return;
      }
      break;
    default:
      break;
    }
    STALL_EP0();
    return;
  }

  // Everything else is handled by the main loop. The host does not start a
  // new control transfer before this one completes, so a second SETUP while
  // one is pending means the previous one was abandoned: refuse it.
  if(svc_setup) {
    STALL_EP0();
    return;
  }
  svc_setup = 1;
  svc = 1;
}

// ---------------------------------------------------------------------------
// Configuration and interface
// ---------------------------------------------------------------------------

static void reset_toggles(void) {
  TOGCTL = 0x10 | 1;          // EP1 IN
  TOGCTL = 0x10 | 1 | _R;
  TOGCTL = 0x10 | 2;          // EP2 IN
  TOGCTL = 0x10 | 2 | _R;
}

bool handle_usb_set_configuration(uint8_t config_value) {
  if(config_value > 1)
    return false;
  usb_config_value = config_value;
  reset_toggles();
  // A configuration change resets the data toggles: stop any stream.
  svc_bus = 1;
  svc = 1;
  return true;
}

bool handle_usb_set_interface(uint8_t interface, uint8_t alt_setting) {
  if(interface != 0 || alt_setting != 0)
    return false;
  reset_toggles();
  return true;
}

// ---------------------------------------------------------------------------
// Bus reset and high-speed handshake
// ---------------------------------------------------------------------------

void isr_USBRESET(void) __interrupt {
  usb_config_value = 0;
  svc_bus = 1;
  svc = 1;
  CLEAR_USB_IRQ();
  USBIRQ = _URES;
}

void isr_HISPEED(void) __interrupt {
  svc_bus = 1;
  svc = 1;
  CLEAR_USB_IRQ();
  USBIRQ = _HSGRANT;
}
