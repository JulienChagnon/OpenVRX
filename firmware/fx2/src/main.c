// SPDX-License-Identifier: GPL-3.0-or-later
//
// OpenVRX FX2 streaming firmware: start-up and main loop.
//
// The 8051 is not in the sample path: in the normal (AUTOIN) mode the slave
// FIFO packetises the C5's lanes into EP2 by itself. The main loop spends its
// time in one of the hand-written loops of hotloops.asm and leaves it only
// when an interrupt sets `svc` (SETUP packet, status tick, bus event).

#include <fx2regs.h>
#include <fx2ints.h>
#include <fx2delay.h>
#include <fx2usb.h>
#include "openvrx.h"

volatile __bit svc;
volatile __bit svc_setup;
volatile __bit svc_tick;
volatile __bit svc_bus;

int main(void) {
  CPUCS = _CLKSPD1;             // 48 MHz CPU, CLKOUT pin not driven
  CKCON &= ~(_MD2 | _MD1 | _MD0);  // MOVX in 2 cycles (no stretch)

  // Present the OpenVRX identity: detach from the bus while initialising so
  // the host forgets the boot-loader device, then ReNumerate.
  USBCS |= _DISCON | _RENUM;

  stream_init();
  board_init();                 // timer, EEPROM, board ID (enables EA)

  delay_ms(100);
  usb_self_powered = false;
  usb_init(/*disconnect=*/false);
  USBIRQ = _URES | _HSGRANT;
  USBIE |= _URES | _HSGRANT;

  for(;;) {
    svc = 0;
    if(svc_bus) {
      svc_bus = 0;
      stream_stop();
    }
    if(svc_setup)
      requests_service();
    if(svc_tick) {
      svc_tick = 0;
      stream_tick();
    }
    stream_run();
  }
}
