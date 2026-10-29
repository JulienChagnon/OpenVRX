<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# OpenVRX FX2 firmware

Streaming firmware for the CY7C68013A (EZ-USB FX2LP) boards that carry the
ESP32-C5 I/Q lanes to the PC. The slave FIFO latches the lanes on the C5's
sample clock (external IFCLK) and the chip sends them on EP2 (bulk IN, 512
bytes, quad buffered, AUTOIN). EP1 IN carries a 32-byte status block every
8 ms. The protocol is in `docs/PROTOCOL.md`, section 2.

Built with SDCC and a pinned copy of libfx2 (0BSD, `third_party/libfx2/`,
commit recorded in `PINNED.txt`).

## Building

Needs SDCC 4.x (tested with 4.5.0) and Python 3.8 or newer.

```
python build.py            # Windows, Linux, macOS
python build.py --test     # also run the descriptor and image tests
make / make test           # same, on systems with make
```

SDCC is found through `SDCC_HOME`, `PATH`, or `C:\Program Files\SDCC`.
Outputs:

| File | Use |
|---|---|
| `build/openvrx-fx2.ihx` | RAM image (Intel HEX) |
| `build/openvrx-fx2.iic` | EEPROM image, C2 boot format, for a 24LC128/256 at 0x51 |
| `prebuilt/openvrx-fx2.ihx`, `.iic` | the same, committed; the host embeds the `.ihx` |

The build fails on any compiler warning in `src/`. It also runs
`tools/cycles.py`, which checks the cycle annotations of the hand-written
loops in `src/hotloops.asm` against the assembler listing.

Memory map: code 0x0000-0x37FF (about 10.6 KB used), xdata 0x3800-0x3FEF,
board ID preset 0x3FF0-0x3FF7 (written by the host, never linked).

## Getting the firmware onto the board

**Normal use: nothing to do.** The FX2 has no flash. When a blank board is
plugged in it enumerates with its boot ROM (usually `04b4:8613`; logic
analyzer clones may show `0925:3881` or `1d50:608c`). The OpenVRX host app
uploads the embedded image to RAM (vendor request `0xA0`), the chip
disconnects and comes back as `1209:0001` "OpenVRX FX2". Windows binds WinUSB
to that by itself (Microsoft OS 2.0 descriptors), no driver install.

**One-time Zadig step (Windows only).** The boot-ROM device has no driver,
so the host app cannot upload to it until WinUSB is bound to the boot-loader
ID once:

1. Plug in the FX2 board, start Zadig (https://zadig.akeo.ie).
2. Options, List All Devices; pick the device with ID `04B4 8613` (or the
   ID your board shows).
3. Driver: WinUSB, Install Driver.

Do this once per boot-loader VID:PID on each PC. Linux needs a udev rule
granting access to both the boot-loader IDs and `1209:0001`.

**Standalone boot from EEPROM (optional).** Fit a 24LC128 or 24LC256,
strapped to address 0x51 (A0 high; on the common blue board this is the
EEPROM jumper), and have the host write `prebuilt/openvrx-fx2.iic` with the
`EEPROM_WRITE` request (or `fx2tool -B update`). The boot ROM then loads the
firmware itself at power-up. Keep the board ID record at 0x3FF8 when writing
the image (the image is smaller than that).

**Manual upload for testing:** `pip install fx2`, then
`fx2tool -d 04b4:8613 load prebuilt/openvrx-fx2.ihx`.

## First light without custom firmware

Before wiring a C5, check the board, the cable and the USB link with sigrok:

1. Install PulseView (https://sigrok.org/wiki/Downloads). On Windows, run
   Zadig as above for the board's ID.
2. In PulseView choose the driver "fx2lafw" and scan. The board is found
   and fx2lafw is uploaded to RAM automatically.
3. Capture 8 channels at 1-24 MHz. Touch PB0..PB7 with a 3.3 V wire, or
   wire the C5 and run its `selftest walk`, and watch the lines toggle.

fx2lafw samples on the FX2's internal clock and is only a sanity check; real
captures need this firmware and the C5's clock on IFCLK. Unplug and replug
the board afterwards to return to the boot ROM.

## Wiring summary

PB0-7 / PD0-7 = lanes, IFCLK = C5 sample clock, SLWR (RDY1) to GND,
FIFOADR0/1 (PA4/PA5) to GND, SLRD (RDY0), SLOE (PA2), PKTEND (PA6) to 3.3 V,
two ground wires. The internal-clock throughput test (`START` bit 2) needs
the same SLWR and FIFOADR ties; the lanes may float. The test pattern (bit 5)
needs no wiring at all. Start the C5 clock before `START` and stop the stream
before `clk off`.

## How it works

- **Start-up**: 48 MHz CPU, MOVX without stretch, detach from USB, set up
  the FIFOs, the 1 ms timer, EEPROM detection and the board ID, then
  ReNumerate after 100 ms.
- **START**: under the internal 48 MHz clock with the pins disconnected
  (ports mode): NAKALL, FIFOPINPOLAR, AUTOIN off, reset EP2, AUTOINLEN
  (512, or 64 at full speed), final EP2FIFOCFG (WORDWIDE, AUTOIN). Then the
  final clock source and polarity are selected, the reset sequence of TRM
  9.3.13 is repeated under that clock (the TRM wants an external IFCLK
  running before it is selected), NAKALL is released and only then are the
  pins connected (IFCFG = 11). STOP switches back to the internal clock
  before flushing. Every FIFO register write is followed by a 16-cycle
  SYNCDELAY (TRM 15.15, safe down to a 5 MHz IFCLK).
- **AUTOIN streaming**: the CPU only polls the EP2 full flag (10 cycles per
  poll) and counts polls that find it full.
- **Watch mode** (manual commit, `src/hotloops.asm`): a packet is ready when
  `EP2FIFOBC/512 - NPAK > 0`; it is committed with `INPKTEND = 0x02` or
  skipped with `0x82` (REVCTL.ENH_PKT). Idle iteration 39 cycles (3.25 us),
  commit or skip 66-67 cycles (5.6 us), so up to about 179,000 packets/s
  (about 91 MB/s of FIFO input). 13.33 MS/s at 16 bit is 52,083 packets/s,
  29 % of the budget. Internal 48 MHz at 16 bit (187,500 packets/s) is just
  beyond it and shows FIFO-full events. Live switching uses the TRM 9.3.13
  sequence with the pins disconnected.
- **Test pattern**: the CPU fills packets through autopointer 2 at 7 cycles
  per word: about 6,300 packets/s, 3.2 MB/s. With `SET_OPTION 4 = 1` only the
  high bytes are rewritten: about 11,000 packets/s, 5.7 MB/s.
- **Control requests**: descriptor and Microsoft OS requests are answered in
  the SETUP interrupt through the setup data pointer (copied to word-aligned
  scratch RAM, clamped to wLength). Vendor requests run in the main loop;
  the hot loops return within one iteration when a request arrives.

## Needs confirmation on hardware

1. Watch-mode packet detection: whether EP2FIFOBC includes packets already
   committed to USB. The default test is correct either way; option 1 of
   `SET_OPTION` (byte count only) is faster to react but would send short
   packets if the count does include them.
2. Test-pattern fast fill (`SET_OPTION 4 = 1`): relies on EP2 buffers
   keeping their contents and rotating in order.
3. The effect of selecting an external IFCLK that is not running (the
   firmware avoids it during configuration; the C5 clock should run before
   `START`).
4. LED on PA0/PA1 polarity (identify blinks both pins at 4 Hz).
5. EEPROM page write sizes on the board's actual part (override with
   `EEPROM_WRITE` wIndex).
