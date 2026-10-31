# OpenVRX hardware: wiring a C5 receiver to an FX2LP board

This page covers the radio head side: which ESP32-C5 pin goes to which FX2LP
pin for every lane preset, the FX2 control-pin ties, grounding, drive
strength, power, and which preset to wire first. The lane presets are the
ones in `host/crates/openvrx-core/src/lanes.rs`; the firmware is
`firmware/c5/` and the console protocol is `docs/PROTOCOL.md` section 1.

The pin maps here are the ones compiled into the firmware
(`firmware/c5/main/logic/openvrx_boards.c`). `pins` on the C5 console prints
the live map for the selected board and preset, and `selftest walk` lets
the host prove the wiring lane by lane before any radio data is trusted.

## How the lanes work

- The C5's MODEM_DIAG bus carries Q on bus bits 0..9 and I on bus bits
  10..19 (10-bit two's complement, only bits 9..2 live). The GPIO matrix
  connects any bus bit straight to a pad; nothing re-times it.
- 16-bit presets (`8x8 7x7 6x6 5x5`): Q is MSB aligned on FX2 PB7 downward,
  I on PD7 downward. Lanes a preset does not carry read 0.
- 8-bit presets (`4x4top 4x4fine 4x4ultra`): Q on PB3..PB0, I on PB7..PB4.
  The ADC bits are 9,8,7,6 (`top`), 9,7,6,5 (`fine`) or 9,6,5,4 (`ultra`,
  C5VRX's ultrafine lane), MSB first.
- The sample clock is PARLIO TX output, 240 MHz / divider (`clk 18` =
  13.33 MHz, `clk 12` = 20 MHz, `clk 9` = 26.67 MHz, `clk 6` = 40 MHz).
  Dividers that are multiples of 3 are locked to the 80 MHz bus.
- A board map fixes one C5 pin per FX2 pin. Pins of the active wiring that
  a smaller preset does not use are driven low by the C5, so they read 0
  like a GND tie. Pins that belong only to the other wiring of the same
  board stay high-impedance with a weak pull-down, so a leftover wire never
  fights a driven pin.
- FX2 pins with no C5 pin ("GND" below) must be tied to GND.

## FX2 control pins (every wiring)

Slave FIFO, synchronous, external IFCLK (OpenVRX-DESIGN.txt section 4.3).
These pins are inputs in slave FIFO mode; a 1 kohm resistor per tie is
cheap insurance against firmware that ever drives one, a direct wire also
works.

| FX2 pin | Connect to | Why |
|---|---|---|
| IFCLK | C5 clock pin (table below) | sample clock |
| RDY1 (SLWR) | GND | write strobe permanently active (active-low default); see the board note below |
| PA4 (FIFOADR0) | GND | selects EP2 |
| PA5 (FIFOADR1) | GND | selects EP2 |
| RDY0 (SLRD) | 3.3 V (header VCC) | read strobe inactive; see the board note below |
| PA2 (SLOE) | 3.3 V | FIFO output enable inactive (FX2 never drives FD) |
| PA6 (PKTEND) | 3.3 V | no short packets |
| PA7 (SLCS#) | leave open | chip select disabled in firmware |
| GND | C5 GND | at least two wires, one right beside the clock wire |

**LCSoft-style mini board: tie by function, not by silkscreen.** The table
above names the FX2 chip's signals. On the owner's blue CY7C68013A mini
board the header labels for RDY0 and RDY1 are the other way round, so
there the wiring is:

| Header label | Connect to |
|---|---|
| RDY1 | VCC |
| RDY0 | GND |

Measured 2026-10-30: wired by the labels the other way (RDY1 to GND), the
FX2's own readback showed the chip's RDY1 high, and the FIFO only accepted
data with the write strobe switched to active high. Whatever the board,
run `fx2_diag --live` once after wiring: it must show `RDY1 L` and
`RDY0 H` (chip signals), and the firmware option `SLWR_ACTIVE_HIGH`
(PROTOCOL.md, 0xB7) can cover a board wired the other way.

The FD pins (PB, PD) are FX2 inputs for an IN endpoint and are
high-impedance before the FX2 firmware loads, so the C5 strapping pins used
as lanes (GPIO2, 7, 25, 28) are not disturbed at reset.

## Seeed XIAO ESP32-C5

Header, USB at the top, component side up: left column D0..D6 = GPIO1, 0,
25, 7, 23, 24, 11; right column 5V, GND, 3V3, D10, D9, D8, D7 = GPIO10, 9,
8, 12. Back pads MTMS, MTDI, MTCK, MTDO = GPIO2, 3, 4, 5. GPIO27 (user LED),
GPIO28 (BOOT), GPIO6/26 (battery sense) and GPIO13/14 (USB) are not used.

The clock is on D10 (GPIO10): the GPIO header pin nearest the GND pin and
not a strapping pin. Run the GND wire from the GND pin two positions above
it, next to the clock wire.

### XIAO 16-bit wiring (7x7, 6x6, 5x5)

| FX2 pin | C5 pin | 7x7 | 6x6 | 5x5 |
|---|---|---|---|---|
| PB7 | GPIO1 (D0) | Q9 | Q9 | Q9 |
| PB6 | GPIO0 (D1) | Q8 | Q8 | Q8 |
| PB5 | GPIO25 (D2) | Q7 | Q7 | Q7 |
| PB4 | GPIO7 (D3) | Q6 | Q6 | Q6 |
| PB3 | GPIO23 (D4) | Q5 | Q5 | Q5 |
| PB2 | GPIO2 (MTMS pad) | Q4 | Q4 | low |
| PB1 | GPIO3 (MTDI pad) | Q3 | low | low |
| PB0 | GND | - | - | - |
| PD7 | GPIO24 (D5) | I9 | I9 | I9 |
| PD6 | GPIO11 (D6) | I8 | I8 | I8 |
| PD5 | GPIO12 (D7) | I7 | I7 | I7 |
| PD4 | GPIO8 (D8) | I6 | I6 | I6 |
| PD3 | GPIO9 (D9) | I5 | I5 | I5 |
| PD2 | GPIO4 (MTCK pad) | I4 | I4 | low |
| PD1 | GPIO5 (MTDO pad) | I3 | low | low |
| PD0 | GND | - | - | - |
| IFCLK | GPIO10 (D10) | clock | clock | clock |

5x5 on front pins only: wire just the D0..D10 rows and tie PB2, PB1, PD2,
PD1 (and PB0, PD0) to GND instead of wiring the back pads. The back pads
are then unconnected and the firmware's low drive on them is harmless.
8x8 does not fit the XIAO (`lanes 8x8` answers `ERR lanes range`).

The XIAO needs its back pads for 6x6 and 7x7: use headers or wire the pads
before soldering the board down.

### XIAO 8-bit wiring (4x4top, 4x4fine, 4x4ultra), front pins only

D0..D4 stay on the same FX2 pins as in the 16-bit wiring; D5, D6, D7 move
from PD7, PD6, PD5 to PB2, PB1, PB0.

| FX2 pin | C5 pin | 4x4top | 4x4fine | 4x4ultra |
|---|---|---|---|---|
| PB7 | GPIO1 (D0) | I9 | I9 | I9 |
| PB6 | GPIO0 (D1) | I8 | I7 | I6 |
| PB5 | GPIO25 (D2) | I7 | I6 | I5 |
| PB4 | GPIO7 (D3) | I6 | I5 | I4 |
| PB3 | GPIO23 (D4) | Q9 | Q9 | Q9 |
| PB2 | GPIO24 (D5) | Q8 | Q7 | Q6 |
| PB1 | GPIO11 (D6) | Q7 | Q6 | Q5 |
| PB0 | GPIO12 (D7) | Q6 | Q5 | Q4 |
| IFCLK | GPIO10 (D10) | clock | clock | clock |

When converting a 16-bit XIAO build: **remove the GND tie from PB0** before
D7 drives it, and disconnect the back-pad wires from PB1/PB2. PD0..PD7 are
ignored by the FX2 in 8-bit mode; tie them to GND or leave them to the FX2
firmware's port configuration. D8 and D9 stay free (high-impedance).

## Waveshare ESP32-C5-Zero

Edge pins, USB at the top, component side up: left column 5V, GND, 3V3,
GPIO0..GPIO5; right column GPIO11, GPIO12, GPIO13 (USB D-), GPIO14 (USB D+),
GPIO10, 9, 8, 7, 6. Back test pads GPIO23, 24, 25, 28. GPIO26 drives the
antenna switch and GPIO27 the RGB LED; neither is touched. GPIO28 also has
the BOOT button and a 10 kohm pull-up. GPIO11 (UART0 TX) reaches its edge
pin through a 499 ohm resistor (R9 on the Waveshare schematic), so it
carries only the least significant I lane (8x8 only); bridge R9 if that
lane looks soft on the edge check.

The clock is on GPIO0, the first GPIO after the 5V/GND/3V3 group, not a
strapping pin. Run its GND wire from the GND pin two positions above.

One wiring serves every preset: the 8-bit presets use the PB half of the
same wiring, so switching between 8x8 and 4x4 needs no rewiring (only the
FX2 word width changes).

| FX2 pin | C5 pin | 8x8 | 7x7 | 6x6 | 5x5 | 4x4top | 4x4fine | 4x4ultra |
|---|---|---|---|---|---|---|---|---|
| PB7 | GPIO1 | Q9 | Q9 | Q9 | Q9 | I9 | I9 | I9 |
| PB6 | GPIO2 | Q8 | Q8 | Q8 | Q8 | I8 | I7 | I6 |
| PB5 | GPIO3 | Q7 | Q7 | Q7 | Q7 | I7 | I6 | I5 |
| PB4 | GPIO4 | Q6 | Q6 | Q6 | Q6 | I6 | I5 | I4 |
| PB3 | GPIO5 | Q5 | Q5 | Q5 | Q5 | Q9 | Q9 | Q9 |
| PB2 | GPIO12 | Q4 | Q4 | Q4 | low | Q8 | Q7 | Q6 |
| PB1 | GPIO24 (pad) | Q3 | Q3 | low | low | Q7 | Q6 | Q5 |
| PB0 | GPIO28 (pad) | Q2 | low | low | low | Q6 | Q5 | Q4 |
| PD7 | GPIO10 | I9 | I9 | I9 | I9 | - | - | - |
| PD6 | GPIO9 | I8 | I8 | I8 | I8 | - | - | - |
| PD5 | GPIO8 | I7 | I7 | I7 | I7 | - | - | - |
| PD4 | GPIO7 | I6 | I6 | I6 | I6 | - | - | - |
| PD3 | GPIO6 | I5 | I5 | I5 | I5 | - | - | - |
| PD2 | GPIO23 (pad) | I4 | I4 | I4 | low | - | - | - |
| PD1 | GPIO25 (pad) | I3 | I3 | low | low | - | - | - |
| PD0 | GPIO11 (499 ohm) | I2 | low | low | low | - | - | - |
| IFCLK | GPIO0 | clock | clock | clock | clock | clock | clock | clock |

