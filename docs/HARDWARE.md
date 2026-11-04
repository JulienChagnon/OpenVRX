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

"-" in an 8-bit column: the C5 pin is high-impedance with a weak pull-down
and the FX2 ignores PD in 8-bit mode.

## Custom boards

`board custom` uses the map set in `idf.py menuconfig` -> OpenVRX radio
head -> Custom pin map: a clock GPIO, one GPIO per FX2 pin for the 16-bit
presets and one per FX2 pin for the 8-bit presets (-1 = tied to GND).
Allowed GPIOs are 0..12 and 23..28 (13/14 are USB, 15..22 flash/PSRAM);
each GPIO may appear once per table and never as both clock and lane. An
invalid map is refused with `ERR board state custom map invalid: <reason>`.

## Signal integrity

- 30 AWG (Kynar) wire, 3-5 cm per signal. At these lengths length matching
  does not matter (2 cm is about 0.1 ns). No breadboard for the final
  build; one is acceptable for first light.
- Grounds: at least two GND wires between the boards, one directly beside
  the clock wire, the other at the far end of the lane bundle. Keep each
  bundle close to its ground to keep the loop area small.
- Drive strength: `drive <lanes> [clk]`, ESP-IDF `gpio_drive_cap_t`
  0..3 (about 5, 10, 20, 40 mA). Default lanes=1, clk=2. Choose by the
  edge check (phase probe at `clk 6`, OpenVRX-DESIGN.txt 5.4), not by
  defaulting to the weakest: weak drive slows edges and eats the 7.7 ns
  setup plus hold window the FX2 needs; strong drive rings. Save the
  result with `save`.
- If lanes still show errors: 22-33 ohm series resistors at the C5 end of
  each lane and the clock.
- The FX2 needs data steady 3.2 ns before and 4.5 ns after each IFCLK
  edge, against a bus that changes every 12.5 ns. The edge can land on one
  of three positions against the bus; `slip` moves it one 4.17 ns step
  and the host's phase calibration picks a clean position.

## Power

- v1, two cables: the C5's own USB-C gives power and the console; the
  FX2's mini-USB carries data. Do not also connect the FX2's 5V to the C5
  in this mode.
- v2, one cable: the FX2 board's 5V (USB VBUS) feeds the C5's 5V pin and
  the console moves to a UART relay through the FX2 (needs two more C5
  pins; decide after v1 works). FX2 board at most 85 mA plus the C5
  receiving at roughly 100-150 mA fits one USB 2.0 port.
- All grounds meet at the laptop through USB. Do not add a separately
  powered supply without a common ground point.

## Which preset to wire first

The owner can rewire freely; the options below are ordered by how quickly
they produce useful data. Byte rates assume the FX2 sustains them on the
laptop (measure with the FX2 throughput test first).

1. **C5-Zero 8x8 at 13.33 MS/s** (`clk 18`, 26.67 MB/s, 16-bit FX2 mode).
   All live ADC bits; one wiring also covers every smaller preset, so the
   host can emulate 7x7 to 4x4 in software or switch presets with no
   rewiring. Best first build if a C5-Zero is available.
2. **XIAO 7x7 at 13.33 MS/s** (`clk 18`, 26.67 MB/s). Needs the four back
   pads. The default on the XIAO.
3. **XIAO 5x5 on front pins only** (`clk 18`, 26.67 MB/s). Fastest XIAO
   build: no back pads, every wire on the header. Later add the back pads
   for 7x7 without moving any front wire.
4. **4x4 in 8-bit mode at 20, 26.67 or 40 MS/s** (`clk 12`, `clk 9`,
   `clk 6`; 20, 26.67, 40 MB/s). Higher sample rate (less phase wrap at
   the sync tip) for fewer bits. On the XIAO this is the 8-bit front-pin
   wiring; on the C5-Zero no rewiring. Try `4x4top` and `4x4ultra`
   (C5VRX's ultrafine slice) on the same capture setup. 40 MB/s is at the
   FX2 ceiling; treat it as an experiment.
5. **C5-Zero 8x8 at 20 MS/s** (`clk 12`, 40 MB/s): the stretch goal at the
   FX2 ceiling; check the FIFO-full counter in the FX2 status block.

Bring-up order for any of them: flash the C5, `board` + `lanes` + `pins`
to confirm the map, `selftest walk` with the FX2 streaming (the host names
stuck, swapped or shorted lanes), `selftest off`, then the phase probe
with `clk 6` and `slip`, then the working divider.

## Simulation ranking

Results of the sample-format simulations (experiments S1/S2) are produced
by the lab tools and live in `lab/results/`. This section will summarise
them once they exist; no numbers are given here until then.

## Flashing the C5

ESP-IDF v6.0.2: in `firmware/c5`, `idf.py set-target esp32c5` (once) and
`idf.py build`. Then flash with

    python tools/flash.py COM9

not `idf.py flash`. A USB DTR/RTS line reset on a running image leaves the
modem domain running into the ROM loader, which then stops answering
(esptool "Write timeout", console writes failing); only unplugging all
power clears it. `tools/flash.py` (adapted from C5VRX) sends the console
command `dl`, which stops the MODEM_DIAG writer and Wi-Fi and reboots into
the loader cleanly, then flashes with `--before no-reset --after
watchdog-reset`. A brand new C5 still running its factory image can be
flashed with `idf.py flash` once.

The same rule holds for any serial tool: open the console with DTR and RTS
low before the open (pySerial defaults assert both, and closing the port
then resets the chip). `tools/c5cmd.py` does this:

    python tools/c5cmd.py COM9 ver status pins

## Running without the C5 on USB (FX2-only, the default)

The app's default control mode is FX2 only: just the FX2 is on the PC's
USB, and the C5 runs the settings it stored with `save`. Set it up once
with the C5 on USB:

    python tools/c5cmd.py COM9 "tune <MHz>" "lanes <preset>" "clk <div>" save status

then power the C5 from the FX2 board and start the app with the same layout
and clock: `openvrx --lanes <preset> --clk <div>` (they must match what the C5
saved, or the picture is garbage). In this mode the app cannot change
channel or gain, and sampling-phase calibration and the lane self-test are
unavailable (both need the console). `--control c5` switches back to full
console control when the C5 is on USB too.

Powering the XIAO from the FX2 board: wire an FX2 header **VCC pin (the
board's 3.3 V rail, see PINOUTS.md) to the XIAO 3V3 pin** (plus the GND wires the data lanes already need). The 3V3 pin is
the XIAO's regulated rail, so this feeds it directly; a receiving C5 draws
roughly 100 to 150 mA, well within the FX2 board's 3.3 V regulator and one
USB port. Do not use BAT+/BAT- or VBAT: they are the LiPo battery input
(3.0 to 4.2 V through the charger), not a supply input for 3.3 V. Remove
the 3V3 wire before plugging the XIAO's own USB in, so two supplies never
drive the rail at once.

## The angle8 preset (not available)

`lanes angle8` answers `ERR lanes unsupported`. An on-chip angle needs
MODEM_DIAG captured by PARLIO RX, a BitScrambler LUT and PARLIO TX out (the
pipeline C5VRX uses for its decoder). On the ESP32-C5 this does not pay
off:

- PARLIO RX has at most 8 data lines (`SOC_PARLIO_RX_UNIT_MAX_DATA_WIDTH 8`
  in ESP-IDF v6.0.2 `soc_caps.h`). The LUT input is therefore at most 4 I
  + 4 Q bits, exactly what the 4x4 presets already deliver; an 8-bit angle
  computed from them carries no information the PC cannot compute from
  the 4x4 word itself, and it loses the magnitude.
- The GPIO matrix reaches peripheral inputs only through pads, so the 8
  captured bus bits need 8 loopback pads besides 8 output lanes and the
  clock: 17 pins, more than the XIAO has and every pin of the C5-Zero.
- There is one PARLIO TX unit, so the sample clock would have to move to
  the data transmitter, and the 40 MS/s capture rate would need a
  BitScrambler decimation program as well.

A worthwhile angle path needs more than 8 input bits per sample, which this
chip's PARLIO RX cannot capture.
