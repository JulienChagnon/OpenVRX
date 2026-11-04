# Board pinouts (reference)

Collected 2026-10-30 from the vendor pages listed under Sources. The
OpenVRX wiring tables themselves are in `HARDWARE.md`; this page is the raw
board facts behind them.

## Seeed XIAO ESP32-C5

From Seeed's pin map (wiki, updated 2026-07-09).

| XIAO pin | GPIO | Board function | Notes for OpenVRX |
|---|---|---|---|
| 5V | - | VBUS, "Power Input/Output" | USB 5 V rail |
| GND | - | ground | |
| 3V3 | - | "3V3_OUT", regulated output | can be fed 3.3 V from the FX2 board when USB-C is unplugged |
| D0 | GPIO1 | ADC, LP_UART_DSRN | lane |
| D1 | GPIO0 | LP_UART_DTRN | lane |
| D2 | GPIO25 | | lane (strapping pin) |
| D3 | GPIO7 | SDIO_DATA1 | lane (strapping pin) |
| D4 | GPIO23 | I2C SDA | lane |
| D5 | GPIO24 | I2C SCL | lane |
| D6 | GPIO11 | UART TX | lane |
| D7 | GPIO12 | UART RX | lane |
| D8 | GPIO8 | SPI SCK | lane in 16-bit layouts, free in 4x4 |
| D9 | GPIO9 | SPI MISO | lane in 16-bit layouts, free in 4x4 |
| D10 | GPIO10 | SPI MOSI | sample clock (IFCLK) |
| MTDO pad (back) | GPIO5 | JTAG, LP_UART_TXD | lane in 6x6/7x7 |
| MTDI pad (back) | GPIO3 | JTAG, ADC | lane in 7x7 |
| MTCK pad (back) | GPIO4 | JTAG, ADC | lane in 6x6/7x7 |
| MTMS pad (back) | GPIO2 | JTAG, ADC | lane in 6x6/7x7 (strapping pin) |
| (internal) | GPIO6 | battery voltage ADC (divider 1/2) | not used |
| (internal) | GPIO26 | battery measurement enable | not used |
| Boot button | GPIO28 | boot mode | not used |
| User LED | GPIO27 | yellow LED | not used |
| Reset button | EN | chip enable | |

Also on the board: battery charger SGM40567 with the LiPo pads (BAT+ /
BAT-, battery input, not a supply input for 3.3 V), red charge LED, U.FL
antenna connector. The 3V3 current limit is not stated by Seeed.

The OpenVRX firmware's XIAO map (`firmware/c5/main/logic/openvrx_boards.c`)
matches this table: the C5 console `pins` command prints the live mapping.

### XIAO ESP32-C5 diagram

Front, USB-C at the top, component side up. Back, same orientation
(USB-C still at the top), so the back pads appear mirrored relative to the
front columns: MTDO sits behind the left column, MTDI behind the right.

```
                FRONT                                   BACK
             +--[USB-C]--+                         +--[USB-C]--+
 GPIO1  D0  -| o       o |-  5V                    |           |
 GPIO0  D1  -| o       o |-  GND                   | MTDO MTDI |  GPIO5   GPIO3
 GPIO25 D2  -| o       o |-  3V3                   | GND  RST  |
 GPIO7  D3  -| o       o |-  D10  GPIO10           | MTCK MTMS |  GPIO4   GPIO2
 GPIO23 D4  -| o       o |-  D9   GPIO9            | 3V3  BOOT |  (3.3V)  GPIO28
 GPIO24 D5  -| o       o |-  D8   GPIO8            |           |
 GPIO11 D6  -| o       o |-  D7   GPIO12           |  BAT- +   |
             | R [UFL] B |                         +-----------+
             +-----------+
   R = reset button, B = boot button
```

Pad numbers in Seeed's table: P1 MTDO, P2 GND, P3 3V3, P4 MTCK, P5 MTDI,
P6 RST, P7 MTMS, P8 BOOT. Alternate functions Seeed lists, for reference:
D0 also LP_UART_DSRN/ADC, D1 LP_UART_DTRN, D3 SDIO_DATA1, D8 SDIO_DATA0,
D9 SDIO_CLK, D10 SDIO_CMD, MTDO LP_UART_TXD, MTCK LP_UART_RXD, MTDI
LP_I2C_SCL, MTMS LP_I2C_SDA. The silkscreen names (SDA, SCL, TX, RX,
SCK, MISO, MOSI) are only defaults: the C5 GPIO matrix can route I2C, UART
or SPI to any of these pins.

## FX2LP "CY7C68013A mini" board (LCSoft, blue)

Facts below come from the LCSoft schematic ("CY7C68013A USB开发板", Rev A,
2010-10-07) and the Cypress FX2LP datasheet (38-08032 Rev *W), checked
2026-11-01.

- Chip: Cypress CY7C68013A-56PVXC (FX2LP, 56-pin SSOP), 24 MHz crystal.
  Supply current 50 typical / 85 mA max with the 8051 running on USB HS.
- **The 56-pin package has no UART pins.** RXD0/TXD0/RXD1/TXD1 exist only
  on the 100 and 128-pin packages (datasheet table 10). Serial to the C5
  would be bit-banged on a port pin; the hardware I2C master (100 or
  400 kHz, master only) is on the headers.
- I/O: 3.3 V, inputs 5 V tolerant (5.25 V max), VIH 2.0 V, outputs
  4 mA, 10 mA max per port.
- Regulator U2: AMS1117-3.3. SOT-223: pin 1 GND, pin 2 Vout (also the
  tab), pin 3 Vin. 47 uF + 0.1 uF on the input, 22 uF + 0.1 uF on the
  output.
- Power path: mini-USB VBUS -> fuse F1 -> slide switch SW1 -> **Vin**
  (about 5 V) -> AMS1117 -> **Vcc** (3.3 V). Vin also feeds the PWR LED
  (R1 1 k). **Vin (5 V) is not on any header pin.** To take 5 V off the
  board, solder to the Vin net after the switch: AMS1117 pin 3, or the
  positive end of C16 (47 uF). Taking it after SW1 means the switch
  still turns both boards off.
- EEPROM U3: AT24C128 on SCL/SDA, A1 and A2 tied to GND, A0 low (the
  AT24C internal pull-down) unless jumper J1 ties it to Vcc: J1 off =
  address 0x50, J1 on = 0x51
  (the FX2 boots firmware from 0x51 only). OpenVRX loads its firmware from
  the host either way.
- SCL and SDA have 2.2 k pull-ups to 3.3 V (R3, R4) already, enough for
  another I2C device on the bus.
- RESET#: 100 k pull-up and 1 uF to GND plus the RST button. WAKEUP#:
  10 k pull-up, not on the headers.
- No user LEDs on this board (only PWR). Some clones put LEDs on PA0/PA1,
  see the variant note below.

### FX2 board header diagram

Component side up, mini-USB on the left, as on the silkscreen. Each
header is 2 x 10, "inner" is the row nearer the chip. Numbers are the
schematic pin numbers (J3 top, J2 bottom).

```
  J3 (top edge)
  outer:  PD3  PD1  PA7  PA5  PA3  PA1  CTL2 CTL0 PB6  PB4      even pins 2..20
           2    4    6    8    10   12   14   16   18   20
  inner:  PD4  PD2  PD0  PA6  PA4  PA2  PA0  CTL1 PB7  PB5      odd pins 1..19
           1    3    5    7    9    11   13   15   17   19

  [USB]          [ CY7C68013A-56PVXC ]          [AT24C128] [J1]

  J2 (bottom edge)
  inner:  PD6  GND  GND  RDY1 GND  IFCLK SDA PB1  PB3  3.3V     even pins 2..20
           2    4    6    8    10   12   14   16   18   20
  outer:  PD5  PD7  CLK  RDY0 GND  GND  SCL  PB0  PB2  3.3V     odd pins 1..19
           1    3    5    7    9    11   13   15   17   19
```

The same as a list, with the FX2 chip pin and its slave FIFO role:

| Header pin | Signal | FX2 chip pin | OpenVRX use |
|---|---|---|---|
| J2-1, 2, 3 | PD5, PD6, PD7 | 1, 2, 3 | FD13..FD15, upper data byte (16-bit modes) |
| J2-4, 6, 9, 10, 11 | GND | | ground |
| J2-5 | CLKOUT (silk "CLK") | 5 | not used (FX2's own 48 MHz clock out) |
| J2-7 | RDY0 / SLRD | 8 | tie high (see note) |
| J2-8 | RDY1 / SLWR | 9 | tie low (see note) |
| J2-12 | IFCLK | 20 | sample clock from the C5 |
| J2-13 | SCL | 22 | I2C clock (EEPROM, 2.2 k pull-up) |
| J2-14 | SDA | 23 | I2C data (EEPROM, 2.2 k pull-up) |
| J2-15..18 | PB0..PB3 | 25..28 | FD0..FD3 |
| J2-19, 20 | Vcc 3.3 V | | AMS1117 output |
| J3-1..5 | PD4, PD3, PD2, PD1, PD0 | 56, 55, 54, 53, 52 | FD12..FD8 |
| J3-6 | PA7 / FLAGD / SLCS# | 47 | leave open |
| J3-7 | PA6 / PKTEND | 46 | tie high |
| J3-8 | PA5 / FIFOADR1 | 45 | tie low |
| J3-9 | PA4 / FIFOADR0 | 44 | tie low |
| J3-10 | PA3 / WU2 | 43 | free |
| J3-11 | PA2 / SLOE | 42 | tie high |
| J3-12 | PA1 / INT1# | 41 | free (identify blink drives it briefly) |
| J3-13 | PA0 / INT0# | 40 | free (identify blink drives it briefly) |
| J3-14, 15, 16 | CTL2, CTL1, CTL0 (FLAGC, B, A) | 38, 37, 36 | not used |
| J3-17..20 | PB7, PB6, PB5, PB4 | 32..29 | FD7..FD4 |

Free for other uses in the 8-bit (4x4) modes: PD0..PD7, PA0, PA1, PA3,
CTL0..CTL2, and SCL/SDA as an I2C bus shared with the EEPROM.

On the LCSoft schematic the RDY0 and RDY1 labels match the chip (J2-7 is
chip pin 8, RDY0; J2-8 is chip pin 9, RDY1).

Bench finding (2026-10-30): on the owner's board, wiring RDY0 to VCC and
RDY1 to GND by the silkscreen gave RDY1 high and RDY0 low in the FX2's own
readback (GPIFREADYSTAT), confirmed by the slave FIFO only writing with
SLWR set active high. Either the RDY0/RDY1 labels are swapped on this
board or the wires sat one pin off; swapping the two wires fixes it. Check
any new board with `fx2_diag --live` before trusting the labels.

Variant note (Geeetech style, per sigrok): LEDs on PA0 and PA1 with J1
connecting them to VCC, and J2 setting the EEPROM address bit A0. OpenVRX
does not use PA0/PA1, so the LED jumper does not matter.

## Sources

- Seeed Studio, Getting Started with XIAO ESP32-C5: https://wiki.seeedstudio.com/xiao_esp32c5_getting_started/
- sigrok wiki, Lcsoft Mini Board: https://sigrok.org/wiki/Lcsoft_Mini_Board
- LCSoft, CY7C68013A USB board schematic, Rev A, 2010-10-07 (lcsoft.taobao.com)
- Cypress (Infineon), CY7C68013A/14A/15A/16A EZ-USB FX2LP datasheet, document 38-08032 Rev *W
- Waveshare, CY7C68013A USB Board (mini) and schematic: https://www.waveshare.com/wiki/CY7C68013A_USB_Board_%28mini%29
