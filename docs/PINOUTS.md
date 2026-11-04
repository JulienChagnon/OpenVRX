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

## FX2LP "CY7C68013A mini" board (LCSoft style, blue)

- Chip: Cypress CY7C68013A-56PVXC (FX2LP), 24 MHz crystal.
- Regulator: AMS1117-3.3 (3.3 V, from USB 5 V). Current rating not stated
  on the vendor pages; the AMS1117 part is rated about 1 A, and the FX2
  itself uses roughly 50 to 100 mA.
- EEPROM: AT24C128 (16 KB I2C). The jumper selects whether the FX2 boots
  from the EEPROM or into the default USB loader mode; OpenVRX loads its
  firmware from the host either way.
- Switch and reset button on board. The sigrok page notes the FX2 I/O is
  5 V tolerant (about -0.5 to 5.25 V); the C5 drives 3.3 V levels.
- Headers: two 20-pin rows break out every FX2 pin, each pin labelled on
  the silkscreen (PB0..PB7, PD0..PD7, PA0..PA7, RDY0, RDY1, CTL0..CTL2,
  IFCLK, CLK, SCL, SDA, RST, WU2 and so on). Neither vendor page gives a
  pin-by-pin header map, so wire by the silkscreen labels.
- **The header pins labelled VCC are the 3.3 V rail** (the AMS1117
  output); the board has no 3.3V label. Before wiring the XIAO to it,
  check with a multimeter: VCC to GND should read about 3.3 V. If it reads
  about 5 V on your board, do not connect it to the XIAO 3V3 pin.

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
- Waveshare, CY7C68013A USB Board (mini) and schematic: https://www.waveshare.com/wiki/CY7C68013A_USB_Board_%28mini%29
