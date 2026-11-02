# OpenVRX device protocols

This document is the contract between the two firmwares (`firmware/c5`,
`firmware/fx2`) and the host (`host/crates/openvrx-hw`). Change it first,
then the code on both sides.

Protocol version: **1**.

## 1. C5 radio head: serial console

Transport: the ESP32-C5's built-in USB Serial/JTAG (Espressif VID `0x303A`).
Any baud rate setting is accepted (it is a USB CDC device).

### Framing

- Host to C5: one ASCII command per line, terminated by `\n` (a preceding
  `\r` is ignored). Maximum 120 characters. Tokens are separated by single
  spaces.
- C5 to host: every command gets exactly one reply line starting with
  `OK ` or `ERR `, in order. `OK` replies list `key=value` pairs after the
  echoed command word so the host can parse them without positional rules.
- Unsolicited lines start with `EVT ` (events) or `LOG ` (human text).
  Anything else (ESP-IDF boot banner, panic output) is ignored by the host
  but recorded in the session log.
- `ERR <command> <code> <text>` codes: `syntax`, `range`, `state`,
  `unsupported`, `hw`.

The host owns all settings. On connect it sends the full state (`board`,
`lanes`, `clk`, `tune`, `gain`, `p11`, `filter`) so a C5 reset is harmless.
The C5 stores nothing in flash unless `save` is sent.

### Commands

| Command | Reply (example) | Notes |
|---|---|---|
| `ver` | `OK ver fw=0.1.0 proto=1 board=xiao idf=v6.0.2 chip=esp32c5 rev=1` | |
| `status` | `OK status mhz=5732 off_khz=0 gain=64 clk=18 clk_on=1 lanes=7x7 p11=1 filter=cal selftest=off up_ms=12345 slips=2 phy=ok` | |
| `board <xiao\|zero\|custom>` | `OK board name=xiao pins=15` | Selects the pin map (see HARDWARE.md). Re-routes lanes. |
| `pins` | `OK pins clk=G10 q=G1,G0,... i=G23,...` | Pin map for the current board and lanes, MSB first. |
| `lanes <preset>` | `OK lanes preset=7x7 data_pins=14` | Presets as `openvrx_core::LanePreset` names: `8x8 7x7 6x6 5x5 4x4top 4x4fine 4x4ultra angle8`. `ERR range` if the board lacks pins. |
| `tune <MHz> [offset_khz]` | `OK tune mhz=5732 off_khz=0` | Raw MHz, any value the PHY accepts (5645..5945 typical). |
| `gain <index>` | `OK gain index=64` | Fixed receive gain table index. Vendor AGC stays off. |
| `gains` | `OK gains max=77 table=...` | Gain table size and optional dB estimates. |
| `clk <divider>` | `OK clk div=18 hz=13333333` | Sample clock = 240 MHz / divider, divider 6..1023. Clock keeps running. |
| `clk off` / `clk on` | `OK clk on=0` | Stops/starts the clock pin (FX2 stops latching). |
| `slip [n]` | `OK slip n=1 total=3` | Moves the sample clock edge against the MODEM_DIAG bus by at least one 240 MHz tick, n times. Used by phase calibration. |
| `p11 <on\|off>` | `OK p11 on=1` | 802.11p receive setting, re-applied on every retune. |
| `filter <cal\|code> [skirt]` | `OK filter code=cal skirt=0` | Analog receive filter (C5VRX's calibrated filter control). |
| `bw <20\|40>` | `OK bw mhz=20` | Wi-Fi front end bandwidth select. |
| `drive <lanes 0-3> [clk 0-3]` | `OK drive lanes=1 clk=2` | Pad drive strength of the lane pins and the clock pin (ESP-IDF `gpio_drive_cap_t`, 0 = weakest). Default lanes=1 clk=2. |
| `selftest <off\|walk\|const HEX\|count>` | `OK selftest mode=walk lanes=14 phase_us=1000` | See below. Lanes are plain GPIO outputs while active. |
| `save` | `OK save` | Stores board/lanes/clk/tune/gain/p11/filter in NVS. |
| `reboot` | `OK reboot` | Then resets. |
| `dl` | `OK dl` | Reboots into the ROM download mode for flashing. |
| `ping` | `OK ping` | |

### Events

- `EVT boot fw=<ver> reason=<reset reason>`: after every reset.
- `EVT phy state=<ok|restored|error>`: PHY re-applied (e.g. after a Wi-Fi
  driver event).

### Self-test patterns

While a self-test runs, the lane pins are driven as ordinary GPIO outputs
(detached from MODEM_DIAG) and the sample clock keeps running, so the FX2
captures the pattern.

Lane order: the lanes of the current preset sorted by the FX2 word bit they
are meant to land on, ascending (bit 0 first). For `7x7` that is Q bit 9..3
on word bits 1..7, then I on word bits 9..15.

- `walk`: repeating sequence of phases, each `phase_us` long (default
  1000 us): phase 0 all lanes low, phase 1 all lanes high, then phase 2+j
  only lane j high, for every lane. The host segments the capture, and
  for each phase compares the word bits it sees with the expected bits,
  naming stuck, swapped, shorted or missing lanes.
- `const HEX`: drive the lanes with a fixed value; bit j of HEX goes to lane j.
- `count`: lane j carries bit j of a counter that increments every
  `phase_us`.

### Additions (firmware 0.1.0)

Added fields and arguments; nothing above changes meaning.

- `selftest walk [phase_us]` and `selftest count [phase_us]`: optional
  phase length, 10..10000000 us (default 1000). `selftest const HEX` replies
  `OK selftest mode=const lanes=<n> value=<hex>`; a value with bits above
  the lane count is `ERR selftest range`.
- `status` also reports `skirt=`, `bw=`, `board=` and `hz=` (sample clock in
  Hz). `phy=` is one of `ok`, `restored`, `error`, `unpinned` (the PHY
  binary is not the one the filter/11p controls were verified on; those
  controls then answer `ERR ... unsupported`), `down` (radio not started).
- `board` replies `OK board name=<b> pins=<n> lanes=<preset|none>`. When the
  current preset does not fit the new board, no lanes are routed
  (`lanes=none`) until a fitting `lanes` command.
- `pins` replies `OK pins board=<b> lanes=<preset> clk=G<n> q=<list> i=<list>`
  (`-` for an empty list).
- `lanes` replies also carry `word_bytes=`.
- `clk <divider>` also reports `locked=1` when the divider is a multiple of
  3 (frequency locked to the 80 MHz MODEM_DIAG bus). `clk off` holds the pad
  low while the clock keeps running internally, so the phase survives
  `clk off` / `clk on`; divider changes also keep it.
- `slip` replies also carry `rel=` (total slips mod 3: the edge position
  relative to boot, in 240 MHz ticks). `slip` needs the clock on
  (`ERR slip state clock is off`); `ERR slip hw clock pad not toggling`
  when the clock pad cannot be observed. Each slip briefly runs the clock
  at 200 kHz (about 25 us), so the FX2 sees a short gap.
- `gains` replies `OK gains max=<index> stages=<first index of each RF stage>`.
- `EVT phy state=<state> cause=<modem|txq|gain|p11>` when the once-a-second
  watchdog finds the receive lock drifted and restores it; `EVT phy
  state=<state>` once after boot.
- A line longer than 120 characters gets `ERR line syntax line longer than
  120 characters`. Commands received while the radio head is still booting
  get `ERR <command> state booting`. Empty lines get no reply.

## 2. FX2LP streaming firmware (USB)

### Identity

- Bootloader (blank chip, before firmware upload): any of `04b4:8613`
  (Cypress default), `04b4:1004`, `0925:3881` (EEPROM-preloaded logic
  analyzer boards), `1d50:608c` (fx2lafw). The host uploads firmware with
  the chip's built-in vendor request `0xA0`: write `0x01` to CPUCS
  (`0xE600`), load RAM, write `0x00` to CPUCS.
- OpenVRX firmware: VID `0x1209`, PID `0x0001` (pid.codes test range),
  manufacturer `OpenVRX`, product `OpenVRX FX2`, serial number = 8 hex digits
  derived from a per-board ID (EEPROM bytes if present, otherwise random at
  first upload and passed in by the host through the `0xBC` request).
  Microsoft OS 2.0 descriptors (BOS + MS OS 2.0 descriptor set, vendor
  code `0xEE`) declare `WINUSB` compatible ID, so Windows binds WinUSB
  without Zadig. Device interface GUID
  `{5C2B2B1E-7D5A-4C1B-9E0B-50435652580A}`.

### Endpoints (interface 0, alternate setting 0)

| EP | Type | Size | Use |
|---|---|---|---|
| EP2 IN (`0x82`) | bulk | 512 | sample stream, quad buffered, AUTOIN |
| EP1 IN (`0x81`) | interrupt | 64 | status block (section 2.4), every 8 ms (bInterval 7 at high speed) |

### Vendor requests

All on the device recipient. `bmRequestType` `0x40` (OUT) or `0xC0` (IN).

| bRequest | Dir | wValue | wIndex | Data | Meaning |
|---|---|---|---|---|---|
| `0xB0` | IN | 0 | 0 | 16 B | `GET_INFO`: magic `"PCVX"`, u8 proto=1, u8 fw_major, u8 fw_minor, u8 fw_patch, u32 features, u32 board_id |
| `0xB1` | OUT | flags | watch_n | - | `START` streaming (flags below) |
| `0xB2` | OUT | 0 | 0 | - | `STOP` streaming, flush FIFOs |
| `0xB3` | IN | 0 | 0 | 32 B | `GET_STATUS` (same block as EP1) |
| `0xB4` | OUT | 0 | 0 | - | `RESET_COUNTERS` |
| `0xB5` | OUT | watch_n | 0 | - | `SET_WATCH`: 0 or 1 = full stream, N>1 = commit 1 packet in N |
| `0xB6` | OUT | 0/1 | 0 | - | `LED` identify blink off/on (boards with an LED on PA0/PA1) |
| `0xBA` | IN | address | 0 | n B | `EEPROM_READ` (I2C 24Cxx at 0x51, two-byte addressing; 0x50 one-byte) |
| `0xBB` | OUT | address | 0 | n B | `EEPROM_WRITE` (page writes handled by firmware) |
| `0xBC` | OUT | id_lo | id_hi | - | `SET_BOARD_ID` (RAM only; used for the serial string) |

`START` flags (wValue bits):

- bit 0 `WIDTH16`: 16-bit FIFO (WORDWIDE=1); 0 = 8-bit.
- bit 1 `IFCLK_INV`: invert IFCLK polarity (IFCONFIG.IFCLKPOL).
- bit 2 `INTERNAL_CLK`: throughput test: use the internal IFCLK and stream
  whatever the port pins read (no C5 needed).
- bit 3 `INTERNAL_48`: with bit 2: 48 MHz internal IFCLK (else 30 MHz).
- bit 4 `WATCH`: start in watch mode with ratio wIndex.
- bit 5 `TEST_PATTERN`: with bit 2: firmware fills packets with a 16-bit
  counter instead of port data (end-to-end loss check at full USB rate,
  slower than the slave FIFO; reports its own rate).

### Status block (32 bytes, little endian)

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `"PCVS"` |
| 4 | u8 | proto version (1) |
| 5 | u8 | flags: bit0 running, bit1 width16, bit2 ifclk_inv, bit3 internal_clk, bit4 watch, bit5 ifclk_active, bit6 test_pattern |
| 6 | u16 | watch_n |
| 8 | u32 | packets committed (counted in watch mode and test-pattern mode; 0 in AUTOIN mode) |
| 12 | u32 | packets skipped (watch mode) |
| 16 | u32 | FIFO full events: polls that found the EP2 FIFO full while running (each one means samples may have been dropped) |
| 20 | u32 | uptime ms |
| 24 | u16 | status sequence number |
| 26 | u16 | firmware version (major << 8 | minor) |
| 28 | u32 | reserved (0) |

### Additions (FX2 firmware 0.1.0)

These extend the requests above without changing them.

- `0xB7` `SET_OPTION` (OUT, wValue = key, wIndex = value, no data). Unknown
  key or value: STALL. Options last until the FX2 is reset.
  - key 1 `WATCH_DETECT`: 0 (default) = a packet is ready when
    EP2FIFOBC/packet_size minus NPAK is above 0; 1 = byte count only.
  - key 2 `SLWR_ACTIVE_HIGH`: 0 (default) active low, 1 active high.
    Applied at the next `START`.
  - key 3 `STATUS_INTERVAL_MS`: 1..255, default 8.
  - key 4 `PATTERN_FILL`: 0 (default) the CPU writes every byte of each
    test-pattern packet; 1 = after the first four packets only the high bytes
    are rewritten (faster; relies on the EP2 buffers keeping their contents).
- `EEPROM_WRITE` wIndex low byte: log2 of the EEPROM page size, 0 = default
  (8 bytes at 0x50, 32 bytes at 0x51). `EEPROM_READ` and `EEPROM_WRITE`
  STALL when no EEPROM answered.
- `START` with `TEST_PATTERN` always uses the internal clock (bit 2 implied).
  `WATCH` with wIndex 0 or 1 starts a full stream. `START` STALLs while the
  device is not configured.
- Test pattern stream: consecutive u16 little-endian words, each one more
  than the previous (mod 65536), starting at 0 after `START`, continuous
  across packets. Each packet is counted in "packets committed".
- Board ID record: 8 bytes `'P' 'C' 'I' 'D'` + u32 board_id (little endian),
  valid unless the ID is 0 or 0xFFFFFFFF. Stored in the EEPROM at 0x00F8
  (one-byte address chip at 0x50) or 0x3FF8 (two-byte address chip at 0x51;
  the last 8 bytes of a 16 KB 24LC128). The host may also write the same
  record into FX2 RAM at 0x3FF0 with the `0xA0` request after loading the
  image and before releasing CPUCS; the firmware then reports that ID from
  the first enumeration (serial number) and clears the record. Priority:
  EEPROM record, then RAM preset; `SET_BOARD_ID` overrides both.
- `GET_INFO` feature bits: 0 watch, 1 internal clock test, 2 test pattern,
  3 `SET_OPTION`, 4 EEPROM present, 5 EEPROM uses two-byte addressing,
  6 ID from EEPROM, 7 ID from RAM preset, 8 ID from `SET_BOARD_ID`,
  9 connected at high speed.
- Status block: `ifclk_active` (flag bit 5) means the EP2 FIFO byte count
  changed, or EP2 sent a packet, during the last status interval while
  running. The sequence number counts EP1 blocks; `GET_STATUS` returns the
  number of the last EP1 block.
- At full speed EP2 packets are 64 bytes (`AUTOINLEN` 64) and EP1 uses
  bInterval 8 (8 ms).

## 3. Host behaviour

- Firmware upload on plug-in: the host finds bootloader-ID devices, uploads
  the embedded `openvrx-fx2.ihx`, waits for the OpenVRX device to appear
  (up to 5 s), matches it by the board ID it assigned.
- Streaming: 8 to 32 bulk IN transfers of 16 to 64 KiB in flight on EP2.
- Phase calibration (sampling-phase lottery): `clk 6` briefly, capture
  bursts, compute the fourth-difference probe score per 512-byte packet,
  `slip` and retry while the score says mixed bits; then set the working
  divider. Re-check on the live stream and after any C5 reset.
