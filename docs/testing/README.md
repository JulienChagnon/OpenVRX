# OpenVRX bench testing log

Running record of hardware tests: what was wired, what was run, what came
out. Newest session at the bottom of "Sessions". Keep entries short and
factual; put raw console output and numbers in, not impressions only.

## Hardware on hand

| Item | Details |
|---|---|
| Radio head | ESP32-C5, Seeed XIAO pin map (`ver`: board=xiao, rev 1.0), console on COM9 |
| USB bridge | FX2LP (CY7C68013A) dev board, blank EEPROM, WinUSB via Zadig; OpenVRX firmware loads from the host (shows as `OpenVRX FX2 1209:0001`, board ID 0596E2B8) |
| Video source | drone with a 5.8 GHz analog VTX (channel/power: TBD) |
| PC | Windows 11 laptop, Iris Xe, one USB port |
| Bench wiring kit | not received yet; jumper wires for now |

## Fixed facts

- C5 firmware 0.1.0, ESP-IDF v6.0.2, flashed with `idf.py -p COM9 flash` in `firmware/c5`.
- C5 boot defaults: 5732 MHz, gain 64, `clk 18` (13.33 MHz), lanes 7x7, p11 on, filter cal.
  Nothing persists unless `save` is sent.
- FX2 control ties needed on every wiring: RDY1, PA4, PA5 to GND; RDY0, PA2, PA6 to 3.3 V
  (`docs/HARDWARE.md`). Without them the throughput test reads 0 MB/s.
- USB selective suspend is on (AC and DC); disable it before long captures
  (`fx2_probe` prints the `powercfg` commands).

## Layout test matrix

Fill in as each layout is wired and tried. Rate = bytes/s needed through the FX2.

| Layout | Word | Clock (`clk`) | FX2 rate | Wired | Selftest walk | Throughput OK | Picture | C/N at breakup | Notes |
|---|---|---|---|---|---|---|---|---|---|
| 4x4top | 8-bit | 6 (40 MS/s) | 40 MB/s | | | | | | first choice for jumper wires |
| 4x4fine | 8-bit | 6 | 40 MB/s | | | | | | same wiring as 4x4top |
| 4x4ultra | 8-bit | 6 | 40 MB/s | | | | | | same wiring as 4x4top |
| 4x4top | 8-bit | 9 (26.67 MS/s) | 26.7 MB/s | yes (2026-10-30) | lanes toggle (0xBD readback) | 26.68 MB/s | clean offline; live drops (DSP too slow) | | first layout under test; saved on C5 with R7 |
| 5x5 | 16-bit | 18 (13.33 MS/s) | 26.7 MB/s | | | | | | front pins only |
| 6x6 | 16-bit | 18 | 26.7 MB/s | | | | | | needs XIAO back pads |
| 7x7 | 16-bit | 18 | 26.7 MB/s | | | | | | needs XIAO back pads |
| 7x7 | 16-bit | 12 (20 MS/s) | 40 MB/s | | | | | | |

Comparison method: same drone, same channel, same placement, record each
layout with `R` in the app (or the CLI), then step VTX power or distance
down until breakup and note the C/N shown on the OSD bars.

## Open questions to answer on the bench

- Does the FX2 sustain 40 MB/s on this laptop's port without `usb_gap` events?
- Real gain table step size (the app assumes 0.5 dB per index).
- Do gain steps applied mid-field cause visible flicker (they are not yet held to vertical blanking)?
- Which sampling-phase ("lottery") state shows up at each clock, and does `slip` clear it?

## Known software gaps at the time of first test

- openvrx-dsp: polar and LUT demods work; PLL and EKF demods fail inside the chain (bug under diagnosis). Use polar/LUT.
- openvrx-sim runs at about 0.3x real time (only matters for simulated tests).
- App uses eframe: about 115 presents/s cap, no monitor picker (drag to HD3, F11).
- App default is FX2-only control (C5 runs its saved settings, powered from the FX2 board 3.3V to XIAO 3V3).
  Start it with `--lanes`/`--clk` matching the C5's saved state. No channel/gain changes, no phase
  calibration or lane self-test in this mode; `--control c5` restores console control.
- Sampling-phase lottery cannot be fixed in FX2-only mode (needs `slip` over the console).

## Sessions

### 2026-10-30: first bring-up

- Flashed C5 on COM9. `ver`: `fw=0.1.0 proto=1 board=xiao idf=v6.0.2 chip=esp32c5 rev=1.0`.
- `status`: `mhz=5732 gain=64 clk=18 clk_on=1 lanes=7x7 p11=1 filter=cal bw=40 phy=ok hz=13333333`.
- `pins` (7x7): `clk=G10 q=G1,G0,G25,G7,G23,G2,G3 i=G24,G11,G12,G8,G9,G4,G5`.
- Earlier the same day: FX2 firmware upload and re-enumeration OK, high speed; throughput 0 MB/s because the control pins were floating (not yet tied).
- C5 hung twice after console sessions: my script opened COM9 with pySerial defaults (DTR/RTS asserted),
  closing reset the chip into a ROM loader that hangs while MODEM_DIAG runs (the C5VRX #275 issue).
  Fixed with `tools/c5cmd.py` and `tools/flash.py` (DTR/RTS low, `dl` before flashing). The app's
  serial code already opened with DTR/RTS low. Recovery from the hang: unplug all power.
- Added FX2-only control mode (default) to the app.
- C5 configured and saved: `tune 5880` (R7), `lanes 4x4top`, `clk 9`. Reply pins: clk=G10,
  q=G23,G24,G11,G12, i=G1,G0,G25,G7. Wired per HARDWARE.md XIAO 8-bit table; RDY1/PA4/PA5 to GND done,
  RDY0/PA2/PA6 to 3.3 V pending.
- Console `reboot` hung the C5 (SW_CPU reset with the modem writer running). Firmware fixed: `rf_reboot`
  stops the writer and Wi-Fi first. Verified: reboot comes back with the saved state.
- `tools/flash.py` missed the `dl` acknowledgement (lost in USB re-enumeration) although the C5 was in the
  loader; it now probes the loader with esptool before giving up.
- FX2 + C5 wired (4x4top), C5 powered from FX2 VCC. USB path OK: test pattern 2.91 MB/s, 0 errors.
  Slave FIFO: 0 bytes in every mode (internal/C5 clock, 8/16-bit, SLWR either polarity, watch).
  New FX2 request 0xBD GET_PINS + `fx2_diag` example: RDY1, PA4, PA5 read HIGH (want low); RDY0, PA2,
  PA6 high (ok); PB0..7 all high, not toggling. Reading: the jumpers are not on their pins (header
  row/column offset) and/or the C5 is unpowered. `fx2_diag -- --live` added for re-seating wires.
- After the GND link: PA4/PA5/PA2/PA6 correct; RDY1 reads high and RDY0 low although wired RDY1->GND,
  RDY0->VCC by the labels. With SLWR set active-high the FIFO streamed **40.88 MB/s 8-bit** (FIFO
  full throughout: this laptop port's USB limit is about 41 MB/s; clk 9 comfortable, clk 6 marginal).
  FX2 dropped off USB when the test switched to the C5 clock. PB still all high (C5 not driving).
  Watch mode committed only 4 packets with the FIFO full: suspected FX2 firmware watch-mode bug (later).
- Swapped the RDY wires (board labels RDY0/RDY1 reversed; HARDWARE.md updated: label RDY1 -> VCC,
  label RDY0 -> GND). All six control ties read correct. Slave FIFO internal 48 MHz: 40.3 to 41.0 MB/s
  in 8- and 16-bit (USB limit, FIFO full). GND link between boards had been loose; fixed.
- C5 powered from FX2 VCC -> XIAO 3V3: FX2 dropped off USB twice mid-test, PB lanes never toggle, no
  IFCLK. Suspected supply droop from the C5's RF current spikes resetting both boards. Next: power the
  XIAO from its own USB-C supply, keep the GND wires.
- C5 on a power bank (own supply), FX2 on the laptop: both stay up. Internal 48 MHz 8-bit: 47.6 MB/s.
  Boot guard added to the C5 firmware (RWDT during bring-up, RTC breadcrumbs, safe mode, `boot=` in
  status) after one cold boot froze; the next cold boot was fine (`boot=ok`).
- Lanes verified with the 0xBD readback: PB3..0 (Q, D4..D7) and PB7..4 (I, D0..D3) toggle with receiver
  noise (no VTX; 4x4top top bits only move when noise crosses zero, I has a small DC offset).
- C5 clock verified: D10 moved onto FX2 PD0 reads TOGGLING. No FIFO writes with the C5 clock on the pin
  labelled IFCLK (nor CLK, which is CLKOUT, an output: do not use). Open issue: which header pin is the
  real IFCLK on this board (labels already wrong for RDY0/RDY1). Waiting on a photo of the header.
- IFCLK label is correct: new FX2 option IFCLK_OUT + `fx2_diag --find-ifclk` (output switched on/off,
  read back on PD1) proved the "IFCLK" header pin is the chip's IFCLK.
- Root cause of "no C5 clock": the FX2 firmware's **watch mode jams the EP2 USB side** (NPAK 3, one full
  packet uncommitted, nothing sent) and the jam persisted into every later diag case. On a fresh FX2,
  **C5 clock 8-bit streams 26.68 MB/s** (= 240 MHz / 9), normal and inverted IFCLK. Watch mode to fix.
- **Layout 4x4top @ clk 9: stream verified end to end.**
- **First live video (drone on R7):** app picture broken into bands. Recordings
  `openvrx-20261031-023504` / `-023517` (5 s each) hold only about 0.7 s of data: 522 `usb_gap`
  events, one 32 KB block kept then 0.7 to 1 MB dropped. Cause: RxDsp runs at about 47 ns/sample
  (about 21 MS/s on one core) against 26.67 MS/s, so the block channel overflows. Not the radio.
- Offline decode (`openvrx-cli inspect/decode`): DSP locks NTSC, line rate 15733 Hz. The sync-free FM
  strip shows the drone OSD sharp and readable (BATT, current, RATE, timer): **4x4top gives a clean
  picture**. Signal very strong: 85 % of I and Q samples at full scale at gain 64 (drone close).
  Carrier offset about -1.6 MHz.
- Next: DSP throughput (agent on it). Workaround until then: clk 18 (13.33 MS/s, 75 ns budget).
- Lab (lab/RECOMMENDATION.md): reference decoder reassembles the gappy captures into complete sharp
  colour frames (line rate 15733.3 Hz, 0H jitter 12.5 to 15 ns, flat SNR 27.5 to 30 dB). Remaining
  noise is clipping: lower gain about 6 dB when close. Ranked layouts: 4x4top clk 9 (lower gain),
  4x4fine clk 9, 7x7 clk 12, 7x7 clk 18, 4x4ultra clk 9, 4x4 clk 6, 6x6, 5x5. A second receiver
  (selection about 6 dB, MRC about 7 to 8 dB fade margin) beats any layout change.
- Next: tie FX2 control pins, wire 4x4 on XIAO front pins, rerun `fx2_probe`, `selftest walk`, first recording.
- DSP speedup (about 14 ns/sample, 2-thread pipeline): recordings `-163724` (16 s) and `-163750` (10 s)
  have 0 gaps. Picture still came and went. Level per second (4x4top, two's complement nibbles):
  between near silence (rms 1.1) and full scale (97 %) within seconds, at fixed gain 64.
- Range test at gain 56, recording `-214743` (52 s, 0 gaps): close in, 99.8 % of samples at full scale;
  walking out, rms falls to 0.7 (most samples 0 or -1, signal under one 4x4top step) and stays there
  for about 30 s: the picture turns to lines. **A fixed gain with 4 bits covers about 24 dB, much less
  than near to far.** Needs gain control on the C5 (FX2-only has no PC link). Note: in FX2-only mode
  the recording metadata `gain` field shows the app default (64), not the C5's saved gain.
- Nov 2 indoor run, recording `-235032` (33 s, 3 usb_gaps of 32 KB at 14.5 s), 4x4top clk 9, fixed gain:
  0 to 5 s 99 % of samples clipped, 12 to 26 s under one step. The decoded picture of the clipped part
  has fine rainbow stripes in dark and textured areas; an unclipped Oct 31 capture (rms about 6 steps)
  decodes clean. Per-axis clipping of 4-bit I/Q leaves only a few phase steps, which land in the colour
  band. Lower gain close in, or the C5 gain loop. The app OSD now warns on overload (30 % clipped) and on
  a signal under one ADC step.
- OSD twitter: between fields the camera picture is placed correctly (no alternating shift) but the
  flight controller OSD jumps exactly one frame row: its chip counts field lines from each vertical sync
  and ignores the second field's half-line offset. New deinterlace `line-double` (app: Steady OSD, key I,
  default on) puts the bottom field one field line lower, as goggles do: OSD shift 0.0 rows on both
  recordings, the camera picture moves one row per field instead. `openvrx-cli decode ... --steady-osd`.
- Nov 3: C5 gain loop (`agc`) flashed on the XIAO (COM9). The old image used the pre-rename settings
  namespace, so the board came up on defaults; re-set 4x4top clk 9 5880 MHz, `agc on`, saved, survives a
  reboot. No VTX on: the loop raised the gain from 56 to the table top (83) in about 2 s; the noise floor
  there reads rms 1.2 to 2.3 steps, clip 0.2 to 2.6 % (gain 56 put the same noise under one step).
- Nov 4, Fedora bench host: standalone SucceX Force VTX fed by the ESP32's NTSC test pattern (att_ctl,
  P25, `card`, 1 Vpp), VTX LEDs band 5 / channel 8 / power 1, so R8 (5917 MHz), lowest power; 70 dB
  fixed plus 2 dB step, about -61 dBm at the C5. The receiver locks NTSC at 15734.5 Hz line rate, VTX
  about 1.6 MHz low. Gain loop on: right after `tune 5917` 43 % clipped at gain 79, settled at gain 70,
  clip 0.1 %, rms 3.9 within seconds. First recording (taken during the settle): C/N 7.4 dB, 3.4 %
  damaged, and a 1-sample sliver at every vertical edge: the ESP32 I2S sends each pair of 16-bit DAC
  slots swapped; fixed in video.cpp. After the fix: card clean (all four stripe zones down to 1 MHz),
  C/N 17.2 dB, 0 % damaged, 6 % clipped (I/Q DC offsets about +128/+110 of 512 push one side into
  clipping). Open: 10 relocks in 3 s with the loop on and 25 (2412 coasted lines) with it off at gain
  70 and 0.03 % clipped, while the simulator's ideal NTSC through the same decode gives 0. Field rate
  60.055 Hz is the last field's 262 lines, not a fault.
