# Wired bench: attenuation sweeps

The bench feeds the quad's VTX into the C5 through coax and attenuators
instead of the air, so a signal level can be set, repeated and swept. The
result of a sweep is the weakest level (dBm at the C5) that still gives a
picture, per gain, layout and DSP setting. That number is what long range
depends on, and it is how changes to OpenVRX get compared.

## Parts

| Part | Use |
|---|---|
| Fixed SMA attenuators 20, 30, 40 dB (2 W) | bring the VTX down to receiver levels |
| AW8637 step attenuator, 0 to 31.75 dB in 0.25 dB steps | the sweep |
| SMA female to MMCX pigtail | VTX antenna port to SMA (use the pigtail that fits your VTX) |
| SMA male-male jumpers, 20 and 100 cm | between the attenuators |
| U.FL to SMA female pigtail | into the XIAO C5's U.FL antenna port |
| 50 ohm SMA dummy loads | cap the VTX whenever it is powered and not in the chain |
| Optional: ESP32 board + 2 relay channels (or photorelays, or NPN transistors) | presses the step attenuator's buttons (automated sweeps) |
| Optional: metal tin | shields the VTX or the C5 (see leakage) |

## Chain

```
quad VTX antenna port -> pigtail -> 40 dB -> 30 dB -> step attenuator -> jumper -> U.FL pigtail -> C5
```

The step attenuator goes after the fixed ones: it takes at most +24 dBm in.
Its power is plain 5 V USB-C (a PC port or a basic charger), never a PD
trigger board or a fast charger. It has no DC block; nothing in this chain
carries DC.

Level at the C5, with the quad's VTX at 25 mW (14 dBm) and about 3 dB of
cable and insertion loss:

| Fixed | Step 0 dB | Step 31.75 dB | Use |
|---|---|---|---|
| 40 + 30 | about -59 dBm | about -91 dBm | start here: clean picture down towards threshold |
| 40 + 20 | about -49 dBm | about -81 dBm | strong signal and overload tests |
| 40 + 30 + 20 | about -79 dBm | about -111 dBm | long range: the threshold region |

Use all three whenever the VTX runs above 25 mW.

### Safety

- Never power the VTX without its antenna port connected to the chain or a
  dummy load.
- Lowest VTX power (25 mW) unless a test needs more, and then add the 20 dB.
- On the bench the quad has no airflow: keep runs short and check the VTX
  temperature by touch between sweeps.

### Leakage

At -90 dBm and below, signal leaking from the VTX through the air can be
stronger than what comes through the coax, and the results then look better
than they are. Keep the quad and the C5 as far apart as the cables allow,
put one of them in a closed metal tin with only the coax coming out, and
take the quad's antenna off (the pigtail replaces it). `bench.py` checks
this: when 8 dB or more of extra attenuation drops the received power by
less than half that, it prints `LEAK?` on that step.

## The source picture

The quad's own camera, on the bench, not flying. Point it at a fixed scene
with detail (a printed chart, a bookshelf) and do not move it during a
sweep. The scorer does not need a known test picture: the first,
strongest step of each sweep is the reference, and every other step is
scored by PSNR against the reference's mean frame. The OSD timer and
camera noise keep the reference's own PSNR finite (about 20 to 25 dB);
`bench.py` calls the picture lost when PSNR falls more than 3 dB under the
reference's or fewer than half its frames decode.

## Running a sweep

Build the tools once (`cargo build --release -p openvrx-cli` in `host`),
set the C5 once with `tools/c5cmd.py` (`tune`, `lanes`, `clk`, `gain`,
`save`), put the C5 on its own power, plug in the FX2. A fixed-gain sweep
needs the C5's gain loop off (`agc off`, then `save`); with it on, the
sweep measures the whole receiver as it flies (`--gains auto`).

### Manual attenuator (one USB port, no extra parts)

```
python tools/bench/bench.py --fixed-db 70 --vtx-dbm 14 --att manual \
    --steps 0:31:2 --lanes 4x4top --clk 9 --mhz 5880 --gains 56 \
    --out D:\FPV\OPENVRX\bench\first-sweep
```

At each step the script asks for an attenuation; set it with the buttons
(+ and - together picks the step size; add the lit LEDs to read the value)
and press Enter. `--gains` is only a label here: the C5 keeps the gain it
saved. To compare gains, re-save the C5's gain and run again.

### Automated

With the C5's USB and the FX2 on the same PC (the Fedora bench host, or a
hub), the script also sets the gain:

```
python3 tools/bench/bench.py --fixed-db 70 --att esp32:/dev/ttyUSB0 \
    --c5 /dev/ttyACM0 --gains auto,48,56,64,72 --steps 0:31.75:1 \
    --lanes 4x4top --clk 9 --mhz 5880
```

### Output

The run folder holds `run.json` (the arguments), `results.jsonl` and
`results.csv` (one row per step: estimated dBm at the C5, frames, PSNR,
C/N, clipping, received power, ok/lost) and `summary.json` (weakest level
with a picture, per gain). Recordings are deleted after scoring except the
reference of each gain; `--keep` keeps them all (27 MB/s). Run folders
stay local, like every recording.

## Step attenuator control (optional)

The AW8637 has no PC interface, only its buttons. `tools/bench/att_ctl`
is a small Arduino sketch for any ESP32 that presses them:

1. One relay on the **+** button (the bench's setup; a second relay on -
   is optional, see below). A button is two contacts that touch when
   pressed; solder one wire to each (the AW8637's buttons have exactly two;
   on a 4-leg tactile switch use two diagonally opposite legs, since legs
   on the same side are always connected).
   Those two wires go to the relay's middle screw terminal (COM) and to NO,
   the end terminal that is open while the relay is off. NC stays empty.
   With NO and NC swapped the button is held down whenever the relay is
   off: the attenuator then changes value on its own.
2. Relay module inputs: VCC to the ESP32's **3V3** (not 5 V: on modules
   with a PNP driver, a 3.3 V high level cannot switch a 5 V module off),
   GND to GND, IN1 to GPIO32 (on a 38-pin ESP-32S board: left side,
   7th pin from the top, printed P32).
   Low-level trigger modules match `ACTIVE_LOW = true`; set it false for
   NPN transistors. GPIO32/33 keep the DAC pins (25, 26) free for a video
   pattern on the same ESP32. The attenuator needs no common ground with a
   relay; with transistors it does.
3. One click of + adds 0.25 dB. The seven LEDs show the value in binary,
   weights 0.25, 0.5, 1, 2, 4, 8, 16 dB (add the lit ones): 1 click =
   0.25, 2 = 0.5, 3 = 0.75 (0.5 + 0.25), 4 = 1 dB, 127 clicks = 31.75 dB.
   A 2 dB sweep step is 8 clicks, about a second at the default timing.
   `bench.py --att-step` defaults to 0.25. If + and - together selects a
   coarser step on your unit, a coarser step means fewer clicks; set it on
   the attenuator and pass the same value to `--att-step`.
4. Flash with the Arduino IDE or `arduino-cli` (board "ESP32 Dev Module").
5. Serial 115200: `step 0.25` (dB per click, the default), `at 0`
   (the value the LEDs show now, no presses), `set 12`, `get`,
   `press + 4`, `timing 60 80` (press and gap in ms; raise them if presses
   get missed).

With only the + relay the ESP32 can go up, never down: `set` to a lower
value answers `ERR set down`. `bench.py --att esp32:PORT` notices this by
itself (`get` reports `minus=no`): it asks you to set 0 dB by hand at the
start, sweeps upward on its own, and asks again whenever a lower value is
needed (the next gain). The attenuator does not wrap from 0 to 31.75, so a
single relay has to be on +, not -.

Optional second relay on -: IN1 to GPIO33 and `PIN_MINUS = 33` in the
sketch. Then `home` works (presses - until 0 dB) and sweeps run without
any hand steps.

There is no readback: after a sweep, check that the LEDs show what `get`
says. If they do not, raise `timing`.

## Standalone VTX and the ESP32 wiring

Instead of the quad, a bare VTX can be the source: an iFlight SucceX
Force 5.8 GHz 800 mW (30x30, pads VBAT, GND, RX, Video; 7 to 36 V in; IRC
Tramp control on the RX pad). The ESP32 that presses the attenuator can
also feed the VTX a test picture and set its channel and power.

ESP-32S NodeMCU, 38-pin board, USB at the bottom, positions counted from
the antenna end (on 30-pin boards wire by the printed labels):

| Goes to | ESP32 label | Side | Position | Notes |
|---|---|---|---|---|
| Relay module VCC | 3V3 | left | 1 | never 5 V/VIN (see the relay notes above) |
| Relay module IN1 (+ button) | P32 | left | 7 | |
| (free: optional - relay) | P33 | left | 8 | |
| VTX Video | P25 | left | 9 | DAC1: composite test pattern |
| Relay module GND | GND | left | 14 | any GND pin |
| VTX GND and the 12 V supply's GND | GND | right | 1 | one common ground |
| VTX RX pad (Tramp), through 1 kohm | P17 | right | 11 | UART2 TX |
| VTX RX pad (Tramp), direct | P16 | right | 12 | UART2 RX, also hears its own bytes |

- VTX VBAT: 12 V from a USB-C PD trigger board or a bench supply (9 V also
  works), never the ESP32. About 95 mA at 25 mW, 380 mA at 800 mW.
- Tramp is one wire both ways, 9600 baud 8N1. The 1 kohm resistor lets
  the VTX answer while P17 still drives the line. Measure the RX pad before
  wiring it (VTX powered, nothing connected): about 3.3 V is fine; about
  5 V needs a 10 kohm resistor in series with P16 as well, since ESP32
  inputs are not 5 V tolerant.
- The ESP32 is powered by its own USB cable; its 5V/VIN pin stays unused.
- RF first: never power the VTX without the attenuator chain (40 dB fixed
  first, the step attenuator takes at most +24 dBm) or a dummy load on its
  antenna port. It may start at the last power used, up to 800 mW.
  Start at 25 mW or pit mode; above 200 mW give it airflow.
- `att_ctl` runs the attenuator, the test pattern (below) and the VTX's
  Tramp control: `vtx` (channel, power, pit mode, limits, temperature),
  `vtx ch R8` or `vtx mhz 5917`, `vtx mw 25` (above 200 mW only with
  `force`), `vtx pit on|off`. Each set is read back to confirm it.
  `bench.py --vtx-ch R8 --vtx-mw 25` sets the VTX before a sweep, and the
  reported power sets `--vtx-dbm` when that is not given.
- Pit-mode guard: the VTX stays in pit mode unless a bench run holds a
  lease. Every 5 s the ESP32 checks it (from boot, so a VTX powered later
  is caught too). `bench.py` takes the lease (`vtx active 120`), renews it
  every step, waits 2 s after leaving pit mode, and ends it when the run
  ends (`vtx active 0`); a crashed run simply lets it run out. Long hand
  steps let it lapse too, so the VTX cools meanwhile. `vtx` shows the
  temperature the VTX reports; nothing acts on it (no thermal cut-off for
  now). To transmit by hand, `vtx active 600`. Through 70 dB of pads a pit-mode VTX is below the
  receiver's noise (no lock). Without Tramp
  (or before checking the RX pad voltage), set the VTX by its buttons:
  the band, channel and power LEDs blink their numbers (bands 1 to 5 =
  A, B, E, F, R; power 1 = lowest).

### Colour test pattern on a resistor ladder (default)

`att_ctl` drives a 6-bit binary-weighted ladder through I2S0 in parallel
(LCD) mode at 4 x fsc (14.318 MHz), so the test picture has a colour burst
and full NTSC detail. Wiring, all into one VIDEO node that goes to the VTX's
video pad (the old P25 wire, moved):

| ESP32 pin | Bit | Resistor to VIDEO |
|---|---|---|
| P18 | 0 | 8.2 kohm |
| P19 | 1 | 3.9 kohm |
| P21 | 2 | 2.0 kohm |
| P22 | 3 | 1.0 kohm |
| P23 | 4 | 470 ohm |
| P26 | 5 | 240 ohm |

VIDEO also gets 200 ohm and 470 pF to the shared GND. P25 stays free.

- `out ladder` (default, saved) or `out dac` (the old monochrome P25 DAC,
  section below); both use I2S0, so only one runs.
- Patterns: `card` is 75 % colour bars, a multiburst (0.5, 1, 2, 3, 4 MHz),
  a ramp and flat grey; `bars` is full-height bars; the others as below.
- `amp` defaults to 1600 on the ladder: the decoder's deviation_scale reads
  1.00 into this VTX (the ladder's swing depends on the VTX input's load).
- 6 bits give about 50 levels from black to white: a ramp shows steps.

### Test pattern on P25

`att_ctl` also feeds the VTX's video input: P25 (DAC1) carries a 525-line
interlaced monochrome NTSC picture from boot (pattern `card`, 1 Vpp). DMA
replays one frame, so relay presses never disturb it.

- Commands, on the same 115200 baud console as the attenuator:
  `pattern card|bars|ramp|check|grey|black|white`, `amp <mV>` (sync tip to
  white, 200 to 3200, default 1000), `video on|off`, and `vget` (state, plus
  the line rate counted from the DMA interrupts; takes 1 s, reads in steps
  of about 15 Hz around 15734). A change restarts the picture and the
  receiver relocks.
- `card`: 8 grey steps, stripes at 1, 0.5, 0.25 and 0.13 MHz, a ramp and a
  checkerboard. `grey` (flat 50 IRE) suits noise and C/N readings.
- Limits: 2.014 MS/s (128 samples per line; the DAC driver warns of
  distortion above about 2 MHz), so about 1 MHz of picture detail and no
  colour (a burst needs 3.58 MHz). It is a repeatable, static source for
  lock, C/N and PSNR; colour and fine detail still need the camera (or the
  MS2107 capture as ground truth).
- If the picture is dim or will not lock, the VTX input may be loading the
  DAC: try `amp 2000`.
- Watch it live: `bench.py --live` runs the OpenVRX app with its window
  open for the whole sweep and records each step through it (the FX2 has
  one owner); key X (or `--pattern`, which starts the app in it) shows the
  received picture beside the card as the ESP32 sends it. App output goes
  to `app.log` in the run folder.
- `bench.py --pattern NAME` (and `--video-amp`) sets it before a sweep; the
  pattern and measured line rate go into run.json.
- Timing follows the simulator's NTSC layout (`host/crates/openvrx-sim/src/cvbs.rs`);
  `tools/bench/att_ctl/test/ntsc_frame_test.c` checks the frame on the PC
  (build line in the file).

## Bench host on Fedora

The laptop has one working USB port, so the automated bench runs on a
Fedora PC: `bash tools/bench/setup-fedora.sh` installs the build tools,
Rust, pyserial and udev rules for the FX2 and serial ports, and builds
`openvrx-cli`. Flashing the C5 there needs ESP-IDF v6.0.2 (Espressif's
Linux install guide), then `python3 tools/flash.py /dev/ttyACM0` as on
Windows.
