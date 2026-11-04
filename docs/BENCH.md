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
| Optional: ESP32 board + 2 photorelays or NPN transistors | presses the step attenuator's buttons (automated sweeps) |
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
`save`), put the C5 on its own power, plug in the FX2.

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
    --c5 /dev/ttyACM0 --gains 48,56,64,72 --steps 0:31.75:1 \
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

1. Two photorelays (or NPN transistors) with their outputs across the +
   and - buttons, inputs on GPIO26 (+) and GPIO27 (-), GND common with the
   attenuator. A 2-channel low-level-trigger relay module fits as is
   (`ACTIVE_LOW` in the sketch; set it false for NPN transistors).
2. Set the attenuator's step size to 0.25 dB by hand, once. It is
   remembered.
3. Flash with the Arduino IDE or `arduino-cli` (board "ESP32 Dev Module").
4. Serial 115200: `home` (presses - 130 times, so 0 dB from anywhere),
   `set 12.5`, `get`, `press + 4`, `timing 60 80` (press and gap in ms;
   raise them if presses get missed).

There is no readback: after a sweep, check that the LEDs show what `get`
says. If they do not, raise `timing`.

## Bench host on Fedora

The laptop has one working USB port, so the automated bench runs on a
Fedora PC: `bash tools/bench/setup-fedora.sh` installs the build tools,
Rust, pyserial and udev rules for the FX2 and serial ports, and builds
`openvrx-cli`. Flashing the C5 there needs ESP-IDF v6.0.2 (Espressif's
Linux install guide), then `python3 tools/flash.py /dev/ttyACM0` as on
Windows.
