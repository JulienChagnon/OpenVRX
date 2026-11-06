# 2026-11-05: pit-mode ladder with the ESP32 on its battery (did not reach the noise floor)

## Setup

- Chain as `colour-ladder-231647` (BENCH-10): VTX in pit mode, 40 + 30 + 20 dB fixed, step attenuator,
  C5 on R7 5880, 4x4top clk 9, gain loop on, ESP32 colour card (`card`, ladder output, 1600 mV).
- New: the ESP32 on its battery over Wi-Fi (`socket://openvrx-bench.local:2323`), not PC USB (BENCH-11).
- VTX reported 80 C in pit mode before the run.
- C5 fw 0.1.0, about 3 minutes after power-up at the first `status`: `i_dc=8 q_dc=109 trim=-15/-96`
  (Q trim at its limit, C5-1).

## Runs

First two tries stopped before any step (no attenuator presses): `no reply to 'agc on'` from the C5,
then `ERR vtx no answer from the VTX` on `vtx active 0` (BENCH-12). After the bench.py retries:

```
python3 tools/bench/bench.py --pit --live --fixed-db 90 \
    --att esp32:socket://openvrx-bench.local:2323 --att-at 0 \
    --c5 /dev/ttyACM0 --gains auto --steps 0:20:1 \
    --pattern card --video-amp 1600 \
    --ref ~/bench-runs/colour-225835/c1600 --ok-psnr 20 \
    --lanes 4x4top --clk 9 --mhz 5880 --seconds 2 --keep \
    --out ~/bench-runs/pit-ladder-battery
```

`vtx active 0` missed three times; the VTX already reported `pit=1`, so the run carried on.

## Numbers

| step dB | 0 | 4 | 7 | 8 | 12 | 16 | 20 |
|---|---|---|---|---|---|---|---|
| C/N dB | 10.7 | 9.9 | 9.7 | 10.3 | 10.0 | 9.8 | 9.4 |
| power_db | 44.1 | 43.5 | 43.3 | 44.0 | 43.7 | 43.5 | 43.2 |
| PSNR median | 23.9 | 23.3 | 22.9 | 23.7 | 23.3 | 23.1 | 22.7 |

C5 gain 83 on every step; Q trim walked from -73 to -96 by step 8 and stayed; q_dc -14 to +30.
The 231647 ladder, same chain with the ESP32 on USB: C/N 11.1 at step 0 down to 0.3 at step 20.

## Observations

- The level at the C5 hardly followed the attenuator: 0.9 dB of power for 20 dB of steps. Step 0
  matches the old ladder (10.7 against 11.1 dB C/N), so the conducted signal is there at the start.
- At 8 dB (7.75 to 8: the 8 dB bit on, the lower bits off) power and C/N rose 0.7 dB: the attenuator
  most likely did step, and another path now carries about as much signal as the pads. To check: the
  attenuator LEDs after the run (20 dB = 16 + 4), and what changed in the setup since 231647 (ESP32
  and battery placement, its wires out of the tin, the lid). Leak check without presses: chain open
  at the attenuator input with the attenuator still at 20 dB, C5 `status`.
- Live view: the app logged "video lost" 60 times, 230 to 250 of 262 lines per field missing, with
  only about 11 lines per field reaching the video pipeline (`lines_in` 643 per 60 fields). The
  recordings are complete and `openvrx-cli decode --live` replays them with 8 missing lines in 118
  fields, at 2.2 s of CPU for 2 s of signal (DSP and low_latency video together). So the lines were
  lost inside the app between the DSP thread and the video thread (DSP-5).
- The compare pane drew the old monochrome DAC card while the ESP32 sends the colour ladder card.

## Changes made

- bench.py: `--pit`, `--ref`, `--ok-psnr`, `--demod`, `--preset`; console retries (BENCH-12); `--live`
  passes the ESP32's video output to the app.
- App: `--compare-out ladder|dac` (default ladder) and the colour card and colour bars of
  `ladder.cpp` (`BenchColourCard`, `BenchColourBars`, test `colour_card_matches_the_ladder`); the
  DSP thread logs lines dropped because the video queue was full (with the DSP's lines_out and load)
  every 5 s, and "video lost" adds the pipeline's average stage time per field.
