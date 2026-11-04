# 2026-11-04: first light with the ESP32 test pattern (Fedora bench host)

## Setup

- Host: Fedora bench PC (Atlas), repo moved from the Windows laptop; `openvrx-cli` built here.
- Source: iFlight SucceX Force VTX, standalone, 12 V. Video from the ESP32 (`att_ctl`, P25, pattern
  `card`, 1 Vpp, 525-line NTSC, monochrome, 2.014 MS/s).
- VTX LEDs: band 5, channel 8, power 1 = R8 (5917 MHz), lowest power. Confirmed by the receiver: locks at
  5917 MHz, carrier about 1.6 MHz low.
- RF: VTX, 40 dB + 30 dB fixed, step attenuator at 2 dB; about 14 - 70 - 3 - 2 = -61 dBm at the C5
  (if the lowest power is 25 mW).
- C5: XIAO, fw 0.1.0 (IDF v6.0.2), 4x4top, clk 9 (26.67 MS/s), gain loop on, tuned 5917 (not saved).
- FX2: blank at plug-in; `openvrx-cli record` uploads the firmware.

## Runs (results in `bench-runs/first-check-134757/`)

| Run | Condition | Lock | C/N dB | Clip % | Damaged | Relocks | Coasted |
|---|---|---|---|---|---|---|---|
| r8 | right after `tune 5917`, loop settling; ESP32 pair-swap bug present | Locked NTSC 15734.5 Hz | 7.36 | 11.2 | 3.4 % | 10 | 438 |
| r8b | pair swap fixed, loop on (gain about 70 to 72) | Locked | 17.23 | 6.0 | 0 % | 10 | 58 |
| r8c | loop off, gain 70 | Locked | 18.75 | 0.03 | 0 % | 25 | 2412 |
| sim | simulator, ideal NTSC, 4x4top div 9, C/N 17 | Locked | 11.9 (est.) | 44 | | 0 | 79 |

Commands (per run): `openvrx-cli record BASE --lanes 4x4top --clk 9 --seconds 3 --mhz 5917 --gain 0`,
`openvrx-cli score BASE --frames 30 --skip 10`, `openvrx-cli decode BASE OUT 40`.

## Observations

- The gain loop works on a real carrier: 43 % clipped at gain 79 right after the tune, then gain 70,
  0.1 % clipped, rms 3.9.
- First decoded frame: every vertical edge had a 1-sample sliver of the wrong level. Cause: the ESP32's
  I2S outputs the two 16-bit DAC slots of each 32-bit word swapped (a b c d leaves as b a d c); every
  edge of the card falls inside a pair. Fixed in `att_ctl/video.cpp` (BENCH-2). r8 also overlapped the
  gain loop settling, so not all of its lower C/N is the swap.
- After the fix the card is clean: grey steps, all four stripe zones down to 1 MHz, ramp, checkerboard.
- I/Q DC offsets are large (C5-1).
- Relocks: 10 to 25 per 3 s on the bench, 0 in the simulator (DSP-1). The picture stays clean.
- `field_rate_hz` 60.055 is line rate over the last field's line count (262), not a fault.

## Tooling added the same day

- The OpenVRX app builds and runs on this Fedora host (Hyprland, Vulkan on the HD 530); live FX2 view
  works. New: key X / `--compare` shows the live picture beside the bench test card (`--compare-pattern`),
  and `--bench-control` takes `record FOLDER`, `stop`, `compare`, `quit` on stdin.
- `bench.py --live` records each step through the app, so the picture shows on screen during a sweep
  (tested: one 2 s step, locked, 15734.5 Hz, 0 % damaged).
- `att_ctl` Tramp control (`vtx ...`) and `bench.py --vtx-ch/--vtx-mhz/--vtx-mw`. RX pad measured 3.3 V
  (fine with P16 direct). The VTX answers: 5917 MHz, 25 mW, limits 5100 to 6000 MHz and 400 mW,
  temperature 80 (BENCH-4).
- Pit-mode guard in `att_ctl` (pit unless `vtx active` lease; no thermal cut-off) and the lease in
  bench.py. Checked at gain 70 fixed: pit mode, no lock (C/N -10 dB); leased, locked, C/N 16.9 dB.
  `power_db` read higher in pit mode than leased (DSP-2).

## Improvements raised

C5-1, C5-2, DSP-1 in [IMPROVEMENTS.md](IMPROVEMENTS.md); fixed and verified: BENCH-1, BENCH-2, BENCH-3.
