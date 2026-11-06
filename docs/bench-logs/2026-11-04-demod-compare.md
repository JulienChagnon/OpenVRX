# 2026-11-04: FM demodulators and picture cleanup near threshold (offline)

Offline only: existing recordings, no hardware, no attenuator presses. Answers DSP-3 for this ladder.

## Setup

- Ladder: `bench-runs/colour-ladder-231647/a00` .. `a20` (step N = N dB extra attenuation), VTX in pit
  mode, 90 dB fixed, ESP32 colour test card `card` (75 % bars, multiburst 0.5 to 4 MHz, ramp, grey),
  NTSC, 4x4top, clk 9 (26.67 MS/s), 2 s per step.
- Reference: `bench-runs/colour-225835/c1600` (strong, clean, same card).
- Caveat: during this ladder the ESP32 (test pattern and Tramp) was powered from the PC's USB, not
  from its battery. That ties the VTX ground to the PC ground the receiver also uses: a possible RF
  leakage path around the attenuators (deep steps read stronger than they are) and ground-loop or USB
  noise on the DAC pattern (caps PSNR on every step). All variants decode the same samples, so the
  ranking and the dB differences between variants hold; the absolute attenuation of each threshold
  does not (BENCH-11).
- Host: commit 100e2bc plus the uncommitted working tree, `openvrx-cli` built on Atlas
  (`CARGO_TARGET_DIR=target/main cargo build --release -p openvrx-cli`).

## Tooling added

- `openvrx-cli score` takes `--demod polar|lut|ls|pll|ekf`, `--preset raw|low_latency|balanced|quality`
  and `--ref-demod D`. Defaults (polar, raw) give the same numbers as before. The reference goes
  through the same chain as the recording unless `--ref-demod` says otherwise. Presets other than raw
  keep their colour, repair, deinterlace and denoise stages but still bypass the combiner (one
  receiver; offline the combiner's waits run on host time). The JSON line gains `demod` (as the DSP
  ran it, so a LUT fallback would show), `ref_demod`, `preset`, `load` and `stage_ns` (ns per input
  sample per DSP stage). Existing keys are unchanged; bench.py reads the line as JSON.
- `timeline --demod` now uses the same parser.
- `tools/bench/demod_compare.py`: `run` (every step x demod x preset, parallel), `summarize`
  (results.csv, thresholds.csv, summary.md), `speed` (DSP cost per demod, one job at a time).

## Runs

Baseline first, defaults, 4 jobs in parallel:
`openvrx-cli score aNN --ref ../colour-225835/c1600 --frames 30 --skip 10`. Reproduced exactly:
reference vs itself 33.21 dB; steps 0/3/5/8/10/14/15/16 C/N 11.13/8.66/7.39/4.23/2.99/1.36/0.65/0.48 dB,
PSNR median 24.39/22.06/20.43/16.23/15.04/11.78/7.78/5.58 dB.

Matrix (345 runs, about 7 min at 4 jobs; equivalent to
`demod_compare.py run bench-runs/colour-ladder-231647 --ref bench-runs/colour-225835/c1600 --jobs 4`):
21 steps x 5 demods x presets raw, low_latency, balanced, plus the reference per chain (against its
own chain and against the polar chain). Then
`demod_compare.py speed bench-runs/colour-ladder-231647 --steps a00,a10 --repeat 2` and
`demod_compare.py summarize bench-runs/colour-ladder-231647`.
Results: `bench-runs/colour-ladder-231647/demod-compare/` (runs/, results.csv, thresholds.csv,
speed.csv, summary.md; `baseline/` holds the reproduction).

## Numbers

PSNR median (dB) of luma, preset raw (cleanup off). C/N is the polar run's.

| att dB | C/N | polar | lut | ls | pll | ekf |
|---|---|---|---|---|---|---|
| 0 | 11.1 | 24.4 | 24.4 | 25.9 | 24.1 | 23.3 |
| 2 | 9.3 | 22.7 | 22.7 | 24.2 | 22.4 | 21.6 |
| 4 | 7.7 | 21.0 | 20.8 | 22.3 | 20.7 | 20.0 |
| 5 | 7.4 | 20.4 | 20.2 | 21.7 | 20.2 | 19.5 |
| 6 | 6.7 | 19.1 | 18.7 | 20.2 | 18.9 | 18.3 |
| 8 | 4.2 | 16.2 | 14.5 | 17.5 | 16.4 | 16.2 |
| 10 | 3.0 | 15.0 | 13.1 | 16.5 | 15.2 | 14.8 |
| 12 | 1.2 | 14.0 | 10.8 | 15.7 | 14.2 | 13.5 |
| 13 | 1.5 | 13.7 | 11.1 | 15.3 | 13.8 | 13.0 |
| 14 | 1.4 | 11.8 | 5.0 | 13.5 | 7.4 | 11.5 |
| 15 | 0.7 | 7.8 | 5.3 | 12.3 | 12.3 | 10.5 |
| 16 | 0.5 | 5.6 | 4.8 | 8.9 | 7.1 | 9.6 |
| 18 | 0.1 | 4.9 | 3.8 | 8.4 | 5.9 | 5.2 |
| 20 | 0.3 | 5.2 | no frames | 7.0 | 5.0 | 3.7 |

All 21 steps and the other presets: `summary.md`. Thresholds (attenuation where PSNR median first falls
below the level, interpolated between steps; gain = variant minus polar on the same preset):

| demod | raw: below 20 dB | gain | raw: below 10 dB | gain | balanced: below 20 dB | gain |
|---|---|---|---|---|---|---|
| polar | 5.3 | 0 | 14.4 | 0 | 7.0 | 0 |
| lut | 5.2 | -0.2 | 13.2 | -1.3 | 6.4 | -0.6 |
| ls | 6.3 | +1.0 | 15.7 | +1.2 | 7.3 | +0.3 |
| pll | 5.2 | -0.2 | 13.6 | -0.8 | 7.0 | 0.0 |
| ekf | 4.0 | -1.3 | 15.5 | +1.1 | 6.1 | -1.0 |

Cleanup (polar): low_latency (the app default) 5.1 dB below 20 dB (-0.2 vs raw), balanced 7.0 dB (+1.7).

Reference against itself (raw): polar 33.21, lut 33.11, ls 34.34, pll 33.01, ekf 32.62; against the
polar chain: ls 33.57, pll 32.87, ekf 32.25.

DSP cost, one job at a time, a00 and a10, median ns per input sample (26.67 MS/s leaves 37.5 ns per
sample on one core for the whole chain):

| demod | demod stage | whole DSP | load |
|---|---|---|---|
| polar | 2.2 | 30.2 | 0.51 |
| lut | 2.2 | 28.3 | 0.50 |
| ls | 8.3 | 35.3 | 0.61 |
| pll | 92.0 | 120.3 | 2.85 |
| ekf | 96.1 | 123.9 | 2.96 |

`load` from the parallel matrix is meaningless (0.9 to 10.8 for the same chain): four jobs and their
video worker threads share four cores.

## Observations

- No demodulator extends the threshold in a way worth switching for. The tracking demods (PLL, EKF)
  do not deliver threshold extension on this ladder: PLL matches polar down to step 13 and then has a
  bad step (14: 7.4 dB, likely cycle slips); EKF is 0.7 to 1.1 dB worse at every watchable level and
  only better (by about 1 dB) where the picture is already below 10 dB. Both cost about 3x real time
  on one core, so neither could run live at 26.67 MS/s as written.
- LUT matches polar on a strong signal and falls behind near threshold (-1.3 dB at the 10 dB level,
  no frames at step 20). It bypasses the channel filter, which matters most when the noise is wide.
- LS (len 3) is 1.1 to 1.5 dB better on every step, including the strong reference (34.3 vs 33.2),
  so this is not threshold extension but a narrower noise bandwidth. Its weights 0.75/1/0.75 are a
  3-tap smoother after the discriminator: -0.6 dB at 2 MHz, -1.35 dB at 3 MHz, -1.95 dB at the 3.58 MHz
  chroma subcarrier, -2.5 dB at 4 MHz (computed from the weights). FM noise rises with frequency, so
  trimming the top of the band removes the noisiest part. The score is luma PSNR only and cannot see
  the lost sharpness or colour saturation. A polar chain with a lower video LPF cutoff is probably the
  same trade at a lower cost (2.2 vs 8.3 ns per sample).
- Cleanup: low_latency costs about 0.3 dB of PSNR against raw (edge-directed deinterlace and the 2D
  comb do not help a static card in noise; repair has nothing to repair here). balanced's recursive
  filter is worth about 1.7 to 2 dB on every step, but this card is static, which is the temporal
  filter's best case; in flight the gain will be smaller and motion trails are the cost.
- The end-of-run lock state is a snapshot: a04 reads Coasting for ls, lut and pll with 30 frames and
  PSNR above 20 dB. It is not a usable "picture lost" criterion; PSNR is.

## Conclusion

Keep polar as the default. No live default changed. Follow-ups: DSP-3 (sharpness and chroma metric,
then LS against a narrower video LPF on polar), DSP-6 (PLL/EKF cost and the PLL slip at step 14),
BENCH-11 (re-run this ladder with the ESP32 on its battery).
