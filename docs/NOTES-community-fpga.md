# Notes: FPGA receiver discussion (C5VRX Discord, 2026-10-30)

A C5VRX community member built an FPGA receiver fed by the C5's 4-bit I/Q
lanes. Points that matter for OpenVRX, with what we do about each.

## What made the FPGA picture clearer

- Same 4-bit I/Q as C5VRX. The gain came from **rate and the demod/sync
  chain**, not bits: the FPGA oversamples the GPIOs at 675 MS/s, a
  phase-tracking resampler recovers one I/Q value per bus update (~80 MS/s),
  and a PLL demod runs on every sample. Their simulation: demod + sync
  separator worth about 3 dB (1.3 to 1.6x distance).
- Oversampling also removes the sampling-phase problem entirely (no clock
  from the C5 needed). The FX2 cannot do this: it latches on an external
  clock at 48 MHz max.

**For OpenVRX:**
- Expect sample rate to matter more than bits past 4 per component. The 4x4
  presets at 40 MS/s (8-bit FX2 mode, 40 MB/s) are a serious candidate
  against 8x8 at 13.33 MS/s; the lab's S1 experiment must include them
  (it does) and the recommendation should weight rate heavily.
- 80 MS/s (every bus sample) is out of reach for one FX2 (80 MB/s). It is
  the argument for the FX3 / FPGA upgrade path in the design notes (H2).
- The PLL and EKF tracking demods in openvrx-dsp are the right direction;
  benchmark them at every rate.

## Diversity numbers (their Monte Carlo, not field data)

Rician K = 6 dB, antenna correlation 0.5, at 1 % picture-breakup time:

| Method | Fade margin saved | Distance |
|---|---|---|
| Post-detection selection (every few us, by envelope C/N) | ~6 dB | ~2x |
| Pre-detection MRC | ~8 dB | ~2.5x |
| MRC, no fading | +3 dB | ~1.4x |

Hard parts they list, which match our design: crystals 120 to 170 kHz
apart at 5.8 GHz, sample clocks drifting 20 to 30 ppm, independent gain per
receiver. Use these values in openvrx-sim diversity scenarios and in the
openvrx-dsp predetect tests.

**For OpenVRX:** our D3 (per-sample repair) is close to their fast selection;
D4 is their MRC. Selection resolution of "a few microseconds" suggests
segment-level selection inside a line is worth it.

## Experiments worth copying

- Record the C5VRX DAC output and the raw I/Q from the **same C5** at the
  same time: identical RF conditions, a direct comparison of demods.
- Selection vs MRC as a diagnostic: selection gains nothing on pure noise,
  so the range selection adds measures how much fading (multipath, pattern
  nulls, polarisation) limits range. Little gain means sensitivity or
  shadowing is the limit.
- If OpenVRX still trails a normal analog receiver after a good demod, the
  remaining gap is the front end: try an LNA and a 5.8 GHz band-pass filter
  (design notes H3, H4).
