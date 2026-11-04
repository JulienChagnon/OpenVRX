# Lane layout, clock divider and diversity: what to try on the bench

OpenVRX builds on C5VRX by Twotoz and the C5VRX contributors. This note sums up the lab results from
2026-10-31 (`lab/results/`) and turns them into an order of things to try on hardware. All C/N figures
are per 20 MHz channel (the C5VRX definition). "Range" means the C/N where 1 % of picture lines break up;
1 dB less C/N needed is roughly 12 % more distance. "Ceiling" is the luma SINAD of a noise-free decode
against the transmitted picture, i.e. how clean a strong signal can look.

## What the first capture showed

The reference decoder now handles recordings with dropped blocks (`decoder.decode_gapped`): each 32 KB
block is line-locked on its own and placed by its absolute sample index, and blocks that sit in the
vertical interval are re-locked onto the global line grid (they first lock onto the half-line
equalising pulses). On both 2026-10-31 recordings every block locked (523 and 414), neighbouring blocks
agree to within 1.3 samples, and the frame phase is consistent, so the `usb_gap` byte counts are exact.
Mosaic frames assembled from about 8 % of the stream are complete and sharp with a readable OSD and
working colour (`results/R0_*_all*.png`). Measured on hardware: line rate 15733.3 Hz, deviation about 1.09
of the C5VRX nominal, blanking at -1.5 to -1.75 MHz, 0H jitter 12.5 to 15 ns, flat-region SNR 27.5 and
30 dB.

The remaining noise on that capture is clipping, not thermal noise. With 85 % of samples at full scale the
I/Q pair is close to hard limited, and the result depends on where the carrier sits: the simulator
reproduces the hardware numbers at the measured -1.75 MHz blanking (2.7 IRE noise, 3 repaired samples
per line) but gives 12.5 IRE at the -1.0 MHz AFC porch target. Mild overdrive (a few dB above the normal
target) is actually the quietest setting, but heavy overdrive next to the AFC target is bad. For the
bench: drop the gain about 6 dB from 64 when the drone is close, and do not judge layouts on a clipped
capture.

## Layout results (simulation, S1)

Full reference decode, NTSC, three VTX deviations, two seeds per point (`results/S1_sample_format.md`).
C/N at 1 % breakup for deviation 1.0, worst deviation (0.7) in brackets, then the noise-free ceiling:

- 7x7 at 20 MS/s (clk 12, 40 MB/s): 8.6 dB (11.3), ceiling 31 dB
- 4x4fine / 4x4ultra at 26.67 MS/s (clk 9, 26.7 MB/s): 9.0 dB (11.9), ceiling 24.6 / 24.2 dB
- 4x4top at 26.67 MS/s: 9.0 dB (12.1), ceiling 22.2 dB
- 4x4 variants at 40 MS/s (clk 6, 40 MB/s): 9.0 to 9.1 dB (11.9), ceiling 24 to 26 dB
- 5x5 / 6x6 / 7x7 / 8x8 at 13.33 MS/s (clk 18, 26.7 MB/s): 9.9 to 10.0 dB (12.9 to 13.0), ceiling 23.6 /
  26.2 / 27.1 / 27.4 dB
- 4x4top at 13.33 MS/s: 10.3 dB (13.9), ceiling 19.2 dB

So rate matters more than bits for range, as the FPGA builder found, but the effect is smaller here:
going from 13.33 to 26.67 MS/s buys about 1 dB, going on to 40 MS/s buys nothing more with 4-bit
samples, and bits past 5 per component buy nothing at threshold. Bits matter for the ceiling: 7x7 gives
a picture about 6 dB cleaner than the 4x4 layouts at good signal. The community's 3 dB came from rate
plus a per-sample PLL demod compared with the C5VRX output chain. Our decoder is already a demod of that
kind (exact polar discriminator with click repair at every sample), and a plain second-order PLL tried in
its place did no better at 40 MS/s or 80 MS/s (at 80 MS/s, a PLL with an 8 MHz loop broke 2 % of lines at
10 dB C/N where the polar decoder broke none, and its ceiling was 22 dB against 26 dB). The PLL sweep
(`experiments/s1c.py`) was not run in full; treat this as a single-seed check.

The folded layouts (4x4fine, 4x4ultra) only work with the C5VRX ultrafine gain policy: signal RMS about
4.2 cells of the lowest carried bit (about 130 ADC units for fine, 65 for ultra) instead of the 2000 or
so on the first capture. That is roughly 24 dB less gain. Ultra also folded badly in one 40 MS/s run at
deviation 1.4, so fine is the safer of the two.

## Order to try on hardware

1. 4x4top, clk 9 (26.67 MS/s). Already wired and verified end to end. Re-record with the gain about
   6 dB lower, then step the VTX power or distance down to breakup and note the C/N. This is the baseline
   every other layout is compared against. The live app needs the DSP throughput fix to run at this rate
   without dropping blocks; until then, record and decode offline, or use clk 18 for live viewing.
2. 4x4fine, clk 9. Same wires, only `lanes 4x4fine` on the C5 and a much lower gain. Same range as
   4x4top in simulation, but a picture about 2.4 dB cleaner. If the gain cannot go low enough on the C5,
   the folds will show as sparkles and this layout is out.
3. 7x7, clk 12 (20 MS/s, 40 MB/s). Best in simulation on both counts (0.4 to 0.6 dB more range than any
   4x4 layout, and the cleanest picture), but it needs the XIAO back pads, 15 data pins and 40 MB/s,
   which is at the edge of this laptop's 41 to 47 MB/s. Try it once the back pads are wired and the FX2
   throughput test runs at 40 MB/s with no `usb_gap` events.
4. 7x7, clk 18 (13.33 MS/s, 26.7 MB/s). The fallback on the same wiring if 40 MB/s drops blocks: about
   1 dB less range than the 26.67 MS/s layouts, but a 27 dB ceiling and the lightest DSP load (75 ns per
   sample, which the current live DSP already meets).
5. 4x4ultra, clk 9. Only if 4x4fine shows visible quantisation; no gain over fine in simulation and more
   fold risk.
6. 4x4fine or 4x4top, clk 6 (40 MS/s). No range gain over clk 9 in simulation, a slightly cleaner
   ceiling for 4x4top, 50 % more DSP load and the FX2 at its limit. Worth one recording to check the
   rate result on hardware, nothing more.
7. 6x6, clk 18. Only if one back pad is unusable for 7x7: same range, about 1 dB lower ceiling.
8. 5x5, clk 18. Front pins only, but worse than 4x4fine at clk 9 on both counts (1 dB less range, same
   ceiling) with more wiring. Skip.

Not possible on the XIAO: 8x8. 4x4top at clk 18 is only a stopgap while the live DSP cannot keep up
with 26.67 MS/s (it gives up 1.3 dB of range and has the worst ceiling). Dividers must stay multiples
of 3 so every sample is a fresh bus value.

## Diversity (S3f)

Rician fading, K = 6 dB, Doppler 100 Hz, one receiver needs 19.3 dB mean C/N for 1 % breakup against
9.0 dB without fading: a 10.3 dB fade margin (`results/S3f_diversity.md`). With a second antenna and
receiver, at the community's settings (K 6 dB, scattered parts correlated 0.5):

- selection (per line, the stronger antenna, which is what D1 and D3 approach) saves 5.5 to 6.6 dB
- pre-detection MRC saves 7.2 to 8.3 dB if ideal, 6.2 to 7.3 dB with 1 dB of implementation loss
- without fading, MRC gains the full 3 dB and selection about nothing

The ranges cover how correlated the two envelopes are (0.03 to 0.24 here, set by the antennas' relative
LOS phase as much as by the 0.5 correlation of the scattered parts). At an envelope correlation of 0.5
both gains drop about 1 dB (selection 4.4, MRC 6.1 dB). This agrees with the community Monte Carlo
(about 6 and 8 dB). MRC beats selection by about 1.7 dB in fading, so it is worth building, but a second
receiver with plain selection is worth more than any layout change above (6 dB against at most 1.5 dB).
Start with D1/D3 selection, which needs no coherence between the receivers, then add D4 MRC.

The single-receiver curve comes from the full-chain S1 run; the fading is a line-rate Monte Carlo on top
of it (fades are flat over a 63.6 us line). The full-chain diversity calibration, which would measure the
real D4 combiner's loss with 120 to 170 kHz carrier offsets and 20 to 30 ppm clock drift, was stopped for
lack of memory on the lab machine. Run `python -m openvrx_lab.experiments.s3fade` on an idle machine to
replace the assumed 0 to 1 dB loss.
