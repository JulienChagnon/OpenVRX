# S3f: selection against pre-detection MRC under Rician fading [sim curve + Monte Carlo]

Per-line breakup probability versus C/N for one receiver from S1 (4x4top at 26.67 MS/s, deviation 1.0, full reference decode, 2 seeds). Fading: two correlated Rician gains at line rate (16-sinusoid processes, Doppler 100 Hz, 60 s per case, one LOS direction with an own LOS phase per antenna unless marked 'same LOS phase', scattered parts with complex correlation rho). With K 6 dB the envelope follows the LOS/scatter interference, so the envelope correlation depends on the antennas' LOS phase difference as much as on rho; the cases bracket envelope correlations 0.03 to 0.5. Fading is flat over a 63.6 us line, so a line breaks with the probability the S1 curve gives at its instantaneous C/N. Selection = per-line choice of the stronger antenna (what D1/D3 approach; D3 works per sample and does slightly better). MRC = sum of the branch C/Ns minus an implementation loss (0 dB ideal; 1 dB covers carrier offset 120 to 170 kHz, 20 to 30 ppm clock drift and phase tracking on 4-bit samples).

Not yet run: the full-chain calibration (`run()` in this module), which decodes two receivers with those offsets and drifts through D1, D3 and the real D4 combiner to measure its loss; it was stopped for lack of memory on the lab machine. Run `python -m openvrx_lab.experiments.s3fade` on an idle machine to replace the assumed 0 to 1 dB loss.

No fading, one receiver: 1 % breakup at 9.0 dB C/N.

| fading case | envelope corr. | single @ 1 % | selection @ 1 % | MRC (loss 0.0 dB) @ 1 % | MRC (loss 1.0 dB) @ 1 % | fade margin (single) | saved by selection | saved by MRC (loss 0.0 dB) | saved by MRC (loss 1.0 dB) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| K 6 dB, rho 0.50 | 0.03 | 19.3 | 12.7 | 10.9 | 11.9 | 10.3 | 6.6 | 8.3 | 7.3 |
| K 6 dB, rho 0.50, same LOS phase | 0.24 | 19.3 | 13.8 | 12.0 | 13.0 | 10.3 | 5.5 | 7.2 | 6.2 |
| K 6 dB, rho 0.71, same LOS phase | 0.49 | 19.2 | 14.8 | 13.1 | 14.1 | 10.2 | 4.4 | 6.1 | 5.1 |
| K 6 dB, rho 0.50 | 0.15 | 19.3 | 13.5 | 11.7 | 12.7 | 10.3 | 5.8 | 7.6 | 6.6 |
| K 3 dB, rho 0.50 | 0.03 | 24.3 | 14.6 | 13.0 | 14.0 | 15.3 | 9.7 | 11.3 | 10.3 |
| K 10 dB, rho 0.50 | 0.04 | 13.9 | 10.8 | 8.6 | 9.6 | 4.9 | 3.1 | 5.3 | 4.3 |

Community Monte Carlo (K 6 dB, correlation 0.5, 1 % breakup): selection about 6 dB, MRC about 8 dB, MRC without fading +3 dB. Without fading, ideal MRC here saves exactly 3 dB and selection about 0.
