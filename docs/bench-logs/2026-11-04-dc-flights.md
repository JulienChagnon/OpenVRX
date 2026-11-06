# 2026-11-04: first flights with the C5 DC correction (C5-1)

## Setup

- Source: the quad flying (camera on), NTSC, VTX on R7 5880 MHz; the last flight at 400 mW.
- C5: firmware built and flashed on Atlas the same evening (ESP-IDF v6.0.2, pinned PHY), `dc=1`,
  `agc=1`, 4x4top, clk 9, saved gain 83. Before the flights, with no VTX signal: old firmware rms 2.70
  steps (about 160 ADC units of DC counted as signal); new firmware rms 0.91 around the mean,
  i_dc +7, q_dc +19, trim -63/-34 (IMPROVEMENTS.md C5-1).
- Host: the app live, FX2 only (`vrx`), recordings with R into ~/Videos/OpenVRX (local only).

## Runs

Four recordings, two app sessions (`start_host_ns` restarts at a relaunch):

| Recording (UTC) | Length | App session | Notes |
|---|---|---|---|
| 231417 | 5 s | first | the session that showed black bars (see below) |
| 231435 | 36 s | second | 6 usb_gap events at 17 s (44 ms) |
| 231530 | 26 s | second | |
| 231926 | 48 s | second | range test at 400 mW: basement to the floor above |

Analysis: `openvrx-cli timeline REC --window 2` and `openvrx-cli decode REC OUT N`.
Video of the range test: `openvrx-cli decode` of every field, then
`ffmpeg -framerate 60000/1001 -i ...-frame%02d.png -c:v mpeg4 -q:v 2 -pix_fmt yuv420p`
(openvrx-20261104-231926-decoded.mp4, 2901 fields, kept with the recordings for later bench use).

## Numbers

- DC over whole flights (decode stats, ADC units, full scale 512): 231417 i_dc +7.0, q_dc +3.1;
  231926 i_dc +4.3, q_dc +3.7. The crash flight before the fix: +216/+136.
- 231417 (first session): locked throughout, C/N 20-22 dB, damaged 1.2-2.0 %, 0 relocks, 0 gaps.
- 231926 (range test): close in C/N 13-19 dB, damaged 2-3 %; at the far end (8-34 s) C/N about
  -10 to +5 dB with 52-100 % damaged and the lock kept (one coasting window, 78 relocks at 12 s);
  back close in at 44-48 s, damaged 1-8 %.
- Clipping still swings widely in all three later flights (0 to 95 % per 2 s window, often at
  moderate C/N right after a fade): the gain loop's fade lag (C5-4), no longer the DC.

## Observations

- Owner: a big range improvement over the crash-flight firmware; the 400 mW flight went from the
  basement to the floor above, estimated at about half the range of a TBS Fusion.
- First session: black bars on some rows that stayed through a drone (VTX) restart and cleared only
  when the app was closed and `vrx` relaunched. The raw recording of that session decodes clean
  offline (no bars, healthy numbers above), so the receiver chain was fine and the fault is state in
  the live app. The offline decoder bypasses the video combiner, which the app uses: see DSP-5.
