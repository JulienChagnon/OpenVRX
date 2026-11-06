# 2026-11-04: first flights recorded on the Fedora host

## Setup

- Quad with its own VTX on R7 (5880 MHz). Receiver: XIAO ESP32-C5, 4x4top, clk 9, gain loop on, FX2,
  the OpenVRX app on this host (FX2 only). The first recordings had the C5 still on the bench coax by
  accident, the later ones on an antenna.
- Recordings stay local in ~/Videos/OpenVRX (they show the room; never pushed).

## Recordings

| Recording (UTC) | Length | USB gaps | Lost |
|---|---|---|---|
| 205706 | 12.2 s | 12 | 0.27 s |
| 205743 | 0.9 s | 0 | 0 |
| 205805 | 78.2 s | 262 | 2.35 s |
| 205956 | 41.6 s | 180 | 2.09 s |
| 210138 | 1.1 s | 0 | 0 |
| 210143 | 48.3 s | 136 | 1.81 s |
| 210237 | 3.1 s | 0 | 0 |
| 210242 | 30.9 s | 43 | 0.44 s |
| 210553 | 44.5 s | 79 | 0.55 s |
| 210656 | 30.5 s | 35 | 0.42 s (last flight: lost video, crash) |

## Last flight, `openvrx-cli timeline openvrx-20261104-210656 --window 0.5`

- 0 to 4 s: C/N 8 to 16 dB, about 1 % damaged.
- 4.5 to 9 s: C/N dips to about 3 dB, 13 to 25 % damaged.
- 10 to 23 s: C/N between -10 and +10 dB, 20 to 50 % damaged; clipping swings 0 to 80 % per window.
- 24.0 s: USB burst, 347 ms lost, 21 relocks; 47 % damaged right after.
- 26.5 to 29.5 s: 48 to 67 % damaged; 29.5 s: second USB burst, 72 ms.

## Reading

Three things together: a weak, fading link from early in the flight; the C5 gain loop too slow for
flight fades, so the 4-bit samples alternately clip and under-range (C5-4, with C5-1); and USB bursts
from the live app on this host (FX2-1). New tool: `openvrx-cli timeline` (BENCH-9).

The owner confirmed the crash at 24 s in the decoded video, the 347 ms USB burst. Cause of the bursts:
btrfs compression and checksums at each 5 s writeback starving the USB thread (47 gaps in 40 s to a
normal folder, 0 to tmpfs or a no-COW folder). Fixed the same day: raw recording files are created
no-COW on Linux; 0 gaps in a normal btrfs folder afterwards (FX2-1).
