# Improvement backlog

From bench sessions (see [README.md](README.md) for the format). Newest first within each part.

## C5

### C5-1: I/Q DC offset clips one side of each axis
- Status: open
- Evidence: [2026-11-04](2026-11-04-esp32-pattern-first-light.md), 4x4top clk 9, about -61 dBm, gain loop
  on: `i_dc` +128, `q_dc` +110 (ADC units, full scale 512), so about 0.22 of full scale. The I and Q
  histograms pile up in the top bin (16 %), and the recording is 6 % clipped although the gain loop
  holds rms near its target. Gain 70 with the loop off: `i_dc` +109, `q_dc` +91, 0.03 % clipped.
- Idea: remove the DC before the lane quantiser on the C5 (the existing DC/IQ calibration, run against a
  carrier, or a slow DC servo), so the full 4-bit range sits around the signal. The gain loop could also
  count clipping per side.
- Verify: `decode` stats `i_dc`/`q_dc` near 0 and `clip_pct` down at the same rms; C/N and damaged at
  threshold in a sweep against the current firmware.

### C5-2: gain loop settling after a retune or picture change
- Status: open (measure first)
- Evidence: 2026-11-04: right after `tune 5917` 43 % clipped at gain 79; settled at gain 70 within the
  few seconds before the next `status`. When the ESP32 restarted its picture, the loop went to gain 72
  with 12 % clipped and was still settling when a recording started.
- Idea: faster attack when clipping is high (step more than 4 indexes per 100 ms above, say, 20 %
  clipped), and hold gain steps to vertical blanking (a known gap of the Nov 3 gain loop).
- Verify: poll `status` every 100 ms after a retune and log gain/clip/rms against time; compare settle
  time before and after.

### C5-3: gain index to dB is unknown, so the gain loop's index cannot be read as a level
- Status: open
- Evidence: 2026-11-04: the loop settled at gain 70 with the step attenuator at 2 dB and at gain 75
  at 30 dB, a 28 dB difference for 5 indexes (2.5 dB if the assumed 0.5 dB per index held). Either the
  attenuation does not reach the C5 (BENCH-7) or the table is far from 0.5 dB per index.
- Idea: measure the per-board table (gain index against a known level step at fixed input), store it,
  and report the gain in-band so recordings know the level.
- Verify: fixed input, sweep `gain` indexes, record rms after DC removal per index.

## FX2

Nothing yet (2026-11-04: 3 s recordings at 26.7 MB/s, 0 USB gaps).

## DSP (host)

### DSP-1: relocks on bench signals that the simulator does not show
- Status: investigating
- Evidence: 2026-11-04: ESP32 NTSC card through the VTX, locked and visually clean, 0 % damaged, yet
  10 relocks in 3 s with the gain loop on and 25 (2412 coasted lines) with it off at 0.03 % clipping.
  The simulator's ideal NTSC (4x4top, divider 9, C/N 17 dB) through the same `decode`: 0 relocks.
- A relock is the sync flywheel deciding its line grid is wrong and jumping to a new pulse train; each
  one can shift or tear lines until the next field.
- Next: record the quad's camera as the source (rules the ESP32 pattern in or out); compare `pattern
  black` with `card` (picture content); find where in the frame the relocks fall.

### DSP-2: `power_db` reads higher with no carrier than with one
- Status: open
- Evidence: 2026-11-04, gain 70 fixed, 1 s each: VTX in pit mode (no lock, C/N -10 dB) `power_db`
  68.64; VTX on (locked, C/N 16.9) `power_db` 45.85. So `power_db` does not track the received level when
  the carrier is gone; likely the I/Q DC offset (C5-1) and noise dominate it.
- Why it matters: bench.py's leakage check expects `power_db` to fall about 1 dB per dB of attenuation;
  near and below threshold it may not.
- Idea: compute power after DC removal (or report the carrier level from the FM demod), and say which
  in the score JSON.
- Verify: a sweep through threshold at fixed gain; `power_db` should fall monotonically with attenuation.

## Bench tools

### BENCH-5: RF leaks around the pads (goggles see the VTX)
- Status: checked at 25 mW: below the C5's noise with its input terminated (fine for sweeps down to
  about -89 dBm); repeat before the threshold runs (extra 20 dB pad)
- Result (`bench-runs/leak-check2-145019`, `leak-check3-145230`): cable ends 1 to 2 m apart, C5
  pigtail still open: still locks, C/N 9.2 dB. C5 pigtail on the spare 20 dB pad (far end open, so the
  input sees about 50 ohm): C/N -10 dB (no signal), C5 status unchanged between pit mode and 25 mW.
  The locks came in through the open pigtail acting as an antenna; around the pads, the leak into a
  terminated C5 is below its noise.
- Evidence: 2026-11-04: goggles on R7 showed the VTX near the bench while it was in pit mode (Tramp
  read `pit=1` before and after), with the VTX and ESP32 on an aluminium cake sheet (open on top). At the
  same time the C5, behind 70 dB of pads, saw no carrier (gain loop at 83, 0 % clipped, noise only).
- Why it matters: anything radiated by the VTX board, its SMA joint or its power and Tramp leads
  bypasses the pads. If enough of it reaches the C5 directly, weak steps of a sweep measure the leak,
  not the pads (BENCH.md asks for this check before trusting anything below -90 dBm).
- First C5 check (`bench-runs/leak-check-144721`): coax unscrewed at the C5, VTX at 25 mW into its pads,
  step at 0 dB: the C5 locked (15734.5 Hz, carrier -1.70 MHz = this VTX), C/N about 10 dB, 99.99 %
  damaged; in pit mode, nothing. Not conclusive: the cable's open end (about -59 dBm) hung a few cm
  from the C5's open pigtail, so coupling between the two open ends can explain it.
- Next: same, with the cable end on a 50 ohm load (or wrapped in foil) and moved 1 to 2 m away. Still
  locks: the VTX board leaks into the C5; shield better first (closed tin with a lid, the C5 further
  away, ferrites on the leads leaving the tin). No lock: leakage is below the C5's noise at 25 mW.

### BENCH-7: the first sweep's 30 dB of step attenuation hardly reaches the C5
- Status: investigating
- Evidence: `bench-runs/20261104-150221` (gain loop on, 40 + 30 dB fixed, 25 mW, step 0 to 30 dB, LEDs
  confirmed 30 dB after): C/N 20.8 to 19.4 dB and PSNR 26.9 to 26.5 at every step, picture ok to the
  end (estimated -90.5 dBm). A real -90 dBm should give a C/N of only a few dB. At 30 dB the loop
  settles at gain 75 (70 at 2 dB).
- Candidates: (a) signal bypasses the step attenuator (leak into the chain after it or into the C5 in
  the new under-table spot); (b) the step attenuator gives far less than its setting at 5.8 GHz;
  (c) the level estimate is high by 15 dB or more (pads below their labels, VTX above 25 mW).
- Bypass check (`bench-runs/bypass-check-151145`): step attenuator input unscrewed (left open, 20 cm
  from the VTX, outside the tin), VTX at 25 mW: C5 gain at 83, C/N -10 dB, 100 % damaged, the same as
  pit mode. So (a) is out: no meaningful path around the step attenuator. The signal goes through it,
  and at 30 dB setting it attenuates far less than 30 dB at 5.8 GHz: (b), possibly with (c).
- The AW8637 part number turns up nothing online; 31.75 dB 7-bit step attenuator chips come rated to
  4 GHz (e.g. PE43701) or 6 GHz (PE43711, RFSA3713). Next: read the chip marking on the board, then
  measure its real attenuation at 5.8 GHz against the fixed pads (known steps). Until then, the
  sweep's rx_dbm column is not a level, and only fixed pads set the level.

### BENCH-6: say when the VTX stops answering
- Status: open
- Evidence: 2026-11-04: the RX wire came off; the pit guard could no longer reach the VTX and nothing
  said so until bench.py failed to start.
- Idea: att_ctl prints `EVT vtx not answering` once when the guard's query fails, and bench.py (and the
  app's bench control) show EVT lines.

### BENCH-4: VTX temperature over Tramp may not be live
- Status: open (check)
- Evidence: 2026-11-04: `temp=80` at 25 mW and again, unchanged, after 1 to 10 minutes in pit mode.
- Why it matters: any thermal cut-off would trust this number (none for now: owner's choice,
  2026-11-04; the pit-mode guard does the protecting).
- Verify: leave the VTX in pit mode 15 minutes and read `vtx`; compare with a touch or IR thermometer.
  If it never moves, treat it as unknown and rely on pit mode plus airflow.

### BENCH-1: Linux serial open reset the C5 and the attenuator ESP32 (fixed)
- Status: verified 2026-11-04
- Evidence: on Linux the kernel raises DTR and RTS on open; pySerial `dtr=False` (and serialport
  `dtr_on_open(false)`) then drops DTR first, DTR low with RTS high, which is the reset. It hung a C5.
- Change: c5cmd.py, flash.py, bench.py, openvrx-hw c5/transport.rs drop RTS before DTR off Windows.
  Verified: NodeMCU kept its state over two opens; C5 uptime unchanged by `c5cmd.py`.

### BENCH-2: ESP32 DAC sample pairs come out swapped (fixed)
- Status: verified 2026-11-04
- Evidence: a 1-sample sliver of the wrong level at every vertical edge of the test card.
- Change: video.cpp swaps each sample pair before the DMA. After: clean card, 0 % damaged.

### BENCH-3: att_ctl relay held on after boot (fixed)
- Status: verified 2026-11-04
- Evidence: Arduino-ESP32 3.x ignores `digitalWrite()` before `pinMode()`, so the relay pin started LOW
  (on) and the attenuator auto-repeated to its maximum.
- Change: att_ctl.ino sets the pin mode first. Verified: 8 presses gave exactly 2.00 dB, no climbing.
