# Improvement backlog

From bench sessions (see [README.md](README.md) for the format). Newest first within each part.

## C5

### C5-1: I/Q DC offset clips one side of each axis
- Status: built 2026-11-04; first bench check good (top picture item)
- Bench check 2026-11-04 (`bench-runs/tin-open-210314`, ESP32 card, VTX 25 mW, gain loop on): C5
  status `dc=1 i_dc=-12..17 q_dc=-7`, recording `i_dc` -3.7, `q_dc` +14.6 (was +128..+216 / +105..+150),
  clipping 0.3 % (was 6 to 27 %), C/N 20.2 dB, 0 % damaged. Still to see: flight fades.
- Built: DC correction ahead of the ADC (`dc on|off`, default on, saved; docs/PROTOCOL.md). The
  PBUS bank-2 DC DAC pair of the gain row in force is held at the vendor code plus a per-row trim
  (phy.c `phy_dco_*`, rf.c `rf_dc_*`: released before every gain write, retune and restore, held
  again after; bank 1 pinned per RF stage per channel setup; watchdog cause `dco`). A servo on the
  gain loop's 4096 pad reads moves the trim of the row in use every 100 ms (logic/openvrx_dc.c):
  proportional steps from a learned slope while the cloud spans zero, a sign-bit bracket (expand,
  then bisect) while it sits on one side, polarity + by default and flipped only on evidence, a
  deadband of half a code plus two standard errors. While the DC is held the gain loop's rms is
  taken around the mean. `status` reports `i_dc q_dc` (ADC units, as `decode`) and `dco trim`.
  bench.py `--dc on|off` and, with `--c5`, the C5's i_dc/q_dc/dco/trim/clip per step.
  After C5VRX's board work on the same chip and pinned libphy (their live servo: board DC median
  about 134 to about 11 ADC units; the bank-1 pin and the release-before-gain-write rules are
  theirs; their 2026-10-28 fix, never gate the servo on other health checks, is followed).
- Host evidence: test/test_logic.c `test_dc`, a plant with an ADC clamp, the real lane cut and
  accumulator: crash-flight offsets (+216/+136, FM carrier at the gain loop's level) to under 20
  ADC units within 3 s, clipping 13 % to 0.1 %, at most 5 steps in the next 5 s; fixed point
  (noise under a step), inverted polarity (both zones), 3 and 45 ADC units per code, DC out of reach
  (parks at the limit, no ping-pong), 15 % I/Q cross-coupling with a wandering DC, row hops, 8x8.
  400 noise seeds pass (`DC_SEED=n`, `DC_TRACE=1` prints each update).
- Built 2026-11-04 on Atlas with ESP-IDF v6.0.2 (-Werror, no warnings, app 0xc7060 bytes): the
  pinned PHY matched (OPENVRX_PHY_PINNED on every main source) and the libphy PBUS calls the hold
  uses (phy_pbus_force_test, phy_pbus_debugmode, phy_pbus_workmode) link. Not yet flashed or run.
  Unknown on this board: the slope (ADC units per code), whether the I/Q DAC labels match the axes
  (C5VRX: block 2 = I), the correlation of the pad reads (OPENVRX_DC_SE_FACTOR 2 assumed), and the
  cost of the release/hold around each gain write (C5VRX: about 7 ms of picture per gain write,
  a correlation).
- Crash flight (whole recording): `i_dc` +216, `q_dc` +136 of 512 (I at 0.42 of full scale), 36 %
  clipped; the timeline's damaged share follows the clipping. The firmware has no DC calibration, and
  the gain loop's rms (openvrx_agc.c, sum of raw squares) counts the DC as signal. DC cannot be fixed by
  gain: it must come off before the samples are cut to 4 bits (radio DC calibration or a DC servo in
  the analog front end). Until then: measure rms around the mean, and count clipping per side.
- Evidence: [2026-11-04](2026-11-04-esp32-pattern-first-light.md), 4x4top clk 9, about -61 dBm, gain loop
  on: `i_dc` +128, `q_dc` +110 (ADC units, full scale 512), so about 0.22 of full scale. The I and Q
  histograms pile up in the top bin (16 %), and the recording is 6 % clipped although the gain loop
  holds rms near its target. Gain 70 with the loop off: `i_dc` +109, `q_dc` +91, 0.03 % clipped.
- Idea: remove the DC before the lane quantiser on the C5 (the existing DC/IQ calibration, run against a
  carrier, or a slow DC servo), so the full 4-bit range sits around the signal. The gain loop could also
  count clipping per side.
- Verify: `decode` stats `i_dc`/`q_dc` near 0 and `clip_pct` down at the same rms; C/N and damaged at
  threshold in a sweep against the current firmware. With the new firmware: `status` with the VTX off
  then on (i_dc/q_dc within about 20 of 0 after a few seconds, dco/trim stable, no `EVT dc`), then
  the same sweep twice, `bench.py --c5 PORT --dc off` and `--dc on` (attenuator reset between), and
  a flight with `dc on` against the crash flight's timeline (clipping and damaged share).
- First board reading 2026-11-04 (flashed on Atlas, R7 5880, 4x4top clk 9, gain 83, no VTX signal):
  old firmware rms 2.70 steps (DC counted, about 160 ADC units of offset); new firmware within 30 s
  trim -63/-34, rms 0.91 around the mean, i_dc +7, q_dc +19; hold took first time, phy=ok. A 60 s
  poll at 3 s: i_dc -21..+14 around 0, q_dc +9..+44 (mean about +22) with the Q trim creeping
  -28 to -30. Readings jump 10-20 ADC units between polls, about 5x the independent-read standard
  error, so the burst reads are strongly correlated (OPENVRX_DC_SE_FACTOR 2 is too low). clip
  bursts of 2-6 % with q_dc dipping negative at the same moment: likely short interference.
- Flights 2026-11-04 ([session](2026-11-04-dc-flights.md)): whole-flight DC +7.0/+3.1 and +4.3/+3.7
  ADC units (crash flight +216/+136); owner reports a big range gain (400 mW, basement to the floor
  above, about half a TBS Fusion). Clipping still swings 0 to 95 % per window: C5-4, not the DC.
- Ideas from that: report the servo's polarity, slope and flips (a `dc` reply or status fields) to
  tell drift from a wrong flip; measure the read correlation (several status means against one
  long mean) and set OPENVRX_DC_SE_FACTOR from it; spread the 4096 reads over the 100 ms instead of
  one burst for a less noisy mean.
- 2026-11-05, 160 s after power-up, pit-mode VTX behind 90 dB (receiver noise only): `status`
  `i_dc=8 q_dc=109 dco=231/163 trim=-15/-96`: I centred, Q still far off with its trim at -96 (it was
  -28 to -30 with q_dc about +22 on Nov 4). Either still converging, near the trim limit, or the
  one-axis runaway below. Poll `status` for a minute next session before trusting a sweep.
- 2026-11-06: trim window widened from +-96 codes to DAC codes 20..500 (logic/openvrx_dc.h
  OPENVRX_DC_CODE_MIN/MAX, after libphy's phy_pbus_rx_dco_cal; a vendor code outside widens the
  window to itself). Host test: Q +400 at 3 ADC units per code now converges (trim about -133).
  Not yet flashed. If Q still parks at a limit or barely moves over 100+ codes, the Q DAC has
  lost authority (or the I/Q labels are swapped) and range was not the cause.
- Watch for on hardware: IQ dead or railing right after a hold (the owned-word set; C5VRX saw it
  when all eleven PBUS blocks were copied), `EVT phy cause=dco` repeating, `EVT dc limit`, and a
  trim that runs away on one axis (would mean the I/Q DAC labels are swapped).

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

### C5-5: 7x7 lanes for headroom against fades
- Status: open (LAB-MATRIX row; needs the back pads wired and a gap-free 40 MB/s FX2 run)
- Evidence: in flight the 4-bit layouts clip or under-range at every fade (C5-4); the simulator gives
  7x7 a 31 dB ceiling against 22 dB for 4x4top at about the same threshold.
- Verify: same flight path, timeline clipping and damage, 4x4top against 7x7.

### C5-4: the gain loop cannot follow flight fades (clipping swings 0 to 80 %)
- Status: implemented (not verified in flight); flashed 2026-11-04
- Built: fast attack in logic/openvrx_agc.c (`openvrx_agc_fast`, `openvrx_agc_slow`), agc.c runs a
  5 ms check of 256 pad reads between the 100 ms updates. Three hot batches in a row (10 % or more
  clipped) drop the gain 6 indexes, again every 10 ms while it clips (closed loop: the dB per index
  is unknown, C5-3); raises held off 500 ms after an attack. `status` reports `atk=` (attacks since
  boot). The DC trims follow the rows (rf_set_gain re-holds with the new row's trim).
- Board, first version (one hot batch enough), no VTX: about 2 attacks a second, the gain dragged
  83 to 65 and back, the DC trims disturbed: interference bursts (most likely 5 GHz Wi-Fi packets)
  fill a 0.1 ms batch. With three in a row: `atk=0` over 20 s, gain held at 83, DC centred.
- Host evidence (test/test_logic.c `test_agc_fades`, 4x4top plant, FM carrier and noise scaled by
  the gain): weak (-20 dB) then +25 dB at once, recovery to under 5 % clipped: old loop 705 ms, new
  60 ms at 0.5 dB per index; 205 against 30 ms at 2 dB per index. At 2 dB per index the old loop also
  hunts while settling (40 writes in 4 s) and the new one damps it (5). Random bursts on 2 % of the
  batches: at most one attack in 5 s. 300 noise seeds pass.
- Still open: the 100 ms loop's raise steps assume 0.5 dB per index; when the real slope is steeper
  they overshoot and the attack catches each one (C5-3). Gain changes are still not held to vertical
  blanking, and the gain is not reported in-band.
- Evidence: flight `openvrx-20261104-210656` (lost video, crash), `openvrx-cli timeline --window 0.5`:
  clipping jumps between 0 and 80 % from one half second to the next while C/N swings from -10 to
  +16 dB; from 10 s on, 20 to 50 % of every frame damaged, 48 to 67 % near the end. The loop steps
  every 100 ms by at most 4 indexes (about 2 dB), so fades of 20 to 30 dB in tens of ms leave it too
  high (clipped) or too low (under one step); 4-bit samples have almost no headroom, and the DC offset
  (C5-1) adds clipping.
- Ideas: fast attack (drop gain within a few ms when clipping crosses a threshold, slow release);
  fix C5-1 first; a layout with more bits per axis (7x7) for headroom; report gain in-band so the
  DSP knows each step.
- Verify: same flight path, timeline clipping mostly under 5 % and damaged share against C/N.
  Compare with the 2026-11-04 range test (231926, 400 mW, basement to the floor above): its 2 s
  windows clipped 0 to 95 %. Check `atk` before and after the flight (`c5cmd.py PORT status`).

## FX2

### FX2-1: the live app drops USB data in bursts on this Linux host
- Status: verified fixed 2026-11-04 (cause found; see "Cause and fix" below)
- Evidence: 2026-11-04 flights recorded with the app: 12 to 262 `usb_gap` events per flight, 0.27 to
  2.35 s of samples lost (up to 5 %), in bursts (crash flight: 25 gaps, 347 ms, within 0.1 s at 24.0 s,
  21 relocks). `openvrx-cli record` on the same host and FX2: 0 gaps (several 2 to 3 s runs). Windows:
  3 small gaps in 33 s (Nov 2).
- Candidates: host stalls in the app (DSP, video, wgpu present, disk writes of 27 MB/s on btrfs),
  too few USB transfers in flight on Linux, thread priority.
- Cause and fix: the bursts come every 5 s (kernel writeback period) while recording to btrfs with
  zstd compression: the writeback compresses and checksums about 130 MB at once, and that kernel CPU
  burst starves the USB stream thread, so blocks drop before the DSP (the live picture loses them too;
  the recorder's own drop counter stayed 0). 40 s app recordings: normal btrfs folder 47 gaps, tmpfs 0,
  `chattr +C` folder 0. Fix: openvrx-format's writer marks each new raw file no-COW on Linux
  (FS_NOCOW_FL, which on btrfs also turns off compression and checksums; ignored elsewhere). Verified:
  normal btrfs folder, 40 s, 0 gaps. The crash flight's video loss at 24.0 s matches its 347 ms burst.
- Still possible: real-time priority for the USB thread on Linux (needs an rtprio limit), for spikes
  from other load.

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

### DSP-3: compare the FM demodulators on picture quality
- Status: investigating (first comparison done 2026-11-04; polar stays the default)
- Evidence: the crash flight through `timeline --demod` polar, pll, ekf and ls: damaged share equal
  to 0.1 %. The damage flags come mostly from clipping and clicks before demodulation, so they cannot
  rank demodulators. The DSP has PLL and EKF trackers (threshold extension), unused by default.
- Result ([2026-11-04-demod-compare.md](2026-11-04-demod-compare.md), colour ladder 231647, luma PSNR,
  ESP32 on USB so relative numbers only): attenuation where PSNR falls below 20 dB, raw preset, gain
  vs polar: lut -0.2, ls +1.0, pll -0.2, ekf -1.3 dB; below 10 dB: lut -1.3, ls +1.2, pll -0.8,
  ekf +1.1 dB. No tracking gain from PLL or EKF. LS is 1.1 to 1.5 dB better on every step including
  the strong reference, which points to a narrower noise bandwidth (its 0.75/1/0.75 weights droop
  1.95 dB at 3.58 MHz and 2.5 dB at 4 MHz), not threshold extension. Cleanup: low_latency -0.2 dB,
  balanced +1.7 dB (static card, the temporal filter's best case).
- Next: give `score` a sharpness and chroma figure (multiburst zone amplitudes, bar saturation from
  the reference's own mean) so a filter trade shows up; then compare LS against polar with the video
  LPF lowered until its luma PSNR matches LS. Only if LS still wins on both, propose it as the
  default (cost 8.3 vs 2.2 ns per sample in the demod stage, DSP load 0.61 vs 0.51).
- Tools: `openvrx-cli score --demod --preset --ref-demod`, `tools/bench/demod_compare.py`.

### DSP-4: second receiver (diversity)
- Status: open (spare FX2s ordered)
- Evidence: LAB-MATRIX and lab simulations: selection about 6 dB, MRC about 7 to 8 dB, more than any
  layout change; a second antenna in another orientation also covers polarisation and multipath fades
  like the crash flight's.

### DSP-5: black bars on some rows in the live app until it is restarted
- Status: implemented (not verified): detection, self-heal and logging; root cause still unknown
- 2026-11-05 ([pit-ladder-battery](2026-11-05-pit-ladder-battery.md), `bench.py --live`): a different
  face of it, from the first field: only about 11 lines per field reached the video pipeline
  (combiner `lines_in` 643 per 60 fields, 230 to 250 of 262 missing), every 5 s a rebuild that did
  not help, while the recordings were complete and `decode --live` replays them clean in real time
  (2.2 s CPU for 2 s). So lines are lost between the DSP thread and the video thread: the video
  queue full (`route_lines` drops, counted only in the OSD) or the video thread stalled. The main
  view still lists rx 0 and 1 with one FX2. Added: the DSP thread logs drops from a full video queue
  every 5 s with its lines_out and load, and "video lost" prints the pipeline's average stage time
  per field. Next live run: those two lines say which side is short.
- Findings 2026-11-04: `openvrx-cli decode REC OUT N --live` (new) replays a recording through the
  app's live path (low_latency preset, blend, streaming, combiner with host times rebuilt from the
  byte position). 231417, the bars session (app 1 was closed within 7 s of its end, so the bars were
  most likely on screen while recording): 308 fields clean, 0 late lines, 0 relocks, 9 missing.
  So the data alone does not make the bars; it was state the session built up before. The settings
  have a 2-receiver main view (rx 0+1, both "any FX2", one FX2 present): pairing makes only rx0 at
  start, so a second receiver would need a reconnect path; not ruled out.
- Calibration (lost lines per field per second, live path): 231926 range test isolated seconds up
  to 17.5 at the range edge, at most 2 in a row; 231435 at most 3.0.
- Built: the app's video thread counts lost lines (assembler missing and late plus combiner late)
  per 60 fields; more than 4 per field for 5 s in a row rebuilds the video pipeline (combiner,
  assembler, repair, deinterlace state, what an app restart reset) and prints the counters on
  stderr; above 1 per field it prints them at most every 10 s. The view's `dropped_lines` now
  carries the total and the OSD shows "video rows lost: pipeline reset (Nx)" for 10 s after one.
  `vrx` keeps stderr in ~/.local/state/openvrx/vrx-*.log (last 20). Test:
  `health_ignores_range_edge_and_catches_persistent_loss`.
- 2026-11-06: one combiner wedge found and fixed. A USB stall makes the stream thread reap a
  backlog at once, so the stall's lines share one host time; the relock then mapped the lone
  receiver onto a field already emitted and dropped every line as late until the stall's end
  (100 ms stall: 1050 late lines, 300 ms: 4200). A lone receiver now never maps onto an emitted
  field. Test `lone_receiver_survives_bunched_host_times`. Does not explain the 2026-11-05 face
  (lines missing before the combiner).
- Next time it happens: the OSD message and the log show whether the rebuild fixed it. If the bars
  survive a rebuild the state is upstream (DSP per receiver); if no loss is counted at all while
  bars show, rows are written black (repair/deinterlace or the display), not dropped.
- Evidence: [2026-11-04 DC flights](2026-11-04-dc-flights.md), first app session: black bars on some
  rows that survived a VTX restart and cleared only on an app relaunch. That session's recording
  (231417) decodes clean offline: locked, C/N 21 dB, 1.4 % damaged, 0 relocks, 0 gaps. So the fault is
  state in the live app's path, which the offline decode does not run: the video combiner.
- Suspect: `Combiner::push` (openvrx-video combiner.rs) drops every line whose (common field, line)
  is at or before `last_emitted` as late. The field mapping (`map_seq`, parity plus host arrival time)
  and `last_emitted` live as long as the app; if the mapping lands part of each field on an emitted
  field, those rows are dropped every field and stay black, and a VTX relock does not reset it.
  Not confirmed: the app logs nothing and shows no combiner counters.
- Ideas: show the combiner's `late_lines`, `timeouts` and `relocks` on the OSD/stats panel and in the
  recording's events (so the next occurrence leaves evidence); reset the combiner when the only
  receiver relocks or when late lines persist (e.g. more than a few per field for a second); replay a
  recording through `push_line` with host times rebuilt from the sample count to reproduce it.
- Verify: reproduce with the replay or on the bench (VTX power cycles, USB gaps); after a fix, late
  lines stay at 0 through VTX restarts and no bars appear.

### DSP-6: PLL and EKF demods are 3x too slow for live and show no threshold gain
- Status: speed fixed 2026-11-06 (threshold gain still open)
- 2026-11-06: the trackers' detector arg(z e^-j th) is now computed as wrap(arg z - th), with arg z
  vectorised ahead of the serial loop (no per-sample sin/cos or atan2 in it), and fastmath
  `wrap_pi`/`sin_cos` no longer call libm floorf/roundf (baseline x86-64 has no roundss). Output
  equal to the old code within 1e-6 rad. Isolated demod at 26.67 MS/s: PLL 73 to 15 ns/sample,
  EKF 72 to 26; bench.rs demod stage at 13.33 MS/s 7x7: PLL 76 to 16, EKF 75 to 28.
- Evidence ([2026-11-04-demod-compare.md](2026-11-04-demod-compare.md)): demod stage 92 ns (PLL) and
  96 ns (EKF) per sample against 2.2 ns for polar; whole DSP about 120 ns per sample, load 2.9 on one
  core at 26.67 MS/s (the budget is 37.5 ns). On the colour ladder PLL tracks polar to step 13 then
  drops to 7.4 dB at step 14 (polar 11.8; likely cycle slips), and EKF is 0.7 to 1.1 dB worse at
  every watchable level, better only below 10 dB PSNR.
- Ideas: before any speed work, check the loop settings against the signal (PLL natural frequency
  6 MHz and damping 0.8; EKF bandwidth 3 MHz designed at C/N 10 dB, our threshold region is C/N 1 to
  7 dB); then a vectorised sin/cos or a phase-detector LUT for speed.
- Verify: rerun `demod_compare.py run` with `--demods polar pll ekf` after each change; a useful
  tracker moves the 10 dB crossing right without moving the 20 dB one left, at load below 1.

## Bench tools

### BENCH-12: att_ctl gives up a VTX set on one missed Tramp read
- Status: worked around in bench.py (2026-11-05); firmware fix open
- Evidence: 2026-11-05, first `bench.py --pit` tries: the full `vtx` query answered (VTX 80 C in
  pit), then `vtx active 0` came back `ERR vtx no answer from the VTX`. In tramp.cpp
  `set_and_confirm` gives up when its first `query_status` misses once, while the reads after a set
  already try twice ("the VTX sometimes misses the first read"). Same run: the C5 did not answer
  `agc on` within 2 s although it had not reset (kernel log: no disconnect, `up_ms` matched the plug
  in) and answered `c5cmd.py ver status` right after; cause unknown, bench.py did not keep what it
  received.
- Done: bench.py `command(..., tries=N)` retries and quotes the last lines received; the sweep pings
  the C5 before relying on it and gives `agc`/`gain` 3 tries; `--pit` tries `vtx active 0` 3 times and
  carries on when the VTX already reports `pit=1` (no lease is held, so the guard keeps it there).
- Next: in tramp.cpp, try the first `query_status` of `set_and_confirm` twice like the later reads
  (needs an ESP32 reflash); if the C5 silence comes back, the quoted lines say what it sent.

### BENCH-11: the colour ladder ran with the ESP32 on PC USB power
- Status: open; the battery rerun did not reach the noise floor
- 2026-11-05 ([pit-ladder-battery](2026-11-05-pit-ladder-battery.md)): same chain, ESP32 on battery
  over Wi-Fi: C/N 10.7 at step 0 (231647: 11.1) but only 9.4 at step 20 (231647: 0.3), power down
  0.9 dB for 20 dB of steps, with a 0.7 dB rise at the 8 dB bit. Most likely a new path around the
  pads since 231647 (placement of the ESP32, its battery and wires); to confirm with the attenuator
  LEDs and an open-chain C5 `status`.
- Evidence: owner note 2026-11-04: during `colour-ladder-231647` the ESP32 (test pattern on P25, Tramp
  to the VTX) was powered from the PC, not its battery. That joins the VTX ground to the PC ground the
  receiver also uses: a possible RF path around the attenuators (deep steps would read stronger) and
  USB or ground-loop noise on the DAC pattern (caps PSNR on every step). The pit90 runs (BENCH-10)
  used the battery.
- Effect: the DSP-3 ranking holds (every demodulator decodes the same samples); absolute thresholds
  from this ladder are not trusted.
- Verify: rerun the ladder with the ESP32 on its battery over Wi-Fi, then
  `demod_compare.py run` and `summarize` on it; compare the polar curve and the reference's PSNR on
  itself (33.2 dB here) with this ladder.

### BENCH-5: RF leaks around the pads (goggles see the VTX)
- Status: open. The terminated-C5 check below passed, but it could not see the path that matters:
  with the chain unscrewed from the C5 there is no metal path along the coax (see BENCH-7). Repeat
  with the chain connected.
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
- Chip marking: ARW637 DSA, 9 kHz to 8 GHz (Concept-RF), so 5.8 GHz is in its range.
- Added the 20 dB pad (40 + 30 + 20 fixed, step attenuator reset to 0 by the owner, so 90 dB;
  folder named att30 by mistake: `bench-runs/pad90-att30-152231`): gain 75, C/N 20.6 dB, 0 % damaged,
  the same as 40 + 30 fixed with the step at 30 (100 dB). Neither the step attenuator nor a fixed
  pad changes what the C5 gets, yet breaking the chain removes it. Best explanation: common-mode
  current on the outside of the coax (connector bodies, pad barrels, braid) from the VTX to the C5's
  ground. Pads only attenuate the signal between centre pin and shield, so this path bypasses all of
  them; it needs the chain's metal to be continuous.
- Next: bond the VTX end (its SMA and the first pad) to the tin, and measure again; then a closed box
  with a bulkhead SMA, and bonding at the C5 end. Ferrites do little at 5.8 GHz.
- Pads taped to the tin, tin closed but for a wire gap (`bench-runs/bonded-153147`, 90 dB, step 0):
  gain 83, C/N 14.8 dB, 59 % damaged (from gain 75, C/N 20.6): the bypass dropped. Then step 10
  (`bonded-step10-153544`): gain 71, C/N 20.0 dB, 0 % damaged, more signal with more attenuation.
  The level still follows the bypass, which shifts with small physical changes, not the chain.
- USB ground loop (`bench-runs/usb-loop-154333.log`, 90 dB, step 0, VTX 25 mW): ESP32 plugged in,
  gain 71, clip 10 to 14 %; ESP32 unplugged (VTX left on), gain 79, clip 9 to 13 %; replugged, the
  guard had the VTX back in pit mode within about 2 s. So the ESP32/USB loop is one path, not the
  only one: with it broken the C5 still gets a strong signal.
- Paths now: (1) common mode on the outside of the coax (bonding helped), (2) the USB ground loop,
  (3) radiation into the receiver side (C5, FX2, their USB) through the tin's gap. A sensitivity bench
  needs about 100 dB of isolation; one tin with a gap gives perhaps 30 to 40.
- Fix plan. Free: receiver as far from the VTX as the coax allows; bonds metal to metal (plain tape
  holds but does not connect); a second tin over the C5 and FX2. Buy: copper or aluminium tape with
  conductive adhesive; two SMA female-female bulkheads; a USB isolator (ADuM3160 class, 12 Mbps) for
  the ESP32; two tight metal boxes or tins; a double-braid coax (RG400/RG142, about 1 m). Then: VTX,
  pigtail and all fixed pads inside the VTX tin, only the attenuated signal out through a bulkhead
  bonded to the wall, the 12 V / Tramp / video wires out through one small hole, twisted and short,
  seams taped; the receiver in its own tin with its own bulkhead.
- Until then: strong-signal work is fine (picture, gain loop, colour, C5-1); threshold and range
  sweeps wait, since below about -75 dBm the leak sets the level.
- Cookie tin, lid closed and sealed, VTX + pigtail + 40/30/20 pads inside, last pad bonded at the
  exit hole, GND bonded at the hole, wires through foil-sealed hole, no capacitors yet
  (`bench-runs/tin-closed-*`, new C5 firmware with DC correction): step 0/10/20/30 dB gave gain loop
  65/70/73/74 and C/N 22.1/22.4/20.9/20.9 dB, all locked, 0 % damaged. The response now rises
  monotonically and flattens (a floor), unlike the taped tin, but at 120 dB total the C5 still gets a
  clean picture: isolation about 100 dB at best. Gain steps cannot be read as dB yet (C5-3), and C/N
  sits at the 4-bit ceiling while strong, so the shape of the gain response is the evidence.
- Leak meter: with the step attenuator left at 30 dB the conducted signal is far below the noise, so
  anything the C5 receives is leak. Next, one change at a time with a short lease each: receiver side
  under a second tin, ESP32 USB unplugged, capacitors at the hole.
- Receiver (C5 + FX2) in its own tin, foil-taped (`bench-runs/rx-tin-*`): ladder 0/10/20/30 dB gave
  gain 65/71/75/~71 and C/N 21.3/21.0/20.7/10.4 dB (2.2 % damaged at 30). But the ground wire from the
  12 V input to the shared ground had come off at some point (owner found it later), so this ladder and
  the next USB test may have run with the ground loop already open.
- Ground loop confirmed as the main path (`bench-runs/gnd-loop-*.log`, step at 30 dB, ground wire
  soldered back): ESP32 on USB, gain 62.4 average (59 to 72), close to the 0 dB level; ESP32 USB
  unplugged, gain 76.1 (70 to 83); replugged, the guard put the VTX in pit (gain 83, noise). The loop:
  VTX ground, shared ground wire, ESP32, its USB, the PC, the C5/FX2 USB. Fix: a USB isolator (ADuM3160
  class) between the PC and the ESP32, keeping the ESP32 and VTX grounds joined for Tramp and video.
  Then capacitors at the hole and seams for what remains (C/N about 10 at 120 dB with the loop open).
- ESP32 on a power bank over Wi-Fi (att_ctl net console, `socket://openvrx-bench.local:2323`), USB
  cable out through the hole to the bank, ground wire connected, lid closed (`bench-runs/wifi-leak-*`,
  step at 30 dB): gain 73 steady, C/N 21.6 dB, locked, 0 % damaged. The PC loop is gone (62 before)
  but a strong leak remains. The ESP32's own Wi-Fi read -41 dBm with the lid open and -83 with it
  closed: the tin blocks about 40 dB at 2.4 GHz, so the leak rides conductors joined to the VTX
  ground that leave the tin (the bank's USB cable, the 12 V leads), not the box walls. The best
  reading so far (C/N 10.4 at 120 dB) was with that ground wire off.
- Next: everything on the VTX ground inside the tin (power bank in, VTX on a LiPo inside instead of
  the 12 V supply), so only the bonded coax leaves; then read the leak meter at 30 dB again.
- First a cheap power cable made the ESP32 brown out on Wi-Fi (never joined the network); a better
  cable fixed it.

### BENCH-8: a resistor-ladder test source (the C5VRX DAC idea)
- Status: built and verified 2026-11-04 (att_ctl ladder.cpp, `out ladder`, the default)
- Result (`bench-runs/colour-225835`, VTX 25 mW): 6-bit ladder on GPIO 18 19 21 22 23 26 (8.2k 3.9k
  2.0k 1.0k 470 240 ohm, 200 ohm and 470 pF to GND), I2S0 LCD mode at 4 fsc (APLL 114.545 MHz / 8,
  measured 14318180 Hz), 1050-descriptor ring (2 frames) over about 20 line buffers. The decoder locks
  the burst first time: 75 % bars in the right colours, multiburst 0.5/1/2/3 MHz resolved, 4 MHz faint
  (next to the subcarrier, removed by the colour separation), ramp in visible 6-bit steps. Level:
  amp 1000 gave deviation_scale 0.62; amp 1600 gives 1.00 (now the ladder default), C/N 19.8 dB.
- Open: 8 missing lines in the assembler on these recordings; a thin glitch line at the multiburst
  band edge at amp 1000 (gone at 1600); Tramp read-back after leaving pit missed once (retry added).
- Evidence: the ESP32's built-in DAC test card runs at 2.014 MS/s: about 1 MHz of detail, no colour
  burst (3.58 MHz), soft sync edges. C5VRX drives a 6-bit binary-weighted resistor ladder (8.2k, 3.9k,
  2.0k, 1.0k, 470, 240 ohm, 200 ohm shunt) from PARLIO at 20 MS/s; C5VRX has no pattern mode itself.
- Idea: the same ladder on the ESP-32S, fed by its I2S peripheral in parallel (LCD) mode at 13.5 to
  20 MS/s: full NTSC detail (4.2 MHz), colour bars with a burst (tests the chroma decoder), multiburst
  and zone plates, sharp syncs. A frame no longer fits in RAM at that rate, so the DMA chain reuses a
  few line buffers (sync, blank and one per distinct picture row). Six free GPIOs needed (not P16,
  P17, P25, P32); the VTX's input load sets the ladder's output level.
- Verify: the received picture against the reference in the app (key X), PSNR per band, and the
  colour decoder locking to the burst.

### BENCH-9: `openvrx-cli timeline` (added)
- Status: implemented 2026-11-04, used on the crash flight
- `openvrx-cli timeline REC [--window S]`: one row per S seconds over the whole recording: lock, C/N,
  clipping, relocks, damaged share of the frames finished in the window, USB gap time. `score` only
  covers the first frames, which hid the crash flight's decline.

### BENCH-10: threshold bench with the VTX in pit mode (works today)
- Status: verified 2026-11-04; bench.py support open
- Why: leak and conducted signal both scale with VTX power, the chain attenuation sets their ratio;
  the leak caught up at about 100 dB. A weak source (pit mode) through 90 dB keeps the conducted
  path on top while reaching the noise floor.
- Result (`bench-runs/pit90-*`, VTX pit, 40 + 30 + 20 fixed, ESP32 card, ESP32 on battery over Wi-Fi):
  step 0/2/4/6/8/10/12/14 dB gave C/N 14.5/12.2/10.3/7.8/4.9/1.7/-1.2/-2.0 dB, rms 2.71 down to 0.59;
  lock until 10 dB, coasting at 12, searching from 14; rms 0.51 to 0.55 (receiver noise) from 18 to
  31.75 dB, so the leak is below the noise in pit mode. Absolute dBm unknown (pit power unknown);
  fine for A/B comparisons (layouts, demodulators, gain loop).
- Next: bench.py mode that keeps the VTX in pit (no lease), keeps every recording, and scores PSNR
  against a strong clean card recording; the `damaged` share saturates (78 % at C/N 14.5, 100 % from
  12.2 dB down), so it cannot rank picture quality near threshold.
- Colour ladder with PSNR (`bench-runs/colour-ladder-231647`, ladder `card`, amp 1600, lid closed,
  reference `colour-225835/c1600` at 25 mW, PSNR of luma against its mean, 33.2 dB on itself):
  step 0/3/5/8/10/14/15/16 dB gave C/N 11.1/8.7/7.4/4.2/3.0/1.4/0.7/0.5 dB and PSNR median
  24.4/22.1/20.4/16.2/15.0/11.8/7.8/5.6 dB; coasting from 16 dB. About 1 dB of PSNR per dB down to
  10 dB, collapse at 15 dB. This is the baseline curve for receiver changes (demodulator, cleanup,
  C5 settings): a better receiver moves it right. Labelled video: colour-threshold-ladder.mp4.
  This ladder ran with the ESP32 on PC USB power, not the battery (BENCH-11): absolute numbers are
  suspect, comparisons between DSP variants on it are not.

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
