# OpenVRX architecture

```text
antenna -> ESP32-C5 (tuner, raw I/Q lanes + sample clock) -> FX2LP (USB 2.0) -> PC
                                                                                |
  capture (openvrx-hw) -> unpack/demod/TBC (openvrx-dsp) -> combine, colour, deinterlace,
  repair, denoise (openvrx-video) -> display (openvrx app) / record (openvrx-format)
```

## Repository layout

| Path | Language | Content |
|---|---|---|
| `firmware/c5/` | C, ESP-IDF v6 | Radio head: tune, gain, route MODEM_DIAG bits to pads, PARLIO TX sample clock, slip, 802.11p, self-test, console protocol |
| `firmware/fx2/` | C, SDCC + libfx2 | Slave FIFO streaming firmware, status endpoint, watch mode, throughput tests, WinUSB descriptors |
| `host/` | Rust workspace | Everything that runs on the PC |
| `lab/` | Python | Reference decoder, signal simulator, experiments, golden vectors |
| `docs/` | Markdown | Protocols, formats, hardware, bring-up |
| `tools/` | scripts | Helpers (flashing, driver setup) |

## Host crates

| Crate | Role |
|---|---|
| `openvrx-core` | Shared types: standards, lane formats, records, quality, clock |
| `openvrx-format` | `.openvrx` recording reader/writer, events, rolling RAM buffer |
| `openvrx-sim` | Synthetic CVBS -> FM -> C5 channel -> noise/fading -> lanes, writes `.openvrx` with truth |
| `openvrx-dsp` | Per-receiver DSP: unpack, DC, AFC, channel filter, FM demodulators, click repair, video filter, sync, time-base correction, quality, phase probe, pre-detection combining |
| `openvrx-video` | Diversity combiner, field assembly, colour decode (NTSC/PAL, 1D/2D/3D comb), dropout repair, deinterlacers, temporal denoise, low-latency streaming pipeline |
| `openvrx-hw` | FX2 discovery/upload/streaming (nusb, WinUSB), C5 serial control, phase calibration, lane self-test, gain/AFC loops, throughput bench |
| `apps/openvrx` | Live receiver app (wgpu display, OSD, recording, controls) |
| `apps/openvrx-cli` | Record, inspect, decode to frames, score and timeline of recordings |

Dependencies point downward only: core <- format, sim, dsp, video, hw <- apps.

## Threads (live)

```text
[USB thread per FX2]  nusb queued bulk IN -> RawBlock -> SPSC ring
[DSP thread per rx]   RxDsp::push_bytes -> LineRecord
[Video thread]        Combiner -> VideoPipeline (line streaming) -> frame buffer
[Display thread]      latest rows -> GPU texture -> present (immediate/mailbox)
[Recorder thread]     RawBlock copies -> disk, rolling buffer
[Control thread]      C5 consoles, gain loop, AFC, calibration, scan
```

Hot threads get MMCSS "Pro Audio" priority on Windows. Channels are bounded;
when a consumer falls behind, the oldest work is dropped and counted, never
blocking capture.

## Latency rules

- The display never waits for a whole frame when a lower-latency mode is
  selected: the video pipeline updates rows as lines arrive and the display
  presents the newest rows (`present_every_lines`).
- Live filters use past fields only; nothing in the live path waits for a
  later field.
- Every stage stamps `clock::now_ns()`; the OSD shows capture-to-present
  latency.

## Picture safety rules (live)

- Repair only modifies samples the damage mask flags.
- A region is filled from the past for at most 3 consecutive fields, then
  the raw (noisy) picture shows again: breakup is the pilot's range warning.
- The per-receiver C/N bars are always visible while any cleanup is on.
- One key toggles all cleanup off (raw view).
