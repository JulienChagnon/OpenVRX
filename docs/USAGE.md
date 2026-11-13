# Using OpenVRX: commands for every board and the app

All commands are for PowerShell on Windows. Paths assume the repo at
`D:\SOFTWARE DEV\OpenVRX`. Build first (once, and after code changes):

    cd "D:\SOFTWARE DEV\OpenVRX\host"; . ..\tools\env.ps1
    $env:CARGO_TARGET_DIR = "target\main"; cargo build --release -p openvrx; cargo build --release -p openvrx-hw --examples

Built programs land in `host\target\main\release\` (the app) and
`host\target\main\release\examples\` (the FX2 tools).

## The receiver app

    & "D:\SOFTWARE DEV\OpenVRX\host\target\main\release\openvrx.exe" --lanes 4x4top --clk 9 --mhz 5880

Only the FX2 goes on the PC's USB (FX2-only, the default). The C5 runs on
its own power with the settings it saved, so tell the app what those are:

| Option | Meaning |
|---|---|
| `--lanes PRESET` | lane layout the C5 saved: 8x8, 7x7, 6x6, 5x5, 4x4top, 4x4fine, 4x4ultra |
| `--clk DIV` | sample clock divider the C5 saved: 6 (40 MS/s), 9 (26.67), 12 (20), 18 (13.33) |
| `--mhz MHZ` | the C5's saved channel, for the OSD and recordings only (the app cannot read it in FX2-only mode) |
| `--control c5` | the C5 console is on USB too: the app tunes, sets gain, calibrates the sampling phase |
| `--hd3` | Fat Shark HD3 mode: fullscreen, 5:3 geometry, goggle-safe OSD |
| `--fullscreen` | start fullscreen |
| `--layout 1\|2\|4` | number of views |
| `--present immediate\|mailbox\|fifo` | swap chain mode (latency vs tearing) |
| `--source test` | built-in test patterns, no hardware |
| `--run-for SECONDS` | run, print latency statistics, exit |
| `--settings PATH` | settings file (default `%APPDATA%\OpenVRX\settings.json`) |

### Keys (F1 shows them in the app)

| Key | Action |
|---|---|
| F1 | help |
| Tab | open/close the side panels (the menu: receivers, video, display, recording) |
| O | OSD on/off |
| Space | cleanup on/off (raw picture) |
| R | start/stop recording |
| H | HD3 mode |
| V | 1, 2 or 4 views |
| A | aspect (fit, fill, stretch) |
| S | scaling filter |
| D | damage overlay |
| P / L | present mode / present trigger |
| C | calibrate sampling phase (needs `--control c5`) |
| X | side by side: the live picture and the bench ESP32 test card it should show (`--compare-pattern NAME`, default card; `--compare` starts in it) |
| [ and ] | channel down/up (needs `--control c5`) |
| F5 | reset latency statistics |
| F11 | fullscreen |
| Esc | close help/panels, leave fullscreen |
| Ctrl+Q | quit |

The side panels (Tab) are the menu. In FX2-only mode the channel and gain
controls there do nothing; change them on the C5 (below).

Video presets (Tab, video): `raw` (demodulation, sync and bob only),
`low_latency` (the default: comb filter, repair, edge-directed
deinterlace), `balanced` (plus a light temporal denoise), `quality` (3D comb,
motion-compensated denoise) and `learned`: the raw picture through a
learned denoise filter on the GPU. It reads only the current and the two
previous fields, so it
adds no delay; the app picks the filter that matches the steady OSD setting
(I). Space still switches all cleanup off.

## The C5 radio head (`tools/c5cmd.py`, `tools/flash.py`)

Plug the C5's USB-C into the PC (it shows as a COM port, COM9 here). Use
the ESP-IDF Python, which has pyserial:

    $py = "C:\Espressif\tools\python\v6.0.2\venv\Scripts\python.exe"
    & $py tools\c5cmd.py COM9 ver status pins

| Task | Command |
|---|---|
| set channel, layout, clock and keep them | `& $py tools\c5cmd.py COM9 "tune 5880" "lanes 4x4top" "clk 9" save status` |
| fixed gain (index, default 64) | `& $py tools\c5cmd.py COM9 "gain 60" save` |
| reboot | `& $py tools\c5cmd.py COM9 reboot` |
| flash new firmware | `idf.py build` in `firmware\c5`, then `& $py tools\flash.py COM9` |
| flash a board already in the ROM loader | `& $py tools\flash.py COM9 --no-dl` |

Never use `idf.py flash` or a serial tool that toggles DTR/RTS on a running
C5: the line reset hangs it until it is unplugged. `status` ends with
`boot=ok`, or `boot=<step>` when the previous boot stalled and the C5
started in safe mode on default settings. Full command list:
`docs/PROTOCOL.md` section 1. Raceband: R1 5658, R2 5695, R3 5732,
R4 5769, R5 5806, R6 5843, R7 5880, R8 5917.

## The FX2 bridge (tools in `host\target\main\release\examples\`)

The FX2 keeps its firmware in RAM; the app and these tools load it after
every plug-in.

| Tool | What it does |
|---|---|
| `fx2_probe.exe` | loads the firmware, runs a 5 s USB throughput test with the FX2's own clock |
| `fx2_diag.exe` | wiring check: control ties, data lanes, FIFO tests with the internal and C5 clock |
| `fx2_diag.exe --c5` | only the C5-clock stream test (run on a freshly plugged FX2) |
| `fx2_diag.exe --live` | pin levels every 0.5 s for 60 s, for re-seating wires |
| `fx2_diag.exe --find-ifclk` | finds the IFCLK header pin (jumper it to PD1) |

Wiring tables: `docs/HARDWARE.md`. Board pinouts: `docs/PINOUTS.md`.

## Offline and bench tools (`openvrx-cli`)

    openvrx-cli inspect REC [OUT_DIR]            lane activity, I/Q histograms, FM strip image
    openvrx-cli decode  REC [OUT_DIR] [FRAMES]   decoded frames as PNG
    openvrx-cli record  OUT_BASE --lanes 4x4top --clk 9 [--seconds 5] [--mhz 5880] [--gain 56]
    openvrx-cli score   REC [--ref REC] [--frames 30] [--skip 10]

`record` captures FX2-only without the app window; `score` prints one JSON
line (lock, C/N, clipping, frames, PSNR against a reference recording).
Attenuation sweeps built on them: `docs/BENCH.md`. Decoded frames and
recordings stay local.
