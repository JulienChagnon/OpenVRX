# OpenVRX

OpenVRX is an analog 5.8 GHz FPV video receiver where the PC does the
decoding. An ESP32-C5 acts as the radio: it tunes the channel and streams its
raw I/Q samples, plus a sample clock, over a few GPIO lanes into a cheap
FX2LP (CY7C68013A) USB board. The PC demodulates the FM video, recovers sync,
decodes colour, combines several receivers when you have them, cleans up the
picture and shows it on the laptop screen or on Fat Shark HD3 goggles over
HDMI.

OpenVRX is based on [C5VRX](https://github.com/Twotoz/C5VRX) by Twotoz and
the C5VRX contributors, which showed that the C5's modem diagnostic bus can
carry receiver I/Q and turned that into a working receiver. See
[CREDITS.md](CREDITS.md) for what was adapted. The C5VRX site,
<https://twotoz.github.io/C5VRX/>, has the project write-up and the
community Discord invite.

## Status

Early but working on real hardware: a Seeed XIAO ESP32-C5 wired to an FX2
board with the 4-bit 4x4top lane layout streams 26.67 MS/s over USB and the
host decodes a sharp NTSC picture from a real quad. Multi-receiver diversity,
the other lane layouts and the HD3 path still need bench time.

## Layout

| Path | What |
|---|---|
| `firmware/c5` | ESP32-C5 radio head (ESP-IDF 6.0.2): tuning, gain, lane routing, sample clock, serial console |
| `firmware/fx2` | FX2LP slave FIFO firmware (SDCC, libfx2), loaded into RAM by the host |
| `host/crates/openvrx-core` | shared types: lane formats, video standards, line and frame records |
| `host/crates/openvrx-hw` | FX2 and C5 control, streaming, calibration, pairing |
| `host/crates/openvrx-dsp` | front end, FM demodulators, sync and time base correction, quality marks |
| `host/crates/openvrx-video` | diversity combiner, colour decoding, repair, deinterlacing, denoise |
| `host/crates/openvrx-format` | recording format, reader, replay, rolling buffer |
| `host/crates/openvrx-sim` | transmitter and channel simulator that writes recordings |
| `host/apps/openvrx` | the live receiver app (wgpu/egui) |
| `host/apps/openvrx-cli` | offline tools: inspect and decode recordings |
| `lab` | Python reference decoder and experiments |
| `tools` | flashing and console helpers |
| `docs` | protocol, hardware wiring, pinouts, recording format, usage |

## Getting started

Wiring for each lane layout is in [docs/HARDWARE.md](docs/HARDWARE.md) and
board pinouts in [docs/PINOUTS.md](docs/PINOUTS.md). Every command, for the
boards and the app, is in [docs/USAGE.md](docs/USAGE.md). The short version:

    # host (Windows, Rust GNU toolchain)
    cd host; . ..\tools\env.ps1
    cargo build --release -p openvrx

    # C5: build with ESP-IDF, flash with the helper (never plain idf.py flash on a running board)
    python tools\flash.py COM9
    python tools\c5cmd.py COM9 "tune 5880" "lanes 4x4top" "clk 9" save

    # run with only the FX2 on USB
    host\target\release\openvrx.exe --lanes 4x4top --clk 9 --mhz 5880

## License

GPL-3.0-or-later, the same as C5VRX. See [LICENSE](LICENSE).
