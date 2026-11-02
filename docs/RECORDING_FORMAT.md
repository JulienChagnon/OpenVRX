# OpenVRX recording format (version 1)

A recording keeps the raw lane words exactly as the FX2 delivered them, so
it can be decoded again later with better code. The simulator writes the
same format, so every tool runs on simulated and real captures alike.

## Files

For a recording named `NAME`:

| File | Content |
|---|---|
| `NAME.openvrx.json` | Sidecar: everything needed to decode (written at start, rewritten at close) |
| `NAME.rx0.bin`, `NAME.rx1.bin`, ... | Raw bytes per receiver, exactly as received |
| `NAME.events.jsonl` | One JSON object per line, appended live (crash-safe) |
| `NAME.truth.f32` | Simulator only: clean composite video, little-endian f32 IRE, one value per *input sample* of rx0 (see `truth`) |

Opening `NAME.openvrx.json` (or any of the files) opens the recording.

## Sidecar

```json
{
  "format": "openvrx-recording",
  "version": 1,
  "created_utc": "2026-10-29T21:14:03Z",
  "software": "openvrx 0.1.0",
  "source": "hardware",
  "notes": "",
  "closed_cleanly": true,
  "receivers": [
    {
      "rx": 0,
      "file": "NAME.rx0.bin",
      "bytes": 1600000000,
      "sample_format": {
        "preset": "7x7",
        "lanes": { "word_bytes": 2, "encoding": { "kind": "iq",
          "i": { "word_msb": 15, "adc_bits": [9,8,7,6,5,4,3] },
          "q": { "word_msb": 7,  "adc_bits": [9,8,7,6,5,4,3] } } },
        "divider": 18
      },
      "sample_rate_hz": 13333333.333,
      "channel_mhz": 5732,
      "offset_khz": 0,
      "gain": 64,
      "p11": true,
      "board": "xiao",
      "c5_fw": "0.1.0",
      "fx2_fw": "0.1.0",
      "fx2_serial": "A1B2C3D4",
      "ifclk_inverted": false,
      "watch_n": 1,
      "start_host_ns": 123456789,
      "antenna": "omni RHCP"
    }
  ],
  "standard_hint": "ntsc",
  "sim": null
}
```

`sample_format` is serde's form of `openvrx_core::SampleFormat`.

For simulated recordings `source` is `"sim"` and `sim` holds the generator
settings (C/N, fading, deviation, standard, seeds, picture source) plus:

```json
"truth": { "file": "NAME.truth.f32", "rx": 0, "unit": "ire",
           "delay_samples": 0.0 }
```

`delay_samples` is the group delay the simulator's channel adds, so a
decoder output at input sample `n` compares with `truth[n - delay]`.

## Events (`NAME.events.jsonl`)

Every event has `rx` (or null for session-wide), `host_ns`, `kind`, and
`byte` (offset in that receiver's .bin when the event was observed; for
commands this is the stream position at the time the command was sent,
since the exact sample a C5 command takes effect is found later from the
data).

| kind | extra fields |
|---|---|
| `gain` | `index` |
| `tune` | `mhz`, `offset_khz` |
| `clk` | `divider` |
| `slip` | `count` |
| `probe` | `score`, `verdict` (`clean`/`mixed`) |
| `fx2_status` | the decoded status block |
| `usb_gap` | `lost_bytes` (estimated), `reason` |
| `transfer` | `bytes` (every 64th bulk completion: timing for latency analysis) |
| `marker` | `text` (user marker key) |
| `c5` | `line` (raw console line) |

## Rolling buffer

The live app keeps the last 30 s (configurable) of raw blocks in RAM per
receiver. Pressing record writes the buffer first, then continues live, so
a recording can start before the button was pressed.

## Size

13.33 MS/s x 2 bytes = 26.7 MB/s = 1.6 GB per minute per receiver.
