# Bench logs

Context and results of hardware bench sessions, and the backlog of improvements they surface.
Recordings, decoded frames and other raw outputs never go here (they stay in `bench-runs/` beside the
repo, gitignored); this folder holds text only.

| File | What |
|---|---|
| [IMPROVEMENTS.md](IMPROVEMENTS.md) | Every potential C5, FX2, DSP or bench-tool improvement a session surfaced: evidence, idea, how to verify, status |
| `YYYY-MM-DD-topic.md` | One file per session: setup, what was run (exact commands), numbers, what it means |

`docs/testing/README.md` keeps the short running log and the layout matrix; a session there links to its
file here for the detail.

## Writing a session file

1. Setup: source (camera or ESP32 pattern and which), VTX channel and power, fixed and step attenuation,
   estimated level at the C5, C5 firmware and settings (`c5cmd.py PORT ver status`), host commit.
2. Runs: the exact `bench.py` / `openvrx-cli` command lines and the result folder name in `bench-runs/`.
3. Numbers: the `score` JSON (lock, C/N, clip, damaged, PSNR), plus anything from `decode` stats that
   matters (relocks, coasted lines, I/Q DC, carrier offset).
4. Observations, then each new improvement idea as an entry in IMPROVEMENTS.md (link both ways).

## Improvement entries

Ids are `C5-n`, `FX2-n`, `DSP-n`, `BENCH-n`. Each entry: evidence (session file and numbers), the idea,
how to verify it on the bench, and a status: `open`, `investigating`, `implemented (not verified)`,
`verified`, `rejected` (with the reason). Implemented items name the change; verified items name the run
that proved them.
