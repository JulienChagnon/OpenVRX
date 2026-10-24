# Credits

OpenVRX builds on **C5VRX by Twotoz and the C5VRX contributors**
(<https://github.com/Twotoz/C5VRX>, project site and Discord invite at
<https://twotoz.github.io/C5VRX/>), GPL-3.0. C5VRX worked out how to get
receiver I/Q out of the ESP32-C5's modem diagnostic bus, how to hold the
radio in a clean receive state, and much of the signal model used here.

Material adapted from C5VRX at commit
`6bc9e4bbb5bef5948d42a065da27eae39315ad2d`, with the changes listed in each
file's header:

| OpenVRX file | Adapted from |
|---|---|
| `firmware/c5/main/rf.c`, `rf.h` | `main/rf.c`: receive lock, forced gain, tuning, download reboot |
| `firmware/c5/main/phy.c`, `phy.h` | `main/phy.c`: PHY register access and filter controls |
| `firmware/c5/main/logic/arc_gain.c`, `arc_gain.h` | `main/arc_phy.c`: receive gain table model |
| `firmware/c5/main/lanes_hw.c` | MODEM_DIAG lane routing |
| `firmware/c5/sdkconfig.defaults`, `main/CMakeLists.txt` | Wi-Fi/PHY configuration and the pinned PHY library check |
| `tools/flash.py` | `tools/flash.py`, `tools/enter_download.py`: flashing a running board without a line reset |
| `host/crates/openvrx-sim` (`tx.rs`, `rx.rs`, `config.rs`) | `research/sim.py`: FM transmitter, channel and receiver model |
| `host/crates/openvrx-dsp` (`frontend.rs`, `cn.rs`, `probe.rs`, `testsig.rs`) | C/N estimate, sampling-phase probe and test signal ideas |
| `host/crates/openvrx-core/src/lanes.rs` | lane layouts, including the folded "ultrafine" window |
| `lab/openvrx_lab` | signal model and metrics from `research/sim.py` |

Third-party code vendored as is:

- [libfx2](https://github.com/whitequark/libfx2) (0BSD) in
  `firmware/fx2/third_party/libfx2`, commit
  `16cdf948a964dbc3506f42753a335db91b63a39b`.
