#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# One-time setup of a Fedora machine as the OpenVRX bench host.
# Run from the repo root:  bash tools/bench/setup-fedora.sh
set -euo pipefail

sudo dnf install -y git gcc make python3 python3-pip python3-pyserial \
    systemd-devel pkgconf-pkg-config arduino-cli || \
sudo dnf install -y git gcc make python3 python3-pip python3-pyserial systemd-devel pkgconf-pkg-config

# Rust (user install).
if ! command -v cargo >/dev/null; then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    . "$HOME/.cargo/env"
fi

# FX2 (blank Cypress loader, fx2lib loader, OpenVRX firmware) and serial
# ports for the user, without root.
sudo tee /etc/udev/rules.d/60-openvrx.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="04b4", ATTR{idProduct}=="8613", MODE="0660", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="608c", MODE="0660", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", MODE="0660", TAG+="uaccess"
# PM QoS for the FX2 stream: holding a 20 us request keeps idle C-states short
# enough that the FX2's 2 KB FIFO cannot overflow while the PC idles.
SUBSYSTEM=="misc", KERNEL=="cpu_dma_latency", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"

# Host tools.
(cd host && CARGO_TARGET_DIR=target/main cargo build --release -p openvrx-cli)

cat <<'EOF'

Done. Log out and back in once (serial port group). Then:
  - C5 console:   /dev/ttyACM0 (python3 tools/c5cmd.py /dev/ttyACM0 status)
  - attenuator ESP32: /dev/ttyUSB0 (CP2102/CH340 on the NodeMCU)
  - sweep:        python3 tools/bench/bench.py --help
For flashing the C5 here, install ESP-IDF v6.0.2 (docs/BENCH.md).
EOF
