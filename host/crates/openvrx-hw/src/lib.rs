// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX hardware layer: everything between the radio heads and the DSP.
//!
//! # Modules and public API
//!
//! - [`fx2`]: FX2LP USB bridges (PROTOCOL.md section 2).
//!   - Discovery: [`fx2::list_fx2`] returns [`fx2::Fx2Found`] for every
//!     bootloader-ID or OpenVRX device with its [`fx2::UsbLocation`]
//!     (controller name, port chain, speed) and bound driver.
//!   - Firmware: [`fx2::FirmwareImage`] (Intel HEX parser and writer),
//!     [`fx2::IicImage`] (C2 EEPROM format), [`fx2::firmware`] (the image
//!     embedded at build time, or one loaded from a path),
//!     [`fx2::upload_and_wait`] / [`fx2::upload_all`] (0xA0 upload, wait for
//!     re-enumeration on the same port, assign a board ID).
//!   - Requests: the [`fx2::Fx2Requests`] extension trait on any
//!     [`fx2::ControlIo`] (GET_INFO, START with [`fx2::StartFlags`], STOP,
//!     GET_STATUS, RESET_COUNTERS, SET_WATCH, LED, SET_BOARD_ID, RAM load,
//!     EEPROM read/write/program with verify), [`fx2::Fx2Status`] and
//!     [`fx2::Fx2Info`] decoding.
//!   - Streaming: [`fx2::Fx2Stream`] runs one capture thread per FX2 over a
//!     [`fx2::StreamPort`] ([`fx2::NusbPort`] for hardware,
//!     [`fx2::endpoint::sim::SimFx2`] for simulation), delivering
//!     `openvrx_core::RawBlock`s on a drop-oldest channel
//!     ([`fx2::block_channel`]), with [`fx2::StreamEvent`]s,
//!     [`fx2::StreamStats`] and watch-mode bookkeeping
//!     ([`fx2::WatchLedger`]). [`fx2::stream::capture_once`] takes short
//!     captures without a thread.
//!   - Bench: [`fx2::run_bench`] streams from several boards at once and
//!     reports MB/s, losses, test-counter breaks and completion jitter.
//! - [`c5`]: ESP32-C5 radio heads over USB Serial/JTAG (PROTOCOL.md
//!   section 1). [`c5::find_c5_ports`], [`c5::C5Client`] (typed commands,
//!   reply matching, events, reconnect and state push), [`c5::C5Settings`]
//!   (the state the host owns), and the transport traits with a system
//!   port and a simulated radio head ([`c5::transport::sim::SimC5`]).
//! - [`calib`]: the fourth-difference phase probe, the calibration
//!   procedure ([`calib::calibrate`] over [`calib::CalibIo`]) and the lane
//!   self-test analyser ([`calib::analyze_walk`]).
//! - [`receiver`]: [`receiver::Receiver`] pairs one C5 with one FX2, owns
//!   the settings, runs calibration and the self-test, and restarts the
//!   stream without changing the block channel; [`receiver::auto_pair`]
//!   pairs boards by self-test pattern.
//! - [`control`]: pure control logic: [`control::GainLoop`],
//!   [`control::Afc`], the channel table and [`control::run_scan`],
//!   [`control::WatchManager`] for 3+ receivers.
//! - [`tracker`]: monopulse antenna tracker ([`tracker::Tracker`]) and a
//!   Pololu Maestro driver ([`tracker::maestro::Maestro`]).
//! - [`power`]: MMCSS "Pro Audio" registration, timer resolution and the
//!   USB selective suspend check.
//!
//! # Threads
//!
//! - One capture thread per FX2 (`openvrx-usb-rxN`, MMCSS "Pro Audio") plus
//!   a light status thread per stream session (EP1 or GET_STATUS polling).
//! - One I/O thread per C5 console (`openvrx-c5-...`).
//! - Everything else (control loops, calibration, self-test, tracker) is
//!   called from the application's control thread; calibration and the
//!   self-test pause the stream and block that thread for up to a few
//!   seconds.
//!
//! # Byte accounting
//!
//! `RawBlock::first_byte` counts every byte received from USB, including
//! blocks dropped on the host because the consumer was slow, so a jump
//! means host loss. Device-side loss (FIFO full) and recovery gaps are
//! reported as events with estimates; they never reached the host.

pub mod c5;
pub mod calib;
pub mod control;
pub mod error;
pub mod fx2;
pub mod power;
pub mod receiver;
pub mod tracker;

pub use error::{HwError, Result, XferError};
