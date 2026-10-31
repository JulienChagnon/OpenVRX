// SPDX-License-Identifier: GPL-3.0-or-later
//! FX2LP USB bridge: discovery, firmware upload, vendor requests, status,
//! streaming engine, watch mode and throughput bench.

pub mod bench;
pub mod endpoint;
pub mod firmware;
pub mod image;
pub mod protocol;
pub mod stream;
pub mod usb;

pub use bench::{run_bench, BenchConfig, BenchMode, BenchResult, PatternCheck};
pub use endpoint::{InCompletion, InEndpoint};
pub use image::{FirmwareImage, IicHeader, IicImage, Segment};
pub use protocol::{ControlIo, Fx2Info, Fx2Requests, Fx2Status, StartFlags, StatusFlags};
pub use stream::{
    block_channel, BlockSink, Fx2Stream, StreamConfig, StreamEvent, StreamLink, StreamPort, StreamStats, WatchLedger,
};
pub use usb::{
    list_fx2, upload_all, upload_and_wait, Fx2Device, Fx2Found, Fx2Kind, Fx2Selector, NusbPort, UsbLocation,
};
