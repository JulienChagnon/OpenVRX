// SPDX-License-Identifier: GPL-3.0-or-later
//! ESP32-C5 radio head control over its USB Serial/JTAG console.

pub mod client;
pub mod proto;
pub mod transport;

pub use client::{C5Client, C5Event, ClientConfig};
pub use proto::{BoardKind, C5Settings, C5Status, FilterCode, PinMap, Reply, SelftestMode, VerInfo};
pub use transport::{find_c5_ports, PortConnector, PortSelector, SerialConnector, SerialLink, SerialPortFound};
