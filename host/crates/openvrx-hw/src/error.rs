// SPDX-License-Identifier: GPL-3.0-or-later
//! Error type shared by every module of the crate.

use serde::{Deserialize, Serialize};

/// Why a USB transfer failed (a host-independent copy of the backend's
/// transfer status, so the streaming logic can be tested with mocks).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum XferError {
    /// Cancelled by the host (stop, restart or timeout).
    Cancelled,
    /// The endpoint returned STALL.
    Stall,
    /// The device went away.
    Disconnected,
    /// Bus or protocol fault (CRC, babble, controller error).
    Fault,
    /// The request was rejected by the OS as invalid.
    InvalidArgument,
    /// Anything else, with the OS description.
    Other(String),
}

impl std::fmt::Display for XferError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            XferError::Cancelled => f.write_str("cancelled"),
            XferError::Stall => f.write_str("endpoint stalled"),
            XferError::Disconnected => f.write_str("device disconnected"),
            XferError::Fault => f.write_str("bus fault"),
            XferError::InvalidArgument => f.write_str("invalid argument"),
            XferError::Other(s) => write!(f, "{s}"),
        }
    }
}

impl From<nusb::transfer::TransferError> for XferError {
    fn from(e: nusb::transfer::TransferError) -> Self {
        use nusb::transfer::TransferError as T;
        match e {
            T::Cancelled => XferError::Cancelled,
            T::Stall => XferError::Stall,
            T::Disconnected => XferError::Disconnected,
            T::Fault => XferError::Fault,
            T::InvalidArgument => XferError::InvalidArgument,
            other => XferError::Other(other.to_string()),
        }
    }
}

/// Errors returned by `openvrx-hw`.
#[derive(Debug, thiserror::Error)]
pub enum HwError {
    /// USB stack error outside a transfer (open, claim, enumerate).
    #[error("USB: {0}")]
    Usb(String),
    /// A control, bulk or interrupt transfer failed.
    #[error("USB transfer: {0}")]
    Transfer(XferError),
    /// No matching device or port.
    #[error("not found: {0}")]
    NotFound(String),
    /// Something did not happen in time.
    #[error("timed out: {0}")]
    Timeout(String),
    /// The device answered something the protocol does not allow.
    #[error("protocol: {0}")]
    Protocol(String),
    /// Intel HEX or EEPROM image parse error.
    #[error("firmware image line {line}: {msg}")]
    Image { line: usize, msg: String },
    /// The FX2 firmware image is not available.
    #[error("FX2 firmware not available: {0}")]
    NoFirmware(String),
    /// Read-back did not match what was written.
    #[error("verify failed: {0}")]
    Verify(String),
    /// The C5 answered `ERR`.
    #[error("C5 `{command}` failed: {code} {text}")]
    C5 {
        command: String,
        code: String,
        text: String,
    },
    /// Serial port error.
    #[error("serial port: {0}")]
    Serial(String),
    /// Invalid argument from the caller.
    #[error("invalid argument: {0}")]
    Invalid(String),
    /// The link went away and the operation cannot continue.
    #[error("disconnected: {0}")]
    Disconnected(String),
    /// The worker was asked to stop.
    #[error("stopped")]
    Stopped,
    /// Operating system I/O error.
    #[error("I/O: {0}")]
    Io(#[from] std::io::Error),
}

impl From<nusb::Error> for HwError {
    fn from(e: nusb::Error) -> Self {
        match e.kind() {
            nusb::ErrorKind::Disconnected => HwError::Disconnected(e.to_string()),
            nusb::ErrorKind::NotFound => HwError::NotFound(e.to_string()),
            _ => HwError::Usb(e.to_string()),
        }
    }
}

impl From<nusb::transfer::TransferError> for HwError {
    fn from(e: nusb::transfer::TransferError) -> Self {
        HwError::Transfer(e.into())
    }
}

impl From<serialport::Error> for HwError {
    fn from(e: serialport::Error) -> Self {
        match e.kind() {
            serialport::ErrorKind::NoDevice => HwError::Disconnected(e.to_string()),
            serialport::ErrorKind::Io(kind) => HwError::Io(std::io::Error::new(kind, e.description)),
            _ => HwError::Serial(e.to_string()),
        }
    }
}

/// Result alias.
pub type Result<T, E = HwError> = std::result::Result<T, E>;
