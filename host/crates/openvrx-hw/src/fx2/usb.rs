// SPDX-License-Identifier: GPL-3.0-or-later
//! Real USB access through `nusb` (WinUSB on Windows): discovery with the
//! bus/port location, opening, endpoints, firmware upload with
//! re-enumeration matching, and a reconnecting [`StreamPort`].

use crate::error::{HwError, Result, XferError};
use crate::fx2::endpoint::{InCompletion, InEndpoint};
use crate::fx2::image::FirmwareImage;
use crate::fx2::protocol::{
    board_serial, ControlIo, Fx2Info, Fx2Requests, BOOTLOADER_IDS, EP_STATUS, EP_STREAM, OPENVRX_PID, OPENVRX_VID,
};
use crate::fx2::stream::{StreamLink, StreamPort};
use nusb::transfer::{Bulk, BulkOrInterrupt, ControlIn, ControlOut, ControlType, In, Interrupt, Recipient};
use nusb::MaybeFuture;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Timeout for vendor control requests.
pub const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);
/// How long to wait for the OpenVRX device after a RAM upload (PROTOCOL.md 3).
pub const RENUMERATION_TIMEOUT: Duration = Duration::from_secs(5);

/// Where a device is plugged in, so the UI can show which controller and
/// port each board uses (one full-rate receiver per USB 2.0 link).
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct UsbLocation {
    /// OS bus / host controller identifier.
    pub bus_id: String,
    /// Port numbers from the root hub down.
    pub port_chain: Vec<u8>,
    /// Host controller name as Device Manager shows it, when known.
    pub controller: Option<String>,
    /// xHCI, EHCI, ... when known.
    pub controller_type: Option<String>,
    /// Negotiated speed ("high" is required for streaming).
    pub speed: Option<String>,
}

impl UsbLocation {
    /// True when both describe the same physical port.
    pub fn same_port(&self, other: &UsbLocation) -> bool {
        !self.bus_id.is_empty() && self.bus_id == other.bus_id && self.port_chain == other.port_chain
    }

    /// Port chain as `2.3.1`.
    pub fn port_path(&self) -> String {
        self.port_chain
            .iter()
            .map(|p| p.to_string())
            .collect::<Vec<_>>()
            .join(".")
    }

    /// True when behind at least one hub (shares a link with its siblings).
    pub fn behind_hub(&self) -> bool {
        self.port_chain.len() > 1
    }
}

impl std::fmt::Display for UsbLocation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "bus {}", if self.bus_id.is_empty() { "?" } else { &self.bus_id })?;
        if let Some(c) = &self.controller {
            write!(f, " ({c})")?;
        }
        if !self.port_chain.is_empty() {
            write!(f, " port {}", self.port_path())?;
        }
        if let Some(s) = &self.speed {
            write!(f, ", {s} speed")?;
        }
        Ok(())
    }
}

/// What a discovered FX2 is running.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Fx2Kind {
    /// Blank chip or another firmware that accepts the `0xA0` loader.
    Bootloader,
    /// OpenVRX streaming firmware.
    Openvrx,
}

/// A discovered FX2.
#[derive(Clone, Debug)]
pub struct Fx2Found {
    pub kind: Fx2Kind,
    pub vid: u16,
    pub pid: u16,
    pub serial: Option<String>,
    pub product: Option<String>,
    pub location: UsbLocation,
    /// Bound driver (Windows), e.g. "WinUSB".
    pub driver: Option<String>,
    /// OS identity of this enumeration (changes on re-enumeration).
    pub key: String,
    info: nusb::DeviceInfo,
}

impl Fx2Found {
    /// One line for logs and the UI.
    pub fn describe(&self) -> String {
        let what = match self.kind {
            Fx2Kind::Bootloader => "FX2 (no OpenVRX firmware)",
            Fx2Kind::Openvrx => "OpenVRX FX2",
        };
        format!(
            "{what} {:04x}:{:04x} serial {} on {}",
            self.vid,
            self.pid,
            self.serial.as_deref().unwrap_or("-"),
            self.location
        )
    }

    /// A hint when the bound driver will not let nusb open the device.
    pub fn driver_hint(&self) -> Option<String> {
        match self.driver.as_deref() {
            Some(d) if !d.eq_ignore_ascii_case("winusb") => Some(format!(
                "this FX2 is bound to the `{d}` driver; bind WinUSB to it once with Zadig so OpenVRX can upload firmware"
            )),
            _ => None,
        }
    }

    /// Opens the device and claims interface 0.
    pub fn open(&self) -> Result<Fx2Device> {
        let device = self.info.open().wait().map_err(|e| self.open_error(e))?;
        let iface = device.claim_interface(0).wait().map_err(|e| self.open_error(e))?;
        Ok(Fx2Device {
            _device: device,
            iface,
            found: self.clone(),
        })
    }

    fn open_error(&self, e: nusb::Error) -> HwError {
        let mut msg = format!("cannot open {}: {e}", self.describe());
        if let Some(h) = self.driver_hint() {
            msg.push_str("; ");
            msg.push_str(&h);
        }
        match e.kind() {
            nusb::ErrorKind::Disconnected => HwError::Disconnected(msg),
            _ => HwError::Usb(msg),
        }
    }
}

fn speed_name(s: nusb::Speed) -> &'static str {
    match s {
        nusb::Speed::Low => "low",
        nusb::Speed::Full => "full",
        nusb::Speed::High => "high",
        nusb::Speed::Super => "super",
        nusb::Speed::SuperPlus => "super+",
        _ => "unknown",
    }
}

fn device_key(d: &nusb::DeviceInfo) -> String {
    #[cfg(windows)]
    {
        d.instance_id().to_string_lossy().into_owned()
    }
    #[cfg(not(windows))]
    {
        format!("{}:{}", d.bus_id(), d.device_address())
    }
}

/// Classifies a VID:PID.
pub fn classify(vid: u16, pid: u16) -> Option<Fx2Kind> {
    if vid == OPENVRX_VID && pid == OPENVRX_PID {
        Some(Fx2Kind::Openvrx)
    } else if BOOTLOADER_IDS.contains(&(vid, pid)) {
        Some(Fx2Kind::Bootloader)
    } else {
        None
    }
}

/// Lists every FX2 the host can see (bootloader and OpenVRX IDs), with the
/// controller name and port of each.
pub fn list_fx2() -> Result<Vec<Fx2Found>> {
    let buses: Vec<nusb::BusInfo> = nusb::list_buses().wait().map(|b| b.collect()).unwrap_or_default();
    let mut out = Vec::new();
    for d in nusb::list_devices().wait()? {
        let Some(kind) = classify(d.vendor_id(), d.product_id()) else {
            continue;
        };
        let bus = buses.iter().find(|b| b.bus_id() == d.bus_id());
        let location = UsbLocation {
            bus_id: d.bus_id().to_string(),
            port_chain: d.port_chain().to_vec(),
            controller: bus.and_then(|b| b.system_name().map(str::to_string)),
            controller_type: bus.and_then(|b| b.controller_type()).map(|t| format!("{t:?}")),
            speed: d.speed().map(|s| speed_name(s).to_string()),
        };
        #[cfg(windows)]
        let driver = d.driver().map(str::to_string);
        #[cfg(not(windows))]
        let driver = None;
        out.push(Fx2Found {
            kind,
            vid: d.vendor_id(),
            pid: d.product_id(),
            serial: d.serial_number().map(str::to_string),
            product: d.product_string().map(str::to_string),
            location,
            driver,
            key: device_key(&d),
            info: d,
        });
    }
    out.sort_by(|a, b| (&a.location.bus_id, &a.location.port_chain).cmp(&(&b.location.bus_id, &b.location.port_chain)));
    Ok(out)
}

/// How to find an OpenVRX FX2 again (after a disconnect or re-enumeration).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Fx2Selector {
    /// Serial number string (8 hex digits from the board ID).
    Serial(String),
    /// Physical port.
    Location { bus_id: String, port_chain: Vec<u8> },
    /// The only OpenVRX FX2 present.
    Only,
}

impl Fx2Selector {
    pub fn matches(&self, f: &Fx2Found) -> bool {
        if f.kind != Fx2Kind::Openvrx {
            return false;
        }
        match self {
            Fx2Selector::Serial(s) => f.serial.as_deref().is_some_and(|x| x.eq_ignore_ascii_case(s)),
            Fx2Selector::Location { bus_id, port_chain } => {
                &f.location.bus_id == bus_id && &f.location.port_chain == port_chain
            }
            Fx2Selector::Only => true,
        }
    }

    /// Finds the matching device; `Only` fails when several are present.
    pub fn find(&self) -> Result<Fx2Found> {
        let all: Vec<Fx2Found> = list_fx2()?.into_iter().filter(|f| self.matches(f)).collect();
        match (self, all.len()) {
            (_, 0) => Err(HwError::NotFound(format!("no OpenVRX FX2 matching {self:?}"))),
            (Fx2Selector::Only, n) if n > 1 => Err(HwError::Invalid(format!(
                "{n} OpenVRX FX2 boards present; select one by serial or port"
            ))),
            _ => Ok(all.into_iter().next().expect("non-empty")),
        }
    }
}

/// An opened FX2 (interface 0 claimed).
pub struct Fx2Device {
    _device: nusb::Device,
    iface: nusb::Interface,
    found: Fx2Found,
}

impl Fx2Device {
    pub fn found(&self) -> &Fx2Found {
        &self.found
    }

    /// The EP2 bulk IN stream endpoint.
    pub fn stream_endpoint(&self) -> Result<NusbIn<Bulk>> {
        Ok(NusbIn {
            ep: self.iface.endpoint::<Bulk, In>(EP_STREAM)?,
        })
    }

    /// The EP1 interrupt IN status endpoint.
    pub fn status_endpoint(&self) -> Result<NusbIn<Interrupt>> {
        Ok(NusbIn {
            ep: self.iface.endpoint::<Interrupt, In>(EP_STATUS)?,
        })
    }

    /// A cheap clonable control handle for other threads.
    pub fn control(&self) -> Arc<dyn ControlIo> {
        Arc::new(NusbControl {
            iface: self.iface.clone(),
        })
    }
}

/// Vendor control requests through a claimed interface.
#[derive(Clone)]
pub struct NusbControl {
    iface: nusb::Interface,
}

fn map_xfer(e: nusb::transfer::TransferError) -> HwError {
    let x: XferError = e.into();
    match x {
        XferError::Disconnected => HwError::Disconnected("device disconnected".into()),
        x => HwError::Transfer(x),
    }
}

impl ControlIo for NusbControl {
    fn vendor_in(&self, request: u8, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
        self.iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length: len,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(map_xfer)
    }

    fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(map_xfer)
    }
}

impl ControlIo for Fx2Device {
    fn vendor_in(&self, request: u8, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
        NusbControl {
            iface: self.iface.clone(),
        }
        .vendor_in(request, value, index, len)
    }
    fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        NusbControl {
            iface: self.iface.clone(),
        }
        .vendor_out(request, value, index, data)
    }
}

/// An nusb IN endpoint behind the [`InEndpoint`] trait.
pub struct NusbIn<T: BulkOrInterrupt> {
    ep: nusb::Endpoint<T, In>,
}

impl<T: BulkOrInterrupt> InEndpoint for NusbIn<T> {
    fn max_packet_size(&self) -> usize {
        self.ep.max_packet_size()
    }

    fn submit(&mut self, buf: Vec<u8>) {
        self.ep.submit(nusb::transfer::Buffer::from(buf));
    }

    fn pending(&self) -> usize {
        self.ep.pending()
    }

    fn wait_complete(&mut self, timeout: Duration) -> Option<InCompletion> {
        if self.ep.pending() == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(10)));
            return None;
        }
        self.ep.wait_next_complete(timeout).map(|c| {
            let status = c.status.map_err(XferError::from);
            let mut data = c.buffer.into_vec();
            data.truncate(c.actual_len);
            InCompletion { data, status }
        })
    }

    fn cancel_all(&mut self) {
        self.ep.cancel_all();
    }

    fn clear_halt(&mut self) -> Result<()> {
        self.ep.clear_halt().wait().map_err(HwError::from)
    }
}

/// A random non-zero board ID (for boards without an ID in EEPROM).
pub fn random_board_id() -> u32 {
    use std::hash::{BuildHasher, Hasher};
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u64(openvrx_core::clock::now_ns());
    h.write_u32(std::process::id());
    let v = (h.finish() ^ (h.finish() >> 32)) as u32;
    if v == 0 {
        0x5043_5658
    } else {
        v
    }
}

/// Picks the device that appeared after an upload: a new key, preferring
/// the same physical port as the bootloader device. Returns the index into
/// `after`, or an error when the choice is ambiguous.
pub fn pick_renumerated(
    origin: &UsbLocation,
    before: &[String],
    after: &[(String, UsbLocation)],
) -> Result<Option<usize>> {
    let fresh: Vec<usize> = (0..after.len()).filter(|&i| !before.contains(&after[i].0)).collect();
    if let Some(&i) = fresh.iter().find(|&&i| after[i].1.same_port(origin)) {
        return Ok(Some(i));
    }
    // The port chain can differ between speeds or be unknown; fall back to
    // the only new device, on the same bus when possible.
    let same_bus: Vec<usize> = fresh
        .iter()
        .copied()
        .filter(|&i| !origin.bus_id.is_empty() && after[i].1.bus_id == origin.bus_id)
        .collect();
    let pool = if same_bus.is_empty() { &fresh } else { &same_bus };
    match pool.len() {
        0 => Ok(None),
        1 => Ok(Some(pool[0])),
        n => Err(HwError::Invalid(format!(
            "{n} new OpenVRX devices appeared at once; upload boards one at a time or reconnect them"
        ))),
    }
}

/// Result of bringing a bootloader FX2 up with OpenVRX firmware.
#[derive(Clone, Debug)]
pub struct UploadOutcome {
    pub device: Fx2Found,
    pub info: Fx2Info,
    /// Board ID the host assigned (None when the board already had one).
    pub assigned_board_id: Option<u32>,
}

/// Uploads firmware into a bootloader-ID device, waits (up to `timeout`)
/// for the OpenVRX device to enumerate on the same port, reads its info and
/// assigns a random board ID when the board has none.
pub fn upload_and_wait(target: &Fx2Found, fw: &FirmwareImage, timeout: Duration) -> Result<UploadOutcome> {
    fw.check_fx2_ram()?;
    let before: Vec<String> = list_fx2()?
        .into_iter()
        .filter(|f| f.kind == Fx2Kind::Openvrx)
        .map(|f| f.key)
        .collect();
    {
        let dev = target.open()?;
        dev.upload_firmware(fw)?;
    }
    let t0 = Instant::now();
    let found = loop {
        std::thread::sleep(Duration::from_millis(100));
        let now: Vec<Fx2Found> = list_fx2()?.into_iter().filter(|f| f.kind == Fx2Kind::Openvrx).collect();
        let keyed: Vec<(String, UsbLocation)> = now.iter().map(|f| (f.key.clone(), f.location.clone())).collect();
        if let Some(i) = pick_renumerated(&target.location, &before, &keyed)? {
            break now[i].clone();
        }
        if t0.elapsed() > timeout {
            return Err(HwError::Timeout(format!(
                "no OpenVRX FX2 appeared within {:.1} s after uploading to {}",
                timeout.as_secs_f64(),
                target.describe()
            )));
        }
    };
    // Windows may need a moment before the new interface can be opened.
    let mut last_err = None;
    let dev = loop {
        match found.open() {
            Ok(d) => break d,
            Err(e) => {
                if t0.elapsed() > timeout + Duration::from_secs(2) {
                    return Err(last_err.unwrap_or(e));
                }
                last_err = Some(e);
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    };
    let mut info = dev.get_info()?;
    let mut assigned = None;
    if info.board_id == 0 {
        let id = random_board_id();
        dev.set_board_id(id)?;
        info.board_id = id;
        assigned = Some(id);
        log::info!("assigned board ID {} to {}", board_serial(id), found.describe());
    }
    Ok(UploadOutcome {
        device: found,
        info,
        assigned_board_id: assigned,
    })
}

/// Uploads `fw` to every bootloader-ID FX2 (one at a time) and returns the
/// upload results; failures are returned per device.
pub fn upload_all(fw: &FirmwareImage) -> Result<Vec<(Fx2Found, Result<UploadOutcome>)>> {
    let targets: Vec<Fx2Found> = list_fx2()?
        .into_iter()
        .filter(|f| f.kind == Fx2Kind::Bootloader)
        .collect();
    Ok(targets
        .into_iter()
        .map(|t| {
            let r = upload_and_wait(&t, fw, RENUMERATION_TIMEOUT);
            (t, r)
        })
        .collect())
}

/// A [`StreamPort`] that finds the device by selector on every connect, so
/// a stream survives unplug/replug and firmware resets.
pub struct NusbPort {
    pub selector: Fx2Selector,
    /// Use EP1 for status (otherwise status is polled with `GET_STATUS`).
    pub use_status_endpoint: bool,
}

impl NusbPort {
    pub fn new(selector: Fx2Selector) -> NusbPort {
        NusbPort {
            selector,
            use_status_endpoint: true,
        }
    }
}

impl StreamPort for NusbPort {
    fn connect(&mut self) -> Result<StreamLink> {
        let found = self.selector.find()?;
        let dev = found.open()?;
        let bulk = dev.stream_endpoint()?;
        let status: Option<Box<dyn InEndpoint>> = if self.use_status_endpoint {
            match dev.status_endpoint() {
                Ok(s) => Some(Box::new(s)),
                Err(e) => {
                    log::warn!("EP1 status endpoint unavailable ({e}); polling GET_STATUS instead");
                    None
                }
            }
        } else {
            None
        };
        Ok(StreamLink {
            control: dev.control(),
            bulk: Box::new(bulk),
            status,
            description: found.describe(),
        })
    }

    fn describe(&self) -> String {
        format!("FX2 {:?}", self.selector)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn loc(bus: &str, chain: &[u8]) -> UsbLocation {
        UsbLocation {
            bus_id: bus.into(),
            port_chain: chain.to_vec(),
            ..Default::default()
        }
    }

    #[test]
    fn classify_ids() {
        assert_eq!(classify(0x04b4, 0x8613), Some(Fx2Kind::Bootloader));
        assert_eq!(classify(0x1d50, 0x608c), Some(Fx2Kind::Bootloader));
        assert_eq!(classify(0x1209, 0x0001), Some(Fx2Kind::Openvrx));
        assert_eq!(classify(0x303a, 0x1001), None);
    }

    #[test]
    fn renumeration_matching() {
        let origin = loc("1", &[2, 3]);
        let before = vec!["old".to_string()];
        // Same port wins even with another new device elsewhere.
        let after = vec![
            ("old".to_string(), loc("1", &[2, 3])),
            ("x".to_string(), loc("2", &[1])),
            ("y".to_string(), loc("1", &[2, 3])),
        ];
        assert_eq!(pick_renumerated(&origin, &before, &after).unwrap(), Some(2));
        // Nothing new yet.
        assert_eq!(pick_renumerated(&origin, &before, &after[..1]).unwrap(), None);
        // Unknown port: the only new device on the same bus.
        let after = vec![("x".to_string(), loc("2", &[1])), ("z".to_string(), loc("1", &[4]))];
        assert_eq!(pick_renumerated(&origin, &before, &after).unwrap(), Some(1));
        // Two new devices on the same bus, neither on the port: ambiguous.
        let after = vec![("x".to_string(), loc("1", &[5])), ("z".to_string(), loc("1", &[4]))];
        assert!(pick_renumerated(&origin, &before, &after).is_err());
    }

    #[test]
    fn location_text() {
        let mut l = loc("3", &[1, 4]);
        l.controller = Some("Intel USB 3.20 eXtensible Host Controller".into());
        l.speed = Some("high".into());
        assert_eq!(
            l.to_string(),
            "bus 3 (Intel USB 3.20 eXtensible Host Controller) port 1.4, high speed"
        );
        assert!(l.behind_hub());
        assert!(!loc("", &[]).same_port(&loc("", &[])));
    }

    #[test]
    fn board_ids_are_nonzero() {
        for _ in 0..100 {
            assert_ne!(random_board_id(), 0);
        }
    }

    #[test]
    fn listing_does_not_fail_without_devices() {
        // On a machine with no FX2 this returns an empty list, not an error.
        let list = list_fx2().unwrap();
        for f in &list {
            assert!(classify(f.vid, f.pid).is_some());
        }
    }
}
