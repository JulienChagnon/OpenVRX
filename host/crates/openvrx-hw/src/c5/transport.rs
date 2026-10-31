// SPDX-License-Identifier: GPL-3.0-or-later
//! Serial transports: the byte-level link trait, the system serial port
//! (Espressif USB Serial/JTAG, VID 0x303A), port discovery, and a simulated
//! C5 radio head that speaks the console protocol for tests and dry runs.

use crate::error::{HwError, Result};
use serde::{Deserialize, Serialize};
use std::io;
use std::time::Duration;

/// Espressif USB vendor ID.
pub const ESPRESSIF_VID: u16 = 0x303A;

/// A byte stream with read timeouts.
pub trait SerialLink: Send {
    /// Reads available bytes, waiting up to `timeout`; `Ok(0)` on timeout.
    /// An error means the link is gone.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
    fn write_all(&mut self, data: &[u8]) -> io::Result<()>;
}

/// Opens (and re-opens after a reset) a serial link.
pub trait SerialConnector: Send + 'static {
    fn connect(&mut self) -> Result<Box<dyn SerialLink>>;
    fn describe(&self) -> String;
}

/// A serial port found on the system.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SerialPortFound {
    pub port_name: String,
    pub vid: u16,
    pub pid: u16,
    /// USB serial number (the C5's MAC address for USB Serial/JTAG).
    pub serial_number: Option<String>,
    pub product: Option<String>,
    pub manufacturer: Option<String>,
    /// USB bus and port chain, when the OS reports them.
    pub bus_id: Option<String>,
    pub port_chain: Vec<u8>,
    /// USB interface number of this port (composite devices).
    pub interface: Option<u8>,
}

/// Lists USB serial ports, optionally only one vendor.
pub fn list_usb_serial(vid: Option<u16>) -> Result<Vec<SerialPortFound>> {
    let ports = serialport::available_ports()?;
    let mut out = Vec::new();
    for p in ports {
        if let serialport::SerialPortType::UsbPort(u) = p.port_type {
            if vid.is_some_and(|v| v != u.vid) {
                continue;
            }
            out.push(SerialPortFound {
                port_name: p.port_name,
                vid: u.vid,
                pid: u.pid,
                serial_number: u.serial_number,
                product: u.product,
                manufacturer: u.manufacturer,
                bus_id: u.location.as_ref().map(|l| l.bus_id().to_string()),
                port_chain: u.location.as_ref().map(|l| l.port_chain().to_vec()).unwrap_or_default(),
                interface: u.interface,
            });
        }
    }
    out.sort_by(|a, b| a.port_name.cmp(&b.port_name));
    Ok(out)
}

/// Lists Espressif USB Serial/JTAG ports (candidate C5 radio heads).
pub fn find_c5_ports() -> Result<Vec<SerialPortFound>> {
    list_usb_serial(Some(ESPRESSIF_VID))
}

/// The system serial port.
pub struct SystemSerial {
    port: Box<dyn serialport::SerialPort>,
    timeout: Duration,
}

impl SystemSerial {
    /// Opens a port. DTR and RTS stay deasserted: on the ESP32-C5 USB
    /// Serial/JTAG those lines drive the chip's reset and boot strapping.
    pub fn open(name: &str) -> Result<SystemSerial> {
        let mut port = serialport::new(name, 115_200)
            .timeout(Duration::from_millis(10))
            .dtr_on_open(false)
            .open()?;
        let _ = port.write_request_to_send(false);
        let _ = port.write_data_terminal_ready(false);
        let _ = port.clear(serialport::ClearBuffer::All);
        Ok(SystemSerial {
            port,
            timeout: Duration::from_millis(10),
        })
    }
}

impl SerialLink for SystemSerial {
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        if timeout != self.timeout {
            self.port.set_timeout(timeout).map_err(io::Error::other)?;
            self.timeout = timeout;
        }
        match self.port.read(buf) {
            Ok(0) => Err(io::Error::new(io::ErrorKind::BrokenPipe, "serial port closed")),
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::TimedOut || e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        io::Write::write_all(&mut self.port, data)?;
        io::Write::flush(&mut self.port)
    }
}

/// How to find a port again after it disappears (C5 reset re-enumerates).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PortSelector {
    /// OS port name (`COM7`).
    Name(String),
    /// USB serial number (stable across resets and COM renumbering).
    UsbSerial { vid: u16, serial: String },
}

/// Connector for system serial ports.
pub struct PortConnector {
    pub selector: PortSelector,
}

impl PortConnector {
    pub fn new(selector: PortSelector) -> PortConnector {
        PortConnector { selector }
    }

    fn resolve(&self) -> Result<String> {
        match &self.selector {
            PortSelector::Name(n) => Ok(n.clone()),
            PortSelector::UsbSerial { vid, serial } => list_usb_serial(Some(*vid))?
                .into_iter()
                .find(|p| {
                    p.serial_number
                        .as_deref()
                        .is_some_and(|s| s.eq_ignore_ascii_case(serial))
                })
                .map(|p| p.port_name)
                .ok_or_else(|| HwError::NotFound(format!("no serial port with USB serial {serial}"))),
        }
    }
}

impl SerialConnector for PortConnector {
    fn connect(&mut self) -> Result<Box<dyn SerialLink>> {
        let name = self.resolve()?;
        Ok(Box::new(SystemSerial::open(&name)?))
    }

    fn describe(&self) -> String {
        match &self.selector {
            PortSelector::Name(n) => n.clone(),
            PortSelector::UsbSerial { serial, .. } => format!("USB serial {serial}"),
        }
    }
}

pub mod sim {
    //! A simulated C5 radio head implementing the console protocol.
    use super::*;
    use std::collections::{HashMap, VecDeque};
    use std::sync::{Arc, Mutex};

    /// Firmware state and scripted behaviour.
    #[derive(Debug)]
    pub struct SimC5State {
        pub board: String,
        pub lanes: String,
        pub divider: u32,
        pub clk_on: bool,
        pub mhz: u32,
        pub off_khz: i32,
        pub gain: u32,
        pub p11: bool,
        pub filter: String,
        pub skirt: u32,
        pub bw: u32,
        pub drive: (u8, u8),
        pub selftest: String,
        pub slips: u32,
        pub up_ms: u64,
        /// Every command line received.
        pub received: Vec<String>,
        /// Commands answered with `ERR <word> <code> <text>`.
        pub reject: HashMap<String, (String, String)>,
        /// Commands not answered at all (timeouts).
        pub ignore: Vec<String>,
        /// USB link down (port missing).
        pub unplugged: bool,
        /// Link generation; links from older generations fail.
        pub generation: u64,
        out: VecDeque<u8>,
        inbuf: Vec<u8>,
    }

    impl Default for SimC5State {
        fn default() -> Self {
            SimC5State {
                board: "xiao".into(),
                lanes: "7x7".into(),
                divider: 18,
                clk_on: true,
                mhz: 5865,
                off_khz: 0,
                gain: 52,
                p11: false,
                filter: "cal".into(),
                skirt: 0,
                bw: 20,
                drive: (1, 2),
                selftest: "off".into(),
                slips: 0,
                up_ms: 0,
                received: Vec::new(),
                reject: HashMap::new(),
                ignore: Vec::new(),
                unplugged: false,
                generation: 0,
                out: VecDeque::new(),
                inbuf: Vec::new(),
            }
        }
    }

    impl SimC5State {
        fn emit(&mut self, line: &str) {
            self.out.extend(line.as_bytes());
            self.out.extend(b"\r\n");
        }

        /// Resets the volatile state as a chip reset would and prints the
        /// boot banner and `EVT boot`.
        pub fn reset(&mut self, reason: &str) {
            let keep = (
                std::mem::take(&mut self.received),
                std::mem::take(&mut self.reject),
                std::mem::take(&mut self.ignore),
                self.generation,
                self.unplugged,
            );
            let out = std::mem::take(&mut self.out);
            *self = SimC5State::default();
            self.out = out;
            self.received = keep.0;
            self.reject = keep.1;
            self.ignore = keep.2;
            self.generation = keep.3;
            self.unplugged = keep.4;
            self.emit("ESP-ROM:esp32c5-eco2-20250121");
            self.emit("I (25) boot: ESP-IDF v6.0.2 2nd stage bootloader");
            self.emit(&format!("EVT boot fw=0.1.0 reason={reason}"));
        }

        fn data_pins(&self) -> usize {
            self.lanes
                .parse::<openvrx_core::LanePreset>()
                .map(|p| p.data_pins())
                .unwrap_or(14)
        }

        fn handle(&mut self, line: &str) {
            self.received.push(line.to_string());
            let toks: Vec<&str> = line.split(' ').collect();
            let word = toks[0].to_string();
            if self.ignore.contains(&word) {
                return;
            }
            if let Some((code, text)) = self.reject.get(&word).cloned() {
                self.emit(&format!("ERR {word} {code} {text}"));
                return;
            }
            let arg = |k: usize| toks.get(k).copied();
            let reply = match (word.as_str(), arg(1)) {
                ("ping", _) => Ok("OK ping".to_string()),
                ("ver", _) => Ok(format!(
                    "OK ver fw=0.1.0 proto=1 board={} idf=v6.0.2 chip=esp32c5 rev=1",
                    self.board
                )),
                ("status", _) => Ok(format!(
                    "OK status mhz={} off_khz={} gain={} clk={} clk_on={} lanes={} p11={} filter={} selftest={} up_ms={} slips={} phy=ok",
                    self.mhz, self.off_khz, self.gain, self.divider, self.clk_on as u8, self.lanes,
                    self.p11 as u8, self.filter, self.selftest, self.up_ms, self.slips
                )),
                ("board", Some(b)) if ["xiao", "zero", "custom"].contains(&b) => {
                    self.board = b.into();
                    Ok(format!("OK board name={b} pins={}", if b == "zero" { 17 } else { 15 }))
                }
                ("lanes", Some(p)) => match p.parse::<openvrx_core::LanePreset>() {
                    Ok(pr) if self.board == "xiao" && pr.data_pins() > 14 => Err("range board has 14 data pins".to_string()),
                    Ok(pr) => {
                        self.lanes = p.into();
                        Ok(format!("OK lanes preset={p} data_pins={}", pr.data_pins()))
                    }
                    Err(_) => Err("syntax unknown preset".into()),
                },
                ("pins", _) => {
                    let n = self.data_pins() / 2;
                    let gp = [23u8, 24, 11, 12, 8, 9, 10, 2];
                    let gq = [1u8, 0, 25, 7, 3, 4, 5, 6];
                    let f = |v: &[u8]| v[..n].iter().map(|g| format!("G{g}")).collect::<Vec<_>>().join(",");
                    Ok(format!("OK pins clk=G27 q={} i={}", f(&gq), f(&gp)))
                }
                ("tune", Some(m)) => match m.parse::<u32>() {
                    Ok(m) if (5000..=6000).contains(&m) => {
                        self.mhz = m;
                        self.off_khz = arg(2).and_then(|o| o.parse().ok()).unwrap_or(0);
                        Ok(format!("OK tune mhz={} off_khz={}", self.mhz, self.off_khz))
                    }
                    Ok(_) => Err("range frequency outside the PHY range".into()),
                    Err(_) => Err("syntax bad frequency".into()),
                },
                ("gain", Some(g)) => match g.parse::<u32>() {
                    Ok(g) if g <= 77 => {
                        self.gain = g;
                        Ok(format!("OK gain index={g}"))
                    }
                    _ => Err("range gain 0..77".into()),
                },
                ("gains", _) => Ok("OK gains max=77".into()),
                ("clk", Some("on")) | ("clk", Some("off")) => {
                    self.clk_on = arg(1) == Some("on");
                    Ok(format!("OK clk on={}", self.clk_on as u8))
                }
                ("clk", Some(d)) => match d.parse::<u32>() {
                    Ok(d) if (6..=1023).contains(&d) => {
                        self.divider = d;
                        Ok(format!("OK clk div={d} hz={}", 240_000_000 / d))
                    }
                    _ => Err("range divider 6..1023".into()),
                },
                ("slip", n) => {
                    let n: u32 = n.and_then(|v| v.parse().ok()).unwrap_or(1);
                    self.slips += n;
                    Ok(format!("OK slip n={n} total={}", self.slips))
                }
                ("p11", Some(v)) => {
                    self.p11 = v == "on";
                    Ok(format!("OK p11 on={}", self.p11 as u8))
                }
                ("filter", Some(c)) => {
                    self.filter = c.into();
                    self.skirt = arg(2).and_then(|s| s.parse().ok()).unwrap_or(0);
                    Ok(format!("OK filter code={c} skirt={}", self.skirt))
                }
                ("bw", Some(b)) => {
                    self.bw = b.parse().unwrap_or(20);
                    Ok(format!("OK bw mhz={}", self.bw))
                }
                ("drive", Some(l)) => {
                    self.drive.0 = l.parse().unwrap_or(1);
                    self.drive.1 = arg(2).and_then(|c| c.parse().ok()).unwrap_or(self.drive.1);
                    Ok(format!("OK drive lanes={} clk={}", self.drive.0, self.drive.1))
                }
                ("selftest", Some(m)) => {
                    self.selftest = m.into();
                    Ok(format!("OK selftest mode={m} lanes={} phase_us=1000", self.data_pins()))
                }
                ("save", _) => Ok("OK save".into()),
                ("reboot", _) | ("dl", _) => {
                    self.emit(&format!("OK {word}"));
                    // The USB device drops and comes back.
                    self.generation += 1;
                    self.reset("sw");
                    return;
                }
                _ => Err("syntax unknown command".into()),
            };
            match reply {
                Ok(r) => self.emit(&r),
                Err(e) => self.emit(&format!("ERR {word} {e}")),
            }
        }
    }

    /// Handle to a simulated radio head.
    #[derive(Clone, Default)]
    pub struct SimC5(pub Arc<Mutex<SimC5State>>);

    impl SimC5 {
        pub fn new() -> SimC5 {
            SimC5::default()
        }

        pub fn with<R>(&self, f: impl FnOnce(&mut SimC5State) -> R) -> R {
            f(&mut self.0.lock().expect("sim lock"))
        }

        /// Chip reset without the USB link dropping.
        pub fn reset_in_place(&self) {
            self.with(|s| s.reset("panic"));
        }

        /// Unplugs (links fail) or plugs back in (with a fresh boot).
        pub fn set_unplugged(&self, unplugged: bool) {
            self.with(|s| {
                s.unplugged = unplugged;
                if unplugged {
                    s.generation += 1;
                } else {
                    s.reset("poweron");
                }
            });
        }

        /// Prints an unsolicited line.
        pub fn emit(&self, line: &str) {
            self.with(|s| s.emit(line));
        }

        pub fn connector(&self) -> SimC5Connector {
            SimC5Connector { dev: self.clone() }
        }
    }

    pub struct SimC5Link {
        dev: SimC5,
        generation: u64,
    }

    impl SerialLink for SimC5Link {
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
            {
                let mut s = self.dev.0.lock().expect("sim lock");
                if s.unplugged || s.generation != self.generation {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, "simulated C5 gone"));
                }
                if !s.out.is_empty() {
                    let n = buf.len().min(s.out.len());
                    for b in buf.iter_mut().take(n) {
                        *b = s.out.pop_front().expect("len checked");
                    }
                    return Ok(n);
                }
                s.up_ms += 1;
            }
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
            Ok(0)
        }

        fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
            let mut s = self.dev.0.lock().expect("sim lock");
            if s.unplugged || s.generation != self.generation {
                return Err(io::Error::new(io::ErrorKind::BrokenPipe, "simulated C5 gone"));
            }
            s.inbuf.extend_from_slice(data);
            while let Some(pos) = s.inbuf.iter().position(|&b| b == b'\n') {
                let raw: Vec<u8> = s.inbuf.drain(..=pos).collect();
                let line = String::from_utf8_lossy(&raw).trim_end_matches(['\r', '\n']).to_string();
                if !line.is_empty() {
                    s.handle(&line);
                }
            }
            Ok(())
        }
    }

    pub struct SimC5Connector {
        dev: SimC5,
    }

    impl SerialConnector for SimC5Connector {
        fn connect(&mut self) -> Result<Box<dyn SerialLink>> {
            let s = self.dev.0.lock().expect("sim lock");
            if s.unplugged {
                return Err(HwError::NotFound("simulated C5 unplugged".into()));
            }
            Ok(Box::new(SimC5Link {
                dev: self.dev.clone(),
                generation: s.generation,
            }))
        }

        fn describe(&self) -> String {
            "simulated C5".into()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn listing_works_without_devices() {
        for p in find_c5_ports().unwrap() {
            assert_eq!(p.vid, ESPRESSIF_VID);
        }
    }

    #[test]
    fn sim_answers_lines() {
        let c5 = sim::SimC5::new();
        let mut link = c5.connector().connect().unwrap();
        link.write_all(b"tune 5732 -20\r\nping\n").unwrap();
        let mut buf = [0u8; 256];
        let mut text = String::new();
        for _ in 0..10 {
            let n = link.read(&mut buf, Duration::from_millis(1)).unwrap();
            text.push_str(&String::from_utf8_lossy(&buf[..n]));
        }
        assert_eq!(text, "OK tune mhz=5732 off_khz=-20\r\nOK ping\r\n");
        c5.set_unplugged(true);
        assert!(link.read(&mut buf, Duration::from_millis(1)).is_err());
        assert!(c5.connector().connect().is_err());
    }
}
