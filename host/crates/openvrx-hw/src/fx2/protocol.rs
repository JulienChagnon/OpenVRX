// SPDX-License-Identifier: GPL-3.0-or-later
//! FX2 USB protocol (PROTOCOL.md section 2): identities, vendor requests,
//! the info and status blocks, and the typed request API over any
//! [`ControlIo`] transport.

use crate::error::{HwError, Result};
use crate::fx2::image::{FirmwareImage, IicImage};
use serde::{Deserialize, Serialize};

/// Devices that accept the `0xA0` RAM loader before OpenVRX firmware runs.
pub const BOOTLOADER_IDS: [(u16, u16); 4] = [(0x04b4, 0x8613), (0x04b4, 0x1004), (0x0925, 0x3881), (0x1d50, 0x608c)];
/// OpenVRX firmware VID (pid.codes).
pub const OPENVRX_VID: u16 = 0x1209;
/// OpenVRX firmware PID (pid.codes test range).
pub const OPENVRX_PID: u16 = 0x0001;
/// Device interface GUID declared by the MS OS 2.0 descriptors.
pub const OPENVRX_INTERFACE_GUID: &str = "{5C2B2B1E-7D5A-4C1B-9E0B-50435652580A}";
/// Protocol version this host implements.
pub const PROTO_VERSION: u8 = 1;

/// Bulk IN stream endpoint (EP2).
pub const EP_STREAM: u8 = 0x82;
/// Interrupt IN status endpoint (EP1).
pub const EP_STATUS: u8 = 0x81;
/// Bulk packet size at high speed; every stream packet is this long.
pub const PACKET_BYTES: usize = 512;
/// EP1 max packet size.
pub const STATUS_EP_BYTES: usize = 64;
/// FX2 CPU control register written by the loader.
pub const CPUCS: u16 = 0xE600;

/// Vendor request numbers.
pub mod req {
    /// Built-in RAM loader (handled by the FX2 core).
    pub const FW_LOAD: u8 = 0xA0;
    pub const GET_INFO: u8 = 0xB0;
    pub const START: u8 = 0xB1;
    pub const STOP: u8 = 0xB2;
    pub const GET_STATUS: u8 = 0xB3;
    pub const RESET_COUNTERS: u8 = 0xB4;
    pub const SET_WATCH: u8 = 0xB5;
    pub const LED: u8 = 0xB6;
    pub const EEPROM_READ: u8 = 0xBA;
    pub const EEPROM_WRITE: u8 = 0xBB;
    pub const SET_BOARD_ID: u8 = 0xBC;
}

/// `START` flags (wValue bits).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct StartFlags(pub u16);

impl StartFlags {
    /// 16-bit FIFO (WORDWIDE=1).
    pub const WIDTH16: StartFlags = StartFlags(1 << 0);
    /// Invert IFCLK polarity.
    pub const IFCLK_INV: StartFlags = StartFlags(1 << 1);
    /// Throughput test on the internal IFCLK.
    pub const INTERNAL_CLK: StartFlags = StartFlags(1 << 2);
    /// With `INTERNAL_CLK`: 48 MHz instead of 30 MHz.
    pub const INTERNAL_48: StartFlags = StartFlags(1 << 3);
    /// Start in watch mode with ratio wIndex.
    pub const WATCH: StartFlags = StartFlags(1 << 4);
    /// With `INTERNAL_CLK`: firmware-generated 16-bit counter.
    pub const TEST_PATTERN: StartFlags = StartFlags(1 << 5);

    pub const fn empty() -> StartFlags {
        StartFlags(0)
    }
    #[inline]
    pub fn contains(self, o: StartFlags) -> bool {
        self.0 & o.0 == o.0
    }
    #[inline]
    pub fn with(self, o: StartFlags, on: bool) -> StartFlags {
        if on {
            StartFlags(self.0 | o.0)
        } else {
            StartFlags(self.0 & !o.0)
        }
    }
    /// Flags for a lane format's word width.
    pub fn for_word_bytes(word_bytes: u8) -> StartFlags {
        StartFlags::empty().with(StartFlags::WIDTH16, word_bytes == 2)
    }
}

impl std::ops::BitOr for StartFlags {
    type Output = StartFlags;
    fn bitor(self, rhs: StartFlags) -> StartFlags {
        StartFlags(self.0 | rhs.0)
    }
}

/// `GET_INFO` block (16 bytes).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fx2Info {
    pub proto: u8,
    pub fw_major: u8,
    pub fw_minor: u8,
    pub fw_patch: u8,
    pub features: u32,
    pub board_id: u32,
}

impl Fx2Info {
    pub const LEN: usize = 16;

    pub fn decode(b: &[u8]) -> Result<Fx2Info> {
        if b.len() < Self::LEN {
            return Err(HwError::Protocol(format!(
                "GET_INFO returned {} bytes, need 16",
                b.len()
            )));
        }
        if &b[0..4] != b"PCVX" {
            return Err(HwError::Protocol(format!(
                "GET_INFO magic {:02X?} is not \"PCVX\"",
                &b[0..4]
            )));
        }
        Ok(Fx2Info {
            proto: b[4],
            fw_major: b[5],
            fw_minor: b[6],
            fw_patch: b[7],
            features: u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
            board_id: u32::from_le_bytes([b[12], b[13], b[14], b[15]]),
        })
    }

    pub fn encode(&self) -> [u8; 16] {
        let mut b = [0u8; 16];
        b[0..4].copy_from_slice(b"PCVX");
        b[4] = self.proto;
        b[5] = self.fw_major;
        b[6] = self.fw_minor;
        b[7] = self.fw_patch;
        b[8..12].copy_from_slice(&self.features.to_le_bytes());
        b[12..16].copy_from_slice(&self.board_id.to_le_bytes());
        b
    }

    pub fn fw_version(&self) -> String {
        format!("{}.{}.{}", self.fw_major, self.fw_minor, self.fw_patch)
    }

    /// Serial string the firmware derives from the board ID.
    pub fn serial(&self) -> String {
        board_serial(self.board_id)
    }
}

/// 8 upper-case hex digits, as the firmware's serial number string.
pub fn board_serial(board_id: u32) -> String {
    format!("{board_id:08X}")
}

/// Status block flag bits.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct StatusFlags(pub u8);

impl StatusFlags {
    pub const RUNNING: u8 = 1 << 0;
    pub const WIDTH16: u8 = 1 << 1;
    pub const IFCLK_INV: u8 = 1 << 2;
    pub const INTERNAL_CLK: u8 = 1 << 3;
    pub const WATCH: u8 = 1 << 4;
    pub const IFCLK_ACTIVE: u8 = 1 << 5;
    pub const TEST_PATTERN: u8 = 1 << 6;

    #[inline]
    fn bit(self, b: u8) -> bool {
        self.0 & b != 0
    }
    pub fn running(self) -> bool {
        self.bit(Self::RUNNING)
    }
    pub fn width16(self) -> bool {
        self.bit(Self::WIDTH16)
    }
    pub fn ifclk_inv(self) -> bool {
        self.bit(Self::IFCLK_INV)
    }
    pub fn internal_clk(self) -> bool {
        self.bit(Self::INTERNAL_CLK)
    }
    pub fn watch(self) -> bool {
        self.bit(Self::WATCH)
    }
    pub fn ifclk_active(self) -> bool {
        self.bit(Self::IFCLK_ACTIVE)
    }
    pub fn test_pattern(self) -> bool {
        self.bit(Self::TEST_PATTERN)
    }
}

/// Decoded 32-byte status block (EP1 and `GET_STATUS`).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fx2Status {
    pub proto: u8,
    pub flags: StatusFlags,
    pub watch_n: u16,
    /// Packets committed (watch and test-pattern modes; 0 in AUTOIN mode).
    pub packets_committed: u32,
    /// Packets skipped (watch mode).
    pub packets_skipped: u32,
    /// Polls that found the EP2 FIFO full while running.
    pub fifo_full: u32,
    pub uptime_ms: u32,
    pub seq: u16,
    /// Firmware version, major << 8 | minor.
    pub fw_version: u16,
}

impl Fx2Status {
    pub const LEN: usize = 32;

    pub fn decode(b: &[u8]) -> Result<Fx2Status> {
        if b.len() < Self::LEN {
            return Err(HwError::Protocol(format!("status block is {} bytes, need 32", b.len())));
        }
        if &b[0..4] != b"PCVS" {
            return Err(HwError::Protocol(format!(
                "status magic {:02X?} is not \"PCVS\"",
                &b[0..4]
            )));
        }
        let u16le = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
        let u32le = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        Ok(Fx2Status {
            proto: b[4],
            flags: StatusFlags(b[5]),
            watch_n: u16le(6),
            packets_committed: u32le(8),
            packets_skipped: u32le(12),
            fifo_full: u32le(16),
            uptime_ms: u32le(20),
            seq: u16le(24),
            fw_version: u16le(26),
        })
    }

    pub fn encode(&self) -> [u8; 32] {
        let mut b = [0u8; 32];
        b[0..4].copy_from_slice(b"PCVS");
        b[4] = self.proto;
        b[5] = self.flags.0;
        b[6..8].copy_from_slice(&self.watch_n.to_le_bytes());
        b[8..12].copy_from_slice(&self.packets_committed.to_le_bytes());
        b[12..16].copy_from_slice(&self.packets_skipped.to_le_bytes());
        b[16..20].copy_from_slice(&self.fifo_full.to_le_bytes());
        b[20..24].copy_from_slice(&self.uptime_ms.to_le_bytes());
        b[24..26].copy_from_slice(&self.seq.to_le_bytes());
        b[26..28].copy_from_slice(&self.fw_version.to_le_bytes());
        b
    }

    /// FIFO-full events since an earlier status (wrapping counter).
    pub fn fifo_full_since(&self, earlier: &Fx2Status) -> u32 {
        self.fifo_full.wrapping_sub(earlier.fifo_full)
    }
}

/// Vendor control transfers on the device recipient. Implemented by the
/// nusb device handle and by test mocks.
pub trait ControlIo: Send + Sync {
    /// `bmRequestType 0xC0`: device to host.
    fn vendor_in(&self, request: u8, value: u16, index: u16, len: u16) -> Result<Vec<u8>>;
    /// `bmRequestType 0x40`: host to device.
    fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()>;
}

/// Chunk size for `0xA0` RAM writes.
pub const LOAD_CHUNK: usize = 1024;
/// Chunk size for EEPROM writes and reads (one EP0 packet).
pub const EEPROM_CHUNK: usize = 64;

/// Typed FX2 requests, available on every [`ControlIo`].
pub trait Fx2Requests: ControlIo {
    fn get_info(&self) -> Result<Fx2Info> {
        Fx2Info::decode(&self.vendor_in(req::GET_INFO, 0, 0, Fx2Info::LEN as u16)?)
    }

    /// `START` with flags; `watch_n` goes in wIndex (used with `WATCH`).
    fn start(&self, flags: StartFlags, watch_n: u16) -> Result<()> {
        self.vendor_out(req::START, flags.0, watch_n, &[])
    }

    fn stop(&self) -> Result<()> {
        self.vendor_out(req::STOP, 0, 0, &[])
    }

    fn get_status(&self) -> Result<Fx2Status> {
        Fx2Status::decode(&self.vendor_in(req::GET_STATUS, 0, 0, Fx2Status::LEN as u16)?)
    }

    fn reset_counters(&self) -> Result<()> {
        self.vendor_out(req::RESET_COUNTERS, 0, 0, &[])
    }

    /// 0 or 1 = full stream, N > 1 = commit one packet in N.
    fn set_watch(&self, n: u16) -> Result<()> {
        self.vendor_out(req::SET_WATCH, n, 0, &[])
    }

    fn led(&self, on: bool) -> Result<()> {
        self.vendor_out(req::LED, on as u16, 0, &[])
    }

    fn set_board_id(&self, id: u32) -> Result<()> {
        self.vendor_out(req::SET_BOARD_ID, id as u16, (id >> 16) as u16, &[])
    }

    /// Writes RAM through the built-in loader.
    fn load_ram(&self, addr: u16, data: &[u8]) -> Result<()> {
        self.vendor_out(req::FW_LOAD, addr, 0, data)
    }

    /// Holds (`true`) or releases (`false`) the 8051 in reset via CPUCS.
    fn cpu_reset(&self, hold: bool) -> Result<()> {
        self.load_ram(CPUCS, &[hold as u8])
    }

    /// Loads an image into RAM and starts it: CPUCS=1, all chunks, CPUCS=0.
    /// The device then disconnects and re-enumerates, so a failure of the
    /// final CPUCS write caused by the disconnect is not an error.
    fn upload_firmware(&self, fw: &FirmwareImage) -> Result<()> {
        fw.check_fx2_ram()?;
        if fw.is_empty() {
            return Err(HwError::NoFirmware("the image has no data".into()));
        }
        self.cpu_reset(true)?;
        for (addr, chunk) in fw.chunks(LOAD_CHUNK) {
            self.load_ram(addr, chunk)?;
        }
        match self.cpu_reset(false) {
            Ok(()) | Err(HwError::Disconnected(_)) => Ok(()),
            Err(HwError::Transfer(crate::error::XferError::Disconnected)) => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn eeprom_read(&self, addr: u16, len: usize) -> Result<Vec<u8>> {
        let mut out = Vec::with_capacity(len);
        let mut a = addr as usize;
        while out.len() < len {
            let n = (len - out.len()).min(EEPROM_CHUNK);
            let got = self.vendor_in(req::EEPROM_READ, a as u16, 0, n as u16)?;
            if got.len() != n {
                return Err(HwError::Protocol(format!(
                    "EEPROM read at 0x{a:04X} returned {} of {n} bytes",
                    got.len()
                )));
            }
            out.extend_from_slice(&got);
            a += n;
        }
        Ok(out)
    }

    fn eeprom_write(&self, addr: u16, data: &[u8]) -> Result<()> {
        if addr as usize + data.len() > 0x1_0000 {
            return Err(HwError::Invalid("EEPROM image beyond 64 KB".into()));
        }
        for (k, chunk) in data.chunks(EEPROM_CHUNK).enumerate() {
            self.vendor_out(req::EEPROM_WRITE, addr + (k * EEPROM_CHUNK) as u16, 0, chunk)?;
        }
        Ok(())
    }

    /// Writes an image at address 0 and reads it back. `progress` gets
    /// (bytes done, total) for the write pass and then the verify pass.
    fn program_eeprom(&self, image: &[u8], progress: &mut dyn FnMut(EepromPhase, usize, usize)) -> Result<()> {
        let total = image.len();
        for (k, chunk) in image.chunks(EEPROM_CHUNK).enumerate() {
            let a = k * EEPROM_CHUNK;
            self.vendor_out(req::EEPROM_WRITE, a as u16, 0, chunk)?;
            progress(EepromPhase::Write, a + chunk.len(), total);
        }
        for (k, chunk) in image.chunks(EEPROM_CHUNK).enumerate() {
            let a = k * EEPROM_CHUNK;
            let got = self.eeprom_read(a as u16, chunk.len())?;
            if let Some(i) = got.iter().zip(chunk).position(|(x, y)| x != y) {
                return Err(HwError::Verify(format!(
                    "EEPROM byte 0x{:04X} reads 0x{:02X}, wrote 0x{:02X}",
                    a + i,
                    got[i],
                    chunk[i]
                )));
            }
            progress(EepromPhase::Verify, a + chunk.len(), total);
        }
        Ok(())
    }

    /// Validates a C2 image, then programs and verifies it.
    fn program_iic(&self, iic: &[u8], progress: &mut dyn FnMut(EepromPhase, usize, usize)) -> Result<IicImage> {
        let parsed = IicImage::parse(iic)?;
        self.program_eeprom(iic, progress)?;
        Ok(parsed)
    }
}

impl<T: ControlIo + ?Sized> Fx2Requests for T {}

/// Which pass `program_eeprom` is reporting.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EepromPhase {
    Write,
    Verify,
}

#[cfg(test)]
pub(crate) mod mock {
    //! An FX2 emulated at the vendor-request level.
    use super::*;
    use std::sync::Mutex;

    #[derive(Default)]
    pub struct MockFx2State {
        pub ram: Vec<u8>,
        pub cpu_held: bool,
        pub started: bool,
        pub runs: u32,
        pub eeprom: Vec<u8>,
        pub info: Option<Fx2Info>,
        pub status: Fx2Status,
        pub log: Vec<(u8, u16, u16, usize)>,
        pub fail_eeprom_at: Option<usize>,
        pub disconnect_on_run: bool,
        pub start_flags: Vec<(StartFlags, u16)>,
        pub watch_n: u16,
    }

    #[derive(Default)]
    pub struct MockFx2 {
        pub s: Mutex<MockFx2State>,
    }

    impl MockFx2 {
        pub fn new() -> MockFx2 {
            let m = MockFx2::default();
            {
                let mut s = m.s.lock().unwrap();
                s.ram = vec![0; 0x10000];
                s.eeprom = vec![0xFF; 0x4000];
                s.info = Some(Fx2Info {
                    proto: 1,
                    fw_major: 0,
                    fw_minor: 1,
                    fw_patch: 0,
                    features: 0,
                    board_id: 0,
                });
            }
            m
        }
    }

    impl ControlIo for MockFx2 {
        fn vendor_in(&self, request: u8, value: u16, _index: u16, len: u16) -> Result<Vec<u8>> {
            let mut s = self.s.lock().unwrap();
            s.log.push((request, value, _index, len as usize));
            match request {
                req::GET_INFO => Ok(s.info.unwrap().encode()[..len as usize].to_vec()),
                req::GET_STATUS => Ok(s.status.encode()[..len as usize].to_vec()),
                req::EEPROM_READ => {
                    let a = value as usize;
                    let mut v = s.eeprom[a..a + len as usize].to_vec();
                    if let Some(bad) = s.fail_eeprom_at {
                        if bad >= a && bad < a + v.len() {
                            v[bad - a] ^= 0x01;
                        }
                    }
                    Ok(v)
                }
                _ => Err(HwError::Transfer(crate::error::XferError::Stall)),
            }
        }

        fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
            let mut s = self.s.lock().unwrap();
            s.log.push((request, value, index, data.len()));
            match request {
                req::FW_LOAD => {
                    if value == CPUCS {
                        let hold = data[0] & 1 == 1;
                        if s.cpu_held && !hold {
                            s.runs += 1;
                            if s.disconnect_on_run {
                                s.cpu_held = false;
                                return Err(HwError::Disconnected("renumerating".into()));
                            }
                        }
                        s.cpu_held = hold;
                    } else {
                        assert!(s.cpu_held, "RAM written while the CPU runs");
                        let a = value as usize;
                        s.ram[a..a + data.len()].copy_from_slice(data);
                    }
                    Ok(())
                }
                req::START => {
                    s.started = true;
                    s.start_flags.push((StartFlags(value), index));
                    s.status.flags.0 |= StatusFlags::RUNNING;
                    Ok(())
                }
                req::STOP => {
                    s.started = false;
                    s.status.flags.0 &= !StatusFlags::RUNNING;
                    Ok(())
                }
                req::RESET_COUNTERS => {
                    s.status.fifo_full = 0;
                    s.status.packets_committed = 0;
                    s.status.packets_skipped = 0;
                    Ok(())
                }
                req::SET_WATCH => {
                    s.watch_n = value;
                    s.status.watch_n = value;
                    Ok(())
                }
                req::LED => Ok(()),
                req::EEPROM_WRITE => {
                    let a = value as usize;
                    s.eeprom[a..a + data.len()].copy_from_slice(data);
                    Ok(())
                }
                req::SET_BOARD_ID => {
                    let id = value as u32 | (index as u32) << 16;
                    if let Some(i) = s.info.as_mut() {
                        i.board_id = id;
                    }
                    Ok(())
                }
                _ => Err(HwError::Transfer(crate::error::XferError::Stall)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::mock::MockFx2;
    use super::*;
    use crate::fx2::image::{IicHeader, Segment};

    #[test]
    fn info_and_status_roundtrip() {
        let info = Fx2Info {
            proto: 1,
            fw_major: 1,
            fw_minor: 2,
            fw_patch: 3,
            features: 0xDEAD_BEEF,
            board_id: 0xA1B2_C3D4,
        };
        let back = Fx2Info::decode(&info.encode()).unwrap();
        assert_eq!(back, info);
        assert_eq!(back.serial(), "A1B2C3D4");
        assert_eq!(back.fw_version(), "1.2.3");
        let mut raw = info.encode();
        raw[0] = b'X';
        assert!(Fx2Info::decode(&raw).is_err());
        assert!(Fx2Info::decode(&raw[..10]).is_err());

        // Hand-built status block, little endian.
        let mut b = [0u8; 32];
        b[0..4].copy_from_slice(b"PCVS");
        b[4] = 1;
        b[5] = 0b0110_0011;
        b[6] = 16;
        b[8..12].copy_from_slice(&1000u32.to_le_bytes());
        b[12..16].copy_from_slice(&15000u32.to_le_bytes());
        b[16..20].copy_from_slice(&7u32.to_le_bytes());
        b[20..24].copy_from_slice(&123_456u32.to_le_bytes());
        b[24..26].copy_from_slice(&0xFFFFu16.to_le_bytes());
        b[26..28].copy_from_slice(&0x0102u16.to_le_bytes());
        let s = Fx2Status::decode(&b).unwrap();
        assert!(s.flags.running() && s.flags.width16() && s.flags.ifclk_active() && s.flags.test_pattern());
        assert!(!s.flags.ifclk_inv() && !s.flags.watch() && !s.flags.internal_clk());
        assert_eq!(s.watch_n, 16);
        assert_eq!(s.packets_committed, 1000);
        assert_eq!(s.packets_skipped, 15000);
        assert_eq!(s.fifo_full, 7);
        assert_eq!(s.uptime_ms, 123_456);
        assert_eq!(s.seq, 0xFFFF);
        assert_eq!(s.fw_version, 0x0102);
        assert_eq!(s.encode(), b);
        let mut later = s;
        later.fifo_full = 2;
        let mut earlier = s;
        earlier.fifo_full = u32::MAX;
        assert_eq!(later.fifo_full_since(&earlier), 3);
    }

    #[test]
    fn start_flags() {
        let f = StartFlags::for_word_bytes(2) | StartFlags::IFCLK_INV;
        assert_eq!(f.0, 0b11);
        assert!(f.contains(StartFlags::WIDTH16));
        assert!(!f.contains(StartFlags::WATCH));
        assert_eq!(StartFlags::for_word_bytes(1).0, 0);
        assert_eq!(
            (StartFlags::INTERNAL_CLK | StartFlags::INTERNAL_48 | StartFlags::TEST_PATTERN).0,
            0b10_1100
        );
    }

    #[test]
    fn upload_sequence() {
        let fx2 = MockFx2::new();
        let fw = FirmwareImage {
            segments: vec![
                Segment {
                    addr: 0,
                    data: (0..3000u32).map(|x| (x * 7) as u8).collect(),
                },
                Segment {
                    addr: 0xE000,
                    data: vec![0x55; 16],
                },
            ],
        };
        fx2.s.lock().unwrap().disconnect_on_run = true;
        fx2.upload_firmware(&fw).unwrap();
        let s = fx2.s.lock().unwrap();
        assert_eq!(s.runs, 1);
        assert_eq!(&s.ram[..3000], &fw.segments[0].data[..]);
        assert_eq!(&s.ram[0xE000..0xE010], &[0x55; 16]);
        // First request holds the CPU, last releases it.
        assert_eq!(s.log.first().unwrap().1, CPUCS);
        assert_eq!(s.log.last().unwrap().1, CPUCS);
        assert!(s.log.iter().all(|e| e.3 <= LOAD_CHUNK));
    }

    #[test]
    fn upload_rejects_bad_images() {
        let fx2 = MockFx2::new();
        assert!(matches!(
            fx2.upload_firmware(&FirmwareImage::default()),
            Err(HwError::NoFirmware(_))
        ));
        let outside = FirmwareImage {
            segments: vec![Segment {
                addr: 0x8000,
                data: vec![1],
            }],
        };
        assert!(fx2.upload_firmware(&outside).is_err());
    }

    #[test]
    fn eeprom_program_and_verify() {
        let fx2 = MockFx2::new();
        let fw = FirmwareImage {
            segments: vec![Segment {
                addr: 0,
                data: vec![0xA5; 700],
            }],
        };
        let iic = IicImage::build(
            IicHeader {
                vid: OPENVRX_VID,
                pid: OPENVRX_PID,
                did: 0,
                config: 0,
            },
            &fw,
        )
        .unwrap();
        let mut calls = Vec::new();
        let parsed = fx2.program_iic(&iic, &mut |p, d, t| calls.push((p, d, t))).unwrap();
        assert_eq!(parsed.firmware, fw);
        assert_eq!(&fx2.s.lock().unwrap().eeprom[..iic.len()], &iic[..]);
        assert_eq!(calls.last().unwrap(), &(EepromPhase::Verify, iic.len(), iic.len()));
        // A flipped bit is reported with its address.
        fx2.s.lock().unwrap().fail_eeprom_at = Some(130);
        match fx2.program_eeprom(&iic, &mut |_, _, _| {}) {
            Err(HwError::Verify(m)) => assert!(m.contains("0x0082"), "{m}"),
            other => panic!("expected verify error, got {other:?}"),
        }
        assert!(fx2.program_iic(&[0xC0, 1, 2], &mut |_, _, _| {}).is_err());
    }

    #[test]
    fn typed_requests() {
        let fx2 = MockFx2::new();
        fx2.set_board_id(0x1234_5678).unwrap();
        assert_eq!(fx2.get_info().unwrap().board_id, 0x1234_5678);
        fx2.start(StartFlags::WIDTH16 | StartFlags::WATCH, 8).unwrap();
        assert!(fx2.get_status().unwrap().flags.running());
        fx2.set_watch(4).unwrap();
        assert_eq!(fx2.get_status().unwrap().watch_n, 4);
        fx2.stop().unwrap();
        assert!(!fx2.get_status().unwrap().flags.running());
        let s = fx2.s.lock().unwrap();
        assert_eq!(s.start_flags[0], (StartFlags(0b1_0001), 8));
    }
}
