// SPDX-License-Identifier: GPL-3.0-or-later
//! The IN endpoint abstraction used by the streaming engine, plus a
//! simulated FX2 (endpoint, control and port) for tests and dry runs.

use crate::error::{HwError, Result, XferError};
use std::time::Duration;

/// A finished IN transfer.
#[derive(Debug)]
pub struct InCompletion {
    /// Bytes received (shorter than requested on a short packet).
    pub data: Vec<u8>,
    pub status: std::result::Result<(), XferError>,
}

/// Queued IN transfers on one endpoint (bulk EP2 or interrupt EP1).
///
/// Mirrors the nusb 0.2 endpoint model: `submit` queues a buffer whose
/// length is the requested size, completions come back in order.
pub trait InEndpoint: Send {
    fn max_packet_size(&self) -> usize;
    /// Queues a transfer; `buf.len()` is the requested length and must be
    /// a multiple of the max packet size.
    fn submit(&mut self, buf: Vec<u8>);
    /// Transfers submitted and not yet returned.
    fn pending(&self) -> usize;
    /// Waits up to `timeout` for the oldest transfer to finish.
    fn wait_complete(&mut self, timeout: Duration) -> Option<InCompletion>;
    /// Requests cancellation of all pending transfers; they still come back
    /// through `wait_complete` (status `Cancelled` or completed).
    fn cancel_all(&mut self);
    /// Clears a STALL on the endpoint.
    fn clear_halt(&mut self) -> Result<()>;
}

pub mod sim {
    //! A simulated FX2: control requests, a bulk stream paced at a byte
    //! rate, a status endpoint, scripted faults and reconnects. Used by the
    //! unit tests and usable by applications for dry runs without hardware.

    use super::*;
    use crate::fx2::protocol::{req, ControlIo, Fx2Info, Fx2Status, StartFlags, StatusFlags, CPUCS};
    use crate::fx2::stream::{StreamLink, StreamPort};
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};
    use std::time::Instant;

    /// Fills stream packets. Gets the device's word counter state.
    pub type Generator = Box<dyn FnMut(&mut [u8], &mut SimDevice) + Send>;

    /// Shared state of one simulated board.
    pub struct SimDevice {
        pub info: Fx2Info,
        pub status: Fx2Status,
        pub running: bool,
        pub flags: StartFlags,
        pub watch_n: u16,
        /// Stream byte rate while running.
        pub byte_rate: f64,
        /// Bytes generated so far.
        pub produced: u64,
        /// 16-bit test counter (next value).
        pub counter: u16,
        /// Completions so far (all endpoints of the stream).
        pub completions: u64,
        /// Faults to inject: (completion number, error).
        pub faults: VecDeque<(u64, XferError)>,
        /// Connect attempts that fail before one succeeds.
        pub connect_failures: u32,
        pub connects: u32,
        /// Words to skip in the test counter at a completion number
        /// (simulates FIFO overflow inside the firmware).
        pub counter_jumps: VecDeque<(u64, u16)>,
        /// Vendor requests seen: (request, value, index).
        pub requests: Vec<(u8, u16, u16)>,
        pub generator: Option<Generator>,
        pub halted: bool,
        pub ram_writes: usize,
        pub cpu_held: bool,
        /// Time of the last delivered completion (pacing).
        next_due: Option<Instant>,
    }

    impl SimDevice {
        pub fn new(byte_rate: f64) -> SimDevice {
            SimDevice {
                info: Fx2Info {
                    proto: 1,
                    fw_major: 0,
                    fw_minor: 1,
                    fw_patch: 0,
                    features: 0,
                    board_id: 0x51D0_0001,
                },
                status: Fx2Status {
                    proto: 1,
                    fw_version: 0x0001,
                    ..Default::default()
                },
                running: false,
                flags: StartFlags::empty(),
                watch_n: 1,
                byte_rate,
                produced: 0,
                counter: 0,
                completions: 0,
                faults: VecDeque::new(),
                connect_failures: 0,
                connects: 0,
                counter_jumps: VecDeque::new(),
                requests: Vec::new(),
                generator: None,
                halted: false,
                ram_writes: 0,
                cpu_held: false,
                next_due: None,
            }
        }

        fn fill(&mut self, buf: &mut [u8]) {
            if let Some(mut g) = self.generator.take() {
                g(buf, self);
                self.generator = Some(g);
            } else {
                // Default: the firmware test counter, 16-bit little endian.
                while let Some(&(at, skip)) = self.counter_jumps.front() {
                    if at <= self.completions {
                        self.counter = self.counter.wrapping_add(skip);
                        self.counter_jumps.pop_front();
                    } else {
                        break;
                    }
                }
                for w in buf.as_chunks_mut::<2>().0 {
                    w.copy_from_slice(&self.counter.to_le_bytes());
                    self.counter = self.counter.wrapping_add(1);
                }
            }
            self.produced += buf.len() as u64;
            let packets = (buf.len() / 512) as u32;
            if self.flags.contains(StartFlags::TEST_PATTERN) || self.watch_n > 1 {
                self.status.packets_committed = self.status.packets_committed.wrapping_add(packets);
            }
            if self.watch_n > 1 {
                self.status.packets_skipped = self
                    .status
                    .packets_skipped
                    .wrapping_add(packets * (self.watch_n as u32 - 1));
            }
        }
    }

    /// Handle to a simulated board (clone it to script faults from a test).
    #[derive(Clone)]
    pub struct SimFx2(pub Arc<Mutex<SimDevice>>);

    impl SimFx2 {
        pub fn new(byte_rate: f64) -> SimFx2 {
            SimFx2(Arc::new(Mutex::new(SimDevice::new(byte_rate))))
        }

        pub fn with<R>(&self, f: impl FnOnce(&mut SimDevice) -> R) -> R {
            f(&mut self.0.lock().expect("sim lock"))
        }

        /// A port that connects to this board.
        pub fn port(&self) -> SimPort {
            SimPort { dev: self.clone() }
        }
    }

    impl ControlIo for SimFx2 {
        fn vendor_in(&self, request: u8, value: u16, index: u16, len: u16) -> Result<Vec<u8>> {
            let mut d = self.0.lock().expect("sim lock");
            d.requests.push((request, value, index));
            let mut v = match request {
                req::GET_INFO => d.info.encode().to_vec(),
                req::GET_STATUS => {
                    d.status.seq = d.status.seq.wrapping_add(1);
                    d.status.encode().to_vec()
                }
                _ => return Err(HwError::Transfer(XferError::Stall)),
            };
            v.truncate(len as usize);
            Ok(v)
        }

        fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
            let mut d = self.0.lock().expect("sim lock");
            d.requests.push((request, value, index));
            match request {
                req::FW_LOAD => {
                    if value == CPUCS {
                        d.cpu_held = data.first() == Some(&1);
                    } else {
                        d.ram_writes += data.len();
                    }
                }
                req::START => {
                    d.running = true;
                    d.flags = StartFlags(value);
                    d.watch_n = if d.flags.contains(StartFlags::WATCH) {
                        index.max(1)
                    } else {
                        1
                    };
                    d.status.watch_n = d.watch_n;
                    let mut f = StatusFlags::RUNNING | StatusFlags::IFCLK_ACTIVE;
                    if d.flags.contains(StartFlags::WIDTH16) {
                        f |= StatusFlags::WIDTH16;
                    }
                    if d.flags.contains(StartFlags::TEST_PATTERN) {
                        f |= StatusFlags::TEST_PATTERN;
                    }
                    if d.flags.contains(StartFlags::INTERNAL_CLK) {
                        f |= StatusFlags::INTERNAL_CLK;
                    }
                    if d.watch_n > 1 {
                        f |= StatusFlags::WATCH;
                    }
                    d.status.flags = StatusFlags(f);
                    d.next_due = None;
                }
                req::STOP => {
                    d.running = false;
                    d.status.flags.0 &= !StatusFlags::RUNNING;
                }
                req::RESET_COUNTERS => {
                    d.status.packets_committed = 0;
                    d.status.packets_skipped = 0;
                    d.status.fifo_full = 0;
                }
                req::SET_WATCH => {
                    d.watch_n = value.max(1);
                    d.status.watch_n = d.watch_n;
                    if d.watch_n > 1 {
                        d.status.flags.0 |= StatusFlags::WATCH;
                    } else {
                        d.status.flags.0 &= !StatusFlags::WATCH;
                    }
                }
                req::LED | req::EEPROM_WRITE => {}
                req::SET_BOARD_ID => d.info.board_id = value as u32 | (index as u32) << 16,
                _ => return Err(HwError::Transfer(XferError::Stall)),
            }
            Ok(())
        }
    }

    /// Simulated bulk IN endpoint paced at the device byte rate.
    pub struct SimBulk {
        dev: SimFx2,
        queue: VecDeque<Vec<u8>>,
        cancelled: bool,
        dead: bool,
    }

    impl InEndpoint for SimBulk {
        fn max_packet_size(&self) -> usize {
            512
        }

        fn submit(&mut self, buf: Vec<u8>) {
            self.queue.push_back(buf);
        }

        fn pending(&self) -> usize {
            self.queue.len()
        }

        fn wait_complete(&mut self, timeout: Duration) -> Option<InCompletion> {
            let Some(mut buf) = self.queue.pop_front() else {
                std::thread::sleep(timeout.min(Duration::from_millis(5)));
                return None;
            };
            if self.cancelled || self.dead {
                let status = if self.dead {
                    XferError::Disconnected
                } else {
                    XferError::Cancelled
                };
                buf.clear();
                return Some(InCompletion {
                    data: buf,
                    status: Err(status),
                });
            }
            let mut d = self.dev.0.lock().expect("sim lock");
            if d.halted {
                buf.clear();
                return Some(InCompletion {
                    data: buf,
                    status: Err(XferError::Stall),
                });
            }
            if !d.running || d.byte_rate <= 0.0 {
                drop(d);
                self.queue.push_front(buf);
                std::thread::sleep(timeout.min(Duration::from_millis(20)));
                return None;
            }
            // Pace: this transfer is due len / rate after the previous one.
            let now = Instant::now();
            let period = Duration::from_secs_f64(buf.len() as f64 / d.byte_rate);
            let floor = now.checked_sub(period * 4).unwrap_or(now);
            let due = d.next_due.map_or(now + period, |t| t.max(floor) + period);
            if due > now + timeout {
                drop(d);
                std::thread::sleep(timeout);
                self.queue.push_front(buf);
                return None;
            }
            d.next_due = Some(due);
            d.completions += 1;
            let n = d.completions;
            if let Some(pos) = d.faults.iter().position(|(at, _)| *at == n) {
                let (_, e) = d.faults.remove(pos).expect("present");
                if e == XferError::Disconnected {
                    self.dead = true;
                    d.running = false;
                }
                if e == XferError::Stall {
                    d.halted = true;
                }
                buf.clear();
                return Some(InCompletion {
                    data: buf,
                    status: Err(e),
                });
            }
            d.fill(&mut buf);
            drop(d);
            let wait = due.saturating_duration_since(Instant::now());
            if !wait.is_zero() {
                std::thread::sleep(wait);
            }
            Some(InCompletion {
                data: buf,
                status: Ok(()),
            })
        }

        fn cancel_all(&mut self) {
            self.cancelled = true;
        }

        fn clear_halt(&mut self) -> Result<()> {
            self.dev.with(|d| d.halted = false);
            self.cancelled = false;
            Ok(())
        }
    }

    /// Simulated EP1: one status block every 8 ms.
    pub struct SimStatusEp {
        dev: SimFx2,
        queue: VecDeque<Vec<u8>>,
        last: Option<Instant>,
        cancelled: bool,
    }

    impl InEndpoint for SimStatusEp {
        fn max_packet_size(&self) -> usize {
            64
        }
        fn submit(&mut self, buf: Vec<u8>) {
            self.queue.push_back(buf);
        }
        fn pending(&self) -> usize {
            self.queue.len()
        }
        fn wait_complete(&mut self, timeout: Duration) -> Option<InCompletion> {
            let mut buf = self.queue.pop_front()?;
            if self.cancelled {
                buf.clear();
                return Some(InCompletion {
                    data: buf,
                    status: Err(XferError::Cancelled),
                });
            }
            let period = Duration::from_millis(8);
            let due = self.last.map_or(Instant::now(), |t| t + period);
            let wait = due.saturating_duration_since(Instant::now());
            if wait > timeout {
                std::thread::sleep(timeout);
                self.queue.push_front(buf);
                return None;
            }
            std::thread::sleep(wait);
            self.last = Some(Instant::now());
            let mut d = self.dev.0.lock().expect("sim lock");
            d.status.seq = d.status.seq.wrapping_add(1);
            d.status.uptime_ms = d.status.uptime_ms.wrapping_add(8);
            let block = d.status.encode();
            buf.clear();
            buf.extend_from_slice(&block);
            Some(InCompletion {
                data: buf,
                status: Ok(()),
            })
        }
        fn cancel_all(&mut self) {
            self.cancelled = true;
        }
        fn clear_halt(&mut self) -> Result<()> {
            Ok(())
        }
    }

    /// Port that connects to a simulated board, honouring scripted
    /// connect failures.
    pub struct SimPort {
        dev: SimFx2,
    }

    impl StreamPort for SimPort {
        fn connect(&mut self) -> Result<StreamLink> {
            let mut d = self.dev.0.lock().expect("sim lock");
            if d.connect_failures > 0 {
                d.connect_failures -= 1;
                return Err(HwError::NotFound("simulated board unplugged".into()));
            }
            d.connects += 1;
            drop(d);
            Ok(StreamLink {
                control: Arc::new(self.dev.clone()),
                bulk: Box::new(SimBulk {
                    dev: self.dev.clone(),
                    queue: VecDeque::new(),
                    cancelled: false,
                    dead: false,
                }),
                status: Some(Box::new(SimStatusEp {
                    dev: self.dev.clone(),
                    queue: VecDeque::new(),
                    last: None,
                    cancelled: false,
                })),
                description: "simulated FX2".into(),
            })
        }

        fn describe(&self) -> String {
            "simulated FX2".into()
        }
    }
}
