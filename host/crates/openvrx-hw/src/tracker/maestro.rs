// SPDX-License-Identifier: GPL-3.0-or-later
//! Pololu Maestro servo controller over its USB virtual serial "Command
//! Port", using the compact protocol:
//!
//! | Command | Bytes |
//! |---|---|
//! | Set Target | `0x84, channel, target & 0x7F, (target >> 7) & 0x7F` (quarter microseconds) |
//! | Set Speed | `0x87, channel, lo7, hi7` (0.25 us per 10 ms; 0 = unlimited) |
//! | Set Acceleration | `0x89, channel, lo7, hi7` (0..255; 0 = unlimited) |
//! | Get Position | `0x90, channel` -> 2 bytes, little endian, quarter microseconds |
//! | Get Moving State | `0x93` -> 1 byte |
//! | Get Errors | `0xA1` -> 2 bytes |
//! | Go Home | `0xA2` |

use crate::c5::transport::{list_usb_serial, SerialLink, SerialPortFound};
use crate::error::{HwError, Result};
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// Pololu USB vendor ID.
pub const POLOLU_VID: u16 = 0x1FFB;
/// Maestro product IDs (6, 12, 18, 24 channels).
pub const MAESTRO_PIDS: [u16; 4] = [0x0089, 0x008A, 0x008B, 0x008C];

/// Lists Maestro command ports (interface 0 of the composite device).
pub fn find_maestro_ports() -> Result<Vec<SerialPortFound>> {
    Ok(list_usb_serial(Some(POLOLU_VID))?
        .into_iter()
        .filter(|p| MAESTRO_PIDS.contains(&p.pid))
        .filter(|p| match p.interface {
            Some(i) => i == 0,
            None => p.product.as_deref().is_none_or(|s| !s.contains("TTL")),
        })
        .collect())
}

/// Calibration of one servo: angle to pulse width.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoCal {
    pub channel: u8,
    /// Pulse at angle 0 (us).
    pub center_us: f32,
    /// Microseconds per degree (negative to reverse).
    pub us_per_deg: f32,
    pub min_us: f32,
    pub max_us: f32,
}

impl ServoCal {
    /// A typical 180-degree servo: 500..2500 us for -90..+90 degrees.
    pub fn servo180(channel: u8) -> ServoCal {
        ServoCal {
            channel,
            center_us: 1500.0,
            us_per_deg: 1000.0 / 90.0,
            min_us: 500.0,
            max_us: 2500.0,
        }
    }

    pub fn us_for(&self, deg: f64) -> f32 {
        (self.center_us + deg as f32 * self.us_per_deg).clamp(self.min_us, self.max_us)
    }

    pub fn deg_for(&self, us: f32) -> f64 {
        ((us - self.center_us) / self.us_per_deg) as f64
    }

    /// Angle range reachable within the pulse limits.
    pub fn range_deg(&self) -> (f64, f64) {
        let a = self.deg_for(self.min_us);
        let b = self.deg_for(self.max_us);
        (a.min(b), a.max(b))
    }
}

/// A Maestro on any byte link.
pub struct Maestro {
    link: Box<dyn SerialLink>,
    timeout: Duration,
}

fn seven(v: u16) -> [u8; 2] {
    [(v & 0x7F) as u8, ((v >> 7) & 0x7F) as u8]
}

impl Maestro {
    pub fn new(link: Box<dyn SerialLink>) -> Maestro {
        Maestro {
            link,
            timeout: Duration::from_millis(100),
        }
    }

    fn send(&mut self, b: &[u8]) -> Result<()> {
        self.link.write_all(b).map_err(HwError::Io)
    }

    fn read_exact(&mut self, n: usize) -> Result<Vec<u8>> {
        let mut out = Vec::with_capacity(n);
        let t0 = Instant::now();
        let mut buf = [0u8; 8];
        while out.len() < n {
            if t0.elapsed() > self.timeout {
                return Err(HwError::Timeout("Maestro did not answer".into()));
            }
            let k = self.link.read(&mut buf[..n - out.len()], Duration::from_millis(10))?;
            out.extend_from_slice(&buf[..k]);
        }
        Ok(out)
    }

    /// Target pulse in microseconds (0 stops pulses).
    pub fn set_target_us(&mut self, channel: u8, us: f32) -> Result<()> {
        let q = (us * 4.0).round().clamp(0.0, 16383.0) as u16;
        let [lo, hi] = seven(q);
        self.send(&[0x84, channel, lo, hi])
    }

    /// Speed limit (0.25 us per 10 ms units; 0 = unlimited).
    pub fn set_speed(&mut self, channel: u8, speed: u16) -> Result<()> {
        let [lo, hi] = seven(speed.min(16383));
        self.send(&[0x87, channel, lo, hi])
    }

    pub fn set_acceleration(&mut self, channel: u8, accel: u8) -> Result<()> {
        let [lo, hi] = seven(accel as u16);
        self.send(&[0x89, channel, lo, hi])
    }

    /// Current pulse in microseconds.
    pub fn get_position_us(&mut self, channel: u8) -> Result<f32> {
        self.send(&[0x90, channel])?;
        let b = self.read_exact(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]) as f32 / 4.0)
    }

    pub fn is_moving(&mut self) -> Result<bool> {
        self.send(&[0x93])?;
        Ok(self.read_exact(1)?[0] != 0)
    }

    pub fn get_errors(&mut self) -> Result<u16> {
        self.send(&[0xA1])?;
        let b = self.read_exact(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }

    pub fn go_home(&mut self) -> Result<()> {
        self.send(&[0xA2])
    }

    pub fn set_angle(&mut self, cal: &ServoCal, deg: f64) -> Result<()> {
        self.set_target_us(cal.channel, cal.us_for(deg))
    }

    pub fn get_angle(&mut self, cal: &ServoCal) -> Result<f64> {
        Ok(cal.deg_for(self.get_position_us(cal.channel)?))
    }
}

pub mod sim {
    //! A simulated Maestro with servos that slew at a fixed rate.
    use super::*;
    use std::io;
    use std::sync::{Arc, Mutex};

    #[derive(Debug)]
    pub struct SimMaestroState {
        /// Quarter microseconds per channel.
        pub target: [u16; 24],
        pub position: [f64; 24],
        pub speed: [u16; 24],
        pub accel: [u8; 24],
        /// Servo slew limit in quarter-us per second.
        pub servo_rate: f64,
        pub out: Vec<u8>,
        inbuf: Vec<u8>,
        pub commands: u64,
    }

    #[derive(Clone)]
    pub struct SimMaestro(pub Arc<Mutex<SimMaestroState>>);

    impl SimMaestro {
        pub fn new(servo_rate_us_per_s: f64) -> SimMaestro {
            SimMaestro(Arc::new(Mutex::new(SimMaestroState {
                target: [6000; 24],
                position: [6000.0; 24],
                speed: [0; 24],
                accel: [0; 24],
                servo_rate: servo_rate_us_per_s * 4.0,
                out: Vec::new(),
                inbuf: Vec::new(),
                commands: 0,
            })))
        }

        /// Advances the servos by `dt` seconds.
        pub fn advance(&self, dt: f64) {
            let mut s = self.0.lock().expect("sim lock");
            for c in 0..24 {
                let t = s.target[c] as f64;
                let mut rate = s.servo_rate;
                if s.speed[c] > 0 {
                    rate = rate.min(s.speed[c] as f64 * 100.0);
                }
                let d = (t - s.position[c]).clamp(-rate * dt, rate * dt);
                s.position[c] += d;
            }
        }

        pub fn link(&self) -> Box<dyn SerialLink> {
            Box::new(SimMaestroLink(self.clone()))
        }

        pub fn position_us(&self, ch: usize) -> f64 {
            self.0.lock().expect("sim lock").position[ch] / 4.0
        }
    }

    struct SimMaestroLink(SimMaestro);

    impl SerialLink for SimMaestroLink {
        fn read(&mut self, buf: &mut [u8], _t: Duration) -> io::Result<usize> {
            let mut s = self.0 .0.lock().expect("sim lock");
            let n = buf.len().min(s.out.len());
            for (k, b) in s.out.drain(..n).enumerate() {
                buf[k] = b;
            }
            Ok(n)
        }

        fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
            let mut s = self.0 .0.lock().expect("sim lock");
            s.inbuf.extend_from_slice(data);
            loop {
                let need = match s.inbuf.first() {
                    None => break,
                    Some(0x84) | Some(0x87) | Some(0x89) => 4,
                    Some(0x90) => 2,
                    Some(0x93) | Some(0xA1) | Some(0xA2) => 1,
                    Some(_) => {
                        s.inbuf.remove(0);
                        continue;
                    }
                };
                if s.inbuf.len() < need {
                    break;
                }
                let cmd: Vec<u8> = s.inbuf.drain(..need).collect();
                s.commands += 1;
                let v = |c: &[u8]| c[2] as u16 | (c[3] as u16) << 7;
                match cmd[0] {
                    0x84 => s.target[cmd[1] as usize] = v(&cmd),
                    0x87 => s.speed[cmd[1] as usize] = v(&cmd),
                    0x89 => s.accel[cmd[1] as usize] = v(&cmd) as u8,
                    0x90 => {
                        let p = s.position[cmd[1] as usize].round() as u16;
                        s.out.extend_from_slice(&p.to_le_bytes());
                    }
                    0x93 => {
                        let moving = (0..24).any(|c| (s.position[c] - s.target[c] as f64).abs() > 1.0);
                        s.out.push(moving as u8);
                    }
                    0xA1 => s.out.extend_from_slice(&[0, 0]),
                    0xA2 => s.target = [6000; 24],
                    _ => {}
                }
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::SimMaestro;
    use super::*;

    #[test]
    fn compact_protocol_bytes_and_readback() {
        let sim = SimMaestro::new(1000.0);
        let mut m = Maestro::new(sim.link());
        m.set_target_us(0, 1750.0).unwrap();
        assert_eq!(sim.0.lock().unwrap().target[0], 7000);
        m.set_speed(1, 300).unwrap();
        m.set_acceleration(1, 20).unwrap();
        assert_eq!(sim.0.lock().unwrap().speed[1], 300);
        assert_eq!(sim.0.lock().unwrap().accel[1], 20);
        sim.advance(0.1);
        assert!(m.is_moving().unwrap());
        let p = m.get_position_us(0).unwrap();
        assert!((p - 1600.0).abs() < 1.0, "{p}");
        sim.advance(1.0);
        assert!((m.get_position_us(0).unwrap() - 1750.0).abs() < 0.5);
        assert_eq!(m.get_errors().unwrap(), 0);
        m.go_home().unwrap();
        assert_eq!(sim.0.lock().unwrap().target[0], 6000);
        // 7000 quarter-us encodes as 0x84 ch 0x58 0x36.
        assert_eq!(seven(7000), [0x58, 0x36]);
    }

    #[test]
    fn servo_calibration() {
        let c = ServoCal::servo180(2);
        assert_eq!(c.us_for(0.0), 1500.0);
        assert_eq!(c.us_for(90.0), 2500.0);
        assert_eq!(c.us_for(200.0), 2500.0);
        assert!((c.deg_for(1000.0) + 45.0).abs() < 1e-6);
        let (a, b) = c.range_deg();
        assert!((a + 90.0).abs() < 1e-6 && (b - 90.0).abs() < 1e-6);
        let _ = find_maestro_ports().unwrap();
    }
}
