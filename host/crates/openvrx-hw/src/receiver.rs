// SPDX-License-Identifier: GPL-3.0-or-later
//! One receiver: a C5 radio head paired with an FX2 bridge.
//!
//! The `Receiver` owns the desired C5 settings (through its [`C5Client`],
//! which pushes them after every connect or reset), the FX2 capture stream
//! (whose block channel survives restarts), the IFCLK polarity found by
//! calibration, and the procedures that need both devices: sampling-phase
//! calibration, the lane self-test, and pairing by self-test pattern.
//!
//! A receiver can also run FX2 only ([`Receiver::start_fx2_only`]): the C5
//! is powered on its own and runs the settings stored with `save`, and the
//! host only captures. Settings changes then only reconfigure the host side,
//! and calibration and the lane self-test are unavailable.

use crate::c5::client::{C5Client, C5Event, ClientConfig};
use crate::c5::proto::{C5Settings, SelftestMode};
use crate::c5::transport::{PortConnector, PortSelector, SerialConnector};
use crate::calib::phase::{calibrate, CalibConfig, CalibIo, CalibReport};
use crate::calib::selftest::{analyze_walk, const_word, lanes_of, match_fraction, WalkReport};
use crate::error::{HwError, Result};
use crate::fx2::protocol::{StartFlags, PACKET_BYTES};
use crate::fx2::stream::{block_channel, capture_once, BlockSink, Fx2Stream, StreamConfig, StreamPort, StreamStats};
use crate::fx2::usb::{Fx2Selector, NusbPort};
use crossbeam_channel::Receiver as Chan;
use openvrx_core::{LaneFormat, RawBlock, RxId};
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Creates a fresh FX2 port (called for the stream and for one-shot
/// captures).
pub type PortFactory = Box<dyn FnMut() -> Box<dyn StreamPort> + Send>;

/// Receiver configuration (user config or auto pairing).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReceiverConfig {
    pub rx: RxId,
    pub c5: PortSelector,
    pub fx2: Fx2Selector,
    pub settings: C5Settings,
    /// Stream settings; width comes from the lane preset, IFCLK polarity
    /// from calibration.
    pub stream: StreamConfig,
    pub calib: CalibConfig,
    /// Run phase calibration after every C5 boot.
    pub calibrate_on_boot: bool,
    pub ifclk_inverted: bool,
    pub block_capacity: usize,
}

impl ReceiverConfig {
    pub fn new(rx: RxId, c5: PortSelector, fx2: Fx2Selector, settings: C5Settings) -> ReceiverConfig {
        ReceiverConfig {
            rx,
            c5,
            fx2,
            settings,
            stream: StreamConfig {
                rx,
                ..Default::default()
            },
            calib: CalibConfig::default(),
            calibrate_on_boot: true,
            ifclk_inverted: false,
            block_capacity: 64,
        }
    }
}

/// Events a control thread acts on.
#[derive(Clone, Debug, PartialEq)]
pub enum ReceiverEvent {
    C5(C5Event),
    Calibrated(CalibReport),
    CalibrationFailed(String),
}

/// A running receiver.
pub struct Receiver {
    cfg: ReceiverConfig,
    /// None when running FX2 only.
    c5: Option<C5Client>,
    ports: PortFactory,
    sink: BlockSink,
    blocks: Chan<RawBlock>,
    stream: Option<Fx2Stream>,
    boot_pending: bool,
    last_calibration: Option<CalibReport>,
}

/// Start flags for a lane format and IFCLK polarity.
pub fn stream_flags(lanes: &LaneFormat, ifclk_inv: bool) -> StartFlags {
    StartFlags::for_word_bytes(lanes.word_bytes).with(StartFlags::IFCLK_INV, ifclk_inv)
}

impl Receiver {
    /// Opens the configured C5 port and FX2 and starts streaming.
    pub fn start(cfg: ReceiverConfig) -> Result<Receiver> {
        let sel = cfg.fx2.clone();
        let connector = PortConnector::new(cfg.c5.clone());
        Self::start_with(
            cfg,
            connector,
            Box::new(move || Box::new(NusbPort::new(sel.clone())) as Box<dyn StreamPort>),
        )
    }

    /// Starts with any C5 connector and FX2 port factory (simulations).
    pub fn start_with<C: SerialConnector>(cfg: ReceiverConfig, connector: C, ports: PortFactory) -> Result<Receiver> {
        cfg.settings.validate()?;
        let c5 = C5Client::start(connector, Some(cfg.settings.clone()), ClientConfig::default())?;
        Self::start_inner(cfg, Some(c5), ports)
    }

    /// Starts on the FX2 alone. `cfg.settings` must describe what the C5 was
    /// left running (lanes and divider decide the stream width and rate);
    /// `cfg.c5` is ignored.
    pub fn start_fx2_only(cfg: ReceiverConfig) -> Result<Receiver> {
        let sel = cfg.fx2.clone();
        Self::start_fx2_only_with(
            cfg,
            Box::new(move || Box::new(NusbPort::new(sel.clone())) as Box<dyn StreamPort>),
        )
    }

    /// FX2-only start with any port factory (simulations).
    pub fn start_fx2_only_with(cfg: ReceiverConfig, ports: PortFactory) -> Result<Receiver> {
        cfg.settings.validate()?;
        Self::start_inner(cfg, None, ports)
    }

    fn start_inner(cfg: ReceiverConfig, c5: Option<C5Client>, ports: PortFactory) -> Result<Receiver> {
        let (sink, blocks) = block_channel(cfg.block_capacity);
        let mut r = Receiver {
            cfg,
            c5,
            ports,
            sink,
            blocks,
            stream: None,
            boot_pending: false,
            last_calibration: None,
        };
        r.start_stream()?;
        Ok(r)
    }

    pub fn rx(&self) -> RxId {
        self.cfg.rx
    }

    pub fn config(&self) -> &ReceiverConfig {
        &self.cfg
    }

    /// The C5 console client, or None when running FX2 only.
    pub fn c5(&self) -> Option<&C5Client> {
        self.c5.as_ref()
    }

    fn c5_required(&self, what: &str) -> Result<&C5Client> {
        self.c5
            .as_ref()
            .ok_or_else(|| HwError::Invalid(format!("{what} needs the C5 console (running FX2 only)")))
    }

    /// Raw blocks (the channel survives stream restarts).
    pub fn blocks(&self) -> &Chan<RawBlock> {
        &self.blocks
    }

    pub fn stream_stats(&self) -> Option<StreamStats> {
        self.stream.as_ref().map(|s| s.stats())
    }

    pub fn stream(&self) -> Option<&Fx2Stream> {
        self.stream.as_ref()
    }

    pub fn last_calibration(&self) -> Option<&CalibReport> {
        self.last_calibration.as_ref()
    }

    /// Current desired settings (owned by the C5 client).
    pub fn settings(&self) -> C5Settings {
        self.c5
            .as_ref()
            .and_then(|c| c.settings())
            .unwrap_or_else(|| self.cfg.settings.clone())
    }

    /// Replaces the settings; restarts the stream when the word width
    /// changes.
    pub fn set_settings(&mut self, s: C5Settings) -> Result<()> {
        let width_changed = s.lanes.format().word_bytes != self.settings().lanes.format().word_bytes;
        if let Some(c5) = &self.c5 {
            c5.set_settings(s.clone())?;
        }
        self.cfg.settings = s;
        if width_changed {
            self.restart_stream()?;
        }
        Ok(())
    }

    fn lanes(&self) -> LaneFormat {
        self.settings().lanes.format()
    }

    fn start_stream(&mut self) -> Result<()> {
        let mut scfg = self.cfg.stream.clone();
        scfg.rx = self.cfg.rx;
        scfg.flags = scfg
            .flags
            .with(StartFlags::WIDTH16, self.lanes().word_bytes == 2)
            .with(StartFlags::IFCLK_INV, self.cfg.ifclk_inverted);
        scfg.nominal_byte_rate = Some(self.settings().sample_format().byte_rate());
        let port = (self.ports)();
        self.stream = Some(Fx2Stream::start_with_sink(port, scfg, self.sink.clone())?);
        Ok(())
    }

    fn stop_stream(&mut self) {
        if let Some(s) = self.stream.take() {
            s.stop();
        }
    }

    pub fn restart_stream(&mut self) -> Result<()> {
        self.stop_stream();
        self.start_stream()
    }

    /// Switches the FX2 to watch mode (n > 1) or full stream (n = 1).
    pub fn set_watch(&mut self, n: u16) {
        self.cfg.stream.watch_n = n.max(1);
        if let Some(s) = &self.stream {
            s.set_watch(n.max(1));
        }
    }

    /// Runs sampling-phase calibration (stream paused meanwhile).
    pub fn calibrate(&mut self) -> Result<CalibReport> {
        self.c5_required("calibration")?;
        self.stop_stream();
        let lanes = self.lanes();
        let divider = self.settings().divider;
        let result = {
            let mut io = LiveCalibIo {
                c5: self.c5.as_ref().expect("checked above"),
                ports: &mut self.ports,
                lanes: lanes.clone(),
            };
            calibrate(&mut io, &lanes, divider, &self.cfg.calib)
        };
        if let Ok(r) = &result {
            if r.success {
                self.cfg.ifclk_inverted = r.ifclk_inverted;
            }
            self.last_calibration = Some(r.clone());
        }
        let restart = self.start_stream();
        let r = result?;
        restart?;
        Ok(r)
    }

    /// Runs the `walk` lane self-test and analyses it (stream paused).
    pub fn lane_selftest(&mut self) -> Result<WalkReport> {
        self.c5_required("the lane self-test")?;
        self.stop_stream();
        let result = self.walk_inner();
        if let Some(c5) = &self.c5 {
            let _ = c5.selftest(&SelftestMode::Off);
        }
        let restart = self.start_stream();
        let r = result?;
        restart?;
        Ok(r)
    }

    fn walk_inner(&mut self) -> Result<WalkReport> {
        let s = self.settings();
        let lanes = s.lanes.format();
        let c5 = self.c5_required("the lane self-test")?;
        let pins = c5.pins().ok();
        let reply = c5.selftest(&SelftestMode::Walk)?;
        let fs = s.sample_format().sample_rate_hz();
        let phase_len = reply.phase_us as f64 * 1e-6 * fs;
        let nl = lanes_of(&lanes, None).len();
        let samples = (phase_len * (nl as f64 + 2.0) * 3.0) as usize;
        let bytes = samples * lanes.word_bytes as usize;
        let flags = stream_flags(&lanes, self.cfg.ifclk_inverted);
        let mut port = (self.ports)();
        let secs = 2.0 + 3.0 * samples as f64 / fs;
        let transfers = capture_once(port.as_mut(), flags, bytes, 64 * 1024, Duration::from_secs_f64(secs))?;
        let data: Vec<u8> = transfers.concat();
        Ok(analyze_walk(&data, &lanes, phase_len, pins.as_ref()))
    }

    /// Drains C5 events; after a C5 boot (once its state push finished)
    /// re-runs calibration when configured. Call from the control thread.
    pub fn poll(&mut self) -> Vec<ReceiverEvent> {
        let Some(c5) = &self.c5 else {
            return Vec::new();
        };
        let events: Vec<C5Event> = c5.events().try_iter().collect();
        let mut out = Vec::with_capacity(events.len());
        for e in events {
            match &e {
                C5Event::Boot { .. } => self.boot_pending = true,
                C5Event::StatePushed { .. } if self.boot_pending && self.cfg.calibrate_on_boot => {
                    self.boot_pending = false;
                    out.push(ReceiverEvent::C5(e.clone()));
                    match self.calibrate() {
                        Ok(r) => out.push(ReceiverEvent::Calibrated(r)),
                        Err(err) => out.push(ReceiverEvent::CalibrationFailed(err.to_string())),
                    }
                    continue;
                }
                _ => {}
            }
            out.push(ReceiverEvent::C5(e));
        }
        out
    }
}

impl Drop for Receiver {
    fn drop(&mut self) {
        self.stop_stream();
    }
}

struct LiveCalibIo<'a> {
    c5: &'a C5Client,
    ports: &'a mut PortFactory,
    lanes: LaneFormat,
}

impl CalibIo for LiveCalibIo<'_> {
    fn set_divider(&mut self, divider: u32) -> Result<()> {
        self.c5.clk_transient(divider).map(|_| ())
    }

    fn slip(&mut self) -> Result<u32> {
        self.c5.slip(1)
    }

    fn capture_packets(&mut self, ifclk_inv: bool, packets: usize) -> Result<Vec<Vec<u8>>> {
        let flags = stream_flags(&self.lanes, ifclk_inv);
        let mut port = (self.ports)();
        let t = capture_once(
            port.as_mut(),
            flags,
            packets * PACKET_BYTES,
            16 * 1024,
            Duration::from_secs(3),
        )?;
        Ok(t.iter()
            .flat_map(|x| x.as_chunks::<PACKET_BYTES>().0.iter().map(|p| p.to_vec()))
            .take(packets)
            .collect())
    }
}

/// Hardware access needed by pairing.
pub trait PairIo {
    fn c5_count(&self) -> usize;
    fn fx2_count(&self) -> usize;
    /// Lane format of a C5 (from its settings).
    fn lanes(&self, c5: usize) -> LaneFormat;
    /// `selftest const HEX` (Some) or `selftest off` (None).
    fn set_const(&mut self, c5: usize, hex: Option<u32>) -> Result<()>;
    /// A short capture from an FX2 with the given word width.
    fn capture(&mut self, fx2: usize, lanes: &LaneFormat) -> Result<Vec<u8>>;
}

/// Pattern pair used to identify a C5 (complementary, so a stuck port
/// cannot match both).
pub fn pairing_patterns(c5: usize, lanes: usize) -> (u32, u32) {
    let mask = if lanes >= 32 { u32::MAX } else { (1u32 << lanes) - 1 };
    let a = (0x5A5A_5A5A ^ (c5 as u32).wrapping_mul(0x0F0F_1337)) & mask;
    (a, !a & mask)
}

/// Pairs C5s with FX2s by driving a constant pattern from one C5 at a time
/// and finding the FX2 that reads it. Returns, per C5, the FX2 index.
pub fn auto_pair(io: &mut dyn PairIo, min_match: f32) -> Result<Vec<Option<usize>>> {
    let mut taken = vec![false; io.fx2_count()];
    let n_c5 = io.c5_count();
    let mut out = vec![None; n_c5];
    for (c, slot) in out.iter_mut().enumerate() {
        let lanes = io.lanes(c);
        let n = lanes_of(&lanes, None).len();
        let (p1, p2) = pairing_patterns(c, n);
        let mut hits = vec![true; io.fx2_count()];
        for p in [p1, p2] {
            io.set_const(c, Some(p))?;
            let want = const_word(&lanes, p);
            for f in 0..io.fx2_count() {
                if taken[f] || !hits[f] {
                    hits[f] = false;
                    continue;
                }
                let data = match io.capture(f, &lanes) {
                    Ok(d) => d,
                    Err(e) => {
                        log::warn!("pairing capture on FX2 {f} failed: {e}");
                        hits[f] = false;
                        continue;
                    }
                };
                hits[f] = match_fraction(&data, &lanes, want) >= min_match;
            }
        }
        io.set_const(c, None)?;
        let found: Vec<usize> = (0..hits.len()).filter(|&f| hits[f]).collect();
        match found.len() {
            0 => {}
            1 => {
                taken[found[0]] = true;
                *slot = Some(found[0]);
            }
            _ => {
                return Err(HwError::Protocol(format!(
                    "C5 {c}'s pattern appears on {} FX2 boards; are their lanes wired together?",
                    found.len()
                )))
            }
        }
    }
    Ok(out)
}

/// [`PairIo`] over live C5 clients and FX2 selectors.
pub struct LivePairIo<'a> {
    pub c5: Vec<&'a C5Client>,
    pub fx2: Vec<Fx2Selector>,
}

impl PairIo for LivePairIo<'_> {
    fn c5_count(&self) -> usize {
        self.c5.len()
    }
    fn fx2_count(&self) -> usize {
        self.fx2.len()
    }
    fn lanes(&self, c5: usize) -> LaneFormat {
        self.c5[c5].settings().unwrap_or_default().lanes.format()
    }
    fn set_const(&mut self, c5: usize, hex: Option<u32>) -> Result<()> {
        let mode = hex.map_or(SelftestMode::Off, SelftestMode::Const);
        self.c5[c5].selftest(&mode).map(|_| ())
    }
    fn capture(&mut self, fx2: usize, lanes: &LaneFormat) -> Result<Vec<u8>> {
        let mut port = NusbPort::new(self.fx2[fx2].clone());
        let t = capture_once(
            &mut port,
            stream_flags(lanes, false),
            64 * 1024,
            16 * 1024,
            Duration::from_millis(500),
        )?;
        Ok(t.concat())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::c5::transport::sim::SimC5;
    use crate::calib::selftest::testgen::{walk_capture, Wiring};
    use crate::fx2::endpoint::sim::SimFx2;
    use openvrx_core::LanePreset;
    use std::sync::{Arc, Mutex};

    /// Simulated pairing world: C5 k is wired to FX2 wiring[k].
    struct World {
        wiring: Vec<Option<usize>>,
        driving: Vec<Option<u32>>,
        lanes: LaneFormat,
    }

    impl PairIo for World {
        fn c5_count(&self) -> usize {
            self.wiring.len()
        }
        fn fx2_count(&self) -> usize {
            3
        }
        fn lanes(&self, _: usize) -> LaneFormat {
            self.lanes.clone()
        }
        fn set_const(&mut self, c5: usize, hex: Option<u32>) -> Result<()> {
            self.driving[c5] = hex;
            Ok(())
        }
        fn capture(&mut self, fx2: usize, lanes: &LaneFormat) -> Result<Vec<u8>> {
            let src = self.wiring.iter().position(|w| *w == Some(fx2));
            let word = match src.and_then(|c| self.driving[c]) {
                Some(p) => const_word(lanes, p),
                None => 0x1234 ^ fx2 as u32, // radio noise
            };
            Ok((0..1000).flat_map(|_| (word as u16).to_le_bytes()).collect())
        }
    }

    #[test]
    fn pairs_by_pattern() {
        let mut w = World {
            wiring: vec![Some(2), Some(0), None],
            driving: vec![None; 3],
            lanes: LanePreset::I7Q7.format(),
        };
        let p = auto_pair(&mut w, 0.9).unwrap();
        assert_eq!(p, vec![Some(2), Some(0), None]);
        assert!(w.driving.iter().all(Option::is_none), "self-test left on");
        let (a, b) = pairing_patterns(1, 14);
        assert_eq!(a & b, 0);
        assert_eq!(a | b, 0x3FFF);
    }

    /// A port factory over one simulated FX2 whose data comes from a
    /// shared generator (so tests can swap in self-test patterns).
    fn factory(sim: &SimFx2) -> PortFactory {
        let s = sim.clone();
        Box::new(move || Box::new(s.port()) as Box<dyn StreamPort>)
    }

    fn rcfg() -> ReceiverConfig {
        let mut c = ReceiverConfig::new(
            0,
            PortSelector::Name("SIM".into()),
            Fx2Selector::Only,
            C5Settings::default(),
        );
        c.stream.mmcss = false;
        c.stream.transfers = 8;
        c.stream.transfer_bytes = 16 * 1024;
        c
    }

    #[test]
    fn streams_and_runs_the_walk_selftest() {
        let c5 = SimC5::new();
        let fx2 = SimFx2::new(26.67e6);
        let lanes = LanePreset::I7Q7.format();
        // While the C5 is in `walk`, the FX2 sees the walk pattern
        // (phase_us 1000 at 13.33 MS/s, with PB2/PB3 swapped).
        let mut wiring = Wiring::ideal(&lanes);
        wiring.drives.swap(1, 2);
        let phase_len = 13_333usize;
        let pattern = walk_capture(&lanes, &wiring, phase_len, phase_len * 16 * 4, 0, 500);
        let pos = Arc::new(Mutex::new(0usize));
        let c5c = c5.clone();
        fx2.with(|d| {
            d.generator = Some(Box::new(move |buf: &mut [u8], _| {
                if c5c.with(|s| s.selftest == "walk") {
                    let mut p = pos.lock().unwrap();
                    for b in buf.iter_mut() {
                        *b = pattern[*p % pattern.len()];
                        *p += 1;
                    }
                } else {
                    buf.fill(0x11);
                }
            }));
        });
        let mut r = Receiver::start_with(rcfg(), c5.connector(), factory(&fx2)).unwrap();
        let b = r.blocks().recv_timeout(Duration::from_secs(3)).unwrap();
        assert!(b.data.iter().all(|&x| x == 0x11));
        let report = r.lane_selftest().unwrap();
        assert!(!report.ok);
        assert!(report.summary().contains("swapped"), "{}", report.summary());
        assert_eq!(c5.with(|s| s.selftest.clone()), "off");
        // Streaming resumed into the same channel.
        assert!(r.blocks().recv_timeout(Duration::from_secs(3)).is_ok());
        assert!(fx2.with(|d| d.flags.contains(StartFlags::WIDTH16)));
    }

    #[test]
    fn calibration_after_boot_restores_divider_and_stream() {
        use crate::calib::probe::testsig::{mix, signal, to_bytes, Rng};
        let c5 = SimC5::new();
        let fx2 = SimFx2::new(26.67e6);
        let lanes = LanePreset::I7Q7.format();
        // Position (slip total mod 3) 0 is mixed at 40 MS/s.
        let c5c = c5.clone();
        let l2 = lanes.clone();
        let rng = Arc::new(Mutex::new(Rng(7)));
        fx2.with(|d| {
            d.generator = Some(Box::new(move |buf: &mut [u8], _| {
                let (div, slips) = c5c.with(|s| (s.divider, s.slips));
                let n = buf.len() / 2;
                let sig = signal(n, 150.0, 8.0, &mut rng.lock().unwrap());
                let w: Vec<u32> = sig.iter().map(|&(i, q)| l2.encode(i, q)).collect();
                let w = if div == 6 && slips % 3 == 0 { mix(&w, 0xC080) } else { w };
                buf.copy_from_slice(&to_bytes(&w, 2));
            }));
        });
        let mut r = Receiver::start_with(rcfg(), c5.connector(), factory(&fx2)).unwrap();
        assert!(r.c5().unwrap().wait_connected(Duration::from_secs(2)));
        c5.reset_in_place();
        let t0 = std::time::Instant::now();
        let mut cal = None;
        while t0.elapsed() < Duration::from_secs(10) && cal.is_none() {
            for e in r.poll() {
                if let ReceiverEvent::Calibrated(c) = e {
                    cal = Some(c);
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let cal = cal.expect("calibration ran after the boot event");
        assert!(cal.success, "{}", cal.message);
        assert_eq!(cal.slips_applied, 1);
        assert_eq!(c5.with(|s| s.divider), 18);
        assert_eq!(r.settings().divider, 18);
        assert!(r.blocks().recv_timeout(Duration::from_secs(3)).is_ok());
    }
}
