// SPDX-License-Identifier: GPL-3.0-or-later
//! `openvrx-cli scan`: a band scan on the live receiver (DSP-25).
//!
//! The C5 keeps everything it runs (lanes, clock, board, gain loop, DC):
//! the console opens without pushing settings, as `phasecal` does, and the
//! scan only retunes. For each frequency of the chosen bands
//! (`openvrx_hw::control::run_scan`): tune, drop the queued blocks (they are
//! from before the retune), feed the settling time to a fresh channel meter
//! without counting it (the C5's gain loop, the DC and I/Q correction and
//! the sync search start over), then the dwell
//! (`openvrx_dsp::meter::ChannelMeter`). Times are counted in samples of the
//! stream, not on the host clock. The level is the dwell's I/Q power less
//! the C5's gain in dB (`status gain=`, measured slopes), so channels read
//! at different gains compare. At the end the C5 goes back to the channel
//! it was on, or with `--stay` to the best transmitter found.

use crate::{ensure_fx2_firmware, opt, opt_num};
use openvrx_core::lanes::SampleFormat;
use openvrx_core::LanePreset;
use openvrx_dsp::meter::{ChannelMeter, ChannelReading};
use openvrx_hw::c5::{find_c5_ports, C5Client, C5Settings, ClientConfig, PortConnector, PortSelector};
use openvrx_hw::control::scan::{channels_in, Band, Channel};
use openvrx_hw::control::{run_scan, ChannelMetrics, GainTable, ScanConfig, ScanIo, ScanReport};
use openvrx_hw::fx2::Fx2Selector;
use openvrx_hw::receiver::{Receiver, ReceiverConfig};
use openvrx_hw::HwError;
use serde::Serialize;
use std::time::{Duration, Instant};

/// What the scan needs from one receiver.
pub trait Radio {
    fn tune(&mut self, mhz: u16) -> Result<(), String>;
    /// Next block of lane bytes; None when none came within `timeout`.
    fn next_block(&mut self, timeout: Duration) -> Option<Vec<u8>>;
    /// Drops the blocks queued so far.
    fn drain(&mut self);
    /// Gain index in force (None: unknown).
    fn gain_index(&mut self) -> Option<u32>;
}

/// One channel's dwell, for the JSON report.
#[derive(Clone, Debug, Serialize)]
pub struct Dwell {
    pub mhz: u16,
    pub gain: Option<u32>,
    pub reading: ChannelReading,
}

/// [`ScanIo`] over radios and channel meters.
pub struct LiveScan<R: Radio> {
    radios: Vec<R>,
    fmt: SampleFormat,
    meters: Vec<Option<(u16, ChannelMeter)>>,
    table: GainTable,
    pub dwells: Vec<Dwell>,
}

/// Longest wait for a block before the stream counts as stopped.
const BLOCK_TIMEOUT: Duration = Duration::from_secs(1);

impl<R: Radio> LiveScan<R> {
    pub fn new(radios: Vec<R>, fmt: SampleFormat, table: GainTable) -> LiveScan<R> {
        let n = radios.len();
        LiveScan {
            radios,
            fmt,
            meters: (0..n).map(|_| None).collect(),
            table,
            dwells: Vec::new(),
        }
    }

    #[cfg(test)]
    fn radios_mut(&mut self) -> &mut [R] {
        &mut self.radios
    }

    /// Feeds `d` of stream time to receiver `rx`'s meter (warm-up or dwell).
    fn feed(&mut self, rx: usize, d: Duration, dwell: bool) -> openvrx_hw::Result<()> {
        let (radio, slot) = (&mut self.radios[rx], &mut self.meters[rx]);
        let Some((_, meter)) = slot.as_mut() else {
            return Err(HwError::Invalid(format!("receiver {rx} measured before a tune")));
        };
        let want = meter.samples() + (d.as_secs_f64() * meter.sample_rate()) as u64;
        while meter.samples() < want {
            let b = radio
                .next_block(BLOCK_TIMEOUT)
                .ok_or_else(|| HwError::Invalid(format!("no data from receiver {rx} in {BLOCK_TIMEOUT:?}")))?;
            if dwell {
                meter.push(&b);
            } else {
                meter.warm(&b);
            }
        }
        Ok(())
    }
}

impl<R: Radio> ScanIo for LiveScan<R> {
    fn tune(&mut self, rx: usize, mhz: u16) -> openvrx_hw::Result<()> {
        let radio = self
            .radios
            .get_mut(rx)
            .ok_or_else(|| HwError::Invalid(format!("no receiver {rx}")))?;
        radio.tune(mhz).map_err(HwError::Invalid)?;
        radio.drain();
        self.meters[rx] = Some((mhz, ChannelMeter::new(self.fmt.clone())));
        Ok(())
    }

    fn wait(&mut self, d: Duration) {
        // The scanner tunes every active receiver, then waits once for all.
        // A stream that stops shows again in measure.
        for rx in 0..self.radios.len() {
            if self.meters[rx].is_some() {
                let _ = self.feed(rx, d, false);
            }
        }
    }

    fn measure(&mut self, rxs: &[usize], dwell: Duration) -> openvrx_hw::Result<Vec<ChannelMetrics>> {
        let mut out = Vec::with_capacity(rxs.len());
        for &rx in rxs {
            self.feed(rx, dwell, true)?;
            let gain = self.radios[rx].gain_index();
            let (mhz, meter) = self.meters[rx].take().expect("fed above");
            let reading = meter.reading();
            out.push(metrics(&reading, gain, &self.table));
            self.dwells.push(Dwell { mhz, gain, reading });
        }
        Ok(out)
    }
}

/// The scanner's view of a reading: the level gain-normalised when the gain
/// is known, C/N floored at -20 dB when there was no figure.
pub fn metrics(r: &ChannelReading, gain: Option<u32>, table: &GainTable) -> ChannelMetrics {
    ChannelMetrics {
        power_db: r.power_db - gain.map(|g| table.db_at(g)).unwrap_or(0.0),
        cn_db: if r.cn_db.is_finite() { r.cn_db } else { -20.0 },
        sync_score: r.sync_score,
    }
}

/// The live receiver: the C5 console and the FX2 stream.
struct LiveRadio<'a> {
    c5: &'a C5Client,
    receiver: &'a Receiver,
}

impl Radio for LiveRadio<'_> {
    fn tune(&mut self, mhz: u16) -> Result<(), String> {
        self.c5.tune(mhz as u32, 0).map(|_| ()).map_err(|e| format!("tune {mhz}: {e}"))
    }

    fn next_block(&mut self, timeout: Duration) -> Option<Vec<u8>> {
        self.receiver.blocks().recv_timeout(timeout).ok().map(|b| b.data)
    }

    fn drain(&mut self) {
        while self.receiver.blocks().try_recv().is_ok() {}
    }

    fn gain_index(&mut self) -> Option<u32> {
        self.c5.status().ok().map(|s| s.gain)
    }
}

/// Bands from letters (`RF`, `rabefl`); all six when absent.
fn parse_bands(s: Option<&str>) -> Result<Vec<Band>, String> {
    let Some(s) = s else { return Ok(Band::ALL.to_vec()) };
    s.chars()
        .map(|c| {
            Band::ALL
                .into_iter()
                .find(|b| b.letter() == c.to_ascii_uppercase())
                .ok_or_else(|| format!("unknown band {c} (bands: RABEFL)"))
        })
        .collect()
}

fn opt_ms(args: &[String], name: &str, default: Duration) -> Result<Duration, String> {
    match opt(args, name) {
        Some(v) => v.parse::<u64>().map(Duration::from_millis).map_err(|_| format!("bad {name} {v}")),
        None => Ok(default),
    }
}

/// Prints the ranked table and the transmitters.
fn print_report(report: &ScanReport, dwells: &[Dwell]) {
    println!("rank  MHz   channels   video  C/N dB  sync  level dB  gain");
    for (k, r) in report.ranked.iter().enumerate() {
        let d = dwells.iter().find(|d| d.mhz == r.mhz);
        let names = r.channels.join("/");
        match (&r.metrics, &r.error) {
            (Some(m), _) => println!(
                "{:>4}  {:>4}  {:<9}  {:<5}  {:>6.1}  {:>4.2}  {:>8.1}  {}{}",
                k + 1,
                r.mhz,
                names,
                if r.video { "yes" } else { "-" },
                m.cn_db,
                m.sync_score,
                m.power_db,
                d.and_then(|d| d.gain).map(|g| g.to_string()).unwrap_or_else(|| "-".into()),
                r.alias_of.map(|a| format!("  (same transmitter as {a})")).unwrap_or_default()
            ),
            (None, Some(e)) => println!("{:>4}  {:>4}  {:<9}  error: {e}", k + 1, r.mhz, names),
            (None, None) => {}
        }
    }
    let tx = report.transmitters();
    if tx.is_empty() {
        println!("no video found");
    } else {
        for t in tx {
            println!("video on {} MHz ({})", t.mhz, t.channels.join("/"));
        }
    }
}

/// `openvrx-cli scan --lanes P --clk D [--c5 PORT] [--bands RABEFL] [--settle MS] [--dwell MS]
/// [--json PATH] [--stay]`
pub fn scan(args: &[String]) -> Result<(), String> {
    let lanes: LanePreset = opt(args, "--lanes")
        .ok_or("scan needs --lanes PRESET")?
        .parse()
        .map_err(|e| format!("{e}"))?;
    let divider: u32 = opt_num(args, "--clk", 9)?;
    let bands = parse_bands(opt(args, "--bands"))?;
    let defaults = ScanConfig::default();
    let cfg = ScanConfig {
        settle: opt_ms(args, "--settle", defaults.settle)?,
        dwell: opt_ms(args, "--dwell", defaults.dwell)?,
        ..defaults
    };
    let stay = args.iter().any(|a| a == "--stay");

    let port = match opt(args, "--c5") {
        Some(p) => p.to_string(),
        None => find_c5_ports()
            .map_err(|e| e.to_string())?
            .into_iter()
            .next()
            .map(|p| p.port_name)
            .ok_or("no C5 on USB (scan needs its console to retune)")?,
    };
    let c5 = C5Client::start(PortConnector::new(PortSelector::Name(port.clone())), None, ClientConfig::default())
        .map_err(|e| e.to_string())?;
    if !c5.wait_connected(Duration::from_secs(3)) {
        return Err(format!("C5 on {port} does not answer"));
    }
    let st = c5.status().map_err(|e| format!("C5 status: {e}"))?;
    if st.lanes != lanes.name() || st.clk != divider || !st.clk_on {
        return Err(format!(
            "the C5 streams {} clk {} (clock {}), not {} clk {divider}",
            st.lanes,
            st.clk,
            if st.clk_on { "on" } else { "off" },
            lanes.name()
        ));
    }
    if st.agc == Some(false) {
        eprintln!("note: the C5's gain loop is off; every channel is read at gain {}", st.gain);
    }

    ensure_fx2_firmware()?;
    let settings = C5Settings {
        lanes,
        divider,
        ..C5Settings::default()
    };
    let fmt = settings.sample_format();
    let rcfg = ReceiverConfig::new(0, PortSelector::Name(String::new()), Fx2Selector::Only, settings);
    let receiver = Receiver::start_fx2_only(rcfg).map_err(|e| format!("starting the FX2: {e}"))?;

    let channels: Vec<Channel> = channels_in(&bands);
    let t0 = Instant::now();
    let mut io = LiveScan::new(vec![LiveRadio { c5: &c5, receiver: &receiver }], fmt, GainTable::c5_measured());
    let report = run_scan(&channels, 1, &cfg, &mut io);
    let took = t0.elapsed();
    print_report(&report, &io.dwells);
    println!("{} frequencies in {:.1} s", io.dwells.len(), took.as_secs_f64());

    let (mhz, off) = match (stay, report.transmitters().first()) {
        (true, Some(t)) => (t.mhz as u32, 0),
        _ => (st.mhz, st.off_khz),
    };
    c5.tune(mhz, off).map_err(|e| format!("tune back to {mhz}: {e}"))?;
    println!("C5 on {mhz} MHz{}", if off != 0 { format!(" {off:+} kHz") } else { String::new() });

    if let Some(path) = opt(args, "--json") {
        let j = serde_json::json!({ "report": report, "dwells": io.dwells, "seconds": took.as_secs_f64() });
        std::fs::write(path, serde_json::to_string_pretty(&j).map_err(|e| e.to_string())?)
            .map_err(|e| format!("{path}: {e}"))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_dsp::testsig::{SigGen, SigSpec};

    /// A band with transmitters at (MHz, C/N dB). Tuned within 4 MHz of one,
    /// the receiver sees its carrier offset by the detuning and its C/N
    /// falling 3 dB per MHz; elsewhere receiver noise only (cheap random
    /// words, so the test stays fast).
    struct FakeBand {
        tx: Vec<(u16, f64)>,
        fmt: SampleFormat,
        gen: Option<SigGen>,
        seed: u64,
        rng: u64,
        tunes: Vec<u16>,
        drains: usize,
    }

    impl FakeBand {
        fn new(tx: Vec<(u16, f64)>) -> FakeBand {
            FakeBand {
                tx,
                fmt: SigSpec::default().sample_format,
                gen: None,
                seed: 1,
                rng: 0x9e37_79b9_7f4a_7c15,
                tunes: Vec::new(),
                drains: 0,
            }
        }

        fn noise_word(&mut self) -> u32 {
            // Two sums of four uniforms: roughly Gaussian I and Q, 100 ADC units RMS.
            let mut g = || {
                let mut s = 0i64;
                for _ in 0..4 {
                    self.rng ^= self.rng << 13;
                    self.rng ^= self.rng >> 7;
                    self.rng ^= self.rng << 17;
                    s += (self.rng >> 54) as i64 - 512;
                }
                (s * 100 / 591) as i32
            };
            let (i, q) = (g(), g());
            self.fmt.lanes.encode(i.clamp(-512, 511), q.clamp(-512, 511))
        }
    }

    impl Radio for FakeBand {
        fn tune(&mut self, mhz: u16) -> Result<(), String> {
            self.tunes.push(mhz);
            self.seed += 1;
            self.gen = self.tx.iter().find(|t| t.0.abs_diff(mhz) <= 4).map(|&(f, cn)| {
                let off_mhz = f as f64 - mhz as f64;
                SigGen::new(SigSpec {
                    cn_db: Some(cn - 3.0 * off_mhz.abs()),
                    blank_hz: -1.0e6 + off_mhz * 1e6,
                    seed: self.seed,
                    ..SigSpec::default()
                })
            });
            Ok(())
        }

        fn next_block(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            // About 1 ms per block.
            let n = (self.fmt.sample_rate_hz() * 1e-3) as usize;
            if let Some(g) = self.gen.as_mut() {
                return Some(g.next(n).bytes);
            }
            let wb = self.fmt.lanes.word_bytes as usize;
            let mut b = Vec::with_capacity(n * wb);
            for _ in 0..n {
                let w = self.noise_word();
                b.extend_from_slice(&w.to_le_bytes()[..wb]);
            }
            Some(b)
        }

        fn drain(&mut self) {
            self.drains += 1;
        }

        fn gain_index(&mut self) -> Option<u32> {
            Some(60)
        }
    }

    #[test]
    fn finds_the_transmitters_on_a_fake_band() {
        // R7 (5880, shared with F8) strong, B3 (5771) weak but locked.
        let band = FakeBand::new(vec![(5880, 20.0), (5771, 9.0)]);
        let fmt = band.fmt.clone();
        let mut io = LiveScan::new(vec![band], fmt, GainTable::c5_measured());
        let channels = channels_in(&[Band::R, Band::B, Band::F]);
        let report = run_scan(&channels, 1, &ScanConfig::default(), &mut io);
        if std::env::var("SCAN_TRACE").is_ok() {
            print_report(&report, &io.dwells);
        }
        let tx = report.transmitters();
        let found: Vec<u16> = tx.iter().map(|t| t.mhz).collect();
        assert_eq!(found, vec![5880, 5771], "{:#?}", &report.ranked[..6]);
        assert!(tx[0].channels.contains(&"R7".to_string()) && tx[0].channels.contains(&"F8".to_string()));
        // Neighbours within 6 MHz of a video channel are the same transmitter (B4 5790 is 19 MHz away).
        let r4 = report.ranked.iter().find(|r| r.mhz == 5769).expect("R4 scanned");
        assert_eq!(r4.alias_of, Some(5771), "{r4:?}");
        // Every frequency once, each after a drain of the old blocks.
        let mut f: Vec<u16> = channels.iter().map(|c| c.mhz).collect();
        f.sort_unstable();
        f.dedup();
        let radio = &io.radios_mut()[0];
        assert_eq!(radio.tunes.len(), f.len());
        assert_eq!(radio.drains, f.len());
        assert_eq!(io.dwells.len(), f.len());
        // The dwell is counted in stream samples: 60 ms each.
        let d = &io.dwells[0].reading;
        assert!((d.samples as f64 / fmt_rate() - 0.060).abs() < 0.002, "{d:?}");
    }

    fn fmt_rate() -> f64 {
        SigSpec::default().sample_format.sample_rate_hz()
    }

    #[test]
    fn level_is_gain_normalised() {
        let t = GainTable::c5_measured();
        let r = ChannelReading {
            power_db: 45.0,
            cn_db: f32::NAN,
            sync_score: 0.0,
            sync_quality: 0.0,
            lines: 0,
            windows: 0,
            standard: None,
            samples: 0,
        };
        let a = metrics(&r, Some(60), &t);
        let b = metrics(&r, Some(70), &t);
        assert!((a.power_db - b.power_db - 9.3).abs() < 1e-3);
        assert_eq!(a.cn_db, -20.0);
        assert_eq!(metrics(&r, None, &t).power_db, 45.0);
    }

    #[test]
    fn bands_parse() {
        assert_eq!(parse_bands(Some("rF")).unwrap(), vec![Band::R, Band::F]);
        assert_eq!(parse_bands(None).unwrap().len(), 6);
        assert!(parse_bands(Some("RX")).is_err());
    }
}
