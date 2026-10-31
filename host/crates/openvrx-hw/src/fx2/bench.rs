// SPDX-License-Identifier: GPL-3.0-or-later
//! Throughput bench (DESIGN 6.4, experiment B1): stream from one or more
//! FX2 boards at once on the internal IFCLK, with or without the firmware's
//! 16-bit test counter, and report MB/s, losses and completion jitter.

use crate::error::Result;
use crate::fx2::protocol::{Fx2Status, StartFlags, PACKET_BYTES};
use crate::fx2::stream::{Fx2Stream, IntervalStats, StreamConfig, StreamEvent, StreamPort};
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// What the firmware streams during the bench.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BenchMode {
    /// Port pins latched on the internal 30 MHz IFCLK.
    Internal30,
    /// Port pins latched on the internal 48 MHz IFCLK.
    Internal48,
    /// Firmware-generated 16-bit counter (end-to-end loss check).
    TestPattern,
}

impl BenchMode {
    pub fn flags(self, width16: bool) -> StartFlags {
        let base = StartFlags::INTERNAL_CLK.with(StartFlags::WIDTH16, width16);
        match self {
            BenchMode::Internal30 => base,
            BenchMode::Internal48 => base | StartFlags::INTERNAL_48,
            BenchMode::TestPattern => base | StartFlags::INTERNAL_48 | StartFlags::TEST_PATTERN | StartFlags::WIDTH16,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BenchConfig {
    pub mode: BenchMode,
    pub duration: Duration,
    pub transfers: usize,
    pub transfer_bytes: usize,
    pub width16: bool,
    /// Optional watch ratio to emulate lower rates (1 = full).
    pub watch_n: u16,
}

impl Default for BenchConfig {
    fn default() -> Self {
        BenchConfig {
            mode: BenchMode::Internal48,
            duration: Duration::from_secs(5),
            transfers: 16,
            transfer_bytes: 64 * 1024,
            width16: true,
            watch_n: 1,
        }
    }
}

/// 16-bit counter continuity check.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PatternCheck {
    pub words: u64,
    /// Places where the counter did not advance by one.
    pub discontinuities: u64,
    /// Words skipped at those places (modulo 65536, so a lower bound).
    pub lost_words_min: u64,
    /// Host-side drops where continuity could not be checked.
    pub host_gaps: u64,
    #[serde(skip)]
    prev: Option<u16>,
    #[serde(skip)]
    next_byte: Option<u64>,
}

impl PatternCheck {
    /// Feeds one block; `first_byte` detects host-side drops.
    pub fn feed(&mut self, first_byte: u64, data: &[u8]) {
        if let Some(nb) = self.next_byte {
            if nb != first_byte {
                self.host_gaps += 1;
                self.prev = None;
            }
        }
        // A block starting on an odd byte cannot be word aligned; skip one.
        let skip = (first_byte % 2) as usize;
        for w in data[skip.min(data.len())..].as_chunks::<2>().0 {
            let v = u16::from_le_bytes([w[0], w[1]]);
            if let Some(p) = self.prev {
                let expect = p.wrapping_add(1);
                if v != expect {
                    self.discontinuities += 1;
                    self.lost_words_min += v.wrapping_sub(expect) as u64;
                }
            }
            self.prev = Some(v);
            self.words += 1;
        }
        self.next_byte = Some(first_byte + data.len() as u64);
    }
}

/// Result for one board.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BenchResult {
    pub name: String,
    pub bytes: u64,
    pub seconds: f64,
    pub mb_per_s: f64,
    pub transfers: u64,
    /// Completion interval statistics (latency jitter), microseconds.
    pub interval: IntervalStats,
    pub interval_p99_us: f64,
    /// Expected interval at the measured rate.
    pub interval_expected_us: f64,
    pub host_dropped_blocks: u64,
    pub fifo_full_events: u64,
    pub transfer_errors: u64,
    pub restarts: u64,
    pub pattern: Option<PatternCheck>,
    /// Rate the firmware itself counted (committed packets over uptime).
    pub firmware_mb_per_s: Option<f64>,
    pub error: Option<String>,
}

impl BenchResult {
    pub fn summary(&self) -> String {
        let mut s = format!(
            "{}: {:.2} MB/s over {:.1} s, {} transfers, interval {:.0} us mean / {:.0} us std / {:.0} us p99 / {:.0} us max, \
             host drops {}, FIFO-full {}, errors {}, restarts {}",
            self.name,
            self.mb_per_s,
            self.seconds,
            self.transfers,
            self.interval.mean_us,
            self.interval.std_us(),
            self.interval_p99_us,
            self.interval.max_us,
            self.host_dropped_blocks,
            self.fifo_full_events,
            self.transfer_errors,
            self.restarts
        );
        if let Some(p) = &self.pattern {
            s.push_str(&format!(
                ", pattern {} words, {} breaks, >= {} words lost",
                p.words, p.discontinuities, p.lost_words_min
            ));
        }
        if let Some(f) = self.firmware_mb_per_s {
            s.push_str(&format!(", firmware counted {f:.2} MB/s"));
        }
        if let Some(e) = &self.error {
            s.push_str(&format!(", ERROR: {e}"));
        }
        s
    }
}

fn percentile(v: &mut [f64], p: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let k = ((v.len() - 1) as f64 * p).round() as usize;
    v[k]
}

fn firmware_rate(first: Option<Fx2Status>, last: Option<Fx2Status>) -> Option<f64> {
    let (a, b) = (first?, last?);
    let dt = b.uptime_ms.wrapping_sub(a.uptime_ms);
    let dp = b.packets_committed.wrapping_sub(a.packets_committed);
    (dt > 0 && dp > 0).then(|| dp as f64 * PACKET_BYTES as f64 / (dt as f64 * 1e-3) / 1e6)
}

/// Runs the bench on all ports concurrently (one capture thread and one
/// consumer thread each) and returns one result per port, in order.
pub fn run_bench(ports: Vec<(String, Box<dyn StreamPort>)>, cfg: &BenchConfig) -> Result<Vec<BenchResult>> {
    let flags = cfg.mode.flags(cfg.width16);
    let check_pattern = cfg.mode == BenchMode::TestPattern;
    let mut runs = Vec::new();
    for (k, (name, port)) in ports.into_iter().enumerate() {
        let scfg = StreamConfig {
            rx: k as u8,
            transfers: cfg.transfers,
            transfer_bytes: cfg.transfer_bytes,
            flags,
            watch_n: cfg.watch_n,
            transfer_event_every: 0,
            ..Default::default()
        };
        match Fx2Stream::start(port, scfg) {
            Ok(stream) => runs.push((name, Some(stream), None)),
            Err(e) => runs.push((name, None, Some(e.to_string()))),
        }
    }
    let t0 = Instant::now();
    let consumers: Vec<_> = runs
        .iter()
        .map(|(name, stream, _)| {
            stream.as_ref().map(|s| {
                let blocks = s.blocks().expect("own channel").clone();
                let duration = cfg.duration;
                std::thread::Builder::new()
                    .name(format!("openvrx-bench-{name}"))
                    .spawn(move || {
                        let mut pat = PatternCheck::default();
                        let mut intervals = Vec::new();
                        let mut prev_ns = 0u64;
                        let mut prev_seq = None;
                        while t0.elapsed() < duration {
                            let Ok(b) = blocks.recv_timeout(Duration::from_millis(100)) else {
                                continue;
                            };
                            if prev_seq == Some(b.seq.wrapping_sub(1)) && prev_ns != 0 {
                                intervals.push((b.host_ns - prev_ns) as f64 * 1e-3);
                            }
                            prev_seq = Some(b.seq);
                            prev_ns = b.host_ns;
                            if check_pattern {
                                pat.feed(b.first_byte, &b.data);
                            }
                        }
                        (pat, intervals)
                    })
            })
        })
        .collect();
    // Watch for the first status of each stream while the bench runs.
    let mut first_status: Vec<Option<Fx2Status>> = vec![None; runs.len()];
    while t0.elapsed() < cfg.duration {
        for (k, (_, stream, _)) in runs.iter().enumerate() {
            if let Some(s) = stream {
                for e in s.events().try_iter() {
                    if let StreamEvent::Status { status, .. } = e {
                        first_status[k].get_or_insert(status);
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let mut out = Vec::new();
    for (((name, stream, err), consumer), first) in runs.into_iter().zip(consumers).zip(first_status) {
        let Some(stream) = stream else {
            out.push(BenchResult {
                name,
                bytes: 0,
                seconds: 0.0,
                mb_per_s: 0.0,
                transfers: 0,
                interval: IntervalStats::default(),
                interval_p99_us: 0.0,
                interval_expected_us: 0.0,
                host_dropped_blocks: 0,
                fifo_full_events: 0,
                transfer_errors: 0,
                restarts: 0,
                pattern: None,
                firmware_mb_per_s: None,
                error: err,
            });
            continue;
        };
        let st = stream.stop();
        let (pat, mut intervals) = match consumer {
            Some(Ok(h)) => h.join().unwrap_or_default(),
            _ => Default::default(),
        };
        let seconds = if st.last_completion_ns > st.started_ns {
            (st.last_completion_ns - st.started_ns) as f64 * 1e-9
        } else {
            cfg.duration.as_secs_f64()
        };
        let mb = st.bytes as f64 / seconds.max(1e-9) / 1e6;
        out.push(BenchResult {
            name,
            bytes: st.bytes,
            seconds,
            mb_per_s: mb,
            transfers: st.transfers,
            interval_p99_us: percentile(&mut intervals, 0.99),
            interval_expected_us: if mb > 0.0 { cfg.transfer_bytes as f64 / mb } else { 0.0 },
            interval: st.interval.clone(),
            host_dropped_blocks: st.dropped_blocks,
            fifo_full_events: st.fifo_full_events,
            transfer_errors: st.transfer_errors,
            restarts: st.restarts,
            pattern: check_pattern.then_some(pat),
            firmware_mb_per_s: firmware_rate(first, st.last_status),
            error: None,
        });
    }
    Ok(out)
}

impl StreamPort for Box<dyn StreamPort> {
    fn connect(&mut self) -> Result<crate::fx2::stream::StreamLink> {
        (**self).connect()
    }
    fn describe(&self) -> String {
        (**self).describe()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fx2::endpoint::sim::SimFx2;

    #[test]
    fn pattern_checker() {
        let mut p = PatternCheck::default();
        let words: Vec<u8> = (0u16..100).flat_map(|w| w.to_le_bytes()).collect();
        p.feed(0, &words[..100]);
        p.feed(100, &words[100..]);
        assert_eq!((p.words, p.discontinuities), (100, 0));
        // A jump of 10 words.
        let more: Vec<u8> = (110u16..120).flat_map(|w| w.to_le_bytes()).collect();
        p.feed(200, &more);
        assert_eq!(p.discontinuities, 1);
        assert_eq!(p.lost_words_min, 10);
        // Host gap: continuity is not judged across it.
        let later: Vec<u8> = (500u16..510).flat_map(|w| w.to_le_bytes()).collect();
        p.feed(10_000, &later);
        assert_eq!(p.discontinuities, 1);
        assert_eq!(p.host_gaps, 1);
        // Wrap at 65536 is continuous.
        let mut q = PatternCheck::default();
        q.feed(0, &[0xFF, 0xFF, 0x00, 0x00]);
        assert_eq!(q.discontinuities, 0);
    }

    #[test]
    fn bench_two_simulated_boards() {
        let a = SimFx2::new(20e6);
        let b = SimFx2::new(10e6);
        b.with(|d| d.counter_jumps.push_back((20, 7)));
        let cfg = BenchConfig {
            mode: BenchMode::TestPattern,
            duration: Duration::from_millis(800),
            transfers: 8,
            transfer_bytes: 16 * 1024,
            ..Default::default()
        };
        let ports: Vec<(String, Box<dyn StreamPort>)> =
            vec![("a".into(), Box::new(a.port())), ("b".into(), Box::new(b.port()))];
        let r = run_bench(ports, &cfg).unwrap();
        assert_eq!(r.len(), 2);
        assert!((r[0].mb_per_s - 20.0).abs() < 4.0, "{}", r[0].summary());
        assert!((r[1].mb_per_s - 10.0).abs() < 2.0, "{}", r[1].summary());
        let pa = r[0].pattern.as_ref().unwrap();
        assert!(pa.words > 1000);
        assert_eq!(pa.discontinuities, 0, "{}", r[0].summary());
        let pb = r[1].pattern.as_ref().unwrap();
        assert_eq!(pb.discontinuities, 1);
        assert_eq!(pb.lost_words_min, 7);
        assert!(r[0].interval.count > 10);
        assert!(r[0].interval_p99_us > 0.0);
        assert!(r[0].firmware_mb_per_s.is_some());
        // START carried the internal clock and test-pattern flags.
        let started = a.with(|d| d.flags);
        assert!(started.contains(StartFlags::INTERNAL_CLK | StartFlags::TEST_PATTERN));
    }
}
