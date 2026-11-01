// SPDX-License-Identifier: GPL-3.0-or-later
//! Fourth-difference phase probe (DESIGN 5.4).
//!
//! At 40 MS/s the C5's receive filter leaves the I/Q signal smooth, so the
//! fourth difference `x[n] - 4x[n+1] + 6x[n+2] - 4x[n+3] + x[n+4]` passes
//! almost nothing of it. When the sample clock edge lands while the lanes
//! change, some bits hold the old sample and some the new one; near zero
//! crossings every upper bit flips and the read jumps by hundreds of ADC
//! units for one sample, which the fourth difference passes at 70 times its
//! power. The score is
//!
//! ```text
//! score = (P(d4) - 70 q) / (P(x) - q)
//! ```
//!
//! summed over I and Q, where `P(x)` is the mean-removed power, `P(d4)` the
//! mean power of the fourth difference and `q` the quantisation noise power
//! of the lane format (each term less its quantisation share). Clean reads
//! score near 0 for a carrier and a few units for noise; mixed reads score
//! tens. The threshold is 12. Scores are computed per 512-byte packet only,
//! because consecutive packets are not contiguous when the FX2 FIFO
//! overflows at 40 MS/s.

use openvrx_core::lanes::LaneEncoding;
use openvrx_core::LaneFormat;
use serde::{Deserialize, Serialize};

/// Mixed-read threshold.
pub const PROBE_THRESHOLD: f32 = 12.0;
/// Noise power gain of the 1 -4 6 -4 1 filter.
pub const D4_GAIN: f32 = 70.0;

/// Quantisation noise power per component, in ADC units squared.
pub fn quant_power(lanes: &LaneFormat) -> f32 {
    match &lanes.encoding {
        LaneEncoding::Iq { i, .. } => {
            let step = (1u32 << i.adc_lsb()) as f32;
            step * step / 12.0
        }
        LaneEncoding::Angle { bits, .. } => {
            let dth = std::f32::consts::TAU / (1u32 << *bits) as f32;
            let a = LaneFormat::ANGLE_MAGNITUDE;
            // Angle error dth^2/12 spread over two components.
            a * a * dth * dth / 12.0 / 2.0
        }
    }
}

/// Reads little-endian words of the format's width.
pub fn words(data: &[u8], word_bytes: u8) -> impl Iterator<Item = u32> + '_ {
    data.chunks_exact(word_bytes as usize).map(move |c| match c.len() {
        1 => c[0] as u32,
        _ => u16::from_le_bytes([c[0], c[1]]) as u32,
    })
}

/// Probe score of one packet (any length, normally 512 bytes), or None
/// when the packet is too short or carries no signal above quantisation.
pub fn probe_packet(data: &[u8], lanes: &LaneFormat) -> Option<f32> {
    let z: Vec<(f32, f32)> = words(data, lanes.word_bytes)
        .map(|w| {
            let c = lanes.decode(w);
            (c.re, c.im)
        })
        .collect();
    probe_samples(&z, quant_power(lanes))
}

/// Probe score of decoded samples with quantisation power `q` per
/// component.
pub fn probe_samples(z: &[(f32, f32)], q: f32) -> Option<f32> {
    let n = z.len();
    if n < 16 {
        return None;
    }
    let mut sum = 0.0f32;
    let mut sum_d4 = 0.0f32;
    for comp in 0..2 {
        let x: Vec<f32> = z.iter().map(|p| if comp == 0 { p.0 } else { p.1 }).collect();
        let mean = x.iter().sum::<f32>() / n as f32;
        let p = x.iter().map(|v| (v - mean) * (v - mean)).sum::<f32>() / n as f32;
        let d4 = x
            .windows(5)
            .map(|w| {
                let d = w[0] - 4.0 * w[1] + 6.0 * w[2] - 4.0 * w[3] + w[4];
                d * d
            })
            .sum::<f32>()
            / (n - 4) as f32;
        sum += p;
        sum_d4 += d4;
    }
    let signal = sum - 2.0 * q;
    // Need the signal clearly above the quantisation floor to judge.
    if signal <= 2.0 * q {
        return None;
    }
    Some((sum_d4 - D4_GAIN * 2.0 * q) / signal)
}

/// How a set of packet scores is judged.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProbeConfig {
    pub threshold: f32,
    /// Fewer valid packets than this: undecided.
    pub min_valid: usize,
    /// At most this fraction of mixed packets is still clean (interference
    /// and the odd overflow seam).
    pub clean_max_fraction: f32,
    /// At least this fraction of mixed packets is mixed.
    pub mixed_min_fraction: f32,
}

impl Default for ProbeConfig {
    fn default() -> Self {
        ProbeConfig {
            threshold: PROBE_THRESHOLD,
            min_valid: 16,
            clean_max_fraction: 0.1,
            mixed_min_fraction: 0.3,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Verdict {
    Clean,
    Mixed,
    /// Between the clean and mixed fractions: suspicious, retry.
    Marginal,
    /// Not enough packets with signal.
    Undecided,
}

/// Robust statistics over many packet scores.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ProbeStats {
    pub packets: usize,
    pub valid: usize,
    pub median: f32,
    pub p90: f32,
    pub mixed_fraction: f32,
    pub verdict: Verdict,
}

fn quantile(sorted: &[f32], p: f32) -> f32 {
    if sorted.is_empty() {
        return f32::NAN;
    }
    sorted[((sorted.len() - 1) as f32 * p).round() as usize]
}

/// Judges a set of packet scores with the median and the fraction above
/// the threshold (insensitive to a few outliers either way).
pub fn judge(scores: &[Option<f32>], cfg: &ProbeConfig) -> ProbeStats {
    let mut v: Vec<f32> = scores.iter().flatten().copied().filter(|s| s.is_finite()).collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let valid = v.len();
    let mixed = v.iter().filter(|&&s| s > cfg.threshold).count();
    let frac = if valid == 0 { 0.0 } else { mixed as f32 / valid as f32 };
    let median = quantile(&v, 0.5);
    let verdict = if valid < cfg.min_valid {
        Verdict::Undecided
    } else if median > cfg.threshold || frac >= cfg.mixed_min_fraction {
        Verdict::Mixed
    } else if frac <= cfg.clean_max_fraction {
        Verdict::Clean
    } else {
        Verdict::Marginal
    };
    ProbeStats {
        packets: scores.len(),
        valid,
        median,
        p90: quantile(&v, 0.9),
        mixed_fraction: frac,
        verdict,
    }
}

/// Scores every whole packet of `packet_bytes` in a buffer.
pub fn probe_packets(data: &[u8], lanes: &LaneFormat, packet_bytes: usize) -> Vec<Option<f32>> {
    data.chunks_exact(packet_bytes)
        .map(|p| probe_packet(p, lanes))
        .collect()
}

#[cfg(test)]
pub(crate) mod testsig {
    //! Band-limited test signals at 40 MS/s and mixed-read corruption.

    /// Small deterministic PRNG (xorshift).
    pub struct Rng(pub u64);
    impl Rng {
        pub fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        pub fn uniform(&mut self) -> f32 {
            (self.next() >> 40) as f32 / (1u64 << 24) as f32
        }
        pub fn gauss(&mut self) -> f32 {
            let u1 = self.uniform().max(1e-7);
            let u2 = self.uniform();
            (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos()
        }
    }

    /// FM-like signal: a carrier whose frequency wanders slowly (video),
    /// amplitude `amp`, plus low-passed noise, as 10-bit ADC values.
    pub fn signal(n: usize, amp: f32, noise: f32, rng: &mut Rng) -> Vec<(i32, i32)> {
        let fs = 40e6f32;
        let mut ph = 0.0f32;
        let mut out = Vec::with_capacity(n);
        let (mut ni, mut nq) = (0.0f32, 0.0f32);
        for k in 0..n {
            let f = 1.5e6 * (std::f32::consts::TAU * k as f32 * 15_734.0 / fs).sin();
            ph += std::f32::consts::TAU * f / fs;
            // One-pole low-pass noise (bandwidth well below fs/2).
            ni = 0.8 * ni + 0.2 * noise * rng.gauss();
            nq = 0.8 * nq + 0.2 * noise * rng.gauss();
            out.push(((amp * ph.cos() + ni) as i32 + 20, (amp * ph.sin() + nq) as i32));
        }
        out
    }

    /// Mixed reads: the bits in `late_mask` still hold the previous
    /// sample's value.
    pub fn mix(words: &[u32], late_mask: u32) -> Vec<u32> {
        let mut out = Vec::with_capacity(words.len());
        let mut prev = words[0];
        for &w in words {
            out.push((w & !late_mask) | (prev & late_mask));
            prev = w;
        }
        out
    }

    pub fn to_bytes(words: &[u32], word_bytes: u8) -> Vec<u8> {
        words
            .iter()
            .flat_map(|&w| {
                let b = (w as u16).to_le_bytes();
                b[..word_bytes as usize].to_vec()
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::testsig::*;
    use super::*;
    use openvrx_core::LanePreset;

    fn scores(preset: LanePreset, late_mask: u32, noise: f32, seed: u64) -> ProbeStats {
        let f = preset.format();
        let mut rng = Rng(seed);
        let sig = signal(256 * 64, 150.0, noise, &mut rng);
        let w: Vec<u32> = sig.iter().map(|&(i, q)| f.encode(i, q)).collect();
        let w = if late_mask != 0 { mix(&w, late_mask) } else { w };
        let bytes = to_bytes(&w, f.word_bytes);
        judge(&probe_packets(&bytes, &f, 512), &ProbeConfig::default())
    }

    #[test]
    fn clean_reads_score_low() {
        for p in [LanePreset::I8Q8, LanePreset::I7Q7, LanePreset::I6Q6] {
            let s = scores(p, 0, 10.0, 1);
            assert_eq!(s.verdict, Verdict::Clean, "{p}: {s:?}");
            assert!(s.median < 3.0, "{p}: {s:?}");
        }
    }

    #[test]
    fn mixed_reads_score_high() {
        // MSB of I and two bits of Q latch late.
        let s = scores(LanePreset::I7Q7, 0x8000 | 0x0060, 10.0, 2);
        assert_eq!(s.verdict, Verdict::Mixed, "{s:?}");
        assert!(s.median > PROBE_THRESHOLD, "{s:?}");
    }

    #[test]
    fn white_noise_is_not_clean_and_silence_is_undecided() {
        let f = LanePreset::I8Q8.format();
        let mut rng = Rng(9);
        let z: Vec<(f32, f32)> = (0..256).map(|_| (100.0 * rng.gauss(), 100.0 * rng.gauss())).collect();
        let s = probe_samples(&z, quant_power(&f)).unwrap();
        assert!((s - 70.0).abs() < 15.0, "white noise scores ~70, got {s}");
        let zero = vec![0u8; 512];
        assert_eq!(probe_packet(&zero, &f), None);
        let st = judge(&[None; 20], &ProbeConfig::default());
        assert_eq!(st.verdict, Verdict::Undecided);
    }

    #[test]
    fn judge_is_robust() {
        let cfg = ProbeConfig::default();
        let mut v: Vec<Option<f32>> = vec![Some(0.5); 95];
        v.extend(vec![Some(60.0); 5]);
        assert_eq!(judge(&v, &cfg).verdict, Verdict::Clean);
        let mut v: Vec<Option<f32>> = vec![Some(0.5); 80];
        v.extend(vec![Some(60.0); 20]);
        assert_eq!(judge(&v, &cfg).verdict, Verdict::Marginal);
        let mut v: Vec<Option<f32>> = vec![Some(0.5); 60];
        v.extend(vec![Some(60.0); 40]);
        assert_eq!(judge(&v, &cfg).verdict, Verdict::Mixed);
    }

    #[test]
    fn quantisation_power() {
        assert_eq!(quant_power(&LanePreset::I7Q7.format()), 64.0 / 12.0);
        assert_eq!(quant_power(&LanePreset::I8Q8.format()), 16.0 / 12.0);
        assert!(quant_power(&LanePreset::Angle8.format()) > 0.0);
    }
}
