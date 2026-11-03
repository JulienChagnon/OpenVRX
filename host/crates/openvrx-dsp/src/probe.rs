// SPDX-License-Identifier: GPL-3.0-or-later
//! Sampling-phase "lottery" probe.
//!
//! When the FX2 latches the bus while the lanes are changing, some bits
//! hold the old value and some the new one; near zero crossings this gives
//! wild jumps that spread energy evenly over the band. A receive-filtered
//! capture at 40 MS/s (the calibration burst) has little energy at the top
//! of the band, so the power passed by the fourth difference
//! (1 -4 6 -4 1) over the total power separates the two:
//!
//! ```text
//! score = (P4 - 70 q) / (P - q)
//! ```
//!
//! where P4 is the mean |fourth difference|^2, P the mean power about the
//! mean, q the quantisation noise power of the lane format (white, so it
//! passes the fourth difference with gain 70). Clean reads give about 0
//! (an FM carrier 0.02-0.05, receiver noise up to a few units), mixed reads
//! 19-80; the decision threshold is [`THRESHOLD`] = 12. The fourth
//! difference probe follows C5VRX (GPL-3.0, Twotoz and the C5VRX
//! contributors, commit 6bc9e4b, `main/dsp.h` `dsp_hf4_sums` and
//! `dsp_hf4_excess_milli`); OpenVRX normalises by the total power instead of
//! reporting an absolute excess, so the score does not depend on gain, and
//! handles any lane format.

use openvrx_core::lanes::{LaneEncoding, LaneFormat};

/// Decision threshold on [`phase_probe_score`].
pub const THRESHOLD: f32 = 12.0;

/// FX2 packet size used for per-packet scoring.
pub const PACKET_BYTES: usize = 512;

/// Verdict for a calibration capture.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    Clean,
    Mixed,
    /// Too little signal or too few samples to judge.
    Unknown,
}

/// Quantisation noise power per complex sample (ADC units^2).
pub fn quantisation_power(fmt: &LaneFormat) -> f64 {
    match &fmt.encoding {
        LaneEncoding::Iq { i, q } => {
            let si = (1u64 << i.adc_lsb()) as f64;
            let sq = (1u64 << q.adc_lsb()) as f64;
            (si * si + sq * sq) / 12.0
        }
        LaneEncoding::Angle { bits, .. } => {
            let step = std::f64::consts::TAU / (1u64 << *bits) as f64 * LaneFormat::ANGLE_MAGNITUDE as f64;
            step * step / 12.0
        }
    }
}

/// Score of a capture given as SoA I/Q with known quantisation power.
pub fn score_iq(re: &[f32], im: &[f32], q: f64) -> f32 {
    let n = re.len().min(im.len());
    if n < 16 {
        return f32::NAN;
    }
    let (mut mi, mut mq) = (0.0f64, 0.0f64);
    for k in 0..n {
        mi += re[k] as f64;
        mq += im[k] as f64;
    }
    mi /= n as f64;
    mq /= n as f64;
    let mut p = 0.0f64;
    for k in 0..n {
        let (x, y) = (re[k] as f64 - mi, im[k] as f64 - mq);
        p += x * x + y * y;
    }
    p /= n as f64;
    let mut p4 = 0.0f64;
    for k in 2..n - 2 {
        let di = re[k - 2] as f64 - 4.0 * re[k - 1] as f64 + 6.0 * re[k] as f64 - 4.0 * re[k + 1] as f64
            + re[k + 2] as f64;
        let dq = im[k - 2] as f64 - 4.0 * im[k - 1] as f64 + 6.0 * im[k] as f64 - 4.0 * im[k + 1] as f64
            + im[k + 2] as f64;
        p4 += di * di + dq * dq;
    }
    p4 /= (n - 4) as f64;
    let denom = p - q;
    if denom <= q.max(1e-9) {
        return f32::NAN;
    }
    ((p4 - 70.0 * q) / denom) as f32
}

fn decode(words: &[u8], fmt: &LaneFormat) -> (Vec<f32>, Vec<f32>) {
    let wb = fmt.word_bytes as usize;
    let mut re = Vec::with_capacity(words.len() / wb);
    let mut im = Vec::with_capacity(words.len() / wb);
    for c in words.chunks_exact(wb) {
        let w = if wb == 1 { c[0] as u32 } else { c[0] as u32 | (c[1] as u32) << 8 };
        let z = fmt.decode(w);
        re.push(z.re);
        im.push(z.im);
    }
    (re, im)
}

/// Probe score of raw words (bytes as received, little-endian words).
/// NaN when the capture is too short or carries no signal above the
/// quantisation noise.
pub fn phase_probe_score(words: &[u8], fmt: &LaneFormat) -> f32 {
    let (re, im) = decode(words, fmt);
    score_iq(&re, &im, quantisation_power(fmt))
}

/// Scores each FX2 packet of a capture separately.
pub fn packet_scores(bytes: &[u8], fmt: &LaneFormat) -> Vec<f32> {
    bytes
        .as_chunks::<PACKET_BYTES>().0.iter()
        .map(|p| phase_probe_score(p, fmt))
        .collect()
}

/// Verdict over a capture: median of the packet scores against the
/// threshold (robust to a packet straddling a transient).
pub fn verdict(bytes: &[u8], fmt: &LaneFormat) -> (Verdict, f32) {
    let mut s: Vec<f32> = packet_scores(bytes, fmt).into_iter().filter(|v| v.is_finite()).collect();
    if s.is_empty() {
        let whole = phase_probe_score(bytes, fmt);
        return if whole.is_finite() {
            (classify(whole), whole)
        } else {
            (Verdict::Unknown, f32::NAN)
        };
    }
    s.sort_by(|a, b| a.total_cmp(b));
    let m = s[s.len() / 2];
    (classify(m), m)
}

/// Threshold decision for one score.
pub fn classify(score: f32) -> Verdict {
    if !score.is_finite() {
        Verdict::Unknown
    } else if score > THRESHOLD {
        Verdict::Mixed
    } else {
        Verdict::Clean
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::lanes::LanePreset;

    #[test]
    fn white_noise_scores_about_seventy() {
        let mut s = 7u64;
        let mut u = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f64 / (1u64 << 31) as f64) - 0.5
        };
        let re: Vec<f32> = (0..20000).map(|_| (u() * 300.0) as f32).collect();
        let im: Vec<f32> = (0..20000).map(|_| (u() * 300.0) as f32).collect();
        let sc = score_iq(&re, &im, 0.0);
        assert!((sc - 70.0).abs() < 3.0, "{sc}");
        // A slow tone scores near zero.
        let re: Vec<f32> = (0..4000).map(|k| (100.0 * (k as f64 * 0.1).cos()) as f32).collect();
        let im: Vec<f32> = (0..4000).map(|k| (100.0 * (k as f64 * 0.1).sin()) as f32).collect();
        assert!(score_iq(&re, &im, 0.0).abs() < 0.01);
        let q = quantisation_power(&LanePreset::I8Q8.format());
        assert!((q - 32.0 / 12.0).abs() < 1e-12);
    }
}
