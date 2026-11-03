// SPDX-License-Identifier: GPL-3.0-or-later
//! End-to-end checks of generated recordings.

use openvrx_core::{Complex32, LanePreset, VideoStandard};
use openvrx_format::{EventKind, Recording, ReplayOptions};
use openvrx_sim::*;
use std::path::Path;

fn base_cfg(seconds: f64) -> SimConfig {
    let mut c = SimConfig {
        seconds,
        seed: 11,
        picture: PictureConfig::Bars,
        ..Default::default()
    };
    c.vtx.nonlinearity = 0.0;
    c.channel.cn_db = 30.0;
    for r in &mut c.receivers {
        r.sampling.lottery = Lottery::Clean;
        r.crystal_ppm = Some(0.0);
    }
    c
}

fn gen(cfg: &SimConfig, dir: &Path, name: &str) -> Recording {
    let base = dir.join(name);
    generate(cfg, &base).unwrap();
    Recording::open(&base).unwrap()
}

fn iq(rec: &Recording, rx: u8) -> Vec<Complex32> {
    let mut r = rec.raw_reader(rx).unwrap();
    let n = r.len_samples() as usize;
    r.read_iq(n).unwrap()
}

fn remove_dc(z: &mut [Complex32]) {
    let n = z.len() as f32;
    let m = z.iter().fold(Complex32::new(0.0, 0.0), |a, b| a + b) / n;
    z.iter_mut().for_each(|x| *x -= m);
}

/// FM discriminator output in Hz, sample n relates to n - 0.5.
fn discriminate(z: &[Complex32], fs: f64) -> Vec<f64> {
    let mut out = vec![0.0; z.len()];
    for n in 1..z.len() {
        let d = z[n] * z[n - 1].conj();
        out[n] = (d.im as f64).atan2(d.re as f64) * fs / std::f64::consts::TAU;
    }
    out[0] = out[1];
    out
}

/// Centred moving average of odd length.
fn smooth(x: &[f64], len: usize) -> Vec<f64> {
    let h = len / 2;
    (0..x.len())
        .map(|i| {
            let a = i.saturating_sub(h);
            let b = (i + h + 1).min(x.len());
            x[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect()
}

fn cn_envelope_db(z: &[Complex32]) -> f64 {
    let p: Vec<f64> = z.iter().map(|v| v.norm_sqr() as f64).collect();
    let n = p.len() as f64;
    let mean = p.iter().sum::<f64>() / n;
    let var = p.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
    let r = mean * mean / var;
    let c = (r - 1.0) + ((r - 1.0) * r).max(0.0).sqrt();
    10.0 * c.log10()
}

#[test]
fn recording_plays_through_the_reader() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.1);
    cfg.set_receiver_count(2);
    cfg.receivers[1].crystal_ppm = Some(12.0);
    cfg.events.push(ScheduledEvent { t_s: 0.05, rx: None, action: Action::Marker { text: "half".into() } });
    let rec = gen(&cfg, dir.path(), "play");
    let s = rec.sidecar();
    assert!(s.closed_cleanly);
    assert_eq!(s.source, openvrx_format::RecordingSource::Sim);
    assert_eq!(s.standard_hint, Some(VideoStandard::Ntsc));
    assert_eq!(s.receivers.len(), 2);
    let rate = s.receivers[0].sample_rate_hz;
    let n0 = rec.rx_samples(0).unwrap();
    assert!((n0 as f64 - 0.1 * rate).abs() < 2.0, "{n0}");
    // rx1 runs 12 ppm fast against the same 0.1 s of its own time.
    assert!((rec.rx_samples(1).unwrap() as f64 - 0.1 * rate).abs() < 2.0);
    let sim = s.sim.as_ref().unwrap();
    assert!(sim.settings.contains_key("config"));
    assert!(sim.settings.contains_key("realised"));
    let back: SimConfig = serde_json::from_value(sim.settings["config"].clone()).unwrap();
    assert_eq!(back, cfg);
    assert_eq!(rec.events().counts().get("marker"), Some(&1));
    // Replay delivers every byte.
    let total: usize = rec
        .replay(ReplayOptions::unpaced())
        .unwrap()
        .map(|b| b.unwrap().data.len())
        .sum();
    assert_eq!(total as u64, rec.rx_bytes(0).unwrap() + rec.rx_bytes(1).unwrap());
    // Receivers have independent noise.
    let a = iq(&rec, 0);
    let b = iq(&rec, 1);
    let same = a.iter().zip(&b).take(10000).filter(|(x, y)| x == y).count();
    assert!(same < 2000, "{same}");
}

#[test]
fn deterministic_for_a_seed() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.04);
    cfg.picture = PictureConfig::Scene;
    cfg.channel.cn_db = 8.0;
    cfg.channel.fading.kind = FadingKind::Rayleigh;
    cfg.receivers[0].sampling.lottery = Lottery::default();
    cfg.receivers[0].crystal_ppm = None;
    let a = gen(&cfg, dir.path(), "a");
    let b = gen(&cfg, dir.path(), "b");
    let ra = std::fs::read(a.rx_path(0).unwrap()).unwrap();
    let rb = std::fs::read(b.rx_path(0).unwrap()).unwrap();
    assert_eq!(ra, rb);
    assert_eq!(a.truth().unwrap().read_all().unwrap(), b.truth().unwrap().read_all().unwrap());
    cfg.seed += 1;
    let c = gen(&cfg, dir.path(), "c");
    let rc = std::fs::read(c.rx_path(0).unwrap()).unwrap();
    assert_ne!(ra[..ra.len().min(rc.len())], rc[..ra.len().min(rc.len())]);
}

#[test]
fn truth_length_and_alignment() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.05);
    cfg.picture = PictureConfig::Ramp;
    cfg.cvbs.colour = false;
    cfg.receivers[0].preset = LanePreset::I8Q8;
    cfg.receivers[0].divider = 12;
    let rec = gen(&cfg, dir.path(), "truth");
    let mut t = rec.truth().unwrap();
    let truth = t.read_all().unwrap();
    assert_eq!(truth.len() as u64, rec.rx_samples(0).unwrap());
    let delay = t.info().delay_samples;
    assert!(delay > 0.5 && delay < 5.0, "{delay}");
    let fs = rec.receiver(0).unwrap().sample_rate_hz;
    let mut z = iq(&rec, 0);
    remove_dc(&mut z);
    let f = smooth(&discriminate(&z, fs), 3);
    // Video estimate in IRE from the deviation law.
    let est: Vec<f64> = f.iter().map(|x| (x + 1.0e6) / 6.7e6 * 140.0).collect();
    let corr = |lag: i64| {
        let (mut sxy, mut sxx, mut syy, mut sx, mut sy, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for i in 2000..est.len() - 2000 {
            let x = est[i];
            let y = truth[(i as i64 - lag) as usize] as f64;
            sxy += x * y;
            sxx += x * x;
            syy += y * y;
            sx += x;
            sy += y;
            n += 1.0;
        }
        let cov = sxy / n - sx / n * sy / n;
        cov / ((sxx / n - (sx / n).powi(2)).sqrt() * (syy / n - (sy / n).powi(2)).sqrt())
    };
    let (best, c) = (-10..=10).map(|l| (l, corr(l))).max_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
    assert!(c > 0.99, "correlation {c}");
    assert!((best as f64 - (delay + 0.5)).abs() <= 1.0, "best lag {best}, delay {delay}");
}

#[test]
fn deviation_and_spectrum() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.03);
    cfg.channel.cn_db = 45.0;
    cfg.picture = PictureConfig::Flat { y: 1.0, u: 0.0, v: 0.0 };
    cfg.receivers[0].preset = LanePreset::I8Q8;
    cfg.receivers[0].divider = 6;
    cfg.receivers[0].lo_offset_hz = Some(0.0);
    let rec = gen(&cfg, dir.path(), "dev");
    let fs = rec.receiver(0).unwrap().sample_rate_hz;
    let truth = rec.truth().unwrap().read_all().unwrap();
    let mut z = iq(&rec, 0);
    remove_dc(&mut z);
    let f = discriminate(&z, fs);
    let delay = rec.truth_info().unwrap().delay_samples.round() as usize;
    let level = |lo: f32, hi: f32| {
        let v: Vec<f64> = (delay + 10..f.len() - 10)
            .filter(|&i| {
                // Flat stretch: the truth stays in range around i.
                (i - delay - 8..i - delay + 8).all(|k| truth[k] >= lo && truth[k] <= hi)
            })
            .map(|i| f[i])
            .collect();
        assert!(v.len() > 1000, "{lo}..{hi}: {}", v.len());
        v.iter().sum::<f64>() / v.len() as f64
    };
    let sync = level(-40.5, -39.5);
    let blank = level(-0.3, 0.3);
    let white = level(99.5, 100.5);
    assert!((blank + 1.0e6).abs() < 30e3, "blank {blank}");
    assert!((sync - (-1.0e6 - 6.7e6 * 40.0 / 140.0)).abs() < 40e3, "sync {sync}");
    assert!((white - (-1.0e6 + 6.7e6 * 100.0 / 140.0)).abs() < 40e3, "white {white}");
    assert!((white - sync - 6.7e6).abs() < 60e3);

    // Power spectrum: the carrier occupies the deviation range, with 99 %
    // of its power inside -5.5 .. +5.5 MHz and none piled up out of band.
    let nfft = 4096;
    let mut psd = vec![0.0f64; nfft];
    let mut segs = 0;
    for s in (0..z.len() - nfft).step_by(nfft) {
        let mut buf: Vec<(f64, f64)> = z[s..s + nfft]
            .iter()
            .enumerate()
            .map(|(k, v)| {
                let w = 0.5 - 0.5 * (std::f64::consts::TAU * k as f64 / nfft as f64).cos();
                (v.re as f64 * w, v.im as f64 * w)
            })
            .collect();
        fft(&mut buf);
        for (p, v) in psd.iter_mut().zip(&buf) {
            *p += v.0 * v.0 + v.1 * v.1;
        }
        segs += 1;
    }
    assert!(segs > 5);
    let total: f64 = psd.iter().sum();
    let freq = |k: usize| {
        let k = k as f64;
        if k < nfft as f64 / 2.0 {
            k * fs / nfft as f64
        } else {
            (k - nfft as f64) * fs / nfft as f64
        }
    };
    let inside: f64 = (0..nfft).filter(|&k| freq(k).abs() < 5.5e6).map(|k| psd[k]).sum();
    assert!(inside / total > 0.99, "{}", inside / total);
    // Power is spread from the sync tip to white, not concentrated.
    let low: f64 = (0..nfft).filter(|&k| freq(k) < -2.0e6).map(|k| psd[k]).sum();
    let high: f64 = (0..nfft).filter(|&k| freq(k) > 2.0e6).map(|k| psd[k]).sum();
    assert!(low / total > 0.03 && high / total > 0.3, "{} {}", low / total, high / total);
}

fn fft(x: &mut [(f64, f64)]) {
    let n = x.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            x.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -std::f64::consts::TAU / len as f64;
        for s in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                let (ar, ai) = x[s + k];
                let (br, bi) = x[s + k + len / 2];
                let (tr, ti) = (br * wr - bi * wi, br * wi + bi * wr);
                x[s + k] = (ar + tr, ai + ti);
                x[s + k + len / 2] = (ar - tr, ai - ti);
            }
        }
        len <<= 1;
    }
}

#[test]
fn cn_calibration() {
    let dir = tempfile::tempdir().unwrap();
    for cn in [0.0, 5.0, 10.0, 15.0, 20.0] {
        let mut cfg = base_cfg(0.03);
        cfg.channel.cn_db = cn;
        cfg.picture = PictureConfig::Scene;
        cfg.receivers[0].preset = LanePreset::I8Q8;
        // A fixed gain that keeps every level inside the ADC range.
        cfg.receivers[0].agc = Agc::Fixed { gain: 64 };
        let rec = gen(&cfg, dir.path(), &format!("cn{cn}"));
        let mut z = iq(&rec, 0);
        remove_dc(&mut z);
        let m = cn_envelope_db(&z[1000..]);
        assert!((m - cn).abs() < 0.5, "configured {cn} dB, measured {m:.2} dB");
    }
}

#[test]
fn lanes_for_every_preset_share_the_adc_stream() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.01);
    cfg.channel.cn_db = 15.0;
    cfg.receivers[0].preset = LanePreset::I8Q8;
    let r8 = gen(&cfg, dir.path(), "reference");
    let ref8 = iq(&r8, 0);
    let gain = r8.receiver(0).unwrap().gain;
    let realised = &r8.sidecar().sim.as_ref().unwrap().settings["realised"][0];
    assert_eq!(realised["initial_gain"].as_u64().unwrap() as u32, gain);
    // The DC the C5 removes before computing angles (10-bit units).
    let rc = &cfg.receivers[0];
    let t = &rc.dc_i_table;
    let k = t.windows(2).position(|w| gain as f64 <= w[1].0).unwrap_or(t.len() - 2);
    let dc8 = t[k].1 + (t[k + 1].1 - t[k].1) * (gain as f64 - t[k].0) / (t[k + 1].0 - t[k].0);
    let dc = ((4.0 * rc.dc_scale * dc8) as f32).round() as i32;
    let dcq = (rc.dc_q as f32).round() as i32;
    for preset in LanePreset::ALL {
        cfg.receivers[0].preset = preset;
        let rec = gen(&cfg, dir.path(), &format!("p{preset}"));
        let z = iq(&rec, 0);
        assert_eq!(z.len(), ref8.len(), "{preset}");
        let f = preset.format();
        let bad = z
            .iter()
            .zip(&ref8)
            .filter(|(a, b)| {
                // The 8x8 value is the ADC value + 2 (half step of 4).
                let adc = (b.re as i32 - 2, b.im as i32 - 2);
                let expect = if f.has_magnitude() {
                    f.decode(f.encode(adc.0, adc.1))
                } else {
                    f.decode(f.encode(adc.0 - dc, adc.1 - dcq))
                };
                **a != expect
            })
            .count();
        assert_eq!(bad, 0, "{preset}");
    }
}

/// Fourth-difference phase probe of the design notes (section 5.4):
/// power through 1 -4 6 -4 1 over total power, both per sample, scaled.
fn probe(z: &[Complex32]) -> f64 {
    let n = z.len();
    let mut p4 = 0.0;
    let mut p = 0.0;
    for i in 2..n - 2 {
        let d = z[i - 2] - z[i - 1] * 4.0 + z[i] * 6.0 - z[i + 1] * 4.0 + z[i + 2];
        p4 += d.norm_sqr() as f64;
        p += z[i].norm_sqr() as f64;
    }
    100.0 * p4 / 70.0 / p
}

#[test]
fn sampling_phase_glitch_only_when_enabled() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.004);
    cfg.receivers[0].preset = LanePreset::I8Q8;
    cfg.receivers[0].divider = 6;
    let clean = gen(&cfg, dir.path(), "clean");
    cfg.receivers[0].sampling.lottery = Lottery::Mixed;
    let mixed = gen(&cfg, dir.path(), "mixed");
    let mut zc = iq(&clean, 0);
    let mut zm = iq(&mixed, 0);
    remove_dc(&mut zc);
    remove_dc(&mut zm);
    let (pc, pm) = (probe(&zc), probe(&zm));
    assert!(pc < 12.0, "clean probe {pc}");
    assert!(pm > 12.0 && pm > 4.0 * pc, "mixed probe {pm} vs clean {pc}");
    // A slip of one tick leaves the bad position.
    cfg.seconds = 0.008;
    cfg.events.push(ScheduledEvent { t_s: 0.004, rx: Some(0), action: Action::Slip { count: 1 } });
    let slipped = gen(&cfg, dir.path(), "slipped");
    let mut z = iq(&slipped, 0);
    let half = z.len() / 2;
    let tail = &mut z[half + 100..];
    remove_dc(tail);
    assert!(probe(tail) < 12.0, "after slip {}", probe(tail));
    assert_eq!(slipped.events().counts().get("slip"), Some(&1));
}

#[test]
fn fading_envelope_is_rayleigh() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.4);
    cfg.picture = PictureConfig::Flat { y: 0.5, u: 0.0, v: 0.0 };
    cfg.channel.cn_db = 32.0;
    cfg.channel.fading = Fading { kind: FadingKind::Rayleigh, doppler_hz: 300.0, k_db: 0.0 };
    cfg.receivers[0].preset = LanePreset::I8Q8;
    cfg.receivers[0].divider = 18;
    cfg.receivers[0].agc = Agc::Fixed { gain: 58 };
    let rec = gen(&cfg, dir.path(), "fade");
    let mut z = iq(&rec, 0);
    remove_dc(&mut z);
    // Short-window RMS envelope (window much shorter than the 0.6 ms
    // coherence time at 300 Hz).
    let w = 200;
    let env: Vec<f64> = z
        .chunks(w)
        .map(|c| (c.iter().map(|v| v.norm_sqr() as f64).sum::<f64>() / c.len() as f64).sqrt())
        .collect();
    let ms = env.iter().map(|a| a * a).sum::<f64>() / env.len() as f64;
    let n = env.len() as f64;
    for x in [0.3f64, 0.6, 1.0, 1.5] {
        let emp = env.iter().filter(|&&a| a * a < x * x * ms).count() as f64 / n;
        let th = 1.0 - (-x * x).exp();
        assert!((emp - th).abs() < 0.06, "x={x}: {emp:.3} vs {th:.3}");
    }
}

/// Line period in input samples from the sync edges of the demodulated
/// stream (normal lines only).
fn line_period(rec: &Recording) -> f64 {
    let fs = rec.receiver(0).unwrap().sample_rate_hz;
    let mut z = iq(rec, 0);
    remove_dc(&mut z);
    let f = smooth(&discriminate(&z, fs), 5);
    let thr = -1.0e6 - 6.7e6 * 20.0 / 140.0;
    let mut edges = Vec::new();
    for i in 1..f.len() {
        if f[i - 1] >= thr && f[i] < thr {
            let frac = (f[i - 1] - thr) / (f[i - 1] - f[i]);
            edges.push(i as f64 - 1.0 + frac);
        }
    }
    let nominal = fs / openvrx_core::standard::NTSC.line_hz;
    // Count lines between edges; keep edges on the whole-line grid.
    let mut pts = Vec::new();
    let e0 = edges[0];
    for &e in &edges {
        let k = ((e - e0) / nominal).round();
        if ((e - e0) - k * nominal).abs() < 0.02 * nominal {
            pts.push((k, e));
        }
    }
    let n = pts.len() as f64;
    let (sk, se) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (mk, me) = (sk / n, se / n);
    let num: f64 = pts.iter().map(|p| (p.0 - mk) * (p.1 - me)).sum();
    let den: f64 = pts.iter().map(|p| (p.0 - mk).powi(2)).sum();
    num / den
}

#[test]
fn clock_ppm_changes_the_line_period() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.06);
    cfg.receivers[0].preset = LanePreset::I8Q8;
    let r0 = gen(&cfg, dir.path(), "ppm0");
    cfg.receivers[0].crystal_ppm = Some(100.0);
    let r1 = gen(&cfg, dir.path(), "ppm100");
    let fs = r0.receiver(0).unwrap().sample_rate_hz;
    let nominal = fs / openvrx_core::standard::NTSC.line_hz;
    let (l0, l1) = (line_period(&r0), line_period(&r1));
    assert!((l0 / nominal - 1.0).abs() < 3e-6, "{l0} vs {nominal}");
    // A receiver crystal 100 ppm fast samples 100 ppm more per line.
    let ppm = (l1 / l0 - 1.0) * 1e6;
    assert!((ppm - 100.0).abs() < 3.0, "{ppm} ppm");
}

#[test]
fn gain_events_and_dc_jump() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.04);
    cfg.channel.cn_db = 20.0;
    cfg.receivers[0].preset = LanePreset::I8Q8;
    cfg.receivers[0].agc = Agc::Fixed { gain: 64 };
    cfg.events.push(ScheduledEvent { t_s: 0.02, rx: Some(0), action: Action::Gain { index: 68 } });
    cfg.events.push(ScheduledEvent { t_s: 0.03, rx: Some(0), action: Action::Tune { mhz: 5732, offset_khz: 500 } });
    let rec = gen(&cfg, dir.path(), "gain");
    let ev: Vec<_> = rec.events().events.iter().filter(|e| matches!(e.kind, EventKind::Gain { .. })).collect();
    assert_eq!(ev.len(), 1);
    let effects = rec.sidecar().sim.as_ref().unwrap().settings["effects"].as_array().unwrap().clone();
    let g = effects.iter().find(|e| e["kind"] == "gain").unwrap();
    let eff = g["effect_sample"].as_u64().unwrap() as usize;
    let fs = rec.receiver(0).unwrap().sample_rate_hz;
    assert!((eff as f64 - 0.02 * fs).abs() < 2.0, "{eff}");
    // Command logged 1 ms before the effect.
    let cmd = ev[0].byte.unwrap();
    assert_eq!(cmd, g["command_byte"].as_u64().unwrap());
    assert!(((eff as u64 * 2 - cmd) as f64 / (2.0 * fs) - 0.001).abs() < 1e-5);
    // The I DC offset jumps from about 80 to about 128 (10-bit units).
    let z = iq(&rec, 0);
    let mean = |s: &[Complex32]| s.iter().map(|v| v.re as f64).sum::<f64>() / s.len() as f64;
    let before = mean(&z[eff - 60_000..eff - 1000]);
    let after = mean(&z[eff + 1000..eff + 60_000]);
    assert!((before - 82.0).abs() < 8.0, "{before}");
    assert!((after - before - 48.0).abs() < 10.0, "{before} -> {after}");
    assert_eq!(rec.events().counts().get("tune"), Some(&1));
}

#[test]
fn usb_loss_keeps_truth_aligned() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.05);
    cfg.receivers[0].usb_loss = UsbLoss { rate_per_s: 200.0, max_packets: 4 };
    let rec = gen(&cfg, dir.path(), "loss");
    let gaps: Vec<_> = rec
        .events()
        .events
        .iter()
        .filter_map(|e| match e.kind {
            EventKind::UsbGap { lost_bytes, .. } => Some((e.byte.unwrap(), lost_bytes)),
            _ => None,
        })
        .collect();
    assert!(gaps.len() > 3, "{}", gaps.len());
    let lost: u64 = gaps.iter().map(|g| g.1).sum();
    let fs = rec.receiver(0).unwrap().sample_rate_hz;
    let expected = (0.05 * fs) as u64 * 2;
    assert!((rec.rx_bytes(0).unwrap() + lost).abs_diff(expected) < 8);
    assert_eq!(rec.truth().unwrap().len(), rec.rx_samples(0).unwrap());
    // The timeline accounts for the gaps.
    assert!((rec.rx_duration_s(0).unwrap() - 0.05).abs() < 1e-5);
}

#[test]
fn sim_source_streams_the_same_data() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.03);
    cfg.set_receiver_count(2);
    let rec = gen(&cfg, dir.path(), "stream");
    let file0 = std::fs::read(rec.rx_path(0).unwrap()).unwrap();
    let src = SimSource::start(&cfg, SimSourceOptions { speed: None, looping: false, ..Default::default() }).unwrap();
    assert_eq!(src.sidecar().receivers.len(), 2);
    let mut got0 = Vec::new();
    let mut got1 = 0;
    let mut last_seq = None;
    for b in src {
        let b = b.unwrap();
        if b.rx == 0 {
            assert_eq!(b.first_byte, got0.len() as u64);
            if let Some(s) = last_seq {
                assert_eq!(b.seq, s + 1);
            }
            last_seq = Some(b.seq);
            got0.extend_from_slice(&b.data);
        } else {
            got1 += b.data.len();
        }
    }
    assert_eq!(got0, file0);
    assert_eq!(got1 as u64, rec.rx_bytes(1).unwrap());

    // Paced: 10 ms of a looping stream takes about 10 ms.
    let mut src = SimSource::start(&cfg, SimSourceOptions::default()).unwrap();
    let start = std::time::Instant::now();
    let mut bytes = 0usize;
    let rate = rec.receiver(0).unwrap().sample_rate_hz * 2.0;
    while (bytes as f64) < rate * 0.06 {
        let b = src.next().unwrap().unwrap();
        if b.rx == 0 {
            bytes += b.data.len();
        }
    }
    let el = start.elapsed().as_secs_f64();
    assert!(el > 0.05, "paced: {el}");
}

#[test]
fn pal_angle_lanes_and_video_source() {
    let dir = tempfile::tempdir().unwrap();
    let mut cfg = base_cfg(0.04);
    cfg.standard = VideoStandard::Pal;
    cfg.osd = true;
    cfg.receivers[0].preset = LanePreset::Angle8;
    cfg.receivers[0].filter = ChannelFilter::P11;
    cfg.vtx.pre_emphasis = Some(PreEmphasis::default());
    let rec = gen(&cfg, dir.path(), "pal");
    assert_eq!(rec.receiver(0).unwrap().sample_format.lanes.word_bytes, 1);
    assert!(rec.receiver(0).unwrap().p11);
    let truth = rec.truth().unwrap().read_all().unwrap();
    let min = truth.iter().cloned().fold(f32::MAX, f32::min);
    assert!((min - (-300.0 / 7.0)).abs() < 2.0, "PAL sync {min}");

    // Video through ffmpeg, when available.
    let clip = dir.path().join("clip.mp4");
    let ok = std::process::Command::new("ffmpeg")
        .args(["-v", "error", "-y", "-f", "lavfi", "-i", "testsrc=size=320x240:rate=25:duration=1"])
        .args(["-pix_fmt", "yuv420p"])
        .arg(&clip)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    if ok {
        let mut v = base_cfg(0.05);
        v.picture = PictureConfig::Video { path: clip.to_string_lossy().into_owned() };
        let rec = gen(&v, dir.path(), "video");
        let t = rec.truth().unwrap().read_all().unwrap();
        let bright = t.iter().filter(|&&x| x > 60.0).count();
        assert!(bright > 1000, "{bright}");
    } else {
        eprintln!("ffmpeg not available: video source not exercised");
    }
}

#[test]
fn presets_generate() {
    let dir = tempfile::tempdir().unwrap();
    for name in ["diversity", "multipath", "flyout"] {
        let mut cfg = SimConfig::preset(name).unwrap();
        cfg.seconds = 0.02;
        let rec = gen(&cfg, dir.path(), name);
        assert_eq!(rec.receivers().len(), cfg.receivers.len());
        assert!(rec.rx_bytes(0).unwrap() > 0);
    }
}
