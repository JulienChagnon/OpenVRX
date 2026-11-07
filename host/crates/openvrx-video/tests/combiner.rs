// SPDX-License-Identifier: GPL-3.0-or-later
//! Diversity combining: D1/D2/D3/segment against single receivers with
//! independent noise and dropouts, field alignment with independent field
//! counters, never stalling on a dead receiver, sub-sample alignment.

mod common;

use common::*;
use openvrx_core::quality::damage;
use openvrx_core::{LineFlags, LineRecord, VideoStandard, COMBINED};
use openvrx_video::combiner::{CombineMode, Combiner, CombinerConfig};
use openvrx_video::encoder::{CompositeEncoder, EncoderConfig};
use openvrx_video::testgen::{add_clicks, add_dropout, add_noise, Scene};
use openvrx_video::util::Rng;

const STD: VideoStandard = VideoStandard::Pal;

/// Clean composite lines of a few fields of a test scene.
fn clean_lines(fields: u64, offset: f64, rx: u8) -> Vec<LineRecord> {
    let p = STD.params();
    let enc = CompositeEncoder::new(
        STD,
        EncoderConfig {
            time_offset: offset,
            rx,
            ..EncoderConfig::default()
        },
    );
    let scene = Scene::new(21, (0.0, 0.0), (2.0, 0.0));
    let mut out = Vec::new();
    for seq in 0..fields {
        let pic = scene.render(p.active_width(), p.frame_height(), seq as f64);
        out.extend(encode(&enc, &pic, seq));
    }
    out
}

/// Active-picture MSE of combined lines against the clean lines.
fn active_mse(out: &[LineRecord], clean: &[LineRecord]) -> f64 {
    let a = STD.params().active_range();
    let mut acc = 0.0;
    let mut n = 0.0;
    for o in out {
        let c = clean
            .iter()
            .find(|c| c.field_seq == o.field_seq && c.field_line == o.field_line)
            .expect("matching clean line");
        if o.quality.flags.contains(LineFlags::VBI) {
            continue;
        }
        for x in a.clone() {
            let d = (o.samples[x] - c.samples[x]) as f64;
            acc += d * d;
            n += 1.0;
        }
    }
    acc / n
}

fn run(mode: CombineMode, rxs: &[Vec<LineRecord>]) -> Vec<LineRecord> {
    let mut c = Combiner::new(
        STD,
        &CombinerConfig {
            mode,
            ..CombinerConfig::default()
        },
    );
    let mut out = Vec::new();
    let n = rxs[0].len();
    for i in 0..n {
        for r in rxs {
            if i < r.len() {
                out.extend(c.push(r[i].clone()));
            }
        }
    }
    out.extend(c.flush());
    out
}

fn db(mse_ref: f64, mse: f64) -> f64 {
    10.0 * (mse_ref / mse).log10()
}

#[test]
fn modes_versus_single_receiver() {
    let clean = clean_lines(4, 0.0, 0);
    let mut rng = Rng::new(1);
    let noisy = |sigma: f32, rx: u8, rng: &mut Rng, dropouts: bool| -> Vec<LineRecord> {
        clean
            .iter()
            .map(|l| {
                let mut l = l.clone();
                l.rx = rx;
                add_noise(&mut l, sigma, rng);
                if dropouts && rng.uniform() < 0.08 {
                    let a = STD.params().active_range();
                    let s = a.start + rng.below((a.len() - 200) as u64) as usize;
                    let len = 20 + rng.below(160) as usize;
                    add_dropout(&mut l, s..s + len, rng);
                }
                if dropouts {
                    let k = rng.below(3) as usize;
                    add_clicks(&mut l, k, rng);
                }
                l
            })
            .collect()
    };

    // Equal independent noise: blending gains ~3 dB.
    let a = noisy(4.0, 0, &mut rng, false);
    let b = noisy(4.0, 1, &mut rng, false);
    let single = active_mse(&a, &clean);
    println!("\nequal noise 4 IRE (gain over one receiver, dB):");
    for m in [CombineMode::Select, CombineMode::Blend, CombineMode::Repair, CombineMode::SelectRepair, CombineMode::Segment, CombineMode::SegmentSelect] {
        let out = run(m, &[a.clone(), b.clone()]);
        let g = db(single, active_mse(&out, &clean));
        println!("  {m:?}: {g:.2}");
        match m {
            CombineMode::Blend | CombineMode::Repair => assert!(g > 2.7, "{m:?} {g}"),
            CombineMode::Segment => assert!(g > 2.3, "{m:?} {g}"),
            _ => assert!(g > -0.2, "{m:?} {g}"),
        }
    }

    // Unequal noise: weighted blend beats selecting the better receiver.
    let a = noisy(2.0, 0, &mut rng, false);
    let b = noisy(6.0, 1, &mut rng, false);
    let best = active_mse(&a, &clean);
    let sel = active_mse(&run(CombineMode::Select, &[a.clone(), b.clone()]), &clean);
    let bl = active_mse(&run(CombineMode::Blend, &[a.clone(), b.clone()]), &clean);
    println!("unequal noise 2/6 IRE: select {:.2} dB, blend {:.2} dB over the better rx", db(best, sel), db(best, bl));
    assert!(db(best, sel).abs() < 0.05);
    assert!(db(best, bl) > 0.3, "blend {}", db(best, bl));

    // Independent dropouts and clicks: per-sample repair wins clearly.
    let a = noisy(3.0, 0, &mut rng, true);
    let b = noisy(3.0, 1, &mut rng, true);
    let single = active_mse(&a, &clean);
    println!("noise 3 IRE + independent dropouts/clicks (gain over one receiver, dB):");
    let mut gains = std::collections::HashMap::new();
    for m in [CombineMode::Select, CombineMode::Blend, CombineMode::Repair, CombineMode::SelectRepair, CombineMode::Segment, CombineMode::SegmentSelect] {
        let out = run(m, &[a.clone(), b.clone()]);
        let g = db(single, active_mse(&out, &clean));
        println!("  {m:?}: {g:.2}");
        gains.insert(m, g);
        if m == CombineMode::Repair {
            let left: usize = out.iter().map(|l| l.damage.iter().filter(|&&d| d >= damage::CLICK).count()).sum();
            let before: usize = a.iter().map(|l| l.damage.iter().filter(|&&d| d >= damage::CLICK).count()).sum();
            assert!(left * 20 < before, "repair left {left} of {before} damaged samples");
            assert!(out.iter().any(|l| l.rx == COMBINED && l.quality.flags.contains(LineFlags::BLENDED)));
            assert!(out.iter().any(|l| l.quality.flags.contains(LineFlags::REPAIRED)));
        }
    }
    assert!(gains[&CombineMode::Repair] > gains[&CombineMode::Blend] + 4.5);
    assert!(gains[&CombineMode::Repair] > 7.5);
    assert!(gains[&CombineMode::SelectRepair] > gains[&CombineMode::Select] + 5.0);
    assert!(gains[&CombineMode::Segment] > gains[&CombineMode::Blend] + 4.5);
    assert!(gains[&CombineMode::SegmentSelect] > gains[&CombineMode::Select] + 4.5);
}

#[test]
fn segment_scoring_handles_partial_line_noise() {
    // Receiver A has a burst of unflagged noise over part of each line
    // (a fade the damage mask did not catch); B is uniformly mediocre.
    let clean = clean_lines(2, 0.0, 0);
    let mut rng = Rng::new(9);
    let a_range = STD.params().active_range();
    let seg = a_range.start + a_range.len() / 4..a_range.start + a_range.len() / 2;
    let a: Vec<LineRecord> = clean
        .iter()
        .map(|l| {
            let mut l = l.clone();
            add_noise(&mut l, 1.5, &mut rng);
            for x in seg.clone() {
                l.samples[x] += 12.0 * rng.gauss() as f32;
            }
            l
        })
        .collect();
    let b: Vec<LineRecord> = clean
        .iter()
        .map(|l| {
            let mut l = l.clone();
            l.rx = 1;
            add_noise(&mut l, 4.0, &mut rng);
            l
        })
        .collect();
    let sel = active_mse(&run(CombineMode::Select, &[a.clone(), b.clone()]), &clean);
    let blend = active_mse(&run(CombineMode::Repair, &[a.clone(), b.clone()]), &clean);
    let segm = active_mse(&run(CombineMode::Segment, &[a.clone(), b.clone()]), &clean);
    let segsel = active_mse(&run(CombineMode::SegmentSelect, &[a, b]), &clean);
    println!(
        "partial-line noise: segment blend {:.2} dB over line weights, segment select {:.2} dB over line select",
        db(blend, segm),
        db(sel, segsel)
    );
    assert!(db(blend, segm) > 3.0);
    assert!(db(sel, segsel) > 2.0);
}

#[test]
fn independent_field_counters_dead_receiver_and_waiting() {
    let clean = clean_lines(6, 0.0, 0);
    let mut a = clean.clone();
    let mut b: Vec<LineRecord> = clean
        .iter()
        .map(|l| {
            let mut l = l.clone();
            l.rx = 1;
            l.field_seq += 1000;
            l.host_ns += 300_000; // arrives 0.3 ms later
            l
        })
        .collect();
    let mut rng = Rng::new(3);
    for l in a.iter_mut().chain(b.iter_mut()) {
        add_noise(l, 2.0, &mut rng);
    }
    let cfg = CombinerConfig {
        max_wait_lines: 6,
        ..CombinerConfig::default()
    };
    let mut c = Combiner::new(STD, &cfg);
    let lines_per_field = 313;
    let die_at = 3 * lines_per_field;
    let mut out = Vec::new();
    let mut max_backlog = 0usize;
    for i in 0..a.len() {
        out.extend(c.push(a[i].clone()));
        // B lags a few lines behind A, then dies.
        if i >= 3 && i - 3 < die_at {
            out.extend(c.push(b[i - 3].clone()));
        }
        max_backlog = max_backlog.max(i + 1 - out.len());
    }
    out.extend(c.flush());
    assert_eq!(out.len(), a.len(), "every line emitted exactly once");
    // Common counter follows the first receiver; raster order kept.
    for w in out.windows(2) {
        assert!((w[0].field_seq, w[0].field_line) < (w[1].field_seq, w[1].field_line));
    }
    assert_eq!(out[0].field_seq, 0);
    // While both were alive lines were blended; afterwards A alone.
    assert!(out[..die_at - 10].iter().filter(|l| l.rx == COMBINED).count() > die_at - 40);
    assert!(out[die_at + 20..].iter().all(|l| l.rx == 0));
    // Never more than the wait budget plus the lag behind.
    assert!(max_backlog <= cfg.max_wait_lines + 1, "backlog {max_backlog}");
    assert_eq!(c.stats().relocks, 0);
}

#[test]
fn lone_receiver_survives_bunched_host_times() {
    // DSP-5: a USB stall makes the stream thread reap a backlog of
    // transfers at once, so every line of the stall carries nearly the same
    // host time. The relock used to map the lone receiver onto a field
    // already emitted and drop its lines as late until the stall's end.
    let clean = clean_lines(24, 0.0, 0);
    let t0 = clean[0].host_ns;
    let (a, b) = (t0 + 200_000_000, t0 + 300_000_000);
    let mut c = Combiner::new(STD, &CombinerConfig::default());
    let mut out = Vec::new();
    for l in &clean {
        let mut l = l.clone();
        if l.host_ns >= a && l.host_ns < b {
            l.host_ns = b;
        }
        out.extend(c.push(l));
    }
    out.extend(c.flush());
    assert!(c.stats().relocks > 0, "the stall should disturb the mapping");
    assert_eq!(c.stats().late_lines, 0);
    assert_eq!(out.len(), clean.len());
    for w in out.windows(2) {
        assert!((w[0].field_seq, w[0].field_line) < (w[1].field_seq, w[1].field_line));
    }
}

#[test]
fn sub_sample_alignment() {
    let clean = clean_lines(2, 0.0, 0);
    let shifted = clean_lines(2, 0.4, 1);
    let mut rng = Rng::new(5);
    let mut a = clean.clone();
    let mut b = shifted;
    for l in a.iter_mut().chain(b.iter_mut()) {
        add_noise(l, 1.0, &mut rng);
    }
    let mut res = Vec::new();
    for align in [false, true] {
        let mut c = Combiner::new(
            STD,
            &CombinerConfig {
                mode: CombineMode::Blend,
                align,
                ..CombinerConfig::default()
            },
        );
        let mut out = Vec::new();
        for i in 0..a.len() {
            out.extend(c.push(a[i].clone()));
            out.extend(c.push(b[i].clone()));
        }
        out.extend(c.flush());
        let m = active_mse(&out, &clean);
        if align {
            let tau = c.timing_offset(1).unwrap();
            assert!((tau - 0.4).abs() < 0.03, "estimated offset {tau}");
        }
        res.push(m);
    }
    println!("0.4-sample offset: alignment improves the blend by {:.2} dB", db(res[0], res[1]));
    assert!(db(res[0], res[1]) > 3.0);
}
