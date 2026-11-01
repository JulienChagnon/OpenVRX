// SPDX-License-Identifier: GPL-3.0-or-later
//! Lane self-test analysis (PROTOCOL.md 1, "Self-test patterns").
//!
//! With `selftest walk` the C5 drives its lane pins as plain GPIOs: phase 0
//! all low, phase 1 all high, then phase 2+j only lane j high, each phase
//! `phase_us` long, repeating. Lanes are numbered in the order of the FX2
//! word bit they should land on, ascending. The analyser finds the phase
//! grid from the bit transitions, reads the majority word in the middle of
//! every phase, aligns the sequence with the expected pattern and names
//! stuck, swapped, shorted or missing lanes in plain language.

use crate::c5::proto::PinMap;
use crate::calib::probe::words;
use openvrx_core::lanes::LaneEncoding;
use openvrx_core::LaneFormat;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Which component a lane carries.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Component {
    I,
    Q,
    /// Phase angle (angle presets).
    A,
}

/// One lane of a lane format.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LaneInfo {
    /// Lane number in self-test order.
    pub index: usize,
    pub component: Component,
    /// ADC bit carried (angle bit for angle presets).
    pub adc_bit: u8,
    /// FX2 word bit it should land on.
    pub word_bit: u8,
    /// C5 GPIO driving it, when the pin map is known.
    pub gpio: Option<u8>,
}

/// FX2 pin name of a word bit: FD0..7 are PB0..7, FD8..15 are PD0..7.
pub fn fx2_pin(word_bit: u8) -> String {
    if word_bit < 8 {
        format!("PB{word_bit}")
    } else {
        format!("PD{}", word_bit - 8)
    }
}

impl LaneInfo {
    /// "lane I bit 7 (ADC bit 9, C5 GPIO 23)": the bit number is the
    /// position within the FX2 port byte the lane should land on.
    pub fn name(&self) -> String {
        let c = match self.component {
            Component::I => "I",
            Component::Q => "Q",
            Component::A => "angle",
        };
        let mut s = format!("lane {c} bit {} (ADC bit {}", self.word_bit % 8, self.adc_bit);
        if self.component == Component::A {
            s = format!("lane {c} bit {} (angle bit {}", self.word_bit % 8, self.adc_bit);
        }
        if let Some(g) = self.gpio {
            s.push_str(&format!(", C5 GPIO {g}"));
        }
        s.push(')');
        s
    }
}

/// Lanes of a format in self-test order (word bit ascending).
pub fn lanes_of(f: &LaneFormat, pins: Option<&PinMap>) -> Vec<LaneInfo> {
    let mut v = Vec::new();
    match &f.encoding {
        LaneEncoding::Iq { i, q } => {
            for (comp, c, gp) in [
                (Component::I, i, pins.map(|p| &p.i)),
                (Component::Q, q, pins.map(|p| &p.q)),
            ] {
                for (k, &b) in c.adc_bits.iter().enumerate() {
                    v.push(LaneInfo {
                        index: 0,
                        component: comp,
                        adc_bit: b,
                        word_bit: c.word_msb - k as u8,
                        gpio: gp.and_then(|g| g.get(k).copied()),
                    });
                }
            }
        }
        LaneEncoding::Angle { word_msb, bits } => {
            for k in 0..*bits {
                v.push(LaneInfo {
                    index: 0,
                    component: Component::A,
                    adc_bit: bits - 1 - k,
                    word_bit: word_msb - k,
                    gpio: pins.and_then(|p| p.a.get(k as usize).copied()),
                });
            }
        }
    }
    v.sort_by_key(|l| l.word_bit);
    for (k, l) in v.iter_mut().enumerate() {
        l.index = k;
    }
    v
}

/// Word for `selftest const HEX`: bit j of `hex` drives lane j.
pub fn const_word(f: &LaneFormat, hex: u32) -> u32 {
    lanes_of(f, None)
        .iter()
        .filter(|l| hex >> l.index & 1 == 1)
        .fold(0, |w, l| w | 1 << l.word_bit)
}

/// Fraction of words equal to `expected` on the format's used bits.
pub fn match_fraction(data: &[u8], f: &LaneFormat, expected: u32) -> f32 {
    let mask = f.used_word_mask();
    let (mut hit, mut n) = (0usize, 0usize);
    for w in words(data, f.word_bytes) {
        n += 1;
        hit += usize::from(w & mask == expected & mask);
    }
    if n == 0 {
        0.0
    } else {
        hit as f32 / n as f32
    }
}

/// Kinds of finding.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueKind {
    StuckHigh,
    Missing,
    WrongPin,
    Swapped,
    Shorted,
    UnusedPinActive,
    NoActivity,
    TooShort,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Issue {
    pub kind: IssueKind,
    pub message: String,
}

/// Result of a walk analysis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WalkReport {
    pub ok: bool,
    pub lanes: Vec<LaneInfo>,
    /// Phases read from the capture.
    pub phases: usize,
    /// Majority word per pattern phase (index = phase in the pattern).
    pub observed: Vec<u32>,
    pub issues: Vec<Issue>,
}

impl WalkReport {
    pub fn summary(&self) -> String {
        if self.ok {
            format!("all {} lanes OK", self.lanes.len())
        } else {
            self.issues
                .iter()
                .map(|i| i.message.clone())
                .collect::<Vec<_>>()
                .join("\n")
        }
    }
}

fn bits(mut m: u32) -> Vec<u8> {
    let mut v = Vec::new();
    while m != 0 {
        let b = m.trailing_zeros() as u8;
        v.push(b);
        m &= m - 1;
    }
    v
}

fn pins_text(m: u32) -> String {
    let v: Vec<String> = bits(m).into_iter().map(fx2_pin).collect();
    match v.len() {
        0 => "no pin".into(),
        1 => v[0].clone(),
        _ => {
            let (last, rest) = v.split_last().expect("len > 1");
            format!("{} and {last}", rest.join(", "))
        }
    }
}

/// Analyses a contiguous `walk` capture. `phase_len` is the phase length
/// in samples (`phase_us` times the sample rate).
pub fn analyze_walk(data: &[u8], f: &LaneFormat, phase_len: f64, pins: Option<&PinMap>) -> WalkReport {
    let lanes = lanes_of(f, pins);
    let nl = lanes.len();
    let period = nl + 2;
    let w: Vec<u32> = words(data, f.word_bytes).collect();
    let mut report = WalkReport {
        ok: false,
        lanes: lanes.clone(),
        phases: 0,
        observed: Vec::new(),
        issues: Vec::new(),
    };
    let issue = |r: &mut WalkReport, kind, message: String| r.issues.push(Issue { kind, message });
    if phase_len < 4.0 {
        issue(
            &mut report,
            IssueKind::TooShort,
            "phase length is under 4 samples".into(),
        );
        return report;
    }
    // Phase grid from transitions (circular mean of positions mod P).
    let (mut sx, mut sy, mut nt) = (0.0f64, 0.0f64, 0usize);
    for k in 1..w.len() {
        if w[k] != w[k - 1] {
            let a = std::f64::consts::TAU * (k as f64 / phase_len).fract();
            sx += a.cos();
            sy += a.sin();
            nt += 1;
        }
    }
    if nt == 0 {
        issue(
            &mut report,
            IssueKind::NoActivity,
            "no FX2 pin ever changes: the self-test is not running, the sample clock does not reach IFCLK, or every lane is disconnected".into(),
        );
        return report;
    }
    let offset = (sy.atan2(sx).rem_euclid(std::f64::consts::TAU)) / std::f64::consts::TAU * phase_len;
    // Majority word in the middle half of every whole phase.
    let mut seg = Vec::new();
    let mut k = 0usize;
    loop {
        let start = offset + k as f64 * phase_len;
        let a = (start + 0.25 * phase_len) as usize;
        let b = (start + 0.75 * phase_len) as usize;
        if b > w.len() {
            break;
        }
        let mut counts: HashMap<u32, usize> = HashMap::new();
        for &x in &w[a..b] {
            *counts.entry(x).or_default() += 1;
        }
        let best = counts
            .into_iter()
            .max_by_key(|&(v, c)| (c, std::cmp::Reverse(v)))
            .map(|(v, _)| v);
        seg.push(best.unwrap_or(0));
        k += 1;
    }
    report.phases = seg.len();
    if seg.len() < period + 1 {
        issue(
            &mut report,
            IssueKind::TooShort,
            format!(
                "capture holds {} phases; the walk needs at least {} (capture longer)",
                seg.len(),
                period + 1
            ),
        );
        return report;
    }
    let all: u32 = lanes.iter().fold(0, |m, l| m | 1 << l.word_bit);
    let expected: Vec<u32> = (0..period)
        .map(|p| match p {
            0 => 0,
            1 => all,
            j => 1 << lanes[j - 2].word_bit,
        })
        .collect();
    // Align the observed sequence with the pattern.
    let shift = (0..period)
        .min_by_key(|&s| {
            seg.iter()
                .enumerate()
                .map(|(k, &v)| (v ^ expected[(k + s) % period]).count_ones())
                .sum::<u32>()
        })
        .expect("period > 0");
    let mut per_phase: Vec<HashMap<u32, usize>> = vec![HashMap::new(); period];
    for (k, &v) in seg.iter().enumerate() {
        *per_phase[(k + shift) % period].entry(v).or_default() += 1;
    }
    let obs: Vec<u32> = per_phase
        .iter()
        .map(|m| {
            m.iter()
                .max_by_key(|&(v, c)| (*c, std::cmp::Reverse(*v)))
                .map_or(0, |(v, _)| *v)
        })
        .collect();
    report.observed = obs.clone();

    let word_mask: u32 = if f.word_bytes == 2 { 0xFFFF } else { 0xFF };
    let stuck_high = obs[0] & word_mask;
    for b in bits(stuck_high) {
        let who = lanes.iter().find(|l| l.word_bit == b);
        let msg = match who {
            Some(l) => format!(
                "FX2 {} reads high while every lane is low: {} or its pin is stuck high (short to 3.3 V, or a floating input)",
                fx2_pin(b),
                l.name()
            ),
            None => format!(
                "FX2 {} reads high while every lane is low; it is unused and should be tied to GND",
                fx2_pin(b)
            ),
        };
        issue(&mut report, IssueKind::StuckHigh, msg);
    }
    let responds: Vec<u32> = (0..nl).map(|j| obs[2 + j] & !stuck_high & word_mask).collect();
    let mut reported_swap = vec![false; nl];
    let mut explained = 0u32;
    for (j, l) in lanes.iter().enumerate() {
        let r = responds[j];
        explained |= r;
        let e = 1u32 << l.word_bit;
        if r == e {
            continue;
        }
        if r == 0 {
            if stuck_high & e != 0 {
                continue; // already reported as stuck high
            }
            issue(
                &mut report,
                IssueKind::Missing,
                format!(
                    "{} does not reach the FX2: no pin goes high when it is driven (open wire, wrong C5 pin, or {} stuck low)",
                    l.name(),
                    fx2_pin(l.word_bit)
                ),
            );
            continue;
        }
        if r.count_ones() == 1 {
            let b = r.trailing_zeros() as u8;
            // Swap: the lane expected on `b` reads on our expected pin.
            if let Some(k) = lanes.iter().position(|o| o.word_bit == b) {
                if responds[k] == e && !reported_swap[j] {
                    reported_swap[j] = true;
                    reported_swap[k] = true;
                    issue(
                        &mut report,
                        IssueKind::Swapped,
                        format!(
                            "{} and {} are swapped: they read on FX2 {} and {}",
                            l.name(),
                            lanes[k].name(),
                            fx2_pin(b),
                            fx2_pin(l.word_bit)
                        ),
                    );
                    continue;
                }
                if reported_swap[j] {
                    continue;
                }
            }
            issue(
                &mut report,
                IssueKind::WrongPin,
                format!(
                    "{} reads on FX2 {} instead of {}",
                    l.name(),
                    fx2_pin(b),
                    fx2_pin(l.word_bit)
                ),
            );
            continue;
        }
        // Several pins respond.
        let others = r & !e;
        let shorted: Vec<&LaneInfo> = lanes.iter().filter(|o| others >> o.word_bit & 1 == 1).collect();
        if r & e != 0 && !shorted.is_empty() {
            // Report each shorted pair once (from the lower lane).
            for o in shorted {
                if o.index > j || responds[o.index] & e == 0 {
                    issue(
                        &mut report,
                        IssueKind::Shorted,
                        format!(
                            "{} is shorted to {}: driving it also raises FX2 {}",
                            l.name(),
                            o.name(),
                            fx2_pin(o.word_bit)
                        ),
                    );
                }
            }
            let stray = others & !all;
            if stray != 0 {
                issue(
                    &mut report,
                    IssueKind::Shorted,
                    format!(
                        "{} also appears on FX2 {}, which should be tied to GND",
                        l.name(),
                        pins_text(stray)
                    ),
                );
            }
        } else if r & e != 0 {
            issue(
                &mut report,
                IssueKind::Shorted,
                format!(
                    "{} also appears on FX2 {}, which should be tied to GND",
                    l.name(),
                    pins_text(others)
                ),
            );
        } else {
            issue(
                &mut report,
                IssueKind::WrongPin,
                format!(
                    "{} reads on FX2 {} instead of {}",
                    l.name(),
                    pins_text(r),
                    fx2_pin(l.word_bit)
                ),
            );
        }
    }
    // Unused pins that go high with all lanes but with no single lane.
    let unused_active = obs[1] & !all & !stuck_high & !explained & word_mask;
    if unused_active != 0 {
        issue(
            &mut report,
            IssueKind::UnusedPinActive,
            format!(
                "FX2 {} goes high when all lanes are high but follows no single lane; it should be tied to GND",
                pins_text(unused_active)
            ),
        );
    }
    report.ok = report.issues.is_empty();
    report
}

#[cfg(test)]
pub(crate) mod testgen {
    use super::*;

    /// Wiring fault model: maps each lane's driven level to FX2 bits.
    pub struct Wiring {
        /// For lane j, the word bits it drives (normally its own bit).
        pub drives: Vec<u32>,
        pub stuck_high: u32,
        pub stuck_low: u32,
    }

    impl Wiring {
        pub fn ideal(f: &LaneFormat) -> Wiring {
            Wiring {
                drives: lanes_of(f, None).iter().map(|l| 1 << l.word_bit).collect(),
                stuck_high: 0,
                stuck_low: 0,
            }
        }
    }

    /// Generates a walk capture: `phase_len` samples per phase, starting
    /// `start_offset` samples into phase `start_phase`, with a few glitches.
    pub fn walk_capture(
        f: &LaneFormat,
        wiring: &Wiring,
        phase_len: usize,
        n: usize,
        start_phase: usize,
        start_offset: usize,
    ) -> Vec<u8> {
        let nl = wiring.drives.len();
        let period = nl + 2;
        let mut out = Vec::new();
        for k in 0..n {
            let t = k + start_offset + start_phase * phase_len;
            let p = (t / phase_len) % period;
            let mut w = 0u32;
            for j in 0..nl {
                let high = p == 1 || p == 2 + j;
                if high {
                    w |= wiring.drives[j];
                }
            }
            w = (w | wiring.stuck_high) & !wiring.stuck_low;
            if k % 997 == 13 {
                w ^= 0x0101; // isolated glitch
            }
            let b = (w as u16).to_le_bytes();
            out.extend_from_slice(&b[..f.word_bytes as usize]);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::testgen::*;
    use super::*;
    use openvrx_core::LanePreset;

    fn run(preset: LanePreset, wiring: Wiring) -> WalkReport {
        let f = preset.format();
        let pins = PinMap {
            clk: Some(27),
            i: vec![23, 24, 11, 12, 8, 9, 10, 2],
            q: vec![1, 0, 25, 7, 3, 4, 5, 6],
            a: vec![],
        };
        let n = lanes_of(&f, None).len();
        let data = walk_capture(&f, &wiring, 200, 200 * (n + 2) * 3, 3, 77);
        analyze_walk(&data, &f, 200.0, Some(&pins))
    }

    #[test]
    fn lane_order_follows_protocol() {
        let l = lanes_of(&LanePreset::I7Q7.format(), None);
        // Q bit 9..3 on word bits 1..7, then I on 9..15.
        assert_eq!(l[0].word_bit, 1);
        assert_eq!((l[0].component, l[0].adc_bit), (Component::Q, 3));
        assert_eq!((l[6].component, l[6].adc_bit, l[6].word_bit), (Component::Q, 9, 7));
        assert_eq!((l[7].component, l[7].word_bit), (Component::I, 9));
        assert_eq!(l.len(), 14);
        assert_eq!(fx2_pin(15), "PD7");
        assert_eq!(fx2_pin(3), "PB3");
        let f = LanePreset::I4Q4Top.format();
        assert_eq!(const_word(&f, 0b1000_0001), 0b1000_0001);
        assert_eq!(const_word(&LanePreset::I7Q7.format(), 1), 1 << 1);
    }

    #[test]
    fn ideal_wiring_passes() {
        for p in [
            LanePreset::I8Q8,
            LanePreset::I7Q7,
            LanePreset::I4Q4Top,
            LanePreset::Angle8,
        ] {
            let r = run(p, Wiring::ideal(&p.format()));
            assert!(r.ok, "{p}: {}", r.summary());
        }
    }

    #[test]
    fn names_wrong_pin_with_gpio() {
        let f = LanePreset::I8Q8.format();
        let mut w = Wiring::ideal(&f);
        // I MSB (lane 15, word bit 15) lands on PD5 (bit 13) only; lane on
        // bit 13 still drives bit 13 -> wrong pin plus a shared pin.
        w.drives[15] = 1 << 13;
        let r = run(LanePreset::I8Q8, w);
        assert!(!r.ok);
        let s = r.summary();
        assert!(
            s.contains("lane I bit 7 (ADC bit 9, C5 GPIO 23) reads on FX2 PD5 instead of PD7"),
            "{s}"
        );
    }

    #[test]
    fn swap_short_missing_stuck() {
        let f = LanePreset::I7Q7.format();
        let mut w = Wiring::ideal(&f);
        w.drives.swap(0, 1); // Q bits on PB1 and PB2 swapped
        w.drives[5] = 0; // Q lane on PB6 missing
        w.drives[9] |= 1 << 12; // I lane on PD3 shorted to PD4
        w.drives[10] |= 1 << 11;
        w.stuck_high = 1 << 0; // unused PB0 floating high
        let r = run(LanePreset::I7Q7, w);
        let kinds: Vec<IssueKind> = r.issues.iter().map(|i| i.kind).collect();
        let s = r.summary();
        assert!(kinds.contains(&IssueKind::Swapped), "{s}");
        assert!(s.contains("are swapped: they read on FX2 PB2 and PB1"), "{s}");
        assert!(kinds.contains(&IssueKind::Missing), "{s}");
        assert!(s.contains("lane Q bit 6"), "{s}");
        assert!(kinds.contains(&IssueKind::Shorted), "{s}");
        assert!(s.contains("is shorted to lane I bit 4"), "{s}");
        assert_eq!(kinds.iter().filter(|k| **k == IssueKind::Shorted).count(), 1, "{s}");
        assert!(
            s.contains("FX2 PB0 reads high while every lane is low; it is unused"),
            "{s}"
        );
    }

    #[test]
    fn stuck_lane_and_dead_capture() {
        let f = LanePreset::I8Q8.format();
        let mut w = Wiring::ideal(&f);
        w.stuck_low = 1 << 8;
        w.stuck_high = 1 << 9;
        let r = run(LanePreset::I8Q8, w);
        let s = r.summary();
        assert!(s.contains("lane I bit 0") && s.contains("PD0 stuck low"), "{s}");
        assert!(
            s.contains("FX2 PD1 reads high while every lane is low: lane I bit 1"),
            "{s}"
        );
        let dead = vec![0u8; 20000];
        let r = analyze_walk(&dead, &f, 200.0, None);
        assert_eq!(r.issues[0].kind, IssueKind::NoActivity);
        let short = walk_capture(&f, &Wiring::ideal(&f), 200, 600, 0, 0);
        let r = analyze_walk(&short, &f, 200.0, None);
        assert_eq!(r.issues[0].kind, IssueKind::TooShort);
    }

    #[test]
    fn const_pattern_match() {
        let f = LanePreset::I7Q7.format();
        let word = const_word(&f, 0x2A5A);
        let data: Vec<u8> = (0..100).flat_map(|_| (word as u16).to_le_bytes()).collect();
        assert_eq!(match_fraction(&data, &f, word), 1.0);
        assert_eq!(match_fraction(&data, &f, word ^ 0x0200), 0.0);
    }
}
