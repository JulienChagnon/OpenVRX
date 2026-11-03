// SPDX-License-Identifier: GPL-3.0-or-later
//! FM demodulators.
//!
//! SINAD_TABLE_PLACEHOLDER
//!
//! All demodulators output the instantaneous frequency in radians per
//! sample in the NCO-shifted domain; the chain converts to Hz and adds the
//! shift back. Integer extra delays (beyond the half sample every
//! phase-difference estimator has) are reported by [`Demod::extra_delay`].

use crate::config::{DemodKind, EkfConfig, LsConfig, PllConfig};
use crate::fastmath::{atan2, sin_cos, wrap_pi};
use crate::vmath::polar;
use openvrx_core::lanes::LaneFormat;
use std::f32::consts::TAU;

/// Signal conditions the tracking demodulators adapt to.
#[derive(Copy, Clone, Debug)]
pub struct ChannelState {
    /// Mean |z|^2 at the demodulator input (ADC units^2).
    pub power: f64,
    /// Linear carrier-to-noise ratio at the demodulator input.
    pub cn: f64,
}

/// Angle table for the LUT path.
#[derive(Clone, Debug)]
struct AngleLut {
    table: Vec<u16>,
    dc: (f32, f32),
    corr: (f32, f32),
}

#[derive(Clone, Debug, Default)]
struct PllState {
    theta: f32,
    /// Posterior phase of the previous sample.
    post: f32,
    omega: f32,
    alpha: f32,
    kp: f32,
    ki: f32,
    kc: f32,
}

#[derive(Clone, Debug, Default)]
struct EkfState {
    phi: f32,
    /// Posterior phase of the previous sample.
    post: f32,
    w: f32,
    a: f32,
    /// p00 p01 p02 p11 p12 p22
    p: [f32; 6],
    q: [f32; 6],
    q_scale: f64,
    init: bool,
}

/// One demodulator instance (state persists across blocks).
#[derive(Clone, Debug)]
pub struct Demod {
    kind: DemodKind,
    fs: f64,
    prev: (f32, f32),
    // Least squares: weights and the last len-1 phasor products.
    ls_w: Vec<f32>,
    ls_hist_re: Vec<f32>,
    ls_hist_im: Vec<f32>,
    pll: PllState,
    ekf: EkfState,
    lut: Option<AngleLut>,
    lut_prev: u16,
    fmt: Option<LaneFormat>,
    ur: Vec<f32>,
    ui: Vec<f32>,
    ar: Vec<f32>,
    ai: Vec<f32>,
}

impl Demod {
    pub fn new(
        kind: DemodKind,
        fs: f64,
        fmt: Option<&LaneFormat>,
        ls: &LsConfig,
        pll: &PllConfig,
        ekf: &EkfConfig,
    ) -> Demod {
        let len = ls.len.max(1) | 1;
        let half = (len as f32 + 1.0) / 2.0;
        let mid = (len as f32 - 1.0) / 2.0;
        let ls_w: Vec<f32> = (0..len)
            .map(|k| 1.0 - ((k as f32 - mid) / half).powi(2))
            .collect();
        let mut d = Demod {
            kind,
            fs,
            prev: (0.0, 0.0),
            ls_w,
            ls_hist_re: vec![0.0; len - 1],
            ls_hist_im: vec![0.0; len - 1],
            pll: PllState::default(),
            ekf: EkfState::default(),
            lut: None,
            lut_prev: 0,
            fmt: fmt.cloned(),
            ur: Vec::new(),
            ui: Vec::new(),
            ar: Vec::new(),
            ai: Vec::new(),
        };
        d.design_pll(pll);
        d.design_ekf(ekf);
        d
    }

    pub fn kind(&self) -> DemodKind {
        self.kind
    }

    /// Integer delay beyond the phase-difference half sample.
    pub fn extra_delay(&self) -> usize {
        match self.kind {
            DemodKind::LeastSquares => (self.ls_w.len() - 1) / 2,
            _ => 0,
        }
    }

    fn design_pll(&mut self, cfg: &PllConfig) {
        let wn = (TAU as f64 * cfg.natural_hz as f64 / self.fs).clamp(1e-4, 2.5);
        let z = (cfg.damping as f64).clamp(0.1, 0.999);
        let r = (-z * wn).exp();
        let th = wn * (1.0 - z * z).sqrt();
        let c1 = -2.0 * r * th.cos();
        let c0 = r * r;
        let (kp, ki, kc) = if cfg.order >= 3 {
            let r3 = (-wn).exp();
            let (d2, d1, d0) = (c1 - r3, c0 - c1 * r3, -c0 * r3);
            let kp = 1.0 + d0;
            let ki = 3.0 - 2.0 * kp - d1;
            let kc = d2 + 3.0 - ki - kp;
            (kp, ki, kc)
        } else {
            let kp = 1.0 - c0;
            let ki = c1 + 2.0 - kp;
            (kp, ki, 0.0)
        };
        self.pll.kp = kp as f32;
        self.pll.ki = ki as f32;
        self.pll.kc = kc as f32;
    }

    fn design_ekf(&mut self, cfg: &EkfConfig) {
        let wb = TAU as f64 * cfg.bandwidth_hz as f64 / self.fs;
        let cd = 10f64.powf(cfg.design_cn_db as f64 / 10.0);
        let r_d = 1.0 / (2.0 * cd);
        let q = wb.powi(6) * r_d;
        self.ekf.q_scale = q;
        let base = [1.0 / 20.0, 1.0 / 8.0, 1.0 / 6.0, 1.0 / 3.0, 1.0 / 2.0, 1.0];
        for k in 0..6 {
            self.ekf.q[k] = (q * base[k]) as f32;
        }
    }

    pub fn reset(&mut self) {
        self.prev = (0.0, 0.0);
        self.ls_hist_re.fill(0.0);
        self.ls_hist_im.fill(0.0);
        self.pll.theta = 0.0;
        self.pll.omega = 0.0;
        self.pll.alpha = 0.0;
        self.ekf.init = false;
        self.lut_prev = 0;
    }

    /// Updates the LUT for the current DC and I/Q correction. Returns true
    /// when the table was rebuilt.
    pub fn update_lut(&mut self, dc: (f32, f32), corr: (f32, f32), step: f32) -> bool {
        let Some(fmt) = &self.fmt else { return false };
        if let Some(l) = &self.lut {
            let tol = 0.2 * step;
            if (l.dc.0 - dc.0).abs() < tol
                && (l.dc.1 - dc.1).abs() < tol
                && (l.corr.0 - corr.0).abs() < 2e-3
                && (l.corr.1 - corr.1).abs() < 2e-3
            {
                return false;
            }
        }
        let n = 1usize << (8 * fmt.word_bytes as usize);
        let mut table = Vec::with_capacity(n);
        let scale = 65536.0 / TAU;
        let mag = fmt.has_magnitude();
        for w in 0..n as u32 {
            let z = fmt.decode(w);
            let (x, y) = if mag {
                let x = z.re - dc.0;
                let y = z.im - dc.1;
                (x, corr.0 * x + corr.1 * y)
            } else {
                (z.re, z.im)
            };
            let a = (y as f64).atan2(x as f64);
            table.push(((a * scale as f64).round() as i64 & 0xffff) as u16);
        }
        self.lut = Some(AngleLut { table, dc, corr });
        true
    }

    /// LUT path on raw words. `shift` is the NCO shift in rad/sample.
    pub fn process_words(&mut self, words: &[u16], shift: f32, out: &mut [f32]) {
        let lut = self.lut.as_ref().expect("update_lut before process_words");
        let sq = (-(shift * 65536.0 / TAU)).round() as i32 as u16;
        let k = TAU / 65536.0;
        let mut prev = self.lut_prev;
        for (o, &w) in out.iter_mut().zip(words) {
            let a = lut.table[w as usize];
            let d = a.wrapping_sub(prev).wrapping_add(sq) as i16;
            *o = d as f32 * k;
            prev = a;
        }
        self.lut_prev = prev;
    }

    /// Demodulates SoA I/Q into `out` (rad/sample).
    pub fn process(&mut self, re: &[f32], im: &[f32], ch: ChannelState, out: &mut [f32]) {
        let n = re.len();
        debug_assert!(im.len() == n && out.len() == n);
        if n == 0 {
            return;
        }
        match self.kind {
            DemodKind::Polar | DemodKind::LutAngle => polar(re, im, self.prev, out),
            DemodKind::LeastSquares => {
                self.products(re, im);
                let l = self.ls_w.len();
                let h = l - 1;
                // Extended arrays: history then current products.
                let mut er = std::mem::take(&mut self.ls_hist_re);
                let mut ei = std::mem::take(&mut self.ls_hist_im);
                er.extend_from_slice(&self.ur);
                ei.extend_from_slice(&self.ui);
                let mut ar = std::mem::take(&mut self.ar);
                let mut ai = std::mem::take(&mut self.ai);
                ar.clear();
                ar.resize(n, 0.0);
                ai.clear();
                ai.resize(n, 0.0);
                for (j, &w) in self.ls_w.iter().enumerate() {
                    let (sr, si) = (&er[j..j + n], &ei[j..j + n]);
                    for k in 0..n {
                        ar[k] += w * sr[k];
                        ai[k] += w * si[k];
                    }
                }
                for k in 0..n {
                    out[k] = atan2(ai[k], ar[k]);
                }
                self.ar = ar;
                self.ai = ai;
                let t = er.len();
                er.copy_within(t - h..t, 0);
                er.truncate(h);
                ei.copy_within(t - h..t, 0);
                ei.truncate(h);
                self.ls_hist_re = er;
                self.ls_hist_im = ei;
            }
            DemodKind::Pll => self.pll_run(re, im, out),
            DemodKind::Ekf => self.ekf_run(re, im, ch, out),
        }
        self.prev = (re[n - 1], im[n - 1]);
    }

    /// u[k] = z[k] conj z[k-1] into self.ur/ui.
    fn products(&mut self, re: &[f32], im: &[f32]) {
        let n = re.len();
        self.ur.resize(n, 0.0);
        self.ui.resize(n, 0.0);
        let (pr, pi) = self.prev;
        self.ur[0] = re[0] * pr + im[0] * pi;
        self.ui[0] = im[0] * pr - re[0] * pi;
        let (ur, ui) = (&mut self.ur[1..], &mut self.ui[1..]);
        let (xr, xi, yr, yi) = (&re[1..], &im[1..], &re[..n - 1], &im[..n - 1]);
        for k in 0..n - 1 {
            ur[k] = xr[k] * yr[k] + xi[k] * yi[k];
            ui[k] = xi[k] * yr[k] - xr[k] * yi[k];
        }
    }

    fn pll_run(&mut self, re: &[f32], im: &[f32], out: &mut [f32]) {
        let s = &mut self.pll;
        let (kp, ki, kc) = (s.kp, s.ki, s.kc);
        let (mut th, mut om, mut al) = (s.theta, s.omega, s.alpha);
        let mut prev = s.post;
        for k in 0..re.len() {
            let (sn, cs) = sin_cos(th);
            // z * exp(-j th)
            let xr = re[k] * cs + im[k] * sn;
            let xi = im[k] * cs - re[k] * sn;
            let e = atan2(xi, xr);
            al += kc * e;
            om += al + ki * e;
            // Output: the step between successive posterior phase
            // estimates. The loop increment `om + kp e` is a one-sample
            // prediction and overshoots (2x gain at 4 MHz video).
            let post = wrap_pi(th + kp * e);
            out[k] = wrap_pi(post - prev);
            prev = post;
            th = wrap_pi(post + om);
        }
        s.post = prev;
        // Keep the integrators bounded (the loop frequency cannot exceed pi).
        s.theta = th;
        s.omega = om.clamp(-std::f32::consts::PI, std::f32::consts::PI);
        s.alpha = al.clamp(-1.0, 1.0);
    }

    fn ekf_run(&mut self, re: &[f32], im: &[f32], ch: ChannelState, out: &mut [f32]) {
        let st = &mut self.ekf;
        let c = ch.cn.max(0.05);
        let a2 = (ch.power * c / (1.0 + c)).max(1e-9);
        let _ = a2;
        let r = (1.0 / (2.0 * c)) as f32;
        if !st.init {
            st.phi = atan2(im[0], re[0]);
            st.post = st.phi;
            st.w = 0.0;
            st.a = 0.0;
            st.p = [1.0, 0.0, 0.0, 1.0, 0.0, 0.1];
            st.init = true;
        }
        let q = st.q;
        let (mut phi, mut w, mut a) = (st.phi, st.w, st.a);
        let mut prev = st.post;
        let [mut p00, mut p01, mut p02, mut p11, mut p12, mut p22] = st.p;
        for k in 0..re.len() {
            // Predict.
            phi = wrap_pi(phi + w + 0.5 * a);
            w += a;
            let n00 = p00 + 2.0 * p01 + p02 + p11 + p12 + 0.25 * p22 + q[0];
            let n01 = p01 + p02 + p11 + 1.5 * p12 + 0.5 * p22 + q[1];
            let n02 = p02 + p12 + 0.5 * p22 + q[2];
            let n11 = p11 + 2.0 * p12 + p22 + q[3];
            let n12 = p12 + p22 + q[4];
            let n22 = p22 + q[5];
            // Update with the exact wrapped phase innovation. The textbook
            // tangential innovation Im(z e^-j phi)/A saturates beyond 90
            // degrees of prediction error, which every sync edge reaches at
            // 13.33 MS/s; the wrapped angle stays linear to +-180 degrees.
            let (sn, cs) = sin_cos(phi);
            let y = atan2(im[k] * cs - re[k] * sn, re[k] * cs + im[k] * sn);
            let s = n00 + r;
            let is = 1.0 / s;
            let (k0, k1, k2) = (n00 * is, n01 * is, n02 * is);
            phi += k0 * y;
            w += k1 * y;
            a += k2 * y;
            p00 = n00 - k0 * n00;
            p01 = n01 - k0 * n01;
            p02 = n02 - k0 * n02;
            p11 = n11 - k1 * n01;
            p12 = n12 - k1 * n02;
            p22 = n22 - k2 * n02;
            w = w.clamp(-std::f32::consts::PI, std::f32::consts::PI);
            // Step between successive posterior phases (the frequency state
            // is the derivative at the sample, half a sample ahead of the
            // other discriminators, and extrapolates).
            phi = wrap_pi(phi);
            out[k] = wrap_pi(phi - prev);
            prev = phi;
        }
        st.phi = wrap_pi(phi);
        st.post = prev;
        st.w = w;
        st.a = a.clamp(-1.0, 1.0);
        st.p = [p00, p01, p02, p11, p12, p22];
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::lanes::LanePreset;

    fn tone_iq(n: usize, f: impl Fn(usize) -> f64, amp: f64) -> (Vec<f32>, Vec<f32>, Vec<f64>) {
        let mut ph = 0.0f64;
        let (mut re, mut im, mut fr) = (vec![], vec![], vec![]);
        for k in 0..n {
            let w = f(k);
            ph += w;
            re.push((amp * ph.cos()) as f32);
            im.push((amp * ph.sin()) as f32);
            fr.push(w);
        }
        (re, im, fr)
    }

    fn mk(kind: DemodKind) -> Demod {
        Demod::new(
            kind,
            13.333e6,
            None,
            &LsConfig::default(),
            &PllConfig::default(),
            &EkfConfig::default(),
        )
    }

    #[test]
    fn polar_exact_on_clean_fm() {
        let (re, im, fr) = tone_iq(5000, |k| 1.2 * (k as f64 * 0.01).sin(), 50.0);
        let mut d = mk(DemodKind::Polar);
        let mut out = vec![0.0; 5000];
        d.process(&re[..1234], &im[..1234], ChannelState { power: 2500.0, cn: 1e4 }, &mut out[..1234]);
        d.process(&re[1234..], &im[1234..], ChannelState { power: 2500.0, cn: 1e4 }, &mut out[1234..]);
        for k in 1..5000 {
            assert!((out[k] as f64 - fr[k]).abs() < 2e-5, "k={k} {} {}", out[k], fr[k]);
        }
    }

    #[test]
    fn least_squares_on_clean_fm() {
        let (re, im, fr) = tone_iq(4000, |k| 0.8 * (k as f64 * 0.02).sin() - 0.3, 50.0);
        let mut d = mk(DemodKind::LeastSquares);
        assert_eq!(d.extra_delay(), 1);
        let mut out = vec![0.0; 4000];
        d.process(&re, &im, ChannelState { power: 2500.0, cn: 1e4 }, &mut out);
        // Output at k estimates the frequency around k - 1 (weighted).
        for k in 10..4000 {
            let expect = (0.75 * fr[k - 2] + fr[k - 1] + 0.75 * fr[k]) / 2.5;
            assert!((out[k] as f64 - expect).abs() < 1e-3, "k={k}");
        }
    }

    #[test]
    fn trackers_follow_clean_fm() {
        for kind in [DemodKind::Pll, DemodKind::Ekf] {
            for order in [2u8, 3] {
                let (re, im, fr) = tone_iq(20000, |k| 1.0 * (k as f64 * 0.05).sin() + 0.4, 60.0);
                let mut d = Demod::new(
                    kind,
                    13.333e6,
                    None,
                    &LsConfig::default(),
                    &PllConfig {
                        order,
                        ..PllConfig::default()
                    },
                    &EkfConfig::default(),
                );
                let mut out = vec![0.0; 20000];
                d.process(&re, &im, ChannelState { power: 3600.0, cn: 1e3 }, &mut out);
                // Allow a small lag: compare against the best of lags 0..3.
                let err = (0..4)
                    .map(|lag| {
                        let mut e = 0.0f64;
                        for k in 2000..20000 {
                            e += (out[k] as f64 - fr[k - lag]).powi(2);
                        }
                        (e / 18000.0).sqrt()
                    })
                    .fold(f64::INFINITY, f64::min);
                assert!(err < 0.05, "{kind:?} order {order} rms err {err}");
            }
        }
    }

    #[test]
    fn lut_path_matches_polar_on_words() {
        let fmt = LanePreset::I8Q8.format();
        let mut d = Demod::new(
            DemodKind::LutAngle,
            13.333e6,
            Some(&fmt),
            &LsConfig::default(),
            &PllConfig::default(),
            &EkfConfig::default(),
        );
        assert!(d.update_lut((0.0, 0.0), (0.0, 1.0), 4.0));
        assert!(!d.update_lut((0.1, 0.0), (0.0, 1.0), 4.0));
        let (re, im, fr) = tone_iq(3000, |k| 2.5 * (k as f64 * 0.003).sin(), 400.0);
        let words: Vec<u16> = re
            .iter()
            .zip(&im)
            .map(|(&r, &i)| fmt.encode(r.round() as i32, i.round() as i32) as u16)
            .collect();
        let mut out = vec![0.0; 3000];
        let shift = 0.37f32;
        d.process_words(&words, shift, &mut out);
        for k in 1..3000 {
            let e = wrap_pi(out[k] - (fr[k] as f32 - shift));
            assert!(e.abs() < 0.02, "k={k} e={e}");
        }
        // Angle-only format.
        let fa = LanePreset::Angle8.format();
        let mut da = Demod::new(
            DemodKind::LutAngle,
            13.333e6,
            Some(&fa),
            &LsConfig::default(),
            &PllConfig::default(),
            &EkfConfig::default(),
        );
        da.update_lut((0.0, 0.0), (0.0, 1.0), 1.0);
        let wa: Vec<u16> = re
            .iter()
            .zip(&im)
            .map(|(&r, &i)| fa.encode((r * 1.2) as i32, (i * 1.2) as i32) as u16)
            .collect();
        da.process_words(&wa, 0.0, &mut out);
        let mut rms = 0.0;
        for k in 1..3000 {
            rms += (wrap_pi(out[k] - fr[k] as f32) as f64).powi(2);
        }
        assert!((rms / 3000.0).sqrt() < TAU as f64 / 256.0, "angle8 rms {rms}");
    }
}
