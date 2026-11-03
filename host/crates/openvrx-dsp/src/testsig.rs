// SPDX-License-Identifier: GPL-3.0-or-later
//! Synthetic test signals: CVBS raster -> FM at 80 MS/s -> C5-like channel
//! -> noise and fading -> unfiltered decimation -> 10-bit ADC with DC and
//! I/Q imbalance -> lane words.
//!
//! The signal model follows the C5VRX receive-chain simulator (GPL-3.0,
//! Twotoz and the C5VRX contributors, commit 6bc9e4b, `research/sim.py`):
//! FM at the 80 MS/s bus rate, 6.7 MHz from sync tip to peak white at
//! deviation 1.0 (6.7 MHz per 140 IRE), blanking at -1 MHz, a 5th-order
//! Butterworth channel at 10 MHz, C/N measured in that channel, decimation
//! by keeping every N-th bus sample. Changes: the raster is rendered
//! analytically in continuous time (raised-cosine edges, so 0H and the
//! clean video are known exactly at any time), the vertical interval is
//! generated from the standard frame-line tables, line and sample clocks
//! can be offset in ppm, Rayleigh fading, DC, I/Q imbalance, lane encoding
//! and sampling-phase mixed reads are added, and generation is streaming.

use crate::filters::{butterworth_lowpass, Sos};
use openvrx_core::lanes::{C5_BUS_HZ, SampleFormat};
use openvrx_core::{FieldParity, StandardParams, VideoStandard};
use std::f64::consts::{PI, TAU};

/// Hz of deviation per IRE at deviation scale 1.0.
pub const HZ_PER_IRE: f64 = 6.7e6 / 140.0;

/// Picture content.
#[derive(Clone, Debug, PartialEq)]
pub enum Picture {
    /// Constant level (IRE).
    Flat(f32),
    /// 75 % colour bars.
    Bars,
    /// Black to white ramp across the active line.
    Ramp,
    /// Smooth random texture (knots every 0.2 us, 5..95 IRE) plus an
    /// 8 IRE subcarrier, new every frame.
    Texture(u64),
    /// White box moving across a grey field.
    Moving,
}

/// Rayleigh flat fading (sum of sinusoids, Clarke model).
#[derive(Clone, Debug, PartialEq)]
pub struct Fading {
    pub doppler_hz: f64,
    pub seed: u64,
}

/// Everything about a synthetic capture.
#[derive(Clone, Debug)]
pub struct SigSpec {
    pub standard: VideoStandard,
    pub picture: Picture,
    pub burst: bool,
    /// 1.0 = 6.7 MHz sync tip to peak white.
    pub deviation: f64,
    /// Blanking frequency relative to the tuned centre.
    pub blank_hz: f64,
    /// Transmitter line-clock error.
    pub tx_ppm: f64,
    /// Receiver sample-clock error (positive = receiver clock fast).
    pub rx_ppm: f64,
    /// Frame line (1-based) at the start, and microseconds into it.
    pub start_line: u16,
    pub start_us: f64,
    /// C/N in the channel bandwidth; None = no noise.
    pub cn_db: Option<f64>,
    pub fading: Option<Fading>,
    /// Butterworth channel -3 dB frequency; None = no channel filter.
    pub channel_hz: Option<f64>,
    pub sample_format: SampleFormat,
    /// Which bus sample of each group of N is kept.
    pub decim_phase: usize,
    /// Signal-plus-noise RMS at the ADC (10-bit units, complex RMS).
    pub rms_adc: f64,
    pub dc: (f64, f64),
    /// Q gain (linear) and phase (rad) imbalance.
    pub iq_gain: f64,
    pub iq_phase: f64,
    /// Word bits that read the next bus sample (sampling-phase lottery).
    pub mixed_mask: u32,
    /// Probability per bit and sample of a random old/new pick on the
    /// bits in `mixed_mask` (0 = always the next sample).
    pub mixed_random: f64,
    /// Constant initial carrier phase and noise seed.
    pub seed: u64,
}

impl Default for SigSpec {
    fn default() -> Self {
        SigSpec {
            standard: VideoStandard::Ntsc,
            picture: Picture::Texture(1),
            burst: true,
            deviation: 1.0,
            blank_hz: -1.0e6,
            tx_ppm: 0.0,
            rx_ppm: 0.0,
            start_line: 1,
            start_us: 30.0,
            cn_db: None,
            fading: None,
            channel_hz: Some(10e6),
            sample_format: SampleFormat::from_preset(openvrx_core::LanePreset::I7Q7, 18),
            decim_phase: 0,
            rms_adc: 180.0,
            dc: (0.0, 0.0),
            iq_gain: 1.0,
            iq_phase: 0.0,
            mixed_mask: 0,
            mixed_random: 0.0,
            seed: 1,
        }
    }
}

/// 0H of a generated line as it appears in the output sample stream.
#[derive(Copy, Clone, Debug)]
pub struct LineTruth {
    /// Output sample index of 0H (fractional), including the channel delay.
    pub h0: f64,
    pub frame_line: u16,
    pub parity: FieldParity,
    /// Global line counter from the start of the generator.
    pub index: u64,
}

/// One generated chunk.
#[derive(Clone, Debug, Default)]
pub struct Chunk {
    /// Lane words as bytes.
    pub bytes: Vec<u8>,
    /// Decoded words (what the unpacker produces).
    pub re: Vec<f32>,
    pub im: Vec<f32>,
    /// Noise-free complex baseband after the channel (before gain and
    /// quantisation, unit carrier amplitude), at the output rate.
    pub clean_re: Vec<f32>,
    pub clean_im: Vec<f32>,
    /// Clean video (IRE) at the time each output sample represents.
    pub truth: Vec<f32>,
}

/// Deterministic xoshiro256++ generator.
#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let mut z = seed.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut s = [0u64; 4];
        for v in &mut s {
            z = z.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            *v = x ^ (x >> 31);
        }
        Rng { s, spare: None }
    }

    pub fn next_u64(&mut self) -> u64 {
        let r = (self.s[0].wrapping_add(self.s[3])).rotate_left(23).wrapping_add(self.s[0]);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        r
    }

    /// Uniform in (0, 1).
    pub fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal.
    pub fn gauss(&mut self) -> f64 {
        if let Some(v) = self.spare.take() {
            return v;
        }
        let r = (-2.0 * self.uniform().ln()).sqrt();
        let t = TAU * self.uniform();
        self.spare = Some(r * t.sin());
        r * t.cos()
    }
}

fn hash(a: u64, b: u64, c: u64) -> f64 {
    let mut x = a
        .wrapping_mul(0x9e37_79b9_7f4a_7c15)
        .wrapping_add(b.wrapping_mul(0xc2b2_ae3d_27d4_eb4f))
        .wrapping_add(c.wrapping_mul(0x1656_67b1_9e37_79f9));
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^= x >> 31;
    (x >> 11) as f64 / (1u64 << 53) as f64
}

/// Raised-cosine step from 0 to 1 centred at 0 with total width `tr`.
#[inline]
fn rc_step(x: f64, tr: f64) -> f64 {
    if x <= -0.5 * tr {
        0.0
    } else if x >= 0.5 * tr {
        1.0
    } else {
        0.5 * (1.0 + (PI * x / tr).sin())
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Pulse {
    None,
    H,
    Eq,
    Broad,
}

/// Pulses at 0 and H/2 and whether the line carries picture in its first
/// and second half, burst, per frame line.
fn layout(std: VideoStandard, l: u16) -> ([Pulse; 2], bool, bool) {
    use Pulse::*;
    let p = std.params();
    let lpf0 = p.lines_per_field[0];
    let first_active = |l: u16| {
        let par = if l <= lpf0 { 0 } else { 1 };
        l >= p.first_active_line[par]
    };
    match std {
        VideoStandard::Ntsc => match l {
            1..=3 | 7..=9 | 264 | 265 | 270 | 271 => ([Eq, Eq], false, false),
            4..=6 | 267 | 268 => ([Broad, Broad], false, false),
            263 => ([H, Eq], true, false),
            266 => ([Eq, Broad], false, false),
            269 => ([Broad, Eq], false, false),
            272 => ([Eq, None], false, false),
            _ => ([H, None], first_active(l), true),
        },
        VideoStandard::Pal => match l {
            1 | 2 | 314 | 315 => ([Broad, Broad], false, false),
            3 => ([Broad, Eq], false, false),
            4 | 5 | 311 | 312 | 316 | 317 | 624 | 625 => ([Eq, Eq], false, false),
            313 => ([Eq, Broad], false, false),
            318 => ([Eq, None], false, false),
            623 => ([H, Eq], true, false),
            _ => ([H, None], first_active(l) && l <= 622 && l != 311, true),
        },
    }
}

/// Analytic CVBS renderer.
#[derive(Clone, Debug)]
pub struct Raster {
    pub std: VideoStandard,
    pub picture: Picture,
    pub burst: bool,
    p: &'static StandardParams,
    tr_sync: f64,
}

impl Raster {
    pub fn new(std: VideoStandard, picture: Picture, burst: bool) -> Raster {
        let p = std.params();
        let rise = if std == VideoStandard::Ntsc { 0.14 } else { 0.25 };
        Raster {
            std,
            picture,
            burst,
            p,
            tr_sync: rise / 0.59,
        }
    }

    pub fn line_us(&self) -> f64 {
        1e6 / self.p.line_hz
    }

    /// IRE at microseconds `t` into frame line `l` of frame `frame`;
    /// `t_abs` (seconds) drives the subcarrier phase.
    pub fn ire(&self, frame: u64, l: u16, t: f64, t_abs: f64) -> f64 {
        let p = self.p;
        let h = self.line_us();
        let sync = p.sync_ire as f64;
        let (pulses, pic1, pic2) = layout(self.std, l);
        // Sync pulses at 0 and H/2.
        let mut low = 0.0;
        for (k, pu) in pulses.iter().enumerate() {
            let w = match pu {
                Pulse::None => continue,
                Pulse::H => p.hsync_us,
                Pulse::Eq => p.eq_pulse_us,
                Pulse::Broad => p.broad_pulse_us,
            };
            let s = k as f64 * h / 2.0;
            // Pulses may wrap from the end of the previous line: none do here.
            low += rc_step(t - s, self.tr_sync) - rc_step(t - s - w, self.tr_sync);
        }
        let mut v = sync * low;
        // Burst on lines with a horizontal sync.
        if self.burst && pulses[0] == Pulse::H && pic2 {
            let bs = p.burst_start_us;
            let bl = p.burst_cycles / p.fsc_hz * 1e6;
            let env = rc_step(t - bs - 0.15, 0.3) - rc_step(t - bs - bl + 0.15, 0.3);
            if env > 0.0 {
                let alt = if self.std == VideoStandard::Pal && l % 2 == 1 { 0.75 } else { -0.75 };
                let ph = if self.std == VideoStandard::Pal { alt * PI } else { PI };
                v += p.burst_ire as f64 * env * (TAU * p.fsc_hz * t_abs + ph).sin();
            }
        }
        // Picture.
        let a0 = p.active_start_us;
        let a1 = a0 + p.active_us;
        let (lo, hi) = match (pic1, pic2) {
            (true, true) => (a0, a1),
            (true, false) => (a0, h / 2.0 - 1.6),
            (false, true) => (h / 2.0 + 0.5, a1),
            (false, false) => (0.0, 0.0),
        };
        if hi > lo {
            let env = rc_step(t - lo, 0.3) - rc_step(t - hi, 0.3);
            if env > 0.0 {
                let x = (t - a0) / (a1 - a0);
                v += env * self.picture_ire(frame, l, x, t, t_abs);
            }
        }
        v
    }

    fn picture_ire(&self, frame: u64, l: u16, x: f64, t: f64, t_abs: f64) -> f64 {
        let p = self.p;
        let black = p.black_ire as f64;
        let span = 100.0 - black;
        let sc = TAU * p.fsc_hz * t_abs;
        match &self.picture {
            Picture::Flat(v) => *v as f64,
            Picture::Ramp => black + span * x.clamp(0.0, 1.0),
            Picture::Bars => {
                // 75 % bars: white, yellow, cyan, green, magenta, red, blue, black.
                const RGB: [(f64, f64, f64); 8] = [
                    (0.75, 0.75, 0.75),
                    (0.75, 0.75, 0.0),
                    (0.0, 0.75, 0.75),
                    (0.0, 0.75, 0.0),
                    (0.75, 0.0, 0.75),
                    (0.75, 0.0, 0.0),
                    (0.0, 0.0, 0.75),
                    (0.0, 0.0, 0.0),
                ];
                let pos = x.clamp(0.0, 0.9999) * 8.0;
                let k = pos.floor() as usize;
                let frac = pos - k as f64;
                let val = |k: usize| {
                    let (r, g, b) = RGB[k.min(7)];
                    let y = 0.299 * r + 0.587 * g + 0.114 * b;
                    let u = 0.492 * (b - y);
                    let v = 0.877 * (r - y);
                    black + span * (y + u * sc.sin() + v * sc.cos())
                };
                // Smooth bar edges over 0.15 us.
                let bw = p.active_us / 8.0;
                let edge = rc_step((frac - 1.0) * bw, 0.15);
                let e0 = rc_step(frac * bw, 0.15);
                let prev = if k > 0 { val(k - 1) } else { val(0) };
                let cur = val(k);
                let next = val(k + 1);
                prev + (cur - prev) * e0 + (next - cur) * edge
            }
            Picture::Texture(seed) => {
                let step = 0.2;
                let u = (t - p.active_start_us) / step;
                let k = u.floor();
                let f = u - k;
                let ki = k as i64 as u64;
                let key = frame.wrapping_mul(1000) + l as u64;
                let a = 5.0 + 90.0 * hash(*seed, key, ki);
                let b = 5.0 + 90.0 * hash(*seed, key, ki.wrapping_add(1));
                let w = 0.5 - 0.5 * (PI * f).cos();
                a + (b - a) * w + 8.0 * (sc + 0.6).sin()
            }
            Picture::Moving => {
                let pos = ((frame % 60) as f64 / 60.0) * 0.8;
                let lw = 0.1;
                let in_box = rc_step((x - pos) * p.active_us, 0.15) - rc_step((x - pos - lw) * p.active_us, 0.15);
                let row = (l as f64 % 200.0) / 200.0;
                let vin = if (0.3..0.6).contains(&row) { 1.0 } else { 0.0 };
                50.0 + 50.0 * in_box * vin
            }
        }
    }
}

/// Streaming generator.
pub struct SigGen {
    pub spec: SigSpec,
    raster: Raster,
    n_dec: usize,
    fs_out: f64,
    /// Bus sample counter (RX time).
    k80: u64,
    phase: f64,
    chan_re: Option<Sos>,
    chan_im: Option<Sos>,
    noise_re: Option<Sos>,
    noise_im: Option<Sos>,
    sig_scale: f64,
    noise_sigma: f64,
    /// Noise sigma for 0 dB C/N.
    unit_noise: f64,
    gain: f64,
    rng: Rng,
    fade: Vec<(f64, f64, f64)>,
    /// Channel group delay at DC (bus samples).
    chan_delay_bus: f64,
    out_index: u64,
    next_line: u64,
    pub lines: Vec<LineTruth>,
    // Previous bus sample (for mixed reads): quantised word.
    prev_word: u32,
}

impl SigGen {
    pub fn new(spec: SigSpec) -> SigGen {
        assert!(spec.sample_format.bus_locked(), "generator needs a divider that is a multiple of 3");
        let n_dec = (spec.sample_format.divider / 3) as usize;
        let fs_out = spec.sample_format.sample_rate_hz();
        let raster = Raster::new(spec.standard, spec.picture.clone(), spec.burst);
        let chan = spec.channel_hz.map(|f| butterworth_lowpass(5, f / C5_BUS_HZ));
        let chan_delay_bus = chan.as_ref().map(|s| s.group_delay(1e-4)).unwrap_or(0.0);
        let mut rng = Rng::new(spec.seed);
        let fade = match &spec.fading {
            Some(f) => {
                let mut r = Rng::new(f.seed);
                (0..16)
                    .map(|_| {
                        (
                            TAU * f.doppler_hz * (TAU * r.uniform()).cos(),
                            TAU * r.uniform(),
                            TAU * r.uniform(),
                        )
                    })
                    .collect()
            }
            None => Vec::new(),
        };
        // Noise power gain of the channel.
        let noise_gain = match &chan {
            Some(s) => {
                let mut s = s.clone();
                (0..20000)
                    .map(|k| {
                        let v = s.run(if k == 0 { 1.0 } else { 0.0 });
                        v * v
                    })
                    .sum::<f64>()
            }
            None => 1.0,
        };
        let mut g = SigGen {
            phase: TAU * rng.uniform(),
            spec: spec.clone(),
            raster,
            n_dec,
            fs_out,
            k80: 0,
            chan_re: chan.clone(),
            chan_im: chan.clone(),
            noise_re: chan.clone(),
            noise_im: chan,
            sig_scale: 1.0,
            noise_sigma: 0.0,
            unit_noise: (1.0 / noise_gain).sqrt(),
            gain: 1.0,
            rng,
            fade,
            chan_delay_bus,
            out_index: 0,
            next_line: 0,
            lines: Vec::new(),
            prev_word: 0,
        };
        // Signal power after the channel: measure on a clean copy of the
        // first line.
        let sig_pow = {
            let mut probe = SigGen {
                spec: SigSpec { cn_db: None, fading: None, ..spec.clone() },
                raster: g.raster.clone(),
                n_dec: 1,
                fs_out: C5_BUS_HZ,
                k80: 0,
                phase: 0.0,
                chan_re: g.chan_re.clone(),
                chan_im: g.chan_im.clone(),
                noise_re: None,
                noise_im: None,
                sig_scale: 1.0,
                noise_sigma: 0.0,
                unit_noise: 0.0,
                gain: 1.0,
                rng: Rng::new(0),
                fade: Vec::new(),
                chan_delay_bus: 0.0,
                out_index: 0,
                next_line: u64::MAX,
                lines: Vec::new(),
                prev_word: 0,
            };
            let mut acc = 0.0;
            let n = 12000;
            for k in 0..n {
                let (r, i, _) = probe.bus_sample();
                if k >= 2000 {
                    acc += r * r + i * i;
                }
            }
            acc / (n - 2000) as f64
        };
        g.sig_scale = 1.0 / sig_pow.sqrt();
        let c = spec.cn_db.map(|d| 10f64.powf(d / 10.0));
        g.noise_sigma = match c {
            Some(c) => (1.0 / c / noise_gain).sqrt(),
            None => 0.0,
        };
        let total = 1.0 + c.map(|c| 1.0 / c).unwrap_or(0.0);
        g.gain = spec.rms_adc / total.sqrt();
        g
    }

    /// Changes the ADC level and DC from the next sample on (a receiver
    /// gain step).
    pub fn set_gain_dc(&mut self, rms_adc: f64, dc: (f64, f64)) {
        self.gain *= rms_adc / self.spec.rms_adc;
        self.spec.rms_adc = rms_adc;
        self.spec.dc = dc;
    }

    /// Sets the C/N from the next sample on (None = no noise). The signal
    /// power is unchanged; only the noise level moves.
    pub fn set_cn(&mut self, cn_db: Option<f64>) {
        self.noise_sigma = cn_db.map(|d| self.unit_noise * 10f64.powf(-d / 20.0)).unwrap_or(0.0);
        self.spec.cn_db = cn_db;
    }

    pub fn sample_rate(&self) -> f64 {
        self.fs_out
    }

    /// Channel group delay in output samples (0H appears this much later).
    pub fn channel_delay(&self) -> f64 {
        self.chan_delay_bus / self.n_dec as f64
    }

    /// Delay between the clean video at time t and the discriminator
    /// output sample (phase difference over one output interval centred
    /// half an interval earlier, plus the channel), in output samples.
    pub fn video_delay(&self) -> f64 {
        self.channel_delay() + 0.5 * (self.n_dec as f64 - 1.0) / self.n_dec as f64
    }

    pub fn line_samples(&self) -> f64 {
        self.fs_out / self.spec.standard.params().line_hz * (1.0 + self.spec.rx_ppm * 1e-6)
            / (1.0 + self.spec.tx_ppm * 1e-6)
    }

    /// TX time (seconds from the start line's 0H) of RX bus time `t_rx`.
    fn tx_time(&self, t_rx: f64) -> f64 {
        t_rx * (1.0 + self.spec.tx_ppm * 1e-6) / (1.0 + self.spec.rx_ppm * 1e-6) + self.spec.start_us * 1e-6
    }

    /// RX output sample index of TX time `t_tx`.
    fn out_pos(&self, t_tx: f64) -> f64 {
        let t_rx = (t_tx - self.spec.start_us * 1e-6) * (1.0 + self.spec.rx_ppm * 1e-6)
            / (1.0 + self.spec.tx_ppm * 1e-6);
        (t_rx * C5_BUS_HZ - self.spec.decim_phase as f64) / self.n_dec as f64
    }

    /// Frame line and frame number for global line `g` (from the start).
    fn line_of(&self, g: u64) -> (u16, u64) {
        let p = self.spec.standard.params();
        let lpf = p.lines_per_frame as u64;
        let x = self.spec.start_line as u64 - 1 + g;
        ((x % lpf) as u16 + 1, x / lpf)
    }

    /// Clean IRE at TX time.
    pub fn ire_at(&self, t_tx: f64) -> f64 {
        let tl = 1.0 / self.spec.standard.params().line_hz;
        let g = (t_tx / tl).floor();
        let (l, frame) = self.line_of(g.max(0.0) as u64);
        self.raster.ire(frame, l, (t_tx - g * tl) * 1e6, t_tx)
    }

    /// Truth for one grid line whose 0H was reported at `src_position`
    /// (output sample index, the convention of `LineRecord`). Returns the
    /// clean IRE on the grid and the frame line of the matching generated
    /// line.
    pub fn truth_line(&self, src_position: f64, spl: usize) -> (Vec<f32>, u16) {
        let pos = src_position - self.channel_delay();
        let t_rx = (pos * self.n_dec as f64 + self.spec.decim_phase as f64) / C5_BUS_HZ;
        let t_tx = self.tx_time(t_rx);
        let tl = 1.0 / self.spec.standard.params().line_hz;
        let g = (t_tx / tl).round();
        let (l, _) = self.line_of(g.max(0.0) as u64);
        let v = (0..spl)
            .map(|k| {
                // Sample from the given position exactly (not the ideal
                // line start), so callers can test fractional alignments.
                let ta = t_tx + (k as f64 / spl as f64) * tl;
                self.ire_at(ta) as f32
            })
            .collect();
        (v, l)
    }

    /// One bus sample: signal (after channel, unit scale) and the TX time.
    fn bus_sample(&mut self) -> (f64, f64, f64) {
        let t_rx = self.k80 as f64 / C5_BUS_HZ;
        let t_tx = self.tx_time(t_rx);
        let ire = self.ire_at(t_tx);
        let f = self.spec.blank_hz + self.spec.deviation * HZ_PER_IRE * ire;
        self.phase += TAU * f / C5_BUS_HZ;
        if self.phase > PI {
            self.phase -= TAU;
        } else if self.phase < -PI {
            self.phase += TAU;
        }
        let (mut re, mut im) = (self.phase.cos(), self.phase.sin());
        if !self.fade.is_empty() {
            let (mut fr, mut fi) = (0.0, 0.0);
            for &(w, a, _) in &self.fade {
                let x = w * t_rx + a;
                fr += x.cos();
                fi += x.sin();
            }
            let s = (1.0 / self.fade.len() as f64).sqrt();
            let (fr, fi) = (fr * s, fi * s);
            let r = re * fr - im * fi;
            im = re * fi + im * fr;
            re = r;
        }
        if let (Some(a), Some(b)) = (&mut self.chan_re, &mut self.chan_im) {
            re = a.run(re);
            im = b.run(im);
        }
        self.k80 += 1;
        (re * self.sig_scale, im * self.sig_scale, ire)
    }

    fn noise_sample(&mut self) -> (f64, f64) {
        if self.noise_sigma == 0.0 {
            return (0.0, 0.0);
        }
        let s = self.noise_sigma / 2f64.sqrt();
        let (mut a, mut b) = (self.rng.gauss() * s, self.rng.gauss() * s);
        if let (Some(x), Some(y)) = (&mut self.noise_re, &mut self.noise_im) {
            a = x.run(a);
            b = y.run(b);
        }
        (a, b)
    }

    fn quantise(&self, re: f64, im: f64) -> (i32, i32) {
        let (g, p) = (self.spec.iq_gain, self.spec.iq_phase);
        let i = re * self.gain + self.spec.dc.0;
        let q = g * (im * p.cos() + re * p.sin()) * self.gain + self.spec.dc.1;
        (
            (i.floor() as i64).clamp(-512, 511) as i32,
            (q.floor() as i64).clamp(-512, 511) as i32,
        )
    }

    /// Generates `n` output samples.
    pub fn next(&mut self, n: usize) -> Chunk {
        let fmt = self.spec.sample_format.lanes.clone();
        let wb = fmt.word_bytes as usize;
        let mut c = Chunk {
            bytes: Vec::with_capacity(n * wb),
            re: Vec::with_capacity(n),
            im: Vec::with_capacity(n),
            clean_re: Vec::with_capacity(n),
            clean_im: Vec::with_capacity(n),
            truth: Vec::with_capacity(n),
        };
        let tl = 1.0 / self.spec.standard.params().line_hz;
        let vdelay = self.video_delay();
        for _ in 0..n {
            let mut kept = None;
            let mut word_next = None;
            for j in 0..self.n_dec {
                let (sr, si, _) = self.bus_sample();
                let (nr, ni) = self.noise_sample();
                if j == self.spec.decim_phase {
                    kept = Some((sr, si, nr, ni));
                } else if j == (self.spec.decim_phase + 1) % self.n_dec && self.spec.mixed_mask != 0 {
                    let (qi, qq) = self.quantise(sr + nr, si + ni);
                    word_next = Some(fmt.encode(qi, qq));
                }
            }
            let (sr, si, nr, ni) = kept.expect("decim phase < N");
            let (qi, qq) = self.quantise(sr + nr, si + ni);
            let mut w = fmt.encode(qi, qq);
            if self.spec.mixed_mask != 0 {
                // With N = 1 the next bus sample is the next output sample.
                let wn = word_next.unwrap_or(self.prev_word);
                let mut mask = self.spec.mixed_mask;
                if self.spec.mixed_random > 0.0 {
                    let mut m = 0;
                    for b in 0..16 {
                        if mask & (1 << b) != 0 && self.rng.uniform() < self.spec.mixed_random {
                            m |= 1 << b;
                        }
                    }
                    mask = m;
                }
                w = (w & !mask) | (wn & mask);
            }
            self.prev_word = w;
            if wb == 1 {
                c.bytes.push(w as u8);
            } else {
                c.bytes.extend_from_slice(&(w as u16).to_le_bytes());
            }
            let z = fmt.decode(w);
            c.re.push(z.re);
            c.im.push(z.im);
            c.clean_re.push((sr / self.sig_scale) as f32);
            c.clean_im.push((si / self.sig_scale) as f32);
            // Truth: the time this output sample's discriminator value
            // represents.
            let pos = self.out_index as f64 - vdelay;
            let t_rx = (pos * self.n_dec as f64 + self.spec.decim_phase as f64) / C5_BUS_HZ;
            c.truth.push(self.ire_at(self.tx_time(t_rx)) as f32);
            self.out_index += 1;
        }
        // Line truth for every 0H inside this chunk's span.
        let p = self.spec.standard.params();
        let lpf0 = p.lines_per_field[0];
        loop {
            let t0 = self.next_line as f64 * tl + if self.spec.start_us > 0.0 { tl } else { 0.0 };
            let h0 = self.out_pos(t0) + self.channel_delay();
            if h0 >= self.out_index as f64 {
                break;
            }
            let g = self.next_line + if self.spec.start_us > 0.0 { 1 } else { 0 };
            let (l, _) = self.line_of(g);
            self.lines.push(LineTruth {
                h0,
                frame_line: l,
                parity: if l <= lpf0 { FieldParity::First } else { FieldParity::Second },
                index: g,
            });
            self.next_line += 1;
        }
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raster_levels_and_vbi() {
        let r = Raster::new(VideoStandard::Ntsc, Picture::Flat(50.0), false);
        assert!((r.ire(0, 100, 2.0, 0.0) + 40.0).abs() < 1e-9);
        assert!(r.ire(0, 100, 0.0, 0.0).abs() - 20.0 < 1e-9); // 50 % point
        assert!((r.ire(0, 100, 30.0, 0.0) - 50.0).abs() < 1e-9);
        // Broad pulse on line 4 at 10 us, eq pulse on line 1 short.
        assert!((r.ire(0, 4, 10.0, 0.0) + 40.0).abs() < 1e-9);
        assert!(r.ire(0, 1, 3.0, 0.0).abs() < 1e-9);
        // Field 2 broad pulse in the second half of line 266.
        assert!((r.ire(0, 266, 40.0, 0.0) + 40.0).abs() < 1e-9);
        assert!(r.ire(0, 266, 10.0, 0.0).abs() < 1e-9);
        let pal = Raster::new(VideoStandard::Pal, Picture::Flat(50.0), false);
        assert!((pal.ire(0, 1, 10.0, 0.0) + 300.0 / 7.0).abs() < 1e-5);
        assert!((pal.ire(0, 313, 40.0, 0.0) + 300.0 / 7.0).abs() < 1e-5);
    }

    #[test]
    fn generator_power_and_lines() {
        let spec = SigSpec {
            cn_db: Some(10.0),
            rms_adc: 80.0,
            sample_format: SampleFormat::from_preset(openvrx_core::LanePreset::I8Q8, 18),
            ..SigSpec::default()
        };
        let mut g = SigGen::new(spec);
        let c = g.next(40000);
        let p: f64 = c.re.iter().zip(&c.im).map(|(a, b)| (*a as f64).powi(2) + (*b as f64).powi(2)).sum::<f64>()
            / c.re.len() as f64;
        assert!((p.sqrt() - 80.0).abs() < 4.0, "rms {}", p.sqrt());
        // Lines are one period apart.
        let ls = &g.lines;
        assert!(ls.len() >= 45);
        for w in ls.windows(2) {
            assert!((w[1].h0 - w[0].h0 - g.line_samples()).abs() < 1e-6);
            assert_eq!(w[1].frame_line, w[0].frame_line % 525 + 1);
        }
        assert!(c.truth.iter().any(|&v| v < -39.0));
    }
}
