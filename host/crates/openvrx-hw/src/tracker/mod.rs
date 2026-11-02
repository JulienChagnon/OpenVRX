// SPDX-License-Identifier: GPL-3.0-or-later
//! Antenna tracker for two patch antennas on a pan/tilt head (DESIGN 10).
//!
//! Amplitude-comparison monopulse: patch A looks `squint` degrees to one
//! side of the head's boresight, patch B the same amount to the other. With
//! roughly Gaussian beams the gain-normalised power difference
//! `(P_A - G_A) - (P_B - G_B)` is proportional to the angle between the
//! boresight and the quad, `48 s / bw^2` dB per degree. A PI loop with a
//! deadband, a slew limit and gain scheduling by signal level steers the
//! azimuth; when both receivers lose the signal the head holds, then sweeps
//! a widening search around the last good direction. Azimuths beyond the
//! pan servo's +-90 degrees use the classic 180/180 flip-over (pan by 180,
//! tilt past the zenith), which also mirrors the patches.

pub mod maestro;

use crate::control::gain::GainTable;
use crate::error::Result;
use maestro::{Maestro, ServoCal};
use serde::{Deserialize, Serialize};

/// Beam geometry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MonopulseConfig {
    /// Half the angle between the two patch boresights.
    pub squint_deg: f64,
    /// -3 dB beamwidth of one patch.
    pub beamwidth_deg: f64,
}

impl Default for MonopulseConfig {
    fn default() -> Self {
        MonopulseConfig {
            squint_deg: 20.0,
            beamwidth_deg: 65.0,
        }
    }
}

impl MonopulseConfig {
    /// Error slope near boresight, dB per degree.
    pub fn slope_db_per_deg(&self) -> f64 {
        48.0 * self.squint_deg / (self.beamwidth_deg * self.beamwidth_deg)
    }

    /// Gain of one patch at `off_deg` from its boresight (dB, Gaussian
    /// main lobe floored at -30 dB). Used by the simulation and tests.
    pub fn pattern_db(&self, off_deg: f64) -> f64 {
        (-12.0 * (off_deg / self.beamwidth_deg).powi(2)).max(-30.0)
    }
}

/// Per-board receive gain to dB conversion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoardGain {
    pub table: GainTable,
    /// Board-to-board offset from the swap test (dB).
    pub offset_db: f64,
}

impl BoardGain {
    pub fn gain_db(&self, index: u32) -> f64 {
        self.table.db_at(index) as f64 + self.offset_db
    }
}

/// Monopulse error in dB from raw powers and gain indices.
pub fn monopulse_error_db(pa_db: f64, gain_a: u32, a: &BoardGain, pb_db: f64, gain_b: u32, b: &BoardGain) -> f64 {
    (pa_db - a.gain_db(gain_a)) - (pb_db - b.gain_db(gain_b))
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackerConfig {
    pub monopulse: MonopulseConfig,
    /// Control period (s); measurements are averaged over it.
    pub period_s: f64,
    /// Error deadband (dB).
    pub deadband_db: f64,
    /// Proportional gain (deg/s per degree of error).
    pub kp: f64,
    /// Integral gain (deg/s per degree-second).
    pub ki: f64,
    /// Slew limit of the azimuth command (deg/s).
    pub max_rate_deg_s: f64,
    /// Plausible largest error in one period (multipath rejection), deg.
    pub max_error_deg: f64,
    /// Gain scheduling: full gain above, `low_gain` below `cn_low_db`.
    pub cn_full_db: f32,
    pub cn_low_db: f32,
    pub low_gain: f64,
    /// Lost signal: both receivers below this for `lost_s`.
    pub lost_cn_db: f32,
    pub lost_s: f64,
    /// Hold time before searching.
    pub hold_s: f64,
    pub search_rate_deg_s: f64,
    /// Search amplitude growth per half sweep and its limit (deg).
    pub search_step_deg: f64,
    pub search_max_deg: f64,
    /// Reacquire when either receiver is above this for `acquire_s`.
    pub acquire_cn_db: f32,
    pub acquire_s: f64,
    /// Pan servo reach (+- deg) and flip-over hysteresis.
    pub pan_limit_deg: f64,
    pub flip_hysteresis_deg: f64,
    /// Elevation held by the tilt servo (deg above the horizon).
    pub elevation_deg: f64,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        TrackerConfig {
            monopulse: MonopulseConfig::default(),
            period_s: 0.02,
            deadband_db: 1.0,
            kp: 2.5,
            ki: 0.5,
            max_rate_deg_s: 90.0,
            max_error_deg: 40.0,
            cn_full_db: 15.0,
            cn_low_db: 6.0,
            low_gain: 0.3,
            lost_cn_db: 4.0,
            lost_s: 0.5,
            hold_s: 2.0,
            search_rate_deg_s: 30.0,
            search_step_deg: 30.0,
            search_max_deg: 180.0,
            acquire_cn_db: 8.0,
            acquire_s: 0.2,
            pan_limit_deg: 90.0,
            flip_hysteresis_deg: 5.0,
            elevation_deg: 5.0,
        }
    }
}

/// One line-rate (or faster) measurement from the diversity pair.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PairSample {
    /// Gain-normalised monopulse error (A minus B), dB.
    pub error_db: f64,
    pub cn_a_db: f32,
    pub cn_b_db: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TrackState {
    Track,
    Hold,
    Search,
}

/// Servo command for the head.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeadCommand {
    pub pan_deg: f64,
    pub tilt_deg: f64,
    /// Azimuth the head points at (relative to the mount's forward).
    pub azimuth_deg: f64,
    pub flipped: bool,
    pub state: TrackState,
}

/// Wraps an angle into (-180, 180].
pub fn wrap180(a: f64) -> f64 {
    let x = (a + 180.0).rem_euclid(360.0) - 180.0;
    if x == -180.0 {
        180.0
    } else {
        x
    }
}

#[derive(Clone, Debug)]
pub struct Tracker {
    cfg: TrackerConfig,
    az: f64,
    integ: f64,
    flipped: bool,
    state: TrackState,
    acc: (f64, f32, usize),
    last_step_ns: u64,
    lost_since: Option<u64>,
    good_since: Option<u64>,
    hold_since: u64,
    last_good_az: f64,
    search_amp: f64,
    search_dir: f64,
}

impl Tracker {
    pub fn new(cfg: TrackerConfig, initial_azimuth: f64) -> Tracker {
        Tracker {
            cfg,
            az: initial_azimuth,
            integ: 0.0,
            flipped: false,
            state: TrackState::Track,
            acc: (0.0, f32::NEG_INFINITY, 0),
            last_step_ns: 0,
            lost_since: None,
            good_since: None,
            hold_since: 0,
            last_good_az: initial_azimuth,
            search_amp: 0.0,
            search_dir: 1.0,
        }
    }

    pub fn state(&self) -> TrackState {
        self.state
    }

    pub fn azimuth(&self) -> f64 {
        self.az
    }

    /// Accumulates one measurement (call at line or field rate).
    pub fn push(&mut self, s: PairSample) {
        if s.error_db.is_finite() {
            self.acc.0 += s.error_db;
            self.acc.2 += 1;
        }
        self.acc.1 = self.acc.1.max(s.cn_a_db.max(s.cn_b_db));
    }

    /// Runs the loop when a control period has passed; returns the new
    /// head command then.
    pub fn step(&mut self, now_ns: u64) -> Option<HeadCommand> {
        if self.last_step_ns == 0 {
            self.last_step_ns = now_ns;
            return Some(self.command());
        }
        let dt = (now_ns.saturating_sub(self.last_step_ns)) as f64 * 1e-9;
        if dt < self.cfg.period_s {
            return None;
        }
        self.last_step_ns = now_ns;
        let (sum, cn, n) = std::mem::replace(&mut self.acc, (0.0, f32::NEG_INFINITY, 0));
        let err_db = if n > 0 { Some(sum / n as f64) } else { None };
        let have_signal = n > 0 && cn >= self.cfg.lost_cn_db;
        // Lost / acquire bookkeeping.
        if have_signal {
            self.lost_since = None;
        } else {
            self.lost_since.get_or_insert(now_ns);
        }
        if n > 0 && cn >= self.cfg.acquire_cn_db {
            self.good_since.get_or_insert(now_ns);
        } else {
            self.good_since = None;
        }
        match self.state {
            TrackState::Track => {
                if self
                    .lost_since
                    .is_some_and(|t| (now_ns - t) as f64 * 1e-9 >= self.cfg.lost_s)
                {
                    self.state = TrackState::Hold;
                    self.hold_since = now_ns;
                    self.integ = 0.0;
                    log::info!("tracker: signal lost, holding at {:.1} deg", self.az);
                } else if let (true, Some(e)) = (have_signal, err_db) {
                    self.track(e, cn, dt);
                    self.last_good_az = self.az;
                }
            }
            TrackState::Hold | TrackState::Search => {
                let acquired = self
                    .good_since
                    .is_some_and(|t| (now_ns - t) as f64 * 1e-9 >= self.cfg.acquire_s);
                if acquired {
                    log::info!("tracker: signal back at {:.1} deg", self.az);
                    self.state = TrackState::Track;
                    self.integ = 0.0;
                } else if self.state == TrackState::Hold {
                    if (now_ns - self.hold_since) as f64 * 1e-9 >= self.cfg.hold_s {
                        self.state = TrackState::Search;
                        self.search_amp = self.cfg.search_step_deg;
                        self.search_dir = 1.0;
                    }
                } else {
                    self.search(dt);
                }
            }
        }
        Some(self.command())
    }

    fn track(&mut self, err_db: f64, cn: f32, dt: f64) {
        let slope = self.cfg.monopulse.slope_db_per_deg();
        let mut theta = if err_db.abs() < self.cfg.deadband_db {
            0.0
        } else {
            err_db / slope
        };
        theta = theta.clamp(-self.cfg.max_error_deg, self.cfg.max_error_deg);
        // Flipped over the top, patch A points the other way.
        if self.flipped {
            theta = -theta;
        }
        let f = ((cn - self.cfg.cn_low_db) / (self.cfg.cn_full_db - self.cfg.cn_low_db)).clamp(0.0, 1.0) as f64;
        let k = self.cfg.low_gain + (1.0 - self.cfg.low_gain) * f;
        if theta != 0.0 {
            self.integ += theta * dt;
        }
        // Anti-windup: the integral alone may not exceed the slew limit.
        let imax = self.cfg.max_rate_deg_s / (self.cfg.ki * k).max(1e-9);
        self.integ = self.integ.clamp(-imax, imax);
        let rate = (k * (self.cfg.kp * theta + self.cfg.ki * self.integ))
            .clamp(-self.cfg.max_rate_deg_s, self.cfg.max_rate_deg_s);
        self.az += rate * dt;
    }

    fn search(&mut self, dt: f64) {
        let target = self.last_good_az + self.search_dir * self.search_amp;
        let step = self.cfg.search_rate_deg_s * dt;
        let d = target - self.az;
        if d.abs() <= step {
            self.az = target;
            self.search_dir = -self.search_dir;
            if self.search_dir > 0.0 {
                self.search_amp = (self.search_amp + self.cfg.search_step_deg).min(self.cfg.search_max_deg);
            }
        } else {
            self.az += step * d.signum();
        }
    }

    /// Maps the azimuth to servo angles with flip-over hysteresis.
    fn command(&mut self) -> HeadCommand {
        let a = wrap180(self.az);
        let lim = self.cfg.pan_limit_deg;
        let h = self.cfg.flip_hysteresis_deg;
        if !self.flipped && a.abs() > lim + h {
            self.flipped = true;
        } else if self.flipped && a.abs() < lim - h {
            self.flipped = false;
        }
        let (pan, tilt) = if self.flipped {
            (wrap180(a - 180.0), 180.0 - self.cfg.elevation_deg)
        } else {
            (a, self.cfg.elevation_deg)
        };
        HeadCommand {
            pan_deg: pan.clamp(-lim, lim),
            tilt_deg: tilt,
            azimuth_deg: a,
            flipped: self.flipped,
            state: self.state,
        }
    }
}

/// Pan/tilt head on a Maestro.
pub struct PanTiltHead {
    pub maestro: Maestro,
    pub pan: ServoCal,
    /// Tilt servo: 0 deg = horizon forward, 180 = horizon behind (its
    /// calibration maps that range to pulses).
    pub tilt: ServoCal,
}

impl PanTiltHead {
    /// Sets speed and acceleration limits (Maestro units) on both servos.
    pub fn configure(&mut self, speed: u16, accel: u8) -> Result<()> {
        for ch in [self.pan.channel, self.tilt.channel] {
            self.maestro.set_speed(ch, speed)?;
            self.maestro.set_acceleration(ch, accel)?;
        }
        Ok(())
    }

    pub fn apply(&mut self, c: &HeadCommand) -> Result<()> {
        let (pan, tilt) = (self.pan.clone(), self.tilt.clone());
        self.maestro.set_angle(&pan, c.pan_deg)?;
        self.maestro.set_angle(&tilt, c.tilt_deg)
    }

    /// Current (pan, tilt) in degrees from the servo positions.
    pub fn angles(&mut self) -> Result<(f64, f64)> {
        let (pan, tilt) = (self.pan.clone(), self.tilt.clone());
        Ok((self.maestro.get_angle(&pan)?, self.maestro.get_angle(&tilt)?))
    }
}

/// Physical azimuth of a head from its servo angles.
pub fn head_azimuth(pan_deg: f64, tilt_deg: f64) -> (f64, bool) {
    if tilt_deg > 90.0 {
        (wrap180(pan_deg + 180.0), true)
    } else {
        (wrap180(pan_deg), false)
    }
}

#[cfg(test)]
mod tests {
    use super::maestro::sim::SimMaestro;
    use super::*;
    use crate::calib::probe::testsig::Rng;

    fn tilt_cal() -> ServoCal {
        // 0..180 deg over 500..2500 us.
        ServoCal {
            channel: 1,
            center_us: 500.0,
            us_per_deg: 2000.0 / 180.0,
            min_us: 500.0,
            max_us: 2500.0,
        }
    }

    struct World {
        sim: SimMaestro,
        head: PanTiltHead,
        tracker: Tracker,
        rng: Rng,
        mono: MonopulseConfig,
    }

    impl World {
        fn new(start_az: f64) -> World {
            let sim = SimMaestro::new(600.0);
            let head = PanTiltHead {
                maestro: Maestro::new(sim.link()),
                pan: ServoCal::servo180(0),
                tilt: tilt_cal(),
            };
            World {
                sim,
                head,
                tracker: Tracker::new(TrackerConfig::default(), start_az),
                rng: Rng(42),
                mono: MonopulseConfig::default(),
            }
        }

        /// Runs `seconds` with the quad at `quad_az(t)` and C/N `cn(t)`;
        /// returns (t, pointing error) samples.
        fn run(
            &mut self,
            t0: f64,
            seconds: f64,
            quad_az: impl Fn(f64) -> f64,
            cn: impl Fn(f64) -> f32,
        ) -> Vec<(f64, f64)> {
            let dt = 0.005;
            let mut out = Vec::new();
            let mut t = t0;
            while t < t0 + seconds {
                let (pan, tilt) = self.head.angles().unwrap();
                let (az, flipped) = head_azimuth(pan, tilt);
                let s = if flipped {
                    -self.mono.squint_deg
                } else {
                    self.mono.squint_deg
                };
                let q = quad_az(t);
                let ga = self.mono.pattern_db(wrap180(q - (az + s)));
                let gb = self.mono.pattern_db(wrap180(q - (az - s)));
                let c = cn(t);
                let noise = 0.8 * self.rng.gauss() as f64;
                let (ca, cb) = if c > 0.0 {
                    (c + ga as f32, c + gb as f32)
                } else {
                    (-10.0, -10.0)
                };
                let err = if c > 0.0 { ga - gb + noise } else { 3.0 * noise };
                self.tracker.push(PairSample {
                    error_db: err,
                    cn_a_db: ca,
                    cn_b_db: cb,
                });
                if let Some(cmd) = self.tracker.step((t * 1e9) as u64 + 1) {
                    self.head.apply(&cmd).unwrap();
                }
                self.sim.advance(dt);
                out.push((t, wrap180(q - az)));
                t += dt;
            }
            out
        }
    }

    #[test]
    fn slope_and_error() {
        let m = MonopulseConfig::default();
        let s = m.slope_db_per_deg();
        let e = m.pattern_db(5.0 - 20.0) - m.pattern_db(5.0 + 20.0);
        assert!((e / 5.0 - s).abs() < 1e-9, "Gaussian beams are exactly linear: {e} {s}");
        let a = BoardGain {
            table: GainTable::linear(78, 0.5),
            offset_db: 1.0,
        };
        let b = BoardGain {
            table: GainTable::linear(78, 0.5),
            offset_db: 0.0,
        };
        assert!((monopulse_error_db(-40.0, 60, &a, -45.0, 50, &b) + 1.0).abs() < 1e-9);
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-180.0), 180.0);
    }

    #[test]
    fn converges_on_a_moving_quad() {
        let mut w = World::new(0.0);
        // Quad starts 35 deg off and then orbits slowly (+-40 deg, 30 s).
        let trace = w.run(
            0.0,
            30.0,
            |t| 35.0 + 40.0 * (std::f64::consts::TAU * t / 30.0).sin(),
            |_| 20.0,
        );
        let late: Vec<f64> = trace.iter().filter(|(t, _)| *t > 5.0).map(|(_, e)| e.abs()).collect();
        let worst = late.iter().cloned().fold(0.0, f64::max);
        let mean = late.iter().sum::<f64>() / late.len() as f64;
        assert!(mean < 4.0 && worst < 10.0, "mean {mean:.2} worst {worst:.2}");
        let first = trace.iter().find(|(_, e)| e.abs() < 5.0).unwrap().0;
        assert!(first < 3.0, "acquired after {first:.2} s");
    }

    #[test]
    fn flips_over_when_the_quad_goes_behind() {
        let mut w = World::new(60.0);
        // Quad sweeps from 60 to 170 deg (behind the mount) at 10 deg/s.
        let trace = w.run(0.0, 16.0, |t| (60.0 + 10.0 * t).min(170.0), |_| 20.0);
        let (pan, tilt) = w.head.angles().unwrap();
        assert!(tilt > 90.0, "head did not flip: pan {pan:.1} tilt {tilt:.1}");
        let end = trace.last().unwrap().1.abs();
        assert!(end < 6.0, "pointing error after the flip {end:.1}");
    }

    #[test]
    fn holds_then_searches_then_reacquires() {
        let mut w = World::new(20.0);
        w.run(0.0, 5.0, |_| 20.0, |_| 20.0);
        // Signal gone for 6 s: hold, then search.
        w.run(5.0, 0.8, |_| 20.0, |_| 0.0);
        assert_eq!(w.tracker.state(), TrackState::Hold);
        w.run(5.8, 3.0, |_| 20.0, |_| 0.0);
        assert_eq!(w.tracker.state(), TrackState::Search);
        // The quad comes back 50 deg away; the search finds it.
        let trace = w.run(8.8, 25.0, |_| 70.0, |_| 20.0);
        assert_eq!(w.tracker.state(), TrackState::Track);
        assert!(trace.last().unwrap().1.abs() < 5.0, "{:?}", trace.last());
    }
}
