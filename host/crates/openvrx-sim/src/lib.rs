// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX simulator: picture -> CVBS -> FM VTX -> radio channel -> ESP32-C5
//! front end and ADC -> PARLIO sample clock -> FX2 lanes, written as
//! `.openvrx` recordings with truth, or streamed live as `RawBlock`s.
//!
//! The receive-chain model is adapted from C5VRX research/sim.py by Twotoz
//! and the C5VRX contributors (https://github.com/Twotoz/C5VRX, GPL-3.0,
//! commit 6bc9e4b): FM at the 80 MS/s MODEM_DIAG rate with the blanking
//! level at a porch offset and the 6.7 MHz sync-to-white deviation scale,
//! the C5 analog channel as a 5th-order Butterworth at 10 MHz, thermal
//! noise at a C/N measured in that channel (per 20 MHz), and unfiltered
//! decimation by the PARLIO clock. What changed: rewritten in Rust as a
//! streaming, parallel, seeded generator; a full NTSC-M / PAL-B/G encoder
//! (equalising and broad pulses, interlace, burst, QAM chroma with the PAL
//! V switch, setup, band-limited luma) replaces the textured raster;
//! picture sources (bars, ramps, multiburst, zone plate, moving scene,
//! images, video via ffmpeg, OSD); VTX carrier error, drift, pre-emphasis
//! and modulator nonlinearity; time-varying C/N scenarios, Rayleigh and
//! Rician fading, two-ray multipath and antenna patterns per receiver; LO
//! and sample clock errors per receiver crystal; the 802.11p filter
//! option; gain-dependent I DC offset, 10-bit quantisation and clipping
//! with a gain loop; the sampling-phase lottery with per-lane skew; clock
//! slips and divider changes; USB loss; lane encoding for every preset;
//! multi-receiver diversity scenarios; events and truth output.
//!
//! Entry points: [`generate`] (write a recording), [`Engine`] (chunked
//! generation) and [`SimSource`] (real-time `RawBlock` stream for demo
//! mode).

pub mod channel;
pub mod config;
pub mod cvbs;
pub mod dsp;
pub mod engine;
pub mod font;
pub mod picture;
pub mod rx;
pub mod source;
pub mod tx;

pub use config::{
    Action, Agc, AntennaPattern, ChannelConfig, ChannelFilter, CvbsConfig, Fading, FadingKind, Lottery,
    Multipath, PictureConfig, PreEmphasis, RxConfig, SamplingConfig, Scenario, ScheduledEvent, SimConfig,
    UsbLoss, VtxConfig,
};
pub use engine::{Chunk, Engine};
pub use source::{SimSource, SimSourceOptions};

use openvrx_format::{RecordingWriter, WriterOptions};
use std::path::Path;
use std::time::Instant;

/// Simulator errors.
#[derive(Debug, thiserror::Error)]
pub enum SimError {
    #[error("configuration: {0}")]
    Config(String),
    #[error("picture source: {0}")]
    Picture(String),
    #[error(transparent)]
    Format(#[from] openvrx_format::FormatError),
}

/// What [`generate`] produced.
#[derive(Clone, Debug)]
pub struct GenerateStats {
    pub seconds: f64,
    pub bytes: Vec<u64>,
    pub truth_samples: u64,
    pub events: usize,
    pub elapsed_s: f64,
    /// Generated signal time per wall-clock time.
    pub realtime_factor: f64,
}

/// Generation options.
#[derive(Clone, Debug, Default)]
pub struct GenerateOptions {
    /// Replace an existing recording of the same name.
    pub overwrite: bool,
    /// Called after every chunk with the stream time reached.
    pub progress: Option<fn(f64, f64)>,
}

/// Writes a simulated recording at `out_base` (`dir/NAME`).
pub fn generate(cfg: &SimConfig, out_base: impl AsRef<Path>) -> Result<GenerateStats, SimError> {
    generate_with(cfg, out_base, &GenerateOptions::default())
}

/// [`generate`] with options.
pub fn generate_with(cfg: &SimConfig, out_base: impl AsRef<Path>, opts: &GenerateOptions) -> Result<GenerateStats, SimError> {
    let start = Instant::now();
    let mut engine = Engine::new(cfg)?;
    let sidecar = engine.sidecar();
    let mut w = RecordingWriter::create(
        out_base.as_ref(),
        sidecar,
        WriterOptions {
            overwrite: opts.overwrite,
            ..Default::default()
        },
    )?;
    // Disk writes overlap generation: a writer thread drains finished
    // chunks while the engine computes the next ones.
    let (tx, rx) = std::sync::mpsc::sync_channel::<Chunk>(2);
    let n_rx = cfg.receivers.len();
    let want_truth = cfg.truth;
    let writer = std::thread::Builder::new()
        .name("openvrx-sim-writer".into())
        .spawn(move || -> Result<(RecordingWriter, usize), SimError> {
            let mut events = 0;
            for chunk in rx {
                for (k, bytes) in chunk.rx_bytes.iter().enumerate() {
                    w.write_raw(k as u8, bytes)?;
                }
                if want_truth {
                    w.write_truth(&chunk.truth)?;
                }
                for ev in &chunk.events {
                    w.log_event(ev)?;
                }
                events += chunk.events.len();
            }
            Ok((w, events))
        })
        .map_err(|e| SimError::Config(format!("cannot start writer thread: {e}")))?;
    let mut gen_err = None;
    loop {
        match engine.next_chunk() {
            Ok(Some(chunk)) => {
                let t = chunk.t_end;
                if tx.send(chunk).is_err() {
                    break;
                }
                if let Some(cb) = opts.progress {
                    cb(t, cfg.seconds);
                }
            }
            Ok(None) => break,
            Err(e) => {
                gen_err = Some(e);
                break;
            }
        }
    }
    drop(tx);
    let joined = writer
        .join()
        .map_err(|_| SimError::Config("writer thread panicked".into()))?;
    if let Some(e) = gen_err {
        return Err(e);
    }
    let (mut w, events) = joined?;
    if let Some(sim) = w.sidecar_mut().sim.as_mut() {
        sim.settings
            .insert("effects".into(), serde_json::Value::Array(engine.effects().to_vec()));
    }
    let bytes = (0..n_rx).map(|k| w.bytes_written(k as u8)).collect();
    let truth_samples = w.truth_samples();
    w.close()?;
    let elapsed_s = start.elapsed().as_secs_f64();
    Ok(GenerateStats {
        seconds: cfg.seconds,
        bytes,
        truth_samples,
        events,
        elapsed_s,
        realtime_factor: cfg.seconds / elapsed_s.max(1e-9),
    })
}
