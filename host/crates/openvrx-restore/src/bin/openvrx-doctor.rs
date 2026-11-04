// SPDX-License-Identifier: GPL-3.0-or-later
//! openvrx-doctor: restore an analog FPV DVR recording.
//!
//! ```text
//! openvrx-doctor input.mp4 -o output.mp4 [--preset max] [--radius 3]
//!     [--crf 16] [--x264-preset medium] [--denoise 1] [--damage auto]
//!     [--start 12.5 --duration 30] [--config cfg.json] [--print-config]
//! ```

use clap::Parser;
use indicatif::{ProgressBar, ProgressStyle};
use openvrx_restore::{ffmpeg, DamageSource, RestoreConfig};
use std::path::PathBuf;
use std::process::ExitCode;

#[derive(Parser, Debug)]
#[command(name = "openvrx-doctor", version, about = "Rebuild breakup in analog FPV DVR video from the frames around it")]
struct Args {
    /// Input video file.
    input: PathBuf,
    /// Output video file (x264; audio is kept).
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// Preset: fast, balanced or max.
    #[arg(long, default_value = "balanced")]
    preset: String,
    /// JSON configuration file (overrides the preset).
    #[arg(long)]
    config: Option<PathBuf>,
    /// Neighbour frames used on each side.
    #[arg(long)]
    radius: Option<usize>,
    /// Temporal denoise of undamaged pixels (0 off, 1 nominal).
    #[arg(long)]
    denoise: Option<f32>,
    /// Damage source: auto, mask, blind or mask-and-blind.
    #[arg(long)]
    damage: Option<DamageSource>,
    /// Blind damage detection sensitivity (1 nominal).
    #[arg(long)]
    sensitivity: Option<f32>,
    /// x264 CRF.
    #[arg(long)]
    crf: Option<u8>,
    /// x264 preset.
    #[arg(long)]
    x264_preset: Option<String>,
    /// Start time in seconds.
    #[arg(long)]
    start: Option<f64>,
    /// Duration in seconds.
    #[arg(long)]
    duration: Option<f64>,
    /// Print the effective configuration as JSON and exit.
    #[arg(long)]
    print_config: bool,
}

fn build_config(a: &Args) -> Result<RestoreConfig, String> {
    let mut cfg = match &a.config {
        Some(p) => {
            let s = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
            serde_json::from_str(&s).map_err(|e| format!("{}: {e}", p.display()))?
        }
        None => RestoreConfig::preset(&a.preset).map_err(|e| e.to_string())?,
    };
    if let Some(r) = a.radius {
        cfg.radius = r;
    }
    if let Some(d) = a.denoise {
        cfg.denoise = d;
    }
    if let Some(d) = a.damage {
        cfg.damage_source = d;
    }
    if let Some(s) = a.sensitivity {
        cfg.blind_sensitivity = s;
    }
    if let Some(c) = a.crf {
        cfg.encode.crf = c;
    }
    if let Some(p) = &a.x264_preset {
        cfg.encode.preset = p.clone();
    }
    if a.start.is_some() {
        cfg.encode.start = a.start;
    }
    if a.duration.is_some() {
        cfg.encode.duration = a.duration;
    }
    cfg.validate().map_err(|e| e.to_string())?;
    Ok(cfg)
}

fn main() -> ExitCode {
    let args = Args::parse();
    let cfg = match build_config(&args) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("openvrx-doctor: {e}");
            return ExitCode::from(2);
        }
    };
    if args.print_config {
        println!("{}", serde_json::to_string_pretty(&cfg).expect("config serialises"));
        return ExitCode::SUCCESS;
    }
    let Some(output) = args.output.clone() else {
        eprintln!("openvrx-doctor: an output file is required (-o)");
        return ExitCode::from(2);
    };
    let bar = ProgressBar::new(0);
    bar.set_style(
        ProgressStyle::with_template(
            "{spinner} [{elapsed_precise}] {wide_bar} {pos}/{len} frames  {msg}  ETA {eta}",
        )
        .expect("valid template"),
    );
    let result = ffmpeg::restore_file(&args.input, &output, &cfg, |p| {
        if let Some(t) = p.total {
            if bar.length() != Some(t.max(p.done) as u64) {
                bar.set_length(t.max(p.done) as u64);
            }
        }
        bar.set_position(p.done as u64);
        bar.set_message(format!("{:.1} fps", p.fps));
    });
    bar.finish_and_clear();
    match result {
        Ok(s) => {
            let broken = s.run.stats.iter().filter(|st| st.interpolated).count();
            let flagged: f32 = s.run.stats.iter().map(|st| st.flagged).sum::<f32>()
                / s.run.stats.len().max(1) as f32;
            println!(
                "{} frames in {:.1} s ({:.1} fps): {} whole frames rebuilt, {:.2}% of pixels flagged -> {}",
                s.run.frames,
                s.run.seconds,
                s.run.fps(),
                broken,
                100.0 * flagged,
                output.display()
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("openvrx-doctor: {e}");
            ExitCode::FAILURE
        }
    }
}
