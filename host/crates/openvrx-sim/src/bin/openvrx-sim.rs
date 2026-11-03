// SPDX-License-Identifier: GPL-3.0-or-later
//! openvrx-sim: writes a simulated `.openvrx` recording with truth.
//!
//! Examples:
//!
//! ```text
//! openvrx-sim -o out/edge --standard pal --preset 7x7 --divider 18 --seconds 5 \
//!     --cn 8 --picture scene --rx 2 --fading rayleigh --doppler 30 --scenario flyout
//! openvrx-sim -o out/x --config my.json --seconds 2
//! openvrx-sim --dump-config --sim-preset diversity > d.json
//! ```

use clap::Parser;
use openvrx_core::{LanePreset, VideoStandard};
use openvrx_sim::*;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "openvrx-sim", version, about = "Generate simulated OpenVRX recordings (CVBS -> FM -> C5 -> lanes) with truth")]
struct Cli {
    /// Output base path (dir/NAME): writes NAME.openvrx.json, NAME.rxK.bin, ...
    #[arg(short, long)]
    out: Option<PathBuf>,
    /// Start from a JSON SimConfig file (options below override it).
    #[arg(long)]
    config: Option<PathBuf>,
    /// Start from a named preset (clean|edge|flyout|diversity|multipath|demo|glitch).
    #[arg(long)]
    sim_preset: Option<String>,
    /// Print the resulting configuration as JSON and exit.
    #[arg(long)]
    dump_config: bool,
    #[arg(long)]
    seed: Option<u64>,
    #[arg(long)]
    seconds: Option<f64>,
    /// ntsc | pal
    #[arg(long)]
    standard: Option<VideoStandard>,
    /// Lane preset for every receiver (8x8, 7x7, 6x6, 5x5, 4x4top, 4x4fine, 4x4ultra, angle8).
    #[arg(long)]
    preset: Option<LanePreset>,
    /// PARLIO divider of the 240 MHz PLL (6, 9, 12, 18 keep the bus locked).
    #[arg(long)]
    divider: Option<u32>,
    /// Mean C/N in dB per 20 MHz channel.
    #[arg(long)]
    cn: Option<f64>,
    /// bars | ramp | multiburst | zoneplate | scene | black | flat:Y | image:PATH | video:PATH
    #[arg(long)]
    picture: Option<PictureConfig>,
    /// Draw the FPV OSD overlay.
    #[arg(long)]
    osd: bool,
    /// Number of receivers (copies rx0's settings).
    #[arg(long)]
    rx: Option<usize>,
    /// none | rayleigh | rician
    #[arg(long)]
    fading: Option<FadingKind>,
    /// Maximum Doppler frequency, Hz.
    #[arg(long)]
    doppler: Option<f64>,
    /// Rician K factor, dB.
    #[arg(long)]
    k_db: Option<f64>,
    /// constant | flyout | orbit
    #[arg(long)]
    scenario: Option<Scenario>,
    /// Two-ray multipath as DELAY_NS:REL_DB (e.g. 150:-6).
    #[arg(long)]
    multipath: Option<String>,
    /// VTX deviation scale (1.0 = 6.7 MHz sync to white).
    #[arg(long)]
    deviation: Option<f64>,
    /// VTX carrier error, kHz.
    #[arg(long)]
    carrier_khz: Option<f64>,
    /// Enable VTX pre-emphasis (default network).
    #[arg(long)]
    pre_emphasis: bool,
    /// 802.11p narrow channel filter on every receiver.
    #[arg(long)]
    p11: bool,
    /// Fixed gain index (disables the gain loop).
    #[arg(long)]
    gain: Option<u32>,
    /// Sampling-phase lottery: random | clean | mixed
    #[arg(long)]
    lottery: Option<Lottery>,
    /// Receiver crystal error, ppm (all receivers).
    #[arg(long)]
    ppm: Option<f64>,
    /// USB loss events per second.
    #[arg(long)]
    usb_loss: Option<f64>,
    /// Camera noise, IRE RMS.
    #[arg(long)]
    camera_noise: Option<f32>,
    /// Do not write the truth file.
    #[arg(long)]
    no_truth: bool,
    /// Replace an existing recording.
    #[arg(long)]
    overwrite: bool,
    /// Free-text notes for the sidecar.
    #[arg(long)]
    notes: Option<String>,
}

fn build(cli: &Cli) -> Result<SimConfig, String> {
    let mut c = match (&cli.config, &cli.sim_preset) {
        (Some(p), _) => {
            let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
            serde_json::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))?
        }
        (None, Some(name)) => SimConfig::preset(name).map_err(|e| e.to_string())?,
        (None, None) => SimConfig::default(),
    };
    if let Some(n) = cli.rx {
        c.set_receiver_count(n);
    }
    if let Some(v) = cli.seed {
        c.seed = v;
    }
    if let Some(v) = cli.seconds {
        c.seconds = v;
    }
    if let Some(v) = cli.standard {
        c.standard = v;
    }
    if let Some(v) = &cli.picture {
        c.picture = v.clone();
    }
    if cli.osd {
        c.osd = true;
    }
    if let Some(v) = cli.cn {
        c.channel.cn_db = v;
    }
    if let Some(v) = cli.fading {
        c.channel.fading.kind = v;
    }
    if let Some(v) = cli.doppler {
        c.channel.fading.doppler_hz = v;
    }
    if let Some(v) = cli.k_db {
        c.channel.fading.k_db = v;
    }
    if let Some(v) = &cli.scenario {
        c.channel.scenario = v.clone();
    }
    if let Some(m) = &cli.multipath {
        let (d, r) = m
            .split_once(':')
            .ok_or_else(|| format!("--multipath wants DELAY_NS:REL_DB, got `{m}`"))?;
        c.channel.multipath = Some(Multipath {
            delay_ns: d.parse().map_err(|_| format!("bad delay `{d}`"))?,
            rel_db: r.parse().map_err(|_| format!("bad level `{r}`"))?,
            ..Default::default()
        });
    }
    if let Some(v) = cli.deviation {
        c.vtx.deviation = v;
    }
    if let Some(v) = cli.carrier_khz {
        c.vtx.freq_error_hz = v * 1e3;
    }
    if cli.pre_emphasis {
        c.vtx.pre_emphasis = Some(PreEmphasis::default());
    }
    if let Some(v) = cli.camera_noise {
        c.cvbs.camera_noise_ire = v;
    }
    for r in &mut c.receivers {
        if let Some(v) = cli.preset {
            r.preset = v;
        }
        if let Some(v) = cli.divider {
            r.divider = v;
        }
        if cli.p11 {
            r.filter = ChannelFilter::P11;
        }
        if let Some(g) = cli.gain {
            r.agc = Agc::Fixed { gain: g };
        }
        if let Some(l) = cli.lottery {
            r.sampling.lottery = l;
        }
        if let Some(p) = cli.ppm {
            r.crystal_ppm = Some(p);
        }
        if let Some(u) = cli.usb_loss {
            r.usb_loss.rate_per_s = u;
        }
    }
    if cli.no_truth {
        c.truth = false;
    }
    if let Some(n) = &cli.notes {
        c.notes = n.clone();
    }
    c.validate().map_err(|e| e.to_string())?;
    Ok(c)
}

fn progress(t: f64, total: f64) {
    eprint!("\r  {:6.2} / {:.2} s", t.min(total), total);
}

fn main() {
    let cli = Cli::parse();
    let cfg = match build(&cli) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("openvrx-sim: {e}");
            std::process::exit(2);
        }
    };
    if cli.dump_config {
        println!("{}", serde_json::to_string_pretty(&cfg).expect("config serializes"));
        return;
    }
    let Some(out) = &cli.out else {
        eprintln!("openvrx-sim: --out is required (or use --dump-config)");
        std::process::exit(2);
    };
    let opts = GenerateOptions {
        overwrite: cli.overwrite,
        progress: Some(progress),
    };
    match generate_with(&cfg, out, &opts) {
        Ok(s) => {
            eprintln!();
            let mb: f64 = s.bytes.iter().sum::<u64>() as f64 / 1e6;
            println!(
                "{}: {:.2} s, {} receiver(s), {:.1} MB, {} truth samples, {} events, {:.2} s wall ({:.2}x real time)",
                out.display(),
                s.seconds,
                s.bytes.len(),
                mb,
                s.truth_samples,
                s.events,
                s.elapsed_s,
                s.realtime_factor
            );
        }
        Err(e) => {
            eprintln!();
            eprintln!("openvrx-sim: {e}");
            std::process::exit(1);
        }
    }
}
