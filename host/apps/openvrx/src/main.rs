// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX live receiver app.

mod app;
mod backend;
mod channels;
mod cli;
mod commands;
mod gpu;
mod keys;
mod layout;
mod live_filter;
mod osd;
mod panels;
mod present;
mod settings;
mod source;
mod stats;

use clap::Parser;
use cli::{Args, SourceKind};
use std::process::ExitCode;

/// Minimal stderr logger: info and above from this workspace, warnings
/// from dependencies.
struct Logger;

impl log::Log for Logger {
    fn enabled(&self, m: &log::Metadata<'_>) -> bool {
        let ours = m.target().starts_with("openvrx");
        m.level() <= if ours { log::Level::Info } else { log::Level::Warn }
    }
    fn log(&self, r: &log::Record<'_>) {
        if self.enabled(r.metadata()) {
            eprintln!("[{} {}] {}", r.level(), r.target(), r.args());
        }
    }
    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

fn snapshot(path: &std::path::Path, s: &settings::AppSettings) -> Result<(), String> {
    use layout::{fit_picture, pixel_aspect, Rect};
    let (w, h) = (1280u32, 720u32);
    let (pattern, standard) = match &s.views[0].source {
        settings::ViewSource::Test { pattern, standard } => (*pattern, *standard),
        settings::ViewSource::Receivers { .. } => {
            (settings::TestPattern::Mix, openvrx_core::VideoStandard::Ntsc)
        }
    };
    let frame = source::test_pattern::render_frame(pattern, standard, 1.0);
    let ex = source::FrameExchange::new();
    ex.producer().publish_frame(&frame, 1);
    let mut st = source::UploadStaging::default();
    ex.take(&mut st).ok_or("no frame")?;
    let eff = s.display.effective();
    let par = pixel_aspect(eff.output_aspect, w as f32, h as f32);
    let geom = fit_picture(
        Rect::new(0.0, 0.0, w as f32, h as f32),
        4.0 / 3.0,
        par,
        eff.aspect,
        s.display.overscan_percent,
    );
    let rgba = gpu::render_offscreen(&st, &geom, &s.display, w, h)?;
    gpu::write_png(path, w, h, &rgba)
}

fn main() -> ExitCode {
    let _ = log::set_logger(&LOGGER).map(|()| log::set_max_level(log::LevelFilter::Info));
    let args = Args::parse();
    let path = args
        .settings
        .clone()
        .unwrap_or_else(settings::default_settings_path);
    let loaded = settings::load(&path);
    if let Some(n) = &loaded.note {
        log::warn!("{n}");
    }
    let mut s = loaded.settings;
    args.apply(&mut s);

    if let Some(png) = &args.snapshot {
        return match snapshot(png, &s) {
            Ok(()) => {
                println!("wrote {}", png.display());
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("snapshot failed: {e}");
                ExitCode::FAILURE
            }
        };
    }

    let store = settings::SettingsStore::new(path, &s);
    let d = &s.display;
    let mut options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("OpenVRX")
            .with_inner_size([1280.0, 800.0])
            .with_fullscreen(d.effective().fullscreen),
        ..Default::default()
    };
    options.wgpu_options.surface = egui_wgpu::SurfaceConfig {
        present_mode: wgpu::PresentMode::Fifo,
        desired_maximum_frame_latency: Some(d.max_frame_latency),
    };
    if let egui_wgpu::WgpuSetup::CreateNew(c) = &mut options.wgpu_options.wgpu_setup {
        // DX12 presents through the DXGI flip model (Mailbox and tearing
        // Immediate both available); WGPU_BACKEND still overrides.
        if cfg!(windows) && wgpu::Backends::from_env().is_none() {
            c.instance_descriptor.backends = wgpu::Backends::DX12;
        }
        c.power_preference = match d.gpu {
            settings::GpuPreference::LowPower => wgpu::PowerPreference::LowPower,
            settings::GpuPreference::HighPerformance => wgpu::PowerPreference::HighPerformance,
        };
    }
    let source = args.source;
    let run_for = args.run_for;
    let note = loaded.note;
    let compare_pattern = settings::TestPattern::bench_on(&args.compare_pattern, &args.compare_out)
        .unwrap_or(settings::TestPattern::BenchColourCard);
    let compare = args.compare;
    let bench_control = args.bench_control;
    let result = eframe::run_native(
        "OpenVRX",
        options,
        Box::new(move |cc| {
            let start = move |waker| match source {
                SourceKind::Test => backend::BackendHandle::spawn("test", waker, backend::test::run),
                SourceKind::Live => backend::BackendHandle::spawn("live", waker, backend::live::run),
            };
            let mut app = app::OpenvrxApp::new(cc, s, store, start, run_for, note)?;
            app.compare_pattern = compare_pattern;
            if compare {
                app.toggle_compare();
            }
            if bench_control {
                app.start_bench_control(cc.egui_ctx.clone());
            }
            Ok(Box::new(app))
        }),
    );
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("openvrx: {e}");
            ExitCode::FAILURE
        }
    }
}
