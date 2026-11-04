// SPDX-License-Identifier: GPL-3.0-or-later
//! The live app: polls the view sources, uploads changed rows, draws the
//! picture through the YUV shader and the OSD, handles keys and panels,
//! and keeps the backend in step with the settings.
//!
//! Presentation: sources wake the UI thread only at field ends or every
//! N lines (slice mode), a [`PresentScheduler`] decides when the newest
//! rows go to the GPU, and the swap chain runs the chosen present mode
//! with a maximum frame latency of 1.

use crate::backend::BackendHandle;
use crate::commands::{diff, full_state, AppCommand, BackendEvent, BackendStatus, MessageLevel};
use crate::gpu::{Uniforms, VideoGpu};
use crate::keys::{action_for, key_name, Action, BINDINGS};
use crate::layout::{fit_picture, pixel_aspect, view_cells, Rect as LRect, CELL_GAP};
use crate::osd::{fmt_latency, paint_global, paint_view, ViewOsd};
use crate::present::{choose_present_mode, mode_name, PresentScheduler};
use crate::settings::{AppSettings, ControlMode, PresentTrigger, SettingsStore, VIEW_SLOTS};
use crate::source::{DirtyRows, FrameSource, SourceStatus, UploadStaging, Waker};
use crate::stats::{LatencyStats, RateMeter, Summary};
use crate::{channels, panels};
use openvrx_core::clock::now_ns;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

/// Latency samples kept for the statistics.
const LATENCY_WINDOW: usize = 2048;
/// How long a message stays on screen.
const MESSAGE_NS: u64 = 8_000_000_000;

struct ViewRt {
    source: Option<Box<dyn FrameSource>>,
    staging: UploadStaging,
    dirty: DirtyRows,
    status: SourceStatus,
    error: Option<String>,
    latency: LatencyStats,
    /// Newest line time of rows polled but not yet uploaded.
    pending_ns: u64,
    has_content: bool,
}

impl ViewRt {
    fn new() -> ViewRt {
        ViewRt {
            source: None,
            staging: UploadStaging::default(),
            dirty: DirtyRows::default(),
            status: SourceStatus::default(),
            error: None,
            latency: LatencyStats::new(LATENCY_WINDOW),
            pending_ns: 0,
            has_content: false,
        }
    }
}

pub struct OpenvrxApp {
    pub settings: AppSettings,
    applied: AppSettings,
    applied_views: usize,
    store: SettingsStore,
    backend: BackendHandle,
    backend_status: BackendStatus,
    backend_seen: u64,
    views: Vec<ViewRt>,
    gpu: VideoGpu,
    scheduler: PresentScheduler,
    latency: LatencyStats,
    frame_rate: RateMeter,
    messages: VecDeque<(u64, String)>,
    pub show_panels: bool,
    show_help: bool,
    surface_applied: Option<(wgpu::PresentMode, u32)>,
    supported_modes: Vec<wgpu::PresentMode>,
    fullscreen_applied: Option<bool>,
    zoom_applied: f32,
    wake_lines_applied: Option<u32>,
    run_until_ns: Option<u64>,
    waker: Waker,
    adapter_name: String,
    /// Uploaded views whose latency is recorded at the end of this frame.
    uploaded: Vec<(usize, u64)>,
}

impl OpenvrxApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        settings: AppSettings,
        store: SettingsStore,
        backend_start: impl FnOnce(Waker) -> std::io::Result<BackendHandle>,
        run_for_s: Option<f64>,
        startup_note: Option<String>,
    ) -> Result<OpenvrxApp, String> {
        let rs = cc
            .wgpu_render_state
            .as_ref()
            .ok_or("the wgpu renderer is required")?;
        let ctx = cc.egui_ctx.clone();
        let waker: Waker = Arc::new(move || ctx.request_repaint());
        let backend = backend_start(Arc::clone(&waker)).map_err(|e| format!("backend: {e}"))?;
        let gpu = VideoGpu::new(&rs.device, &rs.queue, rs.target_format);
        let info = rs.adapter.get_info();
        let adapter_name = format!("{} ({:?})", info.name, info.backend);
        log::info!("GPU: {adapter_name}");
        let supported_modes = probe_present_modes(cc, rs);
        log::info!("present modes: {supported_modes:?}");
        let d = &settings.display;
        let scheduler = PresentScheduler::new(d.present_trigger, d.present_lines, d.max_present_hz);
        let views = settings.display.layout.count();
        for cmd in full_state(&settings, views) {
            backend.send(cmd);
        }
        let mut messages = VecDeque::new();
        if let Some(n) = startup_note {
            messages.push_back((now_ns(), n));
        }
        Ok(OpenvrxApp {
            applied: settings.clone(),
            applied_views: views,
            settings,
            store,
            backend,
            backend_status: BackendStatus::default(),
            backend_seen: 0,
            views: (0..VIEW_SLOTS).map(|_| ViewRt::new()).collect(),
            gpu,
            scheduler,
            latency: LatencyStats::new(LATENCY_WINDOW),
            frame_rate: RateMeter::default(),
            messages,
            show_panels: false,
            show_help: false,
            surface_applied: None,
            supported_modes,
            fullscreen_applied: None,
            zoom_applied: 0.0,
            wake_lines_applied: None,
            run_until_ns: run_for_s.map(|s| now_ns() + (s * 1e9) as u64),
            waker,
            adapter_name,
            uploaded: Vec::new(),
        })
    }

    pub fn adapter_name(&self) -> &str {
        &self.adapter_name
    }

    /// Global latency summary (line to frame submission).
    pub fn latency_summary(&mut self) -> Summary {
        self.latency.summary()
    }

    pub fn backend_status(&self) -> &BackendStatus {
        &self.backend_status
    }

    pub fn view_status(&self, v: usize) -> &SourceStatus {
        &self.views[v].status
    }

    pub fn send(&self, cmd: AppCommand) {
        self.backend.send(cmd);
    }

    pub fn note(&mut self, text: impl Into<String>) {
        self.messages.push_back((now_ns(), text.into()));
        while self.messages.len() > 6 {
            self.messages.pop_front();
        }
    }

    fn wake_lines(&self) -> u32 {
        match self.settings.display.present_trigger {
            PresentTrigger::Lines => self.settings.display.present_lines,
            PresentTrigger::Field | PresentTrigger::Frame => u32::MAX,
        }
    }

    fn pump_backend(&mut self) {
        while let Some(ev) = self.backend.try_event() {
            match ev {
                BackendEvent::ViewOpened { view, mut source } => {
                    let wake_lines = self.wake_lines();
                    if let Some(v) = self.views.get_mut(view) {
                        source.set_waker(Arc::clone(&self.waker));
                        source.set_wake_lines(wake_lines);
                        log::info!("view {view}: {}", source.label());
                        v.source = Some(source);
                        v.error = None;
                        v.status = SourceStatus::default();
                    }
                }
                BackendEvent::ViewFailed { view, error } => {
                    if let Some(v) = self.views.get_mut(view) {
                        v.source = None;
                        v.has_content = false;
                        v.error = Some(error);
                        self.gpu.clear_view(view);
                    }
                }
                BackendEvent::Message { level, text } => {
                    let prefix = match level {
                        MessageLevel::Info => "",
                        MessageLevel::Warning => "warning: ",
                        MessageLevel::Error => "error: ",
                    };
                    self.note(format!("{prefix}{text}"));
                }
            }
        }
        self.backend
            .read_status(&mut self.backend_status, &mut self.backend_seen);
    }

    fn poll_sources(&mut self, now: u64) {
        let n = self.settings.display.layout.count();
        for (k, v) in self.views.iter_mut().enumerate() {
            let Some(src) = v.source.as_mut() else {
                continue;
            };
            src.status(&mut v.status);
            if k >= n {
                continue;
            }
            while let Some(info) = src.poll(&mut v.staging) {
                if v.dirty.rows() != info.height {
                    v.dirty.resize(info.height);
                }
                for r in &v.staging.ranges {
                    v.dirty.mark(r.start as usize..r.end as usize);
                }
                v.pending_ns = v.pending_ns.max(info.newest_line_ns);
                self.scheduler.on_update(&info, now);
            }
        }
    }

    fn upload_pending(&mut self) {
        self.uploaded.clear();
        for (k, v) in self.views.iter_mut().enumerate() {
            if v.dirty.is_empty() {
                continue;
            }
            v.staging.ranges.clear();
            v.dirty.drain_ranges(&mut v.staging.ranges);
            self.gpu.upload(k, &v.staging);
            v.has_content = true;
            if v.pending_ns != 0 {
                self.uploaded.push((k, v.pending_ns));
            }
            v.pending_ns = 0;
        }
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let pressed: Vec<(egui::Key, bool)> = ctx.input(|i| {
            i.events
                .iter()
                .filter_map(|e| match e {
                    egui::Event::Key {
                        key,
                        pressed: true,
                        repeat: false,
                        modifiers,
                        ..
                    } => Some((*key, modifiers.ctrl)),
                    _ => None,
                })
                .collect()
        });
        for (key, ctrl) in pressed {
            if let Some(a) = action_for(key, ctrl) {
                self.act(ctx, a);
            }
        }
    }

    /// Performs a shortcut action.
    pub fn act(&mut self, ctx: &egui::Context, a: Action) {
        let d = &mut self.settings.display;
        match a {
            Action::Help => self.show_help = !self.show_help,
            Action::Fullscreen => d.fullscreen = !d.fullscreen,
            Action::Panels => self.show_panels = !self.show_panels,
            Action::Osd => d.osd = !d.osd,
            Action::Record => self.toggle_recording(),
            Action::Cleanup => self.settings.video.cleanup = !self.settings.video.cleanup,
            Action::Hd3 => d.hd3_mode = !d.hd3_mode,
            Action::Layout => d.layout = d.layout.next(),
            Action::PresentMode => d.present_mode = d.present_mode.next(),
            Action::PresentTrigger => d.present_trigger = d.present_trigger.next(),
            Action::Aspect => {
                if d.hd3_mode {
                    d.hd3_stretch = !d.hd3_stretch;
                } else {
                    d.aspect = d.aspect.next();
                }
            }
            Action::Scaling => d.scaling = d.scaling.next(),
            Action::Damage => d.damage_overlay = !d.damage_overlay,
            Action::Calibrate => {
                for k in 0..self.settings.receivers.len() {
                    self.backend.send(AppCommand::Calibrate { rx: k as u8 });
                }
            }
            Action::ChannelDown | Action::ChannelUp
                if self.settings.receivers.first().map(|r| r.control) == Some(ControlMode::Fx2Only) =>
            {
                self.note("FX2-only mode: the channel is set on the C5 (tools/c5cmd.py ... \"tune MHZ\" save)");
            }
            Action::ChannelDown | Action::ChannelUp => {
                let up = a == Action::ChannelUp;
                if let Some(f) = self.settings.receivers.first().map(|r| r.freq_mhz) {
                    let next = channels::step(f, up);
                    for r in &mut self.settings.receivers {
                        r.freq_mhz = next;
                    }
                    self.note(format!("channel {}", channels::label(next)));
                }
            }
            Action::ResetStats => {
                self.latency.clear();
                for v in &mut self.views {
                    v.latency.clear();
                }
            }
            Action::Escape => {
                if self.show_help || self.show_panels {
                    self.show_help = false;
                    self.show_panels = false;
                } else {
                    d.fullscreen = false;
                    d.hd3_mode = false;
                }
            }
            Action::Quit => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
        }
    }

    pub fn toggle_recording(&mut self) {
        if self.backend_status.recording.active {
            self.backend.send(AppCommand::StopRecording);
        } else {
            self.backend.send(AppCommand::StartRecording {
                folder: self.settings.recording.folder.clone(),
            });
        }
    }

    /// Pushes setting changes to the backend, the swap chain and the
    /// window.
    fn apply_settings(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.settings.sanitize();
        let views = self.settings.display.layout.count();
        if self.settings != self.applied || views != self.applied_views {
            for cmd in diff(&self.applied, &self.settings, self.applied_views, views) {
                self.backend.send(cmd);
            }
            for k in views..self.applied_views {
                self.views[k].source = None;
                self.views[k].has_content = false;
                self.gpu.clear_view(k);
            }
            self.applied = self.settings.clone();
            self.applied_views = views;
            self.store.update(&self.settings);
        }
        let d = &self.settings.display;
        self.scheduler
            .configure(d.present_trigger, d.present_lines, d.max_present_hz);
        let wl = self.wake_lines();
        if self.wake_lines_applied != Some(wl) {
            for v in &mut self.views {
                if let Some(s) = v.source.as_mut() {
                    s.set_wake_lines(wl);
                }
            }
            self.wake_lines_applied = Some(wl);
        }
        let mode = choose_present_mode(d.present_mode, &self.supported_modes);
        let want = (mode, d.max_frame_latency);
        if self.surface_applied != Some(want) {
            frame.set_wgpu_surface_config(egui_wgpu::SurfaceConfig {
                present_mode: mode,
                desired_maximum_frame_latency: Some(d.max_frame_latency),
            });
            self.surface_applied = Some(want);
        }
        let eff = d.effective();
        if self.fullscreen_applied != Some(eff.fullscreen) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(eff.fullscreen));
            self.fullscreen_applied = Some(eff.fullscreen);
        }
        if (self.zoom_applied - eff.ui_scale).abs() > 1e-3 {
            ctx.set_zoom_factor(eff.ui_scale);
            self.zoom_applied = eff.ui_scale;
        }
    }

    fn draw_video(&mut self, ui: &mut egui::Ui) {
        let area = ui.max_rect();
        let screen = ui.ctx().content_rect();
        let d = self.settings.display.clone();
        let eff = d.effective();
        let par = pixel_aspect(eff.output_aspect, screen.width(), screen.height());
        let mut cells = [LRect::default(); 4];
        let n = view_cells(
            LRect::new(area.left(), area.top(), area.width(), area.height()),
            d.layout,
            CELL_GAP,
            &mut cells,
        );
        let painter = ui.painter().clone();
        painter.rect_filled(area, 0.0, egui::Color32::BLACK);
        let goggle = eff.osd_layout == crate::settings::OsdLayout::GoggleSafe;
        let cleanup = self.settings.video.cleanup_active();
        for (k, cell) in cells.iter().enumerate().take(n) {
            let geom = fit_picture(*cell, 4.0 / 3.0, par, eff.aspect, d.overscan_percent);
            let dest = egui::Rect::from_min_size(
                egui::pos2(geom.dest.x, geom.dest.y),
                egui::vec2(geom.dest.w, geom.dest.h),
            );
            let v = &mut self.views[k];
            let (w, h) = self.gpu.view_size(k).unwrap_or((1, 1));
            let u = Uniforms::new(&geom, w, h, &d, d.row_strip && d.osd, v.has_content);
            let paint = self.gpu.prepare(k, &u);
            painter.add(egui_wgpu::Callback::new_paint_callback(dest, paint));
            let latency = v.latency.summary();
            let error = if v.source.is_none() && v.error.is_none() {
                Some("waiting for source")
            } else {
                v.error.as_deref()
            };
            paint_view(
                &painter,
                dest,
                &ViewOsd {
                    status: &v.status,
                    latency,
                    full: d.osd,
                    goggle,
                    cleanup,
                    error,
                },
            );
        }
        if d.osd {
            let now = now_ns();
            while self
                .messages
                .front()
                .is_some_and(|m| now - m.0 > MESSAGE_NS)
            {
                self.messages.pop_front();
            }
            let msgs: Vec<String> = self.messages.iter().map(|m| m.1.clone()).collect();
            paint_global(
                &painter,
                area,
                &self.backend_status,
                self.settings.video.cleanup,
                &msgs,
                now,
                goggle,
            );
        }
    }

    fn help_window(&mut self, ctx: &egui::Context) {
        let mut open = self.show_help;
        egui::Window::new("OpenVRX keys")
            .open(&mut open)
            .collapsible(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .show(ctx, |ui| {
                egui::Grid::new("keys").striped(true).show(ui, |ui| {
                    for (key, ctrl, _, text) in BINDINGS {
                        ui.monospace(key_name(*key, *ctrl));
                        ui.label(*text);
                        ui.end_row();
                    }
                });
                ui.separator();
                ui.label(format!(
                    "Settings: {}   GPU: {}",
                    self.store.path().display(),
                    self.adapter_name
                ));
            });
        self.show_help = open;
    }
}

/// Present modes the window's surface supports, from a probe surface on
/// the same window (no swap chain is created; the probe is dropped).
fn probe_present_modes(
    cc: &eframe::CreationContext<'_>,
    rs: &egui_wgpu::RenderState,
) -> Vec<wgpu::PresentMode> {
    // SAFETY: the handles come from the live window behind `cc`.
    let target = match unsafe { wgpu::SurfaceTargetUnsafe::from_window(cc) } {
        Ok(t) => t,
        Err(e) => {
            log::warn!("cannot probe present modes: {e}");
            return vec![wgpu::PresentMode::Fifo];
        }
    };
    // SAFETY: the window behind `cc` outlives this function, and the
    // surface is dropped before returning.
    match unsafe { rs.instance.create_surface_unsafe(target) } {
        Ok(surface) => surface.get_capabilities(&rs.adapter).present_modes,
        Err(e) => {
            log::warn!("cannot probe present modes: {e}");
            vec![wgpu::PresentMode::Fifo]
        }
    }
}

impl eframe::App for OpenvrxApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let now = now_ns();
        self.pump_backend();
        self.poll_sources(now);
        if self.scheduler.due(now) {
            self.upload_pending();
            self.scheduler.presented(now);
        } else if let Some(deadline) = self.scheduler.next_deadline_ns() {
            ctx.request_repaint_after(Duration::from_nanos(deadline.saturating_sub(now)));
        }
        self.handle_keys(&ctx);

        if self.show_panels {
            egui::Panel::right("panels")
                .resizable(true)
                .default_size(340.0)
                .show(ui, |ui| panels::show(ui, self));
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(egui::Color32::BLACK))
            .show(ui, |ui| self.draw_video(ui));
        if self.show_help {
            self.help_window(&ctx);
        }
        self.apply_settings(&ctx, frame);

        // Line-to-submit latency: the frame built here is submitted and
        // presented right after this call returns.
        let t = now_ns();
        for &(k, newest) in &self.uploaded {
            if newest <= t {
                let dt = t - newest;
                self.views[k].latency.push(dt);
                self.latency.push(dt);
            }
        }
        self.uploaded.clear();
        self.frame_rate.tick(t);
        ctx.request_repaint_after(Duration::from_secs_f32(
            1.0 / self.settings.display.osd_refresh_hz,
        ));
        if let Some(end) = self.run_until_ns {
            if t >= end {
                let s = self.latency.summary();
                println!(
                    "present mode {} trigger {}: {} ({} samples, max {:.1} ms), {:.1} frames/s, GPU {}",
                    self.surface_applied.map_or("?", |m| mode_name(m.0)),
                    self.settings.display.present_trigger.name(),
                    fmt_latency(&s),
                    s.count,
                    s.max_ms,
                    self.frame_rate.rate(),
                    self.adapter_name
                );
                self.run_until_ns = None;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        }
    }

    fn on_exit(&mut self) {
        self.backend.send(AppCommand::StopRecording);
        self.store.update(&self.settings);
        self.store.flush();
    }
}
