// SPDX-License-Identifier: GPL-3.0-or-later
//! Control panels (Tab). They edit `app.settings`; the app turns changes
//! into backend commands, swap-chain and window updates.

use crate::app::OpenvrxApp;
use crate::channels::{self, BANDS};
use crate::commands::AppCommand;
use crate::osd::{fmt_bytes, fmt_cn, fmt_elapsed};
use crate::settings::{
    AspectMode, BoardKind, DiversityMode, GainMode, OutputAspect, PresentModeSetting,
    PresentTrigger, ScalingFilter, StandardChoice, VideoPreset, ViewLayout, DEFAULT_GAIN,
    DIVIDERS,
};
use egui::{CollapsingHeader, ComboBox, DragValue, Slider, Ui};
use openvrx_core::clock::now_ns;
use openvrx_core::LanePreset;

fn combo<T: Copy + PartialEq>(
    ui: &mut Ui,
    id: &str,
    value: &mut T,
    all: &[T],
    name: impl Fn(T) -> String,
) {
    ComboBox::from_id_salt(id)
        .selected_text(name(*value))
        .show_ui(ui, |ui| {
            for &v in all {
                ui.selectable_value(value, v, name(v));
            }
        });
}

pub fn show(ui: &mut Ui, app: &mut OpenvrxApp) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        CollapsingHeader::new("Receivers")
            .default_open(true)
            .show(ui, |ui| receivers(ui, app));
        CollapsingHeader::new("Video")
            .default_open(true)
            .show(ui, |ui| video(ui, app));
        CollapsingHeader::new("Recording")
            .default_open(true)
            .show(ui, |ui| recording(ui, app));
        CollapsingHeader::new("Display").show(ui, |ui| display(ui, app));
        CollapsingHeader::new("Status").show(ui, |ui| status(ui, app));
    });
}

fn receivers(ui: &mut Ui, app: &mut OpenvrxApp) {
    let mut calibrate = None;
    let n = app.settings.receivers.len();
    for k in 0..n {
        let live = app
            .view_status(0)
            .receivers
            .iter()
            .find(|r| r.rx as usize == k)
            .cloned();
        let r = &mut app.settings.receivers[k];
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut r.enabled, "");
                ui.strong(format!("rx{k}"));
                if let Some(s) = &live {
                    ui.label(format!(
                        "C/N {}  gain {}  {}",
                        fmt_cn(s.cn_db),
                        s.gain,
                        if s.sync_locked { "locked" } else { "no sync" }
                    ));
                }
            });
            if r.control == crate::settings::ControlMode::Fx2Only {
                ui.label(format!(
                    "FX2-only mode: channel and gain are the C5's saved ones{} and the controls below do nothing. Lanes and clock must match the C5.",
                    r.fx2_only_mhz.map(|m| format!(" ({})", channels::label(m))).unwrap_or_default()
                ));
            }
            ui.horizontal(|ui| {
                ui.label("Channel");
                let current = channels::name_for(r.freq_mhz);
                for b in &BANDS {
                    let sel = current.is_some_and(|c| c.0 == b.letter);
                    let mut menu = ComboBox::from_id_salt(format!("band{k}{}", b.letter))
                        .width(36.0)
                        .selected_text(match current {
                            Some((l, c)) if sel => format!("{l}{c}"),
                            _ => b.letter.to_string(),
                        });
                    menu = menu.height(240.0);
                    menu.show_ui(ui, |ui| {
                        for (i, &f) in b.mhz.iter().enumerate() {
                            ui.selectable_value(&mut r.freq_mhz, f, format!("{}{} {f}", b.letter, i + 1));
                        }
                    });
                }
            });
            ui.horizontal(|ui| {
                ui.label("MHz");
                ui.add(DragValue::new(&mut r.freq_mhz).range(channels::MIN_MHZ..=channels::MAX_MHZ));
                ui.label(channels::label(r.freq_mhz));
            });
            ui.horizontal(|ui| {
                ui.label("Gain");
                let mut auto = r.gain == GainMode::Auto;
                if ui.checkbox(&mut auto, "auto").changed() {
                    r.gain = if auto {
                        GainMode::Auto
                    } else {
                        GainMode::Manual {
                            index: live.as_ref().map_or(DEFAULT_GAIN, |s| s.gain),
                        }
                    };
                }
                if let GainMode::Manual { index } = &mut r.gain {
                    ui.add(Slider::new(index, 0..=77));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Board");
                combo(ui, &format!("board{k}"), &mut r.board, &[BoardKind::Xiao, BoardKind::Zero, BoardKind::Custom], |b| {
                    b.name().to_string()
                });
                ui.label("Lanes");
                combo(ui, &format!("lanes{k}"), &mut r.lanes, &LanePreset::ALL, |p| p.name().to_string());
            });
            ui.horizontal(|ui| {
                ui.label("Divider");
                combo(ui, &format!("div{k}"), &mut r.divider, &DIVIDERS, |d| {
                    format!("/{d} ({:.2} MS/s)", crate::settings::divider_hz(d) / 1e6)
                });
                if ui.button("Calibrate").clicked() {
                    calibrate = Some(k as u8);
                }
            });
            ui.horizontal(|ui| {
                ui.label("C5 port");
                let mut port = r.c5_port.clone().unwrap_or_default();
                if ui.add(egui::TextEdit::singleline(&mut port).desired_width(60.0).hint_text("auto")).changed() {
                    r.c5_port = (!port.trim().is_empty()).then(|| port.trim().to_string());
                }
                ui.label("FX2");
                let mut serial = r.fx2_serial.clone().unwrap_or_default();
                if ui.add(egui::TextEdit::singleline(&mut serial).desired_width(80.0).hint_text("auto")).changed() {
                    r.fx2_serial = (!serial.trim().is_empty()).then(|| serial.trim().to_string());
                }
            });
        });
    }
    ui.horizontal(|ui| {
        if ui.button("Add receiver").clicked() && n < crate::settings::MAX_RECEIVERS {
            let mut r = app.settings.receivers.last().cloned().unwrap_or_default();
            r.name = format!("rx{n}");
            r.c5_port = None;
            r.fx2_serial = None;
            app.settings.receivers.push(r);
        }
        if ui.button("Remove last").clicked() && n > 1 {
            app.settings.receivers.pop();
        }
    });
    ui.label("Port and serial changes apply on the next start.");
    if let Some(rx) = calibrate {
        app.send(AppCommand::Calibrate { rx });
    }
}

fn video(ui: &mut Ui, app: &mut OpenvrxApp) {
    let v = &mut app.settings.video;
    ui.horizontal(|ui| {
        ui.label("Preset");
        combo(ui, "preset", &mut v.preset, &VideoPreset::ALL, |p| p.name().to_string());
    });
    ui.checkbox(&mut v.cleanup, "Cleanup (Space: off = raw view)");
    ui.checkbox(&mut v.steady_osd, "Steady OSD (I: fields line-doubled like goggles)");
    ui.horizontal(|ui| {
        ui.label("Standard");
        combo(ui, "standard", &mut v.standard, &StandardChoice::ALL, |s| s.name().to_string());
    });
    ui.horizontal(|ui| {
        ui.label("Diversity");
        combo(ui, "diversity", &mut v.diversity, &DiversityMode::ALL, |d| d.name().to_string());
    });
}

fn recording(ui: &mut Ui, app: &mut OpenvrxApp) {
    let rec = app.backend_status().recording.clone();
    ui.horizontal(|ui| {
        ui.label("Folder");
        let mut s = app.settings.recording.folder.display().to_string();
        if ui.text_edit_singleline(&mut s).changed() {
            app.settings.recording.folder = s.into();
        }
    });
    if rec.active {
        ui.label(format!(
            "Recording {}  {}",
            fmt_elapsed(now_ns().saturating_sub(rec.started_ns)),
            fmt_bytes(rec.bytes)
        ));
        if let Some(p) = &rec.path {
            ui.label(p.display().to_string());
        }
        if ui.button("Stop (R)").clicked() {
            app.toggle_recording();
        }
    } else if ui
        .add_enabled(rec.available, egui::Button::new("Record (R)"))
        .clicked()
    {
        app.toggle_recording();
    }
}

fn display(ui: &mut Ui, app: &mut OpenvrxApp) {
    let d = &mut app.settings.display;
    ui.horizontal(|ui| {
        ui.label("Present");
        combo(ui, "present", &mut d.present_mode, &PresentModeSetting::ALL, |m| m.name().to_string());
        combo(ui, "trigger", &mut d.present_trigger, &PresentTrigger::ALL, |t| t.name().to_string());
    });
    if d.present_trigger == PresentTrigger::Lines {
        ui.add(Slider::new(&mut d.present_lines, 4..=288).text("lines per slice"));
    }
    ui.add(Slider::new(&mut d.max_present_hz, 0..=2000).text("max presents/s (0 = no cap)"));
    ui.add(Slider::new(&mut d.max_frame_latency, 1..=3).text("max frame latency"));
    ui.horizontal(|ui| {
        ui.label("Views");
        combo(ui, "layout", &mut d.layout, &ViewLayout::ALL, |l| l.name().to_string());
    });
    ui.horizontal(|ui| {
        ui.label("Aspect");
        combo(ui, "aspect", &mut d.aspect, &AspectMode::ALL, |a| a.name().to_string());
        combo(ui, "outaspect", &mut d.output_aspect, &OutputAspect::ALL, |a| a.name().to_string());
    });
    ui.horizontal(|ui| {
        ui.label("Scaling");
        combo(ui, "scaling", &mut d.scaling, &ScalingFilter::ALL, |s| s.name().to_string());
    });
    ui.add(Slider::new(&mut d.overscan_percent, 0.0..=15.0).text("overscan %"));
    ui.add(Slider::new(&mut d.sharpen, 0.0..=2.0).text("sharpen"));
    ui.add(Slider::new(&mut d.brightness, -0.5..=0.5).text("brightness"));
    ui.add(Slider::new(&mut d.contrast, 0.25..=3.0).text("contrast"));
    ui.add(Slider::new(&mut d.saturation, 0.0..=3.0).text("saturation"));
    ui.checkbox(&mut d.fullscreen, "Fullscreen (F11)");
    ui.checkbox(&mut d.hd3_mode, "HD3 mode (H)");
    ui.checkbox(&mut d.hd3_stretch, "HD3: stretch to full width");
    ui.checkbox(&mut d.osd, "OSD (O)");
    ui.checkbox(&mut d.row_strip, "Receiver strip");
    ui.checkbox(&mut d.damage_overlay, "Damage overlay (D)");
    ui.add(Slider::new(&mut d.ui_scale, 0.5..=3.0).text("UI scale"));
}

fn status(ui: &mut Ui, app: &mut OpenvrxApp) {
    let s = app.backend_status().clone();
    ui.label(format!("Backend: {}", s.name));
    for d in &s.devices {
        ui.label(d);
    }
    for w in &s.warnings {
        ui.colored_label(egui::Color32::from_rgb(255, 140, 120), w);
    }
    ui.label(format!("GPU: {}", app.adapter_name()));
    let lat = app.latency_summary();
    ui.label(crate::osd::fmt_latency(&lat));
}
