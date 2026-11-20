// SPDX-License-Identifier: GPL-3.0-or-later
//! The Esc menu: every shortcut as a clickable row with its key, and a band
//! and channel picker. The keys keep working; the menu is a way to find them.

use crate::channels::{self, BANDS};
use crate::keys::{key_name, Action, BINDINGS};
use egui::{Color32, CornerRadius, RichText, Stroke, Vec2};
use std::sync::mpsc;

const BG: Color32 = Color32::from_rgba_premultiplied(16, 18, 22, 238);
const ROW_HOVER: Color32 = Color32::from_rgba_premultiplied(255, 255, 255, 14);
const BORDER: Color32 = Color32::from_rgba_premultiplied(255, 255, 255, 22);
const TEXT: Color32 = Color32::from_rgb(222, 226, 232);
const DIM: Color32 = Color32::from_rgb(132, 140, 152);
const ACCENT: Color32 = Color32::from_rgb(64, 182, 166);
const CHIP_BG: Color32 = Color32::from_rgb(38, 42, 50);

/// Menu groups in display order. Actions not listed (Escape) stay key-only.
const GROUPS: &[(&str, &[Action])] = &[
    (
        "View",
        &[Action::Fullscreen, Action::Layout, Action::Hd3, Action::Aspect, Action::Scaling, Action::Osd],
    ),
    (
        "Picture",
        &[
            Action::Cleanup,
            Action::Preset,
            Action::SteadyOsd,
            Action::Damage,
            Action::Compare,
            Action::PresentMode,
            Action::PresentTrigger,
        ],
    ),
    (
        "Receiver",
        &[Action::Record, Action::Calibrate, Action::ChannelDown, Action::ChannelUp, Action::ResetStats],
    ),
    ("App", &[Action::Panels, Action::Help, Action::Quit]),
];

/// Short menu label (the F1 help keeps the long text).
fn label(a: Action) -> &'static str {
    match a {
        Action::Help => "All keys",
        Action::Fullscreen => "Fullscreen",
        Action::Panels => "Control panels",
        Action::Osd => "OSD",
        Action::Record => "Record raw",
        Action::Cleanup => "Cleanup",
        Action::Preset => "Video preset",
        Action::SteadyOsd => "Steady OSD",
        Action::Hd3 => "HD3 goggle mode",
        Action::Layout => "Views 1 / 2 / 4",
        Action::PresentMode => "Present mode",
        Action::PresentTrigger => "Present trigger",
        Action::Aspect => "Aspect",
        Action::Scaling => "Scaling",
        Action::Damage => "Damage overlay",
        Action::Calibrate => "Calibrate phase",
        Action::Compare => "Compare with test card",
        Action::ChannelDown => "Channel down",
        Action::ChannelUp => "Channel up",
        Action::ResetStats => "Reset latency stats",
        Action::Escape => "Close",
        Action::Quit => "Quit",
    }
}

/// Actions that close the menu after a click (they open something else or end the app).
fn closes_menu(a: Action) -> bool {
    matches!(a, Action::Help | Action::Panels | Action::Quit | Action::Compare)
}

/// What the app shows next to a row: an on/off state, or a current value.
pub enum RowState {
    None,
    On(bool),
    Value(String),
}

/// What the user picked this frame.
pub enum MenuPick {
    Act(Action),
    Tune(u32),
}

pub struct Menu {
    pub open: bool,
    /// Band whose channels are shown (follows the current channel on open).
    band: char,
    /// A tune running on the C5 console (FX2-only mode): its result.
    tuning: Option<(u32, mpsc::Receiver<Result<u32, String>>)>,
}

impl Menu {
    pub fn new() -> Menu {
        Menu {
            open: false,
            band: 'R',
            tuning: None,
        }
    }

    pub fn toggle(&mut self, mhz: u32) {
        self.open = !self.open;
        if self.open {
            if let Some((b, _)) = channels::name_for(mhz) {
                self.band = b;
            }
        }
    }

    /// Starts a tune on the C5 console in the background.
    pub fn start_tune(&mut self, mhz: u32) {
        let (tx, rx) = mpsc::channel();
        std::thread::Builder::new()
            .name("openvrx-menu-tune".into())
            .spawn(move || {
                let _ = tx.send(tune_on_console(mhz));
            })
            .ok();
        self.tuning = Some((mhz, rx));
    }

    /// Result of a finished background tune.
    pub fn poll_tune(&mut self) -> Option<Result<u32, String>> {
        let (_, rx) = self.tuning.as_ref()?;
        match rx.try_recv() {
            Ok(r) => {
                self.tuning = None;
                Some(r)
            }
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                self.tuning = None;
                Some(Err("tune thread ended".into()))
            }
        }
    }

    /// Draws the menu; returns what was clicked.
    pub fn show(&mut self, ctx: &egui::Context, mhz: u32, state: impl Fn(Action) -> RowState) -> Vec<MenuPick> {
        let mut picks = Vec::new();
        if !self.open {
            return picks;
        }
        let frame = egui::Frame::NONE
            .fill(BG)
            .stroke(Stroke::new(1.0, BORDER))
            .corner_radius(CornerRadius::same(14))
            .inner_margin(egui::Margin::same(20))
            .shadow(egui::epaint::Shadow {
                offset: [0, 6],
                blur: 28,
                spread: 0,
                color: Color32::from_black_alpha(140),
            });
        egui::Area::new(egui::Id::unique("openvrx-menu"))
            .order(egui::Order::Foreground)
            .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
            .show(ctx, |ui| {
                frame.show(ui, |ui| {
                    ui.set_width(460.0);
                    ui.spacing_mut().item_spacing = Vec2::new(8.0, 6.0);
                    self.header(ui, mhz);
                    ui.add_space(14.0);
                    self.channel_picker(ui, mhz, &mut picks);
                    ui.add_space(16.0);
                    egui::ScrollArea::vertical()
                        .max_height((ctx.content_rect().height() - 330.0).max(160.0))
                        .show(ui, |ui| actions(ui, &state, &mut picks));
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Esc").monospace().color(DIM));
                        ui.label(RichText::new("close").color(DIM));
                        ui.label(RichText::new("·").color(DIM));
                        ui.label(RichText::new("the keys work with the menu closed too").color(DIM));
                    });
                });
            });
        for p in &picks {
            if let MenuPick::Act(a) = p {
                if closes_menu(*a) {
                    self.open = false;
                }
            }
        }
        picks
    }

    fn header(&self, ui: &mut egui::Ui, mhz: u32) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("OpenVRX").size(20.0).strong().color(TEXT));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let (text, color) = match &self.tuning {
                    Some((m, _)) => (format!("tuning {} …", channels::label(*m)), DIM),
                    None => (channels::label(mhz), ACCENT),
                };
                ui.label(RichText::new(text).size(15.0).color(color));
            });
        });
    }

    fn channel_picker(&mut self, ui: &mut egui::Ui, mhz: u32, picks: &mut Vec<MenuPick>) {
        section(ui, "Channel");
        ui.horizontal(|ui| {
            for b in BANDS.iter() {
                let on = b.letter == self.band;
                let r = ui
                    .add(pill(RichText::new(b.letter.to_string()).strong(), on).min_size(Vec2::new(40.0, 28.0)))
                    .on_hover_text(b.name);
                if r.clicked() {
                    self.band = b.letter;
                }
            }
        });
        ui.add_space(4.0);
        let band = channels::band(self.band).unwrap_or(&BANDS[0]);
        let busy = self.tuning.is_some();
        ui.horizontal(|ui| {
            for (k, &f) in band.mhz.iter().enumerate() {
                let on = f == mhz;
                let text = RichText::new(format!("{}\n{f}", k + 1)).size(12.5);
                let r = ui.add_enabled(!busy, pill(text, on).min_size(Vec2::new(50.0, 42.0)));
                if r.clicked() && !on {
                    picks.push(MenuPick::Tune(f));
                }
            }
        });
    }
}

/// Tunes the C5 over its console and saves it (FX2-only mode, where the app
/// has no C5 link of its own). Pushes no other setting.
fn tune_on_console(mhz: u32) -> Result<u32, String> {
    use openvrx_hw::c5::{find_c5_ports, C5Client, ClientConfig, PortConnector, PortSelector};
    let port = find_c5_ports()
        .map_err(|e| e.to_string())?
        .into_iter()
        .next()
        .map(|p| p.port_name)
        .ok_or("no C5 on USB: plug its USB in to change channel")?;
    let c5 = C5Client::start(PortConnector::new(PortSelector::Name(port.clone())), None, ClientConfig::default())
        .map_err(|e| e.to_string())?;
    if !c5.wait_connected(std::time::Duration::from_secs(3)) {
        return Err(format!("the C5 on {port} does not answer"));
    }
    let (got, _) = c5.tune(mhz, 0).map_err(|e| format!("tune: {e}"))?;
    c5.save().map_err(|e| format!("save: {e}"))?;
    Ok(got)
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title.to_uppercase()).size(11.0).strong().color(DIM));
}

/// Rounded button, filled with the accent colour when `on`.
fn pill(text: RichText, on: bool) -> egui::Button<'static> {
    let (fill, color) = if on { (ACCENT, Color32::from_rgb(10, 22, 20)) } else { (CHIP_BG, TEXT) };
    egui::Button::new(text.color(color))
        .fill(fill)
        .stroke(Stroke::NONE)
        .corner_radius(CornerRadius::same(8))
}

fn actions(ui: &mut egui::Ui, state: &impl Fn(Action) -> RowState, picks: &mut Vec<MenuPick>) {
    for (g, (title, list)) in GROUPS.iter().enumerate() {
        if g > 0 {
            ui.add_space(10.0);
        }
        section(ui, title);
        for &a in list.iter() {
            let Some((key, ctrl, _, help)) = BINDINGS.iter().find(|b| b.2 == a) else {
                continue;
            };
            if row(ui, label(a), &key_name(*key, *ctrl), state(a)).on_hover_text(*help).clicked() {
                picks.push(MenuPick::Act(a));
            }
        }
    }
}

/// One clickable row: label left, state and key chip right.
fn row(ui: &mut egui::Ui, text: &str, key: &str, state: RowState) -> egui::Response {
    let h = 28.0;
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), h), egui::Sense::click());
    let p = ui.painter();
    if resp.hovered() {
        p.rect_filled(rect, CornerRadius::same(6), ROW_HOVER);
    }
    let font = egui::FontId::proportional(14.0);
    let mid = rect.center().y;
    p.text(egui::pos2(rect.left() + 10.0, mid), egui::Align2::LEFT_CENTER, text, font.clone(), TEXT);
    // Key chip at the right edge.
    let chip_font = egui::FontId::monospace(12.0);
    let galley = p.layout_no_wrap(key.to_string(), chip_font, TEXT);
    let chip = egui::Rect::from_min_size(
        egui::pos2(rect.right() - 10.0 - galley.size().x - 14.0, mid - 10.0),
        Vec2::new(galley.size().x + 14.0, 20.0),
    );
    p.rect_filled(chip, CornerRadius::same(5), CHIP_BG);
    p.galley(chip.center() - galley.size() / 2.0, galley, TEXT);
    // State just left of the chip.
    let (s, color) = match state {
        RowState::None => (String::new(), DIM),
        RowState::On(true) => ("on".to_string(), ACCENT),
        RowState::On(false) => ("off".to_string(), DIM),
        RowState::Value(v) => (v, DIM),
    };
    if !s.is_empty() {
        p.text(egui::pos2(chip.left() - 10.0, mid), egui::Align2::RIGHT_CENTER, s, font, color);
    }
    resp.on_hover_cursor(egui::CursorIcon::PointingHand)
}
