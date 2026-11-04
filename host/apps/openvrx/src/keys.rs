// SPDX-License-Identifier: GPL-3.0-or-later
//! Keyboard shortcuts: one table drives both handling and the F1 help.

use egui::Key;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Action {
    Help,
    Fullscreen,
    Panels,
    Osd,
    Record,
    Cleanup,
    Hd3,
    Layout,
    PresentMode,
    PresentTrigger,
    Aspect,
    Scaling,
    Damage,
    Calibrate,
    ChannelDown,
    ChannelUp,
    ResetStats,
    Escape,
    Quit,
}

/// Key, needs Ctrl, action, help text.
pub const BINDINGS: &[(Key, bool, Action, &str)] = &[
    (Key::F1, false, Action::Help, "Show or hide this help"),
    (Key::F11, false, Action::Fullscreen, "Fullscreen on the window's monitor"),
    (Key::Tab, false, Action::Panels, "Show or hide the control panels"),
    (Key::O, false, Action::Osd, "OSD on/off (C/N bars stay while cleanup is on)"),
    (Key::R, false, Action::Record, "Start or stop raw recording"),
    (Key::Space, false, Action::Cleanup, "Cleanup on/off (raw view)"),
    (Key::H, false, Action::Hd3, "HD3 goggle mode (5:3 output, goggle-safe OSD)"),
    (Key::V, false, Action::Layout, "1 / 2 / 4 views"),
    (Key::P, false, Action::PresentMode, "Present mode: Immediate / Mailbox / Fifo"),
    (Key::L, false, Action::PresentTrigger, "Present every frame / field / N lines"),
    (Key::A, false, Action::Aspect, "Aspect: fit / fill / stretch"),
    (Key::S, false, Action::Scaling, "Scaling: nearest / bilinear / bicubic / Lanczos-2"),
    (Key::D, false, Action::Damage, "Damage-mask overlay"),
    (Key::C, false, Action::Calibrate, "Calibrate the sampling phase of every receiver"),
    (Key::OpenBracket, false, Action::ChannelDown, "Channel down (all receivers)"),
    (Key::CloseBracket, false, Action::ChannelUp, "Channel up (all receivers)"),
    (Key::F5, false, Action::ResetStats, "Reset latency statistics"),
    (Key::Escape, false, Action::Escape, "Close help and panels, leave fullscreen"),
    (Key::Q, true, Action::Quit, "Quit"),
];

/// Action for a key press.
pub fn action_for(key: Key, ctrl: bool) -> Option<Action> {
    BINDINGS
        .iter()
        .find(|(k, c, _, _)| *k == key && *c == ctrl)
        .map(|b| b.2)
}

/// Help line for a binding ("Ctrl+Q", "F11").
pub fn key_name(key: Key, ctrl: bool) -> String {
    if ctrl {
        format!("Ctrl+{}", key.name())
    } else {
        key.name().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bindings_are_unique() {
        for (i, a) in BINDINGS.iter().enumerate() {
            for b in &BINDINGS[i + 1..] {
                assert!(!(a.0 == b.0 && a.1 == b.1), "{:?} bound twice", a.0);
                assert_ne!(a.2, b.2, "{:?} has two keys", a.2);
            }
        }
    }

    #[test]
    fn lookup() {
        assert_eq!(action_for(Key::Space, false), Some(Action::Cleanup));
        assert_eq!(action_for(Key::Q, true), Some(Action::Quit));
        assert_eq!(action_for(Key::Q, false), None);
        assert_eq!(key_name(Key::Q, true), "Ctrl+Q");
        assert_eq!(key_name(Key::F11, false), "F11");
    }
}
