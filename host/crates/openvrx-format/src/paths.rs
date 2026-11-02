// SPDX-License-Identifier: GPL-3.0-or-later
//! File names of a recording.

use openvrx_core::RxId;
use std::path::{Path, PathBuf};

const SIDECAR_EXT: &str = ".openvrx.json";
const EVENTS_EXT: &str = ".events.jsonl";
const TRUTH_EXT: &str = ".truth.f32";

/// Directory and name of a recording, and the file names derived from them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecordingPaths {
    pub dir: PathBuf,
    pub name: String,
}

impl RecordingPaths {
    /// From a base path `dir/NAME` (no extension).
    pub fn from_base(base: impl AsRef<Path>) -> RecordingPaths {
        let base = base.as_ref();
        let dir = base
            .parent()
            .map(Path::to_path_buf)
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| PathBuf::from("."));
        let name = base
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        RecordingPaths { dir, name }
    }

    /// From any file of the recording (sidecar, raw stream, events, truth)
    /// or from the base path itself.
    pub fn resolve(path: impl AsRef<Path>) -> RecordingPaths {
        let path = path.as_ref();
        let file = path
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let dir = path
            .parent()
            .map(Path::to_path_buf)
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| PathBuf::from("."));
        let name = Self::strip_known_suffix(&file).unwrap_or(file);
        RecordingPaths { dir, name }
    }

    /// Recording name for a file name with a known suffix.
    pub fn strip_known_suffix(file: &str) -> Option<String> {
        for ext in [SIDECAR_EXT, EVENTS_EXT, TRUTH_EXT] {
            if let Some(stem) = file.strip_suffix(ext) {
                return Some(stem.to_string());
            }
        }
        // NAME.rxK.bin
        let stem = file.strip_suffix(".bin")?;
        let dot = stem.rfind('.')?;
        let tail = &stem[dot + 1..];
        let digits = tail.strip_prefix("rx")?;
        if !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit()) {
            Some(stem[..dot].to_string())
        } else {
            None
        }
    }

    /// True when the file name is a recording sidecar.
    pub fn is_sidecar(file: &Path) -> bool {
        file.file_name()
            .map(|f| f.to_string_lossy().ends_with(SIDECAR_EXT))
            .unwrap_or(false)
    }

    pub fn base(&self) -> PathBuf {
        self.dir.join(&self.name)
    }

    pub fn sidecar(&self) -> PathBuf {
        self.dir.join(self.sidecar_name())
    }

    pub fn sidecar_name(&self) -> String {
        format!("{}{SIDECAR_EXT}", self.name)
    }

    pub fn events(&self) -> PathBuf {
        self.dir.join(self.events_name())
    }

    pub fn events_name(&self) -> String {
        format!("{}{EVENTS_EXT}", self.name)
    }

    pub fn truth(&self) -> PathBuf {
        self.dir.join(self.truth_name())
    }

    pub fn truth_name(&self) -> String {
        format!("{}{TRUTH_EXT}", self.name)
    }

    pub fn rx_name(&self, rx: RxId) -> String {
        format!("{}.rx{rx}.bin", self.name)
    }

    pub fn rx(&self, rx: RxId) -> PathBuf {
        self.dir.join(self.rx_name(rx))
    }

    /// Resolves a file name stored in the sidecar against the directory.
    pub fn file(&self, name: &str) -> PathBuf {
        let p = Path::new(name);
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            self.dir.join(p)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_every_file() {
        for f in [
            "d/run1.openvrx.json",
            "d/run1.rx0.bin",
            "d/run1.rx12.bin",
            "d/run1.events.jsonl",
            "d/run1.truth.f32",
            "d/run1",
        ] {
            let p = RecordingPaths::resolve(f);
            assert_eq!(p.name, "run1", "{f}");
            assert_eq!(p.dir, PathBuf::from("d"));
        }
        let p = RecordingPaths::resolve("a.b.rx1.bin");
        assert_eq!(p.name, "a.b");
        assert_eq!(p.dir, PathBuf::from("."));
        assert_eq!(RecordingPaths::strip_known_suffix("x.rxa.bin"), None);
        assert_eq!(p.rx_name(3), "a.b.rx3.bin");
    }
}
