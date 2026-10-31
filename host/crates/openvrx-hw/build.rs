// SPDX-License-Identifier: GPL-3.0-or-later
//! Embeds the prebuilt FX2 firmware image when it exists.
//!
//! The FX2 firmware lives in `firmware/fx2` and is built with SDCC into
//! `firmware/fx2/prebuilt/openvrx-fx2.ihx`. When that file is present at build
//! time it is copied into `OUT_DIR` and embedded; otherwise an empty file is
//! written and `fx2::firmware::embedded()` reports that no image is built in.

use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    let src = manifest.join("../../../firmware/fx2/prebuilt/openvrx-fx2.ihx");
    let dst = out.join("openvrx-fx2.ihx");
    println!("cargo:rerun-if-changed={}", src.display());
    println!("cargo:rerun-if-changed=build.rs");
    let bytes = std::fs::read(&src).unwrap_or_default();
    std::fs::write(&dst, bytes).expect("write embedded firmware copy");
}
