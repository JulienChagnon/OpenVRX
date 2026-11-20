// SPDX-License-Identifier: GPL-3.0-or-later
//! The C header and the Rust types describe the same layout: a C program
//! built from the header prints every size and offset, and they must match
//! `size_of` / `offset_of!` here. Skipped (with a note) when no C compiler
//! is on PATH (`CC`, else `cc`).

use openvrx_filter_abi::{Field, FilterV1, Plane};
use std::mem::{offset_of, size_of};
use std::path::PathBuf;
use std::process::Command;

/// (name, value) pairs the C program prints, computed on the Rust side.
fn rust_layout() -> Vec<(&'static str, usize)> {
    vec![
        ("sizeof_plane", size_of::<Plane>()),
        ("plane_name", offset_of!(Plane, name)),
        ("plane_width", offset_of!(Plane, width)),
        ("plane_height", offset_of!(Plane, height)),
        ("plane_stride", offset_of!(Plane, stride)),
        ("plane_data", offset_of!(Plane, data)),
        ("sizeof_field", size_of::<Field>()),
        ("field_width", offset_of!(Field, width)),
        ("field_height", offset_of!(Field, height)),
        ("field_stride", offset_of!(Field, stride)),
        ("field_cwidth", offset_of!(Field, cwidth)),
        ("field_cheight", offset_of!(Field, cheight)),
        ("field_cstride", offset_of!(Field, cstride)),
        ("field_y", offset_of!(Field, y)),
        ("field_u", offset_of!(Field, u)),
        ("field_v", offset_of!(Field, v)),
        ("field_out_y", offset_of!(Field, out_y)),
        ("field_out_u", offset_of!(Field, out_u)),
        ("field_out_v", offset_of!(Field, out_v)),
        ("field_field_index", offset_of!(Field, field_index)),
        ("field_parity", offset_of!(Field, parity)),
        ("field_flags", offset_of!(Field, flags)),
        ("field_cn_db", offset_of!(Field, cn_db)),
        ("field_n_aux", offset_of!(Field, n_aux)),
        ("field_aux", offset_of!(Field, aux)),
        ("sizeof_filter", size_of::<FilterV1>()),
        ("filter_abi", offset_of!(FilterV1, abi)),
        ("filter_size", offset_of!(FilterV1, size)),
        ("filter_create", offset_of!(FilterV1, create)),
        ("filter_info", offset_of!(FilterV1, info)),
        ("filter_process", offset_of!(FilterV1, process)),
        ("filter_destroy", offset_of!(FilterV1, destroy)),
        ("abi", openvrx_filter_abi::OPENVRX_FILTER_ABI as usize),
        ("flag_discontinuity", openvrx_filter_abi::OPENVRX_FIELD_DISCONTINUITY as usize),
        ("flag_steady", openvrx_filter_abi::OPENVRX_FIELD_STEADY_OSD as usize),
        ("written", openvrx_filter_abi::OPENVRX_PROCESS_WRITTEN as usize),
        ("not_written", openvrx_filter_abi::OPENVRX_PROCESS_NOT_WRITTEN as usize),
    ]
}

const PROGRAM: &str = r#"
#include <stdio.h>
#include "openvrx_filter.h"
#define S(n, t) printf("%s=%zu\n", n, sizeof(t))
#define O(n, t, m) printf("%s=%zu\n", n, offsetof(t, m))
#define V(n, x) printf("%s=%zu\n", n, (size_t)(x))
int main(void) {
    S("sizeof_plane", openvrx_plane_t);
    O("plane_name", openvrx_plane_t, name);
    O("plane_width", openvrx_plane_t, width);
    O("plane_height", openvrx_plane_t, height);
    O("plane_stride", openvrx_plane_t, stride);
    O("plane_data", openvrx_plane_t, data);
    S("sizeof_field", openvrx_field_t);
    O("field_width", openvrx_field_t, width);
    O("field_height", openvrx_field_t, height);
    O("field_stride", openvrx_field_t, stride);
    O("field_cwidth", openvrx_field_t, cwidth);
    O("field_cheight", openvrx_field_t, cheight);
    O("field_cstride", openvrx_field_t, cstride);
    O("field_y", openvrx_field_t, y);
    O("field_u", openvrx_field_t, u);
    O("field_v", openvrx_field_t, v);
    O("field_out_y", openvrx_field_t, out_y);
    O("field_out_u", openvrx_field_t, out_u);
    O("field_out_v", openvrx_field_t, out_v);
    O("field_field_index", openvrx_field_t, field_index);
    O("field_parity", openvrx_field_t, parity);
    O("field_flags", openvrx_field_t, flags);
    O("field_cn_db", openvrx_field_t, cn_db);
    O("field_n_aux", openvrx_field_t, n_aux);
    O("field_aux", openvrx_field_t, aux);
    S("sizeof_filter", openvrx_filter_v1);
    O("filter_abi", openvrx_filter_v1, abi);
    O("filter_size", openvrx_filter_v1, size);
    O("filter_create", openvrx_filter_v1, create);
    O("filter_info", openvrx_filter_v1, info);
    O("filter_process", openvrx_filter_v1, process);
    O("filter_destroy", openvrx_filter_v1, destroy);
    V("abi", OPENVRX_FILTER_ABI);
    V("flag_discontinuity", OPENVRX_FIELD_DISCONTINUITY);
    V("flag_steady", OPENVRX_FIELD_STEADY_OSD);
    V("written", OPENVRX_PROCESS_WRITTEN);
    V("not_written", OPENVRX_PROCESS_NOT_WRITTEN);
    return 0;
}
"#;

#[test]
fn c_header_matches_rust_types() {
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".into());
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("openvrx-filter-abi-layout");
    std::fs::create_dir_all(&dir).unwrap();
    let src = dir.join("layout.c");
    let exe = dir.join(format!("layout{}", std::env::consts::EXE_SUFFIX));
    std::fs::write(&src, PROGRAM).unwrap();
    let include = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("include");
    let built = Command::new(&cc)
        .args(["-std=c99", "-Wall", "-Werror", "-I"])
        .arg(&include)
        .arg(&src)
        .arg("-o")
        .arg(&exe)
        .output();
    let built = match built {
        Ok(o) => o,
        Err(e) => {
            eprintln!("skipped: no C compiler ({cc}: {e})");
            return;
        }
    };
    assert!(
        built.status.success(),
        "the header does not compile:\n{}",
        String::from_utf8_lossy(&built.stderr)
    );
    let run = Command::new(&exe).output().unwrap();
    assert!(run.status.success());
    let text = String::from_utf8(run.stdout).unwrap();
    let c: Vec<(String, usize)> = text
        .lines()
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.to_string(), v.trim().parse().unwrap()))
        .collect();
    let rust = rust_layout();
    assert_eq!(c.len(), rust.len(), "C printed {text}");
    for ((ck, cv), (rk, rv)) in c.iter().zip(&rust) {
        assert_eq!(ck, rk);
        assert_eq!(cv, rv, "{ck}: C {cv}, Rust {rv}");
    }
}

#[test]
fn layout_on_64_bit_targets() {
    // The numbers the header comments and plugin authors can rely on.
    if size_of::<usize>() == 8 {
        assert_eq!(size_of::<Plane>(), 32);
        assert_eq!(size_of::<Field>(), 104);
        assert_eq!(size_of::<FilterV1>(), 40);
        assert_eq!(offset_of!(Field, field_index), 72);
        assert_eq!(offset_of!(Field, aux), 96);
    }
}
