// SPDX-License-Identifier: GPL-3.0-or-later
//! A test plugin without any model, for the hook's tests. Its config is
//! JSON (all keys optional):
//!
//! ```text
//! {"mode": "identity" | "blur" | "pass" | "probe",   default identity
//!  "sleep_ms": 0,            sleep this long in every process call
//!  "fail_after": N,          process returns -3 after N good fields
//!  "fail_create": "message", create fails with this message
//!  "name", "pipeline", "geometry", "aux", "provider"   echoed in info}
//! ```
//!
//! `blur` is a 3x3 box blur of every plane (edges clamped), `pass` writes
//! nothing (returns 1), `probe` copies the input and then writes what it
//! saw into the first luma samples of the output: field_index, flags,
//! parity, cn_db, n_aux, the first value of the first side plane (-1 when
//! none), width, height, 1 when the side plane names are those its info
//! asked for, 1 when the call runs on the thread that ran create, and a
//! number for the calling thread.
//!
//! It keeps no global state, so tests may call it from several threads.
//! The cdylib example `openvrx_test_filter` exports it as a plugin library;
//! with the `fixture` feature it is linked into the host crate for tests
//! that register it under a path (`register_static`).

use openvrx_filter_abi as abi;
use std::ffi::{c_char, c_void, CStr, CString};
use std::hash::{Hash, Hasher};
use std::time::Duration;

#[derive(Copy, Clone, PartialEq)]
enum Mode {
    Identity,
    Blur,
    Pass,
    Probe,
}

struct Fixture {
    mode: Mode,
    sleep: Duration,
    fail_after: Option<u64>,
    good: u64,
    aux: Vec<String>,
    thread: std::thread::ThreadId,
    info: CString,
}

fn parse(config: &str) -> Result<Fixture, String> {
    let v: serde_json::Value = if config.trim().is_empty() {
        serde_json::json!({})
    } else {
        serde_json::from_str(config).map_err(|e| format!("test filter config: {e}"))?
    };
    if let Some(m) = v.get("fail_create").and_then(|m| m.as_str()) {
        return Err(m.to_string());
    }
    let mode = match v.get("mode").and_then(|m| m.as_str()).unwrap_or("identity") {
        "identity" => Mode::Identity,
        "blur" => Mode::Blur,
        "pass" => Mode::Pass,
        "probe" => Mode::Probe,
        m => return Err(format!("test filter: unknown mode {m}")),
    };
    let aux: Vec<String> = v
        .get("aux")
        .and_then(|a| a.as_array())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect())
        .unwrap_or_default();
    let mut info = serde_json::json!({
        "name": v.get("name").and_then(|n| n.as_str()).unwrap_or("test filter"),
        "pipeline": v.get("pipeline").cloned().unwrap_or("raw".into()),
        "geometry": v.get("geometry").cloned().unwrap_or("any".into()),
        "aux": aux,
        "mode": v.get("mode").cloned().unwrap_or("identity".into()),
    });
    if let Some(p) = v.get("provider") {
        info["provider"] = p.clone();
    }
    Ok(Fixture {
        mode,
        sleep: Duration::from_secs_f64(v.get("sleep_ms").and_then(|s| s.as_f64()).unwrap_or(0.0).max(0.0) / 1e3),
        fail_after: v.get("fail_after").and_then(|n| n.as_u64()),
        good: 0,
        aux,
        thread: std::thread::current().id(),
        info: CString::new(info.to_string()).map_err(|e| e.to_string())?,
    })
}

unsafe extern "C" fn create(config: *const c_char, err: *mut c_char, err_len: usize) -> *mut c_void {
    let cfg = if config.is_null() {
        String::new()
    } else {
        // SAFETY: the host passes a NUL-terminated string.
        unsafe { CStr::from_ptr(config) }.to_string_lossy().into_owned()
    };
    match parse(&cfg) {
        Ok(f) => Box::into_raw(Box::new(f)) as *mut c_void,
        Err(e) => {
            // SAFETY: the host's buffer of err_len bytes.
            unsafe { abi::write_error(err, err_len, &e) };
            std::ptr::null_mut()
        }
    }
}

unsafe extern "C" fn info(h: *mut c_void) -> *const c_char {
    // SAFETY: a handle from create.
    let f = unsafe { &*(h as *const Fixture) };
    f.info.as_ptr()
}

/// 3x3 box blur of one plane, edges clamped.
pub fn box3(src: &[f32], w: usize, h: usize, stride: usize, dst: &mut [f32]) {
    for y in 0..h {
        for x in 0..w {
            let mut s = 0.0f32;
            for dy in [-1i64, 0, 1] {
                for dx in [-1i64, 0, 1] {
                    let yy = (y as i64 + dy).clamp(0, h as i64 - 1) as usize;
                    let xx = (x as i64 + dx).clamp(0, w as i64 - 1) as usize;
                    s += src[yy * stride + xx];
                }
            }
            dst[y * stride + x] = s / 9.0;
        }
    }
}

fn thread_number() -> f32 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    std::thread::current().id().hash(&mut h);
    (h.finish() % 65_536) as f32
}

unsafe extern "C" fn process(h: *mut c_void, f: *const abi::Field) -> i32 {
    // SAFETY: a handle from create and the host's field, valid in the call.
    let (fx, f) = unsafe { (&mut *(h as *mut Fixture), &*f) };
    if !fx.sleep.is_zero() {
        std::thread::sleep(fx.sleep);
    }
    if fx.fail_after.is_some_and(|n| fx.good >= n) {
        return -3;
    }
    fx.good += 1;
    if fx.mode == Mode::Pass {
        return abi::OPENVRX_PROCESS_NOT_WRITTEN;
    }
    // SAFETY: inside process; the output planes are taken once.
    let (input, output) = unsafe { (f.input(), f.output()) };
    let sizes = [(f.width, f.height, f.stride), (f.cwidth, f.cheight, f.cstride), (f.cwidth, f.cheight, f.cstride)];
    for ((src, dst), (w, h, s)) in input.iter().zip(output).zip(sizes) {
        if fx.mode == Mode::Blur {
            box3(src, w as usize, h as usize, s as usize, dst);
        } else {
            dst.copy_from_slice(src);
        }
    }
    if fx.mode == Mode::Probe {
        // SAFETY: inside process.
        let aux = unsafe { f.aux() };
        // SAFETY: inside process.
        let names: Vec<String> = aux.iter().map(|p| unsafe { p.name() }.to_string_lossy().into_owned()).collect();
        // SAFETY: inside process.
        let first = aux.first().and_then(|p| unsafe { p.data() }.first().copied()).unwrap_or(-1.0);
        let probe = [
            f.field_index as f32,
            f.flags as f32,
            f.parity as f32,
            f.cn_db,
            f.n_aux as f32,
            first,
            f.width as f32,
            f.height as f32,
            (names == fx.aux) as u8 as f32,
            (std::thread::current().id() == fx.thread) as u8 as f32,
            thread_number(),
        ];
        // SAFETY: inside process; the slices above are dropped.
        let out_y = unsafe { f.output() }.into_iter().next().unwrap_or_default();
        for (o, p) in out_y.iter_mut().zip(probe) {
            *o = p;
        }
    }
    abi::OPENVRX_PROCESS_WRITTEN
}

unsafe extern "C" fn destroy(h: *mut c_void) {
    // SAFETY: a handle from create, destroyed once.
    drop(unsafe { Box::from_raw(h as *mut Fixture) });
}

const V1: abi::FilterV1 = abi::FilterV1::new(create, info, process, destroy);

/// The fixture's table.
pub static TABLE: abi::FilterV1 = V1;

/// Tables a host must refuse: another ABI version, a short table, a
/// missing function.
pub static BAD_ABI: abi::FilterV1 = abi::FilterV1 { abi: 2, ..V1 };
pub static SHORT: abi::FilterV1 = abi::FilterV1 { size: 16, ..V1 };
pub static INCOMPLETE: abi::FilterV1 = abi::FilterV1 { process: None, ..V1 };
