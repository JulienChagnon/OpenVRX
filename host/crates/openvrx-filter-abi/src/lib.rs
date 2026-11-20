// SPDX-License-Identifier: GPL-3.0-or-later
//! OpenVRX video filter plugin ABI, version 1.
//!
//! A filter plugin is a shared library (`.so`, `.dll`, `.dylib`) that the
//! OpenVRX app (the `plugin` video preset) and `openvrx-cli decode
//! --filter-plugin` load at run time. The host hands it every displayed
//! field of a view as float planes and shows what it writes back. This
//! crate is the contract: the C types as `#[repr(C)]` Rust types, and the
//! same types as a C header for plugins in other languages
//! (`include/openvrx_filter.h`, also in [`C_HEADER`]). The tests compile the
//! header with the system C compiler and compare every size and offset.
//!
//! # Entry point
//!
//! The library exports one symbol with C linkage, [`ENTRY_SYMBOL`]:
//!
//! ```c
//! const openvrx_filter_v1 *openvrx_filter_entry(void);
//! ```
//!
//! It returns a table ([`FilterV1`]) that stays valid for the life of the
//! process. The host reads `abi` and `size` first and refuses the library
//! unless `abi == OPENVRX_FILTER_ABI`, `size >= size_of::<FilterV1>()` and
//! all four function pointers are set. A later revision of version 1 may
//! append members and report a larger size.
//!
//! # Threading
//!
//! Every call into one library (`create`, `info`, `process` and `destroy`,
//! for every handle it creates) comes from one host thread, one call at a
//! time. The host keeps that thread for the life of the process, so a
//! plugin needs no locking and may keep thread-bound state (a GPU context
//! made in `create` is current in `process`). Two views running the same
//! library share that thread: their calls interleave, never overlap.
//!
//! # Lifetimes
//!
//! - `create(config, err, err_len)`: `config` is a NUL-terminated UTF-8
//!   string (the host's plugin config setting, possibly empty), valid during
//!   the call only. Return a non-NULL handle, or NULL with a NUL-terminated
//!   message in `err` (`err_len` bytes; [`write_error`] truncates). `create`
//!   may take long (loading a model, building an engine): the host shows
//!   the plain picture meanwhile.
//! - `info(h)`: a NUL-terminated UTF-8 JSON object owned by the plugin,
//!   valid until `destroy(h)`. The host reads it once, right after `create`.
//! - `process(h, f)`: `f` and everything it points to (planes, the aux
//!   array, names) are valid during the call only; keep no pointers. Inputs
//!   are read-only; the out planes are host memory with the inputs' layout,
//!   not aliasing them, undefined on entry.
//! - `destroy(h)`: once for every handle `create` returned, on the same
//!   thread; no call uses `h` afterwards.
//!
//! # Errors
//!
//! `process` returns [`OPENVRX_PROCESS_WRITTEN`] (0: the host shows the out
//! planes), [`OPENVRX_PROCESS_NOT_WRITTEN`] (1: the host shows the input,
//! for example while a temporal filter fills its history) or a negative
//! code: the host then stops calling `process` on that handle, destroys it,
//! falls back to the plain picture and shows the code on its OSD (details
//! may go to stderr). A plugin runs inside the host process, so a crash ends
//! the host: never let a C++ exception or a Rust panic unwind out of these
//! functions.
//!
//! # Planes
//!
//! Y is luma, 0 (black) to 1 (white), with noise and overshoot a little past
//! both ends. U and V are the BT.601 colour differences `U = 0.492 (B - Y)`
//! and `V = 0.877 (R - Y)` (about +-0.436 and +-0.615 at full saturation).
//! Strides count floats, not bytes. The ABI 1 host always passes chroma at
//! full resolution (`cwidth == width`, `cheight == height`) and the picture
//! a view shows after each field: a full-height frame per field (NTSC 754 x
//! 480, PAL 922 x 576 on the app's sample grid) from the video pipeline the
//! plugin asks for. With [`OPENVRX_FIELD_STEADY_OSD`] each field line fills
//! two rows, the same rows for both fields (the goggles' steady OSD);
//! otherwise each field sits at its true half-line position with the rows
//! between its lines interpolated.
//!
//! # Field metadata
//!
//! - `field_index` counts the fields the host produced for this handle,
//!   from 0. A jump of more than one means the host skipped fields because
//!   the plugin was still busy; the field after a skip carries
//!   [`OPENVRX_FIELD_DISCONTINUITY`].
//! - `parity`: 0 for the top field (its lines are frame rows 0, 2, 4 ...),
//!   1 for the bottom field.
//! - `flags`: [`OPENVRX_FIELD_DISCONTINUITY`] when the fields before this
//!   one are unrelated to it (first field, retune, sync relock, new standard
//!   or size, pipeline restart, skipped fields): restart any temporal
//!   history from this field. [`OPENVRX_FIELD_STEADY_OSD`] for line-doubled
//!   geometry.
//! - `cn_db`: the field's carrier-to-noise ratio in dB as the receiver
//!   measured it (median over its rows), NaN when unknown.
//! - `aux`: the side planes the plugin named in its info JSON, in that
//!   order ([`AUX_PLANES`] lists what the host has).
//!
//! # Info JSON
//!
//! Keys the host reads, all optional (others are ignored): `name` (for the
//! OSD; default the library file name), `pipeline` (the video preset the
//! plugin expects as input: `raw`, the default, `low_latency`, `balanced` or
//! `quality`), `geometry` (`steady`: line-doubled fields whatever the steady
//! OSD setting; `fields`: true half-line positions; `any`, the default:
//! follow the setting), `aux` (side plane names) and `provider` (short OSD
//! text such as the compute device).
//!
//! # Deadline
//!
//! The app hands each field over as soon as it is complete and waits at
//! most its `plugin_deadline_ms` setting (default 6 ms) for the result, then
//! shows the field unfiltered. If the plugin is still busy with an earlier
//! field when the next one is complete, that field is shown unfiltered and
//! waits in a one-field mailbox (a newer field replaces a waiting one, which
//! is then skipped). So the plugin keeps getting fields in order when it
//! runs late; its output is shown only when it is on time.
//!
//! # Loading
//!
//! A host loads a library once per process and never unloads it (GPU
//! runtimes leave threads behind): restart the app after rebuilding one.

use std::ffi::{c_char, c_void, CStr};

/// ABI version a plugin reports in [`FilterV1::abi`].
pub const OPENVRX_FILTER_ABI: u32 = 1;

/// Field flag: the fields before this one are unrelated (retune, relock,
/// standard change, first field...).
pub const OPENVRX_FIELD_DISCONTINUITY: u32 = 1;
/// Field flag: line-doubled steady-OSD geometry.
pub const OPENVRX_FIELD_STEADY_OSD: u32 = 2;

/// `process` wrote the out planes.
pub const OPENVRX_PROCESS_WRITTEN: i32 = 0;
/// `process` wrote nothing: the host shows the input.
pub const OPENVRX_PROCESS_NOT_WRITTEN: i32 = 1;

/// Name of the one exported symbol.
pub const ENTRY_SYMBOL: &str = "openvrx_filter_entry";

/// The C header, for plugin authors and tools.
pub const C_HEADER: &str = include_str!("../include/openvrx_filter.h");

/// Side planes an ABI 1 host can supply, by name (see the crate docs).
pub const AUX_PLANES: [&str; 7] = [
    "damage",
    "row_cn_db",
    "row_noise_ire",
    "row_sync_score",
    "row_timing_err",
    "row_clicks",
    "row_flags",
];

/// A named side plane (`openvrx_plane_t`). `stride` in floats.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Plane {
    pub name: *const c_char,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub data: *const f32,
}

/// One field (`openvrx_field_t`).
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Field {
    /// Luma plane, stride in floats.
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    /// Chroma planes.
    pub cwidth: u32,
    pub cheight: u32,
    pub cstride: u32,
    /// Live scale: Y 0..1, BT.601 U/V.
    pub y: *const f32,
    pub u: *const f32,
    pub v: *const f32,
    /// Same layout, written by the plugin.
    pub out_y: *mut f32,
    pub out_u: *mut f32,
    pub out_v: *mut f32,
    /// +1 per field fed.
    pub field_index: u64,
    /// 0 top, 1 bottom.
    pub parity: u32,
    /// `OPENVRX_FIELD_*`.
    pub flags: u32,
    /// NaN when unknown.
    pub cn_db: f32,
    pub n_aux: u32,
    /// Optional named side planes, may be 0/NULL.
    pub aux: *const Plane,
}

/// `create`: NULL on error, message in `err`.
pub type CreateFn = unsafe extern "C" fn(config: *const c_char, err: *mut c_char, err_len: usize) -> *mut c_void;
/// `info`: JSON, valid until `destroy`.
pub type InfoFn = unsafe extern "C" fn(h: *mut c_void) -> *const c_char;
/// `process`: 0 = out written, 1 = not written (show input), <0 error.
pub type ProcessFn = unsafe extern "C" fn(h: *mut c_void, f: *const Field) -> i32;
pub type DestroyFn = unsafe extern "C" fn(h: *mut c_void);
/// The exported entry point.
pub type EntryFn = unsafe extern "C" fn() -> *const FilterV1;

/// The table `openvrx_filter_entry` returns (`openvrx_filter_v1`). The
/// function pointers are `Option`s with the layout of nullable C pointers,
/// so a host can check them.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FilterV1 {
    /// [`OPENVRX_FILTER_ABI`].
    pub abi: u32,
    /// `size_of::<FilterV1>()`.
    pub size: u32,
    pub create: Option<CreateFn>,
    pub info: Option<InfoFn>,
    pub process: Option<ProcessFn>,
    pub destroy: Option<DestroyFn>,
}

impl FilterV1 {
    /// A complete version 1 table.
    pub const fn new(create: CreateFn, info: InfoFn, process: ProcessFn, destroy: DestroyFn) -> FilterV1 {
        FilterV1 {
            abi: OPENVRX_FILTER_ABI,
            size: std::mem::size_of::<FilterV1>() as u32,
            create: Some(create),
            info: Some(info),
            process: Some(process),
            destroy: Some(destroy),
        }
    }
}

/// Floats a plane of `width` x `height` at `stride` spans: the last row
/// ends after `width` floats.
pub fn plane_len(width: u32, height: u32, stride: u32) -> usize {
    if width == 0 || height == 0 {
        0
    } else {
        (height as usize - 1) * stride as usize + width as usize
    }
}

/// Writes `msg` into a `create` error buffer: NUL-terminated, truncated to
/// `err_len - 1` bytes at a UTF-8 boundary. Nothing for a NULL buffer or a
/// zero length.
///
/// # Safety
/// `err` must be NULL or valid for `err_len` bytes of writes.
pub unsafe fn write_error(err: *mut c_char, err_len: usize, msg: &str) {
    if err.is_null() || err_len == 0 {
        return;
    }
    let mut n = msg.len().min(err_len - 1);
    while !msg.is_char_boundary(n) {
        n -= 1;
    }
    // SAFETY: n < err_len bytes, the caller vouches for err_len.
    unsafe {
        std::ptr::copy_nonoverlapping(msg.as_ptr() as *const c_char, err, n);
        *err.add(n) = 0;
    }
}

/// # Safety
/// `ptr` must be NULL or valid for [`plane_len`] floats.
unsafe fn slice<'a>(ptr: *const f32, width: u32, height: u32, stride: u32) -> &'a [f32] {
    let n = plane_len(width, height, stride);
    if ptr.is_null() || n == 0 {
        &[]
    } else {
        // SAFETY: as documented.
        unsafe { std::slice::from_raw_parts(ptr, n) }
    }
}

/// # Safety
/// `ptr` must be NULL or valid for [`plane_len`] floats of writes, with no
/// other reference to them.
unsafe fn slice_mut<'a>(ptr: *mut f32, width: u32, height: u32, stride: u32) -> &'a mut [f32] {
    let n = plane_len(width, height, stride);
    if ptr.is_null() || n == 0 {
        &mut []
    } else {
        // SAFETY: as documented.
        unsafe { std::slice::from_raw_parts_mut(ptr, n) }
    }
}

impl Plane {
    /// The plane's name ("" for NULL).
    ///
    /// # Safety
    /// `name` must be NULL or a NUL-terminated string (true inside
    /// `process`).
    pub unsafe fn name(&self) -> &CStr {
        if self.name.is_null() {
            c""
        } else {
            // SAFETY: as documented.
            unsafe { CStr::from_ptr(self.name) }
        }
    }

    /// The plane's floats ([`plane_len`] of them).
    ///
    /// # Safety
    /// As inside `process`: `data` valid for the plane's extent.
    pub unsafe fn data(&self) -> &[f32] {
        // SAFETY: as documented.
        unsafe { slice(self.data, self.width, self.height, self.stride) }
    }
}

impl Field {
    /// The input planes Y, U, V as slices ([`plane_len`] floats each).
    ///
    /// # Safety
    /// Only inside `process`, where the host guarantees the extents.
    pub unsafe fn input(&self) -> [&[f32]; 3] {
        // SAFETY: as documented.
        unsafe {
            [
                slice(self.y, self.width, self.height, self.stride),
                slice(self.u, self.cwidth, self.cheight, self.cstride),
                slice(self.v, self.cwidth, self.cheight, self.cstride),
            ]
        }
    }

    /// The output planes Y, U, V as mutable slices.
    ///
    /// # Safety
    /// Only inside `process`, once (the three planes are distinct host
    /// buffers that nothing else references during the call).
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn output(&self) -> [&mut [f32]; 3] {
        // SAFETY: as documented.
        unsafe {
            [
                slice_mut(self.out_y, self.width, self.height, self.stride),
                slice_mut(self.out_u, self.cwidth, self.cheight, self.cstride),
                slice_mut(self.out_v, self.cwidth, self.cheight, self.cstride),
            ]
        }
    }

    /// The side planes.
    ///
    /// # Safety
    /// Only inside `process`.
    pub unsafe fn aux(&self) -> &[Plane] {
        if self.aux.is_null() || self.n_aux == 0 {
            &[]
        } else {
            // SAFETY: as documented.
            unsafe { std::slice::from_raw_parts(self.aux, self.n_aux as usize) }
        }
    }

    /// The side plane named `name`, if the host passed it.
    ///
    /// # Safety
    /// Only inside `process`.
    pub unsafe fn aux_named(&self, name: &str) -> Option<&Plane> {
        // SAFETY: as documented.
        unsafe { self.aux().iter().find(|p| p.name().to_bytes() == name.as_bytes()) }
    }

    pub fn discontinuity(&self) -> bool {
        self.flags & OPENVRX_FIELD_DISCONTINUITY != 0
    }

    pub fn steady_osd(&self) -> bool {
        self.flags & OPENVRX_FIELD_STEADY_OSD != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_buffer_truncates_at_char_boundaries() {
        let mut buf = [0x55 as c_char; 8];
        unsafe { write_error(buf.as_mut_ptr(), buf.len(), "abc") };
        assert_eq!(unsafe { CStr::from_ptr(buf.as_ptr()) }.to_str().unwrap(), "abc");
        // 7 bytes fit; "é" (2 bytes) at bytes 6..8 must not be split.
        unsafe { write_error(buf.as_mut_ptr(), buf.len(), "abcdeféz") };
        assert_eq!(unsafe { CStr::from_ptr(buf.as_ptr()) }.to_str().unwrap(), "abcdef");
        unsafe { write_error(std::ptr::null_mut(), 10, "x") };
        unsafe { write_error(buf.as_mut_ptr(), 0, "x") };
    }

    #[test]
    fn plane_extent() {
        assert_eq!(plane_len(10, 3, 12), 34);
        assert_eq!(plane_len(10, 0, 12), 0);
        assert_eq!(plane_len(0, 5, 12), 0);
        assert_eq!(plane_len(1, 480, 1), 480);
    }

    #[test]
    fn table_constructor_fills_abi_and_size() {
        unsafe extern "C" fn c(_: *const c_char, _: *mut c_char, _: usize) -> *mut c_void {
            std::ptr::null_mut()
        }
        unsafe extern "C" fn i(_: *mut c_void) -> *const c_char {
            c"{}".as_ptr()
        }
        unsafe extern "C" fn p(_: *mut c_void, _: *const Field) -> i32 {
            1
        }
        unsafe extern "C" fn d(_: *mut c_void) {}
        static T: FilterV1 = FilterV1::new(c, i, p, d);
        assert_eq!(T.abi, OPENVRX_FILTER_ABI);
        assert_eq!(T.size as usize, std::mem::size_of::<FilterV1>());
        assert!(T.create.is_some() && T.info.is_some() && T.process.is_some() && T.destroy.is_some());
        assert!(C_HEADER.contains("#define OPENVRX_FILTER_ABI 1u"));
    }
}
