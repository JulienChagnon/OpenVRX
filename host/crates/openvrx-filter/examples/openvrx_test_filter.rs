// SPDX-License-Identifier: GPL-3.0-or-later
//! The test plugin (src/fixture.rs) as a plugin library, for the hook's
//! tests: `openvrx_filter_entry` is the real entry point; the other entry
//! points return tables a host must refuse (tests/plugin.rs opens them with
//! `Library::open_entry`).

#[path = "../src/fixture.rs"]
mod fixture;

use openvrx_filter_abi as abi;

#[no_mangle]
pub extern "C" fn openvrx_filter_entry() -> *const abi::FilterV1 {
    &fixture::TABLE
}

#[no_mangle]
pub extern "C" fn openvrx_test_entry_bad_abi() -> *const abi::FilterV1 {
    &fixture::BAD_ABI
}

#[no_mangle]
pub extern "C" fn openvrx_test_entry_short() -> *const abi::FilterV1 {
    &fixture::SHORT
}

#[no_mangle]
pub extern "C" fn openvrx_test_entry_incomplete() -> *const abi::FilterV1 {
    &fixture::INCOMPLETE
}

#[no_mangle]
pub extern "C" fn openvrx_test_entry_null() -> *const abi::FilterV1 {
    std::ptr::null()
}
