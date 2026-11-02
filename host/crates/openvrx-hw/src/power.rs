// SPDX-License-Identifier: GPL-3.0-or-later
//! Windows scheduling and power helpers: MMCSS "Pro Audio" registration
//! for hot threads, 1 ms timer resolution, and a check of the USB
//! selective suspend power setting (which can suspend an idle FX2 and
//! break a stream). On other systems the helpers are harmless no-ops.

use crate::error::{HwError, Result};
use serde::{Deserialize, Serialize};

/// Registers the calling thread with MMCSS while alive. Create and drop it
/// on the same thread.
pub struct MmcssGuard {
    #[cfg(windows)]
    handle: windows_sys::Win32::Foundation::HANDLE,
    task_index: u32,
}

impl MmcssGuard {
    /// Registers the current thread in the "Pro Audio" task class at high
    /// priority.
    pub fn pro_audio() -> Result<MmcssGuard> {
        Self::register("Pro Audio")
    }

    /// Registers the current thread in any MMCSS task class.
    pub fn register(task: &str) -> Result<MmcssGuard> {
        #[cfg(windows)]
        {
            use windows_sys::Win32::System::Threading::{
                AvSetMmThreadCharacteristicsW, AvSetMmThreadPriority, AVRT_PRIORITY_HIGH,
            };
            let wide: Vec<u16> = task.encode_utf16().chain(std::iter::once(0)).collect();
            let mut index = 0u32;
            // SAFETY: `wide` is a NUL-terminated UTF-16 string that outlives
            // the call; `index` is a valid out pointer.
            let handle = unsafe { AvSetMmThreadCharacteristicsW(wide.as_ptr(), &mut index) };
            if handle.is_null() {
                return Err(HwError::Io(std::io::Error::last_os_error()));
            }
            // SAFETY: `handle` was just returned by MMCSS for this thread.
            unsafe { AvSetMmThreadPriority(handle, AVRT_PRIORITY_HIGH) };
            Ok(MmcssGuard {
                handle,
                task_index: index,
            })
        }
        #[cfg(not(windows))]
        {
            let _ = task;
            Ok(MmcssGuard { task_index: 0 })
        }
    }

    /// MMCSS task index assigned to the thread.
    pub fn task_index(&self) -> u32 {
        self.task_index
    }
}

impl Drop for MmcssGuard {
    fn drop(&mut self) {
        #[cfg(windows)]
        {
            use windows_sys::Win32::System::Threading::AvRevertMmThreadCharacteristics;
            // SAFETY: the handle came from AvSetMmThreadCharacteristicsW.
            unsafe { AvRevertMmThreadCharacteristics(self.handle) };
        }
    }
}

/// Raises the system timer resolution while alive (timeBeginPeriod).
pub struct TimerResolution {
    period_ms: u32,
}

impl TimerResolution {
    /// Requests `period_ms` (1 is typical for capture and display).
    pub fn request(period_ms: u32) -> Result<TimerResolution> {
        let period_ms = period_ms.max(1);
        #[cfg(windows)]
        {
            // SAFETY: plain value call.
            let r = unsafe { windows_sys::Win32::Media::timeBeginPeriod(period_ms) };
            if r != 0 {
                return Err(HwError::Invalid(format!(
                    "timeBeginPeriod({period_ms}) refused (code {r})"
                )));
            }
        }
        Ok(TimerResolution { period_ms })
    }

    pub fn period_ms(&self) -> u32 {
        self.period_ms
    }
}

impl Drop for TimerResolution {
    fn drop(&mut self) {
        #[cfg(windows)]
        {
            // SAFETY: matches the timeBeginPeriod in `request`.
            unsafe { windows_sys::Win32::Media::timeEndPeriod(self.period_ms) };
        }
    }
}

/// USB selective suspend setting of the active power plan.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SelectiveSuspend {
    /// On mains power (None when unreadable).
    pub ac_enabled: Option<bool>,
    /// On battery.
    pub dc_enabled: Option<bool>,
}

/// Power setting subgroup "USB settings".
pub const USB_SUBGROUP: &str = "2a737441-1930-4402-8d77-b2bebba308a3";
/// Power setting "USB selective suspend setting".
pub const USB_SELECTIVE_SUSPEND: &str = "48e6b7a6-50f5-4782-a5d4-53bb8f07e226";

impl SelectiveSuspend {
    /// Reads the active plan's setting (Windows only).
    pub fn read() -> Result<SelectiveSuspend> {
        #[cfg(windows)]
        {
            use windows_sys::core::GUID;
            use windows_sys::Win32::Foundation::LocalFree;
            use windows_sys::Win32::System::Power::{
                PowerGetActiveScheme, PowerReadACValueIndex, PowerReadDCValueIndex,
            };
            let sub = GUID::from_u128(0x2a737441_1930_4402_8d77_b2bebba308a3);
            let set = GUID::from_u128(0x48e6b7a6_50f5_4782_a5d4_53bb8f07e226);
            let mut scheme: *mut GUID = std::ptr::null_mut();
            // SAFETY: out pointer; the returned GUID is freed with LocalFree.
            let r = unsafe { PowerGetActiveScheme(std::ptr::null_mut(), &mut scheme) };
            if r != 0 || scheme.is_null() {
                return Err(HwError::Invalid(format!("PowerGetActiveScheme failed (code {r})")));
            }
            let mut ac = 0u32;
            let mut dc = 0u32;
            // SAFETY: valid GUID pointers and out pointers.
            let rac = unsafe { PowerReadACValueIndex(std::ptr::null_mut(), scheme, &sub, &set, &mut ac) };
            // SAFETY: as above.
            let rdc = unsafe { PowerReadDCValueIndex(std::ptr::null_mut(), scheme, &sub, &set, &mut dc) };
            // SAFETY: `scheme` was allocated by PowerGetActiveScheme.
            unsafe { LocalFree(scheme.cast()) };
            Ok(SelectiveSuspend {
                ac_enabled: (rac == 0).then_some(ac != 0),
                dc_enabled: (rdc == 0).then_some(dc != 0),
            })
        }
        #[cfg(not(windows))]
        {
            Ok(SelectiveSuspend {
                ac_enabled: None,
                dc_enabled: None,
            })
        }
    }

    /// True when suspend may hit a running FX2 on either power source.
    pub fn any_enabled(&self) -> bool {
        self.ac_enabled == Some(true) || self.dc_enabled == Some(true)
    }

    /// A user hint with the commands that disable it, or None when fine.
    pub fn hint(&self) -> Option<String> {
        if !self.any_enabled() {
            return None;
        }
        let mut which = Vec::new();
        if self.ac_enabled == Some(true) {
            which.push("plugged in");
        }
        if self.dc_enabled == Some(true) {
            which.push("on battery");
        }
        Some(format!(
            "USB selective suspend is enabled ({}). It can suspend an FX2 and interrupt capture. \
             Disable it in Power Options > Change plan settings > Advanced settings > USB settings, or run:\n  \
             powercfg /SETACVALUEINDEX SCHEME_CURRENT {USB_SUBGROUP} {USB_SELECTIVE_SUSPEND} 0\n  \
             powercfg /SETDCVALUEINDEX SCHEME_CURRENT {USB_SUBGROUP} {USB_SELECTIVE_SUSPEND} 0\n  \
             powercfg /SETACTIVE SCHEME_CURRENT",
            which.join(" and ")
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mmcss_and_timer_on_this_thread() {
        // MMCSS may be unavailable on a build machine; it must fail cleanly.
        if let Ok(g) = MmcssGuard::pro_audio() {
            let _ = g.task_index();
        }
        let t = TimerResolution::request(1).unwrap();
        assert_eq!(t.period_ms(), 1);
    }

    #[test]
    fn suspend_hint() {
        let off = SelectiveSuspend {
            ac_enabled: Some(false),
            dc_enabled: None,
        };
        assert!(off.hint().is_none());
        let on = SelectiveSuspend {
            ac_enabled: Some(true),
            dc_enabled: Some(true),
        };
        let h = on.hint().unwrap();
        assert!(h.contains("plugged in and on battery"));
        assert!(h.contains(USB_SELECTIVE_SUSPEND));
        // Reading the real setting works or fails cleanly.
        let _ = SelectiveSuspend::read();
    }
}
