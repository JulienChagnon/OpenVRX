// SPDX-License-Identifier: GPL-3.0-or-later
//! One plugin instance and its info JSON.

use crate::field::FieldBuffers;
use crate::library::Library;
use openvrx_filter_abi as abi;
use serde_json::Value;
use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr::NonNull;

/// Video pipelines a plugin can ask for (`openvrx_video::VideoConfig::preset`
/// names).
pub const PIPELINES: [&str; 4] = ["raw", "low_latency", "balanced", "quality"];

/// Field geometry a plugin wants.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Geometry {
    /// Whatever the steady OSD setting gives (flagged per field).
    Any,
    /// Line-doubled fields (steady OSD) whatever the setting.
    Steady,
    /// Fields at their true half-line position whatever the setting.
    Fields,
}

impl Geometry {
    /// Line-doubled or not, given the steady OSD setting.
    pub fn steady(self, steady_osd: bool) -> bool {
        match self {
            Geometry::Any => steady_osd,
            Geometry::Steady => true,
            Geometry::Fields => false,
        }
    }
}

/// What a plugin's `info` JSON says, checked.
#[derive(Clone, Debug, PartialEq)]
pub struct PluginInfo {
    pub name: String,
    /// One of [`PIPELINES`].
    pub pipeline: String,
    pub geometry: Geometry,
    /// Side planes, each one of `abi::AUX_PLANES`.
    pub aux: Vec<String>,
    pub provider: Option<String>,
    /// The JSON as the plugin returned it.
    pub json: String,
}

impl PluginInfo {
    /// Parses and checks an info JSON object; `fallback_name` names a
    /// plugin that gives none.
    pub fn parse(json: &str, fallback_name: &str) -> Result<PluginInfo, String> {
        let v: Value = serde_json::from_str(json).map_err(|e| format!("plugin info is not JSON: {e}"))?;
        let obj = v.as_object().ok_or("plugin info is not a JSON object")?;
        let text = |k: &str| -> Result<Option<String>, String> {
            match obj.get(k) {
                None | Some(Value::Null) => Ok(None),
                Some(Value::String(s)) => Ok(Some(s.clone())),
                Some(_) => Err(format!("plugin info: \"{k}\" is not a string")),
            }
        };
        let name = text("name")?.filter(|s| !s.trim().is_empty()).unwrap_or_else(|| fallback_name.to_string());
        let pipeline = text("pipeline")?.unwrap_or_else(|| "raw".into());
        if !PIPELINES.contains(&pipeline.as_str()) {
            return Err(format!(
                "plugin info: pipeline \"{pipeline}\" is not one of {}",
                PIPELINES.join(", ")
            ));
        }
        let geometry = match text("geometry")?.as_deref() {
            None | Some("any") => Geometry::Any,
            Some("steady") => Geometry::Steady,
            Some("fields") => Geometry::Fields,
            Some(g) => return Err(format!("plugin info: geometry \"{g}\" is not steady, fields or any")),
        };
        let mut aux = Vec::new();
        match obj.get("aux") {
            None | Some(Value::Null) => {}
            Some(Value::Array(a)) => {
                for x in a {
                    let s = x.as_str().ok_or("plugin info: \"aux\" holds a non-string")?;
                    if !abi::AUX_PLANES.contains(&s) {
                        return Err(format!(
                            "plugin needs the side plane \"{s}\"; this host has {}",
                            abi::AUX_PLANES.join(", ")
                        ));
                    }
                    if aux.iter().any(|a| a == s) {
                        return Err(format!("plugin info: side plane \"{s}\" named twice"));
                    }
                    aux.push(s.to_string());
                }
            }
            Some(_) => return Err("plugin info: \"aux\" is not an array".into()),
        }
        Ok(PluginInfo {
            name,
            pipeline,
            geometry,
            aux,
            provider: text("provider")?.filter(|s| !s.is_empty()),
            json: json.to_string(),
        })
    }

    /// "name (provider)" for messages.
    pub fn label(&self) -> String {
        match &self.provider {
            Some(p) => format!("{} ({p})", self.name),
            None => self.name.clone(),
        }
    }
}

/// Bytes `create` may write its error message into.
const ERR_LEN: usize = 1024;

/// One plugin instance. Every call goes to the thread that made it: the
/// type is neither `Send` nor `Sync` (the ABI's threading contract).
pub struct Plugin {
    lib: Library,
    handle: NonNull<c_void>,
    info: PluginInfo,
    /// The side plane list of the call in progress.
    planes: Vec<abi::Plane>,
}

impl Plugin {
    /// Calls `create(config)` and reads `info`.
    pub fn create(lib: &Library, config: &str) -> Result<Plugin, String> {
        let name = lib.short_name();
        let c = CString::new(config).map_err(|_| "the plugin config contains a NUL byte".to_string())?;
        let mut err = vec![0 as c_char; ERR_LEN];
        // SAFETY: a checked table; NUL-terminated config; err holds ERR_LEN.
        let h = unsafe { (lib.create)(c.as_ptr(), err.as_mut_ptr(), err.len()) };
        let Some(handle) = NonNull::new(h) else {
            err[ERR_LEN - 1] = 0;
            // SAFETY: NUL-terminated (just forced).
            let msg = unsafe { CStr::from_ptr(err.as_ptr()) }.to_string_lossy().trim().to_string();
            return Err(if msg.is_empty() {
                format!("{name}: create failed (no message)")
            } else {
                format!("{name}: {msg}")
            });
        };
        // SAFETY: a live handle.
        let ptr = unsafe { (lib.info)(handle.as_ptr()) };
        let json = if ptr.is_null() {
            "{}".to_string()
        } else {
            // SAFETY: the ABI's NUL-terminated string, valid until destroy.
            unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
        };
        match PluginInfo::parse(&json, &name) {
            Ok(info) => Ok(Plugin {
                lib: lib.clone(),
                handle,
                info,
                planes: Vec::new(),
            }),
            Err(e) => {
                // SAFETY: the handle create returned, destroyed once.
                unsafe { (lib.destroy)(handle.as_ptr()) };
                Err(format!("{name}: {e}"))
            }
        }
    }

    pub fn info(&self) -> &PluginInfo {
        &self.info
    }

    /// Runs the plugin on one field: Ok(true) when it wrote the output
    /// planes, Ok(false) when it left them (show the input), Err(code) for
    /// a plugin error (or -1000 for buffers that do not hold a picture).
    pub fn process(&mut self, field: &mut FieldBuffers) -> Result<bool, i32> {
        if !field.consistent() {
            return Err(-1000);
        }
        let f = field.abi_field(&mut self.planes);
        // SAFETY: a live handle; `f` points into `field` and `planes`,
        // which are not touched until the call returns.
        let r = unsafe { (self.lib.process)(self.handle.as_ptr(), &f) };
        match r {
            abi::OPENVRX_PROCESS_WRITTEN => Ok(true),
            abi::OPENVRX_PROCESS_NOT_WRITTEN => Ok(false),
            c if c < 0 => Err(c),
            // Positive codes are reserved: treat as "not written".
            _ => Ok(false),
        }
    }
}

impl Drop for Plugin {
    fn drop(&mut self) {
        // SAFETY: the handle create returned, destroyed once, on this thread.
        unsafe { (self.lib.destroy)(self.handle.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn info_defaults_and_checks() {
        let i = PluginInfo::parse("{}", "lib").unwrap();
        assert_eq!((i.name.as_str(), i.pipeline.as_str(), i.geometry), ("lib", "raw", Geometry::Any));
        assert!(i.aux.is_empty() && i.provider.is_none());
        let i = PluginInfo::parse(
            r#"{"name":"n","pipeline":"quality","geometry":"steady","aux":["damage","row_flags"],"provider":"gpu","x":1}"#,
            "lib",
        )
        .unwrap();
        assert_eq!(i.pipeline, "quality");
        assert_eq!(i.geometry, Geometry::Steady);
        assert_eq!(i.aux, vec!["damage", "row_flags"]);
        assert_eq!(i.label(), "n (gpu)");
        for bad in [
            "[1]",
            "not json",
            r#"{"pipeline":"fancy"}"#,
            r#"{"geometry":"tilted"}"#,
            r#"{"aux":["envelope"]}"#,
            r#"{"aux":["damage","damage"]}"#,
            r#"{"aux":"damage"}"#,
            r#"{"name":3}"#,
        ] {
            assert!(PluginInfo::parse(bad, "lib").is_err(), "{bad}");
        }
    }

    #[test]
    fn geometry_against_the_setting() {
        assert!(Geometry::Any.steady(true) && !Geometry::Any.steady(false));
        assert!(Geometry::Steady.steady(false));
        assert!(!Geometry::Fields.steady(true));
    }

    #[test]
    fn pipelines_exist_in_the_video_crate() {
        for p in PIPELINES {
            assert!(openvrx_video::VideoConfig::preset(p, openvrx_core::VideoStandard::Ntsc).is_some(), "{p}");
        }
    }
}
