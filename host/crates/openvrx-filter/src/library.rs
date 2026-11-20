// SPDX-License-Identifier: GPL-3.0-or-later
//! Loading a plugin library and checking its table.
//!
//! A library is never unloaded: a plugin may leave threads behind (GPU and
//! inference runtimes do), and unloading code a thread still runs crashes
//! the process. So a path is opened once per process; opening it again
//! returns the same code (restart after rebuilding a plugin).

use openvrx_filter_abi as abi;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

/// A loaded library's checked function table. Cheap to clone; the code
/// stays loaded for the life of the process.
#[derive(Clone, Debug)]
pub struct Library {
    path: PathBuf,
    pub(crate) create: abi::CreateFn,
    pub(crate) info: abi::InfoFn,
    pub(crate) process: abi::ProcessFn,
    pub(crate) destroy: abi::DestroyFn,
}

/// Tables linked into this process, by the path that stands for them.
static STATIC_TABLES: Mutex<Vec<(PathBuf, &'static abi::FilterV1)>> = Mutex::new(Vec::new());

/// Makes `path` resolve to `table`, a filter linked into this process,
/// instead of a file (tests, built-in filters). Later registrations of the
/// same path win.
pub fn register_static(path: impl Into<PathBuf>, table: &'static abi::FilterV1) {
    let path = path.into();
    let mut t = STATIC_TABLES.lock().unwrap_or_else(|e| e.into_inner());
    t.retain(|(p, _)| *p != path);
    t.push((path, table));
}

fn static_table(path: &Path) -> Option<&'static abi::FilterV1> {
    let t = STATIC_TABLES.lock().unwrap_or_else(|e| e.into_inner());
    t.iter().find(|(p, _)| p == path).map(|e| e.1)
}

impl Library {
    /// Loads the plugin at `path` (or the table registered for it) and
    /// checks its ABI version, table size and functions.
    pub fn open(path: &Path) -> Result<Library, String> {
        Library::open_entry(path, abi::ENTRY_SYMBOL)
    }

    /// [`Library::open`] through another entry symbol (the ABI tests).
    #[doc(hidden)]
    pub fn open_entry(path: &Path, symbol: &str) -> Result<Library, String> {
        if let Some(t) = static_table(path) {
            // SAFETY: a complete table with a static lifetime.
            return unsafe { Library::from_table(path, t) };
        }
        // SAFETY: loading runs the library's initialisers; that is what
        // choosing a plugin means.
        let lib = unsafe { libloading::Library::new(path) }
            .map_err(|e| format!("cannot load the plugin {}: {e}", path.display()))?;
        // SAFETY: the symbol's type is fixed by the ABI.
        let entry: abi::EntryFn = unsafe { lib.get::<abi::EntryFn>(symbol.as_bytes()) }
            .map(|s| *s)
            .map_err(|e| format!("{} is not an OpenVRX filter plugin (no {symbol}: {e})", path.display()))?;
        // Never unloaded (see the module docs): the entry point and the
        // table it returns stay valid.
        std::mem::forget(lib);
        // SAFETY: the ABI's entry point takes nothing and returns the table.
        let table = unsafe { entry() };
        // SAFETY: the ABI promises a table valid for the process lifetime.
        unsafe { Library::from_table(path, table) }
    }

    /// Checks a table: version and size before anything else is read (an
    /// older or foreign table may be shorter), then the four functions.
    ///
    /// # Safety
    /// `table` is NULL or points to a table whose first 8 bytes are `abi`
    /// and `size`, valid for `size` bytes.
    unsafe fn from_table(path: &Path, table: *const abi::FilterV1) -> Result<Library, String> {
        let name = path.display();
        if table.is_null() {
            return Err(format!("{name}: the plugin's entry point returned no table"));
        }
        // SAFETY: the first two members exist in any table version.
        let (version, size) = unsafe {
            (
                std::ptr::addr_of!((*table).abi).read(),
                std::ptr::addr_of!((*table).size).read() as usize,
            )
        };
        if version != abi::OPENVRX_FILTER_ABI {
            return Err(format!(
                "{name}: plugin ABI version {version}, this host has version {}",
                abi::OPENVRX_FILTER_ABI
            ));
        }
        let need = std::mem::size_of::<abi::FilterV1>();
        if size < need {
            return Err(format!("{name}: plugin table of {size} bytes, ABI 1 needs {need}"));
        }
        // SAFETY: at least a whole version 1 table, as just checked.
        let t = unsafe { table.read() };
        match (t.create, t.info, t.process, t.destroy) {
            (Some(create), Some(info), Some(process), Some(destroy)) => Ok(Library {
                path: path.to_path_buf(),
                create,
                info,
                process,
                destroy,
            }),
            _ => Err(format!("{name}: the plugin table lacks a function")),
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Short name for messages: the file name without `lib` and extension.
    pub fn short_name(&self) -> String {
        short_name(&self.path)
    }
}

pub(crate) fn short_name(path: &Path) -> String {
    let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let p = std::env::consts::DLL_PREFIX;
    match stem.strip_prefix(p) {
        Some(s) if !p.is_empty() && !s.is_empty() => s.to_string(),
        _ => stem,
    }
}
