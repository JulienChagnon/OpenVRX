// SPDX-License-Identifier: GPL-3.0-or-later
//! `decode ... --filter-plugin LIB`: a filter plugin (the openvrx-filter
//! ABI, as the app's `plugin` preset loads it) on every decoded field, with
//! the same metadata the app gives it, and its time per field.
//!
//! Offline every field waits for the plugin (no deadline), so each saved
//! picture is the plugin's; the summary counts the fields that took longer
//! than the live deadline, i.e. the ones the app would show plain.

use crate::{opt, opt_num};
use openvrx_core::Frame;
use openvrx_filter::{Feed, FieldBuffers, Library, Plugin, PluginInfo, Timing, DEFAULT_DEADLINE_MS};
use std::path::Path;
use std::time::Instant;

pub struct CliFilter {
    plugin: Plugin,
    library: String,
    feed: Feed,
    buf: FieldBuffers,
    out: Frame,
    timing: Timing,
    deadline_ms: f32,
    create_s: f64,
    written: u64,
    not_written: u64,
    over: u64,
    steady: Option<bool>,
}

impl CliFilter {
    /// The plugin of the decode arguments (None without --filter-plugin),
    /// created and ready.
    pub fn from_args(args: &[String]) -> Result<Option<CliFilter>, String> {
        let Some(path) = opt(args, "--filter-plugin") else {
            return Ok(None);
        };
        let config = opt(args, "--plugin-config").unwrap_or("");
        let deadline_ms: f32 = opt_num(args, "--plugin-deadline-ms", DEFAULT_DEADLINE_MS)?;
        let t0 = Instant::now();
        let lib = Library::open(Path::new(path))?;
        let plugin = Plugin::create(&lib, config)?;
        let create_s = t0.elapsed().as_secs_f64();
        eprintln!("filter plugin ready in {create_s:.2} s: {}", plugin.info().json);
        Ok(Some(CliFilter {
            plugin,
            library: path.to_string(),
            feed: Feed::new(),
            buf: FieldBuffers::default(),
            out: Frame::new(1, 1),
            timing: Timing::new(usize::MAX),
            deadline_ms,
            create_s,
            written: 0,
            not_written: 0,
            over: 0,
            steady: None,
        }))
    }

    pub fn info(&self) -> &PluginInfo {
        self.plugin.info()
    }

    /// The picture to save for one field (`frame` as decoded, `steady` when
    /// line-doubled): the plugin's, or the field itself when the plugin
    /// writes nothing.
    pub fn apply<'a>(&'a mut self, frame: &'a Frame, steady: bool) -> Result<&'a Frame, String> {
        self.steady = Some(steady);
        let meta = self.feed.next(frame, steady);
        self.buf.fill(frame, meta, &self.plugin.info().aux);
        let t0 = Instant::now();
        let r = self.plugin.process(&mut self.buf);
        let ms = t0.elapsed().as_secs_f32() * 1e3;
        self.timing.push(ms);
        if ms > self.deadline_ms {
            self.over += 1;
        }
        match r {
            Ok(true) => {
                self.buf.take_output(frame, &mut self.out);
                self.written += 1;
                Ok(&self.out)
            }
            Ok(false) => {
                self.not_written += 1;
                Ok(frame)
            }
            Err(c) => Err(format!("filter plugin error {c} at field {}", meta.index)),
        }
    }

    /// The timing summary on stderr, and as JSON to `--plugin-json PATH`.
    pub fn report(&self, args: &[String]) -> Result<(), String> {
        let s = self.timing.summary();
        let info = self.plugin.info();
        let pct = if s.count > 0 {
            100.0 * self.over as f64 / s.count as f64
        } else {
            0.0
        };
        eprintln!(
            "filter plugin {}: {} fields, ms per field p50 {:.2} p99 {:.2} max {:.2} mean {:.2}; \
             written {}, not written {}; over the {} ms deadline {} ({pct:.1} %); create {:.2} s",
            info.label(),
            s.count,
            s.p50_ms,
            s.p99_ms,
            s.max_ms,
            s.mean_ms,
            self.written,
            self.not_written,
            self.deadline_ms,
            self.over,
            self.create_s
        );
        if let Some(path) = opt(args, "--plugin-json") {
            let summary = serde_json::json!({
                "library": self.library,
                "info": serde_json::from_str::<serde_json::Value>(&info.json).unwrap_or(serde_json::Value::Null),
                "pipeline": info.pipeline,
                "steady": self.steady,
                "fields": s.count,
                "written": self.written,
                "not_written": self.not_written,
                "p50_ms": s.p50_ms,
                "p99_ms": s.p99_ms,
                "max_ms": s.max_ms,
                "mean_ms": s.mean_ms,
                "deadline_ms": self.deadline_ms,
                "over_deadline": self.over,
                "create_s": self.create_s,
            });
            std::fs::write(path, format!("{summary}\n")).map_err(|e| format!("{path}: {e}"))?;
        }
        Ok(())
    }
}
