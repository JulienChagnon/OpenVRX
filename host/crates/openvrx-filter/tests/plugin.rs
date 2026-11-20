// SPDX-License-Identifier: GPL-3.0-or-later
//! The hook through real libraries: the test plugin built as a cdylib
//! (examples/openvrx_test_filter.rs, which `cargo test` builds first) and,
//! when a C compiler is on PATH, the C example plugin of the ABI crate.

use openvrx_core::{FieldParity, Frame, VideoStandard};
use openvrx_filter::abi;
use openvrx_filter::{
    Feed, FieldBuffers, Instance, Library, LiveFilter, Outcome, Phase, Plugin, ReportPhase, Submit,
};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

const READY: Duration = Duration::from_secs(10);

fn fixture() -> PathBuf {
    let exe = std::env::current_exe().unwrap();
    let dir = exe.parent().unwrap().parent().unwrap().join("examples");
    let p = dir.join(format!(
        "{}openvrx_test_filter{}",
        std::env::consts::DLL_PREFIX,
        std::env::consts::DLL_SUFFIX
    ));
    assert!(p.exists(), "{} is missing (cargo test builds it from the examples)", p.display());
    p
}

/// A small picture with structure in every plane, as field `seq`.
fn picture(w: usize, h: usize, seq: u64) -> Frame {
    let mut f = Frame::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let k = y * w + x;
            f.y[k] = ((x * 7 + y * 13) % 17) as f32 / 16.0;
            f.u[k] = ((x * 3 + y * 5) % 11) as f32 / 25.0 - 0.2;
            f.v[k] = ((x + 2 * y) % 7) as f32 / 12.0 - 0.25;
            f.damage[k] = ((x + y) % 4 * 64) as u8;
        }
    }
    for q in &mut f.row_quality {
        q.cn_db = 12.0;
    }
    f.meta.field_seq = seq;
    f.meta.standard = Some(VideoStandard::Ntsc);
    f.meta.parity = Some(if seq % 2 == 0 { FieldParity::First } else { FieldParity::Second });
    f
}

fn plugin(config: &str) -> Plugin {
    let lib = Library::open(&fixture()).unwrap();
    Plugin::create(&lib, config).unwrap()
}

#[test]
fn abi_checks_refuse_bad_libraries() {
    let p = fixture();
    assert!(Library::open(&p).is_ok());
    let err = |sym: &str| Library::open_entry(&p, sym).unwrap_err();
    assert!(err("openvrx_test_entry_bad_abi").contains("ABI version 2"));
    assert!(err("openvrx_test_entry_short").contains("16 bytes"));
    assert!(err("openvrx_test_entry_incomplete").contains("lacks a function"));
    assert!(err("openvrx_test_entry_null").contains("no table"));
    assert!(err("no_such_entry").contains("not an OpenVRX filter plugin"));
    let missing = Library::open(Path::new("/nonexistent/libnothing.so")).unwrap_err();
    assert!(missing.contains("cannot load"), "{missing}");
}

#[test]
fn blur_matches_the_reference() {
    let mut p = plugin(r#"{"mode":"blur"}"#);
    assert_eq!(p.info().name, "test filter");
    assert_eq!(p.info().pipeline, "raw");
    let f = picture(12, 6, 0);
    let mut b = FieldBuffers::default();
    b.fill(&f, Feed::new().next(&f, false), &[]);
    assert_eq!(p.process(&mut b), Ok(true));
    let mut want = vec![0.0; 72];
    for (src, got) in [(&f.y, &b.out[0]), (&f.u, &b.out[1]), (&f.v, &b.out[2])] {
        reference_blur(src, 12, 6, &mut want);
        for (a, b) in got.iter().zip(&want) {
            assert!((a - b).abs() < 1e-6);
        }
    }
}

/// 3x3 box blur with clamped edges: nine samples, edge rows and columns
/// repeated.
fn reference_blur(src: &[f32], w: usize, h: usize, dst: &mut [f32]) {
    let at = |x: isize, y: isize| src[y.clamp(0, h as isize - 1) as usize * w + x.clamp(0, w as isize - 1) as usize];
    for y in 0..h as isize {
        for x in 0..w as isize {
            let s: f32 = (-1..=1).flat_map(|dy| (-1..=1).map(move |dx| (dx, dy))).map(|(dx, dy)| at(x + dx, y + dy)).sum();
            dst[y as usize * w + x as usize] = s / 9.0;
        }
    }
}

#[test]
fn create_errors_and_bad_info_are_reported() {
    let lib = Library::open(&fixture()).unwrap();
    let e = Plugin::create(&lib, r#"{"fail_create":"no model in /x"}"#).err().unwrap();
    assert!(e.contains("no model in /x"), "{e}");
    assert!(Plugin::create(&lib, "{not json").is_err());
    let e = Plugin::create(&lib, r#"{"pipeline":"fancy"}"#).err().unwrap();
    assert!(e.contains("pipeline"), "{e}");
    let e = Plugin::create(&lib, r#"{"aux":["envelope"]}"#).err().unwrap();
    assert!(e.contains("envelope"), "{e}");
    let ok = Plugin::create(&lib, r#"{"geometry":"steady","aux":["damage"],"provider":"cpu"}"#).unwrap();
    assert_eq!(ok.info().geometry, openvrx_filter::Geometry::Steady);
    assert_eq!(ok.info().label(), "test filter (cpu)");
    let e = Instance::start(&fixture(), r#"{"fail_create":"bad"}"#).wait_ready(READY);
    assert!(matches!(e, Phase::Failed(ref m) if m.contains("bad")), "{e:?}");
}

#[test]
fn live_filter_shows_the_plugin_picture_in_time() {
    let cfg = r#"{"mode":"probe","aux":["damage","row_cn_db"]}"#;
    let mut lf = LiveFilter::start(&fixture(), cfg, 2000.0);
    assert!(matches!(lf.wait_ready(READY), Phase::Ready(_)));
    let f0 = picture(16, 8, 41);
    let out = lf.field(&f0, true).expect("in time").clone();
    let d = abi::OPENVRX_FIELD_DISCONTINUITY as f32;
    let s = abi::OPENVRX_FIELD_STEADY_OSD as f32;
    // index, flags, parity (NTSC's second field is the top one), C/N, two
    // side planes, damage[0] / 255, size, names as asked, create thread.
    assert_eq!(&out.y[..10], &[0.0, d + s, 0.0, 12.0, 2.0, 0.0, 16.0, 8.0, 1.0, 1.0]);
    assert_eq!(&out.y[11..], &f0.y[11..]);
    assert_eq!(out.u, f0.u);
    assert_eq!(out.damage, f0.damage);
    let f1 = picture(16, 8, 42);
    let out = lf.field(&f1, false).expect("in time");
    assert_eq!(&out.y[..3], &[1.0, 0.0, 1.0]);
    let r = lf.report();
    assert_eq!(r.phase, ReportPhase::Ready);
    assert_eq!((r.fields, r.shown, r.late), (2, 2, 0));
    assert_eq!(r.timing.count, 2);
    assert!(r.osd_line().starts_with("plugin test filter "), "{}", r.osd_line());
}

#[test]
fn slow_plugin_fields_are_late_and_shown_plain() {
    let mut lf = LiveFilter::start(&fixture(), r#"{"mode":"identity","sleep_ms":60}"#, 5.0);
    assert!(matches!(lf.wait_ready(READY), Phase::Ready(_)));
    let t0 = Instant::now();
    // Deadline missed: shown plain after about 5 ms, not 60.
    assert!(lf.field(&picture(8, 4, 0), false).is_none());
    assert!(t0.elapsed() < Duration::from_millis(40), "{:?}", t0.elapsed());
    // The worker is still busy: plain at once, queued for later.
    let t1 = Instant::now();
    assert!(lf.field(&picture(8, 4, 1), false).is_none());
    assert!(t1.elapsed() < Duration::from_millis(5), "{:?}", t1.elapsed());
    let r = lf.report();
    assert_eq!((r.fields, r.shown, r.late), (2, 0, 2));
    assert!(r.busy_ms > 0.0);
    // With a generous deadline the same plugin is on time again.
    std::thread::sleep(Duration::from_millis(200));
    lf.set_deadline_ms(500.0);
    assert!(lf.field(&picture(8, 4, 2), false).is_some());
    assert_eq!(lf.report().shown, 1);
}

#[test]
fn mailbox_skips_and_flags_the_next_field() {
    let inst = Instance::start(&fixture(), r#"{"mode":"probe","sleep_ms":80}"#);
    assert!(matches!(inst.wait_ready(READY), Phase::Ready(_)));
    let mut feed = Feed::new();
    let job = |inst: &Instance, feed: &mut Feed, seq: u64| {
        let f = picture(8, 4, seq);
        let mut b = inst.buffer();
        b.fill(&f, feed.next(&f, false), &[]);
        b
    };
    assert_eq!(inst.submit(job(&inst, &mut feed, 0)), Submit::Started);
    // Once the worker holds field 0, field 1 waits in the mailbox.
    let t0 = Instant::now();
    while inst.stats().busy_ms == 0.0 {
        assert!(t0.elapsed() < Duration::from_secs(2), "the worker never took field 0");
        std::thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(inst.submit(job(&inst, &mut feed, 1)), Submit::Queued { replaced: false });
    assert_eq!(inst.submit(job(&inst, &mut feed, 2)), Submit::Queued { replaced: true });
    // Field 1 never reaches the plugin; field 2 comes with a break.
    let (b, o) = inst.wait(2, Instant::now() + Duration::from_secs(3)).expect("field 2");
    assert_eq!(o, Outcome::Written);
    assert_eq!(b.out[0][0], 2.0);
    assert_eq!(b.out[0][1], abi::OPENVRX_FIELD_DISCONTINUITY as f32);
    assert_eq!(inst.stats().skipped, 1);
    assert_eq!(inst.stats().timing.count, 2);
}

#[test]
fn process_errors_disable_the_instance() {
    let mut lf = LiveFilter::start(&fixture(), r#"{"fail_after":1}"#, 2000.0);
    assert!(matches!(lf.wait_ready(READY), Phase::Ready(_)));
    assert!(lf.field(&picture(8, 4, 0), false).is_some());
    assert!(lf.field(&picture(8, 4, 1), false).is_none());
    assert!(matches!(lf.phase(), Phase::Failed(ref m) if m.contains("-3")), "{:?}", lf.phase());
    assert!(lf.field(&picture(8, 4, 2), false).is_none());
    let r = lf.report();
    assert!(r.warning().unwrap().contains("-3"));
    assert_eq!(r.fields, 2);
    // A plugin that writes nothing: the field shows plain, counted.
    let mut pass = LiveFilter::start(&fixture(), r#"{"mode":"pass"}"#, 2000.0);
    assert!(matches!(pass.wait_ready(READY), Phase::Ready(_)));
    assert!(pass.field(&picture(8, 4, 0), false).is_none());
    assert_eq!(pass.report().not_written, 1);
}

#[test]
fn one_worker_thread_per_library() {
    let probe = |lf: &mut LiveFilter, seq: u64| {
        let o = lf.field(&picture(12, 4, seq), false).expect("in time");
        (o.y[9], o.y[10])
    };
    let mut a = LiveFilter::start(&fixture(), r#"{"mode":"probe"}"#, 2000.0);
    let mut b = LiveFilter::start(&fixture(), r#"{"mode":"probe","name":"b"}"#, 2000.0);
    assert!(matches!(a.wait_ready(READY), Phase::Ready(_)));
    assert!(matches!(b.wait_ready(READY), Phase::Ready(_)));
    let (a_same, a_thread) = probe(&mut a, 0);
    let (b_same, b_thread) = probe(&mut b, 0);
    assert_eq!((a_same, b_same), (1.0, 1.0), "process runs on the create thread");
    assert_eq!(a_thread, b_thread, "both instances share the library's worker");
    assert_eq!(b.report().name, "b");
}

/// The C example plugin of the ABI crate, built with the system compiler.
#[test]
fn c_plugin_through_the_host() {
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".into());
    let abi_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../openvrx-filter-abi");
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("openvrx-filter-c");
    std::fs::create_dir_all(&dir).unwrap();
    let lib = dir.join(format!("{}identity_c{}", std::env::consts::DLL_PREFIX, std::env::consts::DLL_SUFFIX));
    let built = Command::new(&cc)
        .args(["-std=c99", "-O2", "-Wall", "-Werror", "-shared", "-fPIC", "-fvisibility=hidden", "-I"])
        .arg(abi_dir.join("include"))
        .arg(abi_dir.join("examples/identity.c"))
        .arg("-o")
        .arg(&lib)
        .output();
    let Ok(built) = built else {
        eprintln!("skipped: no C compiler ({cc})");
        return;
    };
    assert!(built.status.success(), "{}", String::from_utf8_lossy(&built.stderr));
    let l = Library::open(&lib).unwrap();
    let e = Plugin::create(&l, "bogus").err().unwrap();
    assert!(e.contains("unknown config"), "{e}");
    let mut p = Plugin::create(&l, "gain=0.5").unwrap();
    assert_eq!(p.info().name, "identity (C)");
    let f = picture(10, 4, 0);
    let mut b = FieldBuffers::default();
    b.fill(&f, Feed::new().next(&f, false), &[]);
    assert_eq!(p.process(&mut b), Ok(true));
    for (got, src) in [(&b.out[0], &f.y), (&b.out[1], &f.u), (&b.out[2], &f.v)] {
        assert!(got.iter().zip(src.iter()).all(|(g, s)| (g - 0.5 * s).abs() < 1e-7));
    }
}
