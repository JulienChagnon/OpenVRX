// SPDX-License-Identifier: GPL-3.0-or-later
//! End-to-end tests of the recording format: write, crash, read, seek,
//! replay, rolling buffer, cut.

use openvrx_core::{LanePreset, RawBlock, SampleFormat};
use openvrx_format::*;
use std::sync::Arc;

fn sidecar(n_rx: u8, preset: LanePreset, truth: bool) -> Sidecar {
    let receivers = (0..n_rx)
        .map(|rx| {
            let mut r = ReceiverInfo::new(rx, SampleFormat::from_preset(preset, 18));
            r.channel_mhz = 5732;
            r.gain = 64;
            r.start_host_ns = 1_000_000_000;
            r
        })
        .collect();
    let mut s = Sidecar::new(
        if truth {
            RecordingSource::Sim
        } else {
            RecordingSource::Hardware
        },
        receivers,
    );
    if truth {
        s.sim = Some(SimInfo {
            truth: Some(TruthInfo {
                file: String::new(),
                rx: 0,
                unit: "ire".into(),
                delay_samples: 2.5,
            }),
            settings: serde_json::Map::new(),
        });
    }
    s
}

fn pattern(rx: u8, n: usize, offset: usize) -> Vec<u8> {
    (0..n).map(|k| ((k + offset) as u32 * 7 + rx as u32 * 13) as u8).collect()
}

#[test]
fn write_close_read_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("run");
    let mut w = RecordingWriter::create(&base, sidecar(2, LanePreset::I8Q8, true), WriterOptions::default()).unwrap();
    for k in 0..10 {
        w.write_raw(0, &pattern(0, 4000, k * 4000)).unwrap();
        w.write_raw(1, &pattern(1, 2000, k * 2000)).unwrap();
    }
    w.log_event(&Event::new(Some(0), 5, Some(800), EventKind::Gain { index: 68 }))
        .unwrap();
    w.log_event(&Event::new(None, 6, None, EventKind::Marker { text: "go".into() }))
        .unwrap();
    let truth: Vec<f32> = (0..20000).map(|k| k as f32 * 0.5).collect();
    w.write_truth(&truth).unwrap();
    let sc = w.close().unwrap();
    assert!(sc.closed_cleanly);
    assert_eq!(sc.receivers[0].bytes, 40000);
    assert_eq!(sc.receivers[0].file, "run.rx0.bin");

    for opener in ["run.openvrx.json", "run.rx1.bin", "run.events.jsonl", "run.truth.f32", "run"] {
        let rec = Recording::open(dir.path().join(opener)).unwrap();
        assert_eq!(rec.name(), "run");
        assert_eq!(rec.rx_bytes(0).unwrap(), 40000);
        assert_eq!(rec.rx_bytes(1).unwrap(), 20000);
        assert_eq!(rec.rx_samples(0).unwrap(), 20000);
    }
    let rec = Recording::open(&base).unwrap();
    assert_eq!(rec.events().events.len(), 2);
    assert_eq!(rec.sidecar().receivers[0].gain, 64);
    let mut r = rec.raw_reader(0).unwrap();
    let mut all = vec![0u8; 40000];
    assert_eq!(r.read(&mut all).unwrap(), 40000);
    let expect: Vec<u8> = (0..10).flat_map(|k| pattern(0, 4000, k * 4000)).collect();
    assert_eq!(all, expect);
    let mut t = rec.truth().unwrap();
    assert_eq!(t.len(), 20000);
    assert_eq!(t.info().delay_samples, 2.5);
    assert_eq!(t.read_range(100, 3).unwrap(), vec![50.0, 50.5, 51.0]);
    assert_eq!(t.read_all().unwrap(), truth);
    // Refuses to clobber.
    assert!(matches!(
        RecordingWriter::create(&base, sidecar(1, LanePreset::I8Q8, false), WriterOptions::default()),
        Err(FormatError::Exists(_))
    ));
}

#[test]
fn unclosed_recording_reads_from_file_sizes() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("crash");
    let mut w = RecordingWriter::create(&base, sidecar(1, LanePreset::I7Q7, false), WriterOptions::default()).unwrap();
    w.write_raw(0, &pattern(0, 10001, 0)).unwrap();
    w.log_event(&Event::new(Some(0), 1, Some(10), EventKind::Slip { count: 1 }))
        .unwrap();
    w.flush().unwrap();
    // Simulate a crash: the process dies without close or drop.
    std::mem::forget(w);
    // Append a partial event line as a crash mid-write would.
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .append(true)
        .open(dir.path().join("crash.events.jsonl"))
        .unwrap();
    f.write_all(br#"{"rx":0,"host_ns":9,"kind":"ga"#).unwrap();
    drop(f);
    let rec = Recording::open(&base).unwrap();
    assert!(!rec.sidecar().closed_cleanly);
    assert_eq!(rec.sidecar().receivers[0].bytes, 0);
    // Whole words only.
    assert_eq!(rec.rx_bytes(0).unwrap(), 10000);
    assert_eq!(rec.events().events.len(), 1);
    assert_eq!(rec.events().skipped.len(), 1);
}

#[test]
fn drop_without_close_checkpoints() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("dropped");
    {
        let mut w = RecordingWriter::create(&base, sidecar(1, LanePreset::I8Q8, false), WriterOptions::default()).unwrap();
        w.write_raw(0, &pattern(0, 600, 0)).unwrap();
    }
    let rec = Recording::open(&base).unwrap();
    assert!(!rec.sidecar().closed_cleanly);
    assert_eq!(rec.sidecar().receivers[0].bytes, 600);
    assert_eq!(rec.rx_bytes(0).unwrap(), 600);
}

#[test]
fn seek_by_byte_sample_time_and_gaps() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("seek");
    let mut w = RecordingWriter::create(&base, sidecar(1, LanePreset::I8Q8, false), WriterOptions::default()).unwrap();
    let rate = 240e6 / 18.0 * 2.0;
    // 0.01 s of data, then a block that skips 0.005 s worth of bytes.
    let n1 = (rate * 0.01) as u64;
    let n1 = n1 - n1 % 2;
    let gap = (rate * 0.005) as u64;
    let gap = gap - gap % 2;
    let b1 = RawBlock { rx: 0, seq: 0, first_byte: 1000, host_ns: 7, data: pattern(0, n1 as usize, 0) };
    let b2 = RawBlock { rx: 0, seq: 2, first_byte: 1000 + n1 + gap, host_ns: 8, data: pattern(0, 40000, n1 as usize) };
    w.write_block(&b1).unwrap();
    w.write_block(&b2).unwrap();
    // A duplicate overlap is skipped.
    let dup = RawBlock { rx: 0, seq: 3, first_byte: 1000 + n1 + gap + 39000, host_ns: 9, data: pattern(0, 3000, n1 as usize + 39000) };
    assert_eq!(w.write_block(&dup).unwrap(), 2000);
    assert_eq!(w.file_byte_for_stream(0, 1000 + n1 + gap + 10), Some(n1 + 10));
    assert_eq!(w.file_byte_for_stream(0, 999), None);
    w.close().unwrap();

    let rec = Recording::open(&base).unwrap();
    let gaps: Vec<_> = rec
        .events()
        .events
        .iter()
        .filter(|e| matches!(e.kind, EventKind::UsbGap { .. }))
        .collect();
    assert_eq!(gaps.len(), 1);
    assert_eq!(gaps[0].byte, Some(n1));
    let tl = rec.timeline(0).unwrap();
    assert!((tl.time_of_byte(n1 / 2) - 0.005).abs() < 1e-6);
    assert!((tl.time_of_byte(n1) - 0.015).abs() < 1e-6, "{}", tl.time_of_byte(n1));
    assert_eq!(tl.byte_of_time(0.012), n1, "inside the gap -> where data resumes");
    assert!((rec.rx_duration_s(0).unwrap() - (0.015 + 42000.0 / rate)).abs() < 1e-6);

    let mut r = rec.raw_reader(0).unwrap();
    assert_eq!(r.seek_byte(1001).unwrap(), 1000);
    assert_eq!(r.seek_sample(250).unwrap(), 250);
    assert_eq!(r.position_bytes(), 500);
    let mut b = [0u8; 4];
    r.read(&mut b).unwrap();
    assert_eq!(b.to_vec(), pattern(0, 4, 500));
    let at = r.seek_time(0.016).unwrap();
    let want = n1 + ((0.001 * rate) as u64 & !1);
    assert!(at.abs_diff(want) <= 2, "{at} vs {want}");
    assert!((r.position_time() - 0.016).abs() < 1e-6);
    assert_eq!(r.seek_byte(u64::MAX).unwrap(), rec.rx_bytes(0).unwrap());
    assert!(r.read_block(4096).unwrap().is_none());
}

#[test]
fn chunked_iteration_and_iq() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("chunks");
    let mut w = RecordingWriter::create(&base, sidecar(1, LanePreset::I8Q8, false), WriterOptions::default()).unwrap();
    let f = LanePreset::I8Q8.format();
    let words: Vec<u8> = (0..1000)
        .flat_map(|k| (f.encode(k % 400 - 200, 100 - k % 200) as u16).to_le_bytes())
        .collect();
    w.write_raw(0, &words).unwrap();
    w.close().unwrap();
    let rec = Recording::open(&base).unwrap();
    let blocks: Vec<RawBlock> = rec
        .raw_reader(0)
        .unwrap()
        .blocks(301)
        .collect::<Result<_>>()
        .unwrap();
    assert_eq!(blocks[0].data.len(), 300);
    assert_eq!(blocks.iter().map(|b| b.data.len()).sum::<usize>(), 2000);
    for (k, b) in blocks.iter().enumerate() {
        assert_eq!(b.seq, k as u64);
        assert_eq!(b.first_byte, 300 * k as u64);
        assert!(b.host_ns >= 1_000_000_000);
    }
    let mut r = rec.raw_reader(0).unwrap();
    r.seek_sample(10).unwrap();
    let iq = r.read_iq(5).unwrap();
    for (k, z) in iq.iter().enumerate() {
        let n = 10 + k as i32;
        assert!((z.re - (n % 400 - 200) as f32).abs() <= 2.0);
        assert!((z.im - (100 - n % 200) as f32).abs() <= 2.0);
    }
}

#[test]
fn replay_interleaves_and_paces() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("rep");
    let mut sc = sidecar(2, LanePreset::I8Q8, false);
    // rx1 at half the rate (8-bit words at the same divider).
    sc.receivers[1].sample_format = SampleFormat::from_preset(LanePreset::I4Q4Top, 18);
    let mut w = RecordingWriter::create(&base, sc, WriterOptions::default()).unwrap();
    let rate0 = 240e6 / 18.0 * 2.0;
    let n0 = (rate0 * 0.02) as usize & !1;
    w.write_raw(0, &pattern(0, n0, 0)).unwrap();
    w.write_raw(1, &pattern(1, n0 / 2, 0)).unwrap();
    w.close().unwrap();
    let rec = Recording::open(&base).unwrap();

    // As fast as possible: blocks come in stream-time order.
    let blocks: Vec<RawBlock> = rec
        .replay(ReplayOptions { block_bytes: 16384, ..ReplayOptions::unpaced() })
        .unwrap()
        .collect::<Result<_>>()
        .unwrap();
    let total: usize = blocks.iter().map(|b| b.data.len()).sum();
    assert_eq!(total, n0 + n0 / 2);
    let mut last_t = -1.0;
    for b in &blocks {
        let wb = if b.rx == 0 { 2.0 } else { 1.0 };
        let t = b.first_byte as f64 / (240e6 / 18.0 * wb);
        assert!(t + 1e-9 >= last_t, "out of order");
        last_t = t;
    }

    // Paced at 2x: 20 ms of stream takes about 10 ms.
    let start = std::time::Instant::now();
    let n = rec
        .replay(ReplayOptions { speed: Some(2.0), ..Default::default() })
        .unwrap()
        .count();
    let el = start.elapsed().as_secs_f64();
    assert!(n > 2);
    assert!((0.009..0.2).contains(&el), "{el}");

    // Range and looping.
    let mut rep = rec
        .replay(ReplayOptions {
            start_s: 0.005,
            end_s: Some(0.010),
            looping: true,
            receivers: Some(vec![0]),
            ..ReplayOptions::unpaced()
        })
        .unwrap();
    let span = ((0.010 * rate0) as u64 & !1) - ((0.005 * rate0) as u64 & !1);
    let mut got = 0u64;
    let mut expect_first = (0.005 * rate0) as u64 & !1;
    while got < 3 * span {
        let b = rep.next().unwrap().unwrap();
        assert_eq!(b.first_byte, expect_first, "stream positions continue across loops");
        expect_first += b.data.len() as u64;
        got += b.data.len() as u64;
    }
    assert!(rep.position_s() >= 0.0149);
}

#[test]
fn rolling_buffer_bounds_and_drain() {
    let dir = tempfile::tempdir().unwrap();
    // 1 s of history at 100 kB/s.
    let rb = RollingBuffer::new(&[(0, 100_000.0), (1, 50_000.0)], &RollingConfig { seconds: 1.0, ..Default::default() });
    assert_eq!(rb.budget_bytes(0), 100_000);
    let mut pos = [0u64; 2];
    let mut seq = 0;
    let push = |rb: &RollingBuffer, rx: u8, pos: &mut [u64; 2], seq: &mut u64| {
        let n = 10_000usize;
        let mut data = rb.take_spare(rx, n);
        data.extend_from_slice(&pattern(rx, n, pos[rx as usize] as usize));
        rb.push(RawBlock { rx, seq: *seq, first_byte: pos[rx as usize], host_ns: *seq * 1000, data });
        pos[rx as usize] += n as u64;
        *seq += 1;
    };
    for _ in 0..50 {
        push(&rb, 0, &mut pos, &mut seq);
        push(&rb, 1, &mut pos, &mut seq);
    }
    assert!(rb.buffered_bytes(0) <= 100_000);
    assert_eq!(rb.buffered_bytes(0), 100_000);
    assert_eq!(rb.buffered_bytes(1), 50_000);
    assert_eq!(rb.buffered_range(0), Some(400_000..500_000));
    // Events in stream coordinates: one before the history, one inside.
    rb.push_event(Event::new(Some(0), 1, Some(10_000), EventKind::Gain { index: 60 }));
    rb.push_event(Event::new(Some(0), 2, Some(450_000), EventKind::Gain { index: 66 }));

    let base = dir.path().join("rolled");
    let mut w = RecordingWriter::create(&base, sidecar(2, LanePreset::I8Q8, false), WriterOptions::default()).unwrap();
    let s = rb.drain_into(&mut w).unwrap();
    assert!(rb.is_recording());
    assert_eq!(s.bytes, 150_000);
    assert_eq!(s.events, 1);
    // Live continuation: only new blocks.
    for _ in 0..3 {
        push(&rb, 0, &mut pos, &mut seq);
    }
    let s2 = rb.drain_into(&mut w).unwrap();
    assert_eq!(s2.bytes, 30_000);
    assert_eq!(s2.evicted_unwritten, 0);
    // Recorder falls behind by more than the buffer: a gap is logged.
    for _ in 0..15 {
        push(&rb, 0, &mut pos, &mut seq);
    }
    let s3 = rb.drain_into(&mut w).unwrap();
    assert_eq!(s3.evicted_unwritten, 5);
    assert_eq!(s3.bytes, 100_000);
    w.close().unwrap();
    rb.stop_recording();
    assert!(!rb.is_recording());

    let rec = Recording::open(&base).unwrap();
    assert_eq!(rec.rx_bytes(0).unwrap(), 230_000);
    let mut r = rec.raw_reader(0).unwrap();
    let mut first = vec![0u8; 100];
    r.read(&mut first).unwrap();
    assert_eq!(first, pattern(0, 100, 400_000));
    let gain = rec
        .events()
        .events
        .iter()
        .find(|e| matches!(e.kind, EventKind::Gain { .. }))
        .unwrap();
    assert_eq!(gain.byte, Some(50_000));
    let gap = rec
        .events()
        .events
        .iter()
        .find_map(|e| match e.kind {
            EventKind::UsbGap { lost_bytes, .. } => Some((e.byte, lost_bytes)),
            _ => None,
        })
        .unwrap();
    assert_eq!(gap, (Some(130_000), 50_000));
}

#[test]
fn rolling_push_is_cheap_and_shared() {
    let rb = Arc::new(RollingBuffer::new(&[(0, 26.67e6)], &RollingConfig { seconds: 0.5, ..Default::default() }));
    let rb2 = rb.clone();
    let t = std::thread::spawn(move || {
        let mut pos = 0u64;
        let start = std::time::Instant::now();
        for seq in 0..2000u64 {
            let mut data = rb2.take_spare(0, 16384);
            data.resize(16384, (seq & 0xff) as u8);
            rb2.push(RawBlock { rx: 0, seq, first_byte: pos, host_ns: seq, data });
            pos += 16384;
        }
        start.elapsed()
    });
    let el = t.join().unwrap();
    // 2000 x 16 KiB = 32 MB pushed: well under a second including memset.
    assert!(el.as_secs_f64() < 1.0, "{el:?}");
    assert!(rb.buffered_bytes(0) <= rb.budget_bytes(0));
    assert!(rb.buffered_bytes(0) + 16384 > rb.budget_bytes(0));
    let snap = rb.snapshot(0);
    assert_eq!(snap.last().unwrap().seq, 1999);
}

#[test]
fn extract_and_split_keep_data_events_and_truth() {
    let dir = tempfile::tempdir().unwrap();
    let base = dir.path().join("long");
    let mut w = RecordingWriter::create(&base, sidecar(2, LanePreset::I8Q8, true), WriterOptions::default()).unwrap();
    let rate = 240e6 / 18.0 * 2.0;
    let n = (rate * 0.03) as usize & !1;
    w.write_raw(0, &pattern(0, n, 0)).unwrap();
    w.write_raw(1, &pattern(1, n, 0)).unwrap();
    let truth: Vec<f32> = (0..n / 2).map(|k| k as f32).collect();
    w.write_truth(&truth).unwrap();
    let b_gain = (rate * 0.005) as u64 & !1;
    let b_mark = (rate * 0.015) as u64 & !1;
    w.log_event(&Event::new(Some(0), 1_005_000_000, Some(b_gain), EventKind::Gain { index: 70 }))
        .unwrap();
    w.log_event(&Event::new(Some(1), 1_015_000_000, Some(b_mark), EventKind::Tune { mhz: 5800, offset_khz: 5 }))
        .unwrap();
    w.log_event(&Event::new(None, 1_016_000_000, None, EventKind::Marker { text: "x".into() }))
        .unwrap();
    w.log_event(&Event::new(None, 1_029_000_000, None, EventKind::Marker { text: "late".into() }))
        .unwrap();
    w.close().unwrap();
    let rec = Recording::open(&base).unwrap();

    let cut = extract_range(&rec, dir.path().join("cut"), 0.010, Some(0.020), WriterOptions::default()).unwrap();
    let b0 = (rate * 0.010) as u64 & !1;
    let b1 = (rate * 0.020) as u64 & !1;
    assert_eq!(cut.rx_bytes(0).unwrap(), b1 - b0);
    assert!(cut.sidecar().notes.contains("cut from long"));
    let r0 = cut.receiver(0).unwrap();
    assert_eq!(r0.gain, 70, "gain in force at the cut");
    assert!(r0.start_host_ns.abs_diff(1_010_000_000) < 1000);
    let mut r = cut.raw_reader(0).unwrap();
    let mut head = vec![0u8; 64];
    r.read(&mut head).unwrap();
    assert_eq!(head, pattern(0, 64, b0 as usize));
    let kinds: Vec<_> = cut.events().events.iter().map(|e| (e.kind.name().to_string(), e.byte)).collect();
    assert_eq!(kinds, vec![("tune".to_string(), Some(b_mark - b0)), ("marker".to_string(), None)]);
    let mut t = cut.truth().unwrap();
    assert_eq!(t.len(), (b1 - b0) / 2);
    assert_eq!(t.read_range(0, 1).unwrap()[0], (b0 / 2) as f32);

    let pieces = split(&rec, dir.path().join("pieces"), 0.012, WriterOptions::default()).unwrap();
    assert_eq!(pieces.len(), 3);
    let total: u64 = pieces.iter().map(|p| p.rx_bytes(1).unwrap()).sum();
    assert_eq!(total, n as u64);
    assert_eq!(pieces[2].name(), "long_002");

    let (list, bad) = list_recordings(dir.path()).unwrap();
    assert!(bad.is_empty());
    let names: Vec<_> = list.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, vec!["cut", "long"]);
    let s = &list[1];
    assert!(s.has_truth);
    assert!((s.duration_s - n as f64 / rate).abs() < 1e-9);
    assert_eq!(s.event_counts["marker"], 2);
    let text = s.to_string();
    assert!(text.contains("rx1: 8x8 /18 13.333 MS/s"), "{text}");
    assert!(extract_range(&rec, dir.path().join("bad"), 0.02, Some(0.01), WriterOptions::default()).is_err());
}
