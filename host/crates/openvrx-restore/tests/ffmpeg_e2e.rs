// SPDX-License-Identifier: GPL-3.0-or-later
//! End to end: write a damaged clip with audio through ffmpeg, restore it
//! with the blind estimate, and check the output.

mod common;

use common::{quality, truth_sequence, Rng};
use openvrx_core::Frame;
use openvrx_restore::ffmpeg::{probe, restore_file, FfmpegSink, FfmpegSource, MediaInfo};
use openvrx_restore::{FrameSource, RestoreConfig};
use std::process::Command;

fn decode_all(path: &std::path::Path) -> Vec<Frame> {
    let info = probe(path).unwrap();
    let mut src = FfmpegSource::open(path, &info, None, None).unwrap();
    let mut out = Vec::new();
    while let Some(f) = src.next_frame().unwrap() {
        out.push(f);
    }
    out
}

#[test]
fn restore_file_end_to_end() {
    let dir = std::env::temp_dir().join(format!("openvrx-restore-e2e-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (w, h, n) = (320usize, 240usize, 30usize);
    let truth = truth_sequence(w, h, n);

    // Blind damage: noise line bursts on a few frames, one static frame.
    let mut rng = Rng(42);
    let mut damaged = truth.clone();
    for (k, f) in damaged.iter_mut().enumerate() {
        if k == 12 {
            f.y.iter_mut().for_each(|v| *v = rng.f32());
        } else if k % 4 == 1 {
            let r0 = rng.range(20, h - 20);
            for r in r0..r0 + 6 {
                for x in 0..w {
                    f.y[r * w + x] = rng.f32();
                }
            }
        }
    }

    let audio = dir.join("tone.wav");
    let st = Command::new("ffmpeg")
        .args(["-v", "error", "-y", "-f", "lavfi", "-i", "sine=frequency=440:duration=1.2"])
        .arg(&audio)
        .status()
        .unwrap();
    assert!(st.success());

    let input = dir.join("damaged.mp4");
    let info = MediaInfo {
        width: w,
        height: h,
        fps_num: 25,
        fps_den: 1,
        duration: None,
        frame_count: None,
        video_codec: String::new(),
        audio_codec: Some("pcm_s16le".into()),
        rotation: 0,
    };
    let mut enc_cfg = RestoreConfig::balanced();
    enc_cfg.encode.crf = 6;
    let mut sink = FfmpegSink::create(&input, &info, Some(&audio), &enc_cfg).unwrap();
    for f in &damaged {
        sink.write(f).unwrap();
    }
    sink.close().unwrap();

    let output = dir.join("restored.mp4");
    let mut cfg = RestoreConfig::balanced();
    cfg.encode.crf = 6;
    let mut calls = 0;
    let summary = restore_file(&input, &output, &cfg, |_| calls += 1).unwrap();
    assert_eq!(summary.run.frames, n);
    assert_eq!(calls, n);
    assert!(summary.run.stats[12].interpolated, "static frame not rebuilt");

    let out_info = probe(&output).unwrap();
    assert_eq!((out_info.width, out_info.height), (w, h));
    assert_eq!((out_info.fps_num, out_info.fps_den), (25, 1));
    assert_eq!(out_info.audio_codec.as_deref(), Some("aac"));

    let before = decode_all(&input);
    let after = decode_all(&output);
    assert_eq!(before.len(), n);
    assert_eq!(after.len(), n);
    let (bp, bs) = quality(&before, &truth);
    let (ap, as_) = quality(&after, &truth);
    println!("file: {bp:.1} dB / {bs:.3} -> {ap:.1} dB / {as_:.3}, {:.1} fps", summary.run.fps());
    assert!(ap > bp + 8.0, "PSNR {bp:.2} -> {ap:.2}");
    assert!(as_ > bs);
    let _ = std::fs::remove_dir_all(&dir);
}
