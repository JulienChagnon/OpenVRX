// SPDX-License-Identifier: GPL-3.0-or-later
//! Minimal ffmpeg path for ordinary DVR files: probe with ffprobe, decode to
//! a raw 16-bit 4:4:4 pipe, restore, encode with x264 and copy (or
//! transcode) the original audio. ffmpeg and ffprobe run as child
//! processes; nothing is linked.
//!
//! The decode is forced to constant frame rate at the stream's average rate
//! so the restored video stays in sync with the untouched audio track.

use crate::{FrameSource, Progress, RestoreConfig, RestoreError, Restorer, RunSummary};
use openvrx_core::Frame;
use serde::Serialize;
use std::io::{Read, Write};
use std::path::Path;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::thread::JoinHandle;

/// What ffprobe says about the input.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MediaInfo {
    /// Displayed size (after rotation metadata is applied).
    pub width: usize,
    pub height: usize,
    /// Frame rate as a fraction.
    pub fps_num: u32,
    pub fps_den: u32,
    pub duration: Option<f64>,
    pub frame_count: Option<usize>,
    pub video_codec: String,
    pub audio_codec: Option<String>,
    pub rotation: i32,
}

impl MediaInfo {
    pub fn fps(&self) -> f64 {
        self.fps_num as f64 / self.fps_den.max(1) as f64
    }
}

fn ffmpeg_err(what: &str, e: std::io::Error) -> RestoreError {
    RestoreError::Ffmpeg(format!("cannot run {what} ({e}); is it installed and on PATH?"))
}

fn parse_rate(s: &str) -> Option<(u32, u32)> {
    let (n, d) = s.split_once('/').unwrap_or((s, "1"));
    let n: u32 = n.trim().parse().ok()?;
    let d: u32 = d.trim().parse().ok()?;
    (n > 0 && d > 0).then_some((n, d))
}

/// Probes a media file.
pub fn probe(path: &Path) -> Result<MediaInfo, RestoreError> {
    let out = Command::new("ffprobe")
        .args(["-v", "error", "-print_format", "json", "-show_format", "-show_streams"])
        .arg(path)
        .output()
        .map_err(|e| ffmpeg_err("ffprobe", e))?;
    if !out.status.success() {
        return Err(RestoreError::Ffmpeg(format!(
            "ffprobe failed on {}: {}",
            path.display(),
            String::from_utf8_lossy(&out.stderr).trim()
        )));
    }
    let j: serde_json::Value = serde_json::from_slice(&out.stdout)
        .map_err(|e| RestoreError::Ffmpeg(format!("ffprobe output: {e}")))?;
    let streams = j["streams"].as_array().cloned().unwrap_or_default();
    let v = streams
        .iter()
        .find(|s| s["codec_type"] == "video" && s["disposition"]["attached_pic"] != 1)
        .ok_or_else(|| RestoreError::Ffmpeg(format!("{} has no video stream", path.display())))?;
    let a = streams.iter().find(|s| s["codec_type"] == "audio");
    let num = |x: &serde_json::Value| -> Option<f64> {
        x.as_f64().or_else(|| x.as_str().and_then(|s| s.parse().ok()))
    };
    let (fps_num, fps_den) = v["avg_frame_rate"]
        .as_str()
        .and_then(parse_rate)
        .or_else(|| v["r_frame_rate"].as_str().and_then(parse_rate))
        .unwrap_or((25, 1));
    let mut rotation = v["side_data_list"]
        .as_array()
        .and_then(|l| l.iter().find_map(|sd| num(&sd["rotation"])))
        .or_else(|| num(&v["tags"]["rotate"]))
        .unwrap_or(0.0) as i32;
    rotation = rotation.rem_euclid(360);
    let mut width = num(&v["width"]).unwrap_or(0.0) as usize;
    let mut height = num(&v["height"]).unwrap_or(0.0) as usize;
    if rotation == 90 || rotation == 270 {
        std::mem::swap(&mut width, &mut height);
    }
    if width == 0 || height == 0 {
        return Err(RestoreError::Ffmpeg("video stream has no size".into()));
    }
    Ok(MediaInfo {
        width,
        height,
        fps_num,
        fps_den,
        duration: num(&v["duration"]).or_else(|| num(&j["format"]["duration"])),
        frame_count: num(&v["nb_frames"]).map(|n| n as usize),
        video_codec: v["codec_name"].as_str().unwrap_or("").to_string(),
        audio_codec: a.map(|a| a["codec_name"].as_str().unwrap_or("").to_string()),
        rotation,
    })
}

fn spawn_stderr_reader(child: &mut Child) -> Option<JoinHandle<String>> {
    let mut err = child.stderr.take()?;
    Some(std::thread::spawn(move || {
        let mut s = String::new();
        let _ = err.read_to_string(&mut s);
        s
    }))
}

// yuv444p16le, limited range: Y 16..235 and C 16..240 scaled by 256.
const Y_BLACK: f32 = 4096.0;
const Y_SPAN: f32 = 56064.0;
const C_ZERO: f32 = 32768.0;
const C_SPAN: f32 = 57344.0;
// Cb/Cr (-0.5..0.5) to BT.601 U/V scale.
const U_SCALE: f32 = 0.492 * 1.772;
const V_SCALE: f32 = 0.877 * 1.402;

/// Decodes a file to frames through an ffmpeg raw pipe.
pub struct FfmpegSource {
    rx: Receiver<Result<Frame, RestoreError>>,
    child: Child,
    reader: Option<JoinHandle<()>>,
    stderr: Option<JoinHandle<String>>,
    total: Option<usize>,
    finished: bool,
}

impl FfmpegSource {
    pub fn open(
        path: &Path,
        info: &MediaInfo,
        start: Option<f64>,
        duration: Option<f64>,
    ) -> Result<FfmpegSource, RestoreError> {
        let mut cmd = Command::new("ffmpeg");
        cmd.args(["-nostdin", "-hide_banner", "-loglevel", "error"]);
        if let Some(s) = start {
            cmd.args(["-ss", &format!("{s:.6}")]);
        }
        cmd.arg("-i").arg(path);
        if let Some(d) = duration {
            cmd.args(["-t", &format!("{d:.6}")]);
        }
        cmd.args(["-map", "0:v:0", "-an", "-sn", "-dn"])
            .args(["-vf", "scale=out_range=tv,format=yuv444p16le"])
            .args(["-fps_mode", "cfr", "-r", &format!("{}/{}", info.fps_num, info.fps_den)])
            .args(["-f", "rawvideo", "pipe:1"])
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = cmd.spawn().map_err(|e| ffmpeg_err("ffmpeg", e))?;
        let stderr = spawn_stderr_reader(&mut child);
        let mut out = child.stdout.take().expect("piped stdout");
        let (w, h) = (info.width, info.height);
        let (tx, rx) = sync_channel(3);
        let reader = std::thread::spawn(move || {
            let n = w * h;
            let mut buf = vec![0u8; n * 6];
            loop {
                match read_full(&mut out, &mut buf) {
                    Ok(false) => break,
                    Ok(true) => {}
                    Err(e) => {
                        let _ = tx.send(Err(RestoreError::Io(e)));
                        break;
                    }
                }
                let s = |k: usize| u16::from_le_bytes([buf[2 * k], buf[2 * k + 1]]) as f32;
                let mut f = Frame::new(w, h);
                for k in 0..n {
                    f.y[k] = (s(k) - Y_BLACK) / Y_SPAN;
                    f.u[k] = (s(n + k) - C_ZERO) / C_SPAN * U_SCALE;
                    f.v[k] = (s(2 * n + k) - C_ZERO) / C_SPAN * V_SCALE;
                }
                if tx.send(Ok(f)).is_err() {
                    break;
                }
            }
        });
        let span = match (info.duration, duration) {
            (Some(d), Some(t)) => Some(t.min(d - start.unwrap_or(0.0))),
            (Some(d), None) => Some(d - start.unwrap_or(0.0)),
            (None, t) => t,
        };
        let total = span.filter(|s| *s > 0.0).map(|s| (s * info.fps()).round() as usize);
        Ok(FfmpegSource {
            rx,
            child,
            reader: Some(reader),
            stderr,
            total,
            finished: false,
        })
    }

    fn finish(&mut self) -> Result<(), RestoreError> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;
        if let Some(r) = self.reader.take() {
            let _ = r.join();
        }
        let status = self.child.wait()?;
        let err = self.stderr.take().and_then(|h| h.join().ok()).unwrap_or_default();
        if !status.success() {
            return Err(RestoreError::Ffmpeg(format!("decoder failed: {}", err.trim())));
        }
        Ok(())
    }
}

/// Reads exactly `buf.len()` bytes; `Ok(false)` on a clean end of stream.
fn read_full(r: &mut impl Read, buf: &mut [u8]) -> std::io::Result<bool> {
    let mut got = 0;
    while got < buf.len() {
        match r.read(&mut buf[got..]) {
            Ok(0) if got == 0 => return Ok(false),
            Ok(0) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "truncated frame from decoder",
                ))
            }
            Ok(k) => got += k,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(true)
}

impl FrameSource for FfmpegSource {
    fn next_frame(&mut self) -> Result<Option<Frame>, RestoreError> {
        match self.rx.recv() {
            Ok(Ok(f)) => Ok(Some(f)),
            Ok(Err(e)) => Err(e),
            Err(_) => {
                self.finish()?;
                Ok(None)
            }
        }
    }
    fn len_hint(&self) -> Option<usize> {
        self.total
    }
}

impl Drop for FfmpegSource {
    fn drop(&mut self) {
        if !self.finished {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

/// Encodes frames with x264 (yuv420p) and muxes the input's audio.
pub struct FfmpegSink {
    tx: Option<SyncSender<Vec<u8>>>,
    writer: Option<JoinHandle<std::io::Result<()>>>,
    child: Child,
    stderr: Option<JoinHandle<String>>,
    width: usize,
    height: usize,
    out_w: usize,
    out_h: usize,
}

fn audio_args(output: &Path, codec: &str) -> Vec<String> {
    let ext = output
        .extension()
        .map(|e| e.to_string_lossy().to_ascii_lowercase())
        .unwrap_or_default();
    let copy = match ext.as_str() {
        "mkv" => true,
        "mp4" | "m4v" | "mov" => matches!(codec, "aac" | "mp3" | "alac" | "ac3" | "eac3" | "opus"),
        _ => codec == "aac",
    };
    if copy {
        vec!["-c:a".into(), "copy".into()]
    } else {
        vec!["-c:a".into(), "aac".into(), "-b:a".into(), "192k".into()]
    }
}

impl FfmpegSink {
    pub fn create(
        output: &Path,
        info: &MediaInfo,
        audio_from: Option<&Path>,
        cfg: &RestoreConfig,
    ) -> Result<FfmpegSink, RestoreError> {
        let (w, h) = (info.width, info.height);
        let (out_w, out_h) = (w + w % 2, h + h % 2);
        let mut cmd = Command::new("ffmpeg");
        cmd.args(["-nostdin", "-hide_banner", "-loglevel", "error", "-y"])
            .args(["-f", "rawvideo", "-pix_fmt", "yuv444p16le"])
            .args(["-s", &format!("{out_w}x{out_h}")])
            .args(["-framerate", &format!("{}/{}", info.fps_num, info.fps_den)])
            .args(["-i", "pipe:0"]);
        let audio = audio_from.zip(info.audio_codec.as_deref());
        if let Some((src, _)) = audio {
            if let Some(s) = cfg.encode.start {
                cmd.args(["-ss", &format!("{s:.6}")]);
            }
            if let Some(d) = cfg.encode.duration {
                cmd.args(["-t", &format!("{d:.6}")]);
            }
            cmd.arg("-i").arg(src);
        }
        cmd.args(["-map", "0:v:0"]);
        if let Some((_, codec)) = audio {
            cmd.args(["-map", "1:a:0"]).args(audio_args(output, codec)).arg("-shortest");
        }
        cmd.args(["-c:v", "libx264", "-pix_fmt", "yuv420p"])
            .args(["-crf", &cfg.encode.crf.to_string(), "-preset", &cfg.encode.preset])
            .args(["-color_range", "tv"]);
        let ext = output.extension().map(|e| e.to_string_lossy().to_ascii_lowercase());
        if matches!(ext.as_deref(), Some("mp4" | "m4v" | "mov")) {
            cmd.args(["-movflags", "+faststart"]);
        }
        cmd.arg(output).stdin(Stdio::piped()).stdout(Stdio::null()).stderr(Stdio::piped());
        let mut child = cmd.spawn().map_err(|e| ffmpeg_err("ffmpeg", e))?;
        let stderr = spawn_stderr_reader(&mut child);
        let stdin: ChildStdin = child.stdin.take().expect("piped stdin");
        let (tx, rx) = sync_channel::<Vec<u8>>(3);
        let writer = std::thread::spawn(move || {
            let mut stdin = stdin;
            for buf in rx {
                stdin.write_all(&buf)?;
            }
            stdin.flush()
        });
        Ok(FfmpegSink {
            tx: Some(tx),
            writer: Some(writer),
            child,
            stderr,
            width: w,
            height: h,
            out_w,
            out_h,
        })
    }

    /// Queues one frame (edges replicated when the size is odd).
    pub fn write(&mut self, f: &Frame) -> Result<(), RestoreError> {
        let (w, h, ow, oh) = (self.width, self.height, self.out_w, self.out_h);
        let n = ow * oh;
        let mut buf = vec![0u8; n * 6];
        for (pi, (plane, zero, span)) in [
            (&f.y, Y_BLACK, Y_SPAN),
            (&f.u, C_ZERO, C_SPAN / U_SCALE),
            (&f.v, C_ZERO, C_SPAN / V_SCALE),
        ]
        .into_iter()
        .enumerate()
        {
            let base = pi * n * 2;
            for yy in 0..oh {
                let sy = yy.min(h - 1);
                for xx in 0..ow {
                    let sx = xx.min(w - 1);
                    let v = (plane[sy * w + sx] * span + zero).round().clamp(0.0, 65535.0) as u16;
                    let k = base + 2 * (yy * ow + xx);
                    buf[k..k + 2].copy_from_slice(&v.to_le_bytes());
                }
            }
        }
        match &self.tx {
            Some(tx) if tx.send(buf).is_ok() => Ok(()),
            _ => self.close().and_then(|_| {
                Err(RestoreError::Ffmpeg("encoder stopped accepting frames".into()))
            }),
        }
    }

    /// Ends the stream and waits for the encoder.
    pub fn close(&mut self) -> Result<(), RestoreError> {
        drop(self.tx.take());
        let wres = self.writer.take().map(|h| h.join());
        let status = self.child.wait()?;
        let err = self.stderr.take().and_then(|h| h.join().ok()).unwrap_or_default();
        if !status.success() {
            return Err(RestoreError::Ffmpeg(format!("encoder failed: {}", err.trim())));
        }
        if let Some(Ok(Err(e))) = wres {
            return Err(RestoreError::Io(e));
        }
        Ok(())
    }
}

impl Drop for FfmpegSink {
    fn drop(&mut self) {
        if self.writer.is_some() {
            drop(self.tx.take());
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

/// Result of restoring a file.
#[derive(Clone, Debug)]
pub struct FileSummary {
    pub info: MediaInfo,
    pub run: RunSummary,
}

/// Restores `input` into `output` (x264, audio kept). Trimming and
/// encoder settings come from `cfg.encode`.
pub fn restore_file<P: FnMut(&Progress)>(
    input: &Path,
    output: &Path,
    cfg: &RestoreConfig,
    progress: P,
) -> Result<FileSummary, RestoreError> {
    let restorer = Restorer::new(cfg.clone())?;
    let info = probe(input)?;
    let mut src = FfmpegSource::open(input, &info, cfg.encode.start, cfg.encode.duration)?;
    let mut sink = FfmpegSink::create(output, &info, Some(input), cfg)?;
    let run = restorer.run(&mut src, |f, _| sink.write(&f), progress);
    let closed = sink.close();
    let run = run?;
    closed?;
    if run.frames == 0 {
        return Err(RestoreError::Ffmpeg("no frames decoded".into()));
    }
    Ok(FileSummary { info, run })
}
