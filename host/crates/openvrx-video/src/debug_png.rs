// SPDX-License-Identifier: GPL-3.0-or-later
//! PNG dumps of frames and composite fields for debugging (feature
//! `debug-png`).

use openvrx_core::{Field, Frame};
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

fn write_rgba(path: &Path, width: usize, height: usize, rgba: &[u8]) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut enc = png::Encoder::new(BufWriter::new(file), width as u32, height as u32);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    let mut w = enc.write_header().map_err(std::io::Error::other)?;
    w.write_image_data(rgba).map_err(std::io::Error::other)?;
    Ok(())
}

/// Writes a frame as an RGB PNG (BT.601).
pub fn write_frame(path: impl AsRef<Path>, frame: &Frame) -> std::io::Result<()> {
    write_rgba(path.as_ref(), frame.width, frame.height, &frame.to_rgba8())
}

/// Writes a frame's damage mask as a grey PNG (white = damaged).
pub fn write_damage(path: impl AsRef<Path>, frame: &Frame) -> std::io::Result<()> {
    let mut rgba = Vec::with_capacity(frame.damage.len() * 4);
    for &d in &frame.damage {
        rgba.extend_from_slice(&[d, d, d, 255]);
    }
    write_rgba(path.as_ref(), frame.width, frame.height, &rgba)
}

/// Writes a composite field (whole lines, IRE -40..120 mapped to grey).
pub fn write_field(path: impl AsRef<Path>, field: &Field) -> std::io::Result<()> {
    let mut rgba = Vec::with_capacity(field.video.len() * 4);
    for &v in &field.video {
        let g = (((v + 45.0) / 165.0).clamp(0.0, 1.0) * 255.0 + 0.5) as u8;
        rgba.extend_from_slice(&[g, g, g, 255]);
    }
    write_rgba(path.as_ref(), field.width, field.lines, &rgba)
}
