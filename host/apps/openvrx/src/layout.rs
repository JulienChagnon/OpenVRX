// SPDX-License-Identifier: GPL-3.0-or-later
//! View placement: split-screen cells, aspect ratio, overscan, and
//! non-square output pixels (the HD3 shows 1280x720 on an 800x480 area).

use crate::settings::{AspectMode, OutputAspect, ViewLayout};

/// Axis-aligned rectangle in pixels.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Rect {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl Rect {
    pub const fn new(x: f32, y: f32, w: f32, h: f32) -> Rect {
        Rect { x, y, w, h }
    }
    pub fn right(&self) -> f32 {
        self.x + self.w
    }
    pub fn bottom(&self) -> f32 {
        self.y + self.h
    }
    pub fn is_empty(&self) -> bool {
        self.w < 1.0 || self.h < 1.0
    }
    /// Snaps to whole pixels (edges rounded).
    pub fn snapped(&self) -> Rect {
        let x0 = self.x.round();
        let y0 = self.y.round();
        let x1 = self.right().round();
        let y1 = self.bottom().round();
        Rect::new(x0, y0, (x1 - x0).max(0.0), (y1 - y0).max(0.0))
    }
}

/// Gap between split-screen cells, pixels.
pub const CELL_GAP: f32 = 2.0;

/// Splits `area` into the layout's cells. Two views sit side by side on a
/// wide area and stacked on a tall one; four views form a 2x2 grid.
/// Returns the number of cells written.
pub fn view_cells(area: Rect, layout: ViewLayout, gap: f32, out: &mut [Rect; 4]) -> usize {
    let n = layout.count();
    match layout {
        ViewLayout::Single => out[0] = area,
        ViewLayout::Dual => {
            if area.w >= area.h {
                let w = (area.w - gap) / 2.0;
                out[0] = Rect::new(area.x, area.y, w, area.h);
                out[1] = Rect::new(area.x + w + gap, area.y, w, area.h);
            } else {
                let h = (area.h - gap) / 2.0;
                out[0] = Rect::new(area.x, area.y, area.w, h);
                out[1] = Rect::new(area.x, area.y + h + gap, area.w, h);
            }
        }
        ViewLayout::Quad => {
            let w = (area.w - gap) / 2.0;
            let h = (area.h - gap) / 2.0;
            out[0] = Rect::new(area.x, area.y, w, h);
            out[1] = Rect::new(area.x + w + gap, area.y, w, h);
            out[2] = Rect::new(area.x, area.y + h + gap, w, h);
            out[3] = Rect::new(area.x + w + gap, area.y + h + gap, w, h);
        }
    }
    for c in out.iter_mut().take(n) {
        *c = c.snapped();
    }
    n
}

/// Physical width of an output pixel relative to its height.
pub fn pixel_aspect(output: OutputAspect, target_w: f32, target_h: f32) -> f32 {
    match output.ratio() {
        Some(r) if target_w > 0.0 && target_h > 0.0 => r / (target_w / target_h),
        _ => 1.0,
    }
}

/// Where the picture goes and which part of the source it shows.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct PictureGeometry {
    /// Destination rectangle, pixels.
    pub dest: Rect,
    /// Source window in normalised texture coordinates: u0, v0, u1, v1.
    pub uv: [f32; 4],
}

/// Places a picture of display aspect `picture_aspect` in `cell`.
/// `overscan_percent` is cropped from every edge first.
pub fn fit_picture(
    cell: Rect,
    picture_aspect: f32,
    pixel_aspect: f32,
    mode: AspectMode,
    overscan_percent: f32,
) -> PictureGeometry {
    let c = (overscan_percent / 100.0).clamp(0.0, 0.45);
    let base = [c, c, 1.0 - c, 1.0 - c];
    if cell.is_empty() || picture_aspect <= 0.0 || pixel_aspect <= 0.0 {
        return PictureGeometry {
            dest: cell,
            uv: base,
        };
    }
    let cw = cell.w * pixel_aspect;
    let ch = cell.h;
    let cell_aspect = cw / ch;
    match mode {
        AspectMode::Stretch => PictureGeometry {
            dest: cell.snapped(),
            uv: base,
        },
        AspectMode::Fit => {
            let (w_px, h_px) = if cell_aspect > picture_aspect {
                (picture_aspect * ch / pixel_aspect, ch)
            } else {
                (cell.w, cw / picture_aspect)
            };
            let dest = Rect::new(
                cell.x + (cell.w - w_px) / 2.0,
                cell.y + (cell.h - h_px) / 2.0,
                w_px,
                h_px,
            )
            .snapped();
            PictureGeometry { dest, uv: base }
        }
        AspectMode::Fill => {
            let (fu, fv) = if cell_aspect > picture_aspect {
                (1.0, picture_aspect / cell_aspect)
            } else {
                (cell_aspect / picture_aspect, 1.0)
            };
            let span_u = base[2] - base[0];
            let span_v = base[3] - base[1];
            let mu = span_u * (1.0 - fu) / 2.0;
            let mv = span_v * (1.0 - fv) / 2.0;
            PictureGeometry {
                dest: cell.snapped(),
                uv: [base[0] + mu, base[1] + mv, base[2] - mu, base[3] - mv],
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A43: f32 = 4.0 / 3.0;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-3
    }

    #[test]
    fn fit_on_16x9() {
        let g = fit_picture(Rect::new(0.0, 0.0, 1920.0, 1080.0), A43, 1.0, AspectMode::Fit, 0.0);
        assert_eq!(g.dest, Rect::new(240.0, 0.0, 1440.0, 1080.0));
        assert_eq!(g.uv, [0.0, 0.0, 1.0, 1.0]);
    }

    #[test]
    fn fit_on_tall_cell() {
        let g = fit_picture(Rect::new(10.0, 20.0, 400.0, 600.0), A43, 1.0, AspectMode::Fit, 0.0);
        assert_eq!(g.dest, Rect::new(10.0, 170.0, 400.0, 300.0));
    }

    #[test]
    fn hd3_output_is_corrected() {
        // 1280x720 shown on an 800x480 (5:3) area: a 4:3 picture is
        // 1024 output pixels wide.
        let par = pixel_aspect(OutputAspect::Hd3Panel5x3, 1280.0, 720.0);
        assert!(close(par, (800.0 / 480.0) / (1280.0 / 720.0)));
        let g = fit_picture(Rect::new(0.0, 0.0, 1280.0, 720.0), A43, par, AspectMode::Fit, 0.0);
        assert_eq!(g.dest, Rect::new(128.0, 0.0, 1024.0, 720.0));
        assert_eq!(pixel_aspect(OutputAspect::Auto, 1280.0, 720.0), 1.0);
        // 800x600 mode on a 4:3 screen: square pixels.
        assert!(close(pixel_aspect(OutputAspect::Standard4x3, 800.0, 600.0), 1.0));
    }

    #[test]
    fn fill_crops_source() {
        let g = fit_picture(Rect::new(0.0, 0.0, 1920.0, 1080.0), A43, 1.0, AspectMode::Fill, 0.0);
        assert_eq!(g.dest, Rect::new(0.0, 0.0, 1920.0, 1080.0));
        // Visible height fraction = (1080 * 4/3) / 1920 = 0.75.
        assert!(close(g.uv[0], 0.0) && close(g.uv[2], 1.0));
        assert!(close(g.uv[1], 0.125) && close(g.uv[3], 0.875));
        let g = fit_picture(Rect::new(0.0, 0.0, 300.0, 600.0), A43, 1.0, AspectMode::Fill, 0.0);
        // Visible width fraction = 0.5 / (4/3) = 0.375.
        assert!(close(g.uv[2] - g.uv[0], 0.375));
        assert!(close(g.uv[1], 0.0));
    }

    #[test]
    fn stretch_and_overscan() {
        let g = fit_picture(Rect::new(0.0, 0.0, 1280.0, 720.0), A43, 1.0, AspectMode::Stretch, 5.0);
        assert_eq!(g.dest, Rect::new(0.0, 0.0, 1280.0, 720.0));
        assert!(close(g.uv[0], 0.05) && close(g.uv[3], 0.95));
        let f = fit_picture(Rect::new(0.0, 0.0, 1920.0, 1080.0), A43, 1.0, AspectMode::Fill, 10.0);
        assert!(close(f.uv[0], 0.1) && close(f.uv[2], 0.9));
        assert!(close(f.uv[3] - f.uv[1], 0.8 * 0.75));
    }

    #[test]
    fn cells() {
        let mut out = [Rect::default(); 4];
        let area = Rect::new(0.0, 0.0, 1000.0, 500.0);
        assert_eq!(view_cells(area, ViewLayout::Single, 2.0, &mut out), 1);
        assert_eq!(out[0], area);
        assert_eq!(view_cells(area, ViewLayout::Dual, 2.0, &mut out), 2);
        assert_eq!(out[0], Rect::new(0.0, 0.0, 499.0, 500.0));
        assert_eq!(out[1], Rect::new(501.0, 0.0, 499.0, 500.0));
        let tall = Rect::new(0.0, 0.0, 400.0, 802.0);
        view_cells(tall, ViewLayout::Dual, 2.0, &mut out);
        assert_eq!(out[1], Rect::new(0.0, 402.0, 400.0, 400.0));
        assert_eq!(view_cells(area, ViewLayout::Quad, 2.0, &mut out), 4);
        assert_eq!(out[3], Rect::new(501.0, 251.0, 499.0, 249.0));
        for c in &out {
            assert!(c.right() <= area.right() + 0.5 && c.bottom() <= area.bottom() + 0.5);
        }
    }

    #[test]
    fn degenerate_inputs() {
        let g = fit_picture(Rect::new(0.0, 0.0, 0.0, 0.0), A43, 1.0, AspectMode::Fit, 0.0);
        assert_eq!(g.dest, Rect::default());
    }

}
