// SPDX-License-Identifier: GPL-3.0-or-later
//! Fields as a plugin gets them: host-owned plane buffers with the side
//! planes, and the metadata of each field (index, parity, C/N, flags) from
//! the frames a view shows.

use openvrx_core::{Frame, LineFlags, LineQuality, VideoStandard};
use openvrx_filter_abi as abi;
use std::ffi::CStr;

/// What a plugin learns about a field besides its planes.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FieldMeta {
    /// `field_index`: +1 per field the host produced for the instance.
    pub index: u64,
    /// 0 top field, 1 bottom field.
    pub parity: u32,
    /// `OPENVRX_FIELD_*`.
    pub flags: u32,
    /// Median C/N of the field's rows in dB, NaN when unknown.
    pub cn_db: f32,
}

impl Default for FieldMeta {
    fn default() -> Self {
        FieldMeta {
            index: 0,
            parity: 0,
            flags: 0,
            cn_db: f32::NAN,
        }
    }
}

/// A side plane: a name from [`abi::AUX_PLANES`] and its values.
struct Aux {
    name: &'static CStr,
    width: u32,
    height: u32,
    data: Vec<f32>,
}

/// The C name of a known side plane.
fn aux_cname(name: &str) -> Option<&'static CStr> {
    Some(match name {
        "damage" => c"damage",
        "row_cn_db" => c"row_cn_db",
        "row_noise_ire" => c"row_noise_ire",
        "row_sync_score" => c"row_sync_score",
        "row_timing_err" => c"row_timing_err",
        "row_clicks" => c"row_clicks",
        "row_flags" => c"row_flags",
        _ => return None,
    })
}

/// A row side plane's value for one picture row.
fn row_value(name: &str, q: &LineQuality) -> f32 {
    match name {
        "row_cn_db" => q.cn_db,
        "row_noise_ire" => q.noise_ire,
        "row_sync_score" => q.sync_score,
        "row_timing_err" => q.timing_err,
        "row_clicks" => q.clicks as f32,
        "row_flags" => q.flags.0 as f32,
        _ => f32::NAN,
    }
}

/// One field's planes, owned by the host: the input copied from a frame,
/// the output the plugin writes, and the side planes it asked for. Planes
/// are tight (stride = width); chroma is at full resolution.
#[derive(Default)]
pub struct FieldBuffers {
    pub width: usize,
    pub height: usize,
    pub y: Vec<f32>,
    pub u: Vec<f32>,
    pub v: Vec<f32>,
    /// Output Y, U, V.
    pub out: [Vec<f32>; 3],
    aux: Vec<Aux>,
    pub meta: FieldMeta,
}

impl FieldBuffers {
    /// Copies `frame`'s planes and computes the side planes named in `aux`
    /// (unknown names are left out; `PluginInfo` refuses them earlier).
    /// Allocates only when the size or the side planes change.
    pub fn fill(&mut self, frame: &Frame, meta: FieldMeta, aux: &[String]) {
        let (w, h) = (frame.width, frame.height);
        self.width = w;
        self.height = h;
        self.y.clone_from(&frame.y);
        self.u.clone_from(&frame.u);
        self.v.clone_from(&frame.v);
        for o in &mut self.out {
            o.resize(w * h, 0.0);
        }
        self.meta = meta;
        let mut k = 0;
        for name in aux {
            let Some(cname) = aux_cname(name) else { continue };
            if self.aux.len() <= k {
                self.aux.push(Aux {
                    name: cname,
                    width: 0,
                    height: 0,
                    data: Vec::new(),
                });
            }
            let a = &mut self.aux[k];
            a.name = cname;
            if name == "damage" {
                a.width = w as u32;
                a.height = h as u32;
                a.data.clear();
                a.data.extend(frame.damage.iter().map(|&d| d as f32 / 255.0));
                a.data.resize(w * h, 0.0);
            } else {
                a.width = 1;
                a.height = h as u32;
                a.data.clear();
                a.data.extend(frame.row_quality.iter().map(|q| row_value(name, q)));
                a.data.resize(h, f32::NAN);
            }
            k += 1;
        }
        self.aux.truncate(k);
    }

    /// The ABI description of these buffers, with the side planes listed in
    /// `planes` (the caller's scratch). Its pointers stay valid while
    /// neither `self`'s planes nor `planes` change.
    pub(crate) fn abi_field(&mut self, planes: &mut Vec<abi::Plane>) -> abi::Field {
        planes.clear();
        for a in &self.aux {
            planes.push(abi::Plane {
                name: a.name.as_ptr(),
                width: a.width,
                height: a.height,
                stride: a.width,
                data: a.data.as_ptr(),
            });
        }
        let (w, h) = (self.width as u32, self.height as u32);
        let [oy, ou, ov] = &mut self.out;
        abi::Field {
            width: w,
            height: h,
            stride: w,
            cwidth: w,
            cheight: h,
            cstride: w,
            y: self.y.as_ptr(),
            u: self.u.as_ptr(),
            v: self.v.as_ptr(),
            out_y: oy.as_mut_ptr(),
            out_u: ou.as_mut_ptr(),
            out_v: ov.as_mut_ptr(),
            field_index: self.meta.index,
            parity: self.meta.parity,
            flags: self.meta.flags,
            cn_db: self.meta.cn_db,
            n_aux: planes.len() as u32,
            aux: if planes.is_empty() { std::ptr::null() } else { planes.as_ptr() },
        }
    }

    /// True when the planes hold a whole picture of the stated size (the
    /// host checks before every call, so a plugin never sees a short plane).
    pub(crate) fn consistent(&self) -> bool {
        let n = self.width * self.height;
        n > 0
            && self.y.len() == n
            && self.u.len() == n
            && self.v.len() == n
            && self.out.iter().all(|o| o.len() == n)
    }

    /// Makes `out` the plugin's picture: the output planes are swapped in
    /// (no copy; `out`'s old planes become this buffer's next output), the
    /// rest (damage, rows, metadata) is copied from `src`, the field's input.
    pub fn take_output(&mut self, src: &Frame, out: &mut Frame) {
        out.width = src.width;
        out.height = src.height;
        let [oy, ou, ov] = &mut self.out;
        std::mem::swap(oy, &mut out.y);
        std::mem::swap(ou, &mut out.u);
        std::mem::swap(ov, &mut out.v);
        out.damage.clone_from(&src.damage);
        out.row_source.clone_from(&src.row_source);
        out.row_quality.clone_from(&src.row_quality);
        out.meta.clone_from(&src.meta);
    }
}

/// Builds the metadata of each field of one view, in order: the index,
/// parity and C/N, and `OPENVRX_FIELD_DISCONTINUITY` on the first field and
/// wherever the history breaks: a field number that does not follow the
/// previous one (lost fields, a receiver restart after a retune), a new
/// size or standard, a sync relock inside the field, or a break the caller
/// reports with [`Feed::mark_break`] (pipeline restart, retune).
#[derive(Debug, Default)]
pub struct Feed {
    next_index: u64,
    last: Option<(u64, usize, usize, Option<VideoStandard>)>,
    broken: bool,
    scratch: Vec<f32>,
}

impl Feed {
    pub fn new() -> Feed {
        Feed {
            broken: true,
            ..Feed::default()
        }
    }

    /// The next field starts a new history.
    pub fn mark_break(&mut self) {
        self.broken = true;
    }

    /// Index the next field gets.
    pub fn next_index(&self) -> u64 {
        self.next_index
    }

    /// Metadata of the next field: `frame` is the picture after it,
    /// `steady` says it is line-doubled.
    pub fn next(&mut self, frame: &Frame, steady: bool) -> FieldMeta {
        let seq = frame.meta.field_seq;
        let follows = self.last.is_some_and(|(s, w, h, std)| {
            s.wrapping_add(1) == seq && w == frame.width && h == frame.height && std == frame.meta.standard
        });
        let relock = frame.row_quality.iter().any(|q| q.flags.contains(LineFlags::RELOCK));
        let mut flags = 0;
        if self.broken || !follows || relock {
            flags |= abi::OPENVRX_FIELD_DISCONTINUITY;
        }
        if steady {
            flags |= abi::OPENVRX_FIELD_STEADY_OSD;
        }
        self.broken = false;
        self.last = Some((seq, frame.width, frame.height, frame.meta.standard));
        let meta = FieldMeta {
            index: self.next_index,
            parity: parity(frame),
            flags,
            cn_db: median_cn(frame, &mut self.scratch),
        };
        self.next_index += 1;
        meta
    }
}

/// 0 when the frame's newest field is the standard's top field.
pub fn parity(frame: &Frame) -> u32 {
    match (frame.meta.parity, frame.meta.standard) {
        (Some(p), Some(s)) => (p != s.params().top_field) as u32,
        _ => 0,
    }
}

/// Median of the rows' finite C/N readings, NaN when there are none.
fn median_cn(frame: &Frame, scratch: &mut Vec<f32>) -> f32 {
    scratch.clear();
    scratch.extend(frame.row_quality.iter().map(|q| q.cn_db).filter(|c| c.is_finite()));
    if scratch.is_empty() {
        return f32::NAN;
    }
    let mid = scratch.len() / 2;
    let (_, m, _) = scratch.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    *m
}

#[cfg(test)]
mod tests {
    use super::*;
    use openvrx_core::FieldParity;

    fn frame(seq: u64, parity: FieldParity, std: VideoStandard) -> Frame {
        let mut f = Frame::new(8, 4);
        f.meta.field_seq = seq;
        f.meta.parity = Some(parity);
        f.meta.standard = Some(std);
        f
    }

    #[test]
    fn feed_flags_breaks() {
        let ntsc = VideoStandard::Ntsc;
        let mut feed = Feed::new();
        let d = abi::OPENVRX_FIELD_DISCONTINUITY;
        let m: Vec<FieldMeta> = [10, 11, 13, 14]
            .iter()
            .map(|&s| feed.next(&frame(s, FieldParity::First, ntsc), false))
            .collect();
        assert_eq!(m.iter().map(|x| x.flags).collect::<Vec<_>>(), vec![d, 0, d, 0]);
        assert_eq!(m.iter().map(|x| x.index).collect::<Vec<_>>(), vec![0, 1, 2, 3]);
        // A relock row, a caller's break, a size change, a new standard.
        let mut f = frame(15, FieldParity::Second, ntsc);
        f.row_quality[2].flags.insert(LineFlags::RELOCK);
        assert_eq!(feed.next(&f, true).flags, d | abi::OPENVRX_FIELD_STEADY_OSD);
        assert_eq!(feed.next(&frame(16, FieldParity::First, ntsc), false).flags, 0);
        feed.mark_break();
        assert_eq!(feed.next(&frame(17, FieldParity::Second, ntsc), false).flags, d);
        let mut big = frame(18, FieldParity::First, ntsc);
        big.width = 9;
        assert_eq!(feed.next(&big, false).flags, d);
        assert_eq!(feed.next(&frame(19, FieldParity::Second, VideoStandard::Pal), false).flags, d);
    }

    #[test]
    fn parity_follows_the_top_field() {
        // NTSC's second field holds the top row, PAL's first.
        assert_eq!(parity(&frame(0, FieldParity::Second, VideoStandard::Ntsc)), 0);
        assert_eq!(parity(&frame(0, FieldParity::First, VideoStandard::Ntsc)), 1);
        assert_eq!(parity(&frame(0, FieldParity::First, VideoStandard::Pal)), 0);
        assert_eq!(parity(&Frame::new(2, 2)), 0);
    }

    #[test]
    fn cn_is_the_median_of_known_rows() {
        let mut f = frame(0, FieldParity::First, VideoStandard::Ntsc);
        let mut s = Vec::new();
        assert!(median_cn(&f, &mut s).is_nan());
        f.row_quality[0].cn_db = 10.0;
        f.row_quality[1].cn_db = 30.0;
        f.row_quality[3].cn_db = 12.0;
        assert_eq!(median_cn(&f, &mut s), 12.0);
    }

    #[test]
    fn buffers_carry_planes_meta_and_side_planes() {
        let mut f = frame(3, FieldParity::First, VideoStandard::Ntsc);
        for (k, y) in f.y.iter_mut().enumerate() {
            *y = k as f32 / 32.0;
        }
        f.u.fill(-0.1);
        f.v.fill(0.2);
        f.damage[1] = 255;
        f.row_quality[2].cn_db = 9.5;
        let mut b = FieldBuffers::default();
        let meta = FieldMeta {
            index: 7,
            parity: 1,
            flags: 3,
            cn_db: 9.5,
        };
        let aux = vec!["damage".to_string(), "row_cn_db".to_string(), "bogus".to_string()];
        b.fill(&f, meta, &aux);
        assert!(b.consistent());
        let mut scratch = Vec::new();
        let a = b.abi_field(&mut scratch);
        assert_eq!((a.width, a.height, a.stride, a.cwidth, a.cheight, a.cstride), (8, 4, 8, 8, 4, 8));
        assert_eq!((a.field_index, a.parity, a.flags, a.cn_db), (7, 1, 3, 9.5));
        assert_eq!(a.n_aux, 2);
        let planes = unsafe { a.aux() };
        assert_eq!(unsafe { planes[0].name() }.to_str().unwrap(), "damage");
        assert_eq!(unsafe { planes[0].data() }[1], 1.0);
        assert_eq!((planes[1].width, planes[1].height, planes[1].stride), (1, 4, 1));
        assert_eq!(unsafe { planes[1].data() }[2], 9.5);
        assert!(unsafe { planes[1].data() }[0].is_nan());
        assert_eq!(unsafe { a.input() }[0][5], 5.0 / 32.0);
        // Output: written planes swap into the shown frame.
        b.out[0].fill(0.25);
        let mut shown = Frame::new(1, 1);
        b.take_output(&f, &mut shown);
        assert_eq!((shown.width, shown.height), (8, 4));
        assert!(shown.y.iter().all(|&y| y == 0.25));
        assert_eq!(shown.damage[1], 255);
        assert_eq!(shown.meta.field_seq, 3);
        // Without side planes the pointer is NULL.
        b.fill(&f, meta, &[]);
        let a = b.abi_field(&mut scratch);
        assert_eq!(a.n_aux, 0);
        assert!(a.aux.is_null());
    }
}
