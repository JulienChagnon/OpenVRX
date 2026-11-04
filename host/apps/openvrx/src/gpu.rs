// SPDX-License-Identifier: GPL-3.0-or-later
//! GPU side of the video display: half-float Y and UV textures updated
//! row range by row range, converted and scaled in `video.wgsl`.
//!
//! The same renderer draws into the egui frame (through a paint callback)
//! and into offscreen textures for snapshots.

use crate::layout::PictureGeometry;
use crate::settings::DisplaySettings;
use crate::source::UploadStaging;
use bytemuck::{Pod, Zeroable};
use openvrx_core::COMBINED;

/// Strip colours per receiver (index 8: combined lines).
pub const PALETTE: [[f32; 4]; 9] = [
    [0.10, 0.85, 0.95, 1.0],
    [1.00, 0.55, 0.10, 1.0],
    [0.65, 0.95, 0.20, 1.0],
    [0.95, 0.30, 0.85, 1.0],
    [0.95, 0.90, 0.25, 1.0],
    [0.35, 0.50, 1.00, 1.0],
    [1.00, 0.35, 0.35, 1.0],
    [0.40, 1.00, 0.70, 1.0],
    [0.95, 0.95, 0.95, 1.0],
];

/// Palette index of a row source.
pub fn palette_index(rx: u8) -> usize {
    if rx == COMBINED {
        8
    } else {
        (rx as usize) % 8
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct Uniforms {
    pub uv: [f32; 4],
    pub tex: [f32; 4],
    pub adj: [f32; 4],
    pub dest: [f32; 4],
    pub flags: [u32; 4],
    pub palette: [[f32; 4]; 9],
}

impl Uniforms {
    /// Shader parameters for a view.
    pub fn new(
        geom: &PictureGeometry,
        tex_w: usize,
        tex_h: usize,
        d: &DisplaySettings,
        strip: bool,
        has_content: bool,
    ) -> Uniforms {
        let w = tex_w.max(1) as f32;
        let h = tex_h.max(1) as f32;
        let strip_px = (geom.dest.w * 0.008).clamp(4.0, 12.0);
        Uniforms {
            uv: geom.uv,
            tex: [w, h, 1.0 / w, 1.0 / h],
            adj: [d.brightness, d.contrast, d.saturation, d.sharpen],
            dest: [geom.dest.w, geom.dest.h, if strip { strip_px } else { 0.0 }, 0.0],
            flags: [
                d.scaling.code(),
                d.damage_overlay as u32,
                strip as u32,
                has_content as u32,
            ],
            palette: PALETTE,
        }
    }
}

struct Planes {
    width: usize,
    height: usize,
    y: wgpu::Texture,
    uv: wgpu::Texture,
    dmg: wgpu::Texture,
    row: wgpu::Texture,
    bind_group: wgpu::BindGroup,
}

/// One view's textures and uniforms.
pub struct ViewGpu {
    uniforms: wgpu::Buffer,
    planes: Option<Planes>,
    empty_bind_group: wgpu::BindGroup,
}

/// Pipeline and per-view resources.
pub struct VideoGpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    views: Vec<ViewGpu>,
}

/// Everything a paint callback needs (cheap clones of GPU handles).
#[derive(Clone)]
pub struct VideoPaint {
    pub pipeline: wgpu::RenderPipeline,
    pub bind_group: wgpu::BindGroup,
}

impl VideoPaint {
    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind_group, &[]);
        pass.draw(0..3, 0..1);
    }
}

impl egui_wgpu::CallbackTrait for VideoPaint {
    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _resources: &egui_wgpu::CallbackResources,
    ) {
        self.draw(render_pass);
    }
}

fn tex_entry(binding: u32, sample_type: wgpu::TextureSampleType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            multisampled: false,
            sample_type,
            view_dimension: wgpu::TextureViewDimension::D2,
        },
        count: None,
    }
}

impl VideoGpu {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, format: wgpu::TextureFormat) -> VideoGpu {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("openvrx video"),
            source: wgpu::ShaderSource::Wgsl(include_str!("video.wgsl").into()),
        });
        let float = wgpu::TextureSampleType::Float { filterable: true };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("openvrx video"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                tex_entry(1, float),
                tex_entry(2, float),
                tex_entry(3, float),
                tex_entry(4, wgpu::TextureSampleType::Uint),
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("openvrx video"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("openvrx video"),
            layout: Some(&pl),
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("openvrx video linear"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        VideoGpu {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            layout,
            sampler,
            views: Vec::new(),
        }
    }

    fn make_planes(&self, w: usize, h: usize) -> (wgpu::Texture, wgpu::Texture, wgpu::Texture, wgpu::Texture) {
        let mk = |label: &str, width: usize, format: wgpu::TextureFormat| {
            self.device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d {
                    width: width.max(1) as u32,
                    height: h.max(1) as u32,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            })
        };
        (
            mk("video y", w, wgpu::TextureFormat::R16Float),
            mk("video uv", w, wgpu::TextureFormat::Rg16Float),
            mk("video damage", w, wgpu::TextureFormat::R8Unorm),
            mk("video rows", 1, wgpu::TextureFormat::R8Uint),
        )
    }

    fn bind(&self, uniforms: &wgpu::Buffer, t: [&wgpu::Texture; 4]) -> wgpu::BindGroup {
        let v: Vec<wgpu::TextureView> = t
            .iter()
            .map(|t| t.create_view(&wgpu::TextureViewDescriptor::default()))
            .collect();
        self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("openvrx video"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniforms.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&v[0]),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&v[1]),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::TextureView(&v[2]),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::TextureView(&v[3]),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        })
    }

    /// Makes sure view slot `k` exists.
    pub fn ensure_view(&mut self, k: usize) {
        while self.views.len() <= k {
            let uniforms = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("video uniforms"),
                size: std::mem::size_of::<Uniforms>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let (y, uv, d, r) = self.make_planes(1, 1);
            let empty_bind_group = self.bind(&uniforms, [&y, &uv, &d, &r]);
            self.views.push(ViewGpu {
                uniforms,
                planes: None,
                empty_bind_group,
            });
        }
    }

    /// Drops a view's textures (source closed).
    pub fn clear_view(&mut self, k: usize) {
        if let Some(v) = self.views.get_mut(k) {
            v.planes = None;
        }
    }

    /// Size of view `k`'s picture, when it has one.
    pub fn view_size(&self, k: usize) -> Option<(usize, usize)> {
        self.views
            .get(k)
            .and_then(|v| v.planes.as_ref())
            .map(|p| (p.width, p.height))
    }

    /// Uploads the staged rows of view `k`.
    pub fn upload(&mut self, k: usize, st: &UploadStaging) {
        self.ensure_view(k);
        let (w, h) = (st.width, st.height);
        if w == 0 || h == 0 {
            return;
        }
        let needs_new = self.views[k]
            .planes
            .as_ref()
            .is_none_or(|p| p.width != w || p.height != h);
        if needs_new {
            let (y, uv, d, r) = self.make_planes(w, h);
            let bind_group = self.bind(&self.views[k].uniforms, [&y, &uv, &d, &r]);
            self.views[k].planes = Some(Planes {
                width: w,
                height: h,
                y,
                uv,
                dmg: d,
                row: r,
                bind_group,
            });
        }
        let Some(p) = self.views[k].planes.as_ref() else {
            return;
        };
        for r in &st.ranges {
            let (a, b) = (r.start as usize, (r.end as usize).min(h));
            if a >= b {
                continue;
            }
            let rows = (b - a) as u32;
            let write = |tex: &wgpu::Texture, data: &[u8], bpr: usize, width: u32| {
                self.queue.write_texture(
                    wgpu::TexelCopyTextureInfo {
                        texture: tex,
                        mip_level: 0,
                        origin: wgpu::Origin3d {
                            x: 0,
                            y: a as u32,
                            z: 0,
                        },
                        aspect: wgpu::TextureAspect::All,
                    },
                    data,
                    wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(bpr as u32),
                        rows_per_image: Some(rows),
                    },
                    wgpu::Extent3d {
                        width,
                        height: rows,
                        depth_or_array_layers: 1,
                    },
                );
            };
            write(&p.y, bytemuck::cast_slice(&st.y[a * w..b * w]), 2 * w, w as u32);
            write(&p.uv, bytemuck::cast_slice(&st.uv[2 * a * w..2 * b * w]), 4 * w, w as u32);
            write(&p.dmg, &st.damage[a * w..b * w], w, w as u32);
            write(&p.row, &st.row_source[a..b], 1, 1);
        }
    }

    /// Writes view `k`'s uniforms and returns its paint handle.
    pub fn prepare(&mut self, k: usize, u: &Uniforms) -> VideoPaint {
        self.ensure_view(k);
        let v = &self.views[k];
        self.queue.write_buffer(&v.uniforms, 0, bytemuck::bytes_of(u));
        let bind_group = v
            .planes
            .as_ref()
            .map_or_else(|| v.empty_bind_group.clone(), |p| p.bind_group.clone());
        VideoPaint {
            pipeline: self.pipeline.clone(),
            bind_group,
        }
    }
}

/// Renders one frame offscreen and returns RGBA8 pixels (for snapshots
/// and shader checks). Creates its own device; returns an error when no
/// adapter is available.
pub fn render_offscreen(
    staging: &UploadStaging,
    geom: &PictureGeometry,
    display: &DisplaySettings,
    out_w: u32,
    out_h: u32,
) -> Result<Vec<u8>, String> {
    let instance = wgpu::Instance::default();
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
        .map_err(|e| format!("no GPU adapter: {e}"))?;
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
        .map_err(|e| format!("no GPU device: {e}"))?;
    let format = wgpu::TextureFormat::Rgba8Unorm;
    let mut gpu = VideoGpu::new(&device, &queue, format);
    gpu.upload(0, staging);
    let u = Uniforms::new(geom, staging.width, staging.height, display, display.row_strip, true);
    let paint = gpu.prepare(0, &u);
    let target = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("snapshot"),
        size: wgpu::Extent3d {
            width: out_w,
            height: out_h,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = target.create_view(&wgpu::TextureViewDescriptor::default());
    let padded = (4 * out_w).div_ceil(256) * 256;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("snapshot readback"),
        size: (padded * out_h) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
    {
        let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("snapshot"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
                depth_slice: None,
            })],
            depth_stencil_attachment: None,
            occlusion_query_set: None,
            timestamp_writes: None,
            multiview_mask: None,
        });
        let d = geom.dest;
        pass.set_viewport(d.x, d.y, d.w, d.h, 0.0, 1.0);
        paint.draw(&mut pass);
    }
    enc.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &target,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(out_h),
            },
        },
        wgpu::Extent3d {
            width: out_w,
            height: out_h,
            depth_or_array_layers: 1,
        },
    );
    queue.submit([enc.finish()]);
    let slice = buffer.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|e| format!("GPU poll: {e}"))?;
    rx.recv()
        .map_err(|e| e.to_string())?
        .map_err(|e| format!("readback: {e}"))?;
    let data = slice.get_mapped_range().map_err(|e| format!("readback: {e}"))?;
    let mut out = Vec::with_capacity((4 * out_w * out_h) as usize);
    for row in data.chunks(padded as usize).take(out_h as usize) {
        out.extend_from_slice(&row[..(4 * out_w) as usize]);
    }
    Ok(out)
}

/// Writes RGBA8 pixels as a PNG.
pub fn write_png(path: &std::path::Path, w: u32, h: u32, rgba: &[u8]) -> Result<(), String> {
    let file = std::fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    let mut wr = enc.write_header().map_err(|e| e.to_string())?;
    wr.write_image_data(rgba).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::Rect;
    use crate::settings::{ScalingFilter, TestPattern};
    use crate::source::test_pattern::render_frame;
    use crate::source::{FrameExchange, UploadStaging};
    use openvrx_core::VideoStandard;

    #[test]
    fn uniforms_layout() {
        assert_eq!(std::mem::size_of::<Uniforms>(), 16 * 5 + 16 * 9);
        assert_eq!(palette_index(COMBINED), 8);
        assert_eq!(palette_index(9), 1);
    }

    /// Nearest scaling at 1:1 must reproduce the CPU conversion. Skips
    /// when the machine has no GPU adapter.
    #[test]
    fn shader_matches_cpu_reference() {
        let frame = render_frame(TestPattern::Bars, VideoStandard::Ntsc, 0.0);
        let ex = FrameExchange::new();
        ex.producer().publish_frame(&frame, 1);
        let mut st = UploadStaging::default();
        ex.take(&mut st).unwrap();
        let (w, h) = (frame.width as u32, frame.height as u32);
        let geom = PictureGeometry {
            dest: Rect::new(0.0, 0.0, w as f32, h as f32),
            uv: [0.0, 0.0, 1.0, 1.0],
        };
        let display = DisplaySettings {
            scaling: ScalingFilter::Nearest,
            sharpen: 0.0,
            row_strip: false,
            ..DisplaySettings::default()
        };
        let gpu = match render_offscreen(&st, &geom, &display, w, h) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("skipping GPU check: {e}");
                return;
            }
        };
        let cpu = frame.to_rgba8();
        let worst = gpu
            .iter()
            .zip(&cpu)
            .map(|(a, b)| (*a as i32 - *b as i32).abs())
            .max()
            .unwrap();
        // Half-float storage costs at most a couple of codes.
        assert!(worst <= 3, "max difference {worst}");
    }
}
