// SPDX-License-Identifier: GPL-3.0-or-later
//! GPU side of the video display: half-float Y and UV textures updated
//! row range by row range, converted and scaled in `video.wgsl`.
//!
//! The same renderer draws into the egui frame (through a paint callback)
//! and into offscreen textures for snapshots.

use crate::layout::PictureGeometry;
use crate::settings::DisplaySettings;
use crate::live_filter::LiveFilter;
use crate::source::exchange::HISTORY;
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
    /// The live filter's inputs and outputs, while the view uses it.
    filt: Option<FilterPlanes>,
}

/// Live learned filter (`denoise.wgsl`) for one view: the last complete
/// fields, the per-row field flags, and the filtered Y and UV that the
/// display samples instead of the raw ones.
struct FilterPlanes {
    hist_y: [wgpu::Texture; HISTORY],
    hist_uv: [wgpu::Texture; HISTORY],
    row_new: wgpu::Texture,
    // Bound through `compute` and `display`; read directly only by tests.
    #[cfg_attr(not(test), allow(dead_code))]
    out_y: wgpu::Texture,
    #[cfg_attr(not(test), allow(dead_code))]
    out_uv: wgpu::Texture,
    params: wgpu::Buffer,
    compute: wgpu::BindGroup,
    display: wgpu::BindGroup,
}

/// `denoise.wgsl` parameters.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default, Pod, Zeroable)]
struct FilterParams {
    size: [u32; 2],
    pad: [u32; 2],
    slots: [u32; 4],
}

/// One view's textures and uniforms.
pub struct ViewGpu {
    uniforms: wgpu::Buffer,
    planes: Option<Planes>,
    empty_bind_group: wgpu::BindGroup,
    /// Show this view through the live filter (when one is loaded).
    filter_on: bool,
}

/// Pipeline and per-view resources.
pub struct VideoGpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    views: Vec<ViewGpu>,
    filter_pipeline: wgpu::ComputePipeline,
    filter_layout: wgpu::BindGroupLayout,
    /// The live filter's coefficients (`LiveFilter::packed`), when loaded.
    filter_coef: Option<wgpu::Buffer>,
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
        let (filter_pipeline, filter_layout) = Self::filter_pipeline(device);
        VideoGpu {
            device: device.clone(),
            queue: queue.clone(),
            pipeline,
            layout,
            sampler,
            views: Vec::new(),
            filter_pipeline,
            filter_layout,
            filter_coef: None,
        }
    }

    /// The live filter's compute pipeline (`denoise.wgsl`).
    fn filter_pipeline(device: &wgpu::Device) -> (wgpu::ComputePipeline, wgpu::BindGroupLayout) {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("openvrx live filter"),
            source: wgpu::ShaderSource::Wgsl(include_str!("denoise.wgsl").into()),
        });
        let entry = |binding: u32, ty: wgpu::BindingType| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty,
            count: None,
        };
        let tex = |binding: u32, sample_type: wgpu::TextureSampleType| {
            entry(
                binding,
                wgpu::BindingType::Texture {
                    multisampled: false,
                    sample_type,
                    view_dimension: wgpu::TextureViewDimension::D2,
                },
            )
        };
        let float = wgpu::TextureSampleType::Float { filterable: false };
        let store = |binding: u32| {
            entry(
                binding,
                wgpu::BindingType::StorageTexture {
                    access: wgpu::StorageTextureAccess::WriteOnly,
                    format: wgpu::TextureFormat::Rgba16Float,
                    view_dimension: wgpu::TextureViewDimension::D2,
                },
            )
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("openvrx live filter"),
            entries: &[
                entry(
                    0,
                    wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                ),
                entry(
                    1,
                    wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                ),
                tex(2, float),
                tex(3, float),
                tex(4, float),
                tex(5, float),
                tex(6, float),
                tex(7, float),
                tex(8, float),
                tex(9, float),
                tex(10, wgpu::TextureSampleType::Uint),
                store(11),
                store(12),
            ],
        });
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("openvrx live filter"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("openvrx live filter"),
            layout: Some(&pl),
            module: &module,
            entry_point: Some("cs_main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });
        (pipeline, layout)
    }

    /// Loads the live filter (None: no view can use it).
    pub fn set_filter(&mut self, filter: Option<&LiveFilter>) {
        self.filter_coef = filter.map(|f| {
            let data = f.packed();
            let buf = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("live filter coefficients"),
                size: (4 * data.len()) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.queue.write_buffer(&buf, 0, bytemuck::cast_slice(&data));
            buf
        });
        // Bind groups hold the old buffer: rebuild on the next upload.
        for v in &mut self.views {
            if let Some(p) = v.planes.as_mut() {
                p.filt = None;
            }
        }
    }

    /// Shows view `k` through the live filter (when one is loaded). Off
    /// drops its filter textures.
    pub fn set_view_filter(&mut self, k: usize, on: bool) {
        self.ensure_view(k);
        let v = &mut self.views[k];
        v.filter_on = on;
        if !on {
            if let Some(p) = v.planes.as_mut() {
                p.filt = None;
            }
        }
    }

    fn make_filter_planes(&self, uniforms: &wgpu::Buffer, p: &Planes, coef: &wgpu::Buffer) -> FilterPlanes {
        let (w, h) = (p.width, p.height);
        let mk = |label: &str, width: usize, format: wgpu::TextureFormat, usage: wgpu::TextureUsages| {
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
                usage,
                view_formats: &[],
            })
        };
        let input = wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST;
        let output = wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::STORAGE_BINDING
            | wgpu::TextureUsages::COPY_SRC;
        let hist_y = std::array::from_fn(|_| mk("filter history y", w, wgpu::TextureFormat::R16Float, input));
        let hist_uv = std::array::from_fn(|_| mk("filter history uv", w, wgpu::TextureFormat::Rg16Float, input));
        let row_new = mk("filter row flags", 1, wgpu::TextureFormat::R8Uint, input);
        let out_y = mk("filtered y", w, wgpu::TextureFormat::Rgba16Float, output);
        let out_uv = mk("filtered uv", w, wgpu::TextureFormat::Rgba16Float, output);
        let params = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("live filter params"),
            size: std::mem::size_of::<FilterParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let view = |t: &wgpu::Texture| t.create_view(&wgpu::TextureViewDescriptor::default());
        let textures = [
            &p.y, &p.uv, &hist_y[0], &hist_y[1], &hist_y[2], &hist_uv[0], &hist_uv[1], &hist_uv[2], &row_new,
            &out_y, &out_uv,
        ];
        let views: Vec<wgpu::TextureView> = textures.iter().map(|t| view(t)).collect();
        let mut entries = vec![
            wgpu::BindGroupEntry {
                binding: 0,
                resource: params.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: coef.as_entire_binding(),
            },
        ];
        for (i, v) in views.iter().enumerate() {
            entries.push(wgpu::BindGroupEntry {
                binding: 2 + i as u32,
                resource: wgpu::BindingResource::TextureView(v),
            });
        }
        let compute = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("openvrx live filter"),
            layout: &self.filter_layout,
            entries: &entries,
        });
        let display = self.bind(uniforms, [&out_y, &out_uv, &p.dmg, &p.row]);
        FilterPlanes {
            hist_y,
            hist_uv,
            row_new,
            out_y,
            out_uv,
            params,
            compute,
            display,
        }
    }

    /// Uploads the new history fields and row flags of view `k` and runs
    /// the filter over the whole picture.
    fn run_filter(&mut self, k: usize, st: &UploadStaging) {
        let Some(coef) = self.filter_coef.as_ref() else {
            return;
        };
        let v = &self.views[k];
        let Some(p) = v.planes.as_ref() else {
            return;
        };
        if p.filt.is_none() {
            let f = self.make_filter_planes(&v.uniforms, p, coef);
            if let Some(p) = self.views[k].planes.as_mut() {
                p.filt = Some(f);
            }
        }
        let Some(p) = self.views[k].planes.as_ref() else {
            return;
        };
        let Some(f) = p.filt.as_ref() else {
            return;
        };
        let (w, h) = (p.width, p.height);
        let full = |tex: &wgpu::Texture, data: &[u8], bpr: usize, width: usize| {
            self.queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                data,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bpr as u32),
                    rows_per_image: Some(h as u32),
                },
                wgpu::Extent3d {
                    width: width as u32,
                    height: h as u32,
                    depth_or_array_layers: 1,
                },
            );
        };
        for &n in &st.history_new {
            let slot = (n % HISTORY as u64) as usize;
            let hf = &st.history[slot];
            if hf.y.len() == w * h && hf.uv.len() == 2 * w * h {
                full(&f.hist_y[slot], bytemuck::cast_slice(&hf.y), 2 * w, w);
                full(&f.hist_uv[slot], bytemuck::cast_slice(&hf.uv), 4 * w, w);
            }
        }
        if st.row_new.len() == h {
            full(&f.row_new, &st.row_new, 1, 1);
        }
        let n = st.history_count;
        let ready = n >= HISTORY as u64;
        let slot = |back: u64| if ready { ((n - back) % HISTORY as u64) as u32 } else { 0 };
        let params = FilterParams {
            size: [w as u32, h as u32],
            pad: [0, 0],
            slots: [slot(1), slot(2), slot(3), ready as u32],
        };
        self.queue.write_buffer(&f.params, 0, bytemuck::bytes_of(&params));
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("live filter") });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("live filter"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.filter_pipeline);
            pass.set_bind_group(0, &f.compute, &[]);
            pass.dispatch_workgroups((w as u32).div_ceil(8), (h as u32).div_ceil(8), 1);
        }
        self.queue.submit([enc.finish()]);
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
                filter_on: false,
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
                filt: None,
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
        if self.views[k].filter_on {
            self.run_filter(k, st);
        }
    }

    /// Writes view `k`'s uniforms and returns its paint handle.
    pub fn prepare(&mut self, k: usize, u: &Uniforms) -> VideoPaint {
        self.ensure_view(k);
        let v = &self.views[k];
        self.queue.write_buffer(&v.uniforms, 0, bytemuck::bytes_of(u));
        let bind_group = v.planes.as_ref().map_or_else(
            || v.empty_bind_group.clone(),
            |p| match (&p.filt, v.filter_on && self.filter_coef.is_some()) {
                (Some(f), true) => f.display.clone(),
                _ => p.bind_group.clone(),
            },
        );
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

    /// Reads an Rgba16Float texture back as (r, g) per texel.
    fn read_rg(device: &wgpu::Device, queue: &wgpu::Queue, tex: &wgpu::Texture, w: u32, h: u32) -> Vec<[f32; 2]> {
        let padded = (8 * w).div_ceil(256) * 256;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: (padded * h) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        enc.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(padded),
                    rows_per_image: Some(h),
                },
            },
            wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
        );
        queue.submit([enc.finish()]);
        let slice = buffer.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
        let data = slice.get_mapped_range().unwrap();
        let mut out = Vec::new();
        for row in data.chunks(padded as usize).take(h as usize) {
            let px: &[half::f16] = bytemuck::cast_slice(&row[..(8 * w) as usize]);
            out.extend(px.chunks_exact(4).map(|c| [c[0].to_f32(), c[1].to_f32()]));
        }
        out
    }

    /// The live filter pass must match `LiveFilter::apply` on the same
    /// (half-float) inputs: fields k-3..k-1 for rows still holding the
    /// previous field, k-2, k-1 and the current picture for rows of the
    /// field in progress. Skips when the machine has no GPU adapter.
    #[test]
    fn live_filter_matches_cpu_reference() {
        use crate::live_filter::{LiveFilter, FILTER_FIELDS};
        use crate::source::RowUpdate;
        let instance = wgpu::Instance::default();
        let Ok(adapter) = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())) else {
            eprintln!("skipping GPU check: no adapter");
            return;
        };
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).unwrap();
        let (w, h) = (24usize, 16usize);
        // Four fields of pseudo-random pictures.
        let field = |n: u32| {
            let mut f = openvrx_core::Frame::new(w, h);
            let mut s = 12345u32.wrapping_mul(n + 1);
            let mut rnd = || {
                s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
                (s >> 8) as f32 / (1 << 24) as f32
            };
            for k in 0..w * h {
                f.y[k] = 0.2 + 0.6 * rnd();
                f.u[k] = 0.2 * (rnd() - 0.5);
                f.v[k] = 0.2 * (rnd() - 0.5);
            }
            f
        };
        let ex = FrameExchange::new();
        let mut p = ex.producer();
        let up = |rows: std::ops::Range<usize>, field_done: bool| RowUpdate {
            rows,
            lines: 1,
            field_done,
            frame_done: false,
            newest_line_ns: 0,
        };
        p.publish(&field(9), up(0..h, false)); // sizes the exchange
        ex.set_history(true);
        let fields: Vec<_> = (0..4).map(field).collect();
        for f in &fields[..3] {
            p.publish(f, up(0..h, true));
        }
        p.publish(&fields[3], up(0..h / 2, false));
        let mut st = UploadStaging::default();
        ex.take(&mut st).unwrap();
        assert_eq!(st.history_count, 3);

        let filter = LiveFilter::from_json(FILTER_FIELDS).unwrap();
        let mut gpu = VideoGpu::new(&device, &queue, wgpu::TextureFormat::Rgba8Unorm);
        gpu.set_filter(Some(&filter));
        gpu.set_view_filter(0, true);
        gpu.upload(0, &st);
        let pl = gpu.views[0].planes.as_ref().unwrap();
        let f = pl.filt.as_ref().unwrap();
        let ys = read_rg(&device, &queue, &f.out_y, w as u32, h as u32);
        let uvs = read_rg(&device, &queue, &f.out_uv, w as u32, h as u32);

        // CPU reference on the same half-float values the GPU saw.
        let planes = |y: &[half::f16], uv: &[half::f16]| -> [Vec<f32>; 3] {
            [
                y.iter().map(|v| v.to_f32()).collect(),
                uv.iter().step_by(2).map(|v| v.to_f32()).collect(),
                uv.iter().skip(1).step_by(2).map(|v| v.to_f32()).collect(),
            ]
        };
        let hist: Vec<[Vec<f32>; 3]> = (0..3).map(|n| planes(&st.history[n].y, &st.history[n].uv)).collect();
        let cur = planes(&st.y, &st.uv);
        let fresh = filter.apply(w, h, [&hist[1], &hist[2], &cur], &cur);
        let stale = filter.apply(w, h, [&hist[0], &hist[1], &hist[2]], &cur);
        let mut worst = 0f32;
        for row in 0..h {
            let want = if st.row_new[row] == 1 { &fresh } else { &stale };
            for x in 0..w {
                let k = row * w + x;
                let got = [ys[k][0], uvs[k][0], uvs[k][1]];
                for o in 0..3 {
                    worst = worst.max((got[o] - want[o][k]).abs());
                }
            }
        }
        assert_eq!(&st.row_new[..h / 2], &vec![1u8; h / 2][..]);
        // Half-float output: about 1e-3 at these levels.
        assert!(worst < 3e-3, "max difference {worst}");
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
