// SPDX-License-Identifier: GPL-3.0-or-later
// YUV (BT.601) to RGB with display-only adjustments, selectable scaling
// (nearest, bilinear, Catmull-Rom bicubic, Lanczos-2) on luma, optional
// unsharp mask, damage overlay and the row-source strip.

struct Uniforms {
    // Source window in normalised texture coordinates: u0, v0, u1, v1.
    uv: vec4<f32>,
    // Texture width, height, 1/width, 1/height.
    tex: vec4<f32>,
    // Brightness, contrast, saturation, sharpen amount.
    adj: vec4<f32>,
    // Destination width and height in pixels, strip width in pixels.
    dest: vec4<f32>,
    // Filter code, damage overlay, strip, has content.
    flags: vec4<u32>,
    // Colours per receiver 0..7, index 8 = combined.
    palette: array<vec4<f32>, 9>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;
@group(0) @binding(1) var t_y: texture_2d<f32>;
@group(0) @binding(2) var t_uv: texture_2d<f32>;
@group(0) @binding(3) var t_dmg: texture_2d<f32>;
@group(0) @binding(4) var t_row: texture_2d<u32>;
@group(0) @binding(5) var s_lin: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) t: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) i: u32) -> VsOut {
    // One triangle covering the viewport.
    let x = f32(i32(i & 1u) * 4 - 1);
    let y = f32(i32(i & 2u) * 2 - 1);
    var o: VsOut;
    o.pos = vec4<f32>(x, y, 0.0, 1.0);
    o.t = vec2<f32>(x * 0.5 + 0.5, 0.5 - y * 0.5);
    return o;
}

fn kern(x: f32, kind: u32) -> f32 {
    let a = abs(x);
    if (kind == 2u) {
        // Catmull-Rom.
        if (a < 1.0) {
            return 1.5 * a * a * a - 2.5 * a * a + 1.0;
        }
        if (a < 2.0) {
            return -0.5 * a * a * a + 2.5 * a * a - 4.0 * a + 2.0;
        }
        return 0.0;
    }
    // Lanczos-2.
    if (a < 1e-4) {
        return 1.0;
    }
    if (a >= 2.0) {
        return 0.0;
    }
    let pi = 3.14159265;
    return 2.0 * sin(pi * a) * sin(pi * a * 0.5) / (pi * pi * a * a);
}

fn luma_4x4(p: vec2<f32>, kind: u32) -> f32 {
    let size = u.tex.xy;
    let c = p * size - vec2<f32>(0.5, 0.5);
    let base = floor(c);
    let f = c - base;
    let maxq = vec2<i32>(size) - vec2<i32>(1, 1);
    var sum = 0.0;
    var wsum = 0.0;
    for (var j = -1; j <= 2; j = j + 1) {
        let wy = kern(f.y - f32(j), kind);
        for (var i = -1; i <= 2; i = i + 1) {
            let wx = kern(f.x - f32(i), kind);
            let q = clamp(vec2<i32>(base) + vec2<i32>(i, j), vec2<i32>(0, 0), maxq);
            let w = wx * wy;
            sum = sum + textureLoad(t_y, q, 0).r * w;
            wsum = wsum + w;
        }
    }
    return sum / max(wsum, 1e-6);
}

fn texel(p: vec2<f32>) -> vec2<i32> {
    let maxq = vec2<i32>(u.tex.xy) - vec2<i32>(1, 1);
    return clamp(vec2<i32>(floor(p * u.tex.xy)), vec2<i32>(0, 0), maxq);
}

fn luma(p: vec2<f32>) -> f32 {
    let kind = u.flags.x;
    if (kind == 0u) {
        return textureLoad(t_y, texel(p), 0).r;
    }
    if (kind == 1u) {
        return textureSampleLevel(t_y, s_lin, p, 0.0).r;
    }
    return luma_4x4(p, kind);
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    if (u.flags.w == 0u) {
        return vec4<f32>(0.04, 0.04, 0.05, 1.0);
    }
    let p = mix(u.uv.xy, u.uv.zw, in.t);

    if (u.flags.z == 1u && in.t.x * u.dest.x < u.dest.z) {
        let row = clamp(i32(p.y * u.tex.y), 0, i32(u.tex.y) - 1);
        let id = textureLoad(t_row, vec2<i32>(0, row), 0).r;
        return u.palette[min(id, 8u)];
    }

    var y = luma(p);
    let amount = u.adj.w;
    if (amount > 0.0) {
        let d = u.tex.zw;
        let blur = 0.25 * (textureSampleLevel(t_y, s_lin, p + vec2<f32>(d.x, 0.0), 0.0).r
            + textureSampleLevel(t_y, s_lin, p - vec2<f32>(d.x, 0.0), 0.0).r
            + textureSampleLevel(t_y, s_lin, p + vec2<f32>(0.0, d.y), 0.0).r
            + textureSampleLevel(t_y, s_lin, p - vec2<f32>(0.0, d.y), 0.0).r);
        y = y + amount * (y - blur);
    }
    var c: vec2<f32>;
    if (u.flags.x == 0u) {
        c = textureLoad(t_uv, texel(p), 0).rg;
    } else {
        c = textureSampleLevel(t_uv, s_lin, p, 0.0).rg;
    }
    y = (y - 0.5) * u.adj.y + 0.5 + u.adj.x;
    c = c * u.adj.z;
    var rgb = vec3<f32>(
        y + 1.13983 * c.y,
        y - 0.39465 * c.x - 0.58060 * c.y,
        y + 2.03211 * c.x
    );
    if (u.flags.y == 1u) {
        let d = textureLoad(t_dmg, texel(p), 0).r;
        rgb = mix(rgb, vec3<f32>(1.0, 0.0, 0.2), 0.7 * d);
    }
    return vec4<f32>(clamp(rgb, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
}
