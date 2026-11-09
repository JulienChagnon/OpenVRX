// SPDX-License-Identifier: GPL-3.0-or-later
// Live learned denoise (live_filter.rs): a causal linear filter, 5x5 pixels
// x 3 fields over Y, U, V, at source resolution. Rows already holding the
// field in progress read (k-2, k-1, k); rows still holding the previous
// field read (k-3, k-2, k-1), which is that previous field filtered. Output
// goes to two textures the display shader samples instead of the raw ones.

struct Params {
    // Width, height.
    size: vec2<u32>,
    // Unused (alignment).
    pad: vec2<u32>,
    // History slots of fields k-1 (newest), k-2, k-3; w = 1 when all three
    // exist (before that the picture passes through).
    slots: vec4<u32>,
};

@group(0) @binding(0) var<uniform> p: Params;
// Per output plane Y, U, V: 225 taps in (field oldest first, plane, dy, dx)
// order, then the bias (live_filter::TAPS = 226).
@group(0) @binding(1) var<storage, read> coef: array<f32>;
@group(0) @binding(2) var cur_y: texture_2d<f32>;
@group(0) @binding(3) var cur_uv: texture_2d<f32>;
@group(0) @binding(4) var h0_y: texture_2d<f32>;
@group(0) @binding(5) var h1_y: texture_2d<f32>;
@group(0) @binding(6) var h2_y: texture_2d<f32>;
@group(0) @binding(7) var h0_uv: texture_2d<f32>;
@group(0) @binding(8) var h1_uv: texture_2d<f32>;
@group(0) @binding(9) var h2_uv: texture_2d<f32>;
@group(0) @binding(10) var row_new: texture_2d<u32>;
@group(0) @binding(11) var out_y: texture_storage_2d<rgba16float, write>;
@group(0) @binding(12) var out_uv: texture_storage_2d<rgba16float, write>;

const TAPS: u32 = 226u;

// Y, U, V of field `f` at `q`: 0 = current picture, 1 + slot = history.
fn yuv(f: u32, q: vec2<i32>) -> vec3<f32> {
    switch f {
        case 0u: {
            return vec3<f32>(textureLoad(cur_y, q, 0).r, textureLoad(cur_uv, q, 0).rg);
        }
        case 1u: {
            return vec3<f32>(textureLoad(h0_y, q, 0).r, textureLoad(h0_uv, q, 0).rg);
        }
        case 2u: {
            return vec3<f32>(textureLoad(h1_y, q, 0).r, textureLoad(h1_uv, q, 0).rg);
        }
        default: {
            return vec3<f32>(textureLoad(h2_y, q, 0).r, textureLoad(h2_uv, q, 0).rg);
        }
    }
}

@compute @workgroup_size(8, 8)
fn cs_main(@builtin(global_invocation_id) id: vec3<u32>) {
    if (id.x >= p.size.x || id.y >= p.size.y) {
        return;
    }
    let q = vec2<i32>(id.xy);
    var out = yuv(0u, q);
    let r = 2;
    let inside = q.x >= r && q.y >= r && q.x < i32(p.size.x) - r && q.y < i32(p.size.y) - r;
    if (p.slots.w == 1u && inside) {
        var fields: array<u32, 3>;
        if (textureLoad(row_new, vec2<i32>(0, q.y), 0).r == 1u) {
            fields = array<u32, 3>(1u + p.slots.y, 1u + p.slots.x, 0u);
        } else {
            fields = array<u32, 3>(1u + p.slots.z, 1u + p.slots.y, 1u + p.slots.x);
        }
        var acc = vec3<f32>(0.0);
        for (var f = 0u; f < 3u; f = f + 1u) {
            for (var dy = 0; dy < 5; dy = dy + 1) {
                for (var dx = 0; dx < 5; dx = dx + 1) {
                    let v = yuv(fields[f], q + vec2<i32>(dx - r, dy - r));
                    let base = (f * 3u * 5u + u32(dy)) * 5u + u32(dx);
                    for (var plane = 0u; plane < 3u; plane = plane + 1u) {
                        let t = base + plane * 25u;
                        acc = acc + v[plane] * vec3<f32>(coef[t], coef[TAPS + t], coef[2u * TAPS + t]);
                    }
                }
            }
        }
        out = acc + vec3<f32>(coef[TAPS - 1u], coef[2u * TAPS - 1u], coef[3u * TAPS - 1u]);
    }
    textureStore(out_y, q, vec4<f32>(out.x, 0.0, 0.0, 1.0));
    textureStore(out_uv, q, vec4<f32>(out.y, out.z, 0.0, 1.0));
}
