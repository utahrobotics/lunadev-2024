struct Transform {
    origin: vec3<f32>,
    matrix: mat3x3<f32>,
}

struct Intrinsics {
    transform: Transform,
    min_depth: f32
}

@group(0) @binding(0) var<storage, read> depths: array<f32>;
@group(0) @binding(1) var<storage, read_write> points: array<vec4<f32>>;
@group(0) @binding(2) var<uniform, read> intrinsics: Intrinsics;

@compute
@workgroup_size(1, 1, 1)
fn main(
    @builtin(global_invocation_id) global_invocation_id : vec3<u32>
) {
    let depth = depths[global_invocation_id.x];
    if depth < intrinsics.min_depth {
        points[global_invocation_id.x].w = 0.0;
        return;
    }
    var point = points[global_invocation_id.x].xyz * depth;
    point = intrinsics.transform.matrix * point + intrinsics.transform.origin;
    points[global_invocation_id.x] = vec4<f32>(point, 1.0);
}