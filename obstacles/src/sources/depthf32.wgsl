struct Cylinder {
    height: f32,
    radius: f32,
    inv_matrix: mat4x4<f32>,
}

@group(0) @binding(0) var<storage, read_write> returned: array<f32>;
@group(0) @binding(1) var<storage, read> rays: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read> depths: array<f32>;
@group(0) @binding(3) var<storage, read> cylinders: array<Cylinder>;
@group(0) @binding(4) var<storage, read> cylinder_count: u32;
@group(0) @binding(5) var<storage, read> transform: mat4x4<f32>;

@compute
@workgroup_size(1, 1, 1)
fn main(
    @builtin(local_invocation_index) local_invocation_index : u32,
) {
    let point = transform * rays[local_invocation_index] * depths[local_invocation_index];
    for (var i = 0; i < cylinder_count; i++) {
        let cylinder = cylinders[i];
        let local_point = cylinders.inv_matrix * point;
        let half_height = cylinder.height / 2.0;
        if (local_point.y < -half_height || local_point.y > half_height) {
            continue;
        }
        if (length(local_point.xz) > cylinder.radius) {
            continue;
        }
        returned[index] = point.y;
    }
    returned[index] = f32.min_positive;
}