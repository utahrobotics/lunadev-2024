struct Shape {
    origin: vec3<f32>,
    variant: u32,
    inv_matrix: mat3x3<f32>,
    start_index: u32,
    data: vec3<f32>,
    max_points: u32,
}

@group(0) @binding(0) var<storage, read_write> heights: array<f32>;
@group(0) @binding(1) var<storage, read> points: array<vec4<f32>>;
@group(0) @binding(2) var<uniform, read> shapes: array<Shape>;
@group(0) @binding(3) var<storage, read_write> height_indices: array<atomic<u32>>;

@compute
@workgroup_size(1, 1, 1)
fn main(
    @builtin(global_invocation_id) global_invocation_id : vec3<u32>
) {
    let point_data = points[global_invocation_id.x];
    if point_data.w == 0.0 {
        return;
    }
    let point = point_data.xyz;
    let shape = shapes[global_invocation_id.y];
    let local_point = shape.inv_matrix * (point - shape.origin);
    switch shape.variant {
        case 0: {
            let half_height = cylinder.data.y / 2.0;
            if (local_point.y < -half_height || local_point.y > half_height || length(local_point.xz) > cylinder.data.x) {
                return;
            }
        }
    }
    let index = atomicAdd(&height_indices[global_invocation_id.y], 1);
    if index >= shape.max_points {
        return;
    }
    returned[index + shape.start_index] = point.y;
}