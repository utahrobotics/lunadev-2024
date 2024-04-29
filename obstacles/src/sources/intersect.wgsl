struct Shape {
    origin: vec3<f32>,
    variant: u32,
    inv_matrix: mat3x3<f32>,
    data: vec3<f32>,
    start_index: u32,
    max_points: u32,
}

@group(0) @binding(0) var<storage, read_write> heights: array<f32>;
@group(0) @binding(1) var<storage, read> points: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> shapes: array<Shape>;
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
    
    let shape = shapes[global_invocation_id.y];
    if atomicLoad(&height_indices[global_invocation_id.y]) >= shape.max_points {
        return;
    }
    let point = point_data.xyz;
    let local_point = shape.inv_matrix * (point - shape.origin);
    switch shape.variant {
        case 0u: {
            let half_height = shape.data.y / 2.0;
            if (local_point.y < -half_height || local_point.y > half_height || length(local_point.xz) > shape.data.x) {
                return;
            }
        }
        default: {
            atomicAdd(&height_indices[global_invocation_id.y], 1u);
            return;
        }
    }
    let index = atomicAdd(&height_indices[global_invocation_id.y], 1u);
    if index >= shape.max_points {
        return;
    }
    heights[index + shape.start_index] = point.y;
}