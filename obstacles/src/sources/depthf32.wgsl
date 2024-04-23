struct Transform {
    origin: vec3<f32>,
    matrix: mat3x3<f32>,
}


struct Cylinder {
    origin: vec3<f32>,
    inv_matrix: mat3x3<f32>,
    height: f32,
    radius: f32,
}

@group(0) @binding(0) var<storage, read_write> returned: array<f32>;
@group(0) @binding(1) var<storage, read> rays: array<vec3<f32>>;
@group(0) @binding(2) var<storage, read> depths: array<f32>;
@group(0) @binding(3) var<storage, read> cylinders: array<Cylinder>;
@group(0) @binding(4) var<storage, read> cylinder_count: u32;
@group(0) @binding(5) var<storage, read> transform: Transform;

@compute
@workgroup_size(1, 1, 1)
fn main(
    @builtin(global_invocation_id) global_invocation_id : vec3<u32>
) {
    let index = global_invocation_id.x;
    let depth = depths[index];
    if depth == 0.0 {
        returned[index] = 3.40282347e+38;
        return;
    }
    let point = transform.matrix * rays[index] * depth + transform.origin;
    var past_all_shapes = true;
    for (var i: u32 = 0; i < cylinder_count; i++) {
        let cylinder = cylinders[i];
        let local_point = cylinder.inv_matrix * (point - cylinder.origin);
        let half_height = cylinder.height / 2.0;
        if (local_point.y < -half_height || local_point.y > half_height || length(local_point.xz) > cylinder.radius) {
            if past_all_shapes {
                let distance = length(cylinder.origin);
                if distance < depth {
                    past_all_shapes = false;
                }
            }
            continue;
        }
        returned[index] = point.y;
        return;
    }
    if past_all_shapes {
        returned[index] = 3.40282347e+38;
    } else {
        returned[index] = -3.40282347e+38;
    }
}