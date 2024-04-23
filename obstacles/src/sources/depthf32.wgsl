struct Transform {
    matrix: mat3x3<f32>,
    origin: vec3<f32>,
}


struct Cylinder {
    height: f32,
    radius: f32,
    origin: vec3<f32>,
    inv_matrix: mat3x3<f32>,
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
    @builtin(local_invocation_index) local_invocation_index : u32,
) {
    let depth = depths[local_invocation_index];
    if depth == 0.0 {
        returned[local_invocation_index] = 3.40282347e+38;
        return;
    }
    let ray = transform.matrix * rays[local_invocation_index] + transform.origin;
    let point = ray * depth;
    var past_all_shapes = true;
    for (var i: u32 = 0; i < cylinder_count; i++) {
        let cylinder = cylinders[i];
        var local_point = cylinder.inv_matrix * point;
        local_point = local_point - cylinder.origin;
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
        returned[local_invocation_index] = point.y;
        return;
    }
    if past_all_shapes {
        returned[local_invocation_index] = 3.40282347e+38;
    } else {
        returned[local_invocation_index] = -3.40282347e+38;
    }
}