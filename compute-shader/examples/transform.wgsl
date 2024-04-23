struct Transform {
    origin: vec3<f32>,
    matrix: mat3x3<f32>,
}

@group(0) @binding(0) var<storage, read_write> returned: Transform;
@group(0) @binding(1) var<storage, read> transform: Transform;

@compute
@workgroup_size(1)
fn main() {
    returned = transform;
}