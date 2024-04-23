
@group(0) @binding(0) var<storage, read_write> returned: array<vec3<f32>>;
@group(0) @binding(1) var<storage, read> param: array<vec3<f32>>;

@compute
@workgroup_size(1)
fn main(@builtin(global_invocation_id) global_invocation_id : vec3<u32>) {
    returned[global_invocation_id.x] = param[global_invocation_id.x] * 2.0;
}