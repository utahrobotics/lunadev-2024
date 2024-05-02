@group(0) @binding(0) var<storage, read_write> vectors: array<vec3<u32>>;

@compute
@workgroup_size(1)
fn main(@builtin(global_invocation_id) global_invocation_id : vec3<u32>) {
    vectors[global_invocation_id.x] = vectors[global_invocation_id.x] * 2;
}