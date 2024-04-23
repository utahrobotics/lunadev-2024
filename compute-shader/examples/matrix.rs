use compute_shader::buffers::{DynamicSize, StaticSize};
use compute_shader::Compute;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use rand::{thread_rng, Rng};
use std::{f32::consts::PI, sync::Arc};
use wgpu::include_wgsl;

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Transform {
    origin: [f32; 4],
    matrix: [[f32; 4]; 3],
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let compute = Compute::<(&[[f32; 4]], &Transform), [[f32; 4]]>::new(
        include_wgsl!("matrix.wgsl"),
        (DynamicSize::new(5), StaticSize::default()),
        DynamicSize::new(5),
        (5, 1, 1),
    )
    .await?;

    for _ in 0..100 {
        let input: Box<[_]>;
        let isometry;
        {
            let mut rand = thread_rng();
            input = (0..5)
                .map(|_| {
                    [
                        rand.gen_range(-10.0..10.0),
                        rand.gen_range(-10.0..10.0),
                        rand.gen_range(-10.0..10.0),
                        0.0,
                    ]
                })
                .collect();
            let rotation_axis = Vector3::new(
                rand.gen_range(-1.0..1.0),
                rand.gen_range(-1.0..1.0),
                rand.gen_range(-1.0..1.0),
            );
            let rotation_axis = UnitVector3::new_normalize(rotation_axis);
            let rotation_angle = rand.gen_range(-PI..PI);
            let rotation = UnitQuaternion::from_axis_angle(&rotation_axis, rotation_angle);
            isometry = Isometry3::from_parts(
                Translation3::new(
                    rand.gen_range(-10.0..10.0),
                    rand.gen_range(-10.0..10.0),
                    rand.gen_range(-10.0..10.0),
                ),
                rotation,
            );
        };

        let transform = Transform {
            origin: [
                isometry.translation.vector.x,
                isometry.translation.vector.y,
                isometry.translation.vector.z,
                0.0,
            ],
            matrix: isometry
                .rotation
                .to_rotation_matrix()
                .into_inner()
                .data
                .0
                .map(|v| [v[0], v[1], v[2], 0.0]),
        };

        let output = compute.call(&input, &transform).await;
        for (input, output) in input.iter().zip(output.iter()) {
            let input = Point3::new(input[0], input[1], input[2]);
            let output = Point3::new(output[0], output[1], output[2]);
            assert_eq!(isometry * input, output);
        }
    }

    Ok(())
}
