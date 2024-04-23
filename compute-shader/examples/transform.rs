use compute_shader::buffers::{DynamicSize, StaticSize};
use compute_shader::Compute;
use rand::{thread_rng, Rng};
use std::{f32::consts::PI, sync::Arc};
use wgpu::include_wgsl;
use nalgebra::{Translation3, UnitQuaternion, Vector3, Isometry3, UnitVector3};
use std::ops::Deref;

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable, PartialEq)]
struct Transform {
    origin: [f32; 4],
    matrix: [[f32; 4]; 3],
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let compute = Compute::<(&Transform,), Transform>::new(
        include_wgsl!("transform.wgsl"),
        (StaticSize::default(),),
        StaticSize::default(),
        (1, 1, 1),
    )
    .await?;

    for _ in 0..100 {
        let isometry;
        {
            let mut rand = thread_rng();
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
            origin: [isometry.translation.vector.x, isometry.translation.vector.y, isometry.translation.vector.z, 0.0],
            matrix: isometry.rotation.to_rotation_matrix().into_inner().data.0.map(|v| [v[0], v[1], v[2], 0.0]),
        };
        
        let output = compute.call(&transform).await;
        assert_eq!(output.deref(), &transform);
    }

    Ok(())
}
