use compute_shader::buffers::DynamicSize;
use compute_shader::create_compute;
use wgpu::include_wgsl;

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let mut compute = create_compute::<(&[f32],), Vec<f32>>(
        include_wgsl!("mul2.wgsl"),
        (DynamicSize::new(4),),
        DynamicSize::new(4),
        (1, 1, 1),
    )?;
    println!("{:?}", compute((&[2.0, 2.3, 1.6, 7.6],)));

    Ok(())
}
