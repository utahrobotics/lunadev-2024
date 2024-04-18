use compute_shader::create_compute;
use wgpu::include_wgsl;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let mut compute = create_compute::<([f32; 4],), [f32; 4]>(include_wgsl!("mul2.wgsl")).await?;
    let mut compute = move |n: [f32; 4]| compute((n,));
    println!("{:?}", compute([2.0, 2.3, 1.6, 7.6]));

    Ok(())
}
