use compute_shader::buffers::DynamicSize;
use compute_shader::Compute;
use std::ops::Deref;
use std::sync::Arc;
use wgpu::include_wgsl;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let compute = Compute::<(&[f32],), [f32]>::new(
        include_wgsl!("mul2.wgsl"),
        (DynamicSize::new(4),),
        DynamicSize::new(4),
        (1, 1, 1),
    )
    .await?;
    let compute = Arc::new(compute);
    let compute2 = compute.clone();

    let task = tokio::spawn(async move {
        for _ in 0..100 {
            assert_eq!(
                "[4.0, 4.6, 3.2, 15.2]",
                format!("{:?}", compute2.call(&[2.0, 2.3, 1.6, 7.6]).await.deref())
            );
        }
    });
    for _ in 0..100 {
        assert_eq!(
            "[9.4, 12.6, 13.8, 4.2]",
            format!("{:?}", compute.call(&[4.7, 6.3, 6.9, 2.1]).await.deref())
        );
    }
    task.await.unwrap();

    Ok(())
}
