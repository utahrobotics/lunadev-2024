use compute_shader::buffers::DynamicSize;
use compute_shader::Compute;
use rand::{thread_rng, Rng};
use std::sync::Arc;
use wgpu::include_wgsl;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let compute = Compute::<(&[[f32; 4]],), [[f32; 4]]>::new(
        include_wgsl!("mul2.wgsl"),
        (DynamicSize::new(50),),
        DynamicSize::new(50),
        (50, 1, 1),
    )
    .await?;
    let compute = Arc::new(compute);
    let compute2 = compute.clone();

    macro_rules! compute {
        ($compute: ident) => {{
            let input: Box<[_]> = {
                let mut rand = thread_rng();
                (0..50)
                    .map(|_| {
                        [
                            rand.gen_range(-10.0..10.0),
                            rand.gen_range(-10.0..10.0),
                            rand.gen_range(-10.0..10.0),
                            0.0
                        ]
                    })
                    .collect()
            };
            let output = $compute.call(&input).await;
            for (i, (input, output)) in input.iter().zip(output.iter()).enumerate() {
                assert_eq!(
                    [input[0] * 2.0, input[1] * 2.0, input[2] * 2.0],
                    [output[0], output[1], output[2]]
                );
            }
        }};
    }

    let task = tokio::spawn(async move {
        for _ in 0..100 {
            compute!(compute2);
        }
    });
    for _ in 0..100 {
        compute!(compute);
    }
    task.await.unwrap();

    Ok(())
}
