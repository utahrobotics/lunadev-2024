use std::{ops::Deref, sync::Arc};

use async_trait::async_trait;
use compute_shader::{
    buffers::{DynamicSize, StaticSize},
    wgpu::include_wgsl,
    Compute,
};
use nalgebra::{Matrix4, UnitVector3};
use rig::RobotElementRef;
use unros::{
    anyhow,
    float::Float,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, Subscriber},
    setup_logging,
    tokio::sync::{
        mpsc::{channel as async_channel, Receiver as AsyncReceiver, Sender as AsyncSender},
        oneshot,
    },
};

use crate::Shape;

use super::{HeightAndVariance, HeightOnly, ObstacleSource};

enum Request<N: Float> {
    HeightOnlyWithin {
        shape: Shape<N>,
        sender: oneshot::Sender<HeightOnly<N>>,
    },
    HeightVarianceWithin {
        shape: Shape<N>,
        sender: oneshot::Sender<HeightAndVariance<N>>,
    },
}

pub struct DepthMap<N: Float, D> {
    rays: Arc<[[N; 3]]>,
    map_width: usize,

    pub max_cylinders: usize,

    depth_sub: Subscriber<D>,
    requests: AsyncReceiver<Request<N>>,
    robot_element_ref: RobotElementRef,
}

impl<N: Float, D: Send + 'static> DepthMap<N, D> {
    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
}

pub struct DepthMapSource<N: Float> {
    requests_sender: AsyncSender<Request<N>>,
}

#[async_trait]
impl<N: Float> ObstacleSource<N> for DepthMapSource<N> {
    async fn get_height_only_within(&self, shape: Shape<N>) -> Option<HeightOnly<N>> {
        let (sender, receiver) = oneshot::channel();
        self.requests_sender
            .send(Request::HeightOnlyWithin { shape, sender })
            .await
            .ok()?;
        Some(receiver.await.unwrap_or_else(|_| HeightOnly {
            height: N::zero(),
            unknown: N::one(),
        }))
    }
    async fn get_height_and_variance_within(
        &self,
        shape: Shape<N>,
    ) -> Option<HeightAndVariance<N>> {
        let (sender, receiver) = oneshot::channel();
        self.requests_sender
            .send(Request::HeightVarianceWithin { shape, sender })
            .await
            .ok()?;
        Some(receiver.await.unwrap_or_else(|_| HeightAndVariance {
            height: N::zero(),
            variance: N::zero(),
            unknown: N::one(),
        }))
    }
}

pub fn new_depth_map<N: Float, D: Send + 'static>(
    queue_size: usize,
    rays: impl IntoIterator<Item = UnitVector3<N>>,
    map_width: usize,
    robot_element_ref: RobotElementRef,
) -> (DepthMap<N, D>, DepthMapSource<N>) {
    let (requests_sender, requests) = async_channel(queue_size);
    let rays: Arc<[_]> = rays.into_iter().map(|v| v.into_inner().into()).collect();
    assert_eq!(rays.len() % map_width, 0);

    (
        DepthMap {
            rays,
            map_width,
            max_cylinders: 8,
            depth_sub: Subscriber::new(1),
            requests,
            robot_element_ref,
        },
        DepthMapSource { requests_sender },
    )
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
struct Cylinder<N: Float> {
    height: N,
    radius: N,
    matrix: [[N; 4]; 4],
    inv_matrix: [[N; 4]; 4],
}
unsafe impl<N: Float + bytemuck::Pod + bytemuck::NoUninit> bytemuck::Pod for Cylinder<N> {}
unsafe impl<N: Float + bytemuck::Zeroable + bytemuck::NoUninit> bytemuck::Zeroable for Cylinder<N> {}

impl<D: Deref<Target = [f32]> + Send + 'static> AsyncNode for DepthMap<f32, D> {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: unros::runtime::RuntimeContext) -> Self::Result {
        setup_logging!(context);
        let Some(mut depth) = self.depth_sub.recv_or_closed().await else {
            return Ok(());
        };
        let pixel_count = self.rays.len();
        let map_height = pixel_count / self.map_width;
        let height_within_compute: Compute<
            (
                Option<&[[f32; 3]]>,
                Option<&[f32]>,
                &[Cylinder<f32>],
                &u32,
                &[[f32; 4]; 4],
            ),
            [f32],
        > = Compute::new(
            include_wgsl!("depthf32.wgsl"),
            (
                DynamicSize::new(pixel_count),
                DynamicSize::new(pixel_count),
                DynamicSize::new(self.max_cylinders),
                StaticSize::default(),
                StaticSize::default(),
            ),
            DynamicSize::new(pixel_count),
            (self.map_width as u32, map_height as u32, 1),
        )
        .await?;
        let mut height_within_compute_rays = Some(self.rays);
        let mut height_within_compute_depth = Some(depth.deref());

        let mut cylinder_buf = vec![];

        loop {
            let Some(request) = self.requests.recv().await else {
                break Ok(());
            };
            if let Some(new_depth) = self.depth_sub.try_recv() {
                depth = new_depth;
                height_within_compute_depth = Some(depth.deref());
            }
            let transform = self
                .robot_element_ref
                .get_isometry_from_base()
                .to_homogeneous()
                .data
                .0;

            match request {
                Request::HeightOnlyWithin { shape, sender } => {
                    cylinder_buf.clear();
                    match shape {
                        Shape::Cylinder {
                            radius,
                            height,
                            isometry,
                        } => {
                            let matrix = isometry.to_homogeneous();
                            let inv_matrix = matrix.try_inverse().unwrap();
                            cylinder_buf.push(Cylinder {
                                radius,
                                height,
                                matrix: matrix.data.0,
                                inv_matrix: inv_matrix.data.0,
                            });
                        }
                    }
                    let heights = height_within_compute
                        .call(
                            height_within_compute_rays.take().as_deref(),
                            height_within_compute_depth.take(),
                            &cylinder_buf,
                            &(cylinder_buf.len() as u32),
                            &transform,
                        )
                        .await;
                    let _ = sender.send(HeightOnly::from_iter(heights.into_iter().copied().filter(|n| *n != f32::MAX).map(
                        |n| {
                            if n == f32::MIN {
                                None
                            } else {
                                Some(n)
                            }
                        },
                    )));
                }

                Request::HeightVarianceWithin { shape, sender } => {
                    cylinder_buf.clear();
                    match shape {
                        Shape::Cylinder {
                            radius,
                            height,
                            isometry,
                        } => {
                            let matrix = isometry.to_homogeneous();
                            let inv_matrix = matrix.try_inverse().unwrap();
                            cylinder_buf.push(Cylinder {
                                radius,
                                height,
                                matrix: matrix.data.0,
                                inv_matrix: inv_matrix.data.0,
                            });
                        }
                    }
                    let heights = height_within_compute
                        .call(
                            height_within_compute_rays.take().as_deref(),
                            height_within_compute_depth.take(),
                            &cylinder_buf,
                            &(cylinder_buf.len() as u32),
                            &transform,
                        )
                        .await;
                    let _ = sender.send(HeightAndVariance::from_iter(
                        heights.into_iter().copied().filter(|n| *n != f32::MAX).map(|n| {
                            if n == f32::MIN {
                                None
                            } else {
                                Some(n)
                            }
                        }),
                    ));
                }
            }
        }
    }
}
