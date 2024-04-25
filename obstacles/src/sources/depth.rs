use std::{any::Any, ops::Deref, sync::Arc};

use async_trait::async_trait;
use compute_shader::{
    buffers::{DynamicSize, StaticSize},
    wgpu::include_wgsl,
    Compute,
};
use crossbeam::atomic::AtomicCell;
use futures::{stream::FuturesUnordered, StreamExt};
use nalgebra::UnitVector3;
use rig::RobotElementRef;
use unros::{
    anyhow,
    float::Float,
    node::AsyncNode,
    pubsub::{subs::DirectSubscription, Subscriber},
    setup_logging,
    tokio::{
        self,
        sync::{
            mpsc::{channel as async_channel, Receiver as AsyncReceiver, Sender as AsyncSender},
            oneshot,
        },
    },
};

use crate::Shape;

use super::{HeightAndVariance, HeightOnly, ObstacleSource};

impl<N: Float, D: Send + 'static> DepthMapSource<N, D> {
    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
}

pub struct DepthMapSource<N: Float, D> {
    pub max_cylinders: usize,
    depth_sub: Subscriber<D>,
    robot_element_ref: RobotElementRef,

    height_within_compute: Box<dyn Any + Send + Sync>,
    height_within_compute_data: AtomicCell<Option<(D, Transform<N>)>>,
    height_within_rays: AtomicCell<Option<Arc<[[N; 4]]>>>,
    height_variance_within_rays: AtomicCell<Option<Arc<[[N; 4]]>>>,
}

#[async_trait]
impl<D> ObstacleSource<f32> for DepthMapSource<f32, D>
where
    D: Send + Deref<Target = [f32]> + Clone + 'static,
{
    async fn get_height_only_within(&self, shape: &Shape<f32>) -> Option<HeightOnly<f32>> {
        let mut depth = self.depth_sub.try_recv();
        let mut transform = None;
        if let Some(tmp_depth) = depth {
            let isometry = self.robot_element_ref.get_isometry_from_base();
            let tmp = Transform {
                origin: [
                    isometry.translation.x,
                    isometry.translation.y,
                    isometry.translation.z,
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
            // self.other_data.store(Some((tmp_depth.clone(), tmp)));
            transform = Some(tmp);
        } else if let Some((old_depth, old_transform)) = self.height_within_compute_data.take() {
            depth = Some(old_depth);
            transform = Some(old_transform);
        }
        let mut cylinder_buf = vec![];
        let height_within_compute: &Compute<
            (
                Option<&[[f32; 4]]>,
                Option<&[f32]>,
                &[Cylinder<f32>],
                &u32,
                Option<&Transform<f32>>,
            ),
            [f32],
        > = self.height_within_compute.downcast_ref().unwrap();
        height_within_compute
            .call(
                self.height_within_rays.take().as_deref(),
                depth.as_deref(),
                &[shape.clone()],
                &1,
                &transform,
            )
            .await;
        None
    }
    async fn get_height_and_variance_within(
        &self,
        shape: &Shape<f32>,
    ) -> Option<HeightAndVariance<f32>> {
        None
    }
}

pub fn new_depth_map<N: Float, D: Send + 'static>(
    queue_size: usize,
    rays: impl IntoIterator<Item = UnitVector3<N>>,
    robot_element_ref: RobotElementRef,
) -> (DepthMap<N, D>, DepthMapSource<N>) {
    let (requests_sender, requests) = async_channel(queue_size);
    let rays: Arc<[_]> = rays
        .into_iter()
        .map(|v| [v.x, v.y, v.z, N::zero()])
        .collect();
    if rays.len() > u32::MAX as usize {
        panic!("Too many rays (maximum is u32::MAX)");
    }

    (
        DepthMap {
            rays,
            max_cylinders: 8,
            max_concurrent: 16,
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
    origin: [N; 4],
    inv_matrix: [[N; 4]; 3],
    height: N,
    radius: N,
}
unsafe impl<N: Float + bytemuck::Pod + bytemuck::NoUninit> bytemuck::Pod for Cylinder<N> {}
unsafe impl<N: Float + bytemuck::Zeroable + bytemuck::NoUninit> bytemuck::Zeroable for Cylinder<N> {}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
struct Transform<N: Float> {
    origin: [N; 4],
    matrix: [[N; 4]; 3],
}
unsafe impl<N: Float + bytemuck::Pod + bytemuck::NoUninit> bytemuck::Pod for Transform<N> {}
unsafe impl<N: Float + bytemuck::Zeroable + bytemuck::NoUninit> bytemuck::Zeroable
    for Transform<N>
{
}
impl<D: Deref<Target = [f32]> + Clone + Send + 'static> AsyncNode for DepthMap<f32, D> {
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: unros::runtime::RuntimeContext) -> Self::Result {
        setup_logging!(context);
        let pixel_count = self.rays.len();
        let height_within_compute: Compute<
            (
                Option<&[[f32; 4]]>,
                Option<&[f32]>,
                &[Cylinder<f32>],
                &u32,
                &Transform<f32>,
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
            (pixel_count as u32, 1, 1),
        )
        .await?;
        let Some(starting_depth) = self.depth_sub.recv_or_closed().await else {
            return Ok(());
        };
        let mut height_within_compute_rays = Some(self.rays);
        let mut height_within_compute_depth = Some(starting_depth);

        let mut height_only_pending = FuturesUnordered::new();
        let mut height_variance_pending = FuturesUnordered::new();

        let mut transform = {
            let isometry = self.robot_element_ref.get_isometry_from_base();
            Transform {
                origin: [
                    isometry.translation.x,
                    isometry.translation.y,
                    isometry.translation.z,
                    0.0,
                ],
                matrix: isometry
                    .rotation
                    .to_rotation_matrix()
                    .into_inner()
                    .data
                    .0
                    .map(|v| [v[0], v[1], v[2], 0.0]),
            }
        };

        loop {
            let request;
            tokio::select! {
                _ = async {
                    if height_only_pending.is_empty() {
                        std::future::pending::<()>().await;
                    } else {
                        height_only_pending.next().await;
                    }
                } => {
                    continue;
                }
                _ = async {
                    if height_variance_pending.is_empty() {
                        std::future::pending::<()>().await;
                    } else {
                        height_variance_pending.next().await;
                    }
                } => {
                    continue;
                }
                option = self.requests.recv() => {
                    let Some(tmp) = option else {
                        break Ok(());
                    };
                    request = tmp;
                }
            }

            if let Some(new_depth) = self.depth_sub.try_recv() {
                height_within_compute_depth = Some(new_depth.clone());
                let isometry = self.robot_element_ref.get_isometry_from_base();
                transform = Transform {
                    origin: [
                        isometry.translation.x,
                        isometry.translation.y,
                        isometry.translation.z,
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
            }

            match request {
                Request::HeightOnlyWithin { shape, sender } => {
                    let mut cylinder_buf = vec![];
                    match shape {
                        Shape::Cylinder {
                            radius,
                            height,
                            isometry,
                        } => {
                            let inv_matrix = isometry
                                .rotation
                                .to_rotation_matrix()
                                .inverse()
                                .into_inner();
                            cylinder_buf.push(Cylinder {
                                radius,
                                height,
                                origin: [
                                    isometry.translation.x,
                                    isometry.translation.y,
                                    isometry.translation.z,
                                    0.0,
                                ],
                                inv_matrix: inv_matrix.data.0.map(|v| [v[0], v[1], v[2], 0.0]),
                            });
                        }
                    }
                    let rays = height_within_compute_rays.take();
                    let depth = height_within_compute_depth.take();
                    let transform = Box::leak(Box::new(transform));
                    while height_only_pending.len() >= self.max_concurrent {
                        unros::log::info!("a");
                        let _ = height_only_pending.next().await;
                    }
                    height_only_pending.push(async {
                        let rays = rays;
                        let depth = depth;
                        let cylinder_buf = cylinder_buf;
                        let heights = height_within_compute
                            .call(
                                rays.as_deref(),
                                depth.as_deref(),
                                &cylinder_buf,
                                &(cylinder_buf.len() as u32),
                                transform,
                            )
                            .await;
                        unsafe {
                            let _ = Box::from_raw(transform);
                        }
                        let _ = sender.send(
                            heights
                                .into_iter()
                                .copied()
                                .filter(|n| *n != f32::MAX)
                                .map(|n| if n == f32::MIN { None } else { Some(n) })
                                .collect(),
                        );
                    });
                }

                Request::HeightVarianceWithin { shape, sender } => {
                    let mut cylinder_buf = vec![];
                    match shape {
                        Shape::Cylinder {
                            radius,
                            height,
                            isometry,
                        } => {
                            let inv_matrix = isometry
                                .rotation
                                .to_rotation_matrix()
                                .inverse()
                                .into_inner();
                            cylinder_buf.push(Cylinder {
                                radius,
                                height,
                                origin: [
                                    isometry.translation.x,
                                    isometry.translation.y,
                                    isometry.translation.z,
                                    0.0,
                                ],
                                inv_matrix: inv_matrix.data.0.map(|v| [v[0], v[1], v[2], 0.0]),
                            });
                        }
                    }
                    let rays = height_within_compute_rays.take();
                    let depth = height_within_compute_depth.take();
                    let transform = Box::leak(Box::new(transform));
                    while height_only_pending.len() >= self.max_concurrent {
                        let _ = height_only_pending.next().await;
                    }
                    height_variance_pending.push(async {
                        let rays = rays;
                        let depth = depth;
                        let cylinder_buf = cylinder_buf;
                        let heights = height_within_compute
                            .call(
                                rays.as_deref(),
                                depth.as_deref(),
                                &cylinder_buf,
                                &(cylinder_buf.len() as u32),
                                transform,
                            )
                            .await;
                        unsafe {
                            let _ = Box::from_raw(transform);
                        }
                        let _ = sender.send(
                            heights
                                .into_iter()
                                .copied()
                                .filter(|n| *n != f32::MAX)
                                .map(|n| if n == f32::MIN { None } else { Some(n) })
                                .collect(),
                        );
                    });
                }
            }
        }
    }
}
