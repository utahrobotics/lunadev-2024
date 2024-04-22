use std::{
    any::Any,
    marker::PhantomData,
    ops::Deref,
    sync::{
        mpsc::{Receiver, SyncSender},
        Arc,
    },
};

use compute_shader::{
    buffers::{DynamicSize, StaticSize},
    wgpu::include_wgsl,
    Compute,
};
use crossbeam::queue::ArrayQueue;
use nalgebra::{UnitVector3, Vector3};
use unros::{
    anyhow,
    float::Float,
    node::{AsyncNode, SyncNode},
    pubsub::{subs::DirectSubscription, Subscriber, WatchSubscriber},
    rayon, setup_logging,
    tokio::sync::{
        mpsc::{Receiver as AsyncReceiver, Sender as AsyncSender, channel as async_channel},
        oneshot,
    },
};

use crate::{sources::HeightDistribution, Shape};

use super::{HeightAndVariance, HeightOnly};

enum Request<N: Float> {
    HeightOnlyWithin {
        origin: Vector3<N>,
        shape: Shape<N>,
        sender: AsyncSender<HeightOnly<N>>,
        sender_sender: oneshot::Sender<AsyncSender<HeightOnly<N>>>,
    },
    HeightVarianceWithin {
        origin: Vector3<N>,
        shape: Shape<N>,
        sender: AsyncSender<HeightAndVariance<N>>,
        sender_sender: oneshot::Sender<AsyncSender<HeightAndVariance<N>>>,
    },
}

struct Queues<N: Float> {
    height: ArrayQueue<(AsyncSender<HeightOnly<N>>, AsyncReceiver<HeightOnly<N>>)>,
    height_variance: ArrayQueue<(
        AsyncSender<HeightAndVariance<N>>,
        AsyncReceiver<HeightAndVariance<N>>,
    )>,
}

pub struct DepthMap<N: Float, D> {
    rays: Arc<[[N; 3]]>,
    map_width: usize,

    pub max_cylinders: usize,

    depth_sub: Subscriber<D>,
    requests: AsyncReceiver<Request<N>>,
}

impl<N: Float, D: Send + 'static> DepthMap<N, D> {
    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
}

struct DepthMapSourceInner<N: Float> {
    queues: Queues<N>,
    requests_sender: AsyncSender<Request<N>>,
}


pub struct DepthMapSource<N: Float> {
    inner: Arc<DepthMapSourceInner<N>>,
}


impl<N: Float> ObstacleSource for DepthMapSource<N> {

}

pub fn new_depth_map<N: Float, D: Send + 'static>(queue_size: usize, rays: impl IntoIterator<Item=UnitVector3<N>>, map_width: usize) -> (DepthMap<N, D>, DepthMapSource<N>) {
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
        },
        DepthMapSource {
            inner: Arc::new(DepthMapSourceInner {
                queues: Queues {
                    height: ArrayQueue::new(queue_size),
                    height_variance: ArrayQueue::new(queue_size),
                },
                requests_sender
            }),
        },
    )
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
struct Cylinder<N: Float> {
    height: N,
    radius: N,
}
unsafe impl<N: Float + bytemuck::Pod + bytemuck::NoUninit> bytemuck::Pod for Cylinder<N> {}
unsafe impl<N: Float + bytemuck::Zeroable + bytemuck::NoUninit> bytemuck::Zeroable for Cylinder<N> {}

// #[repr(C)]
// #[derive(Copy, Clone, Debug)]
// struct HeightAndVarianceReturn<N: Float> {
//     height: N,
//     variance: N,
// }
// unsafe impl<N: Float + bytemuck::Pod + bytemuck::NoUninit> bytemuck::Pod
//     for HeightAndVarianceReturn<N>
// {
// }
// unsafe impl<N: Float + bytemuck::Zeroable + bytemuck::NoUninit> bytemuck::Zeroable
//     for HeightAndVarianceReturn<N>
// {
// }

impl<N: Float + bytemuck::Pod, D: Deref<Target = [N]> + Send + 'static> AsyncNode
    for DepthMap<N, D>
{
    type Result = anyhow::Result<()>;

    async fn run(mut self, context: unros::runtime::RuntimeContext) -> Self::Result {
        setup_logging!(context);
        let Some(mut depth) = self.depth_sub.recv_or_closed().await else {
            return Ok(());
        };
        let shader = if N::is_f32() {
            include_wgsl!("depthf32.wgsl")
        } else {
            include_wgsl!("depthf64.wgsl")
        };
        let pixel_count = self.rays.len();
        let map_height = pixel_count / self.map_width;
        let height_within_compute: Compute<(Option<&[[N; 3]]>, Option<&[N]>, &[N; 3], &[Cylinder<N>]), [N]> =
            Compute::new(
                shader,
                (
                    DynamicSize::new(pixel_count),
                    DynamicSize::new(pixel_count),
                    StaticSize::default(),
                    DynamicSize::new(self.max_cylinders),
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

            match request {
                Request::HeightOnlyWithin {
                    origin,
                    shape,
                    sender,
                    sender_sender,
                } => {
                    cylinder_buf.clear();
                    match shape {
                        Shape::Cylinder { radius, height } => {
                            cylinder_buf.push(Cylinder { radius, height });
                        }
                    }
                    let heights = height_within_compute
                        .call(
                            height_within_compute_rays.take().as_deref(),
                            height_within_compute_depth.take(),
                            &origin.into(),
                            &cylinder_buf,
                        )
                        .await;
                    let _ = sender.send(HeightOnly::from_iter(
                        heights.into_iter().copied().map(|n| {
                            if n.is_sign_negative() {
                                None
                            } else {
                                Some(n)
                            }
                        }),
                    ));
                    let _ = sender_sender.send(sender);
                }

                Request::HeightVarianceWithin {
                    origin,
                    shape,
                    sender,
                    sender_sender,
                } => {
                    cylinder_buf.clear();
                    match shape {
                        Shape::Cylinder { radius, height } => {
                            cylinder_buf.push(Cylinder { radius, height });
                        }
                    }
                    let heights = height_within_compute
                        .call(
                            height_within_compute_rays.take().as_deref(),
                            height_within_compute_depth.take(),
                            &origin.into(),
                            &cylinder_buf,
                        )
                        .await;
                    let _ = sender.send(HeightAndVariance::from_iter(
                        heights.into_iter().copied().map(|n| {
                            if n.is_sign_negative() {
                                None
                            } else {
                                Some(n)
                            }
                        }),
                    ));
                    let _ = sender_sender.send(sender);
                }
            }
        }
    }
}
