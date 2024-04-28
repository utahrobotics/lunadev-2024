use std::{any::Any, ops::Deref, sync::Arc};

use async_trait::async_trait;
use compute_shader::{
    buffers::{BufferType, DynamicSize, HostReadOnly, HostReadWrite, HostWriteOnly, ShaderReadOnly, ShaderReadWrite, StaticSize, StorageOnly, UniformOnly},
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

use crate::{utils::RecycledVec, HeightMap, Shape};

impl<D: Send + 'static> DepthMapSource<D> {
    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
}

pub struct DepthMapSource<D> {
    pub max_cylinders: usize,
    depth_sub: Subscriber<D>,
    robot_element_ref: RobotElementRef,

    project_depth: Arc<
        Compute<(
            BufferType<[f32], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[FloatShape], HostWriteOnly, ShaderReadOnly, UniformOnly>,
        )>,
    >,
    height_map_compute: Arc<
        Compute<(
            BufferType<[f32], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[FloatShape], HostWriteOnly, ShaderReadOnly, UniformOnly>,
            BufferType<[u32], HostReadWrite, ShaderReadWrite, UniformOnly>,
        )>,
    >,
}

#[async_trait]
impl<D, I> HeightMap<f32, I> for DepthMapSource<D>
where
    D: Send + Deref<Target = [f32]> + Clone + 'static,
    for<'a> &'a I: IntoIterator<Item = Shape<f32>>,
{
    async fn query_height(&self, shapes: &I) -> Option<RecycledVec<f32>> {
        let mut depth = self.depth_sub.try_recv();
        let mut transform = None;
        if let Some(tmp_depth) = depth {
            let isometry = self.robot_element_ref.get_isometry_from_base();
            let tmp = FloatTransform {
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
}

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct FloatShape {
    origin: [f32; 3],
    variant: u32,
    inv_matrix_2: [[f32; 4]; 2],
    inv_matrix_1: [f32; 3],
    start_index: u32,
    data: [f32; 3],
    max_points: u32,
}

// #[repr(C, align(16))]
// #[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
// struct FloatTransform {
//     origin: [f32; 4],
//     matrix: [[f32; 4]; 3],
// }

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct FloatIntrinsics {
    origin: [f32; 4],
    matrix_2: [[f32; 4]; 3],
    matrix_1: [f32; 3],
    min_depth: f32
}

// impl<D: Deref<Target = [f32]> + Clone + Send + 'static> AsyncNode for DepthMap<f32, D> {
//     type Result = anyhow::Result<()>;

//     async fn run(mut self, context: unros::runtime::RuntimeContext) -> Self::Result {
//         setup_logging!(context);
//         let pixel_count = self.rays.len();
//         let height_within_compute: Compute<
//             (
//                 Option<&[[f32; 4]]>,
//                 Option<&[f32]>,
//                 &[Cylinder<f32>],
//                 &u32,
//                 &Transform<f32>,
//             ),
//             [f32],
//         > = Compute::new(
//             include_wgsl!("depthf32.wgsl"),
//             (
//                 DynamicSize::new(pixel_count),
//                 DynamicSize::new(pixel_count),
//                 DynamicSize::new(self.max_cylinders),
//                 StaticSize::default(),
//                 StaticSize::default(),
//             ),
//             DynamicSize::new(pixel_count),
//             (pixel_count as u32, 1, 1),
//         )
//         .await?;
//         let Some(starting_depth) = self.depth_sub.recv_or_closed().await else {
//             return Ok(());
//         };
//         let mut height_within_compute_rays = Some(self.rays);
//         let mut height_within_compute_depth = Some(starting_depth);

//         let mut height_only_pending = FuturesUnordered::new();
//         let mut height_variance_pending = FuturesUnordered::new();

//         let mut transform = {
//             let isometry = self.robot_element_ref.get_isometry_from_base();
//             Transform {
//                 origin: [
//                     isometry.translation.x,
//                     isometry.translation.y,
//                     isometry.translation.z,
//                     0.0,
//                 ],
//                 matrix: isometry
//                     .rotation
//                     .to_rotation_matrix()
//                     .into_inner()
//                     .data
//                     .0
//                     .map(|v| [v[0], v[1], v[2], 0.0]),
//             }
//         };

//         loop {
//             let request;
//             tokio::select! {
//                 _ = async {
//                     if height_only_pending.is_empty() {
//                         std::future::pending::<()>().await;
//                     } else {
//                         height_only_pending.next().await;
//                     }
//                 } => {
//                     continue;
//                 }
//                 _ = async {
//                     if height_variance_pending.is_empty() {
//                         std::future::pending::<()>().await;
//                     } else {
//                         height_variance_pending.next().await;
//                     }
//                 } => {
//                     continue;
//                 }
//                 option = self.requests.recv() => {
//                     let Some(tmp) = option else {
//                         break Ok(());
//                     };
//                     request = tmp;
//                 }
//             }

//             if let Some(new_depth) = self.depth_sub.try_recv() {
//                 height_within_compute_depth = Some(new_depth.clone());
//                 let isometry = self.robot_element_ref.get_isometry_from_base();
//                 transform = Transform {
//                     origin: [
//                         isometry.translation.x,
//                         isometry.translation.y,
//                         isometry.translation.z,
//                         0.0,
//                     ],
//                     matrix: isometry
//                         .rotation
//                         .to_rotation_matrix()
//                         .into_inner()
//                         .data
//                         .0
//                         .map(|v| [v[0], v[1], v[2], 0.0]),
//                 };
//             }

//             match request {
//                 Request::HeightOnlyWithin { shape, sender } => {
//                     let mut cylinder_buf = vec![];
//                     match shape {
//                         Shape::Cylinder {
//                             radius,
//                             height,
//                             isometry,
//                         } => {
//                             let inv_matrix = isometry
//                                 .rotation
//                                 .to_rotation_matrix()
//                                 .inverse()
//                                 .into_inner();
//                             cylinder_buf.push(Cylinder {
//                                 radius,
//                                 height,
//                                 origin: [
//                                     isometry.translation.x,
//                                     isometry.translation.y,
//                                     isometry.translation.z,
//                                     0.0,
//                                 ],
//                                 inv_matrix: inv_matrix.data.0.map(|v| [v[0], v[1], v[2], 0.0]),
//                             });
//                         }
//                     }
//                     let rays = height_within_compute_rays.take();
//                     let depth = height_within_compute_depth.take();
//                     let transform = Box::leak(Box::new(transform));
//                     while height_only_pending.len() >= self.max_concurrent {
//                         unros::log::info!("a");
//                         let _ = height_only_pending.next().await;
//                     }
//                     height_only_pending.push(async {
//                         let rays = rays;
//                         let depth = depth;
//                         let cylinder_buf = cylinder_buf;
//                         let heights = height_within_compute
//                             .call(
//                                 rays.as_deref(),
//                                 depth.as_deref(),
//                                 &cylinder_buf,
//                                 &(cylinder_buf.len() as u32),
//                                 transform,
//                             )
//                             .await;
//                         unsafe {
//                             let _ = Box::from_raw(transform);
//                         }
//                         let _ = sender.send(
//                             heights
//                                 .into_iter()
//                                 .copied()
//                                 .filter(|n| *n != f32::MAX)
//                                 .map(|n| if n == f32::MIN { None } else { Some(n) })
//                                 .collect(),
//                         );
//                     });
//                 }

//                 Request::HeightVarianceWithin { shape, sender } => {
//                     let mut cylinder_buf = vec![];
//                     match shape {
//                         Shape::Cylinder {
//                             radius,
//                             height,
//                             isometry,
//                         } => {
//                             let inv_matrix = isometry
//                                 .rotation
//                                 .to_rotation_matrix()
//                                 .inverse()
//                                 .into_inner();
//                             cylinder_buf.push(Cylinder {
//                                 radius,
//                                 height,
//                                 origin: [
//                                     isometry.translation.x,
//                                     isometry.translation.y,
//                                     isometry.translation.z,
//                                     0.0,
//                                 ],
//                                 inv_matrix: inv_matrix.data.0.map(|v| [v[0], v[1], v[2], 0.0]),
//                             });
//                         }
//                     }
//                     let rays = height_within_compute_rays.take();
//                     let depth = height_within_compute_depth.take();
//                     let transform = Box::leak(Box::new(transform));
//                     while height_only_pending.len() >= self.max_concurrent {
//                         let _ = height_only_pending.next().await;
//                     }
//                     height_variance_pending.push(async {
//                         let rays = rays;
//                         let depth = depth;
//                         let cylinder_buf = cylinder_buf;
//                         let heights = height_within_compute
//                             .call(
//                                 rays.as_deref(),
//                                 depth.as_deref(),
//                                 &cylinder_buf,
//                                 &(cylinder_buf.len() as u32),
//                                 transform,
//                             )
//                             .await;
//                         unsafe {
//                             let _ = Box::from_raw(transform);
//                         }
//                         let _ = sender.send(
//                             heights
//                                 .into_iter()
//                                 .copied()
//                                 .filter(|n| *n != f32::MAX)
//                                 .map(|n| if n == f32::MIN { None } else { Some(n) })
//                                 .collect(),
//                         );
//                     });
//                 }
//             }
//         }
//     }
// }
