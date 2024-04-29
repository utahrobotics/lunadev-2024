use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

use async_trait::async_trait;
use compute_shader::{
    buffers::{
        BufferType, HostReadOnly, HostReadWrite, HostWriteOnly, OpaqueBuffer, ShaderReadOnly,
        ShaderReadWrite, StorageOnly, UniformOnly,
    },
    Compute,
};
use crossbeam::queue::ArrayQueue;
use rig::RobotElementRef;
use unros::{
    pubsub::{subs::DirectSubscription, Subscriber},
    tokio::sync::Mutex as AsyncMutex,
};

use crate::{utils::RecycledVec, HeightMap, HeightQuery, Shape};

impl<D: Send + IntoDepthData + 'static> DepthMapSource<D> {
    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
}

pub struct DepthMapSource<D: IntoDepthData> {
    min_depth: f32,
    max_shapes: usize,
    depth_sub: Subscriber<D>,
    robot_element_ref: RobotElementRef,
    points_buffer: AsyncMutex<OpaqueBuffer>,

    heights_queue: ArrayQueue<Vec<f32>>,
    shapes_queue: ArrayQueue<Vec<FloatShape>>,
    indices_queue: ArrayQueue<Box<[u32]>>,

    project_depth: Arc<
        Compute<(
            BufferType<[f32], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[[f32; 4]], HostReadWrite, ShaderReadWrite, StorageOnly>,
            BufferType<FloatIntrinsics, HostWriteOnly, ShaderReadOnly, UniformOnly>,
        )>,
    >,
    height_map_compute: Arc<
        Compute<(
            BufferType<[f32], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[FloatShape], HostWriteOnly, ShaderReadOnly, UniformOnly>,
            BufferType<[u32], HostReadWrite, ShaderReadWrite, StorageOnly>,
        )>,
    >,
}

#[async_trait]
impl<D> HeightMap<f32> for DepthMapSource<D>
where
    D: Send + IntoDepthData + 'static,
{
    async fn query_height<'a>(
        &self,
        queries: Arc<RecycledVec<HeightQuery<f32>>>,
    ) -> Option<RecycledVec<RecycledVec<f32>>> {
        let mut shapes = self
            .shapes_queue
            .pop()
            .map(|mut buf| {
                buf.clear();
                buf
            })
            .unwrap_or_else(|| Vec::with_capacity(queries.len()));
        let mut heights_count = 0usize;
        shapes.extend(queries.iter().map(|query| {
            let (data, variant) = match query.shape {
                Shape::Cylinder { height, radius } => ([radius, height, 0.0], 0),
            };
            let matrix = query
                .isometry
                .rotation
                .to_rotation_matrix()
                .inverse()
                .into_inner()
                .data
                .0;
            let old_start_index = heights_count;
            heights_count += query.max_points;
            FloatShape {
                origin: query.isometry.translation.into(),
                variant,
                inv_matrix_2: [matrix[0], matrix[1]].map(|row| [row[0], row[1], row[2], 0.0]),
                inv_matrix_1: matrix[2],
                start_index: old_start_index.try_into().unwrap(),
                data,
                max_points: query.max_points.try_into().unwrap(),
            }
        }));
        let mut heights_buf = self
            .heights_queue
            .pop()
            .map(|mut buf| {
                buf.resize(heights_count, 0.0);
                buf
            })
            .unwrap_or_else(|| vec![0.0; heights_count]);
        let mut indices_buf = self
            .indices_queue
            .pop()
            .map(|mut buf| {
                buf.iter_mut().for_each(|x| *x = 0);
                buf
            })
            .unwrap_or_else(|| std::iter::repeat(0).take(self.max_shapes).collect());

        let pass = if let Some(depth) = self.depth_sub.try_recv() {
            let depth = depth.into_depth_data();
            let isometry = self.robot_element_ref.get_isometry_from_base();
            let matrix = isometry.rotation.to_rotation_matrix().into_inner().data.0;
            let intrinsics = FloatIntrinsics {
                origin: [
                    isometry.translation.x,
                    isometry.translation.y,
                    isometry.translation.z,
                    0.0,
                ],
                matrix_2: [matrix[0], matrix[1]].map(|row| [row[0], row[1], row[2], 0.0]),
                matrix_1: matrix[2],
                min_depth: self.min_depth,
            };
            let mut guard = self.points_buffer.lock().await;
            self.project_depth
                .new_pass(depth.deref(), (), &intrinsics)
                .call((), guard.deref_mut(), ())
                .await;
            self.height_map_compute
                .new_pass((), guard.deref(), shapes.deref(), indices_buf.deref())
        } else {
            self.height_map_compute
                .new_pass((), (), shapes.deref(), indices_buf.deref())
        };

        let indices_mut = indices_buf.deref_mut().split_at_mut(shapes.len()).0;

        pass.call(heights_buf.deref_mut(), (), (), indices_mut)
            .await;

        let mut heights_iter = heights_buf.iter().copied();
        let mut result = RecycledVec::default();
        for height_count in indices_buf.iter().take(shapes.len()).copied() {
            let mut heights = RecycledVec::default();
            heights.extend(heights_iter.by_ref().take(height_count as usize));
            result.push(heights);
        }

        Some(result)
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

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct FloatIntrinsics {
    origin: [f32; 4],
    matrix_2: [[f32; 4]; 2],
    matrix_1: [f32; 3],
    min_depth: f32,
}

pub trait IntoDepthData {
    type Data: Send + Deref<Target = [f32]>;
    fn into_depth_data(self) -> Self::Data;
}

impl<D: Deref<Target = [f32]> + Send> IntoDepthData for D {
    type Data = D;

    fn into_depth_data(self) -> Self::Data {
        self
    }
}
