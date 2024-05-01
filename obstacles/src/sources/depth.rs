use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

use async_trait::async_trait;
use compute_shader::{
    buffers::{
        BufferType, DynamicSize, HostReadOnly, HostReadWrite, HostWriteOnly, OpaqueBuffer,
        ShaderReadOnly, ShaderReadWrite, StorageOnly, UniformOnly,
    },
    wgpu::include_wgsl,
    Compute,
};
use crossbeam::queue::ArrayQueue;
use nalgebra::UnitVector3;
use rand::{prelude::SliceRandom, thread_rng};
use rig::RobotElementRef;
use unros::{
    anyhow::{self, Context},
    pubsub::{subs::DirectSubscription, Subscriber},
    tokio::sync::Mutex as AsyncMutex,
};

use crate::{utils::RecycledVec, HeightMap, HeightQuery, Shape};

pub struct DepthMapSource<D: IntoDepthData> {
    min_depth: f32,
    max_shapes: u32,
    max_points_per_shape: usize,
    depth_sub: Subscriber<D>,
    robot_element_ref: RobotElementRef,
    points_buffer: AsyncMutex<OpaqueBuffer>,
    point_count: u32,

    heights_queue: ArrayQueue<Vec<f32>>,
    shapes_queue: ArrayQueue<Vec<FloatShape>>,
    shape_indices_queue: ArrayQueue<Box<[u32]>>,
    point_indices_queue: ArrayQueue<Box<[u32]>>,

    project_depth: Arc<
        Compute<(
            BufferType<[f32], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[[f32; 4]], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<FloatIntrinsics, HostWriteOnly, ShaderReadOnly, UniformOnly>,
        )>,
    >,
    height_map_compute: Arc<
        Compute<(
            BufferType<[f32], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[FloatShape], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[u32], HostReadWrite, ShaderReadWrite, StorageOnly>,
            BufferType<[u32], HostWriteOnly, ShaderReadOnly, StorageOnly>,
        )>,
    >,
}

impl<D: Send + IntoDepthData + 'static> DepthMapSource<D> {
    pub async fn new(
        rays: impl IntoIterator<Item = UnitVector3<f32>>,
        robot_element_ref: RobotElementRef,
        min_depth: f32,
        max_shapes: u32,
        max_points_per_shape: usize,
    ) -> anyhow::Result<Self> {
        let rays: Box<[[f32; 4]]> = rays
            .into_iter()
            .map(|ray| [ray.x, ray.y, ray.z, 0.0])
            .collect();

        let project_depth = Compute::<(
            BufferType<[f32], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[[f32; 4]], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<FloatIntrinsics, HostWriteOnly, ShaderReadOnly, UniformOnly>,
        )>::new(
            include_wgsl!("depth.wgsl"),
            BufferType::new_dyn(rays.len()),
            BufferType::new_dyn(rays.len()),
            BufferType::new_dyn(rays.len()),
            BufferType::new(),
        )
        .await?;

        project_depth.write_args((), rays.deref(), (), ()).await;

        let height_map_compute = Compute::<(
            BufferType<[f32], HostReadOnly, ShaderReadWrite, StorageOnly>,
            BufferType<[[f32; 4]], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[FloatShape], HostWriteOnly, ShaderReadOnly, StorageOnly>,
            BufferType<[u32], HostReadWrite, ShaderReadWrite, StorageOnly>,
            BufferType<[u32], HostWriteOnly, ShaderReadOnly, StorageOnly>,
        )>::new(
            include_wgsl!("intersect.wgsl"),
            BufferType::new_dyn(max_points_per_shape * max_shapes as usize),
            BufferType::new_dyn(rays.len()),
            BufferType::new_dyn(max_shapes as usize),
            BufferType::new_dyn(max_shapes as usize),
            BufferType::new_dyn(rays.len()),
        )
        .await?;

        Ok(Self {
            min_depth,
            max_shapes,
            max_points_per_shape,
            depth_sub: Subscriber::new(1),
            robot_element_ref,
            point_count: rays
                .len()
                .try_into()
                .context("The maximum number of rays is u32::MAX")?,
            points_buffer: AsyncMutex::new(
                OpaqueBuffer::new(DynamicSize::<[f32; 4]>::new(rays.len())).await?,
            ),

            heights_queue: ArrayQueue::new(16),
            shapes_queue: ArrayQueue::new(16),
            shape_indices_queue: ArrayQueue::new(16),
            point_indices_queue: ArrayQueue::new(16),

            project_depth: Arc::new(project_depth),
            height_map_compute: Arc::new(height_map_compute),
        })
    }

    pub fn create_depth_subscription(&self) -> DirectSubscription<D> {
        self.depth_sub.create_subscription()
    }
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
        if queries.is_empty() {
            return Some(RecycledVec::default());
        }
        let mut shapes = self
            .shapes_queue
            .pop()
            .map(|mut buf| {
                buf.clear();
                buf
            })
            .unwrap_or_else(|| Vec::with_capacity(queries.len()));
        let mut heights_count = 0usize;
        shapes.extend(queries.iter().copied().map(|mut query| {
            assert!(query.max_points <= self.max_points_per_shape);
            let (data, variant) = match query.shape {
                Shape::Cylinder { height, radius } => ([radius, height, 0.0], 0),
            };
            let element_isometry = self.robot_element_ref.get_global_isometry();
            query.isometry = element_isometry.inv_mul(&query.isometry);
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
                inv_matrix: matrix.map(|row| [row[0], row[1], row[2], 0.0]),
                start_index: old_start_index.try_into().unwrap(),
                data,
                max_points: query.max_points.try_into().unwrap(),
                padding: [0; 3],
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
        let mut shape_indices_buf = self
            .shape_indices_queue
            .pop()
            .map(|mut buf| {
                buf.iter_mut().for_each(|x| *x = 0);
                buf
            })
            .unwrap_or_else(|| {
                std::iter::repeat(0)
                    .take(self.max_shapes as usize)
                    .collect()
            });
        let mut point_indices_buf = self
            .point_indices_queue
            .pop()
            .unwrap_or_else(|| (0..self.point_count).into_iter().collect());

        point_indices_buf.shuffle(&mut thread_rng());

        let mut pass = if let Some(depth) = self.depth_sub.try_recv() {
            let depth = depth.into_depth_data();
            let isometry = self.robot_element_ref.get_isometry_from_base();
            let matrix = isometry.rotation.to_rotation_matrix().into_inner().data.0;
            let intrinsics = FloatIntrinsics {
                origin: [
                    isometry.translation.x,
                    isometry.translation.y,
                    isometry.translation.z,
                ],
                min_depth: self.min_depth,
                matrix: matrix.map(|row| [row[0], row[1], row[2], 0.0]),
            };
            let mut guard = self.points_buffer.lock().await;
            self.project_depth
                .new_pass(depth.deref(), (), (), &intrinsics)
                .workgroup_size(self.point_count, 1, 1)
                .call((), (), guard.deref_mut(), ())
                .await;
            self.height_map_compute.new_pass(
                (),
                guard.deref(),
                shapes.deref(),
                shape_indices_buf.deref(),
                point_indices_buf.deref(),
            )
        } else {
            self.height_map_compute.new_pass(
                (),
                (),
                shapes.deref(),
                shape_indices_buf.deref(),
                point_indices_buf.deref(),
            )
        };
        pass.workgroup_size = (self.point_count, shapes.len() as u32, 1);

        let indices_mut = shape_indices_buf.deref_mut().split_at_mut(shapes.len()).0;
        let _ = self.point_indices_queue.push(point_indices_buf);

        pass.call(heights_buf.deref_mut(), (), (), indices_mut, ())
            .await;

        let mut result = RecycledVec::default();
        for (&height_count, shape) in shape_indices_buf.iter().zip(&shapes) {
            let mut heights = RecycledVec::default();
            // if height_count as usize > heights_buf.split_at(shape.start_index as usize).1.len() {
            //     println!("{} {}", heights_buf.split_at(shape.start_index as usize).1.len(), height_count);
            // }
            let heights_slice = heights_buf
                .split_at(shape.start_index as usize)
                .1
                .split_at(height_count.min(shape.max_points) as usize)
                .0;
            heights.extend_from_slice(heights_slice);
            result.push(heights);
        }

        let _ = self.heights_queue.push(heights_buf);
        let _ = self.shapes_queue.push(shapes);
        let _ = self.shape_indices_queue.push(shape_indices_buf);

        Some(result)
    }
}

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct FloatShape {
    origin: [f32; 3],
    variant: u32,
    inv_matrix: [[f32; 4]; 3],
    data: [f32; 3],
    start_index: u32,
    max_points: u32,
    padding: [u32; 3],
}

#[repr(C, align(16))]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct FloatIntrinsics {
    origin: [f32; 3],
    min_depth: f32,
    matrix: [[f32; 4]; 3],
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
