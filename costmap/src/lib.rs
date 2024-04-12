#![feature(new_uninit, ptr_metadata, alloc_layout_extra)]

use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
};

use dst_init::{dst, BoxExt, Slice, SliceExt};
use image::GrayImage;
use nalgebra::{Isometry3, Point2, Point3, RealField};
use quadtree_rs::{area::AreaBuilder, Quadtree};
use rig::RobotElementRef;
use simba::scalar::{SubsetOf, SupersetOf};
use unros::{
    node::AsyncNode, pubsub::{subs::Subscription, Publisher, PublisherRef, Subscriber}, rayon::iter::{IntoParallelIterator, ParallelIterator}, runtime::RuntimeContext, setup_logging, DontDrop
};

#[derive(Clone, Copy)]
struct HeightCell<N> {
    total_height: N,
    count: usize,
}

#[derive(Clone)]
pub struct Points<T> {
    pub points: T,
    pub robot_element: RobotElementRef,
}

struct CostmapFrame<N> {
    quadtree: Quadtree<usize, HeightCell<N>>,
    max_density: usize,
    min_x: isize,
    min_y: isize,
    max_height: N,
    min_height: N,
    resolution: N,
    isometry: Isometry3<N>,
}

#[dst]
struct CostmapInner<N> {
    point_count: usize,
    threshold: N,
    frames: [Arc<CostmapFrame<N>>],
}

#[derive(Clone)]
pub struct Costmap<N = f64> {
    inner: Arc<CostmapInner<N>>,
}

impl<
        N: RealField + Copy + SupersetOf<usize> + SupersetOf<isize> + SupersetOf<i64> + SupersetOf<u8>,
    > Costmap<N>
{
    pub fn is_global_point_safe(&self, point: Point3<N>, radius: N, max_diff: N) -> bool {
        assert!(!radius.is_negative());
        assert!(!max_diff.is_negative());

        for frame in self.inner.frames.iter() {
            let point3d = frame.isometry.inverse_transform_point(&point);
            let mut point2d = Point2::<isize>::new(
                (point3d.x / frame.resolution).round().to_subset_unchecked(),
                (point3d.z / frame.resolution).round().to_subset_unchecked(),
            );
            point2d.x -= frame.min_x;
            point2d.y -= frame.min_y;
            if point2d.x < 0 || point2d.y < 0 {
                continue;
            }
            let point2d = Point2::new(point2d.x as usize, point2d.y as usize);
            let mut radius_int: usize = (radius / frame.resolution).round().to_subset_unchecked();
            if radius_int == 0 {
                radius_int = 1;
            }
            let cells = frame.quadtree.query(
                AreaBuilder::default()
                    .anchor(quadtree_rs::point::Point {
                        x: point2d.x.saturating_sub(radius_int),
                        y: point2d.y.saturating_sub(radius_int),
                    })
                    .dimensions((radius_int * 2, radius_int * 2))
                    .build()
                    .unwrap(),
            );

            let threshold: usize = (self.inner.threshold * nalgebra::convert(frame.max_density))
                .round()
                .to_subset_unchecked();

            for cell in cells {
                let anchor = cell.anchor();
                let cell = cell.value_ref();

                if cell.count < threshold {
                    continue;
                }

                let sum = anchor.x.abs_diff(point2d.x).pow(2) + anchor.y.abs_diff(point2d.y).pow(2);
                let sum: N = nalgebra::convert(sum);

                if sum.sqrt() * frame.resolution > radius {
                    continue;
                }

                let height = cell.total_height / nalgebra::convert(cell.count) - point3d.y;
                if height.abs() > max_diff {
                    return false;
                }
            }
        }
        true
    }

    pub fn is_global_point_in_bounds(&self, point: Point3<N>) -> bool {
        for frame in self.inner.frames.iter() {
            let point3d = frame.isometry.inverse_transform_point(&point);
            let mut point2d = Point2::<isize>::new(
                (point3d.x / frame.resolution).round().to_subset_unchecked(),
                (point3d.z / frame.resolution).round().to_subset_unchecked(),
            );
            point2d.x -= frame.min_x;
            point2d.y -= frame.min_y;
            if point2d.x >= 0
                && point2d.y >= 0
                && point2d.x < frame.quadtree.width() as isize
                && point2d.y < frame.quadtree.height() as isize
            {
                return true;
            }
        }
        false
    }

    pub fn get_obstacle_map(
        &self,
        point: Point3<N>,
        resolution: N,
        width: u32,
        height: u32,
        radius: N,
        max_diff: N,
    ) -> GrayImage {
        assert!(resolution.is_positive());
        let data: Vec<_> = (0..height as i64)
            .into_par_iter()
            .flat_map(|mut y| {
                y -= height as i64 / 2;
                (0..width as i64).into_par_iter().map(move |mut x| {
                    x -= width as i64 / 2;
                    let mut transformed_point = point;
                    transformed_point.x += nalgebra::convert::<_, N>(x) * resolution;
                    transformed_point.z += nalgebra::convert::<_, N>(y) * resolution;
                    if self.is_global_point_safe(transformed_point, radius, max_diff) {
                        0u8
                    } else {
                        255u8
                    }
                })
            })
            .collect();

        GrayImage::from_vec(width, height, data).expect("Failed to create obstacle map")
    }

    pub fn get_cost_map(
        &self,
        point: Point3<N>,
        resolution: N,
        width: u32,
        height: u32,
    ) -> GrayImage {
        assert!(resolution.is_positive());
        let data: Vec<_> = (0..height as i64)
            .into_par_iter()
            .flat_map(|mut y| {
                y -= height as i64 / 2;
                (0..width as i64).into_par_iter().map(move |mut x| {
                    x -= width as i64 / 2;
                    let mut transformed_point = point;
                    transformed_point.x += nalgebra::convert::<_, N>(x) * resolution;
                    transformed_point.z += nalgebra::convert::<_, N>(y) * resolution;

                    let mut total_height = N::zero();
                    let mut count = 0usize;
                    let mut max_height = N::zero();
                    let mut min_height = N::zero();

                    for frame in self.inner.frames.iter() {
                        max_height = max_height.max(frame.max_height);
                        min_height = min_height.min(frame.min_height);

                        let point3d = frame.isometry.inverse_transform_point(&transformed_point);
                        let mut point2d = Point2::<isize>::new(
                            (point3d.x / frame.resolution).round().to_subset_unchecked(),
                            (point3d.z / frame.resolution).round().to_subset_unchecked(),
                        );
                        point2d.x -= frame.min_x;
                        point2d.y -= frame.min_y;
                        if point2d.x < 0 || point2d.y < 0 {
                            continue;
                        }
                        let mut cells = frame.quadtree.query(
                            AreaBuilder::default()
                                .anchor(quadtree_rs::point::Point {
                                    x: point2d.x as usize,
                                    y: point2d.y as usize,
                                })
                                .dimensions((1, 1))
                                .build()
                                .unwrap(),
                        );

                        let threshold: usize = (self.inner.threshold
                            * nalgebra::convert(frame.max_density))
                        .round()
                        .to_subset_unchecked();

                        if let Some(cell) = cells.next() {
                            let cell = cell.value_ref();
                            if cell.count < threshold {
                                continue;
                            }
                            let height = cell.total_height / nalgebra::convert(cell.count);
                            total_height += height;
                            count += 1;
                        }

                        assert!(cells.next().is_none());
                    }

                    (
                        total_height / nalgebra::convert(count),
                        max_height,
                        min_height,
                    )
                })
            })
            .collect();

        let comparator = |a: &N, b: &N| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal);
        let max_height = data.iter().map(|(_, h, _)| *h).max_by(comparator).unwrap();
        let min_height = data.iter().map(|(_, _, h)| *h).min_by(comparator).unwrap();
        let divisor = max_height.abs().max(min_height.abs());

        let data: Vec<u8> = data
            .into_iter()
            .map(|(height, _, _)| {
                let height = height.abs() / divisor * nalgebra::convert(255.0);
                height.round().to_subset_unchecked()
            })
            .collect();

        GrayImage::from_vec(width, height, data).expect("Failed to create cost map")
    }
}

pub struct CostmapGenerator<N: RealField + Copy = f32> {
    pub threshold: N,
    pub window_length: usize,
    quadtree_sub: Subscriber<CostmapFrame<N>>,
    dont_drop: DontDrop,
    costmap_pub: Publisher<Costmap<N>>,
}

impl CostmapGenerator {
    pub fn new(frame_buffer_size: usize) -> Self {
        Self {
            threshold: 0.5,
            window_length: 10,
            quadtree_sub: Subscriber::new(frame_buffer_size),
            dont_drop: DontDrop::new("costmap-generator"),
            costmap_pub: Default::default(),
        }
    }
}

impl<N: RealField + Copy + SupersetOf<usize> + SupersetOf<isize>> CostmapGenerator<N> {
    pub fn create_points_sub<T>(&self, resolution: N) -> impl Subscription<Item = Points<T>>
    where
        T: IntoIterator<Item = Point3<N>>,
        f32: SubsetOf<N>,
    {
        assert!(resolution.is_positive());
        self.quadtree_sub
            .create_subscription()
            .filter_map(move |original_points: Points<T>| {
                let points: Box<[_]> = original_points
                    .points
                    .into_iter()
                    .map(|mut p| {
                        let iso: Isometry3<N> = nalgebra::convert(
                            original_points.robot_element.get_isometry_from_base(),
                        );
                        p = iso.transform_point(&p);

                        let pt = Point2::<isize>::new(
                            (p.x / resolution).round().to_subset_unchecked(),
                            (p.z / resolution).round().to_subset_unchecked(),
                        );
                        (pt, p.y)
                    })
                    .collect();

                let mut points_iter = points.iter().copied();
                let Some((first_point, first_height)) = points_iter.next() else {
                    return None;
                };
                let mut min_x = first_point.x;
                let mut min_y = first_point.y;
                let mut min_height = first_height;
                let mut max_x = first_point.x;
                let mut max_y = first_point.y;
                let mut max_height = first_height;

                for (point, height) in points_iter {
                    if point.x < min_x {
                        min_x = point.x;
                    } else if point.x > max_x {
                        max_x = point.x;
                    }
                    if point.y < min_y {
                        min_y = point.y;
                    } else if point.y > max_y {
                        max_y = point.y;
                    }
                    if height < min_height {
                        min_height = height;
                    } else if height > max_height {
                        max_height = height;
                    }
                }

                let max_range = (max_x - min_x).max(max_y - min_y) as usize + 1;

                let depth = if max_range == 0 {
                    1usize
                } else {
                    nalgebra::convert::<_, N>(max_range)
                        .log2()
                        .ceil()
                        .to_subset_unchecked()
                };
                let mut max_density = 0;

                let mut quadtree = Quadtree::<usize, HeightCell<N>>::new(depth);

                for (point, height) in points.iter().copied() {
                    let mut modified_count = AtomicUsize::default();

                    let anchor = quadtree_rs::point::Point {
                        x: (point.x - min_x) as usize,
                        y: (point.y - min_y) as usize,
                    };
                    quadtree.modify(
                        AreaBuilder::default()
                            .anchor(anchor)
                            .dimensions((1, 1))
                            .build()
                            .unwrap(),
                        |pt| {
                            pt.count += 1;
                            modified_count.store(pt.count, Ordering::Relaxed);
                            pt.total_height += height;
                        },
                    );

                    if *modified_count.get_mut() == 0 {
                        let inserted = quadtree
                            .insert_pt(
                                anchor,
                                HeightCell {
                                    total_height: height,
                                    count: 1,
                                },
                            )
                            .is_some();
                        assert!(inserted, "{anchor:?} {depth} {max_range}");
                        max_density = max_density.max(1);
                    } else {
                        max_density = max_density.max(*modified_count.get_mut());
                    }
                }

                Some(CostmapFrame {
                    quadtree,
                    max_density,
                    min_x,
                    min_y,
                    max_height,
                    min_height,
                    resolution,
                    isometry: nalgebra::convert(
                        original_points.robot_element.get_isometry_of_base(),
                    ),
                })
            })
    }

    pub fn get_costmap_pub(&self) -> PublisherRef<Costmap<N>> {
        self.costmap_pub.get_ref()
    }
}

impl<N: RealField + Copy> AsyncNode for CostmapGenerator<N> {
    type Result = ();

    async fn run(mut self, context: RuntimeContext) -> Self::Result {
        setup_logging!(context);
        self.dont_drop.ignore_drop = true;
        let mut costmap_frames: Box<[Arc<CostmapFrame<N>>]> = std::iter::repeat_with(|| {
            Arc::new(CostmapFrame {
                quadtree: Quadtree::new(0),
                max_density: 0,
                min_x: 0,
                min_y: 0,
                max_height: N::zero(),
                min_height: N::zero(),
                resolution: N::one(),
                isometry: nalgebra::Isometry3::identity(),
            })
            .into()
        })
        .take(self.window_length)
        .collect();
        let mut frame_index = 0usize;

        loop {
            let inner = CostmapInnerInit {
                point_count: costmap_frames.iter().map(|x| x.quadtree.len()).sum(),
                frames: Slice::iter_init(costmap_frames.len(), costmap_frames.iter().cloned()),
                threshold: self.threshold,
            };
            let inner = Box::emplace(inner);
            self.costmap_pub.set(Costmap {
                inner: inner.into(),
            });

            let Some(frame) = self.quadtree_sub.recv_or_closed().await else {
                break;
            };
            costmap_frames[frame_index] = Arc::new(frame);
            frame_index += 1;
            if frame_index >= self.window_length {
                frame_index = 0;
            }
        }
    }
}
