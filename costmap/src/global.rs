use std::{
    cmp::Reverse,
    collections::BinaryHeap,
    sync::{
        atomic::{AtomicIsize, AtomicUsize, Ordering},
        mpsc::channel,
        Arc,
    },
    time::{Duration, Instant},
};

use nalgebra::{Dyn, Matrix, Point2, Point3, VecStorage};
use ordered_float::NotNan;
use spin_sleep::SpinSleeper;
use unros::{
    anyhow, async_trait,
    pubsub::{subs::Subscription, Subscriber},
    rayon::{
        self,
        iter::{
            IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator,
            ParallelIterator,
        },
    },
    setup_logging, Node, NodeIntrinsics, RuntimeContext,
};

use crate::{PointMeasurement, Points};

pub struct GlobalCostmap {
    pub window_duration: Duration,
    pub min_frequency: f32,
    area_width: usize,
    area_length: usize,
    cell_width: f32,
    height_step: f32,
    x_offset: f32,
    y_offset: f32,
    points_sub: Subscriber<Arc<[PointMeasurement]>>,
    heights: Arc<[AtomicIsize]>,
    counts: Arc<[AtomicUsize]>,
    intrinsics: NodeIntrinsics<Self>,
}

impl GlobalCostmap {
    pub fn new(
        area_width: usize,
        area_length: usize,
        cell_width: f32,
        x_offset: f32,
        y_offset: f32,
        height_step: f32,
    ) -> Self {
        Self {
            window_duration: Duration::from_secs(5),
            min_frequency: 0.33,
            area_width,
            area_length,
            height_step,
            cell_width,
            x_offset,
            y_offset,
            points_sub: Subscriber::new(8),
            heights: (0..(area_length * area_width))
                .map(|_| Default::default())
                .collect(),
            counts: (0..(area_length * area_width))
                .map(|_| Default::default())
                .collect(),
            intrinsics: Default::default(),
        }
    }

    pub fn create_points_sub<T: Send + IntoParallelIterator<Item = Point3<f32>> + 'static>(
        &self,
    ) -> impl Subscription<Item = Points<T>> {
        let cell_width = self.cell_width;
        let area_width = self.area_width;
        let area_length = self.area_length;
        let x_offset = self.x_offset;
        let y_offset = self.y_offset;
        let height_step = self.height_step;

        self.points_sub.create_subscription().map(
            move |Points {
                      points,
                      robot_element,
                  }: Points<T>| {
                let isometry = robot_element.get_global_isometry();
                points
                    .into_par_iter()
                    .filter_map(|mut point| {
                        point = isometry * point;
                        let height = (point.y / height_step).round() as isize;

                        point.x += x_offset;
                        point.z += y_offset;
                        point /= cell_width;

                        let x = point.x.round();
                        let y = point.z.round();

                        if x < 0.0 || y < 0.0 {
                            return None;
                        }

                        let x = x as usize;
                        let y = y as usize;

                        if x >= area_width || y >= area_length {
                            return None;
                        }

                        Some(PointMeasurement {
                            index: x * area_length + y,
                            height,
                        })
                    })
                    .collect()
            },
        )
    }

    pub fn get_ref(&self) -> GlobalCostmapRef {
        GlobalCostmapRef {
            heights: self.heights.clone(),
            counts: self.counts.clone(),
            area_length: self.area_length,
            area_width: self.area_width,
            cell_width: self.cell_width,
            x_offset: self.x_offset,
            y_offset: self.y_offset,
            min_frequency: self.min_frequency,
            height_step: self.height_step,
        }
    }
}

#[derive(Clone)]
pub struct GlobalCostmapRef {
    area_width: usize,
    area_length: usize,
    heights: Arc<[AtomicIsize]>,
    counts: Arc<[AtomicUsize]>,
    cell_width: f32,
    x_offset: f32,
    y_offset: f32,
    min_frequency: f32,
    height_step: f32,
}

impl GlobalCostmapRef {
    pub fn get_costmap(&self) -> Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>> {
        let mut counts: BinaryHeap<_> = self
            .counts
            .par_iter()
            .map(|count| Reverse(count.load(Ordering::Relaxed)))
            .collect();

        for _ in 0..((counts.len() as f32 * self.min_frequency).ceil() as usize - 1) {
            counts.pop();
        }

        let min_count = counts.pop().unwrap().0;

        let data = self
            .heights
            .par_iter()
            .zip(self.counts.par_iter())
            .map(|(height, count)| {
                let count = count.load(Ordering::Relaxed);
                if count == 0 || count < min_count {
                    0.0
                } else {
                    height.load(Ordering::Relaxed) as f32 / count as f32 * self.height_step
                }
            })
            .collect();

        Matrix::from_data(VecStorage::new(
            Dyn(self.area_length),
            Dyn(self.area_width),
            data,
        ))
    }

    pub fn get_area_width(&self) -> usize {
        self.area_width
    }

    pub fn get_area_length(&self) -> usize {
        self.area_length
    }

    pub fn get_cell_width(&self) -> f32 {
        self.cell_width
    }

    pub fn get_x_offset(&self) -> f32 {
        self.x_offset
    }

    pub fn get_y_offset(&self) -> f32 {
        self.y_offset
    }

    pub fn global_to_local(&self, global: Point2<f32>) -> Point2<isize> {
        Point2::new(
            ((global.x + self.x_offset) / self.cell_width).round() as isize,
            ((global.y + self.y_offset) / self.cell_width).round() as isize,
        )
    }

    pub fn local_to_global(&self, local: Point2<isize>) -> Point2<f32> {
        Point2::new(
            local.x as f32 * self.cell_width - self.x_offset,
            local.y as f32 * self.cell_width - self.y_offset,
        )
    }

    pub fn costmap_to_obstacle(
        &self,
        matrix: &Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>>,
        max_diff: f32,
        current_height: f32,
        agent_radius: f32,
    ) -> Matrix<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>> {
        let agent_radius: usize = (agent_radius / self.cell_width).round() as usize;
        let tmp = Matrix::<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>>::from_iterator(
            matrix.nrows(),
            matrix.ncols(),
            matrix
                .into_iter()
                .map(|x| (x - current_height).abs() <= max_diff),
        );
        let mut out = tmp.clone();

        tmp.row_iter().enumerate().for_each(|(y, row)| {
            row.into_iter().enumerate().for_each(|(x, safe)| {
                if !safe {
                    for n_y in (y.saturating_sub(agent_radius))..(y + agent_radius) {
                        for n_x in (x.saturating_sub(agent_radius))..(x + agent_radius) {
                            let Some(mutref) = out.get_mut((n_y, n_x)) else {
                                continue;
                            };
                            if ((n_x as isize - x as isize).pow(2) as f32
                                + (n_y as isize - y as isize).pow(2) as f32)
                                .sqrt()
                                <= agent_radius as f32
                            {
                                *mutref = false;
                            }
                        }
                    }
                }
            });
        });

        out
    }

    pub fn costmap_to_img(
        &self,
        matrix: &Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>>,
    ) -> (image::GrayImage, f32) {
        let max = matrix
            .data
            .as_slice()
            .iter()
            .map(|n| NotNan::new(*n).unwrap())
            .max()
            .unwrap()
            .into_inner();
        let min = matrix
            .data
            .as_slice()
            .iter()
            .map(|n| NotNan::new(*n).unwrap())
            .min()
            .unwrap()
            .into_inner();
        let max = min.abs().max(max);

        let buf = matrix
            .row_iter()
            .flat_map(|x| {
                x.into_iter()
                    .map(|x| (x.abs() / max * 255.0) as u8)
                    .collect::<Vec<_>>()
            })
            .collect();

        (
            image::GrayImage::from_vec(self.area_width as u32, self.area_length as u32, buf)
                .unwrap(),
            max,
        )
    }

    pub fn obstacles_to_img(
        &self,
        matrix: &Matrix<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>>,
    ) -> image::GrayImage {
        let buf = matrix
            .row_iter()
            .flat_map(|x| {
                x.into_iter()
                    .copied()
                    .map(|x| if x { 0 } else { 255 })
                    .collect::<Vec<_>>()
            })
            .collect();

        image::GrayImage::from_vec(self.area_width as u32, self.area_length as u32, buf).unwrap()
    }
}

#[async_trait]
impl Node for GlobalCostmap {
    const DEFAULT_NAME: &'static str = "costmap";

    fn get_intrinsics(&mut self) -> &mut NodeIntrinsics<Self> {
        &mut self.intrinsics
    }

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let (del_sender, del_recv) = channel::<(Duration, Arc<[PointMeasurement]>)>();
        let start = Instant::now();
        let start2 = start;
        let heights2 = self.heights.clone();
        let counts2 = self.counts.clone();

        rayon::spawn(move || {
            let sleeper = SpinSleeper::default();
            loop {
                let Ok((next_duration, new_points)) = del_recv.recv() else {
                    break;
                };
                sleeper.sleep(next_duration.saturating_sub(start2.elapsed()));
                new_points.par_iter().for_each(|p| {
                    counts2[p.index].fetch_sub(1, Ordering::Relaxed);
                    heights2[p.index].fetch_sub(p.height, Ordering::Relaxed);
                });
            }
        });

        loop {
            let new_points = self.points_sub.recv().await;
            let new_points2 = new_points.clone();
            let heights = self.heights.clone();
            let counts = self.counts.clone();

            let _ = del_sender.send((start.elapsed() + self.window_duration, new_points2));

            rayon::spawn(move || {
                new_points.par_iter().for_each(|p| {
                    counts[p.index].fetch_add(1, Ordering::Relaxed);
                    heights[p.index].fetch_add(p.height, Ordering::Relaxed);
                });
            });
        }
    }
}
