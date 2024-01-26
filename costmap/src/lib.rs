use std::{
    sync::{
        atomic::{AtomicIsize, AtomicUsize, Ordering},
        mpsc::channel,
        Arc,
    },
    time::{Duration, Instant},
};

use nalgebra::{Dyn, Matrix, Point3, VecStorage};
use ordered_float::NotNan;
use spin_sleep::SpinSleeper;
use unros_core::{
    anyhow, async_trait,
    pubsub::{Subscriber, Subscription},
    rayon::{
        self,
        iter::{
            IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator, ParallelIterator
        },
    },
    setup_logging, Node, RuntimeContext,
};

struct PointMeasurement {
    index: usize,
    height: isize,
}

pub struct Costmap {
    pub window_duration: Duration,
    area_width: usize,
    area_length: usize,
    cell_width: f32,
    height_step: f32,
    x_offset: f32,
    y_offset: f32,
    points_sub: Subscriber<Arc<[PointMeasurement]>>,
    heights: Arc<[AtomicIsize]>,
    counts: Arc<[AtomicUsize]>,
}

impl Costmap {
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
            area_width,
            area_length,
            height_step,
            cell_width,
            x_offset,
            y_offset,
            points_sub: Subscriber::default(),
            heights: (0..(area_length * area_width))
                .map(|_| Default::default())
                .collect(),
            counts: (0..(area_length * area_width))
                .map(|_| Default::default())
                .collect(),
        }
    }

    pub fn create_points_sub<T: Send + IntoParallelIterator<Item = Point3<f32>> + 'static>(
        &mut self,
    ) -> Subscription<T> {
        let cell_width = self.cell_width;
        let area_width = self.area_width;
        let area_length = self.area_length;
        let x_offset = self.x_offset;
        let y_offset = self.y_offset;
        let height_step = self.height_step;

        self.points_sub.create_subscription(8).map(move |x: T| {
            x.into_par_iter()
                .filter_map(|mut point| {
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
        })
    }

    pub fn get_ref(&self) -> CostmapRef {
        CostmapRef {
            heights: self.heights.clone(),
            counts: self.counts.clone(),
            area_length: self.area_length,
            area_width: self.area_width,
            cell_width: self.cell_width,
        }
    }
}

pub struct CostmapRef {
    area_width: usize,
    area_length: usize,
    heights: Arc<[AtomicIsize]>,
    counts: Arc<[AtomicUsize]>,
    cell_width: f32,
}

impl CostmapRef {
    pub fn get_costmap(&self) -> Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>> {
        let data = self
            .heights
            .par_iter()
            .zip(self.counts.par_iter())
            .map(|(height, count)| {
                let count = count.load(Ordering::Relaxed);
                if count == 0 {
                    0.0
                } else {
                    height.load(Ordering::Relaxed) as f32 / count as f32
                }
            })
            .collect();

        Matrix::from_data(VecStorage::new(
            Dyn(self.area_length),
            Dyn(self.area_width),
            data,
        ))
    }

    pub fn costmap_to_obstacle(
        &self,
        mut matrix: Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>>,
        max_diff: f32,
        current_height: f32,
        agent_radius: f32
    ) -> Matrix<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>> {
        matrix.iter_mut().for_each(|x| {
            *x -= current_height;
        });
        let agent_radius: usize = (agent_radius / self.cell_width as f32).round() as usize;
        let tmp = Matrix::<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>>::from_iterator(matrix.nrows(), matrix.ncols(), matrix.into_iter().map(|x| x.abs() <= max_diff));
        let mut out = Matrix::<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>>::from_iterator(matrix.nrows(), matrix.ncols(), std::iter::repeat(true));

        tmp.row_iter().enumerate().for_each(|(y, row)| {
            row.into_iter()
                .enumerate()
                .for_each(|(x, safe)| if !safe {
                    for n_y in (y.saturating_sub(agent_radius))..(y + agent_radius) {
                        for n_x in (x.saturating_sub(agent_radius))..(x + agent_radius) {
                            let Some(mutref) = out.get_mut((n_x, n_y)) else { continue; };
                            if ((n_x - x).pow(2) as f32 + (n_y - y).pow(2) as f32).sqrt() > agent_radius as f32 {
                                continue;
                            }
                            *mutref = false;
                        }
                    }
                });
        });

        out
    }

    pub fn costmap_to_img(
        &self,
        matrix: Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>>,
    ) -> (image::GrayImage, f32) {
        let max = matrix
            .data
            .as_slice()
            .into_iter()
            .map(|n| NotNan::new(*n).unwrap())
            .max()
            .unwrap()
            .into_inner();
        let min = matrix
            .data
            .as_slice()
            .into_iter()
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
        matrix: Matrix<bool, Dyn, Dyn, VecStorage<bool, Dyn, Dyn>>,
    ) -> image::GrayImage {
        let buf = matrix
            .row_iter()
            .flat_map(|x| {
                x.into_iter()
                    .copied()
                    .map(|x| if x {
                        255
                    } else {
                        0
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        
            image::GrayImage::from_vec(self.area_width as u32, self.area_length as u32, buf)
                .unwrap()
    }
}

#[async_trait]
impl Node for Costmap {
    const DEFAULT_NAME: &'static str = "costmap";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        let (del_sender, del_recv) = channel::<(Duration, Arc<[PointMeasurement]>)>();
        let start = Instant::now();
        let start2 = start.clone();
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
