use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc, Mutex, MutexGuard,
};

use nalgebra::{Dyn, Matrix, Point2, Point3, VecStorage};
use ordered_float::NotNan;
use quadtree_rs::{area::AreaBuilder, point::Point, Quadtree};
use unros::{
    pubsub::{Subscriber, Subscription},
    tokio,
};

use crate::Points;

#[derive(Clone, Copy)]
struct HeightCell {
    total_height: isize,
    count: usize,
}

struct HeightMap {
    map: Quadtree<usize, HeightCell>,
    max_count: usize,
}

#[derive(Clone)]
pub struct LocalCostmap {
    pub min_frequency: f32,
    cell_width: f32,
    height_step: f32,
    heightmap: Arc<Mutex<HeightMap>>,
    area_width: usize,
}

impl LocalCostmap {
    pub fn new(area_width: usize, cell_width: f32, height_step: f32) -> Self {
        Self {
            min_frequency: 0.33,
            cell_width,
            height_step,
            heightmap: Arc::new(Mutex::new(HeightMap {
                map: Quadtree::new((area_width as f32).log2().ceil() as usize),
                max_count: 0,
            })),
            area_width,
        }
    }

    pub fn global_to_local(&self, global: Point2<f32>) -> Point2<isize> {
        let offset = (self.area_width as f32) / 2.0;

        Point2::new(
            (global.x / self.cell_width + offset).round() as isize,
            (global.y / self.cell_width + offset).round() as isize,
        )
    }

    pub fn local_to_global(&self, local: Point2<isize>) -> Point2<f32> {
        let offset = (self.area_width as f32) / 2.0;

        Point2::new(
            (local.x as f32 - offset) * self.cell_width,
            (local.y as f32 - offset) * self.cell_width,
        )
    }

    pub fn create_points_sub<T: Send + IntoIterator<Item = Point3<f32>> + Clone + 'static>(
        &self,
    ) -> Subscription<Points<T>> {
        let cell_width = self.cell_width;
        let area_width = self.area_width;
        let height_step = self.height_step;
        let mut sub = Subscriber::<Points<T>>::new(1);
        let subscription = sub.create_subscription();
        let heightmap = self.heightmap.clone();

        tokio::spawn(async move {
            loop {
                let Some(Points {
                    points,
                    robot_element,
                }) = sub.recv_or_closed().await
                else {
                    break;
                };
                let isometry = robot_element.get_isometry_from_base();
                let mut heightmap = heightmap.lock().unwrap();
                heightmap.map.reset();
                let Some(max_count) = points
                    .into_iter()
                    .filter_map(|mut point| {
                        point = isometry * point;
                        let height = (point.y / height_step).round() as isize;

                        let offset = (area_width as f32) / 2.0;
                        point /= cell_width;
                        point.x += offset;
                        point.z += offset;

                        let x = point.x.round() as isize;
                        let y = point.z.round() as isize;

                        if x < 0 || y < 0 {
                            return None;
                        }

                        let x = x as usize;
                        let y = y as usize;

                        if x >= area_width || y >= area_width {
                            return None;
                        }

                        let mut modified_count = AtomicUsize::default();

                        let anchor = Point { x, y };
                        heightmap.map.modify(
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
                            heightmap.map.insert_pt(
                                anchor,
                                HeightCell {
                                    total_height: height,
                                    count: 1,
                                },
                            );
                            Some(1usize)
                        } else {
                            Some(*modified_count.get_mut())
                        }
                    })
                    .max()
                else {
                    continue;
                };

                heightmap.max_count = max_count;
            }
        });

        subscription
    }

    pub fn lock(&self) -> LocalCostmapGuard {
        LocalCostmapGuard {
            costmap_ref: self,
            guard: self.heightmap.lock().unwrap(),
        }
    }

    pub fn get_area_width(&self) -> usize {
        self.area_width
    }

    pub fn get_cell_width(&self) -> f32 {
        self.cell_width
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
            image::GrayImage::from_vec(self.area_width as u32, self.area_width as u32, buf)
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

        image::GrayImage::from_vec(self.area_width as u32, self.area_width as u32, buf).unwrap()
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
}

pub struct LocalCostmapGuard<'a> {
    costmap_ref: &'a LocalCostmap,
    guard: MutexGuard<'a, HeightMap>,
}

impl<'a> LocalCostmapGuard<'a> {
    pub fn get_costmap(&self) -> Matrix<f32, Dyn, Dyn, VecStorage<f32, Dyn, Dyn>> {
        let mut data = Matrix::from_data(VecStorage::new(
            Dyn(self.costmap_ref.area_width),
            Dyn(self.costmap_ref.area_width),
            vec![0.0; self.costmap_ref.area_width.pow(2)],
        ));

        for entry in self.guard.map.iter() {
            let Point { x, y } = entry.anchor();
            let cell = entry.value_ref();
            let mutref = data.get_mut((y, x)).unwrap();
            *mutref = cell.total_height as f32 / cell.count as f32 * self.costmap_ref.height_step;
        }

        data
    }

    pub fn is_cell_safe(&self, radius: f32, point: Point2<usize>, max_diff: f32) -> bool {
        let radius = (radius / self.costmap_ref.cell_width).round() as usize;

        let cells = self.guard.map.query(
            AreaBuilder::default()
                .anchor(Point {
                    x: point.x - radius,
                    y: point.y - radius,
                })
                .dimensions((radius * 2 + 1, radius * 2 + 1))
                .build()
                .unwrap(),
        );

        let threshold =
            (self.guard.max_count as f32 * self.costmap_ref.min_frequency).round() as usize;

        for cell in cells {
            let cell = cell.value_ref();
            if cell.count < threshold {
                continue;
            }
            let height =
                cell.total_height as f32 / cell.count as f32 * self.costmap_ref.height_step;
            if height.abs() > max_diff {
                return false;
            }
        }

        true
    }
}
