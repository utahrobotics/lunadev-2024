use std::{sync::{Arc, atomic::{AtomicUsize, Ordering}}, time::Duration};

use nalgebra::{Point3, Matrix, Dyn, VecStorage};
use unros_core::{Node, async_trait, RuntimeContext, anyhow, signal::unbounded::UnboundedSubscription, setup_logging, rayon::{self, iter::{IntoParallelRefIterator, ParallelIterator, IntoParallelIterator}}, tokio};

pub struct Costmap {
    pub window_duration: Duration,
    area_width: usize,
    area_length: usize,
    cell_width: f32,
    x_offset: f32,
    y_offset: f32,
    points_sub: UnboundedSubscription<Arc<[usize]>>,
    points: Arc<[AtomicUsize]>
}


impl Costmap {
    pub fn new(area_width: usize, area_length: usize, cell_width: f32, x_offset: f32, y_offset: f32) -> Self {
        Self {
            window_duration: Duration::from_secs(5),
            area_width,
            area_length,
            cell_width,
            x_offset,
            y_offset,
            points_sub: UnboundedSubscription::none(),
            points: (0..(area_length * area_width)).map(|_| Default::default()).collect(),
        }
    }

    pub fn add_points_sub<T: Send + IntoParallelIterator<Item=Point3<f32>> + 'static>(&mut self, sub: UnboundedSubscription<T>) {
        let cell_width = self.cell_width;
        let area_width = self.area_width;
        let area_length = self.area_length;
        let x_offset = self.x_offset;
        let y_offset = self.y_offset;

        self.points_sub += sub.map(move |x| {
            x.into_par_iter()
                .filter_map(|mut point| {
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

                    Some(x * area_length + y)
                })
                .collect()
        });
    }

    pub fn get_ref(&self) -> CostmapRef {
        CostmapRef { points: self.points.clone(), area_length: self.area_length, area_width: self.area_width }
    }
}


pub struct CostmapRef {
    area_width: usize,
    area_length: usize,
    points: Arc<[AtomicUsize]>
}


impl CostmapRef {
    pub fn get_costmap(&self) -> Matrix<usize, Dyn, Dyn, VecStorage<usize, Dyn, Dyn>> {
        let data = self.points
            .par_iter()
            .map(|x|
                x.load(Ordering::Relaxed)
            )
            .collect();
        Matrix::from_data(VecStorage::new(Dyn(self.area_length), Dyn(self.area_width), data))
    }

    // #[cfg(feature = "image")]
    pub fn get_costmap_img(&self) -> image::GrayImage {
        let costmap = self.get_costmap();

        let max = *costmap.data.as_slice().into_iter().max().unwrap() as f32;

        let buf = costmap.row_iter()
            .flat_map(|x|
                x.column_iter().map(|x| {
                    (x[0] as f32 / max * 255.0) as u8
                }).collect::<Vec<_>>()
            )
            .collect();

        image::GrayImage::from_vec(self.area_width as u32, self.area_length as u32, buf).unwrap()
    }
}


#[async_trait]
impl Node for Costmap {
    const DEFAULT_NAME: &'static str = "points-filter";

    async fn run(mut self, context: RuntimeContext) -> anyhow::Result<()> {
        setup_logging!(context);

        loop {
            let new_points = self.points_sub.recv().await;
            let new_points2 = new_points.clone();
            let points = self.points.clone();
            let points2 = points.clone();

            tokio::spawn(async move {
                tokio::time::sleep(self.window_duration).await;
                rayon::spawn(move || {
                    new_points.par_iter()
                        .for_each(|i| {
                            points[*i].fetch_sub(1, Ordering::Relaxed);
                        });
                });
            });
            rayon::spawn(move || {
                new_points2.par_iter()
                    .for_each(|i| {
                        points2[*i].fetch_add(1, Ordering::Relaxed);
                    });
            });
        }
    }
}