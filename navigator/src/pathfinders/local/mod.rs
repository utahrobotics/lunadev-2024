use std::{
    sync::{
        atomic::{AtomicU8, Ordering},
        Arc,
    },
    time::Instant,
};

use costmap::local::{LocalCostmap, LocalCostmapGuard};
use nalgebra::{Point2, Vector2};
use ordered_float::NotNan;
use unros::service::Pending;

use crate::{pathfinders::NavigationError, Float};

use self::pathfinding::astar;

use super::{CostmapReference, DirectPathfinder};

mod pathfinding;

impl CostmapReference for Arc<LocalCostmap> {
    fn process(
        node: &mut DirectPathfinder<Self>,
        dest: nalgebra::Point3<Float>,
        pending_task: Pending<Result<(), NavigationError>>,
        completion_percentage: Arc<AtomicU8>,
    ) {
        let sleeper = spin_sleep::SpinSleeper::default();
        let dest = Point2::new(dest.x, dest.z);

        let mut start_time = Instant::now();

        loop {
            let real_pos = node.robot_base.get_isometry().translation.vector;
            if (dest.coords.cast() - Vector2::new(real_pos.x, real_pos.z)).magnitude() <= node.completion_distance {
                break;
            }

            let mut dest_grid = node.costmap_ref.global_to_local(dest);
            if dest_grid.x < 0 {
                dest_grid.x = 0;
            }
            if dest_grid.y < 0 {
                dest_grid.y = 0;
            }
            let mut dest_grid = Vector2::new(dest_grid.x as usize, dest_grid.y as usize);
            if dest_grid.x >= node.costmap_ref.get_area_width() {
                dest_grid.x = node.costmap_ref.get_area_width() - 1;
            }
            if dest_grid.y >= node.costmap_ref.get_area_width() {
                dest_grid.y = node.costmap_ref.get_area_width() - 1;
            }

            let guard = node.costmap_ref.lock();

            let (path, _distance) = astar(
                // In local space, our start position is always (0, 0)
                &Vector2::<usize>::new(node.costmap_ref.get_area_width() / 2, node.costmap_ref.get_area_width() / 2),
                |current| {
                    let current = current.cast::<isize>();
                    const ROOT_2: Float = std::f64::consts::SQRT_2 as Float;
                    [
                        (Vector2::new(-1, -1) + current, ROOT_2),
                        (Vector2::new(-1, 0) + current, 1.0),
                        (Vector2::new(-1, 1) + current, ROOT_2),
                        (Vector2::new(0, -1) + current, 1.0),
                        (Vector2::new(0, 1) + current, 1.0),
                        (Vector2::new(1, -1) + current, ROOT_2),
                        (Vector2::new(1, 0) + current, 1.0),
                        (Vector2::new(1, 1) + current, ROOT_2),
                    ]
                    .into_iter()
                    .filter_map(|(next, cost)| {
                        if next.x < 0 || next.y < 0 {
                            None
                        } else {
                            let next = Vector2::new(next.x as usize, next.y as usize);

                            if next.x >= node.costmap_ref.get_area_width()
                                || next.y >= node.costmap_ref.get_area_width()
                            {
                                None
                            } else if guard.is_cell_safe(
                                node.agent_radius,
                                next.into(),
                                node.max_height_diff,
                            ) {
                                Some((next, NotNan::new(cost).unwrap()))
                            } else {
                                None
                            }
                        }
                    })
                },
                |current| {
                    NotNan::new((current.cast::<Float>() - dest_grid.cast()).magnitude()).unwrap()
                },
                |current| *current == dest_grid,
            );

            let mut new_path = Vec::with_capacity(path.len());
            let mut path = path.into_iter();

            let mut start = path.next().unwrap();
            new_path.push(start.cast::<isize>());
            let mut last = path.next().unwrap();

            for next in path {
                if traverse_to(start, next, node.agent_radius, node.max_height_diff, &guard) {
                    last = next;
                } else {
                    new_path.push(last.cast::<isize>());
                    start = last;
                    last = next;
                }
            }
            drop(guard);

            new_path.push(last.cast::<isize>());

            let mut path: Vec<_> = new_path
                .into_iter()
                .map(|point| {
                    node.costmap_ref
                        .local_to_global(point.into())
                        .cast::<Float>()
                })
                .collect();

            let isometry = node.robot_base.get_isometry();
            *path.first_mut().unwrap() =
                Point2::new(isometry.translation.x, isometry.translation.z);
            *path.last_mut().unwrap() = dest;

            node.path_signal.set(path);

            let elapsed = start_time.elapsed();
            sleeper.sleep(node.refresh_rate.saturating_sub(elapsed));
            start_time += elapsed;
        }

        completion_percentage.store(255, Ordering::Relaxed);
        pending_task.finish(Ok(()));
    }
}

#[inline]
fn traverse_to(
    from: Vector2<usize>,
    to: Vector2<usize>,
    agent_radius: Float,
    max_height_diff: Float,
    costmap: &LocalCostmapGuard,
) -> bool {
    let mut travel = to.cast::<Float>() - from.cast();
    let distance = travel.magnitude();
    travel.unscale_mut(distance);

    for i in 0..distance.floor() as usize {
        let intermediate = from.cast() + travel * i as Float;

        if !costmap.is_cell_safe(
            agent_radius,
            Point2::new(
                intermediate.x.round() as usize,
                intermediate.y.round() as usize,
            ),
            max_height_diff,
        ) {
            return false;
        }
    }

    true
}
