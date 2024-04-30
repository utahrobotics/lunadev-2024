use std::ops::Deref;

use nalgebra::{convert as nconvert, Isometry3, Point3, UnitQuaternion, Vector2, Vector3};
use obstacles::{utils::RecycledVec, HeightQuery, ObstacleHub, Shape};
use unros::{float::Float, runtime::RuntimeContext, setup_logging};

use crate::pathfinding::alg::astar;

use super::{alg::AStarModule, PathfindingEngine};

struct DirectPathfinderSafefinder<'a, N: Float> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: Shape<N>,
    max_height_diff: N,
    max_frac: N,
}

impl<'a, N: Float> AStarModule<Node<N>, usize> for DirectPathfinderSafefinder<'a, N>
where
    RecycledVec<HeightQuery<N>>: Default,
{
    async fn successors(&mut self, current: Node<N>, mut out: impl FnMut(Node<N>, usize)) {
        let successors = [
            current.position + Vector2::new(0, 1),
            current.position + Vector2::new(1, 0),
            current.position + Vector2::new(0, -1),
            current.position + Vector2::new(-1, 0),
        ];

        let queries = successors.into_iter().map(|next| {
            let next = Vector3::new(
                nconvert::<_, N>(next.x) * self.resolution,
                current.height,
                nconvert::<_, N>(next.y) * self.resolution,
            );
            HeightQuery {
                max_points: 32,
                shape: self.shape,
                isometry: Isometry3::from_parts(next.into(), UnitQuaternion::default()),
            }
        });

        self.obstacle_hub
            .query_height(queries, |heights, _| {
                heights.iter().zip(successors).for_each(|(heights, successor)| {
                    if heights.is_empty() {
                        return;
                    }
                    out(
                        Node {
                            position: successor,
                            height: heights.iter().copied().sum::<N>() / nconvert(heights.len()),
                        },
                        1,
                    );
                });
                std::future::ready(Some(()))
            })
            .await;
    }

    async fn success(&mut self, current: &Node<N>) -> bool {
        let next = Vector3::new(
            nconvert::<_, N>(current.position.x) * self.resolution,
            current.height,
            nconvert::<_, N>(current.position.y) * self.resolution,
        );
        self.obstacle_hub
            .query_height(
                std::iter::once(HeightQuery {
                    max_points: 32,
                    shape: self.shape,
                    isometry: Isometry3::from_parts(next.into(), UnitQuaternion::default()),
                }),
                |heights, _| {
                    let heights = &heights[0];
                    if heights.is_empty() {
                        return std::future::ready(None);
                    }
                    let mut height = N::zero();
                    let mut too_high_count = 0usize;

                    for &h in heights.iter() {
                        height += h;
                        if (h - current.height).abs() > self.max_height_diff {
                            too_high_count += 1;
                        }
                    }
                    height /= nconvert(heights.len());
                    if too_high_count
                        >= (nconvert::<_, N>(heights.len()) * self.max_frac)
                            .round()
                            .to_usize()
                    {
                        std::future::ready(Some(()))
                    } else {
                        std::future::ready(None)
                    }
                },
            )
            .await
            .is_none()
    }
}

struct DirectPathfinderModule<'a, N: Float> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: Shape<N>,
    max_height_diff: N,
    end_node: Node<N>,
    max_frac: N,
}

impl<'a, N: Float> AStarModule<Node<N>, usize> for DirectPathfinderModule<'a, N>
where
    RecycledVec<HeightQuery<N>>: Default,
{
    async fn successors(&mut self, current: Node<N>, mut out: impl FnMut(Node<N>, usize)) {
        let successors = [
            current.position + Vector2::new(0, 1),
            current.position + Vector2::new(1, 0),
            current.position + Vector2::new(0, -1),
            current.position + Vector2::new(-1, 0),
        ];
        let end_pos = {
            let end_diff = self.end_node.position - current.position;
            if (end_diff.x == 0 && end_diff.y.abs() <= 1)
                || (end_diff.y == 0 && end_diff.x.abs() <= 1)
            {
                Some(self.end_node.position)
            } else {
                None
            }
        };
        let successors: Box<[_]> = successors.into_iter().chain(end_pos).collect();

        let queries = successors.iter().copied().map(|next| {
            let next = Vector3::new(
                nconvert::<_, N>(next.x) * self.resolution,
                current.height,
                nconvert::<_, N>(next.y) * self.resolution,
            );
            HeightQuery {
                max_points: 32,
                shape: self.shape,
                isometry: Isometry3::from_parts(next.into(), UnitQuaternion::default()),
            }
        });

        self.obstacle_hub
            .query_height(queries, |heights, _| {
                heights.iter().zip(successors.deref()).for_each(|(heights, successor)| {
                    if heights.is_empty() {
                        return;
                    }
                    let mut height = N::zero();
                    let mut too_high_count = 0usize;

                    for &h in heights.iter() {
                        height += h;
                        if (h - current.height).abs() > self.max_height_diff {
                            too_high_count += 1;
                        }
                    }
                    height /= nconvert(heights.len());
                    if too_high_count >= (nconvert::<_, N>(heights.len()) * self.max_frac).round().to_usize() {
                        return;
                    }
                    out(
                        Node {
                            position: *successor,
                            height,
                        },
                        1,
                    );
                });
                std::future::ready(Some(()))
            })
            .await;
    }

    async fn success(&mut self, node: &Node<N>) -> bool {
        node == &self.end_node
    }
}

#[derive(Clone, Copy)]
pub struct DirectPathfinder<N: Float> {
    pub max_frac: N,
}

impl<N: Float> Default for DirectPathfinder<N> {
    fn default() -> Self {
        Self {
            max_frac: nconvert(0.333),
        }
    }
}

impl<N: Float> PathfindingEngine<N> for DirectPathfinder<N>
where
    RecycledVec<HeightQuery<N>>: Default,
{
    async fn pathfind(
        &mut self,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        shape: Shape<N>,
        max_height_diff: N,
        context: &RuntimeContext,
    ) -> Option<Vec<Point3<N>>> {
        setup_logging!(context);
        let mut pre_path = vec![];
        let mut start_node = Node {
            position: Vector2::<isize>::new(0, 0),
            height: N::zero(),
        };

        if let Some((mut path, _)) = astar(
            &start_node,
            &mut DirectPathfinderSafefinder {
                obstacle_hub,
                resolution,
                shape,
                max_height_diff,
                max_frac: self.max_frac,
            },
            |_| 0,
        )
        .await
        {
            start_node = path.pop().unwrap();
            pre_path = path;
        }
        let end_node = Node {
            position: Vector2::new(
                (end.x / resolution).round().to_isize(),
                (end.z / resolution).round().to_isize(),
            ),
            height: end.y,
        };

        let mut successors = DirectPathfinderModule {
            obstacle_hub,
            resolution,
            shape,
            max_height_diff,
            end_node,
            max_frac: self.max_frac,
        };

        let result = astar(&start_node, &mut successors, |current| {
            let diff = current.position - end_node.position;
            let diff: Vector2<N> = nconvert(diff);
            diff.magnitude().to_usize()
        })
        .await;

        let (mut post_path, _distance) = result?;

        if post_path.len() == 1 {
            post_path.push(end_node);
        }

        let mut path = pre_path;
        path.append(&mut post_path);

        let mut new_path: Vec<Point3<N>> = Vec::with_capacity(path.len());
        let mut path = path.into_iter().map(|p| {
            Point3::new(
                nconvert::<_, N>(p.position.x) * resolution,
                p.height,
                nconvert::<_, N>(p.position.y) * resolution,
            )
        });

        let mut start = path.next().unwrap();
        new_path.push(start);
        let mut last = path.next().unwrap();

        for next in path {
            if self
                .traverse_to(
                    start.coords,
                    next.coords,
                    shape.clone(),
                    max_height_diff,
                    &obstacle_hub,
                    resolution,
                )
                .await
            {
                last = next;
            } else {
                new_path.push(last);
                start = last;
                last = next;
            }
        }

        new_path.push(end);

        Some(new_path)
    }

    async fn traverse_to(
        &mut self,
        from: Vector3<N>,
        to: Vector3<N>,
        shape: Shape<N>,
        max_height_diff: N,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
    ) -> bool
    where
        RecycledVec<HeightQuery<N>>: Default,
    {
        let mut travel = to - from;
        let distance = travel.magnitude();
        travel.unscale_mut(distance);

        let count: usize = (distance / resolution).floor().to_subset_unchecked();

        let queries = (1..count).into_iter().map(|i| {
            let intermediate: Vector3<N> = from + travel * nconvert::<_, N>(i);
            HeightQuery {
                max_points: 32,
                shape: shape.clone(),
                isometry: Isometry3::from_parts(intermediate.into(), UnitQuaternion::default()),
            }
        });

        obstacle_hub
            .query_height(queries, |heights, _| {
                let heights = &heights[0];
                if heights.is_empty() {
                    return std::future::ready(None);
                }
                let mut height = N::zero();
                let mut count = 0usize;
                let mut too_high_count = 0usize;

                for &h in heights.iter() {
                    height += h;
                    count += 1;
                    if h > max_height_diff {
                        too_high_count += 1;
                    }
                }
                height /= nconvert(count);
                if too_high_count >= (nconvert::<_, N>(count) * self.max_frac).round().to_usize() {
                    std::future::ready(Some(()))
                } else {
                    std::future::ready(None)
                }
            })
            .await
            .is_none()
    }
}

#[derive(Clone, Copy, Debug)]
struct Node<N: Float> {
    position: Vector2<isize>,
    height: N,
}

impl<N: Float> PartialEq for Node<N> {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

impl<N: Float> std::hash::Hash for Node<N> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.position.hash(state);
    }
}

impl<N: Float> Eq for Node<N> {}
