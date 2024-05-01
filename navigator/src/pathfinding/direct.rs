use nalgebra::{convert as nconvert, Isometry3, Point2, Point3, UnitQuaternion, Vector2, Vector3};
use obstacles::{utils::RecycledVec, HeightQuery, ObstacleHub, Shape};
use unros::{float::Float, runtime::RuntimeContext, setup_logging};

use crate::pathfinding::alg::astar;

use super::{alg::AStarModule, PathfindingEngine};

struct DirectPathfinderSafefinder<'a, N: Float, F> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: Shape<N>,
    max_height_diff: N,
    max_frac: N,
    filter: &'a F,
}

impl<'a, N, F> AStarModule<Node<N>, usize> for DirectPathfinderSafefinder<'a, N, F>
where
    RecycledVec<HeightQuery<N>>: Default,
    N: Float,
    F: Fn(Point2<isize>) -> bool,
{
    async fn successors(&mut self, current: Node<N>, mut out: impl FnMut(Node<N>, usize)) {
        let successors = [
            current.position + Vector2::new(0, 1),
            current.position + Vector2::new(1, 0),
            current.position + Vector2::new(0, -1),
            current.position + Vector2::new(-1, 0),
        ];

        let queries = successors
            .into_iter()
            .filter(|p| (self.filter)((*p).into()))
            .map(|next| {
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

        let mut pending = self.obstacle_hub.query_height(queries).await;

        while let Some(mut vec_of_heights) = pending.next().await {
            vec_of_heights
                .drain(..)
                .zip(successors)
                .for_each(|(mut heights, successor)| {
                    let mut height = N::zero();
                    let mut count = 0usize;
                    for h in heights.drain(..) {
                        height += h;
                        count += 1;
                    }
                    if count == 0 {
                        out(
                            Node {
                                position: successor,
                                height: current.height,
                            },
                            1,
                        );
                    } else {
                        out(
                            Node {
                                position: successor,
                                height: height / nconvert(count),
                            },
                            1,
                        );
                    }
                });
        }
    }

    async fn success(&mut self, current: &Node<N>) -> bool {
        let next = Vector3::new(
            nconvert::<_, N>(current.position.x) * self.resolution,
            current.height,
            nconvert::<_, N>(current.position.y) * self.resolution,
        );
        self.obstacle_hub
            .safe_by_height(
                std::iter::once(HeightQuery {
                    max_points: 32,
                    shape: self.shape,
                    isometry: Isometry3::from_parts(next.into(), UnitQuaternion::default()),
                }),
                current.height,
                self.max_height_diff,
                self.max_frac,
            )
            .await[0]
    }
}

struct DirectPathfinderModule<'a, N: Float, F> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: Shape<N>,
    max_height_diff: N,
    end_node: Node<N>,
    max_frac: N,
    filter: &'a F,
}

impl<'a, N, F> AStarModule<Node<N>, usize> for DirectPathfinderModule<'a, N, F>
where
    RecycledVec<HeightQuery<N>>: Default,
    N: Float,
    F: Fn(Point2<isize>) -> bool,
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
        let successors = successors
            .into_iter()
            .chain(end_pos)
            .filter(|p| (self.filter)((*p).into()));

        let queries = successors.clone().map(|next| {
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

        let mut pending = self.obstacle_hub.query_height(queries).await;

        while let Some((mut vec_of_heights, queries)) = pending.next_with_queries().await {
            vec_of_heights
                .drain(..)
                .zip(queries.iter())
                .zip(successors.clone())
                .for_each(|((mut heights, query), successor)| {
                    let mut height = N::zero();
                    let mut count = 0usize;
                    let mut too_high_count = 0usize;

                    for h in heights.drain(..) {
                        height += h;
                        count += 1;
                        if (h - current.height).abs() > self.max_height_diff {
                            too_high_count += 1;
                        }
                    }

                    if count == 0 {
                        out(
                            Node {
                                position: successor,
                                height: current.height,
                            },
                            1,
                        );
                    } else if nconvert::<_, N>(too_high_count)
                        <= nconvert::<_, N>(query.max_points) * self.max_frac
                    {
                        out(
                            Node {
                                position: successor,
                                height: height / nconvert(count),
                            },
                            1,
                        );
                    }
                });
        }
    }

    async fn success(&mut self, node: &Node<N>) -> bool {
        node == &self.end_node
    }
}

#[derive(Clone, Copy)]
pub struct DirectPathfinder<N: Float, F> {
    pub max_frac: N,
    pub pathfind_shape: Shape<N>,
    pub unsafe_shape: Shape<N>,
    pub max_height_diff: N,
    pub filter: F,
}

impl<N, F> DirectPathfinder<N, F>
where
    RecycledVec<HeightQuery<N>>: Default,
    N: Float,
    F: Fn(Point2<isize>) -> bool + Send + Sync + 'static,
{
    async fn traverse_to(
        &mut self,
        isometry: Isometry3<N>,
        from: Point3<N>,
        to: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
    ) -> bool
    where
        RecycledVec<HeightQuery<N>>: Default,
    {
        let from = isometry.inverse_transform_point(&from).coords;
        let to = isometry.inverse_transform_point(&to).coords;
        let mut travel = to - from;
        let distance = travel.magnitude();
        travel.unscale_mut(distance);

        let count: usize = (distance / resolution).floor().to_subset_unchecked();

        let queries = (1..count).into_iter().map(|i| {
            let intermediate: Vector3<N> = from + travel * nconvert::<_, N>(i);
            HeightQuery {
                max_points: 32,
                shape: self.pathfind_shape,
                isometry: Isometry3::from_parts(intermediate.into(), UnitQuaternion::default()),
            }
        });

        obstacle_hub
            .safe_by_height(queries, from.y, self.max_height_diff, self.max_frac)
            .await
            .into_iter()
            .all(|safe| safe)
    }
}

impl<N, F> PathfindingEngine<N> for DirectPathfinder<N, F>
where
    RecycledVec<HeightQuery<N>>: Default,
    N: Float,
    F: Fn(Point2<isize>) -> bool + Send + Sync + 'static,
{
    async fn pathfind(
        &mut self,
        from: Isometry3<N>,
        mut end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        context: &RuntimeContext,
    ) -> Option<Vec<Point3<N>>> {
        setup_logging!(context);
        end = from.inverse_transform_point(&end);
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
                shape: self.pathfind_shape,
                max_height_diff: self.max_height_diff,
                max_frac: self.max_frac,
                filter: &self.filter,
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
            shape: self.pathfind_shape,
            max_height_diff: self.max_height_diff,
            end_node,
            max_frac: self.max_frac,
            filter: &self.filter,
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
                .traverse_to(Isometry3::default(), start, next, &obstacle_hub, resolution)
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
        new_path
            .iter_mut()
            .for_each(|p| *p = from.transform_point(p));

        Some(new_path)
    }

    async fn is_currently_unsafe(
        &mut self,
        isometry: Isometry3<N>,
        obstacle_hub: &ObstacleHub<N>,
    ) -> bool {
        !obstacle_hub
            .safe_by_height(
                std::iter::once(HeightQuery {
                    max_points: 32,
                    shape: self.unsafe_shape,
                    isometry: Isometry3::from_parts(
                        isometry.translation,
                        UnitQuaternion::default(),
                    ),
                }),
                isometry.translation.y,
                self.max_height_diff,
                self.max_frac,
            )
            .await[0]
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
