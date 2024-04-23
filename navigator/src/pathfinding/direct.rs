use nalgebra::{convert as nconvert, Point3, Vector2, Vector3};
use obstacles::{ObstacleHub, Shape};
use unros::{float::Float, runtime::RuntimeContext, setup_logging, tokio};

use crate::pathfinding::{alg::astar, traverse_to};

use super::{alg::AStarModule, PathfindingEngine};

struct DirectPathfinderSafefinder<'a, N: Float> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: &'a mut Shape<N>,
    max_height_diff: N,
}

impl<'a, N: Float> AStarModule<Node<N>, usize> for DirectPathfinderSafefinder<'a, N> {
    async fn successors(&mut self, current: Node<N>, mut out: impl FnMut(Node<N>, usize)) {
        for position in [
            current.position + Vector2::new(0, 1),
            current.position + Vector2::new(1, 0),
            current.position + Vector2::new(0, -1),
            current.position + Vector2::new(-1, 0),
        ] {
            let nextf = Vector3::new(
                nconvert::<_, N>(position.x) * self.resolution,
                current.height,
                nconvert::<_, N>(position.y) * self.resolution,
            );
            self.shape.set_origin(nextf);
            self
                .obstacle_hub
                .get_height_only_within(self.shape, |result| {
                    out(Node {
                        position,
                        height: result.height,
                    }, 1);
                    Some(())
                })
                .await;
        }
    }

    async fn success(&mut self, node: &Node<N>) -> bool {
        let nextf = Vector3::new(
            nconvert::<_, N>(node.position.x) * self.resolution,
            node.height,
            nconvert::<_, N>(node.position.y) * self.resolution,
        );
        self.shape.set_origin(nextf);
        self.obstacle_hub
            .get_height_only_within(self.shape, |result| {
                if result.height.abs() > self.max_height_diff {
                    Some(())
                } else {
                    None
                }
            })
            .await
            .is_none()
    }
}

struct DirectPathfinderModule<'a, N: Float> {
    obstacle_hub: &'a ObstacleHub<N>,
    resolution: N,
    shape: &'a Shape<N>,
    max_height_diff: N,
    end_node: Node<N>,
}

impl<'a, N: Float> AStarModule<Node<N>, usize> for DirectPathfinderModule<'a, N> {
    async fn successors(&mut self, current: Node<N>, mut out: impl FnMut(Node<N>, usize)) {
        macro_rules! successor_func {
            ($next: expr) => {
                async {
                    let nextf = Vector3::new(
                        nconvert::<_, N>($next.x) * self.resolution,
                        current.height,
                        nconvert::<_, N>($next.y) * self.resolution,
                    );
                    let mut shape = self.shape.clone();
                    shape.set_origin(nextf);
                    self.obstacle_hub
                        .get_height_only_within(&shape, |result| {
                            if result.height.abs() > self.max_height_diff {
                                Some(Node {
                                    position: $next,
                                    height: result.height,
                                })
                            } else {
                                None
                            }
                        })
                        .await
                }
            };
        }
        let (a, b, c, d, e) = tokio::join!(
            successor_func!(current.position + Vector2::new(0, 1)),
            successor_func!(current.position + Vector2::new(1, 0)),
            successor_func!(current.position + Vector2::new(0, -1)),
            successor_func!(current.position + Vector2::new(-1, 0)),
            async {
                let end_diff = self.end_node.position - current.position;
                let end_diff: Vector2<N> = nconvert(end_diff);
                if end_diff.magnitude() < N::one() {
                    successor_func!(self.end_node.position).await
                } else {
                    None
                }
            }
        );
        [a, b, c, d, e]
            .into_iter()
            .filter_map(|x| x)
            .for_each(|x| out(x, 1));
    }

    async fn success(&mut self, node: &Node<N>) -> bool {
        node == &self.end_node
    }
}

#[derive(Default)]
pub struct DirectPathfinder;

impl<N: Float> PathfindingEngine<N> for DirectPathfinder {
    async fn pathfind(
        &mut self,
        start: Point3<N>,
        end: Point3<N>,
        obstacle_hub: &ObstacleHub<N>,
        resolution: N,
        mut shape: Shape<N>,
        max_height_diff: N,
        context: &RuntimeContext,
    ) -> Option<Vec<Point3<N>>> {
        setup_logging!(context);
        let mut start = start.coords;
        let mut pre_path = vec![];
        shape.set_origin(start);
        let start_node = Node {
            position: Vector2::<isize>::new(0, 0),
            height: start.y,
        };

        if obstacle_hub
            .get_height_only_within(&shape, |result| {
                if result.height.abs() > max_height_diff {
                    Some(())
                } else {
                    None
                }
            })
            .await
            .is_some()
        {
            if let Some((mut path, _)) = astar(
                &start_node,
                &mut DirectPathfinderSafefinder {
                    obstacle_hub,
                    resolution,
                    shape: &mut shape,
                    max_height_diff,
                },
                |_| 0,
            )
            .await
            {
                let start_node = path.pop().unwrap();
                start = Vector3::new(
                    nconvert::<_, N>(start_node.position.x) * resolution,
                    start_node.height,
                    nconvert::<_, N>(start_node.position.y) * resolution,
                );
                pre_path = path;
            }
        }
        shape.set_origin(start);
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
            shape: &shape,
            max_height_diff,
            end_node,
        };

        let result = astar(&start_node, &mut successors, |current| {
            let diff = current.position - end_node.position;
            let diff: Vector2<N> = nconvert(diff);
            diff.magnitude().round().to_usize()
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
            if traverse_to(
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
}

#[derive(Clone, Copy)]
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
