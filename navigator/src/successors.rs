use nalgebra::{DMatrix, Rotation2, UnitVector2, Vector2};
use ordered_float::NotNan;

use crate::Float;

const SUCCESSORS: [Vector2<isize>; 8] = [
    Vector2::new(-1, -1),
    Vector2::new(-1, 0),
    Vector2::new(-1, 1),
    Vector2::new(0, -1),
    Vector2::new(0, 1),
    Vector2::new(1, -1),
    Vector2::new(1, 0),
    Vector2::new(1, 1),
];
const PI: Float = std::f64::consts::PI as Float;

#[derive(Clone, Copy, Debug)]
pub(super) struct RobotState<T=usize> {
    pub(super) position: Vector2<T>,
    pub(super) forward: UnitVector2<Float>,
    pub(super) left_steering: NotNan<Float>,
    pub(super) right_steering: NotNan<Float>,
}

impl PartialEq for RobotState {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

impl Eq for RobotState {}

impl std::hash::Hash for RobotState {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.position.hash(state);
    }
}

pub(super) fn successors<'a>(
    current: RobotState,
    obstacles: &'a DMatrix<bool>,
    can_reverse: bool,
    width: Float
) -> impl IntoIterator<Item = (RobotState, NotNan<Float>)> + 'a {
    SUCCESSORS.into_iter().filter_map(move |offset| {
        let cell = offset + current.position.cast();
        if cell.x < 0 || cell.y < 0 {
            return None;
        }
        let cell = Vector2::new(cell.x as usize, cell.y as usize);
        if !*obstacles.get((cell.y as usize, cell.x as usize))? {
            return None;
        }
        let mut offset: Vector2<Float> = offset.cast();
        let distance = offset.magnitude();
        offset.unscale_mut(distance);
        let mut cross = (current.forward.x * offset.y - current.forward.y * offset.x).signum();
        let mut angle = offset.angle(&current.forward);

        let mut left_steering;
        let mut right_steering;
        if angle > PI / 2.0 && can_reverse {
            left_steering = - distance;
            right_steering = - distance;
            cross *= -1.0;
            angle = PI - angle;
        } else {
            left_steering = distance;
            right_steering = distance;
        }

        let arc = angle * width / 2.0;
        if cross > 0.0 {
            right_steering += arc;
            left_steering -= arc;
        } else {
            right_steering -= arc;
            left_steering += arc;
        }

        let cost = (left_steering.abs() + right_steering.abs()) / 2.0;

        if left_steering.abs() > right_steering.abs() {
            right_steering /= left_steering.abs();
            left_steering = left_steering.signum();
        } else {
            left_steering /= right_steering.abs();
            right_steering = right_steering.signum();
        }

        Some((
            RobotState {
                position: cell,
                forward: Rotation2::new(angle) * current.forward,
                left_steering: NotNan::new(left_steering).unwrap(),
                right_steering: NotNan::new(right_steering).unwrap()
            },
            NotNan::new(cost).unwrap(),
        ))
    })
}
