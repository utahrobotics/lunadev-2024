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
    pub(super) arc_angle: Float,
    pub(super) radius: Float,
    pub(super) reversing: bool,
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
) -> impl IntoIterator<Item = (RobotState, NotNan<Float>)> + 'a {
    SUCCESSORS.into_iter().filter_map(move |offset| {
        let cell = offset + current.position.cast();
        if cell.x < 0 || cell.y < 0 {
            return None;
        }
        let cell = Vector2::new(cell.x as usize, cell.y as usize);
        if !*obstacles.get((cell.x as usize, cell.y as usize))? {
            return None;
        }
        let mut offset: Vector2<Float> = offset.cast();
        let distance = offset.magnitude();
        offset.unscale_mut(distance);
        let inner_angle = offset.angle(&current.forward);
        let mut cross = (current.forward.x * offset.y - current.forward.y * offset.x).signum();

        if !cross.is_finite() {
            cross = 1.0;
        }

        let arc_angle;
        let radius;
        let reversing;
        if inner_angle > PI / 2.0 && can_reverse {
                arc_angle = (PI - inner_angle) * -cross;
                radius = distance / (2.0 * (1.0 - inner_angle.cos())).sqrt();
                reversing = true;
        } else {
            arc_angle = inner_angle * cross;
            radius = distance / (2.0 * (1.0 - inner_angle.cos())).sqrt();
            reversing = false;
        }

        let cost = if !radius.is_finite() {
            distance
        } else {
            radius * arc_angle.abs()
        };

        Some((
            RobotState {
                position: cell,
                forward: Rotation2::new(arc_angle) * current.forward,
                arc_angle,
                radius,
                reversing
            },
            NotNan::new(cost).unwrap(),
        ))
    })
}
