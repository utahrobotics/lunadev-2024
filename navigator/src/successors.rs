use std::sync::Arc;

use nalgebra::{DMatrix, Rotation2, UnitVector2, Vector2};
use ordered_float::NotNan;

use crate::Float;

pub const SUCCESSORS: [Vector2<isize>; 8] = [
    Vector2::new(-1, -1),
    Vector2::new(-1, 0),
    Vector2::new(-1, 1),
    Vector2::new(0, -1),
    Vector2::new(0, 1),
    Vector2::new(1, -1),
    Vector2::new(1, 0),
    Vector2::new(1, 1),
];

#[derive(Clone, Copy, Debug)]
pub(super) struct RobotState {
    pub(super) position: Vector2<usize>,
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
        NotNan::new(self.forward.x).unwrap().hash(state);
        NotNan::new(self.forward.y).unwrap().hash(state);
    }
}

pub(super) fn successors(
    current: RobotState,
    obstacles: Arc<DMatrix<bool>>,
    can_reverse: bool,
) -> impl IntoIterator<Item = (RobotState, NotNan<Float>)> {
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
        if inner_angle > PI / 2.0 {
            if can_reverse {
                arc_angle = PI * 2.0 - 2.0 * inner_angle * cross;
                radius = distance / (2.0 * (1.0 - arc_angle.abs().cos())).sqrt();
                reversing = true;
            } else {
                arc_angle = 2.0 * inner_angle * -cross;
                radius = distance / (2.0 * (1.0 - (PI * 2.0 - arc_angle.abs()).cos())).sqrt();
                reversing = false;
            }
        } else {
            arc_angle = 2.0 * inner_angle * cross;
            radius = distance / (2.0 * (1.0 - arc_angle.abs().cos())).sqrt();
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

const PI: Float = std::f64::consts::PI as Float;

// #[inline]
// pub(super) fn traverse_to(
//     from: Vector2<usize>,
//     to: Vector2<usize>,
//     forward: UnitVector2<Float>,
//     obstacles: &DMatrix<bool>,
//     can_reverse: bool,
//     width: Float,
// ) -> Option<(RobotState, Float)> {
//     if obstacles.get((to.x, to.y)).copied() == Some(false) {
//         return None;
//     }
//     let travel = to.cast::<Float>() - from.cast();
//     let mut angle_to_travel = forward.angle(&travel);
//     let distance = travel.magnitude();

//     // angle_to_travel cannot be greater than 180 degrees,
//     // so if it is greater than 90 degrees then it must be obtuse
//     let obtuse = angle_to_travel > PI / 2.0;
//     let reversing = can_reverse && obtuse;
//     if reversing {
//         angle_to_travel -= PI / 2.0;
//     }
//     let inner_angle = (PI / 2.0 - angle_to_travel).abs();

//     // Approx 1 degree
//     if inner_angle < 0.02 {
//         return Some((
//             RobotState {
//                 position: to,
//                 forward,
//                 reversing,
//                 arc_angle: 0.0,
//                 turn_first: false,
//                 // A straight line is just a segment of an infinitely large circle
//                 radius: Float::INFINITY,
//             },
//             distance,
//         ));
//     }

//     let offset_length = distance / 2.0 * inner_angle.tan();

//     let mut cross = (forward.x * travel.y - forward.y * travel.x).signum();
//     if reversing {
//         cross *= -1.0;
//     }
//     let full_arc_angle = PI - inner_angle * 2.0;
//     let mut offset = (Rotation2::new(cross * PI / 2.0) * travel).normalize() * offset_length;
//     if !reversing && obtuse {
//         offset *= -1.0;
//     }
//     let radius_vec = -offset - travel / 2.0;
//     let radius = radius_vec.magnitude();
//     let radial_cell = 1.0 / radius;
//     let circle_origin = from.cast() - radius_vec;

//     for i in 1..((full_arc_angle / radial_cell) as usize) {
//         let next = Rotation2::new(cross * i as Float * radial_cell) * radius_vec + circle_origin;
//         let next = Vector2::new(next.x.round(), next.y.round());
//         if next.x < 0.0 || next.y < 0.0 {
//             return None;
//         }
//         if obstacles.get((next.x as usize, next.y as usize)).copied() == Some(false) {
//             return None;
//         }
//     }

//     Some((
//         RobotState {
//             position: to,
//             forward: Rotation2::new(cross * full_arc_angle) * forward,
//             reversing,
//             arc_angle: cross * full_arc_angle,
//             turn_first: false,
//             radius,
//         },
//         (radius + width / 2.0) * full_arc_angle,
//     ))
// }
