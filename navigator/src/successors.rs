use std::sync::Arc;

use nalgebra::{DMatrix, Rotation2, UnitVector2, Vector2};
use ordered_float::NotNan;
use unros_core::rayon::{self, iter::{IntoParallelIterator, ParallelIterator}};

use crate::Float;

pub const SUCCESSORS: [Vector2<isize>; 80] = [
    Vector2::new(-5, 0),
    Vector2::new(-4, -3),
    Vector2::new(-4, -2),
    Vector2::new(-4, -1),
    Vector2::new(-4, 0),
    Vector2::new(-4, 1),
    Vector2::new(-4, 2),
    Vector2::new(-4, 3),
    Vector2::new(-3, -4),
    Vector2::new(-3, -3),
    Vector2::new(-3, -2),
    Vector2::new(-3, -1),
    Vector2::new(-3, 0),
    Vector2::new(-3, 1),
    Vector2::new(-3, 2),
    Vector2::new(-3, 3),
    Vector2::new(-3, 4),
    Vector2::new(-2, -4),
    Vector2::new(-2, -3),
    Vector2::new(-2, -2),
    Vector2::new(-2, -1),
    Vector2::new(-2, 0),
    Vector2::new(-2, 1),
    Vector2::new(-2, 2),
    Vector2::new(-2, 3),
    Vector2::new(-2, 4),
    Vector2::new(-1, -4),
    Vector2::new(-1, -3),
    Vector2::new(-1, -2),
    Vector2::new(-1, -1),
    Vector2::new(-1, 0),
    Vector2::new(-1, 1),
    Vector2::new(-1, 2),
    Vector2::new(-1, 3),
    Vector2::new(-1, 4),
    Vector2::new(0, -5),
    Vector2::new(0, -4),
    Vector2::new(0, -3),
    Vector2::new(0, -2),
    Vector2::new(0, -1),
    Vector2::new(0, 1),
    Vector2::new(0, 2),
    Vector2::new(0, 3),
    Vector2::new(0, 4),
    Vector2::new(0, 5),
    Vector2::new(1, -4),
    Vector2::new(1, -3),
    Vector2::new(1, -2),
    Vector2::new(1, -1),
    Vector2::new(1, 0),
    Vector2::new(1, 1),
    Vector2::new(1, 2),
    Vector2::new(1, 3),
    Vector2::new(1, 4),
    Vector2::new(2, -4),
    Vector2::new(2, -3),
    Vector2::new(2, -2),
    Vector2::new(2, -1),
    Vector2::new(2, 0),
    Vector2::new(2, 1),
    Vector2::new(2, 2),
    Vector2::new(2, 3),
    Vector2::new(2, 4),
    Vector2::new(3, -4),
    Vector2::new(3, -3),
    Vector2::new(3, -2),
    Vector2::new(3, -1),
    Vector2::new(3, 0),
    Vector2::new(3, 1),
    Vector2::new(3, 2),
    Vector2::new(3, 3),
    Vector2::new(3, 4),
    Vector2::new(4, -3),
    Vector2::new(4, -2),
    Vector2::new(4, -1),
    Vector2::new(4, 0),
    Vector2::new(4, 1),
    Vector2::new(4, 2),
    Vector2::new(4, 3),
    Vector2::new(5, 0),
];

#[derive(Clone, Copy, PartialEq)]
pub(super) struct RobotState {
    pub(super) position: Vector2<usize>,
    pub(super) forward: UnitVector2<Float>,
    pub(super) reversing: bool,
    pub(super) arc_angle: Float,
    pub(super) turn_first: bool,
    pub(super) radius: Float,
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
    width: Float,
) -> impl IntoIterator<Item = (RobotState, NotNan<Float>)> {
    let (successor_sender, successor_recv) = std::sync::mpsc::sync_channel(SUCCESSORS.len() * 2);

    rayon::spawn(move || {
        SUCCESSORS.into_par_iter().for_each(|mut cell| {
            cell += current.position.cast();
            if cell.x < 0 || cell.y < 0 {
                return;
            }
            let cell = Vector2::new(cell.x as usize, cell.y as usize);
            if let Some((state, cost, is_direct)) = traverse_to(
                current.position,
                cell,
                current.forward,
                &obstacles,
                can_reverse,
                width,
            ) {
                let _ = successor_sender.send((state, NotNan::new(cost).unwrap()));
                if is_direct {
                    return;
                }
            }
            let direct = UnitVector2::new_normalize(cell.cast::<Float>() - current.position.cast());
            if let Some((mut state, mut cost, _)) = traverse_to(
                current.position,
                cell,
                direct,
                &obstacles,
                can_reverse,
                width,
            ) {
                cost += direct.angle(&current.forward) * width / 2.0;
                state.turn_first = true;
                let _ = successor_sender.send((state, NotNan::new(cost).unwrap()));
            }
        });
    });

    successor_recv
}

const PI: Float = std::f64::consts::PI as Float;

#[inline]
fn traverse_to(
    from: Vector2<usize>,
    to: Vector2<usize>,
    forward: UnitVector2<Float>,
    obstacles: &DMatrix<bool>,
    can_reverse: bool,
    width: Float,
) -> Option<(RobotState, Float, bool)> {
    if obstacles.get((to.x, to.y)).copied() == Some(false) {
        return None;
    }
    let travel = to.cast::<Float>() - from.cast();
    let mut angle_to_travel = forward.angle(&travel);
    let distance = travel.magnitude();

    // angle_to_travel cannot be greater than 180 degrees,
    // so if it is greater than 90 degrees then it must be obtuse
    let obtuse = angle_to_travel > PI / 2.0;
    let reversing = can_reverse && obtuse;
    if reversing {
        angle_to_travel -= PI / 2.0;
    }
    let inner_angle = (PI / 2.0 - angle_to_travel).abs();

    // Approx 1 degree
    if inner_angle < 0.02 {
        return Some((
            RobotState {
                position: to,
                forward,
                reversing,
                arc_angle: 0.0,
                turn_first: false,
                // A straight line is just a segment of an infinitely large circle
                radius: Float::INFINITY,
            },
            distance,
            true,
        ));
    }

    let offset_length = distance / 2.0 * inner_angle.tan();

    let mut cross = (forward.x * travel.y - forward.y * travel.x).signum();
    if reversing {
        cross *= -1.0;
    }
    let full_arc_angle = PI - inner_angle * 2.0;
    let mut offset = (Rotation2::new(cross * PI / 2.0) * travel).normalize() * offset_length;
    if !reversing && obtuse {
        offset *= -1.0;
    }
    let radius_vec = -offset - travel / 2.0;
    let radius = radius_vec.magnitude();
    let radial_cell = 1.0 / radius;
    let circle_origin = from.cast() - radius_vec;

    for i in 1..((full_arc_angle / radial_cell) as usize) {
        let next = Rotation2::new(cross * i as Float * radial_cell) * radius_vec + circle_origin;
        let next = Vector2::new(next.x.round(), next.y.round());
        if next.x < 0.0 || next.y < 0.0 {
            return None;
        }
        if obstacles.get((next.x as usize, next.y as usize)).copied() == Some(false) {
            return None;
        }
    }

    Some((
        RobotState {
            position: to,
            forward: Rotation2::new(cross * full_arc_angle) * forward,
            reversing,
            arc_angle: cross * full_arc_angle,
            turn_first: false,
            radius,
        },
        (radius + width / 2.0) * full_arc_angle,
        false,
    ))
}
