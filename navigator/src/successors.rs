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
pub(super) struct RobotState<T = usize> {
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

#[inline]
pub(super) fn successors<'a>(
    current: RobotState,
    obstacles: &'a DMatrix<bool>,
    can_reverse: bool,
    width: Float,
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
        let mut inner_angle = offset.angle(&current.forward);
        let mut reversing = false;

        if inner_angle > PI / 2.0 {
            if !can_reverse {
                return None;
            }
            reversing = true;
            cross *= -1.0;
            inner_angle = PI - inner_angle;
        }
        let arc_angle = inner_angle * 2.0;
        let radius = distance / arc_angle.sin() * (arc_angle / 2.0).cos();

        let cost;
        let mut left_steering;
        let mut right_steering;
        if radius.is_finite() {
            cost = arc_angle * radius;
            let smaller_ratio = (radius - width / 2.0) / (radius + width / 2.0);
            if cross > 0.0 {
                left_steering = smaller_ratio;
                right_steering = 1.0;
            } else {
                right_steering = smaller_ratio;
                left_steering = 1.0;
            }
        } else {
            cost = distance;
            left_steering = 1.0;
            right_steering = 1.0;
        };

        if reversing {
            left_steering *= -1.0;
            right_steering *= -1.0;
        }

        Some((
            RobotState {
                position: cell,
                forward: Rotation2::new(arc_angle * cross) * current.forward,
                left_steering: NotNan::new(left_steering).unwrap(),
                right_steering: NotNan::new(right_steering).unwrap(),
            },
            NotNan::new(cost).unwrap(),
        ))
    })
}

#[inline]
pub(super) fn traverse_to(from: &RobotState, to: &RobotState, obstacles: &DMatrix<bool>, can_reverse: bool, width: Float) -> Option<RobotState> {
    let mut offset = to.position.cast::<Float>() - from.position.cast();
    let distance = offset.magnitude();
    offset.unscale_mut(distance);
    let mut cross = (from.forward.x * offset.y - from.forward.y * offset.x).signum();
    let mut inner_angle = offset.angle(&from.forward);
    let mut reversing = false;
    println!("{:?} {:?}", from.forward, offset);
    println!("{:.2}", inner_angle * cross / PI * 180.0);

    if inner_angle > PI / 2.0 {
        if !can_reverse {
            return None;
        }
        reversing = true;
        cross *= -1.0;
        inner_angle = PI - inner_angle;
    }
    let arc_angle = inner_angle * 2.0;
    let radius = distance / arc_angle.sin() * (arc_angle / 2.0).cos();

    let mut left_steering;
    let mut right_steering;
    let cost;
    if radius.is_finite() {
        cost = arc_angle * radius;
        let smaller_ratio = (radius - width / 2.0) / (radius + width / 2.0);
        if cross > 0.0 {
            left_steering = smaller_ratio;
            right_steering = 1.0;
        } else {
            right_steering = smaller_ratio;
            left_steering = 1.0;
        }
    } else {
        cost = distance;
        left_steering = 1.0;
        right_steering = 1.0;
    };

    if reversing {
        left_steering *= -1.0;
        right_steering *= -1.0;
    }

    if radius.is_finite() {
        // Cross of up-vector (+y) with forward vector
        let forward_cross = - from.forward.x.signum();
        let forward_angle = Vector2::y_axis().dot(&from.forward);
        let rotation = Rotation2::new(forward_cross * forward_angle);

        let speed = (left_steering + right_steering) / 2.0;
        let time_scale = 1.0 / speed.abs();
        let angular_vel = (left_steering - right_steering) / width;

        for i in 1..(cost.round() as usize) {
            let t = i as Float * time_scale;
            let x = - (speed * (t * angular_vel).cos() - speed) / angular_vel;
            let y = speed * (t * angular_vel).sin() / angular_vel;
            let coords = rotation * Vector2::new(x, y) + from.position.cast();
    
            if coords.x < 0.0 || coords.y < 0.0 {
                return None;
            }
    
            if !*obstacles.get((coords.y as usize, coords.x as usize))? {
                return None;
            }
        }

    } else {
        for i in 1..(cost.round() as usize) {
            let coords = from.forward.scale(i as Float) + from.position.cast();
            
            if !*obstacles.get((coords.y as usize, coords.x as usize)).unwrap() {
                return None;
            }
        }
    }

    Some(
        RobotState {
            position: to.position,
            forward: Rotation2::new(arc_angle * cross) * from.forward,
            left_steering: NotNan::new(left_steering).unwrap(),
            right_steering: NotNan::new(right_steering).unwrap(),
        }
    )
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use nalgebra::{Rotation2, Vector2};

    #[test]
    fn test01() {
        let a = Vector2::<f32>::x_axis();
        let b = Vector2::<f32>::y_axis();

        assert!((Rotation2::new(- PI / 2.0) * b).relative_eq(&a, 0.001, 0.001));
    }

    #[test]
    fn test02() {
        let a = Vector2::<f32>::x_axis();
        let b = Vector2::<f32>::y_axis();

        assert!((a.x * b.y - a.y * b.x).signum() > 0.0);
    }
}