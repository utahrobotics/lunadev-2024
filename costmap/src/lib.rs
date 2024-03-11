#![feature(binary_heap_drain_sorted)]

use rig::RobotElementRef;

pub mod global;
pub mod local;

struct PointMeasurement {
    index: usize,
    height: isize,
}

#[derive(Clone)]
pub struct Points<T> {
    pub points: T,
    pub robot_element: RobotElementRef,
}
