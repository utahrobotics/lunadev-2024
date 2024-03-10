#![feature(binary_heap_drain_sorted)]

pub mod global;
pub mod local;

struct PointMeasurement {
    index: usize,
    height: isize,
}
