#![no_std]

use ordered_float::NotNan;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Steering {
    pub drive: NotNan<f32>,
    pub steering: NotNan<f32>
}