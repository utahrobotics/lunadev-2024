use std::iter::Sum;

use nalgebra::RealField;
use simba::scalar::{SubsetOf, SupersetOf};

pub trait Float:
    RealField
    + Copy
    + Default
    + SupersetOf<f32>
    + SubsetOf<f32>
    + SupersetOf<f64>
    + SubsetOf<f64>
    + SupersetOf<usize>
    + SupersetOf<u64>
    + SupersetOf<u32>
    + SupersetOf<u16>
    + SupersetOf<u8>
    + SupersetOf<isize>
    + SupersetOf<i64>
    + SupersetOf<i32>
    + SupersetOf<i16>
    + SupersetOf<i8>
    + Sum
{
    fn to_f32(self) -> f32;
    fn to_f64(self) -> f64;
    fn is_nan(self) -> bool;

    fn is_f32() -> bool;
    fn is_f64() -> bool;
}

impl Float for f32 {
    fn to_f32(self) -> f32 {
        self
    }

    fn to_f64(self) -> f64 {
        self as f64
    }

    fn is_nan(self) -> bool {
        self.is_nan()
    }

    fn is_f32() -> bool {
        true
    }

    fn is_f64() -> bool {
        false
    }
}
impl Float for f64 {
    fn to_f32(self) -> f32 {
        self as f32
    }

    fn to_f64(self) -> f64 {
        self
    }

    fn is_nan(self) -> bool {
        self.is_nan()
    }

    fn is_f32() -> bool {
        false
    }

    fn is_f64() -> bool {
        true
    }
}
