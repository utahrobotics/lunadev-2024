use core::{fmt::Debug, ops::Deref};

use ordered_float::{FloatCore, NotNan};
use unros::float::Float;

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub struct Steering<N: Float + FloatCore = f32> {
    pub left: NotNan<N>,
    pub right: NotNan<N>,
}

impl<N: Float + FloatCore> Debug for Steering<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Steering")
            .field("left", self.left.deref())
            .field("right", self.right.deref())
            .finish()
    }
}

impl<N: Float + FloatCore> Steering<N> {
    /// Shorthand to make this struct if you know the given values are not `NaN`.
    ///
    /// # Panics
    /// Panics if either left or right are `NaN`. To handle this possibility gracefully,
    /// you should just construct this struct normally as the fields are public.
    pub fn new(left: N, right: N) -> Self {
        Self {
            left: NotNan::new(left).unwrap(),
            right: NotNan::new(right).unwrap(),
        }
    }

    pub fn from_drive_and_steering(drive: NotNan<N>, steering: NotNan<N>) -> Self {
        let mut left = drive;
        let mut right = drive;

        if steering.into_inner() > N::zero() {
            right *=
                (nalgebra::convert::<_, N>(0.5) - steering.into_inner()) * nalgebra::convert(2.0);
        } else {
            left *=
                (nalgebra::convert::<_, N>(0.5) + steering.into_inner()) * nalgebra::convert(2.0);
        }

        Self { left, right }
    }
}
