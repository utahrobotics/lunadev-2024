use eigenvalues::{
    lanczos::{HermitianLanczos, LanczosError},
    SpectrumTarget,
};
use nalgebra::{
    convert as nconvert, DMatrix, Matrix4, Quaternion, UnitQuaternion, UnitVector3, Vector3,
};
use rand::{rngs::SmallRng, Rng};
use unros::float::Float;

// pub(crate) struct UnorderedQueue<T> {
//     queue: Box<[T]>,
//     index: usize,
// }

// impl<T> UnorderedQueue<T> {
//     pub fn push(&mut self, value: T) {
//         self.queue[self.index] = value;
//         self.index += 1;
//         if self.index >= self.queue.len() {
//             self.index = 0;
//         }
//     }

//     pub fn as_slice(&self) -> &[T] {
//         &self.queue
//     }

//     // pub fn as_mut_slice(&mut self) -> &mut [T] {
//     //     &mut self.queue
//     // }
// }

// impl<T> FromIterator<T> for UnorderedQueue<T> {
//     fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
//         UnorderedQueue {
//             queue: iter.into_iter().collect(),
//             index: 0,
//         }
//     }
// }

pub fn quat_mean<N, T, I>(quats: T) -> Option<Result<UnitQuaternion<N>, LanczosError>>
where
    N: Float,
    T: IntoIterator<Item = (UnitQuaternion<N>, N), IntoIter = I>,
    I: Iterator<Item = (UnitQuaternion<N>, N)>,
{
    let quats = quats.into_iter();

    let rotation_matrix: Matrix4<N> = quats
        .map(|(q, weight)| {
            let q_vec = q.as_vector();
            q_vec * q_vec.transpose() * weight
        })
        .sum();

    // https://math.stackexchange.com/questions/61146/averaging-quaternions
    match HermitianLanczos::new::<DMatrix<f64>>(
        nconvert(rotation_matrix),
        10,
        SpectrumTarget::Highest,
    ) {
        Ok(x) => {
            let ev = x.eigenvectors.column(0);
            Some(Ok(UnitQuaternion::new_normalize(Quaternion::new(
                nconvert(ev[3]),
                nconvert(ev[0]),
                nconvert(ev[1]),
                nconvert(ev[2]),
            ))))
        }
        Err(e) => Some(Err(e)),
    }
}

// #[inline]
// pub fn normal<N: Float>(mean: N, std_dev: N, x: N) -> N {
//     let two = N::one() + N::one();
//     let e: N = nconvert(std::f64::consts::E);
//     let tau: N = nconvert(std::f64::consts::TAU);
//     e.powf(((x - mean) / std_dev).powi(2) / -two) / std_dev / tau.sqrt()
// }

#[inline(always)]
pub fn gravity<N: Float>() -> Vector3<N> {
    Vector3::new(N::zero(), nconvert(-9.81), N::zero())
}

// #[inline]
// fn rand_quat(rng: &mut QuickRng) -> UnitQuaternion {
//     let u: Float = rng.gen_range(0.0..1.0);
//     let v: Float = rng.gen_range(0.0..1.0);
//     let w: Float = rng.gen_range(0.0..1.0);
//     // h = ( sqrt(1-u) sin(2πv), sqrt(1-u) cos(2πv), sqrt(u) sin(2πw), sqrt(u) cos(2πw))
//     UnitQuaternion::new_unchecked(Quaternion::new(
//         (1.0 - u).sqrt() * (TAU * v).sin(),
//         (1.0 - u).sqrt() * (TAU * v).cos(),
//         u.sqrt() * (TAU * w).sin(),
//         u.sqrt() * (TAU * w).cos(),
//     ))
// }

#[inline]
pub fn random_unit_vector<N: Float>(rng: &mut SmallRng) -> nalgebra::UnitVector3<N> {
    loop {
        let x = rng.gen_range(-1.0..1.0);
        let y = rng.gen_range(-1.0..1.0);
        let z = rng.gen_range(-1.0..1.0);
        let vec: nalgebra::Vector3<N> = nconvert(Vector3::new(x, y, z));
        let length = vec.magnitude();
        if length <= N::one() {
            break UnitVector3::new_unchecked(vec.unscale(length));
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, UnitVector3};

    use crate::{quat_mean, Float, Vector3};

    const EPSILON: Float = 0.001;

    #[test]
    fn quat_mean_zeroes() {
        assert_eq!(
            quat_mean([Default::default(); 30]).unwrap().unwrap(),
            Default::default()
        );
    }

    #[test]
    fn quat_mean_all_equal() {
        let quat = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            0.4,
        );
        assert!(quat_mean([quat; 30]).unwrap().unwrap().angle_to(&quat) < EPSILON);
    }

    #[test]
    fn quat_mean_all_opposing() {
        let quat01 = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            0.4,
        );
        let quat02 = UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
            -0.4,
        );
        assert!(
            quat_mean([quat01, quat02])
                .unwrap()
                .unwrap()
                .angle_to(&Default::default())
                < EPSILON
        );
    }
}
