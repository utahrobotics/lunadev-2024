use std::sync::Arc;

use compute_shader::buffers::ReturnBuffer;
use nalgebra::Point3;

#[derive(Clone)]
pub struct RealSensePoints {
    pub(crate) buffer: Arc<ReturnBuffer<[[f32; 4]]>>,
    pub(crate) len: usize,
}

pub struct RealSensePointsIter {
    buffer: Arc<ReturnBuffer<[[f32; 4]]>>,
    index: usize,
    len: usize,
}

impl IntoIterator for RealSensePoints {
    type Item = Point3<f32>;
    type IntoIter = RealSensePointsIter;

    fn into_iter(self) -> Self::IntoIter {
        RealSensePointsIter {
            buffer: self.buffer,
            index: 0,
            len: self.len,
        }
    }
}

impl Iterator for RealSensePointsIter {
    type Item = Point3<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let [x, y, z, v] = self.buffer.get(self.index)?;
            self.index += 1;
            if *v == 0.0 {
                continue;
            }
            break Some(Point3::new(*x, *y, *z));
        }
    }
}

impl ExactSizeIterator for RealSensePointsIter {
    fn len(&self) -> usize {
        self.len
    }
}
