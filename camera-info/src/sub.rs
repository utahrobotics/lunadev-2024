use std::sync::Arc;

use image::{DynamicImage, ImageBuffer, Rgb};
use opencv::{calib3d::undistort, core::{Mat, MatTraitConstManual, MatTraitManual, Vector, CV_8UC3}};
use unros::{log, pubsub::subs::{PublisherToken, Subscription}};

use crate::DistortionData;

pub struct Undistorter<I> {
    pub(super) inner: I,
    pub(super) distortion_data: Arc<DistortionData>
}


impl<I: Subscription<Item = Arc<DynamicImage>>> Subscription for Undistorter<I> {
    type Item = Arc<DynamicImage>;

    fn push(&mut self, dyn_img: Self::Item, token: PublisherToken) -> bool {
        let mut src = Mat::new_rows_cols_with_default(
            dyn_img.height() as i32,
            dyn_img.width() as i32,
            CV_8UC3,
            1.into(),
        )
        .unwrap();
        src.data_bytes_mut()
            .unwrap()
            .copy_from_slice(&dyn_img.to_rgb8());
        let mut dst = Mat::new_rows_cols_with_default(
            dyn_img.height() as i32,
            dyn_img.width() as i32,
            CV_8UC3,
            1.into(),
        )
        .unwrap();
        let camera_matrix =
            Mat::from_slice_rows_cols(&self.distortion_data.camera_matrix, 3, 3).unwrap();
        let new_camera_matrix =
            Mat::from_slice_rows_cols(&self.distortion_data.new_camera_matrix, 3, 3).unwrap();
        let dist_coeffs: Vector<f64> = self.distortion_data
            .distortion_coefficients
            .iter()
            .copied()
            .collect();

        match undistort(
            &src,
            &mut dst,
            &camera_matrix,
            &dist_coeffs,
            &new_camera_matrix,
        ) {
            Ok(()) => {}
            Err(e) => {
                log::error!("Failed to undistort image: {e}");
                return true;
            }
        }

        let img = ImageBuffer::<Rgb<u8>, _>::from_vec(
            dyn_img.width(),
            dyn_img.height(),
            dst.data_bytes().unwrap().to_vec(),
        )
        .unwrap();
        // let mut img: DynamicImage = img.into();
        // img = img
        //     .crop_imm(
        //         distortion_data.roi_x as u32,
        //         distortion_data.roi_y as u32,
        //         distortion_data.roi_width as u32,
        //         distortion_data.roi_height as u32,
        //     )
        //     .resize_to_fill(dyn_img.width(), dyn_img.height(), FilterType::Triangle);

        self.inner.push(Arc::new(img.into()), token)
    }

    fn set_name_mut(&mut self, name: Box<str>) {
        self.inner.set_name_mut(name);
    }

    fn increment_publishers(&self, token: PublisherToken) {
        self.inner.increment_publishers(token);
    }

    fn decrement_publishers(&self, token: PublisherToken) {
        self.inner.decrement_publishers(token);
    }
}
