use std::mem::size_of;

use bytemuck::bytes_of;


pub trait IntoBuffers {
    fn sizes() -> impl Iterator<Item = usize>;
    fn into_buffers(&self, buffers: &[wgpu::Buffer]);
}

impl<T: bytemuck::Pod> IntoBuffers for (T,) {
    fn sizes() -> impl Iterator<Item = usize> {
        std::iter::once(size_of::<T>())
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer]) {
        buffers[0].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.0));
    }
}

impl<T: bytemuck::Pod, T1: bytemuck::Pod> IntoBuffers for (T, T1) {
    fn sizes() -> impl Iterator<Item = usize> {
        std::iter::once(size_of::<T>())
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer]) {
        buffers[0].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.0));
        buffers[1].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.1));
    }
}

impl<T: bytemuck::Pod, T1: bytemuck::Pod, T2: bytemuck::Pod> IntoBuffers for (T, T1, T2) {
    fn sizes() -> impl Iterator<Item = usize> {
        std::iter::once(size_of::<T>())
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer]) {
        buffers[0].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.0));
        buffers[1].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.1));
        buffers[2].slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(&self.2));
    }
}
