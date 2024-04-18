use std::mem::size_of;

use bytemuck::bytes_of;

pub trait IntoBuffers {
    fn sizes() -> impl IntoIterator<Item = usize>;
    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue);
}

impl<T: bytemuck::Pod> IntoBuffers for (T,) {
    fn sizes() -> impl IntoIterator<Item = usize> {
        [size_of::<T>()]
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        queue.write_buffer(&buffers[0], 0, bytes_of(&self.0));
    }
}

impl<T: bytemuck::Pod, T1: bytemuck::Pod> IntoBuffers for (T, T1) {
    fn sizes() -> impl IntoIterator<Item = usize> {
        [size_of::<T>(), size_of::<T1>()]
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        queue.write_buffer(&buffers[0], 0, bytes_of(&self.0));
        queue.write_buffer(&buffers[1], 0, bytes_of(&self.1));
    }
}

impl<T: bytemuck::Pod, T1: bytemuck::Pod, T2: bytemuck::Pod> IntoBuffers for (T, T1, T2) {
    fn sizes() -> impl IntoIterator<Item = usize> {
        [size_of::<T>(), size_of::<T1>(), size_of::<T2>()]
    }

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        queue.write_buffer(&buffers[0], 0, bytes_of(&self.0));
        queue.write_buffer(&buffers[1], 0, bytes_of(&self.1));
        queue.write_buffer(&buffers[2], 0, bytes_of(&self.2));
    }
}
