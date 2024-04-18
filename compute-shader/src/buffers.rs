use std::{marker::PhantomData, mem::size_of};

use bytemuck::{bytes_of, bytes_of_mut, from_bytes};


pub trait BufferSize: Copy {
    fn size(&self) -> u64;
}


pub struct StaticSize<T>(PhantomData<T>);

impl<T> Default for StaticSize<T> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<T> Copy for StaticSize<T> {}
impl<T> Clone for StaticSize<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> BufferSize for StaticSize<T> {
    fn size(&self) -> u64 {
        size_of::<T>() as u64
    }
}


pub struct DynamicSize<T>(pub usize, PhantomData<T>);

impl<T> BufferSize for DynamicSize<T> {
    fn size(&self) -> u64 {
        (self.0 * size_of::<T>()) as u64
    }
}

impl<T> DynamicSize<T> {
    pub fn new(len: usize) -> Self {
        Self(len, PhantomData)
    }
}
impl<T> Copy for DynamicSize<T> {}
impl<T> Clone for DynamicSize<T> {
    fn clone(&self) -> Self {
        *self
    }
}


pub trait BufferSizeIter {
    fn into_iter(self) -> impl Iterator<Item = u64>;
}


impl<T: BufferSize> BufferSizeIter for (T,) {
    fn into_iter(self) -> impl Iterator<Item = u64> {
        std::iter::once(self.0.size())
    }
}


impl<T: BufferSize, T1: BufferSize> BufferSizeIter for (T, T1) {
    fn into_iter(self) -> impl Iterator<Item = u64> {
        [self.0.size(), self.1.size()].into_iter()
    }
}


impl<T: BufferSize, T1: BufferSize, T2: BufferSize> BufferSizeIter for (T, T1, T2) {
    fn into_iter(self) -> impl Iterator<Item = u64> {
        [self.0.size(), self.1.size(), self.2.size()].into_iter()
    }
}


pub trait IntoBuffer {
    type Size: BufferSize;
    fn get_size(&self) -> Self::Size;
    fn into_buffer(&self, buffer: &wgpu::Buffer, queue: &wgpu::Queue);
}


pub struct Pod<T: bytemuck::Pod>(pub T);


impl<T: bytemuck::Pod> IntoBuffer for Pod<T> {
    type Size = StaticSize<T>;

    fn get_size(&self) -> Self::Size {
        StaticSize::default()
    }

    fn into_buffer(&self, buffer: &wgpu::Buffer, queue: &wgpu::Queue) {
        queue.write_buffer(buffer, 0, bytes_of(&self.0));
    }
}


impl<T: bytemuck::Pod> IntoBuffer for &[T] {
    type Size = DynamicSize<T>;

    fn get_size(&self) -> Self::Size {
        DynamicSize::new(self.len())
    }

    fn into_buffer(&self, buffer: &wgpu::Buffer, queue: &wgpu::Queue) {
        for (i, item) in self.iter().enumerate() {
            queue.write_buffer(buffer, (i * size_of::<T>()) as u64, bytes_of(item));
        }
    }
}


pub trait IntoBuffers {
    type Sizes: BufferSizeIter;
    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue);
}

impl<T: IntoBuffer> IntoBuffers for (T,) {
    type Sizes = (T::Size,);

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        self.0.into_buffer(&buffers[0], queue);
    }
}

impl<T: IntoBuffer, T1: IntoBuffer> IntoBuffers for (T, T1) {
    type Sizes = (T::Size, T1::Size);

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        self.0.into_buffer(&buffers[0], queue);
        self.1.into_buffer(&buffers[1], queue);
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, T2: IntoBuffer> IntoBuffers for (T, T1, T2) {
    type Sizes = (T::Size, T1::Size, T2::Size);

    fn into_buffers(&self, buffers: &[wgpu::Buffer], queue: &wgpu::Queue) {
        self.0.into_buffer(&buffers[0], queue);
        self.1.into_buffer(&buffers[1], queue);
        self.2.into_buffer(&buffers[2], queue);
    }
}


pub trait FromBuffer {
    type Size: BufferSize;

    fn from_buffer(buffer: &wgpu::BufferView, size: Self::Size) -> Self;
}


impl<T: bytemuck::Pod> FromBuffer for Pod<T> {
    type Size = StaticSize<T>;
    
    fn from_buffer(buffer: &wgpu::BufferView, _size: Self::Size) -> Self {
        let mut return_val = T::zeroed();
        let return_val_bytes = bytes_of_mut(&mut return_val);
        return_val_bytes.copy_from_slice(buffer);
        Pod(return_val)
    }
}


impl<T: bytemuck::Pod> FromBuffer for Vec<T> {
    type Size = DynamicSize<T>;
    
    fn from_buffer(buffer: &wgpu::BufferView, size: Self::Size) -> Self {
        let mut return_val = Vec::with_capacity(size.0 as usize);
        for i in 0..size.0 as usize{
            let item_slice = &buffer[(i * size_of::<T>())..((i + 1) * size_of::<T>())];
            let item: &T = from_bytes(item_slice);
            return_val.push(*item);
        
        }
        return_val
    }
}