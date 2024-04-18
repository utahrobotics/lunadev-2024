use std::{marker::PhantomData, mem::size_of};

use bytemuck::{bytes_of, bytes_of_mut, from_bytes};

pub trait BufferSize: Copy + Send + 'static {
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

impl<T: Send + 'static> BufferSize for StaticSize<T> {
    fn size(&self) -> u64 {
        size_of::<T>() as u64
    }
}

pub struct DynamicSize<T>(pub usize, PhantomData<T>);

impl<T: Send + 'static> BufferSize for DynamicSize<T> {
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

pub trait IntoBuffer: Copy {
    type Size: BufferSize;
    fn get_size(&self) -> Self::Size;
    fn into_buffer(self, buffer: &wgpu::Buffer, queue: &wgpu::Queue);
}

impl<T: bytemuck::Pod + Send + 'static> IntoBuffer for &T {
    type Size = StaticSize<T>;

    fn get_size(&self) -> Self::Size {
        StaticSize::default()
    }

    fn into_buffer(self, buffer: &wgpu::Buffer, queue: &wgpu::Queue) {
        queue.write_buffer(buffer, 0, bytes_of(self));
    }
}

impl<T: bytemuck::Pod + Send + 'static> IntoBuffer for &[T] {
    type Size = DynamicSize<T>;

    fn get_size(&self) -> Self::Size {
        DynamicSize::new(self.len())
    }

    fn into_buffer(self, buffer: &wgpu::Buffer, queue: &wgpu::Queue) {
        for (i, item) in self.iter().enumerate() {
            queue.write_buffer(buffer, (i * size_of::<T>()) as u64, bytes_of(item));
        }
    }
}

impl<T: Send + 'static> IntoBuffer for Option<&T>
where
    for<'a> &'a T: IntoBuffer,
{
    type Size = StaticSize<T>;

    fn get_size(&self) -> Self::Size {
        StaticSize::default()
    }

    fn into_buffer(self, buffer: &wgpu::Buffer, queue: &wgpu::Queue) {
        match self {
            Some(item) => item.into_buffer(buffer, queue),
            None => {}
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
        (&self.0).into_buffer(&buffers[0], queue);
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

pub trait FromBuffer: Send + 'static {
    type Receiver: Default + Send + 'static;
    type Size: BufferSize;

    fn from_buffer(recv: Self::Receiver, buffer: &wgpu::BufferView, size: Self::Size) -> Self;
}

pub struct Single<T>(pub T);

impl<T: bytemuck::Pod + Send + 'static> FromBuffer for Single<T> {
    type Receiver = ();
    type Size = StaticSize<T>;

    fn from_buffer(_recv: Self::Receiver, buffer: &wgpu::BufferView, _size: Self::Size) -> Self {
        let mut return_val = T::zeroed();
        let return_val_bytes = bytes_of_mut(&mut return_val);
        return_val_bytes.copy_from_slice(buffer);
        Single(return_val)
    }
}

impl<T: bytemuck::Pod + Send + 'static> FromBuffer for Vec<T> {
    type Receiver = Vec<T>;
    type Size = DynamicSize<T>;

    fn from_buffer(mut recv: Self::Receiver, buffer: &wgpu::BufferView, size: Self::Size) -> Self {
        recv.reserve(size.0 as usize);
        for i in 0..size.0 as usize {
            let item_slice = &buffer[(i * size_of::<T>())..((i + 1) * size_of::<T>())];
            let item: &T = from_bytes(item_slice);
            recv.push(*item);
        }
        recv
    }
}
