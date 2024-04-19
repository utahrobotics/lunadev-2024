use std::{
    marker::PhantomData,
    mem::size_of,
    num::NonZeroU64,
    ops::{Deref, DerefMut},
    sync::Arc,
};

use bytemuck::{bytes_of, from_bytes_mut};
use crossbeam::queue::SegQueue;
use wgpu::{util::StagingBelt, CommandEncoder};

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

pub trait BufferSizeIter: Copy {
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

impl<T: BufferSize, T1: BufferSize, T2: BufferSize, T3: BufferSize> BufferSizeIter
    for (T, T1, T2, T3)
{
    fn into_iter(self) -> impl Iterator<Item = u64> {
        [self.0.size(), self.1.size(), self.2.size(), self.3.size()].into_iter()
    }
}

pub trait IntoBuffer: Copy {
    type Size: BufferSize;
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    );
}

impl<T: bytemuck::Pod + Send + 'static> IntoBuffer for &T {
    type Size = StaticSize<T>;

    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        stager
            .write_buffer(
                command_encoder,
                buffer,
                0,
                NonZeroU64::new(size_of::<T>() as u64).unwrap(),
                device,
            )
            .copy_from_slice(bytes_of(self));
    }
}

impl<T: bytemuck::Pod + Send + 'static> IntoBuffer for &[T] {
    type Size = DynamicSize<T>;

    // fn get_size(&self) -> Self::Size {
    //     DynamicSize::new(self.len())
    // }

    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        for (i, item) in self.iter().enumerate() {
            stager
                .write_buffer(
                    command_encoder,
                    buffer,
                    (i * size_of::<T>()) as u64,
                    NonZeroU64::new(size_of::<T>() as u64).unwrap(),
                    device,
                )
                .copy_from_slice(bytes_of(item));
        }
    }
}

impl<'a, T: ?Sized> IntoBuffer for Option<&'a T>
where
    &'a T: IntoBuffer,
{
    type Size = <&'a T as IntoBuffer>::Size;

    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        match self {
            Some(item) => item.into_buffer(command_encoder, buffer, stager, device),
            None => {}
        }
    }
}

pub trait IntoBuffers {
    type Sizes: BufferSizeIter;
    fn into_buffers(
        &self,
        command_encoder: &mut CommandEncoder,
        buffers: &[wgpu::Buffer],
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    );
}

impl<T: IntoBuffer> IntoBuffers for (T,) {
    type Sizes = (T::Size,);

    fn into_buffers(
        &self,
        command_encoder: &mut CommandEncoder,
        buffers: &[wgpu::Buffer],
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        (&self.0).into_buffer(command_encoder, &buffers[0], stager, device);
    }
}

impl<T: IntoBuffer, T1: IntoBuffer> IntoBuffers for (T, T1) {
    type Sizes = (T::Size, T1::Size);

    fn into_buffers(
        &self,
        command_encoder: &mut CommandEncoder,
        buffers: &[wgpu::Buffer],
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        (&self.0).into_buffer(command_encoder, &buffers[0], stager, device);
        (&self.1).into_buffer(command_encoder, &buffers[1], stager, device);
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, T2: IntoBuffer> IntoBuffers for (T, T1, T2) {
    type Sizes = (T::Size, T1::Size, T2::Size);

    fn into_buffers(
        &self,
        command_encoder: &mut CommandEncoder,
        buffers: &[wgpu::Buffer],
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        (&self.0).into_buffer(command_encoder, &buffers[0], stager, device);
        (&self.1).into_buffer(command_encoder, &buffers[1], stager, device);
        (&self.2).into_buffer(command_encoder, &buffers[2], stager, device);
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, T2: IntoBuffer, T3: IntoBuffer> IntoBuffers
    for (T, T1, T2, T3)
{
    type Sizes = (T::Size, T1::Size, T2::Size, T3::Size);

    fn into_buffers(
        &self,
        command_encoder: &mut CommandEncoder,
        buffers: &[wgpu::Buffer],
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        (&self.0).into_buffer(command_encoder, &buffers[0], stager, device);
        (&self.1).into_buffer(command_encoder, &buffers[1], stager, device);
        (&self.2).into_buffer(command_encoder, &buffers[2], stager, device);
        (&self.3).into_buffer(command_encoder, &buffers[3], stager, device);
    }
}

pub trait FromBuffer: Send + 'static {
    type Size: BufferSize;

    fn from_buffer<'a>(buffer: &'a mut wgpu::BufferViewMut) -> &'a mut Self;
}

impl<T: bytemuck::Pod + Send> FromBuffer for T {
    type Size = StaticSize<T>;

    fn from_buffer<'a>(buffer: &'a mut wgpu::BufferViewMut) -> &'a mut Self {
        from_bytes_mut(buffer)
    }
}

impl<T: bytemuck::Pod + Send> FromBuffer for [T] {
    type Size = DynamicSize<T>;

    fn from_buffer<'a>(buffer: &'a mut wgpu::BufferViewMut) -> &'a mut Self {
        let buffer: &mut [u8] = buffer;
        if buffer.len() % size_of::<T>() != 0 {
            panic!("Buffer size is not a multiple of the size of T");
        }
        let count = buffer.len() / size_of::<T>();
        unsafe { std::slice::from_raw_parts_mut(buffer.as_mut_ptr().cast(), count) }
    }
}

pub struct ReturnBuffer<T: ?Sized> {
    buffer: Arc<wgpu::Buffer>,
    pointer: *mut T,
    buffer_queue: Arc<SegQueue<Arc<wgpu::Buffer>>>,
}

impl<T: FromBuffer + ?Sized> ReturnBuffer<T> {
    pub fn new(buffer: Arc<wgpu::Buffer>, buffer_queue: Arc<SegQueue<Arc<wgpu::Buffer>>>) -> Self {
        let pointer: *mut T = {
            let mut view = buffer.slice(..).get_mapped_range_mut();
            T::from_buffer(&mut view)
        };
        Self {
            buffer,
            pointer,
            buffer_queue,
        }
    }
}

impl<T: ?Sized> Deref for ReturnBuffer<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.pointer }
    }
}

impl<T: ?Sized> DerefMut for ReturnBuffer<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.pointer }
    }
}

unsafe impl<T: Send + ?Sized> Send for ReturnBuffer<T> {}
unsafe impl<T: Sync + ?Sized> Sync for ReturnBuffer<T> {}

impl<T: ?Sized> Drop for ReturnBuffer<T> {
    fn drop(&mut self) {
        self.buffer.unmap();
        self.buffer_queue.push(self.buffer.clone());
    }
}

impl<'a, T> IntoIterator for &'a ReturnBuffer<[T]> {
    type Item = &'a T;
    type IntoIter = std::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.deref().iter()
    }
}

impl<'a, T> IntoIterator for &'a mut ReturnBuffer<[T]> {
    type Item = &'a mut T;
    type IntoIter = std::slice::IterMut<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.deref_mut().iter_mut()
    }
}
