use std::{
    future::Future,
    marker::PhantomData,
    mem::{align_of, size_of, transmute},
    num::NonZeroU64,
    ops::Deref,
    sync::RwLock,
};

use bytemuck::{bytes_of, from_bytes, from_bytes_mut};
use crossbeam::queue::SegQueue;
use fxhash::FxHashMap;
use tokio::sync::oneshot;
use wgpu::{util::StagingBelt, CommandEncoder, Maintain, MapMode};

use crate::{get_gpu_device, GpuDevice};


#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReadOrWrite {
    ReadOnly,
    ReadWrite,
}

pub trait BufferSize: Copy + Default + Send + 'static {
    fn size(&self) -> u64;
}

pub struct StaticSize<T>(PhantomData<T>);

unsafe impl<T> Send for StaticSize<T> {}
unsafe impl<T> Sync for StaticSize<T> {}

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

impl<T: 'static> BufferSize for StaticSize<T> {
    fn size(&self) -> u64 {
        size_of::<T>() as u64
    }
}

pub struct DynamicSize<T>(pub usize, PhantomData<T>);

unsafe impl<T> Send for DynamicSize<T> {}
unsafe impl<T> Sync for DynamicSize<T> {}

impl<T: 'static> BufferSize for DynamicSize<T> {
    fn size(&self) -> u64 {
        let stride = size_of::<T>().next_multiple_of(align_of::<T>()) as u64;
        self.0 as u64 * stride
    }
}

impl<T> DynamicSize<T> {
    pub fn new(len: usize) -> Self {
        Self(len, PhantomData)
    }
}
impl<T> Default for DynamicSize<T> {
    fn default() -> Self {
        Self(0, PhantomData)
    }
}
impl<T> Copy for DynamicSize<T> {}
impl<T> Clone for DynamicSize<T> {
    fn clone(&self) -> Self {
        *self
    }
}

pub trait BufferInit {
    type Size: BufferSize;
}

impl<T: 'static> BufferInit for T {
    type Size = StaticSize<T>;
}

impl<T: 'static> BufferInit for [T] {
    type Size = DynamicSize<T>;
}

pub trait BufferSource<T: BufferInit> {
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    );
}

impl<T: BufferInit> BufferSource<T> for () {
    fn into_buffer(
        self,
        _command_encoder: &mut CommandEncoder,
        _buffer: &wgpu::Buffer,
        _stager: &mut StagingBelt,
        _device: &wgpu::Device,
    ) {
    }
}

impl<T: BufferInit + bytemuck::Pod> BufferSource<T> for &T {
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

impl<T: BufferInit + bytemuck::Pod> BufferSource<T> for &[T] {
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        let stride = size_of::<T>().next_multiple_of(align_of::<T>()) as u64;
        if const { size_of::<T>() % align_of::<T>() == 0 } {
            stager
                .write_buffer(
                    command_encoder,
                    buffer,
                    0,
                    NonZeroU64::new(stride * self.len() as u64).unwrap(),
                    device,
                )
                .copy_from_slice(unsafe { transmute(self) });
        } else {
            for (i, item) in self.iter().enumerate() {
                stager
                    .write_buffer(
                        command_encoder,
                        buffer,
                        i as u64 * stride,
                        NonZeroU64::new(stride).unwrap(),
                        device,
                    )
                    .copy_from_slice(bytes_of(item));
            }
        }
    }
}

impl<T: BufferInit + bytemuck::Pod> BufferSource<T> for Option<&T> {
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    ) {
        if let Some(item) = self {
            item.into_buffer(command_encoder, buffer, stager, device);
        }
    }
}

pub trait BufferDestination<T: BufferInit> {
    type State;
    fn enqueue(
        self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        device: &wgpu::Device,
    ) -> Self::State;
    fn from_buffer(
        self,
        state: Self::State,
        device: &wgpu::Device,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) -> impl Future<Output = ()>;
}

impl<T: BufferInit + bytemuck::Pod> BufferDestination<T> for &mut T {
    type State = wgpu::Buffer;

    fn enqueue(
        self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        device: &wgpu::Device,
    ) -> Self::State {
        let buffer = {
            let reader = buffers.read().unwrap();
            reader
                .get(&(size_of::<T>() as u64))
                .and_then(|queue| queue.pop())
                .unwrap_or_else(|| {
                    device.create_buffer(&wgpu::BufferDescriptor {
                        label: None,
                        size: size_of::<T>() as u64,
                        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                        mapped_at_creation: false,
                    })
                })
        };

        command_encoder.copy_buffer_to_buffer(&src_buffer, 0, &buffer, 0, size_of::<T>() as u64);

        buffer
    }

    async fn from_buffer(
        self,
        buffer: Self::State,
        device: &wgpu::Device,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) {
        {
            let slice = buffer.slice(..);
            let (sender, receiver) = oneshot::channel::<()>();
            slice.map_async(MapMode::Read, move |result| {
                if result.is_err() {
                    return;
                }
                let _ = sender.send(());
            });
            device.poll(Maintain::Poll);
            receiver.await.expect("Failed to map buffer");
            let slice = slice.get_mapped_range();
            let buffer_ref: &T = from_bytes(&slice);
            *self = *buffer_ref;
        }

        buffer.unmap();

        {
            let reader = buffers.read().unwrap();
            if let Some(queue) = reader.get(&(size_of::<T>() as u64)) {
                queue.push(buffer);
                return;
            }
        }

        let queue = SegQueue::new();
        queue.push(buffer);
        buffers
            .write()
            .unwrap()
            .insert(size_of::<T>() as u64, queue);
    }
}

impl<T: BufferInit + bytemuck::Pod + Send> BufferDestination<T> for &mut [T] {
    type State = wgpu::Buffer;

    fn enqueue(
        self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        device: &wgpu::Device,
    ) -> Self::State {
        let stride = size_of::<T>().next_multiple_of(align_of::<T>()) as u64;
        let size = stride * self.len() as u64;
        let buffer = {
            let reader = buffers.read().unwrap();
            reader
                .get(&size)
                .and_then(|queue| queue.pop())
                .unwrap_or_else(|| {
                    device.create_buffer(&wgpu::BufferDescriptor {
                        label: None,
                        size,
                        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                        mapped_at_creation: false,
                    })
                })
        };

        command_encoder.copy_buffer_to_buffer(&src_buffer, 0, &buffer, 0, size);

        buffer
    }

    async fn from_buffer(
        self,
        buffer: Self::State,
        device: &wgpu::Device,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) {
        {
            let slice = buffer.slice(..);
            let (sender, receiver) = oneshot::channel::<()>();
            slice.map_async(MapMode::Read, move |result| {
                if result.is_err() {
                    return;
                }
                let _ = sender.send(());
            });
            device.poll(Maintain::Poll);
            receiver.await.expect("Failed to map buffer");
            let slice = slice.get_mapped_range();

            let buffer_ref: &[T] = unsafe { transmute(slice.deref()) };
            self.copy_from_slice(buffer_ref);
        }

        buffer.unmap();
        let stride = size_of::<T>().next_multiple_of(align_of::<T>()) as u64;

        {
            let reader = buffers.read().unwrap();
            if let Some(queue) = reader.get(&stride) {
                queue.push(buffer);
                return;
            }
        }

        let queue = SegQueue::new();
        queue.push(buffer);
        buffers.write().unwrap().insert(stride, queue);
    }
}


pub struct OpaqueBuffer {
    buffer: wgpu::Buffer,
}


impl<T: 'static> BufferSource<T> for &OpaqueBuffer {
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        _stager: &mut StagingBelt,
        _device: &wgpu::Device,
    ) {
        command_encoder.copy_buffer_to_buffer(&self.buffer, 0, &buffer, 0, size_of::<T>() as u64);
    }
}

impl<T: 'static> BufferDestination<T> for &mut OpaqueBuffer {
    type State = ();

    fn enqueue(
        self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        _device: &wgpu::Device,
    ) -> Self::State {
        command_encoder.copy_buffer_to_buffer(&src_buffer, 0, &self.buffer, 0, self.buffer.size());
    }

    async fn from_buffer(
        self,
        (): Self::State,
        _device: &wgpu::Device,
        _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) { }
}

impl OpaqueBuffer {
    pub async fn new(size: impl BufferSize) -> anyhow::Result<Self> {
        let size = size.size();
        let GpuDevice { device, .. } = get_gpu_device().await?;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size,
            mapped_at_creation: false,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        });
        Ok(Self {
            buffer
        })
    }
    
    pub async fn new_from_value<T: bytemuck::Pod>(value: &T) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: size_of::<T>() as u64,
            mapped_at_creation: true,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        });
        buffer.slice(..).get_mapped_range_mut().copy_from_slice(bytes_of(value));
        buffer.unmap();
        Ok(Self {
            buffer
        })
    }
    
    pub async fn new_from_slice<T: bytemuck::Pod>(slice: &[T]) -> anyhow::Result<Self> {
        let stride = size_of::<T>().next_multiple_of(align_of::<T>()) as u64;
        let GpuDevice { device, .. } = get_gpu_device().await?;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: stride * slice.len() as u64,
            mapped_at_creation: true,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        });
        let bytes: &[u8] = unsafe { transmute(slice) };
        buffer.slice(..).get_mapped_range_mut().copy_from_slice(bytes);
        buffer.unmap();
        Ok(Self {
            buffer
        })
    }
}


// pub struct ManualBuffer<T: ?Sized> {
//     buffer: wgpu::Buffer,
//     _phantom: PhantomData<T>,
// }

// impl<T: bytemuck::Pod + 'static> BufferSource<T> for &ManualBuffer<T> {
//     fn into_buffer(
//         self,
//         command_encoder: &mut CommandEncoder,
//         buffer: &wgpu::Buffer,
//         _stager: &mut StagingBelt,
//         _device: &wgpu::Device,
//     ) {
//         command_encoder.copy_buffer_to_buffer(&self.buffer, 0, &buffer, 0, size_of::<T>() as u64);
//     }
// }

// impl<T: 'static> BufferDestination<T> for &mut ManualBuffer<T> {
//     type State = ();

//     fn enqueue(
//         self,
//         command_encoder: &mut CommandEncoder,
//         src_buffer: &wgpu::Buffer,
//         _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
//         _device: &wgpu::Device,
//     ) -> Self::State {
//         command_encoder.copy_buffer_to_buffer(&src_buffer, 0, &self.buffer, 0, self.buffer.size());
//     }

//     async fn from_buffer(
//         self,
//         (): Self::State,
//         _device: &wgpu::Device,
//         _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
//     ) { }
// }

// impl<T: 'static> ManualBuffer<T> {
//     pub async fn lock(&mut self) -> ManualBufferLock<T> {
//         let GpuDevice { device, .. } = get_gpu_device().await.unwrap();
//         let slice = self.buffer.slice(..);
//         let (sender, receiver) = oneshot::channel::<()>();
//         slice.map_async(MapMode::Read, move |result| {
//             if result.is_err() {
//                 return;
//             }
//             let _ = sender.send(());
//         });
//         device.poll(Maintain::Poll);
//         receiver.await.expect("Failed to map buffer");
//         let range = slice.get_mapped_range_mut();

//         ManualBufferLock {
//             buffer: &self.buffer,
//             _slice: slice,
//             range,
//             phantom: PhantomData,
//         }
//     }
// }

// pub struct ManualBufferLock<'a, T> {
//     buffer: &'a wgpu::Buffer,
//     _slice: wgpu::BufferSlice<'a>,
//     range: wgpu::BufferViewMut<'a>,
//     phantom: PhantomData<T>,
// }

// impl<'a, T: bytemuck::Pod> Deref for ManualBufferLock<'a, T> {
//     type Target = T;

//     fn deref(&self) -> &Self::Target {
//         let ptr: *const _ = &self.range;
//         let ptr: *const T = ptr.cast();
//         unsafe { &*ptr }
//     }
// }

// impl<'a, T: bytemuck::Pod> DerefMut for ManualBufferLock<'a, T> {
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         let ptr: *mut _ = &mut self.range;
//         let ptr: *mut T = ptr.cast();
//         unsafe { &mut *ptr }
//     }
// }

// impl<'a, T> Drop for ManualBufferLock<'a, T> {
//     fn drop(&mut self) {
//         self.buffer.unmap();
//     }
// }

// impl<'a, T: bytemuck::Pod> ManualBuffer<T> {
//     pub async fn new(value: &T) -> anyhow::Result<Self> {
//         let GpuDevice { device, .. } = get_gpu_device().await?;
//         let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
//             label: None,
//             contents: bytes_of(value),
//             usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
//         });
//         todo!()
//     }
// }