use std::{
    future::Future,
    marker::PhantomData,
    mem::{align_of, size_of, transmute},
    num::NonZeroU64,
    ops::Deref,
    sync::RwLock,
};

use bytemuck::{bytes_of, from_bytes};
use crossbeam::queue::SegQueue;
use fxhash::FxHashMap;
use tokio::sync::oneshot;
use wgpu::{util::StagingBelt, CommandEncoder, Maintain, MapMode};

use crate::{get_gpu_device, GpuDevice};


pub trait HostReadableWritable {
    const CAN_READ: bool;
    const CAN_WRITE: bool;
}

#[derive(Debug, Clone, Copy)]
pub struct HostReadOnly;

impl HostReadableWritable for HostReadOnly {
    const CAN_READ: bool = true;
    const CAN_WRITE: bool = false;
}
#[derive(Debug, Clone, Copy)]
pub struct HostWriteOnly;

impl HostReadableWritable for HostWriteOnly {
    const CAN_READ: bool = false;
    const CAN_WRITE: bool = true;
}

#[derive(Debug, Clone, Copy)]
pub struct HostReadWrite;

impl HostReadableWritable for HostReadWrite {
    const CAN_READ: bool = true;
    const CAN_WRITE: bool = true;
}

pub trait ShaderWritable {
    const CAN_WRITE: bool;
}

#[derive(Debug, Clone, Copy)]
pub struct ShaderReadOnly;

impl ShaderWritable for ShaderReadOnly {
    const CAN_WRITE: bool = false;
}

#[derive(Debug, Clone, Copy)]
pub struct ShaderReadWrite;

impl ShaderWritable for ShaderReadWrite {
    const CAN_WRITE: bool = true;
}

pub struct BufferType<T: BufferSized, H, S> {
    size: T::Size,
    _phantom: PhantomData<(T, H, S)>,
}

impl<T: BufferSized, H, S> Clone for BufferType<T, H, S> {
    fn clone(&self) -> Self {
        Self { size: self.size, _phantom: PhantomData }
    }
}

impl<T: BufferSized, H, S> Copy for BufferType<T, H, S> { }

impl<T: BufferSized<Size=StaticSize<T>>, H, S> BufferType<T, H, S> {
    pub fn new() -> Self {
        Self {
            size: StaticSize::default(),
            _phantom: PhantomData,
        }
    }
}

impl<T: BufferSized<Size=DynamicSize<T>>, H, S> BufferType<T, H, S> {
    pub fn new_dyn(len: usize) -> Self {
        Self {
            size: DynamicSize::new(len),
            _phantom: PhantomData,
        }
    }
}

impl<T: BufferSized, H: HostReadableWritable, S: ShaderWritable> BufferType<T, H, S> {
    const HOST_CAN_READ: bool = H::CAN_READ;
    const HOST_CAN_WRITE: bool = H::CAN_WRITE;
    const SHADER_CAN_WRITE: bool = S::CAN_WRITE;

    pub fn size(&self) -> u64 {
        self.size.size()
    }

    pub(crate) fn into_buffer(&self, index: usize, device: &wgpu::Device) -> wgpu::Buffer {
        let additional_usage = if Self::HOST_CAN_READ && Self::HOST_CAN_WRITE {
            wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST
        } else if Self::HOST_CAN_WRITE {
            wgpu::BufferUsages::COPY_DST
        } else {
            wgpu::BufferUsages::COPY_SRC
        };
        
        if Self::SHADER_CAN_WRITE || self.size.size() > 64000 {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(&format!("Arg Buffer {index}")),
                size: self.size.size(),
                usage: wgpu::BufferUsages::STORAGE | additional_usage,
                mapped_at_creation: false,
            })
        } else {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(&format!("Arg Buffer {index}")),
                size: self.size.size(),
                usage: wgpu::BufferUsages::UNIFORM | additional_usage,
                mapped_at_creation: false,
            })
        }
    }

    pub(crate) fn into_layout(&self, binding: u32) -> wgpu::BindGroupLayoutEntry {
        if Self::SHADER_CAN_WRITE {
            wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }
        } else if self.size.size() > 64000 {
            wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }
        } else {
            wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }
        }
    }
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

pub trait BufferSized {
    type Size: BufferSize;
}

impl<T: 'static> BufferSized for T {
    type Size = StaticSize<T>;
}

impl<T: 'static> BufferSized for [T] {
    type Size = DynamicSize<T>;
}

pub trait BufferSource<T: BufferSized> {
    fn into_buffer(
        self,
        command_encoder: &mut CommandEncoder,
        buffer: &wgpu::Buffer,
        stager: &mut StagingBelt,
        device: &wgpu::Device,
    );
}

impl<T: 'static> BufferSource<T> for () {
    fn into_buffer(
        self,
        _command_encoder: &mut CommandEncoder,
        _buffer: &wgpu::Buffer,
        _stager: &mut StagingBelt,
        _device: &wgpu::Device,
    ) {
    }
}

impl<T: BufferSized + bytemuck::Pod> BufferSource<T> for &T {
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

impl<T: BufferSized + bytemuck::Pod> BufferSource<T> for &[T] {
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

impl<T: BufferSized + bytemuck::Pod> BufferSource<T> for Option<&T> {
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

pub trait BufferDestination<T: BufferSized> {
    type State;
    fn enqueue(
        &self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        device: &wgpu::Device,
    ) -> Self::State;
    fn from_buffer(
        &mut self,
        state: Self::State,
        device: &wgpu::Device,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) -> impl Future<Output = ()>;
}

impl<T: BufferSized + bytemuck::Pod> BufferDestination<T> for &mut T {
    type State = wgpu::Buffer;

    fn enqueue(
        &self,
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
        &mut self,
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
            **self = *buffer_ref;
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

impl<T: BufferSized + bytemuck::Pod + Send> BufferDestination<T> for &mut [T] {
    type State = wgpu::Buffer;

    fn enqueue(
        &self,
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
        &mut self,
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


impl<'a, T: 'static> BufferDestination<T> for Option<&'a mut T> where &'a mut T: BufferDestination<T> {
    type State = Option<<&'a mut T as BufferDestination<T>>::State>;

    fn enqueue(
        &self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        device: &wgpu::Device,
    ) -> Self::State {
        if let Some(item) = self {
            Some(item.enqueue(command_encoder, src_buffer, buffers, device))
        } else {
            None
        }
    }

    async fn from_buffer(
        &mut self,
        state: Self::State,
        device: &wgpu::Device,
        buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) {
        if let Some(item) = self {
            item.from_buffer(state.unwrap(), device, buffers).await;
        }
    }
}


impl<T: 'static> BufferDestination<T> for () {
    type State = ();

    fn enqueue(
        &self,
        _command_encoder: &mut CommandEncoder,
        _src_buffer: &wgpu::Buffer,
        _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        _device: &wgpu::Device,
    ) -> Self::State {

    }

    async fn from_buffer(
        &mut self,
        _state: Self::State,
        _device: &wgpu::Device,
        _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
    ) {
        
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
        &self,
        command_encoder: &mut CommandEncoder,
        src_buffer: &wgpu::Buffer,
        _buffers: &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        _device: &wgpu::Device,
    ) -> Self::State {
        command_encoder.copy_buffer_to_buffer(&src_buffer, 0, &self.buffer, 0, self.buffer.size());
    }

    async fn from_buffer(
        &mut self,
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