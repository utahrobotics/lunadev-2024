use std::sync::RwLock;

use buffers::{
    BufferDestination, BufferSized, BufferSource, BufferType, HostReadableWritable, ShaderWritable, UniformOrStorage, ValidBufferType
};
use crossbeam::queue::SegQueue;
use futures::FutureExt;
use fxhash::FxHashMap;
use tokio::sync::OnceCell;
use wgpu::{util::StagingBelt, BindGroupLayoutEntry, CommandEncoder};

pub use bytemuck;
pub mod buffers;
pub use wgpu;

#[cfg(test)]
mod tests;

struct GpuDevice {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

static GPU_DEVICE: OnceCell<GpuDevice> = OnceCell::const_new();

async fn get_gpu_device() -> anyhow::Result<&'static GpuDevice> {
    GPU_DEVICE
        .get_or_try_init(|| async {
            // The instance is a handle to our GPU
            // Backends::all => Vulkan + Metal + DX12 + Browser WebGPU
            let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
                backends: wgpu::Backends::all(),
                ..Default::default()
            });

            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::default(),
                    compatible_surface: None,
                    force_fallback_adapter: false,
                })
                .await
                .ok_or_else(|| anyhow::anyhow!("Failed to request adapter"))?;

            let (device, queue) = adapter
                .request_device(
                    &wgpu::DeviceDescriptor {
                        required_features: wgpu::Features::empty(),
                        // WebGL doesn't support all of wgpu's features, so if
                        // we're building for the web, we'll have to disable some.
                        required_limits: if cfg!(target_arch = "wasm32") {
                            wgpu::Limits::downlevel_webgl2_defaults()
                        } else {
                            wgpu::Limits::default()
                        },
                        label: None,
                    },
                    None, // Trace path
                )
                .await?;
            Ok(GpuDevice { device, queue })
        })
        .await
}

pub struct Compute<A> {
    arg_buffers: Box<[wgpu::Buffer]>,

    staging_belt_size: u64,
    staging_belts: SegQueue<StagingBelt>,
    compute_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,

    buffers: RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,

    phantom: std::marker::PhantomData<A>,
}

impl<A> Compute<A> {
    fn new_inner(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        device: &wgpu::Device,
        arg_buffers: Box<[wgpu::Buffer]>,
        entries: Box<[BindGroupLayoutEntry]>,
        staging_belt_size: u64,
    ) -> Self {
        let module = device.create_shader_module(shader_module_decsriptor);

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &entries,
            label: Some("bind_group_layout"),
        });

        let entries: Box<[_]> = arg_buffers
            .iter()
            .enumerate()
            .map(|(i, buf)| wgpu::BindGroupEntry {
                binding: i as u32,
                resource: wgpu::BindingResource::Buffer(buf.as_entire_buffer_binding()),
            })
            .collect();

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &entries,
            label: Some("bind_group"),
        });

        let compute_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Compute Pipeline Layout"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        let compute_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Compute Pipeline"),
            layout: Some(&compute_pipeline_layout),
            module: &module,
            entry_point: "main",
        });

        Self {
            staging_belts: SegQueue::new(),
            staging_belt_size,
            arg_buffers,
            compute_pipeline,
            bind_group,
            buffers: RwLock::new(FxHashMap::default()),
            phantom: std::marker::PhantomData,
        }
    }

    fn new_pass_inner(
        &self,
        into_buffer: impl FnOnce(&mut CommandEncoder, &[wgpu::Buffer], &mut StagingBelt, &wgpu::Device),
    ) -> ComputePass<A> {
        let GpuDevice { device, .. } = get_gpu_device().now_or_never().unwrap().unwrap();

        let mut command_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        let mut stager = self
            .staging_belts
            .pop()
            .unwrap_or_else(|| StagingBelt::new(self.staging_belt_size));
        into_buffer(&mut command_encoder, &self.arg_buffers, &mut stager, device);
        ComputePass {
            command_encoder,
            compute: self,
            stager,
            workgroup_size: (1, 1, 1),
        }
    }

    pub async fn write_args_inner(
        &self,
        into_buffer: impl FnOnce(&mut CommandEncoder, &[wgpu::Buffer], &mut StagingBelt, &wgpu::Device),
    ) {
        let GpuDevice { device, queue } = get_gpu_device().now_or_never().unwrap().unwrap();
        let ComputePass {
            mut stager,
            command_encoder,
            ..
        } = self.new_pass_inner(into_buffer);
        stager.finish();
        let idx = queue.submit(std::iter::once(command_encoder.finish()));
        stager.recall();

        self.staging_belts.push(stager);

        tokio::task::spawn_blocking(|| {
            device.poll(wgpu::MaintainBase::WaitForSubmissionIndex(idx));
        })
        .await
        .unwrap();
    }
}

impl<T1, H1, S1, O1> Compute<(BufferType<T1, H1, S1, O1>,)>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    H1: HostReadableWritable,
    S1: ShaderWritable,
    O1: UniformOrStorage,
{
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        type1: BufferType<T1, H1, S1, O1>,
    ) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;

        Ok(Self::new_inner(
            shader_module_decsriptor,
            device,
            Box::new([type1.into_buffer(0, device)]),
            Box::new([type1.into_layout(0)]),
            [type1.size()].into_iter().max().unwrap(),
        ))
    }

    pub fn new_pass(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
    ) -> ComputePass<(BufferType<T1, H1, S1, O1>,)> {
        self.new_pass_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
        })
    }

    pub async fn write_args(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
    ) {
        self.write_args_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
        })
        .await;
    }
}

impl<T1, H1, S1, O1, T2, H2, S2, O2> Compute<(BufferType<T1, H1, S1, O1>, BufferType<T2, H2, S2, O2>)>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    H1: HostReadableWritable,
    S1: ShaderWritable,
    O1: UniformOrStorage,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    T2: BufferSized + 'static,
    H2: HostReadableWritable,
    S2: ShaderWritable,
    O2: UniformOrStorage,
{
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        type1: BufferType<T1, H1, S1, O1>,
        type2: BufferType<T2, H2, S2, O2>,
    ) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;

        Ok(Self::new_inner(
            shader_module_decsriptor,
            device,
            Box::new([type1.into_buffer(0, device), type2.into_buffer(1, device)]),
            Box::new([type1.into_layout(0), type2.into_layout(1)]),
            [type1.size(), type2.size()].into_iter().max().unwrap(),
        ))
    }

    pub fn new_pass(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
    ) -> ComputePass<(BufferType<T1, H1, S1, O1>, BufferType<T2, H2, S2, O2>)> {
        self.new_pass_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
        })
    }

    pub async fn write_args(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
    ) {
        self.write_args_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
        })
        .await;
    }
}

impl<T1, H1, S1, O1, T2, H2, S2, O2, T3, H3, S3, O3>
    Compute<(
        BufferType<T1, H1, S1, O1>,
        BufferType<T2, H2, S2, O2>,
        BufferType<T3, H3, S3, O3>,
    )>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    H1: HostReadableWritable,
    S1: ShaderWritable,
    O1: UniformOrStorage,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    T2: BufferSized + 'static,
    H2: HostReadableWritable,
    S2: ShaderWritable,
    O2: UniformOrStorage,
    BufferType<T3, H3, S3, O3>: ValidBufferType,
    T3: BufferSized + 'static,
    H3: HostReadableWritable,
    S3: ShaderWritable,
    O3: UniformOrStorage,
{
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        type1: BufferType<T1, H1, S1, O1>,
        type2: BufferType<T2, H2, S2, O2>,
        type3: BufferType<T3, H3, S3, O3>,
    ) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;

        Ok(Self::new_inner(
            shader_module_decsriptor,
            device,
            Box::new([
                type1.into_buffer(0, device),
                type2.into_buffer(1, device),
                type3.into_buffer(2, device),
            ]),
            Box::new([
                type1.into_layout(0),
                type2.into_layout(1),
                type3.into_layout(2),
            ]),
            [type1.size(), type2.size(), type3.size()]
                .into_iter()
                .max()
                .unwrap(),
        ))
    }

    pub fn new_pass(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
        arg3: impl BufferSource<<BufferType<T3, H3, S3, O3> as ValidBufferType>::WriteType>,
    ) -> ComputePass<(
        BufferType<T1, H1, S1, O1>,
        BufferType<T2, H2, S2, O2>,
        BufferType<T3, H3, S3, O3>,
    )> {
        self.new_pass_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
            arg3.into_buffer(command_encoder, &arg_buffers[2], stager, device);
        })
    }

    pub async fn write_args(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
        arg3: impl BufferSource<<BufferType<T3, H3, S3, O3> as ValidBufferType>::WriteType>,
    ) {
        self.write_args_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
            arg3.into_buffer(command_encoder, &arg_buffers[2], stager, device);
        })
        .await;
    }
}

impl<T1, H1, S1, O1, T2, H2, S2, O2, T3, H3, S3, O3, T4, H4, S4, O4>
    Compute<(
        BufferType<T1, H1, S1, O1>,
        BufferType<T2, H2, S2, O2>,
        BufferType<T3, H3, S3, O3>,
        BufferType<T4, H4, S4, O4>,
    )>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    H1: HostReadableWritable,
    S1: ShaderWritable,
    O1: UniformOrStorage,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    T2: BufferSized + 'static,
    H2: HostReadableWritable,
    S2: ShaderWritable,
    O2: UniformOrStorage,
    BufferType<T3, H3, S3, O3>: ValidBufferType,
    T3: BufferSized + 'static,
    H3: HostReadableWritable,
    S3: ShaderWritable,
    O3: UniformOrStorage,
    BufferType<T4, H4, S4, O4>: ValidBufferType,
    T4: BufferSized + 'static,
    H4: HostReadableWritable,
    S4: ShaderWritable,
    O4: UniformOrStorage,
{
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        type1: BufferType<T1, H1, S1, O1>,
        type2: BufferType<T2, H2, S2, O2>,
        type3: BufferType<T3, H3, S3, O3>,
        type4: BufferType<T4, H4, S4, O4>,
    ) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;

        Ok(Self::new_inner(
            shader_module_decsriptor,
            device,
            Box::new([
                type1.into_buffer(0, device),
                type2.into_buffer(1, device),
                type3.into_buffer(2, device),
                type4.into_buffer(3, device),
            ]),
            Box::new([
                type1.into_layout(0),
                type2.into_layout(1),
                type3.into_layout(2),
                type4.into_layout(3),
            ]),
            [type1.size(), type2.size(), type3.size(), type4.size()]
                .into_iter()
                .max()
                .unwrap(),
        ))
    }

    pub fn new_pass(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
        arg3: impl BufferSource<<BufferType<T3, H3, S3, O3> as ValidBufferType>::WriteType>,
        arg4: impl BufferSource<<BufferType<T4, H4, S4, O4> as ValidBufferType>::WriteType>,
    ) -> ComputePass<(
        BufferType<T1, H1, S1, O1>,
        BufferType<T2, H2, S2, O2>,
        BufferType<T3, H3, S3, O3>,
        BufferType<T4, H4, S4, O4>,
    )> {
        self.new_pass_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
            arg3.into_buffer(command_encoder, &arg_buffers[2], stager, device);
            arg4.into_buffer(command_encoder, &arg_buffers[3], stager, device);
        })
    }

    pub async fn write_args(
        &self,
        arg1: impl BufferSource<<BufferType<T1, H1, S1, O1> as ValidBufferType>::WriteType>,
        arg2: impl BufferSource<<BufferType<T2, H2, S2, O2> as ValidBufferType>::WriteType>,
        arg3: impl BufferSource<<BufferType<T3, H3, S3, O3> as ValidBufferType>::WriteType>,
        arg4: impl BufferSource<<BufferType<T4, H4, S4, O4> as ValidBufferType>::WriteType>,
    ) {
        self.write_args_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
            arg2.into_buffer(command_encoder, &arg_buffers[1], stager, device);
            arg3.into_buffer(command_encoder, &arg_buffers[2], stager, device);
            arg4.into_buffer(command_encoder, &arg_buffers[3], stager, device);
        })
        .await;
    }
}

pub struct ComputePass<'a, A> {
    command_encoder: CommandEncoder,
    compute: &'a Compute<A>,
    stager: StagingBelt,
    pub workgroup_size: (u32, u32, u32),
}

impl<'a, A> ComputePass<'a, A> {
    async fn call_inner<T>(
        mut self,
        after_dispatch: impl FnOnce(
            &mut CommandEncoder,
            &[wgpu::Buffer],
            &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
            &wgpu::Device,
        ) -> T,
    ) -> (
        &'static wgpu::Device,
        &'a RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>,
        T,
    ) {
        let GpuDevice { queue, device } = get_gpu_device().await.unwrap();
        let Compute {
            compute_pipeline,
            bind_group,
            arg_buffers,
            buffers,
            staging_belts,
            ..
        } = self.compute;

        self.stager.finish();
        {
            let mut compute_pass =
                self.command_encoder
                    .begin_compute_pass(&wgpu::ComputePassDescriptor {
                        label: Some("Render Pass"),
                        timestamp_writes: None,
                    });

            compute_pass.set_pipeline(compute_pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);
            compute_pass.dispatch_workgroups(
                self.workgroup_size.0,
                self.workgroup_size.1,
                self.workgroup_size.2,
            );
        }

        let state = after_dispatch(&mut self.command_encoder, arg_buffers, &buffers, device);

        let idx = queue.submit(std::iter::once(self.command_encoder.finish()));
        self.stager.recall();
        staging_belts.push(self.stager);

        tokio::task::spawn_blocking(|| {
            device.poll(wgpu::MaintainBase::WaitForSubmissionIndex(idx));
        })
        .await
        .unwrap();

        (device, buffers, state)
    }

    pub fn workgroup_size(mut self, x: u32, y: u32, z: u32) -> Self {
        self.workgroup_size = (x, y, z);
        self
    }
}

impl<'a, T1, H1, S1, O1> ComputePass<'a, (BufferType<T1, H1, S1, O1>,)>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
{
    pub async fn call(
        self,
        mut arg1: impl BufferDestination<<BufferType<T1, H1, S1, O1> as ValidBufferType>::ReadType>,
    ) {
        let (device, buffers, (state1,)) = self
            .call_inner(|command_encoder, arg_buffers, buffers, device| {
                (arg1.enqueue(command_encoder, &arg_buffers[0], &buffers, device),)
            })
            .await;

        arg1.from_buffer(state1, device, buffers).await;
    }
}

impl<'a, T1, H1, S1, O1, T2, H2, S2, O2> ComputePass<'a, (BufferType<T1, H1, S1, O1>, BufferType<T2, H2, S2, O2>)>
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    T2: ?Sized + BufferSized + 'static,
{
    pub async fn call(
        self,
        mut arg1: impl BufferDestination<<BufferType<T1, H1, S1, O1> as ValidBufferType>::ReadType>,
        mut arg2: impl BufferDestination<<BufferType<T2, H2, S2, O2> as ValidBufferType>::ReadType>,
    ) {
        let (device, buffers, (state1, state2)) = self
            .call_inner(|command_encoder, arg_buffers, buffers, device| {
                (
                    arg1.enqueue(command_encoder, &arg_buffers[0], &buffers, device),
                    arg2.enqueue(command_encoder, &arg_buffers[1], buffers, device),
                )
            })
            .await;

        arg1.from_buffer(state1, device, buffers).await;
        arg2.from_buffer(state2, device, buffers).await;
    }
}

impl<'a, T1, H1, S1, O1, T2, H2, S2, O2, T3, H3, S3, O3>
    ComputePass<
        'a,
        (
            BufferType<T1, H1, S1, O1>,
            BufferType<T2, H2, S2, O2>,
            BufferType<T3, H3, S3, O3>,
        ),
    >
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    BufferType<T3, H3, S3, O3>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    T2: ?Sized + BufferSized + 'static,
    T3: ?Sized + BufferSized + 'static,
{
    pub async fn call(
        self,
        mut arg1: impl BufferDestination<<BufferType<T1, H1, S1, O1> as ValidBufferType>::ReadType>,
        mut arg2: impl BufferDestination<<BufferType<T2, H2, S2, O2> as ValidBufferType>::ReadType>,
        mut arg3: impl BufferDestination<<BufferType<T3, H3, S3, O3> as ValidBufferType>::ReadType>,
    ) {
        let (device, buffers, (state1, state2, state3)) = self
            .call_inner(|command_encoder, arg_buffers, buffers, device| {
                (
                    arg1.enqueue(command_encoder, &arg_buffers[0], &buffers, device),
                    arg2.enqueue(command_encoder, &arg_buffers[1], &buffers, device),
                    arg3.enqueue(command_encoder, &arg_buffers[2], &buffers, device),
                )
            })
            .await;

        arg1.from_buffer(state1, device, buffers).await;
        arg2.from_buffer(state2, device, buffers).await;
        arg3.from_buffer(state3, device, buffers).await;
    }
}

impl<'a, T1, H1, S1, O1, T2, H2, S2, O2, T3, H3, S3, O3, T4, H4, S4, O4>
    ComputePass<
        'a,
        (
            BufferType<T1, H1, S1, O1>,
            BufferType<T2, H2, S2, O2>,
            BufferType<T3, H3, S3, O3>,
            BufferType<T4, H4, S4, O4>,
        ),
    >
where
    BufferType<T1, H1, S1, O1>: ValidBufferType,
    BufferType<T2, H2, S2, O2>: ValidBufferType,
    BufferType<T3, H3, S3, O3>: ValidBufferType,
    BufferType<T4, H4, S4, O4>: ValidBufferType,
    T1: ?Sized + BufferSized + 'static,
    T2: ?Sized + BufferSized + 'static,
    T3: ?Sized + BufferSized + 'static,
    T4: ?Sized + BufferSized + 'static,
{
    pub async fn call(
        self,
        mut arg1: impl BufferDestination<<BufferType<T1, H1, S1, O1> as ValidBufferType>::ReadType>,
        mut arg2: impl BufferDestination<<BufferType<T2, H2, S2, O2> as ValidBufferType>::ReadType>,
        mut arg3: impl BufferDestination<<BufferType<T3, H3, S3, O3> as ValidBufferType>::ReadType>,
        mut arg4: impl BufferDestination<<BufferType<T4, H4, S4, O4> as ValidBufferType>::ReadType>,
    ) {
        let (device, buffers, (state1, state2, state3, state4)) = self
            .call_inner(|command_encoder, arg_buffers, buffers, device| {
                (
                    arg1.enqueue(command_encoder, &arg_buffers[0], &buffers, device),
                    arg2.enqueue(command_encoder, &arg_buffers[1], &buffers, device),
                    arg3.enqueue(command_encoder, &arg_buffers[2], &buffers, device),
                    arg4.enqueue(command_encoder, &arg_buffers[3], &buffers, device),
                )
            })
            .await;

        arg1.from_buffer(state1, device, buffers).await;
        arg2.from_buffer(state2, device, buffers).await;
        arg3.from_buffer(state3, device, buffers).await;
        arg4.from_buffer(state4, device, buffers).await;
    }
}
