// use std::sync::Arc;

use std::sync::RwLock;

use buffers::{
    BufferDestination, BufferSized, BufferSource, BufferType, HostReadableWritable, ShaderWritable,
    ValidBufferType,
};
use crossbeam::queue::SegQueue;
use futures::FutureExt;
use fxhash::FxHashMap;
// use buffers::{BufferSize, BufferSizeIter, FromBuffer, IntoBuffer, IntoBuffers, ReturnBuffer};
// use crossbeam::queue::SegQueue;
use tokio::sync::OnceCell;
use wgpu::{util::StagingBelt, BindGroupLayoutEntry, CommandEncoder};
// pub use wgpu;
// use wgpu::util::StagingBelt;

pub mod buffers;

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
        after_dispatch: impl FnOnce(&mut CommandEncoder, &[wgpu::Buffer], &mut StagingBelt, &wgpu::Device)
    ) -> ComputePass<A> {
        let GpuDevice { device, .. } = get_gpu_device().now_or_never().unwrap().unwrap();

        let mut command_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        let mut stager = self
            .staging_belts
            .pop()
            .unwrap_or_else(|| StagingBelt::new(self.staging_belt_size));
        after_dispatch(
            &mut command_encoder,
            &self.arg_buffers,
            &mut stager,
            device,
        );
        ComputePass {
            command_encoder,
            compute: self,
            stager,
            workgroup_size: (1, 1, 1),
        }
    }
}

impl<T1, H1, S1> Compute<(BufferType<T1, H1, S1>,)>
where
    BufferType<T1, H1, S1>: ValidBufferType,
    T1: BufferSized + 'static,
    H1: HostReadableWritable,
    S1: ShaderWritable,
{
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        type1: BufferType<T1, H1, S1>,
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
        arg1: impl BufferSource<<BufferType<T1, H1, S1> as ValidBufferType>::WriteType>,
    ) -> ComputePass<(BufferType<T1, H1, S1>,)> {
        self.new_pass_inner(|command_encoder, arg_buffers, stager, device| {
            arg1.into_buffer(command_encoder, &arg_buffers[0], stager, device);
        })
    }
}

pub struct ComputePass<'a, A> {
    command_encoder: CommandEncoder,
    compute: &'a Compute<A>,
    stager: StagingBelt,
    pub workgroup_size: (u32, u32, u32),
}

impl<'a, A> ComputePass<'a, A> {
    async fn call_inner<T>(mut self, after_dispatch: impl FnOnce(&mut CommandEncoder, &[wgpu::Buffer], &RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>, &wgpu::Device) -> T) -> (&'static wgpu::Device, &'a RwLock<FxHashMap<u64, SegQueue<wgpu::Buffer>>>, T) {
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
}

impl<'a, T1, H1, S1> ComputePass<'a, (BufferType<T1, H1, S1>,)>
where
    BufferType<T1, H1, S1>: ValidBufferType,
    T1: 'static,
{
    pub async fn call(
        self,
        mut arg1: impl BufferDestination<<BufferType<T1, H1, S1> as ValidBufferType>::ReadType>,
    ) {
        let (device, buffers, (state1,)) = self.call_inner(|command_encoder, arg_buffers, buffers, device| {
            (arg1.enqueue(command_encoder, &arg_buffers[0], &buffers, device),)
        }).await;

        arg1.from_buffer(state1, device, buffers).await;
    }
}
