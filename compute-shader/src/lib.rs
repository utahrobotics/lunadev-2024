use std::sync::Arc;

use buffers::{BufferSize, BufferSizeIter, FromBuffer, IntoBuffer, IntoBuffers, ReturnBuffer};
use crossbeam::queue::SegQueue;
use tokio::sync::{oneshot, OnceCell};
pub use wgpu;
use wgpu::util::StagingBelt;

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
                        features: wgpu::Features::empty(),
                        // WebGL doesn't support all of wgpu's features, so if
                        // we're building for the web, we'll have to disable some.
                        limits: if cfg!(target_arch = "wasm32") {
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

pub struct Compute<A, V: ?Sized> {
    arg_buffers: Box<[wgpu::Buffer]>,
    staging_belt_size: u64,
    state: State,
    staging_belts: Arc<SegQueue<StagingBelt>>,
    phantom: std::marker::PhantomData<(A, V)>,
}

impl<A: IntoBuffers, V: FromBuffer + ?Sized> Compute<A, V> {
    pub async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        arg_sizes: A::Sizes,
        return_size: V::Size,
        workgroup_size: (u32, u32, u32),
    ) -> anyhow::Result<Self> {
        let GpuDevice { device, .. } = get_gpu_device().await?;
        let arg_buffers: Box<[_]> = arg_sizes
            .into_iter()
            .enumerate()
            .map(|(i, size)| {
                device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some(&format!("Arg Buffer {i}")),
                    size: size,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                })
            })
            .collect();

        let staging_belt_size = arg_sizes.into_iter().max().unwrap() as u64;

        let state = State::new(
            shader_module_decsriptor,
            &arg_buffers,
            return_size.size(),
            workgroup_size,
            device,
        )
        .await;

        Ok(Self {
            staging_belts: Arc::default(),
            staging_belt_size,
            arg_buffers,
            state,
            phantom: std::marker::PhantomData,
        })
    }

    async fn call_inner(&self, args: A) -> ReturnBuffer<V> {
        let GpuDevice { queue, device } = get_gpu_device().await.unwrap();

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        let mut stager = self
            .staging_belts
            .pop()
            .unwrap_or_else(|| StagingBelt::new(self.staging_belt_size));
        args.into_buffers(&mut encoder, &self.arg_buffers, &mut stager, device);
        stager.finish();
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Render Pass"),
                // timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.state.compute_pipeline);
            compute_pass.set_bind_group(0, &self.state.bind_group, &[]);
            compute_pass.dispatch_workgroups(
                self.state.workgroup_size.0,
                self.state.workgroup_size.1,
                self.state.workgroup_size.2,
            );
        }
        let return_staging_buffer = self.state.return_staging_buffers.pop().unwrap_or_else(|| {
            Arc::new(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(&format!("Return Buffer")),
                size: self.state.return_size,
                mapped_at_creation: false,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            }))
        });
        encoder.copy_buffer_to_buffer(
            &self.state.return_buffer,
            0,
            &return_staging_buffer,
            0,
            self.state.return_size,
        );
        let return_staging_buffers = self.state.return_staging_buffers.clone();

        let idx = queue.submit(std::iter::once(encoder.finish()));
        stager.recall();
        self.staging_belts.push(stager);
        let (sender, receiver) = oneshot::channel();
        rayon::spawn(move || {
            device.poll(wgpu::MaintainBase::WaitForSubmissionIndex(idx));
            V::from_buffer(
                return_staging_buffer,
                device,
                return_staging_buffers,
                sender,
            );
        });
        receiver.await.unwrap()
    }
}

impl<T: IntoBuffer, V: FromBuffer + ?Sized> Compute<(T,), V> {
    pub async fn call(&self, arg: T) -> ReturnBuffer<V> {
        self.call_inner((arg,)).await
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, V: FromBuffer + ?Sized> Compute<(T, T1), V> {
    pub async fn call(&self, arg1: T, arg2: T1) -> ReturnBuffer<V> {
        self.call_inner((arg1, arg2)).await
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, T2: IntoBuffer, V: FromBuffer + ?Sized>
    Compute<(T, T1, T2), V>
{
    pub async fn call(&self, arg1: T, arg2: T1, arg3: T2) -> ReturnBuffer<V> {
        self.call_inner((arg1, arg2, arg3)).await
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, T2: IntoBuffer, T3: IntoBuffer, V: FromBuffer + ?Sized>
    Compute<(T, T1, T2, T3), V>
{
    pub async fn call(&self, arg1: T, arg2: T1, arg3: T2, arg4: T3) -> ReturnBuffer<V> {
        self.call_inner((arg1, arg2, arg3, arg4)).await
    }
}

impl<
        T: IntoBuffer,
        T1: IntoBuffer,
        T2: IntoBuffer,
        T3: IntoBuffer,
        T4: IntoBuffer,
        V: FromBuffer + ?Sized,
    > Compute<(T, T1, T2, T3, T4), V>
{
    pub async fn call(&self, arg1: T, arg2: T1, arg3: T2, arg4: T3, arg5: T4) -> ReturnBuffer<V> {
        self.call_inner((arg1, arg2, arg3, arg4, arg5)).await
    }
}

struct State {
    compute_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
    return_buffer: wgpu::Buffer,
    return_staging_buffers: Arc<SegQueue<Arc<wgpu::Buffer>>>,
    return_size: u64,
    workgroup_size: (u32, u32, u32),
}

impl State {
    // Creating some of the wgpu types requires async code
    async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        arg_buffers: &[wgpu::Buffer],
        return_size: u64,
        workgroup_size: (u32, u32, u32),
        device: &wgpu::Device,
    ) -> Self {
        let module = device.create_shader_module(shader_module_decsriptor);

        let return_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(&format!("Return Buffer")),
            size: return_size,
            mapped_at_creation: false,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        });

        let entries: Box<[_]> = std::iter::once(wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .chain(
            arg_buffers
                .iter()
                .enumerate()
                .map(|(i, _)| wgpu::BindGroupLayoutEntry {
                    binding: i as u32 + 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }),
        )
        .collect();

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &entries,
            label: Some("bind_group_layout"),
        });

        let entries: Box<[_]> = std::iter::once(wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::Buffer(return_buffer.as_entire_buffer_binding()),
        })
        .chain(
            arg_buffers
                .iter()
                .enumerate()
                .map(|(i, buf)| wgpu::BindGroupEntry {
                    binding: i as u32 + 1,
                    resource: wgpu::BindingResource::Buffer(buf.as_entire_buffer_binding()),
                }),
        )
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
            compute_pipeline,
            bind_group,
            return_buffer,
            return_staging_buffers: Arc::default(),
            return_size,
            workgroup_size,
        }
    }
}
