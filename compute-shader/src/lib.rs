#![feature(once_cell_try)]
use std::{
    mem::size_of,
    sync::{mpsc, OnceLock},
};

use buffers::IntoBuffers;
pub use bytemuck;
use bytemuck::bytes_of_mut;
pub use wgpu;
use wgpu::util::DeviceExt;

pub mod buffers;

struct GpuDevice {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

static GPU_DEVICE: OnceLock<GpuDevice> = OnceLock::new();

fn get_gpu_device() -> anyhow::Result<&'static GpuDevice> {
    GPU_DEVICE.get_or_try_init(|| {
        let (sender, receiver) = mpsc::sync_channel::<anyhow::Result<GpuDevice>>(0);
        tokio::spawn(async move {
            let f = || async {
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
            };
            let _ = sender.send(f().await);
        });
        receiver.recv().unwrap()
    })
}

pub async fn create_compute<A, V>(
    shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
) -> anyhow::Result<impl FnMut(A) -> V>
where
    A: IntoBuffers,
    V: bytemuck::Pod,
{
    let GpuDevice { device, .. } = get_gpu_device()?;
    let arg_buffers: Box<[_]> = A::sizes()
        .enumerate()
        .map(|(i, size)| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(&format!("Arg Buffer{i}")),
                contents: &vec![0; size].into_boxed_slice(),
                usage: wgpu::BufferUsages::UNIFORM,
            })
        })
        .collect();
    let return_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(&format!("Return Buffer")),
        contents: &vec![0; size_of::<V>()].into_boxed_slice(),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let mut state = State::new(shader_module_decsriptor, &arg_buffers, &return_buffer).await;

    Ok(move |args: A| {
        args.into_buffers(&arg_buffers);
        state.run();
        let mut return_val = V::zeroed();
        let return_val_bytes = bytes_of_mut(&mut return_val);
        return_val_bytes.copy_from_slice(&return_buffer.slice(..).get_mapped_range());
        return_val
    })
}

struct State {
    compute_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
}

impl State {
    // Creating some of the wgpu types requires async code
    async fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        arg_buffers: &[wgpu::Buffer],
        return_buffer: &wgpu::Buffer,
    ) -> Self {
        let GpuDevice { device, .. } = get_gpu_device().unwrap();

        let module = device.create_shader_module(shader_module_decsriptor);

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
                        ty: wgpu::BufferBindingType::Uniform,
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
            label: Some("diffuse_bind_group"),
        });

        let compute_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Compute Pipeline Layout"),
                bind_group_layouts: &[],
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
        }
    }

    fn run(&mut self) {
        let GpuDevice { device, queue } = GPU_DEVICE.get().unwrap();
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        });
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Render Pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.compute_pipeline);
            compute_pass.set_bind_group(0, &self.bind_group, &[]);
        }

        queue.submit(std::iter::once(encoder.finish()));
        let (sender, receiver) = mpsc::sync_channel::<()>(0);
        queue.on_submitted_work_done(|| {
            let _sender = sender;
        });

        let _ = receiver.recv();
    }
}
