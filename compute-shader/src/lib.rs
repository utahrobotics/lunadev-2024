#![feature(once_cell_try)]
use std::sync::{mpsc, Arc, OnceLock};

use buffers::{BufferSize, BufferSizeIter, FromBuffer, IntoBuffers};
pub use bytemuck;
use pollster::FutureExt;
pub use wgpu;
use wgpu::MapMode;

pub mod buffers;

struct GpuDevice {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

static GPU_DEVICE: OnceLock<GpuDevice> = OnceLock::new();

fn get_gpu_device() -> anyhow::Result<&'static GpuDevice> {
    GPU_DEVICE.get_or_try_init(|| {
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
            .block_on()
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
            .block_on()?;
        Ok(GpuDevice { device, queue })
    })
}

pub fn create_compute<A, V>(
    shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
    arg_sizes: A::Sizes,
    ret_size: V::Size,
    workgroup_size: (u32, u32, u32),
) -> anyhow::Result<impl FnMut(A) -> V>
where
    A: IntoBuffers,
    V: FromBuffer,
{
    let GpuDevice { device, queue } = get_gpu_device()?;
    let arg_buffers: Box<[_]> = arg_sizes
        .into_iter()
        .enumerate()
        .map(|(i, size)| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(&format!("Arg Buffer {i}")),
                size: size,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        })
        .collect();
    let return_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(&format!("Return Buffer")),
        size: ret_size.size(),
        mapped_at_creation: false,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
    });
    let return_staging_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(&format!("Return Buffer")),
        size: ret_size.size(),
        mapped_at_creation: false,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
    });
    let return_staging_buffer = Arc::new(return_staging_buffer);
    let mut state = State::new(
        shader_module_decsriptor,
        &arg_buffers,
        return_buffer,
        return_staging_buffer.clone(),
        ret_size.size(),
        workgroup_size,
    );

    Ok(move |args: A| {
        args.into_buffers(&arg_buffers, &queue);
        state.run();

        let (sender, receiver) = mpsc::sync_channel::<()>(0);
        return_staging_buffer
            .slice(..)
            .map_async(MapMode::Read, move |_| {
                let _sender = sender;
            });
        queue.submit(std::iter::empty());
        let _ = receiver.recv();

        V::from_buffer(
            &return_staging_buffer.slice(..).get_mapped_range(),
            ret_size,
        )
    })
}

struct State {
    compute_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
    return_buffer: wgpu::Buffer,
    return_staging_buffer: Arc<wgpu::Buffer>,
    return_size: u64,
    workgroup_size: (u32, u32, u32),
}

impl State {
    // Creating some of the wgpu types requires async code
    fn new(
        shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
        arg_buffers: &[wgpu::Buffer],
        return_buffer: wgpu::Buffer,
        return_staging_buffer: Arc<wgpu::Buffer>,
        return_size: u64,
        workgroup_size: (u32, u32, u32),
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
            return_staging_buffer,
            return_size,
            workgroup_size,
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
            compute_pass.dispatch_workgroups(
                self.workgroup_size.0,
                self.workgroup_size.1,
                self.workgroup_size.2,
            );
        }
        encoder.copy_buffer_to_buffer(
            &self.return_buffer,
            0,
            &self.return_staging_buffer,
            0,
            self.return_size,
        );

        let (sender, receiver) = mpsc::sync_channel::<()>(0);
        queue.on_submitted_work_done(|| {
            let _sender = sender;
        });
        queue.submit(std::iter::once(encoder.finish()));

        let _ = receiver.recv();
    }
}
