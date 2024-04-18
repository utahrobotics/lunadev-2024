#![feature(once_cell_try)]
use std::sync::{Arc, Mutex};

use buffers::{BufferSize, BufferSizeIter, FromBuffer, IntoBuffer, IntoBuffers};
pub use bytemuck;
use crossbeam::{queue::SegQueue, utils::Backoff};
use tokio::sync::{oneshot, OnceCell};
pub use wgpu;
use wgpu::MapMode;

pub mod buffers;

struct GpuDevice {
    device: wgpu::Device,
    queue: Mutex<wgpu::Queue>,
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
            Ok(GpuDevice {
                device,
                queue: Mutex::new(queue),
            })
        })
        .await
}

pub async fn create_compute<A, V>(
    shader_module_decsriptor: wgpu::ShaderModuleDescriptor<'_>,
    arg_sizes: A::Sizes,
    return_size: V::Size,
    workgroup_size: (u32, u32, u32),
) -> anyhow::Result<Compute<A, V>>
where
    A: IntoBuffers,
    V: FromBuffer,
{
    let GpuDevice { device, .. } = get_gpu_device().await?;
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

    let state = State::new(
        shader_module_decsriptor,
        &arg_buffers,
        return_size.size(),
        workgroup_size,
        device,
    )
    .await;

    Ok(Compute {
        return_recv: Arc::default(),
        return_size,
        arg_buffers,
        state,
        phantom: std::marker::PhantomData,
    })
}

pub struct Compute<A, V: FromBuffer> {
    return_recv: Arc<SegQueue<V::Receiver>>,
    return_size: V::Size,
    arg_buffers: Box<[wgpu::Buffer]>,
    state: State,
    phantom: std::marker::PhantomData<A>,
}

impl<A: IntoBuffers, V: FromBuffer> Compute<A, V> {
    pub fn provide_return_recv(&mut self, recv: V::Receiver) {
        self.return_recv.push(recv);
    }

    async fn call_inner(&self, args: A) -> V {
        let (sender, receiver) = oneshot::channel();
        {
            let GpuDevice { queue, device } = get_gpu_device().await.unwrap();
            let queue_guard = queue.lock().unwrap();
            // println!("lock_buf");
            args.into_buffers(&self.arg_buffers, &queue_guard);
            // println!("unlock_buf");
            let ret_size = self.return_size;
            let recv = self.return_recv.pop().unwrap_or_default();
            self.state.run(
                move |buffer, buffer_queue| {
                    let mut buffer = Arc::new(buffer);
                    let buffer2 = buffer.clone();
                    buffer.slice(..).map_async(MapMode::Read, move |_| {
                        let _ = sender.send(V::from_buffer(
                            recv,
                            &buffer2.slice(..).get_mapped_range(),
                            ret_size,
                        ));
                        buffer2.unmap();
                    });
                    // println!("lock_queue");
                    let idx = queue.lock().unwrap().submit(std::iter::empty());
                    // println!("unlock_queue");
                    device.poll(wgpu::MaintainBase::WaitForSubmissionIndex(idx));
                    let backoff = Backoff::new();
                    loop {
                        match Arc::try_unwrap(buffer) {
                            Ok(x) => {
                                buffer_queue.push(x);
                                break;
                            }
                            Err(e) => {
                                buffer = e;
                                backoff.snooze();
                                continue;
                            }
                        }
                    }
                },
                device,
                &queue_guard,
            );
        }
        receiver.await.unwrap()
    }
}

impl<T: IntoBuffer, V: FromBuffer> Compute<(T,), V> {
    pub async fn call(&self, arg: T) -> V {
        self.call_inner((arg,)).await
    }
}

impl<T: IntoBuffer, T1: IntoBuffer, V: FromBuffer> Compute<(T, T1), V> {
    pub async fn call(&self, arg1: T, arg2: T1) -> V {
        self.call_inner((arg1, arg2)).await
    }
}

struct State {
    compute_pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
    return_buffer: wgpu::Buffer,
    return_staging_buffers: Arc<SegQueue<wgpu::Buffer>>,
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
            return_staging_buffers: Arc::default(),
            return_size,
            workgroup_size,
        }
    }

    fn run(
        &self,
        callback: impl FnOnce(wgpu::Buffer, Arc<SegQueue<wgpu::Buffer>>) + Send + 'static,
        device: &'static wgpu::Device,
        queue: &wgpu::Queue,
    ) {
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
        let return_staging_buffer = self.return_staging_buffers.pop().unwrap_or_else(|| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(&format!("Return Buffer")),
                size: self.return_size,
                mapped_at_creation: false,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            })
        });
        encoder.copy_buffer_to_buffer(
            &self.return_buffer,
            0,
            &return_staging_buffer,
            0,
            self.return_size,
        );
        let return_staging_buffers = self.return_staging_buffers.clone();

        let idx = queue.submit(std::iter::once(encoder.finish()));
        rayon::spawn(move || {
            device.poll(wgpu::MaintainBase::WaitForSubmissionIndex(idx));
            callback(return_staging_buffer, return_staging_buffers);
        });
    }
}
