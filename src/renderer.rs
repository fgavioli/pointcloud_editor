use wgpu::util::DeviceExt;
use winit::{
    event::*,
    keyboard::{Key, NamedKey},
    window::Window,
};
use crate::PointCloud;
use crate::camera::OrbitCamera;
use crate::camera_controller::{CameraController};

const Z_NEAR: f32 = 0.1;
const CAMERA_FOV: f32 = 60.0_f32.to_radians();

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl Vertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

fn get_rainbow_color(intensity: u8, min_intensity: u8, max_intensity: u8) -> [f32; 3] {
    let normalized = if max_intensity > min_intensity {
        (intensity - min_intensity) as f32 / (max_intensity - min_intensity) as f32
    } else {
        0.5
    };
    // Reversed RGB rainbow: Red -> Yellow -> Green -> Cyan -> Blue
    match (normalized * 4.0) as i32 {
        0 => [1.0, normalized * 4.0, 0.0], // Red to Yellow
        1 => [1.0 - ((normalized * 4.0) - 1.0), 1.0, 0.0], // Yellow to Green
        2 => [0.0, 1.0, (normalized * 4.0) - 2.0], // Green to Cyan
        3 => [0.0, 1.0 - ((normalized * 4.0) - 3.0), 1.0], // Cyan to Blue
        _ => [0.0, 0.0, 1.0], // Blue
    }
}

pub struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    pub size: winit::dpi::PhysicalSize<u32>,
    render_pipeline: wgpu::RenderPipeline,
    vertex_buffers: Vec<wgpu::Buffer>,
    vertex_counts: Vec<u32>,
    camera: OrbitCamera,
    camera_controller: CameraController,
    vp_mat: [[f32; 4]; 4],
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    camera_bind_group_layout: wgpu::BindGroupLayout,
    last_render_time: std::time::Instant,
    window: std::sync::Arc<Window>,
    z_far: f32,
}

impl Renderer {
    pub async fn new(window: std::sync::Arc<Window>, pointcloud: &PointCloud) -> Self {
        let size = window.inner_size();
        let initial_target = glam::Vec3::new(
            (pointcloud.max_coord[0] + pointcloud.min_coord[0]) / 2.0,
            (pointcloud.max_coord[1] + pointcloud.min_coord[1]) / 2.0,
            pointcloud.min_coord[2],
        );
        let max_extent = pointcloud.size.x.max(pointcloud.size.y).max(pointcloud.size.z);
        let z_far = max_extent * 20.0;
        // Initialize WGPU resources
        let (surface, device, queue, config) = Self::init_wgpu(window.clone(), size).await;

        // Setup camera
        let camera = OrbitCamera::new(
            config.width as f32 / config.height as f32,
            CAMERA_FOV,
            Z_NEAR,
            z_far,
        );

        // Initial camera distance is calculated as the distance at which the frustum
        // plane has a width equal to the maximum horizontal size of the model.
        // d = (modelMaxHSize / (2tan(fov))) + (horizontalSize / 2)
        let initial_distance = (pointcloud.size.x / 2.0) / (camera.get_fov() / 2.0).tan();
        let camera_controller = CameraController::new(
            max_extent * 0.1,
            initial_target,
            initial_distance,
        );

        // Setup camera uniforms
        let (vp_mat, camera_buffer, camera_bind_group, camera_bind_group_layout) =
            Self::setup_camera_uniforms(&device, &camera);

        // Create vertex data and buffers
        let vertices = Self::create_vertices_from_pointcloud(pointcloud);
        let (vertex_buffers, vertex_counts) = Self::create_vertex_buffers(&device, vertices);

        // Create render pipeline
        let render_pipeline = Self::create_render_pipeline(&device, &config, &camera_bind_group_layout);

        Self {
            surface,
            device,
            queue,
            config,
            size,
            render_pipeline,
            vertex_buffers,
            vertex_counts,
            camera,
            camera_controller,
            vp_mat,
            camera_buffer,
            camera_bind_group,
            camera_bind_group_layout,
            last_render_time: std::time::Instant::now(),
            window,
            z_far,
        }
    }

    // WGPU Initialization
    async fn init_wgpu(
        window: std::sync::Arc<Window>,
        size: winit::dpi::PhysicalSize<u32>,
    ) -> (wgpu::Surface<'static>, wgpu::Device, wgpu::Queue, wgpu::SurfaceConfiguration) {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let surface = instance.create_surface(window).unwrap();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::default(),
                },
                None,
            )
            .await
            .unwrap();

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };

        surface.configure(&device, &config);
        (surface, device, queue, config)
    }

    // Camera Setup
    fn setup_camera_uniforms(
        device: &wgpu::Device,
        camera: &OrbitCamera,
    ) -> ([[f32; 4]; 4], wgpu::Buffer, wgpu::BindGroup, wgpu::BindGroupLayout) {
        let vp_mat = camera.get_vp_matrix().to_cols_array_2d();

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[vp_mat]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("camera_bind_group_layout"),
            });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
            label: Some("camera_bind_group"),
        });

        (vp_mat, camera_buffer, camera_bind_group, camera_bind_group_layout)
    }

    // Vertex Processing
    fn create_vertices_from_pointcloud(pointcloud: &PointCloud) -> Vec<Vertex> {
        let mut vertices = Vec::with_capacity(pointcloud.x.len());

        for i in 0..pointcloud.x.len() {
            let position = [pointcloud.x[i], pointcloud.y[i], pointcloud.z[i]];
            let color = get_rainbow_color(
                pointcloud.intensity[i],
                pointcloud.min_intensity,
                pointcloud.max_intensity,
            );
            vertices.push(Vertex { position, color });
        }

        vertices
    }

    fn create_vertex_buffers(
        device: &wgpu::Device,
        vertices: Vec<Vertex>,
    ) -> (Vec<wgpu::Buffer>, Vec<u32>) {
        const MAX_BUFFER_SIZE: usize = 200_000_000;
        let vertex_size = std::mem::size_of::<Vertex>();
        let max_vertices_per_buffer = MAX_BUFFER_SIZE / vertex_size;

        let mut buffers = Vec::new();
        let mut counts = Vec::new();

        println!("Total vertices: {}", vertices.len());
        println!("Max vertices per buffer: {}", max_vertices_per_buffer);

        for chunk in vertices.chunks(max_vertices_per_buffer) {
            let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Vertex Buffer Chunk"),
                contents: bytemuck::cast_slice(chunk),
                usage: wgpu::BufferUsages::VERTEX,
            });

            buffers.push(buffer);
            counts.push(chunk.len() as u32);
        }

        println!("Created {} vertex buffers", buffers.len());
        (buffers, counts)
    }

    // Pipeline Setup
    fn create_render_pipeline(
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> wgpu::RenderPipeline {
        let shader = device.create_shader_module(wgpu::include_wgsl!("shader.wgsl"));

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::PointList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        })
    }

    // Public Interface
    pub fn window(&self) -> &Window {
        &self.window
    }

    // window resize callback
    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
            self.camera.update_proj(
                self.config.width as f32 / self.config.height as f32,
                CAMERA_FOV,
                Z_NEAR,
                self.z_far,
            );
        }
    }

    pub fn input(&mut self, event: &WindowEvent) -> bool {
        self.camera_controller.process_events(event)
    }

    pub fn update(&mut self) {
        let dt = self.last_render_time.elapsed();
        self.last_render_time = std::time::Instant::now();

        self.camera_controller.update(&mut self.camera, dt);
        self.vp_mat = self
            .camera
            .get_vp_matrix()
            .to_cols_array_2d();
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.vp_mat]),
        );
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.2,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);

            for (buffer, &count) in self.vertex_buffers.iter().zip(&self.vertex_counts) {
                render_pass.set_vertex_buffer(0, buffer.slice(..));
                render_pass.draw(0..count, 0..1);
            }
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
