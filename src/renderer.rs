use wgpu::util::DeviceExt;
use winit::{
    event::*,
    keyboard::{Key, NamedKey},
    window::Window,
};

use crate::PointCloud;

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

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

pub struct Camera {
    pub eye: glam::Vec3,
    pub target: glam::Vec3,
    pub up: glam::Vec3,
    pub aspect: f32,
    pub fovy: f32,
    pub znear: f32,
    pub zfar: f32,
}

impl Camera {
    pub fn build_view_projection_matrix(&self) -> glam::Mat4 {
        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(self.fovy, self.aspect, self.znear, self.zfar);
        proj * view
    }
}

pub struct CameraController {
    pub speed: f32,
    pub sensitivity: f32,
    pub is_forward_pressed: bool,
    pub is_backward_pressed: bool,
    pub is_left_pressed: bool,
    pub is_right_pressed: bool,
    pub is_up_pressed: bool,
    pub is_down_pressed: bool,
    pub is_mouse_pressed: bool,
    pub last_mouse_pos: glam::Vec2,
}

impl CameraController {
    pub fn new(speed: f32) -> Self {
        Self {
            speed,
            sensitivity: 0.005,
            is_forward_pressed: false,
            is_backward_pressed: false,
            is_left_pressed: false,
            is_right_pressed: false,
            is_up_pressed: false,
            is_down_pressed: false,
            is_mouse_pressed: false,
            last_mouse_pos: glam::Vec2::ZERO,
        }
    }

    pub fn process_events(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key: key,
                        state,
                        ..
                    },
                ..
            } => {
                let is_pressed = *state == ElementState::Pressed;
                match key {
                    Key::Named(NamedKey::ArrowUp) => {
                        self.is_forward_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "w" => {
                        self.is_forward_pressed = is_pressed;
                        true
                    }
                    Key::Named(NamedKey::ArrowDown) => {
                        self.is_backward_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "s" => {
                        self.is_backward_pressed = is_pressed;
                        true
                    }
                    Key::Named(NamedKey::ArrowLeft) => {
                        self.is_left_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "a" => {
                        self.is_left_pressed = is_pressed;
                        true
                    }
                    Key::Named(NamedKey::ArrowRight) => {
                        self.is_right_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "d" => {
                        self.is_right_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "q" => {
                        self.is_up_pressed = is_pressed;
                        true
                    }
                    Key::Character(s) if s == "e" => {
                        self.is_down_pressed = is_pressed;
                        true
                    }
                    _ => false,
                }
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                self.is_mouse_pressed = *state == ElementState::Pressed;
                true
            }
            WindowEvent::CursorMoved { position, .. } => {
                let new_pos = glam::Vec2::new(position.x as f32, position.y as f32);
                if self.is_mouse_pressed {
                    let _delta = new_pos - self.last_mouse_pos; // Keep for future use
                    self.last_mouse_pos = new_pos;
                    // We'll handle the delta in update_camera
                    true
                } else {
                    self.last_mouse_pos = new_pos;
                    false
                }
            }
            _ => false,
        }
    }

    pub fn update_camera(&mut self, camera: &mut Camera, dt: std::time::Duration) {
        let dt = dt.as_secs_f32();

        // Handle keyboard movement - pan camera and target together
        let forward = (camera.target - camera.eye).normalize();
        let right = forward.cross(camera.up).normalize();
        let up = camera.up;

        let mut movement = glam::Vec3::ZERO;
        if self.is_forward_pressed {
            movement += forward;
        }
        if self.is_backward_pressed {
            movement -= forward;
        }
        if self.is_right_pressed {
            movement += right;
        }
        if self.is_left_pressed {
            movement -= right;
        }
        if self.is_up_pressed {
            movement += up;
        }
        if self.is_down_pressed {
            movement -= up;
        }

        movement *= self.speed * dt;
        camera.eye += movement;
        camera.target += movement;
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
    camera: Camera,
    camera_controller: CameraController,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    last_render_time: std::time::Instant,
    window: std::sync::Arc<Window>,
}

impl Renderer {
    pub async fn new(window: std::sync::Arc<Window>, pointcloud: &PointCloud) -> Self {
        let size = window.inner_size();

        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();

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

        // Create camera - since we normalize the point cloud to fit in a 2x2x2 cube centered at origin
        // we can set up the camera to view this normalized space
        let center = glam::Vec3::ZERO; // Point cloud is centered at origin after normalization
        let normalized_extent = 2.0; // Points are scaled to fit in [-1, 1] range

        // Position camera to get a good view of the normalized point cloud
        let camera_distance = normalized_extent * 2.5; // Move camera back to see the whole cloud
        let camera_height = normalized_extent * 0.5; // Slightly above for a better angle

        let camera = Camera {
            eye: glam::Vec3::new(camera_distance, camera_height, camera_distance),
            target: center,
            up: glam::Vec3::Y,
            aspect: config.width as f32 / config.height as f32,
            fovy: 60.0_f32.to_radians(), // Wider field of view for better visibility
            znear: 0.1,
            zfar: normalized_extent * 20.0,
        };

        let camera_controller = CameraController::new(normalized_extent * 0.5);

        let mut camera_uniform = CameraUniform {
            view_proj: [[0.0; 4]; 4],
        };
        camera_uniform.view_proj = camera.build_view_projection_matrix().to_cols_array_2d();

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
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

        // Convert point cloud to vertices with rainbow colors and split into chunks
        let vertices = Self::create_vertices_from_pointcloud(pointcloud);
        let (vertex_buffers, vertex_counts) = Self::create_vertex_buffers(&device, vertices);

        let shader = device.create_shader_module(wgpu::include_wgsl!("shader.wgsl"));

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
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
        });

        Self {
            window,
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
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            last_render_time: std::time::Instant::now(),
        }
    }

    fn create_vertex_buffers(
        device: &wgpu::Device,
        vertices: Vec<Vertex>,
    ) -> (Vec<wgpu::Buffer>, Vec<u32>) {
        const MAX_BUFFER_SIZE: usize = 200_000_000; // 200MB to be safe
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

    fn create_vertices_from_pointcloud(pointcloud: &PointCloud) -> Vec<Vertex> {
        let mut vertices = Vec::new();

        // Normalize coordinates to fit in the view
        let center = [
            (pointcloud.min_coord[0] + pointcloud.max_coord[0]) * 0.5,
            (pointcloud.min_coord[1] + pointcloud.max_coord[1]) * 0.5,
            (pointcloud.min_coord[2] + pointcloud.max_coord[2]) * 0.5,
        ];
        let extent = [
            pointcloud.max_coord[0] - pointcloud.min_coord[0],
            pointcloud.max_coord[1] - pointcloud.min_coord[1],
            pointcloud.max_coord[2] - pointcloud.min_coord[2],
        ];
        let max_extent = extent[0].max(extent[1].max(extent[2]));
        let scale = if max_extent > 0.0 {
            2.0 / max_extent
        } else {
            1.0
        };

        for i in 0..pointcloud.x.len() {
            let position = [
                (pointcloud.x[i] - center[0]) * scale,
                (pointcloud.y[i] - center[1]) * scale,
                (pointcloud.z[i] - center[2]) * scale,
            ];
            // Create rainbow color based on intensity (already normalized to 0-255)
            let color = Self::intensity_to_rainbow_color(
                pointcloud.intensity[i],
                pointcloud.min_intensity,
                pointcloud.max_intensity,
            );

            vertices.push(Vertex { position, color });
        }

        vertices
    }

    fn intensity_to_rainbow_color(
        intensity: u8,
        min_intensity: u8,
        max_intensity: u8,
    ) -> [f32; 3] {
        // Normalize intensity to [0, 1]
        let normalized = if max_intensity > min_intensity {
            (intensity - min_intensity) as f32 / (max_intensity - min_intensity) as f32
        } else {
            0.5 // Default to middle if no intensity variation
        };

        // Create rainbow color: Red -> Yellow -> Green -> Cyan -> Blue -> Magenta -> Red
        // We'll use Red -> Green -> Blue progression for simplicity
        let hue = normalized * 240.0; // 0 to 240 degrees (red to blue in HSV)

        Self::hsv_to_rgb(hue, 1.0, 1.0)
    }

    fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [f32; 3] {
        let c = v * s;
        let h_prime = h / 60.0;
        let x = c * (1.0 - ((h_prime % 2.0) - 1.0).abs());
        let m = v - c;

        let (r_prime, g_prime, b_prime) = if h_prime >= 0.0 && h_prime < 1.0 {
            (c, x, 0.0)
        } else if h_prime >= 1.0 && h_prime < 2.0 {
            (x, c, 0.0)
        } else if h_prime >= 2.0 && h_prime < 3.0 {
            (0.0, c, x)
        } else if h_prime >= 3.0 && h_prime < 4.0 {
            (0.0, x, c)
        } else if h_prime >= 4.0 && h_prime < 5.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };

        [r_prime + m, g_prime + m, b_prime + m]
    }

    pub fn window(&self) -> &Window {
        &self.window
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
            self.camera.aspect = self.config.width as f32 / self.config.height as f32;
        }
    }

    pub fn input(&mut self, event: &WindowEvent) -> bool {
        self.camera_controller.process_events(event)
    }

    pub fn update(&mut self) {
        let now = std::time::Instant::now();
        let dt = now - self.last_render_time;
        self.last_render_time = now;

        self.camera_controller.update_camera(&mut self.camera, dt);
        self.camera_uniform.view_proj = self
            .camera
            .build_view_projection_matrix()
            .to_cols_array_2d();
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
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

            // Render each vertex buffer chunk
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
