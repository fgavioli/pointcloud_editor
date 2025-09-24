use wgpu::util::DeviceExt;
use winit::{
    event::*,
    keyboard::{Key, NamedKey},
    window::Window,
};
use crate::PointCloud;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

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

// ============================================================================
// CAMERA SYSTEM
// ============================================================================

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
    fn new(aspect: f32, fovy: f32, znear: f32, zfar: f32) -> Self {
        Self {
            eye: glam::Vec3::ZERO,
            target: glam::Vec3::ZERO,
            up: glam::Vec3::Z,
            aspect,
            fovy,
            znear,
            zfar,
        }
    }

    pub fn build_view_projection_matrix(&self) -> glam::Mat4 {
        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(self.fovy, self.aspect, self.znear, self.zfar);
        proj * view
    }

    fn update_aspect(&mut self, aspect: f32) {
        self.aspect = aspect;
    }
}

struct InputState {
    // Keyboard states
    is_left_pressed: bool,
    is_right_pressed: bool,
    is_up_pressed: bool,
    is_down_pressed: bool,
    is_pan_up_pressed: bool,
    is_pan_down_pressed: bool,
    // Mouse states
    is_mouse_pressed: bool,
    is_right_mouse_pressed: bool,
    last_mouse_pos: glam::Vec2,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            is_left_pressed: false,
            is_right_pressed: false,
            is_up_pressed: false,
            is_down_pressed: false,
            is_pan_up_pressed: false,
            is_pan_down_pressed: false,
            is_mouse_pressed: false,
            is_right_mouse_pressed: false,
            last_mouse_pos: glam::Vec2::ZERO,
        }
    }
}

pub struct CameraController {
    // Configuration
    speed: f32,
    sensitivity: f32,
    zoom_speed: f32,
    // Input state
    input: InputState,
    // Orbit parameters
    pub distance: f32,
    pub theta: f32, // Horizontal angle (azimuth)
    pub phi: f32,   // Vertical angle (elevation)
    pub target: glam::Vec3, // Center point to orbit around
}

impl CameraController {
    fn new(speed: f32, target: glam::Vec3, initial_distance: f32) -> Self {
        Self {
            speed,
            sensitivity: 0.005,
            zoom_speed: 0.1,
            input: InputState::default(),
            distance: initial_distance,
            theta: 0.0_f32.to_radians(),
            phi: -89.0_f32.to_radians(),
            target,
        }
    }

    fn handle_keyboard_input(&mut self, key: &Key, is_pressed: bool) -> bool {
        match key {
            Key::Named(NamedKey::ArrowUp) => {
                self.input.is_up_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "w" => {
                self.input.is_up_pressed = is_pressed;
                true
            }
            Key::Named(NamedKey::ArrowDown) => {
                self.input.is_down_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "s" => {
                self.input.is_down_pressed = is_pressed;
                true
            }
            Key::Named(NamedKey::ArrowLeft) => {
                self.input.is_left_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "a" => {
                self.input.is_left_pressed = is_pressed;
                true
            }
            Key::Named(NamedKey::ArrowRight) => {
                self.input.is_right_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "d" => {
                self.input.is_right_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "q" => {
                self.input.is_pan_up_pressed = is_pressed;
                true
            }
            Key::Character(s) if s.as_str() == "e" => {
                self.input.is_pan_down_pressed = is_pressed;
                true
            }
            _ => false,
        }
    }

    fn handle_mouse_input(&mut self, state: ElementState, button: MouseButton) -> bool {
        let is_pressed = state == ElementState::Pressed;
        match button {
            MouseButton::Left => {
                self.input.is_mouse_pressed = is_pressed;
                true
            }
            MouseButton::Right => {
                self.input.is_right_mouse_pressed = is_pressed;
                true
            }
            _ => false,
        }
    }

    fn handle_cursor_moved(&mut self, new_pos: glam::Vec2) -> bool {
        if self.input.is_mouse_pressed {
            let delta = new_pos - self.input.last_mouse_pos;
            self.theta -= delta.x * self.sensitivity;
            self.phi -= delta.y * self.sensitivity;
            self.phi = self.phi.clamp(-std::f32::consts::FRAC_PI_2 + 0.1, std::f32::consts::FRAC_PI_2 - 0.1);
            self.input.last_mouse_pos = new_pos;
            true
        } else if self.input.is_right_mouse_pressed {
            let delta = new_pos - self.input.last_mouse_pos;
            let zoom_amount = -delta.y * self.sensitivity * self.distance * 0.5;
            self.distance = (self.distance + zoom_amount).clamp(0.1, f32::MAX);
            self.input.last_mouse_pos = new_pos;
            true
        } else {
            self.input.last_mouse_pos = new_pos;
            false
        }
    }

    fn handle_mouse_wheel(&mut self, delta: &MouseScrollDelta) {
        let base_zoom = match delta {
            MouseScrollDelta::LineDelta(_, y) => -y * self.zoom_speed,
            MouseScrollDelta::PixelDelta(pos) => -pos.y as f32 * 0.01 * self.zoom_speed,
        };
        let zoom_amount = base_zoom * self.distance;
        self.distance = (self.distance + zoom_amount).clamp(0.1, f32::MAX);
    }

    pub fn process_events(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput { event: KeyEvent { logical_key: key, state, .. }, .. } => {
                self.handle_keyboard_input(key, *state == ElementState::Pressed)
            }
            WindowEvent::MouseInput { state, button, .. } => {
                self.handle_mouse_input(*state, *button)
            }
            WindowEvent::CursorMoved { position, .. } => {
                let new_pos = glam::Vec2::new(position.x as f32, position.y as f32);
                self.handle_cursor_moved(new_pos)
            }
            WindowEvent::MouseWheel { delta, .. } => {
                self.handle_mouse_wheel(delta);
                true
            }
            _ => false,
        }
    }

    fn update_distance(&mut self, dt: f32) {
        if self.input.is_up_pressed {
            self.distance -= self.zoom_speed * dt * 10.0;
        }
        if self.input.is_down_pressed {
            self.distance += self.zoom_speed * dt * 10.0;
        }
        self.distance = self.distance.clamp(0.1, f32::MAX);
    }

    fn update_target(&mut self, dt: f32) {
        let camera_vectors = self.calculate_camera_vectors();
        let mut target_movement = glam::Vec3::ZERO;

        if self.input.is_right_pressed { target_movement -= camera_vectors.right; }
        if self.input.is_left_pressed { target_movement += camera_vectors.right; }
        if self.input.is_pan_up_pressed { target_movement += camera_vectors.up; }
        if self.input.is_pan_down_pressed { target_movement -= camera_vectors.up; }

        self.target += target_movement * self.speed * dt;
    }

    fn calculate_camera_vectors(&self) -> CameraVectors {
        let forward = glam::Vec3::new(
            self.theta.cos() * self.phi.cos(),
            self.theta.sin() * self.phi.cos(),
            self.phi.sin(),
        ).normalize();
        let right = forward.cross(glam::Vec3::Z).normalize();
        let up = glam::Vec3::Z;

        CameraVectors { forward, right, up }
    }

    fn update_camera_position(&self, camera: &mut Camera) {
        let base_forward = glam::Vec3::new(0.0, -1.0, 0.0);
        let yaw_rotation = glam::Mat3::from_rotation_z(self.theta);
        let yawed_forward = yaw_rotation * base_forward;
        let right_vector = yawed_forward.cross(glam::Vec3::Z).normalize();
        let pitch_rotation = glam::Mat3::from_axis_angle(right_vector, self.phi);
        let final_forward = pitch_rotation * yawed_forward;

        camera.eye = self.target - final_forward * self.distance;
        camera.target = self.target;
        camera.up = glam::Vec3::Z;
    }

    pub fn update_camera(&mut self, camera: &mut Camera, dt: std::time::Duration) {
        let dt = dt.as_secs_f32();
        self.update_distance(dt);
        self.update_target(dt);
        self.update_camera_position(camera);
    }
}

struct CameraVectors {
    forward: glam::Vec3,
    right: glam::Vec3,
    up: glam::Vec3,
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
    camera: Camera,
    camera_controller: CameraController,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    camera_bind_group_layout: wgpu::BindGroupLayout,
    last_render_time: std::time::Instant,
    window: std::sync::Arc<Window>,
}

impl Renderer {
    pub async fn new(window: std::sync::Arc<Window>, pointcloud: &PointCloud) -> Self {
        let size = window.inner_size();
        let target = glam::Vec3::new(
            (pointcloud.max_coord[0] + pointcloud.min_coord[0]) / 2.0,
            (pointcloud.max_coord[1] + pointcloud.min_coord[1]) / 2.0,
            pointcloud.min_coord[2],
        );
        let max_extent = pointcloud.size.x.max(pointcloud.size.y).max(pointcloud.size.z);

        // Initialize WGPU resources
        let (surface, device, queue, config) = Self::init_wgpu(window.clone(), size).await;

        // Setup camera
        let mut camera = Camera::new(
            config.width as f32 / config.height as f32,
            60.0_f32.to_radians(),
            0.1,
            max_extent * 20.0,
        );

        // Position camera above point cloud
        camera.eye = glam::Vec3::new(0.0, 0.0, pointcloud.max_coord[2] + max_extent * 0.5);
        camera.target = target;

        // Initial camera distance is calculated as the distance at which the frustum
        // plane has a width equal to the maximum horizontal size of the model.
        // d = (modelMaxHSize / (2tan(fov))) + (horizontalSize / 2)
        let initial_camera_distance = (pointcloud.size.x / 2.0) / (camera.fovy / 2.0).tan();
        let camera_controller = CameraController::new(
            max_extent * 0.1,
            target,
            initial_camera_distance,
        );

        // Setup camera uniforms
        let (camera_uniform, camera_buffer, camera_bind_group, camera_bind_group_layout) = 
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
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            camera_bind_group_layout,
            last_render_time: std::time::Instant::now(),
            window,
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
        camera: &Camera,
    ) -> (CameraUniform, wgpu::Buffer, wgpu::BindGroup, wgpu::BindGroupLayout) {
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

        (camera_uniform, camera_buffer, camera_bind_group, camera_bind_group_layout)
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

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
            self.camera.update_aspect(self.config.width as f32 / self.config.height as f32);
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
