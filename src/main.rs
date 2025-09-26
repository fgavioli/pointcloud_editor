use glam::{Mat3, Quat, Vec3};
use pcd_rs::DynReader;
use std::{env, io::Write, sync::Arc};
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::{ActiveEventLoop, EventLoop},
    window::{Window, WindowId},
};
mod camera;
mod camera_controller;
mod gui;
mod renderer;
use gui::GuiState;
use renderer::Renderer;

/// Find the closest point in the point cloud to a ray
fn find_closest_point_to_ray(
    pointcloud: &PointCloud,
    ray_origin: glam::Vec3,
    ray_direction: glam::Vec3,
    max_distance: f32,
) -> Option<glam::Vec3> {
    let mut closest_point = None;
    let mut min_distance = f32::MAX;

    for &point in &pointcloud.points {
        // Calculate distance from point to ray using cross product
        let to_point = point - ray_origin;
        let projection_length = to_point.dot(ray_direction);

        // Only consider points in front of the camera
        if projection_length > 0.0 {
            let projection = ray_origin + ray_direction * projection_length;
            let distance_to_ray = point.distance(projection);

            // Check if this point is closer to the ray and within max distance
            if distance_to_ray < min_distance && distance_to_ray < max_distance {
                min_distance = distance_to_ray;
                closest_point = Some(point);
            }
        }
    }

    closest_point
}

struct App {
    original_pointcloud: PointCloud, // Keep the original for reset
    current_pointcloud: PointCloud,  // Current (potentially aligned) point cloud for rendering
    renderer: Option<Renderer>,
    window: Option<Arc<Window>>,
    gui_state: GuiState,
    egui_ctx: Option<egui::Context>,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    cursor_position: Option<(f64, f64)>,
    needs_renderer_recreation: bool,
    needs_vertex_update: bool,
}

// Fit a plane to the input cloud via PCA
pub fn planefit_pca(points: &[Vec3]) -> Quat {
    if points.len() < 3 {
        return Quat::IDENTITY; // Not enough points to define a plane
    }
    let centroid = points.iter().copied().reduce(|a, b| a + b).unwrap() / (points.len() as f32);
    let mut cov = Mat3::ZERO;
    for &p in points {
        let r = p - centroid;
        cov += Mat3::from_cols(r * r.x, r * r.y, r * r.z);
    }
    cov /= points.len() as f32;

    let epsilon = 1e-6;
    let inv_cov = cov + Mat3::from_diagonal(Vec3::splat(epsilon)); // regularize
    let inv_cov = inv_cov.inverse();

    let mut normal = Vec3::new(1.0, 0.0, 0.0);
    for _ in 0..20 {
        normal = (inv_cov * normal).normalize();
    }

    // Ensure the normal points upward (positive Z direction)
    // If the normal points downward, flip it
    if normal.z < 0.0 {
        normal = -normal;
    }

    let target = Vec3::Z;
    let rot = if normal.abs_diff_eq(target, 1e-6) {
        if normal.dot(target) > 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_axis_angle(Vec3::X, std::f32::consts::PI)
        }
    } else {
        let axis = normal.cross(target).normalize();
        let angle = normal.dot(target).clamp(-1.0, 1.0).acos();
        Quat::from_axis_angle(axis, angle)
    };

    rot
}

/// Align point cloud to ground plane
fn align_pointcloud_to_ground(
    pointcloud: &PointCloud,
    ground_point: glam::Vec3,
    radius: glam::Vec3,
) -> PointCloud {
    // println!(
    //     "Fitting ground plane around {} with radius {}",
    //     ground_point, radius
    // );

    // select points in radius
    let planefit_points: Vec<glam::Vec3> = pointcloud
        .points
        .iter()
        .filter(|&point| {
            let dist = *point - ground_point;
            dist.x.abs() <= radius.x && dist.y.abs() <= radius.y && dist.z.abs() <= radius.z
        })
        .copied()
        .collect();

    let rotation = planefit_pca(&planefit_points);
    println!("Applying rotation: {}", rotation);

    // Apply rotation to all points
    let mut aligned_points = Vec::with_capacity(pointcloud.points.len());
    let mut min_coord = glam::Vec3::splat(f32::INFINITY);
    let mut max_coord = glam::Vec3::splat(f32::NEG_INFINITY);

    // Transform all points and update bounding box
    for &original_point in &pointcloud.points {
        let rotated_point = rotation * original_point;
        aligned_points.push(rotated_point);

        // Update bounding box
        min_coord = min_coord.min(rotated_point);
        max_coord = max_coord.max(rotated_point);
    }

    // Calculate 3D size
    let size = max_coord - min_coord;

    let aligned_pointcloud = PointCloud {
        points: aligned_points,
        intensity: pointcloud.intensity.clone(),
        min_coord,
        max_coord,
        size,
        min_intensity: pointcloud.min_intensity,
        max_intensity: pointcloud.max_intensity,
    };

    println!("Ground plane alignment completed");
    println!(
        "New bounds: min=[{:.2}, {:.2}, {:.2}], max=[{:.2}, {:.2}, {:.2}]",
        min_coord.x, min_coord.y, min_coord.z, max_coord.x, max_coord.y, max_coord.z
    );

    aligned_pointcloud
}

impl App {
    fn new(pointcloud: PointCloud) -> Self {
        // Clone the pointcloud for both original and current
        let original_pointcloud = PointCloud {
            points: pointcloud.points.clone(),
            intensity: pointcloud.intensity.clone(),
            min_coord: pointcloud.min_coord,
            max_coord: pointcloud.max_coord,
            size: pointcloud.size,
            min_intensity: pointcloud.min_intensity,
            max_intensity: pointcloud.max_intensity,
        };

        Self {
            original_pointcloud,
            current_pointcloud: pointcloud,
            renderer: None,
            window: None,
            gui_state: GuiState::new(),
            egui_ctx: None,
            egui_state: None,
            egui_renderer: None,
            cursor_position: None,
            needs_renderer_recreation: false,
            needs_vertex_update: false,
        }
    }

    /// Recreate the renderer with updated point cloud data
    fn recreate_renderer(&mut self) {
        if let Some(ref window) = self.window {
            println!("Recreating renderer with updated point cloud data...");
            
            // Drop the old renderer first to release GPU resources
            self.renderer = None;
            // Also drop egui renderer since it holds references to the old renderer's device
            self.egui_renderer = None;
            
            // Create new renderer with updated point cloud
            let new_renderer = pollster::block_on(Renderer::new(window.clone(), &self.current_pointcloud));
            self.renderer = Some(new_renderer);
            
            println!("Renderer recreation completed");
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window_attributes = Window::default_attributes()
            .with_title("Point Cloud Editor")
            .with_inner_size(winit::dpi::LogicalSize::new(1280, 720));

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
        let renderer = pollster::block_on(Renderer::new(window.clone(), &self.current_pointcloud));

        // Initialize egui
        let egui_ctx = egui::Context::default();
        let egui_state = egui_winit::State::new(
            egui_ctx.clone(),
            egui::viewport::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            None,
        );

        self.window = Some(window);
        self.renderer = Some(renderer);
        self.egui_ctx = Some(egui_ctx);
        self.egui_state = Some(egui_state);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        // Handle egui events first
        if let (Some(ref mut egui_state), Some(ref window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window.as_ref(), &event);
            // For now, we'll always pass events to the 3D renderer too
        }

        // Check if we need to recreate renderer at the start of frame processing
        if self.needs_renderer_recreation {
            self.recreate_renderer();
            self.needs_renderer_recreation = false;
        }

        // Check if we need to update vertex data
        if self.needs_vertex_update {
            if let Some(ref mut renderer) = self.renderer {
                renderer.update_point_cloud(&self.current_pointcloud);
                // Also update crop bounds in shader after alignment
                renderer.update_crop_bounds(
                    self.gui_state.crop_min,
                    self.gui_state.crop_max,
                );
            }
            self.needs_vertex_update = false;
            // Stop progress dialog after vertex update is complete
            self.gui_state.stop_alignment_progress();
        }

        if let Some(ref mut renderer) = self.renderer {
            // Handle point selection before other input processing
            if let (Some(ref mut egui_state), Some(ref window)) =
                (&mut self.egui_state, &self.window)
            {
                // Check if we should process this event for point selection
                let consumed_by_egui = egui_state.on_window_event(window.as_ref(), &event).consumed;

                // Track cursor movement
                if let WindowEvent::CursorMoved { position, .. } = event {
                    self.cursor_position = Some((position.x, position.y));
                }

                // Only process mouse clicks for point selection if egui didn't consume the event
                if !consumed_by_egui && self.gui_state.is_point_selection_mode() {
                    if let WindowEvent::MouseInput {
                        button: winit::event::MouseButton::Left,
                        state: ElementState::Pressed,
                        ..
                    } = event
                    {
                        // Get mouse position from stored cursor position
                        if let Some((cursor_x, cursor_y)) = self.cursor_position {
                            let screen_size = renderer.get_screen_size();
                            let camera = renderer.get_camera();

                            // Convert screen position to world ray
                            let (ray_origin, ray_direction) = camera.screen_to_world_ray(
                                (cursor_x as f32, cursor_y as f32),
                                screen_size,
                            );

                            // Find closest point to the ray
                            println!("Point selection click detected at ({}, {})", cursor_x, cursor_y);
                            if let Some(closest_point) = find_closest_point_to_ray(
                                &self.current_pointcloud,
                                ray_origin,
                                ray_direction,
                                0.5, // Max distance threshold in world units
                            ) {
                                println!("Selected ground point: ({:.2}, {:.2}, {:.2})", 
                                        closest_point.x, closest_point.y, closest_point.z);
                                self.gui_state.set_ground_point(closest_point);
                            } else {
                                println!("No point found near click position");
                                self.gui_state.set_point_selection_failed();
                            }
                        }
                        return; // Don't process this event further
                    }
                }
            }

            if !renderer.input(&event) {
                match event {
                    WindowEvent::CloseRequested
                    | WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                logical_key:
                                    winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape),
                                state: ElementState::Pressed,
                                ..
                            },
                        ..
                    } => event_loop.exit(),
                    WindowEvent::Resized(physical_size) => {
                        renderer.resize(physical_size);
                    }
                    WindowEvent::RedrawRequested => {
                        // Update renderer
                        renderer.update();

                        // Run egui and render
                        let render_result = if let (
                            Some(ref egui_ctx),
                            Some(ref mut egui_state),
                            Some(ref window),
                        ) =
                            (&self.egui_ctx, &mut self.egui_state, &self.window)
                        {
                            // Update GUI state with camera information
                            let camera = renderer.get_camera();
                            let position = camera.get_eye().to_array();
                            let target = camera.get_target().to_array();
                            let distance = camera.get_distance();
                            self.gui_state
                                .update_camera_info(position, target, distance);

                            // Update GUI state with point cloud bounds
                            self.gui_state.update_pointcloud_bounds(
                                glam::Vec3::from(self.current_pointcloud.min_coord),
                                glam::Vec3::from(self.current_pointcloud.max_coord),
                            );

                            let raw_input = egui_state.take_egui_input(&**window);
                            let full_output = egui_ctx.run(raw_input, |ctx| {
                                self.gui_state.render(ctx);
                            });

                            // Update camera controller with actual GUI panel width
                            let actual_gui_width = self.gui_state.get_actual_panel_width();
                            renderer.update_gui_width(actual_gui_width);

                            // Update renderer flags based on GUI state
                            renderer.update_render_flags(
                                self.gui_state.show_axes,
                                self.gui_state.show_target_disc,
                            );

                            // Update crop bounds based on GUI state
                            renderer.update_crop_bounds(
                                self.gui_state.crop_min,
                                self.gui_state.crop_max,
                            );

                            // Handle ground plane alignment requests
                            if self.gui_state.take_alignment_request() {
                                if let Some(ground_point) =
                                    self.gui_state.get_selected_ground_point()
                                {
                                    println!(
                                        "Aligning point cloud to ground plane at selected point..."
                                    );
                                    
                                    // Start progress dialog
                                    self.gui_state.start_alignment_progress();
                                    
                                    let aligned_pointcloud = align_pointcloud_to_ground(
                                        &self.original_pointcloud,
                                        ground_point,
                                        glam::Vec3::new(2.0, 2.0, 4.0),
                                    );
                                    self.current_pointcloud = aligned_pointcloud;

                                    // Update GUI bounds with the aligned point cloud
                                    self.gui_state.update_pointcloud_bounds(
                                        glam::Vec3::from(self.current_pointcloud.min_coord),
                                        glam::Vec3::from(self.current_pointcloud.max_coord),
                                    );

                                    // Reset crop bounds to show the full aligned point cloud
                                    self.gui_state.reset_crop_bounds_to_full_range();

                                    // Mark that we need to update vertex data
                                    self.needs_vertex_update = true;
                                    
                                    // Progress dialog will be stopped after vertex update is complete
                                    
                                    println!(
                                        "Ground plane alignment completed"
                                    );
                                } else {
                                    println!("No ground point selected for alignment");
                                }
                            }                            // Handle reset alignment requests
                            if self.gui_state.take_reset_alignment_request() {
                                println!("Resetting point cloud alignment...");
                                // Reset to original point cloud
                                self.current_pointcloud = PointCloud {
                                    points: self.original_pointcloud.points.clone(),
                                    intensity: self.original_pointcloud.intensity.clone(),
                                    min_coord: self.original_pointcloud.min_coord,
                                    max_coord: self.original_pointcloud.max_coord,
                                    size: self.original_pointcloud.size,
                                    min_intensity: self.original_pointcloud.min_intensity,
                                    max_intensity: self.original_pointcloud.max_intensity,
                                };

                                // Update GUI bounds with the original point cloud
                                self.gui_state.update_pointcloud_bounds(
                                    glam::Vec3::from(self.current_pointcloud.min_coord),
                                    glam::Vec3::from(self.current_pointcloud.max_coord),
                                );

                                // Reset crop bounds to show the full original point cloud
                                self.gui_state.reset_crop_bounds_to_full_range();

                                // Mark that we need to update vertex data
                                self.needs_vertex_update = true;
                                println!("Point cloud alignment reset completed");
                            }

                            // Initialize egui renderer if needed
                            if self.egui_renderer.is_none() {
                                println!("Creating new egui renderer...");
                                self.egui_renderer = Some(egui_wgpu::Renderer::new(
                                    renderer.get_device(),
                                    renderer.get_config().format,
                                    None,
                                    1,
                                    false,
                                ));
                                println!("Egui renderer created successfully");
                            }

                            let render_result =
                                if let Some(ref mut egui_renderer) = self.egui_renderer {
                                    renderer.render_with_egui(egui_renderer, egui_ctx, &full_output)
                                } else {
                                    renderer.render()
                                };

                            egui_state
                                .handle_platform_output(&**window, full_output.platform_output);
                            render_result
                        } else {
                            renderer.render()
                        };

                        match render_result {
                            Ok(_) => {}
                            Err(wgpu::SurfaceError::Lost) => renderer.resize(renderer.size),
                            Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                            Err(e) => eprintln!("{:?}", e),
                        }
                    }
                    _ => {}
                }
            }
        }

        // Renderer recreation is now handled at the start of frame processing
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(ref window) = self.window {
            window.request_redraw();
        }
    }
}

#[derive(Clone)]
struct PointCloud {
    points: Vec<glam::Vec3>,
    intensity: Vec<u8>,
    min_coord: glam::Vec3,
    max_coord: glam::Vec3,
    size: glam::Vec3, // 3D size (extent) of the point cloud
    min_intensity: u8,
    max_intensity: u8,
}

fn load_pcd_streaming(filename: &str) -> Option<PointCloud> {
    let start = std::time::Instant::now();
    let reader = match DynReader::open(filename).ok() {
        Some(r) => r,
        None => {
            println!("{} not found.", filename);
            return None;
        }
    };

    // Check if we can get the total number of points from metadata
    let total_points = reader.meta().width * reader.meta().height;
    // Discover schema at runtime
    let schema = reader.meta().field_defs.fields.clone();
    let x_index = match schema.iter().position(|f| f.name.to_lowercase() == "x") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the x field.");
            return None;
        }
    };
    let y_index = match schema.iter().position(|f| f.name.to_lowercase() == "y") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the y field.");
            return None;
        }
    };
    let z_index = match schema.iter().position(|f| f.name.to_lowercase() == "z") {
        Some(idx) => idx,
        None => {
            eprintln!("PCD file schema does not contain the z field.");
            return None;
        }
    };
    let mut has_intensity = true;
    let intensity_index = match schema
        .iter()
        .position(|f| f.name.to_lowercase() == "intensity")
    {
        Some(idx) => idx,
        None => {
            println!(
                "WARN: PCD file schema does not contain the intensity field. Defaulting to 0."
            );
            has_intensity = false;
            0
        }
    };

    let mut pointcloud = PointCloud {
        points: Vec::new(),
        intensity: Vec::new(),
        min_coord: glam::Vec3::splat(f32::INFINITY),
        max_coord: glam::Vec3::splat(f32::NEG_INFINITY),
        size: glam::Vec3::ZERO,
        min_intensity: 0u8,
        max_intensity: 255u8,
    };

    // Progress tracking variables
    let mut points_loaded = 0usize;
    let update_interval = (total_points as usize) / 100;
    let mut last_progress_update = std::time::Instant::now();

    println!("Opened {}.", filename);
    println!(
        "Available fields: {:?}",
        schema.iter().map(|f| f.name.clone()).collect::<Vec<_>>()
    );

    // Print initial progress bar
    print!("Loading: [");
    for _ in 0..25 {
        print!(" ");
    }
    print!("] 0%\r");
    std::io::stdout().flush().unwrap();

    for record in reader {
        // Access by index~
        let point = match record {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Error reading record: {:?}", e);
                continue;
            }
        };
        let field_x = &point.0[x_index];
        let field_y = &point.0[y_index];
        let field_z = &point.0[z_index];
        let field_intensity = &point.0[intensity_index];

        // Extract specific types

        let x_val = if let pcd_rs::Field::F32(ref values) = field_x {
            values[0]
        } else {
            eprintln!("Expected F32 for x field. Found different type.");
            continue;
        };
        let y_val = if let pcd_rs::Field::F32(ref values) = field_y {
            values[0]
        } else {
            eprintln!("Expected F32 for y field. Found different type.");
            continue;
        };
        let z_val = if let pcd_rs::Field::F32(ref values) = field_z {
            values[0]
        } else {
            eprintln!("Expected F32 for z field. Found different type.");
            continue;
        };
        pointcloud.points.push(glam::Vec3::new(x_val, y_val, z_val));

        if has_intensity {
            pointcloud
                .intensity
                .push(if let pcd_rs::Field::F32(ref values) = field_intensity {
                    // Normalize intensity to 0-255 range
                    values[0] as u8
                } else {
                    0u8
                });
        } else {
            pointcloud.intensity.push(0u8);
        }

        // Update progress tracking
        points_loaded += 1;

        // Update progress bar periodically
        if points_loaded % update_interval == 0 || last_progress_update.elapsed().as_millis() > 100
        {
            let progress_percent = ((points_loaded as f64 / total_points as f64) * 100.0) as usize;
            let progress_percent = progress_percent.min(100);
            let filled_bars = (progress_percent * 25) / 100;

            print!("Loading : [");
            for i in 0..25 {
                if i < filled_bars {
                    print!("=");
                } else if i == filled_bars {
                    print!(">");
                } else {
                    print!(" ");
                }
            }
            print!("] {}% ({} points)\r", progress_percent, points_loaded);
            std::io::stdout().flush().unwrap();
            last_progress_update = std::time::Instant::now();
        }
    }

    // Clear the progress line
    print!("\r");
    for _ in 0..80 {
        print!(" ");
    }
    print!("\r");

    // Compute bounding box
    pointcloud.min_coord = pointcloud
        .points
        .iter()
        .fold(glam::Vec3::splat(f32::INFINITY), |min, p| min.min(*p));
    pointcloud.max_coord = pointcloud
        .points
        .iter()
        .fold(glam::Vec3::splat(f32::NEG_INFINITY), |max, p| max.max(*p));

    // Calculate 3D size (extent) of the point cloud
    pointcloud.size = pointcloud.max_coord - pointcloud.min_coord;

    // rescale intensity to 0-255
    if has_intensity {
        let (min_intensity, max_intensity) = pointcloud
            .intensity
            .iter()
            .fold((u8::MAX, u8::MIN), |(min, max), &val| {
                (min.min(val), max.max(val))
            });
        pointcloud.intensity = pointcloud
            .intensity
            .iter()
            .map(|&val| {
                ((val - min_intensity) as f32 / (max_intensity - min_intensity) as f32 * 255.0)
                    as u8
            })
            .collect();
    }

    let load_time = start.elapsed().as_millis();
    println!(
        "[Load done, {} ms : {} points]",
        load_time,
        pointcloud.points.len()
    );

    Some(pointcloud)
}

fn main() {
    env_logger::init();

    let mut filename = String::new();
    if env::args().len() > 2 {
        println!("Usage: pointcloud_editor [path_to_pcd_file]");
        return;
    }
    if env::args().len() > 1 {
        let arg_filename = env::args().nth(1).unwrap();
        if !arg_filename.is_empty() {
            filename = arg_filename;
        }
    } else {
        // Default to a sample file if no argument provided
        filename = "samples/kaist03_000020.pcd".to_string();
    }

    let pointcloud = load_pcd_streaming(&filename);
    if pointcloud.is_none() {
        println!("Failed to load point cloud");
        return;
    }
    let pointcloud = pointcloud.unwrap();

    println!("Starting renderer...");
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::new(pointcloud);
    event_loop.run_app(&mut app).unwrap();
}
