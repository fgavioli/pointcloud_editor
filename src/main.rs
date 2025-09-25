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

struct App {
    original_pointcloud: PointCloud, // Keep the original for reset
    current_pointcloud: PointCloud,  // Current (potentially aligned) point cloud for rendering
    renderer: Option<Renderer>,
    window: Option<Arc<Window>>,
    gui_state: GuiState,
    egui_ctx: Option<egui::Context>,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
}

/// Align point cloud to ground plane
fn align_pointcloud_to_ground(pointcloud: &PointCloud) -> PointCloud {
    println!("Detecting ground plane...");

    // build histogram by discretizing Z values in 1m bins
    let bin_size = 0.5; // 1 meter bins
    let mut histogram = std::collections::HashMap::new();
    for &z in &pointcloud.z {
        let bin = (z / bin_size).floor() as i32;
        *histogram.entry(bin).or_insert(0) += 1;
    }
    // print the ordered histogram (for debugging)
    let mut ordered_histogram: Vec<_> = histogram.into_iter().collect();
    ordered_histogram.sort_by_key(|&(bin, _)| bin);
    for (bin, count) in ordered_histogram {
        println!("Bin {} [Z={:.1}-{}]:\t{}", bin, bin as f32 * bin_size, (bin + 1) as f32 * bin_size, count);
    }
    // Find the bin with the maximum count


    // println!("Ground plane detected with {} inliers ({:.1}% of points)", 
    //          best_inlier_count, (best_inlier_count as f32 / num_points as f32) * 100.0);
    // println!("Ground plane normal: [{:.3}, {:.3}, {:.3}]", 
    //          ground_normal.x, ground_normal.y, ground_normal.z);
    
    // // Target normal (Z-up)
    // let target_normal = glam::Vec3::new(0.0, 0.0, 1.0);
    
    // // Calculate rotation axis and angle
    // let rotation_axis = ground_normal.cross(target_normal);
    // let rotation_angle = ground_normal.dot(target_normal).acos();
    
    // // Handle case where normals are already aligned or opposite
    // let rotation_matrix = if rotation_axis.length() < 1e-6 {
    //     if ground_normal.dot(target_normal) > 0.0 {
    //         glam::Mat3::IDENTITY // Already aligned
    //     } else {
    //         glam::Mat3::from_rotation_x(std::f32::consts::PI) // Flip 180 degrees
    //     }
    // } else {
    //     let rotation_axis = rotation_axis.normalize();
    //     glam::Mat3::from_axis_angle(rotation_axis, rotation_angle)
    // };
    
    // println!("Applying rotation (angle: {:.2}°)", rotation_angle.to_degrees());
    
    // // Apply rotation to all points
    // let mut aligned_pointcloud = PointCloud {
    //     x: Vec::with_capacity(num_points),
    //     y: Vec::with_capacity(num_points),
    //     z: Vec::with_capacity(num_points),
    //     intensity: pointcloud.intensity.clone(),
    //     min_coord: [f32::INFINITY; 3],
    //     max_coord: [f32::NEG_INFINITY; 3],
    //     size: glam::Vec3::ZERO,
    //     min_intensity: pointcloud.min_intensity,
    //     max_intensity: pointcloud.max_intensity,
    // };
    
    // // Transform all points and update bounding box
    // for i in 0..num_points {
    //     let original_point = glam::Vec3::new(pointcloud.x[i], pointcloud.y[i], pointcloud.z[i]);
    //     let rotated_point = rotation_matrix * original_point;
        
    //     aligned_pointcloud.x.push(rotated_point.x);
    //     aligned_pointcloud.y.push(rotated_point.y);
    //     aligned_pointcloud.z.push(rotated_point.z);
        
    //     // Update bounding box
    //     aligned_pointcloud.min_coord[0] = aligned_pointcloud.min_coord[0].min(rotated_point.x);
    //     aligned_pointcloud.min_coord[1] = aligned_pointcloud.min_coord[1].min(rotated_point.y);
    //     aligned_pointcloud.min_coord[2] = aligned_pointcloud.min_coord[2].min(rotated_point.z);
        
    //     aligned_pointcloud.max_coord[0] = aligned_pointcloud.max_coord[0].max(rotated_point.x);
    //     aligned_pointcloud.max_coord[1] = aligned_pointcloud.max_coord[1].max(rotated_point.y);
    //     aligned_pointcloud.max_coord[2] = aligned_pointcloud.max_coord[2].max(rotated_point.z);
    // }
    
    // // Calculate 3D size
    // aligned_pointcloud.size = glam::Vec3::new(
    //     aligned_pointcloud.max_coord[0] - aligned_pointcloud.min_coord[0],
    //     aligned_pointcloud.max_coord[1] - aligned_pointcloud.min_coord[1],
    //     aligned_pointcloud.max_coord[2] - aligned_pointcloud.min_coord[2],
    // );
    
    // println!("Ground plane alignment completed");
    // println!("New bounds: min=[{:.2}, {:.2}, {:.2}], max=[{:.2}, {:.2}, {:.2}]",
    //          aligned_pointcloud.min_coord[0], aligned_pointcloud.min_coord[1], aligned_pointcloud.min_coord[2],
    //          aligned_pointcloud.max_coord[0], aligned_pointcloud.max_coord[1], aligned_pointcloud.max_coord[2]);
    //
    pointcloud.clone() // Placeholder: return original point cloud for now
}

impl App {
    fn new(pointcloud: PointCloud) -> Self {
        // Clone the pointcloud for both original and current
        let original_pointcloud = PointCloud {
            x: pointcloud.x.clone(),
            y: pointcloud.y.clone(),
            z: pointcloud.z.clone(),
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

        if let Some(ref mut renderer) = self.renderer {
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
                                println!("Aligning point cloud to ground plane...");
                                let aligned_pointcloud = align_pointcloud_to_ground(&self.original_pointcloud);
                                self.current_pointcloud = aligned_pointcloud;
                                
                                // Update GUI bounds with the aligned point cloud
                                self.gui_state.update_pointcloud_bounds(
                                    glam::Vec3::from(self.current_pointcloud.min_coord),
                                    glam::Vec3::from(self.current_pointcloud.max_coord),
                                );
                                
                                // TODO: Need to recreate renderer with new point cloud data
                                println!("Ground plane alignment completed (placeholder implementation)");
                            }

                            // Handle reset alignment requests
                            if self.gui_state.take_reset_alignment_request() {
                                println!("Resetting point cloud alignment...");
                                // Reset to original point cloud
                                self.current_pointcloud = PointCloud {
                                    x: self.original_pointcloud.x.clone(),
                                    y: self.original_pointcloud.y.clone(),
                                    z: self.original_pointcloud.z.clone(),
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
                                
                                // TODO: Need to recreate renderer with original point cloud data
                                println!("Point cloud alignment reset completed");
                            }

                            // Initialize egui renderer if needed
                            if self.egui_renderer.is_none() {
                                self.egui_renderer = Some(egui_wgpu::Renderer::new(
                                    renderer.get_device(),
                                    renderer.get_config().format,
                                    None,
                                    1,
                                    false,
                                ));
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
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(ref window) = self.window {
            window.request_redraw();
        }
    }
}

#[derive(Clone)]
struct PointCloud {
    // Structure of Arrays (SoA) for better memory efficiency
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    intensity: Vec<u8>,
    min_coord: [f32; 3],
    max_coord: [f32; 3],
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
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        intensity: Vec::new(),
        min_coord: [0.0, 0.0, 0.0],
        max_coord: [0.0, 0.0, 0.0],
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
        pointcloud
            .x
            .push(if let pcd_rs::Field::F32(ref values) = field_x {
                values[0]
            } else {
                eprintln!("Expected F32 for x field. Found different type.");
                continue;
            });
        pointcloud
            .y
            .push(if let pcd_rs::Field::F32(ref values) = field_y {
                values[0]
            } else {
                eprintln!("Expected F32 for y field. Found different type.");
                continue;
            });
        pointcloud
            .z
            .push(if let pcd_rs::Field::F32(ref values) = field_z {
                values[0]
            } else {
                eprintln!("Expected F32 for z field. Found different type.");
                continue;
            });
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
    pointcloud.min_coord = [
        pointcloud.x.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        pointcloud.y.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
        pointcloud.z.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
    ];
    pointcloud.max_coord = [
        pointcloud
            .x
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
        pointcloud
            .y
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
        pointcloud
            .z
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b)),
    ];

    // Calculate 3D size (extent) of the point cloud
    pointcloud.size = glam::Vec3::new(
        pointcloud.max_coord[0] - pointcloud.min_coord[0],
        pointcloud.max_coord[1] - pointcloud.min_coord[1],
        pointcloud.max_coord[2] - pointcloud.min_coord[2],
    );

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
        pointcloud.x.len()
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
