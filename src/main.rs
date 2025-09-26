use std::{env, sync::Arc};
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::{ActiveEventLoop, EventLoop},
    window::{Window, WindowId},
};
mod camera;
mod camera_controller;
mod gui;
mod pointcloud;
mod renderer;
use gui::GuiState;
use pointcloud::{PointCloud, align_to_ground, find_closest_point_to_ray, load_pcd_streaming};
use renderer::Renderer;

struct App {
    original_pointcloud: PointCloud,    // Original point cloud
    current_pointcloud: PointCloud,     // Currently rendered point cloud
    renderer: Option<Renderer>,         // Renderer instance
    window: Option<Arc<Window>>,        // Window instance
    gui_state: GuiState,                // GUI state
    egui_ctx: Option<egui::Context>,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    cursor_position: Option<(f64, f64)>,
    reload_vertices: bool,
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
            reload_vertices: false,
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

        // egui event handler
        if let (Some(ref mut egui_state), Some(ref window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window.as_ref(), &event);
        }

        // update
        if self.reload_vertices {
            if let Some(ref mut renderer) = self.renderer {
                renderer.update_point_cloud(&self.current_pointcloud);
                // Also update crop bounds in shader after alignment
                renderer.update_crop_bounds(self.gui_state.crop_min, self.gui_state.crop_max);
            }
            self.reload_vertices = false;
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

                            if let Some(closest_point) = find_closest_point_to_ray(
                                &self.current_pointcloud,
                                ray_origin,
                                ray_direction,
                                0.5, // Max distance threshold in world units
                            ) {
                                self.gui_state.set_ground_point(closest_point);
                                self.gui_state.alignment_requested = true;
                                self.gui_state.point_selection_mode = false; // Exit point selection mode
                            } else {
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
                            let camera_controller = renderer.get_camera_controller();
                            let position = camera.get_eye().to_array();
                            let target = camera.get_target().to_array();
                            let distance = camera.get_distance();
                            let (theta, phi) = camera_controller.get_angles();
                            self.gui_state
                                .update_camera_info(position, target, distance, theta, phi);

                            // Update GUI state with point cloud bounds
                            self.gui_state.update_pointcloud_bounds(
                                glam::Vec3::from(self.current_pointcloud.min_coord),
                                glam::Vec3::from(self.current_pointcloud.max_coord),
                            );

                            let raw_input = egui_state.take_egui_input(&**window);
                            let full_output = egui_ctx.run(raw_input, |ctx| {
                                self.gui_state.render(ctx);
                            });

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

                                    let aligned_pointcloud = align_to_ground(
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
                                    self.reload_vertices = true;

                                    // Progress dialog will be stopped after vertex update is complete

                                    println!("Ground plane alignment completed");
                                } else {
                                    println!("No ground point selected for alignment");
                                }
                            } // Handle reset alignment requests
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
                                self.reload_vertices = true;
                            }

                            // Handle export PCD requests
                            if self.gui_state.take_export_pcd_request() {
                                println!("Export PCD requested...");
                                
                                // Open file dialog to choose save location
                                if let Some(path) = rfd::FileDialog::new()
                                    .set_title("Save Point Cloud")
                                    .add_filter("PCD Files", &["pcd"])
                                    .set_file_name("exported_pointcloud.pcd")
                                    .save_file()
                                {
                                    println!("Saving point cloud to: {:?}", path);
                                    
                                    // Convert path to string and call export function
                                    if let Some(path_str) = path.to_str() {
                                        if let Some(_) = crate::pointcloud::export_pcd(path_str, &self.current_pointcloud) {
                                            println!("Point cloud exported successfully!");
                                        } else {
                                            println!("Failed to export point cloud");
                                        }
                                    } else {
                                        println!("Invalid file path selected");
                                    }
                                } else {
                                    println!("Export cancelled by user");
                                }
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

        // Renderer recreation is now handled at the start of frame processing
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(ref window) = self.window {
            window.request_redraw();
        }
    }
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
