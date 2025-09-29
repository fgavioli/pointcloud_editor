/// Main application logic and event handling
use crate::gui::GuiState;
use crate::pointcloud::{align_to_ground, find_closest_point_to_ray, PointCloud};
use crate::renderer::Renderer;
use std::sync::Arc;
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::ActiveEventLoop,
    window::{Window, WindowId},
};

pub struct App {
    original_pointcloud: PointCloud, // Original point cloud
    current_pointcloud: PointCloud,  // Currently rendered point cloud
    renderer: Option<Renderer>,      // Renderer instance
    window: Option<Arc<Window>>,     // Window instance
    gui_state: GuiState,             // GUI state
    egui_ctx: Option<egui::Context>,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    cursor_position: Option<(f64, f64)>,
    reload_vertices: bool,
}

impl App {
    pub fn new(pointcloud: PointCloud) -> Self {
        let original_pointcloud = pointcloud.clone();

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
        // reload vertex data if marked for update
        if self.reload_vertices {
            if let Some(ref mut renderer) = self.renderer {
                renderer.update_point_cloud(&self.current_pointcloud);
                // Also update crop bounds in shader after alignment
                renderer.update_crop_bounds(self.gui_state.crop_min, self.gui_state.crop_max);
            }
            self.reload_vertices = false;
        }
        let mut consumed_by_egui = false;
        if let Some(ref mut renderer) = self.renderer {
            // Handle point selection before other input processing
            if let (Some(ref mut egui_state), Some(ref window)) =
                (&mut self.egui_state, &self.window)
            {
                // Check if we should process this event for point selection
                consumed_by_egui = egui_state.on_window_event(window.as_ref(), &event).consumed;

                // Track cursor movement
                if let WindowEvent::CursorMoved { position, .. } = event {
                    self.cursor_position = Some((position.x, position.y));
                }

                // Only process mouse clicks for point selection if egui didn't consume the event
                if !consumed_by_egui && self.gui_state.point_selection_mode {
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

                                // Perform alignment directly
                                println!(
                                    "Aligning point cloud to ground plane at selected point..."
                                );
                                let aligned_pointcloud = align_to_ground(
                                    &self.original_pointcloud,
                                    closest_point,
                                    glam::Vec3::new(1.25, 1.25, 0.75),
                                );
                                self.current_pointcloud = aligned_pointcloud;

                                // Update GUI bounds with the aligned point cloud
                                self.gui_state.update_pointcloud_bounds(
                                    glam::Vec3::from(self.current_pointcloud.min_coord),
                                    glam::Vec3::from(self.current_pointcloud.max_coord),
                                );
                                self.gui_state.reset_crop();

                                // Mark that we need to update vertex data
                                self.reload_vertices = true;

                                self.gui_state.point_selection_mode = false; // Exit point selection mode
                            } else {
                                self.gui_state.point_selection_failed = true;
                            }
                        }
                        return; // Don't process this event further
                    }
                }
            }

            if !consumed_by_egui && !renderer.on_window_event(&event) {
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

                            // Handle reset alignment requests
                            if self.gui_state.try_consume_alignment_request() {
                                println!("Resetting point cloud alignment...");
                                // Reset to original point cloud
                                self.current_pointcloud = self.original_pointcloud.clone();

                                // Update GUI bounds with the original point cloud
                                self.gui_state.update_pointcloud_bounds(
                                    glam::Vec3::from(self.current_pointcloud.min_coord),
                                    glam::Vec3::from(self.current_pointcloud.max_coord),
                                );
                                self.gui_state.reset_crop();

                                // Mark that we need to update vertex data
                                self.reload_vertices = true;
                            }

                            // Handle export PCD requests
                            if self.gui_state.try_consume_export_pcd() {
                                println!("Export PCD requested...");

                                if let Some(path) = rfd::FileDialog::new()
                                    .set_title("Save Point Cloud")
                                    .add_filter("PCD Files", &["pcd"])
                                    .set_file_name("exported.pcd")
                                    .save_file()
                                {
                                    if let Some(path_str) = path.to_str() {
                                        let cropped_pointcloud = crate::pointcloud::crop(
                                            &self.current_pointcloud,
                                            self.gui_state.crop_min,
                                            self.gui_state.crop_max,
                                        );
                                        if None
                                            == crate::pointcloud::export_pcd(
                                                path_str,
                                                &cropped_pointcloud,
                                            )
                                        {
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

                            let render_result = renderer.render_with_egui(
                                self.egui_renderer.as_mut().unwrap(),
                                egui_ctx,
                                &full_output,
                            );
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
