use egui::{Context, SidePanel};

#[derive(Default)]
pub struct GuiState {
    pub show_axes: bool,
    pub show_target_disc: bool,
    pub camera_info: CameraInfo,
    // Point cloud cropping
    pub crop_min: glam::Vec3, // minimum bounds
    pub crop_max: glam::Vec3, // maximum bounds
    // Point cloud bounds (updated from the loaded point cloud)
    pub pointcloud_min: glam::Vec3,
    pub pointcloud_max: glam::Vec3,
    // Ground plane alignment
    pub alignment_requested: bool,
    pub reset_alignment_requested: bool,
    // Point selection mode
    pub point_selection_mode: bool,
    pub selected_ground_point: Option<glam::Vec3>,
    pub point_selection_failed: bool,
    // Alignment progress
    pub alignment_in_progress: bool,
}

#[derive(Default)]
pub struct CameraInfo {
    pub position: [f32; 3],
    pub target: [f32; 3],
    pub distance: f32,
}

impl GuiState {
    pub fn new() -> Self {
        Self {
            show_axes: true,
            show_target_disc: true,
            camera_info: CameraInfo::default(),
            // Initialize cropping to wide ranges (will be updated based on point cloud bounds)
            crop_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            crop_max: glam::Vec3::new(100.0, 100.0, 100.0),
            // Initialize point cloud bounds to default values
            pointcloud_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            pointcloud_max: glam::Vec3::new(100.0, 100.0, 100.0),
            // Initialize alignment flags
            alignment_requested: false,
            reset_alignment_requested: false,
            // Initialize point selection
            point_selection_mode: false,
            selected_ground_point: None,
            point_selection_failed: false,
            // Initialize alignment progress
            alignment_in_progress: false,
        }
    }

    pub fn update_camera_info(&mut self, position: [f32; 3], target: [f32; 3], distance: f32) {
        self.camera_info.position = position;
        self.camera_info.target = target;
        self.camera_info.distance = distance;
    }

    pub fn update_pointcloud_bounds(&mut self, min_coords: glam::Vec3, max_coords: glam::Vec3) {
        self.pointcloud_min = min_coords;
        self.pointcloud_max = max_coords;
        // Initialize crop bounds to full range on first update
        let default_min = glam::Vec3::new(-100.0, -100.0, -100.0);
        let default_max = glam::Vec3::new(100.0, 100.0, 100.0);
        if self.crop_min == default_min && self.crop_max == default_max {
            self.crop_min = min_coords;
            self.crop_max = max_coords;
        }
    }

    pub fn reset_crop_bounds_to_full_range(&mut self) {
        self.crop_min = self.pointcloud_min;
        self.crop_max = self.pointcloud_max;
    }

    pub fn render(&mut self, ctx: &Context) {
        SidePanel::right("control_panel")
            .exact_width(350.0)
            .resizable(false)
            .show(ctx, |ui| {
                ui.heading("Point Cloud Editor");

                ui.separator();

                ui.collapsing("Display", |ui| {
                    ui.checkbox(&mut self.show_axes, "Show Coordinate Axes");
                    ui.checkbox(&mut self.show_target_disc, "Show Target Position");
                });

                ui.separator();

                ui.collapsing("Ground plane alignment", |ui| {
                    if self.point_selection_mode {
                        if self.selected_ground_point.is_none() && !self.point_selection_failed {
                            ui.colored_label(
                                egui::Color32::YELLOW,
                                "Click on a point in the 3D view to select ground point"
                            );
                        }
                        if ui.button("Cancel point selection").clicked() {
                            self.point_selection_mode = false;
                            self.point_selection_failed = false;
                        }
                        if let Some(point) = self.selected_ground_point {
                            ui.label(format!(
                                "Selected point: ({:.2}, {:.2}, {:.2})",
                                point.x, point.y, point.z
                            ));
                            if ui.button("Align to selected point").clicked() {
                                println!("Align button clicked! Setting alignment_requested = true");
                                self.alignment_requested = true;
                                self.point_selection_mode = false; // Exit point selection mode after alignment
                                self.point_selection_failed = false;
                            }
                        } else if self.point_selection_failed {
                            ui.colored_label(
                                egui::Color32::RED,
                                "No valid point found"
                            );
                        }
                    } else {
                        ui.label("Align point cloud to ground plane:");
                        if ui.button("Select ground point").clicked() {
                            self.point_selection_mode = true;
                            self.selected_ground_point = None;
                            self.point_selection_failed = false;
                        }
                    }
                    if ui.button("Reset alignment").clicked() {
                        self.reset_alignment_requested = true;
                    }
                }).fully_open();

                ui.separator();

                ui.collapsing("Cropping", |ui| {
                    ui.label("X Axis:");
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_min.x,
                            self.pointcloud_min.x..=self.pointcloud_max.x,
                        )
                        .text("Min X"),
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_max.x,
                            self.pointcloud_min.x..=self.pointcloud_max.x,
                        )
                        .text("Max X"),
                    );

                    ui.label("Y Axis:");
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_min.y,
                            self.pointcloud_min.y..=self.pointcloud_max.y,
                        )
                        .text("Min Y"),
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_max.y,
                            self.pointcloud_min.y..=self.pointcloud_max.y,
                        )
                        .text("Max Y"),
                    );

                    ui.label("Z Axis:");
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_min.z,
                            self.pointcloud_min.z..=self.pointcloud_max.z,
                        )
                        .text("Min Z"),
                    );
                    ui.add(
                        egui::Slider::new(
                            &mut self.crop_max.z,
                            self.pointcloud_min.z..=self.pointcloud_max.z,
                        )
                        .text("Max Z"),
                    );

                    // Add a reset button
                    if ui.button("Reset to Full Range").clicked() {
                        self.crop_min = self.pointcloud_min;
                        self.crop_max = self.pointcloud_max;
                    }
                }).fully_open();

                ui.separator();

                // ui.collapsing("File Operations", |ui| {
                //     if ui.button("Open Point Cloud...").clicked() {
                //         // TODO: Implement file dialog
                //         println!("Open file dialog would appear here");
                //     }

                //     if ui.button("Export View...").clicked() {
                //         // TODO: Implement export functionality
                //         println!("Export functionality would be here");
                //     }
                // });
                // ui.separator();

                // Bottom panel
                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    // ui.label("Point Cloud Editor v0.1.0");

                    ui.separator();

                    ui.horizontal(|ui| {
                        ui.label("Distance:");
                        ui.label(format!("{:.1}", self.camera_info.distance));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Target:");
                        ui.label(format!(
                            "({:.1}, {:.1}, {:.1})",
                            self.camera_info.target[0],
                            self.camera_info.target[1],
                            self.camera_info.target[2]
                        ));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Position:");
                        ui.label(format!(
                            "({:.1}, {:.1}, {:.1})",
                            self.camera_info.position[0],
                            self.camera_info.position[1],
                            self.camera_info.position[2]
                        ));
                    });

                    ui.label("Camera Info:");

                    ui.separator();

                    ui.label("• Middle Mouse: Move target");
                    ui.label("• Right Mouse / Mouse Wheel: Zoom");
                    ui.label("• Left Mouse: Orbit camera");
                    ui.label("Camera Controls:");

                    ui.separator();
                });
            });

        // Show alignment progress dialog
        if self.alignment_in_progress {
            egui::Window::new("Aligning Point Cloud")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label("Calculating ground plane alignment...");
                    });
                    ui.add_space(10.0);
                    ui.label("Please wait while the point cloud is being aligned.");
                });
        }
    }

    pub fn get_actual_panel_width(&self) -> f32 {
        350.0 // Return fixed width
    }

    /// Check if alignment was requested and reset the flag
    pub fn take_alignment_request(&mut self) -> bool {
        if self.alignment_requested {
            println!("take_alignment_request: returning true, resetting flag");
            self.alignment_requested = false;
            true
        } else {
            false
        }
    }

    /// Check if reset alignment was requested and reset the flag
    pub fn take_reset_alignment_request(&mut self) -> bool {
        if self.reset_alignment_requested {
            self.reset_alignment_requested = false;
            true
        } else {
            false
        }
    }

    /// Check if we're in point selection mode
    pub fn is_point_selection_mode(&self) -> bool {
        self.point_selection_mode
    }

    /// Set a selected ground point and exit point selection mode
    pub fn set_ground_point(&mut self, point: glam::Vec3) {
        self.selected_ground_point = Some(point);
        // Don't exit point selection mode here - let the user decide what to do next
        // self.point_selection_mode = false;
    }

    /// Get the selected ground point
    pub fn get_selected_ground_point(&self) -> Option<glam::Vec3> {
        self.selected_ground_point
    }

    /// Cancel point selection mode
    pub fn cancel_point_selection(&mut self) {
        self.point_selection_mode = false;
        self.point_selection_failed = false;
    }

    /// Set point selection as failed (no valid point found)
    pub fn set_point_selection_failed(&mut self) {
        self.point_selection_failed = true;
    }

    /// Start alignment progress
    pub fn start_alignment_progress(&mut self) {
        self.alignment_in_progress = true;
    }

    /// Stop alignment progress
    pub fn stop_alignment_progress(&mut self) {
        self.alignment_in_progress = false;
    }
}
