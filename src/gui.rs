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
    // Export functionality
    pub export_pcd_requested: bool,
}

#[derive(Default)]
pub struct CameraInfo {
    pub position: [f32; 3],
    pub target: [f32; 3],
    pub distance: f32,
    pub theta: f32, // Horizontal angle (yaw) in radians
    pub phi: f32,   // Vertical angle (pitch) in radians
}

impl GuiState {
    pub fn new() -> Self {
        Self {
            show_axes: true,
            show_target_disc: true,
            camera_info: CameraInfo::default(),
            crop_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            crop_max: glam::Vec3::new(100.0, 100.0, 100.0),
            pointcloud_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            pointcloud_max: glam::Vec3::new(100.0, 100.0, 100.0),
            alignment_requested: false,
            reset_alignment_requested: false,
            point_selection_mode: false,
            selected_ground_point: None,
            point_selection_failed: false,
            alignment_in_progress: false,
            export_pcd_requested: false,
        }
    }

    pub fn update_camera_info(
        &mut self,
        position: [f32; 3],
        target: [f32; 3],
        distance: f32,
        theta: f32,
        phi: f32,
    ) {
        self.camera_info.position = position;
        self.camera_info.target = target;
        self.camera_info.distance = distance;
        self.camera_info.theta = theta;
        self.camera_info.phi = phi;
    }

    pub fn update_pointcloud_bounds(&mut self, min_coords: glam::Vec3, max_coords: glam::Vec3) {
        self.pointcloud_min = min_coords;
        self.pointcloud_max = max_coords;
        // Initialize crop bounds to full range if they're still at default values
        let defaults = (
            glam::Vec3::new(-100.0, -100.0, -100.0),
            glam::Vec3::new(100.0, 100.0, 100.0),
        );
        if (self.crop_min, self.crop_max) == defaults {
            (self.crop_min, self.crop_max) = (min_coords, max_coords);
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

                egui::CollapsingHeader::new("Ground plane alignment")
                    .default_open(true)
                    .show(ui, |ui| {
                        if self.point_selection_mode {
                            ui.colored_label(
                                egui::Color32::YELLOW,
                                "Click on a point in the 3D view to align ground plane",
                            );
                            if ui.button("Cancel").clicked() {
                                self.point_selection_mode = false;
                                self.selected_ground_point = None;
                                self.point_selection_failed = false;
                            }
                            if self.point_selection_failed {
                                ui.colored_label(egui::Color32::RED, "No valid point found");
                            }
                        } else {
                            if ui.button("Align to Ground").clicked() {
                                self.point_selection_mode = true;
                                self.selected_ground_point = None;
                                self.point_selection_failed = false;
                            }
                        }
                        if ui.button("Reset alignment").clicked() {
                            self.reset_alignment_requested = true;
                            self.point_selection_mode = false;
                            self.selected_ground_point = None;
                            self.point_selection_failed = false;
                        }
                    });

                ui.separator();

                egui::CollapsingHeader::new("Export")
                    .default_open(true)
                    .show(ui, |ui| {
                        if ui.button("Export to PCD").clicked() {
                            self.export_pcd_requested = true;
                        }
                    });

                ui.separator();

                egui::CollapsingHeader::new("Cropping")
                    .default_open(true)
                    .show(ui, |ui| {
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

                        if ui.button("Reset crop coordinates").clicked() {
                            self.crop_min = self.pointcloud_min;
                            self.crop_max = self.pointcloud_max;
                        }
                    });

                ui.separator();

                // Bottom panel
                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
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
                    ui.horizontal(|ui| {
                        ui.label("Angles:");
                        ui.label(format!(
                            "(azimuth: {:.1}°, elevation: {:.1}°)",
                            self.camera_info.theta.to_degrees(),
                            self.camera_info.phi.to_degrees()
                        ));
                    });

                    ui.label("Camera Info:");
                    ui.separator();

                    [
                        "• Left Mouse: Orbit camera",
                        "• Right Mouse / Mouse Wheel: Zoom",
                        "• Middle Mouse: Move target",
                    ]
                    .iter()
                    .for_each(|&control| {
                        ui.label(control);
                    });
                    ui.label("Camera Controls:");
                    ui.separator();
                });
            });
    }

    /// Check if alignment was requested and reset the flag
    pub fn take_alignment_request(&mut self) -> bool {
        std::mem::take(&mut self.alignment_requested)
    }

    /// Check if reset alignment was requested and reset the flag
    pub fn take_reset_alignment_request(&mut self) -> bool {
        std::mem::take(&mut self.reset_alignment_requested)
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

    /// Check if export PCD was requested and reset the flag
    pub fn take_export_pcd_request(&mut self) -> bool {
        std::mem::take(&mut self.export_pcd_requested)
    }
}
