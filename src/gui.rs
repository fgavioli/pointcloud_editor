use crate::camera::OrbitCamera;
use egui::{Context, SidePanel};

pub const GUI_WIDTH: f32 = 250.0;

#[derive(Default)]
pub struct GuiState {
    pub show_axes: bool,
    pub show_target_disc: bool,
    pub camera_info: OrbitCamera,

    // Point cloud cropping
    pub crop_min: glam::Vec3, // minimum bounds
    pub crop_max: glam::Vec3, // maximum bounds

    // Point cloud bounds (updated from the loaded point cloud)
    pub pointcloud_min: glam::Vec3,
    pub pointcloud_max: glam::Vec3,

    // Ground plane alignment
    pub reset_alignment_requested: bool,
    pub point_selection_mode: bool,
    pub point_selection_failed: bool,

    // Export section
    pub export_pcd_requested: bool,
    pub export_png_requested: bool,
    pub png_resolution: f32,
}

impl GuiState {
    pub fn new() -> Self {
        Self {
            show_axes: true,
            show_target_disc: true,
            camera_info: OrbitCamera::default(),
            crop_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            crop_max: glam::Vec3::new(100.0, 100.0, 100.0),
            pointcloud_min: glam::Vec3::new(-100.0, -100.0, -100.0),
            pointcloud_max: glam::Vec3::new(100.0, 100.0, 100.0),
            reset_alignment_requested: false,
            point_selection_mode: false,
            point_selection_failed: false,
            export_pcd_requested: false,
            export_png_requested: false,
            png_resolution: 0.05, // Default to 5cm resolution
        }
    }

    pub fn update_camera_info(&mut self, camera_info: &OrbitCamera) {
        self.camera_info = camera_info.clone();
    }

    pub fn reset_crop(&mut self) {
        (self.crop_min, self.crop_max) = (self.pointcloud_min, self.pointcloud_max);
    }

    pub fn render(&mut self, ctx: &Context) {
        SidePanel::right("control_panel")
            .exact_width(250.0)
            .resizable(false)
            .show(ctx, |ui| {
                ui.heading("Point Cloud Editor");

                ui.separator();

                ui.collapsing("Display", |ui| {
                    ui.separator();
                    ui.checkbox(&mut self.show_axes, "Show Coordinate Axes");
                    ui.checkbox(&mut self.show_target_disc, "Show Target Position");
                });

                ui.separator();

                egui::CollapsingHeader::new("Ground plane alignment")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.separator();
                        if self.point_selection_mode {
                            ui.colored_label(
                                egui::Color32::YELLOW,
                                "Click on a surface in the 3D view to align it to the ground plane. Hint: Choose a surface with a high point density.",
                            );
                            if ui.button("Cancel").clicked() {
                                self.point_selection_mode = false;
                                self.point_selection_failed = false;
                            }
                            if self.point_selection_failed {
                                ui.colored_label(egui::Color32::RED, "No valid point found");
                            }
                        } else {
                            if ui.button("Align to Ground").clicked() {
                                self.point_selection_mode = true;
                                self.point_selection_failed = false;
                            }
                        }
                        if ui.button("Reset alignment").clicked() {
                            self.reset_alignment_requested = true;
                            self.point_selection_mode = false;
                            self.point_selection_failed = false;
                        }
                    });

                ui.separator();

                egui::CollapsingHeader::new("Cropping")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.separator();
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

                egui::CollapsingHeader::new("Export")
                    .default_open(true)
                    .show(ui, |ui| {

                        ui.separator();
                        ui.label("Export PCD:");
                        if ui.button("Export").clicked() {
                            self.export_pcd_requested = true;
                        }

                        ui.separator();

                        ui.label("Export to Occupancy Grid:");
                        ui.horizontal(|ui| {
                            ui.label("Resolution (m):");
                            ui.add(egui::DragValue::new(&mut self.png_resolution)
                                .range(0.01..=1.0)
                                .speed(0.01)
                                .fixed_decimals(2));
                        });

                        if ui.button("Export").clicked() {
                            self.export_png_requested = true;
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
                            self.camera_info.target.x,
                            self.camera_info.target.y,
                            self.camera_info.target.z
                        ));
                    });
                    ui.horizontal(|ui| {
                        let eye = self.camera_info.get_eye();
                        ui.label("Position:");
                        ui.label(format!(
                            "({:.1}, {:.1}, {:.1})",
                            eye.x, eye.y, eye.z
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
                        "• LMB: Orbit camera",
                        "• RMB / Mouse Wheel: Zoom",
                        "• Shift+LMB or MMB: Move target",
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

    /// Check if reset alignment was requested and reset the flag
    pub fn try_consume_alignment_request(&mut self) -> bool {
        let ret = self.reset_alignment_requested.clone();
        self.reset_alignment_requested = false;
        ret
    }

    /// Check if export PCD was requested and reset the flag
    pub fn try_consume_export_pcd(&mut self) -> bool {
        let ret = self.export_pcd_requested.clone();
        self.export_pcd_requested = false;
        ret
    }

    /// Check if export PNG was requested and reset the flag, returning resolution too
    pub fn try_consume_export_png(&mut self) -> Option<f32> {
        if self.export_png_requested {
            self.export_png_requested = false;
            Some(self.png_resolution)
        } else {
            None
        }
    }
}
