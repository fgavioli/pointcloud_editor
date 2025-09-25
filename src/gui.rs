use egui::{Context, SidePanel};

#[derive(Default)]
pub struct GuiState {
    pub point_size: f32,
    pub show_axes: bool,
    pub show_target_disc: bool,
    pub background_color: [f32; 3],
    pub camera_info: CameraInfo,
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
            point_size: 1.0,
            show_axes: true,
            show_target_disc: true,
            background_color: [0.0, 0.0, 0.0], // Black background
            camera_info: CameraInfo::default(),
        }
    }

    pub fn update_camera_info(&mut self, position: [f32; 3], target: [f32; 3], distance: f32) {
        self.camera_info.position = position;
        self.camera_info.target = target;
        self.camera_info.distance = distance;
    }

    pub fn render(&mut self, ctx: &Context) {
        SidePanel::right("control_panel")
            .exact_width(350.0)
            .resizable(false)
            .show(ctx, |ui| {
                ui.heading("Point Cloud Editor");

                ui.separator();

                ui.collapsing("Rendering", |ui| {
                    ui.add(egui::Slider::new(&mut self.point_size, 0.1..=10.0).text("Point Size"));

                    ui.checkbox(&mut self.show_axes, "Show Coordinate Axes");
                    ui.checkbox(&mut self.show_target_disc, "Show Target Disc");

                    ui.label("Background Color:");
                    ui.color_edit_button_rgb(&mut self.background_color);
                });

                ui.separator();

                ui.collapsing("Camera", |ui| {
                    ui.label("Camera Controls:");
                    ui.label("• Left Mouse: Orbit camera");
                    ui.label("• Right Mouse: Zoom");
                    ui.label("• Middle Mouse: Pan target");
                    ui.label("• Mouse Wheel: Zoom");

                    ui.separator();

                    ui.label("Camera Information:");
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
                        ui.label("Target:");
                        ui.label(format!(
                            "({:.1}, {:.1}, {:.1})",
                            self.camera_info.target[0],
                            self.camera_info.target[1],
                            self.camera_info.target[2]
                        ));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Distance:");
                        ui.label(format!("{:.1}", self.camera_info.distance));
                    });
                });

                ui.separator();

                ui.collapsing("File Operations", |ui| {
                    if ui.button("Open Point Cloud...").clicked() {
                        // TODO: Implement file dialog
                        println!("Open file dialog would appear here");
                    }

                    if ui.button("Export View...").clicked() {
                        // TODO: Implement export functionality
                        println!("Export functionality would be here");
                    }
                });

                ui.separator();

                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    ui.label("Point Cloud Editor v0.1.0");
                    ui.label("Use mouse to navigate the 3D view");
                });
            });
    }

    pub fn get_actual_panel_width(&self) -> f32 {
        350.0 // Return fixed width
    }
}
