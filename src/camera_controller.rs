use crate::gui::GUI_WIDTH;
use std::f32::consts::PI;

use winit::event::*;
use winit::keyboard::{Key, NamedKey};

use crate::camera::OrbitCamera;

const INITIAL_THETA: f32 = 0.0_f32;
const INITIAL_PHI: f32 = -(PI - 1e-4) / 2.0;
const INITIAL_FOV: f32 = 60.0_f32.to_radians();

struct InputState {
    // Mouse states
    is_mouse_pressed: bool,
    is_right_mouse_pressed: bool,
    is_middle_mouse_pressed: bool,
    last_mouse_pos: glam::Vec2,

    // Track if mouse interaction started in render area
    mouse_started_in_render_area: bool,

    // Modifier key states
    is_shift_pressed: bool,
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            is_mouse_pressed: false,
            is_right_mouse_pressed: false,
            is_middle_mouse_pressed: false,
            last_mouse_pos: glam::Vec2::ZERO,
            mouse_started_in_render_area: false,
            is_shift_pressed: false,
        }
    }
}

pub struct CameraController {
    // Control parameters
    sensitivity: f32,
    zoom_speed: f32,

    // Input state
    input: InputState,

    // Orbit camera
    camera: OrbitCamera,

    // Window and GUI layout tracking
    window_size: glam::Vec2,
    gui_width: f32,
}

impl CameraController {
    pub fn new(target: glam::Vec3, initial_distance: f32) -> Self {
        let mut camera = OrbitCamera::new(1.0, INITIAL_FOV, 0.1, 1000.0);
        camera.target = target;
        camera.distance = initial_distance;
        camera.theta = INITIAL_THETA;
        camera.phi = INITIAL_PHI;

        Self {
            sensitivity: 0.005,
            zoom_speed: 0.1,
            input: InputState::default(),
            camera,
            window_size: glam::Vec2::new(1024.0, 768.0),
            gui_width: GUI_WIDTH,
        }
    }

    pub fn set_window_size(&mut self, window_size: glam::Vec2) {
        self.window_size = window_size;
    }

    // Check if a mouse position is within the 3D render area
    fn is_in_render_area(&self, mouse_pos: glam::Vec2) -> bool {
        let render_area_width = self.window_size.x - self.gui_width;
        mouse_pos.x >= 0.0
            && mouse_pos.x < render_area_width
            && mouse_pos.y >= 0.0
            && mouse_pos.y < self.window_size.y
    }

    fn handle_mouse_input(&mut self, state: ElementState, button: MouseButton) -> bool {
        let is_pressed = state == ElementState::Pressed;

        // When mouse is pressed, check if it's in the render area
        if is_pressed {
            self.input.mouse_started_in_render_area =
                self.is_in_render_area(self.input.last_mouse_pos);
        }

        // Only process mouse input if it started in the render area
        if !self.input.mouse_started_in_render_area && is_pressed {
            return false;
        }

        match button {
            MouseButton::Left => {
                self.input.is_mouse_pressed = is_pressed;
                true
            }
            MouseButton::Right => {
                self.input.is_right_mouse_pressed = is_pressed;
                true
            }
            MouseButton::Middle => {
                self.input.is_middle_mouse_pressed = is_pressed;
                true
            }
            _ => false,
        }
    }

    fn handle_cursor_moved(&mut self, cursor_pos: glam::Vec2) -> bool {
        // Only process mouse movement if the interaction started in the render area
        if !self.input.mouse_started_in_render_area {
            self.input.last_mouse_pos = cursor_pos;
            return false;
        }

        if self.input.is_mouse_pressed {
            // Check if shift is pressed for target panning with left mouse
            if self.input.is_shift_pressed {
                self.move_target(cursor_pos);
                true
            } else {
                // Left mouse: orbit camera around target
                let delta = cursor_pos - self.input.last_mouse_pos;
                self.camera.theta -= delta.x * self.sensitivity;
                self.camera.theta = self.camera.theta.rem_euclid(2.0 * PI);

                self.camera.phi -= delta.y * self.sensitivity;
                self.camera.phi = self.camera.phi.clamp(
                    -std::f32::consts::FRAC_PI_2 + 1e-4,
                    std::f32::consts::FRAC_PI_2 - 1e-4,
                );
                self.input.last_mouse_pos = cursor_pos;
                true
            }
        } else if self.input.is_right_mouse_pressed {
            // Right mouse: zoom in/out
            let delta = cursor_pos - self.input.last_mouse_pos;
            let zoom_amount = -delta.y * self.sensitivity * self.camera.distance * 0.5;
            self.camera.distance = (self.camera.distance + zoom_amount).clamp(0.1, f32::MAX);
            self.input.last_mouse_pos = cursor_pos;
            true
        } else if self.input.is_middle_mouse_pressed {
            self.move_target(cursor_pos);
            true
        } else {
            self.input.last_mouse_pos = cursor_pos;
            false
        }
    }

    fn move_target(&mut self, cursor_pos: glam::Vec2) {
        // Middle mouse: pan target (RViz-style)
        let delta = cursor_pos - self.input.last_mouse_pos;
        let camera_vectors = self.calculate_camera_vectors();
        let pan_scale = self.camera.distance * self.sensitivity * 0.25;
        self.camera.target += camera_vectors.right * (-delta.x * pan_scale)
            + camera_vectors.up * (delta.y * pan_scale);
        self.input.last_mouse_pos = cursor_pos;
    }

    fn handle_mouse_wheel(&mut self, delta: &MouseScrollDelta) {
        let base_zoom = match delta {
            MouseScrollDelta::LineDelta(_, y) => -y * self.zoom_speed,
            MouseScrollDelta::PixelDelta(pos) => -pos.y as f32 * 0.01 * self.zoom_speed,
        };
        let zoom_amount = base_zoom * self.camera.distance;
        self.camera.distance = (self.camera.distance + zoom_amount).clamp(0.1, f32::MAX);
    }

    pub fn process_events(&mut self, event: &WindowEvent) -> bool {
        match event {
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
            WindowEvent::KeyboardInput { event, .. } => self.handle_keyboard_input(event),
            _ => false,
        }
    }

    fn handle_keyboard_input(&mut self, event: &KeyEvent) -> bool {
        // Track shift key state
        if let Key::Named(NamedKey::Shift) = event.logical_key {
            self.input.is_shift_pressed = event.state == ElementState::Pressed;
            return false; // Don't consume the event, let others handle it too
        }
        false
    }

    fn calculate_camera_vectors(&self) -> CameraVectors {
        let theta = self.camera.theta;
        let phi = self.camera.phi;
        let right = glam::Vec3::new(-theta.cos(), -theta.sin(), 0.0).normalize();
        let up = glam::Vec3::new(-phi.sin() * theta.sin(), phi.sin() * theta.cos(), phi.cos())
            .normalize();
        CameraVectors { right, up }
    }

    pub fn update(&mut self, camera: &mut OrbitCamera) {
        self.camera.distance = self.camera.distance.clamp(0.1, f32::MAX);

        camera.update_view(
            self.camera.target,
            self.camera.distance,
            self.camera.theta,
            self.camera.phi,
        );
    }

    /// Get the camera target
    pub fn get_target(&self) -> glam::Vec3 {
        self.camera.target
    }
}

struct CameraVectors {
    right: glam::Vec3,
    up: glam::Vec3,
}
