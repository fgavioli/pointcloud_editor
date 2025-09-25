use winit::{
    event::*,
    keyboard::{Key, NamedKey},
};

use crate::camera::OrbitCamera;

const INITIAL_THETA: f32 = 0.0_f32.to_radians();
const INITIAL_PHI: f32 = -89.0_f32.to_radians();

// ============================================================================
// CAMERA SYSTEM
// ============================================================================

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
    is_middle_mouse_pressed: bool,
    last_mouse_pos: glam::Vec2,
    // Track if mouse interaction started in render area
    mouse_started_in_render_area: bool,
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
            is_middle_mouse_pressed: false,
            last_mouse_pos: glam::Vec2::ZERO,
            mouse_started_in_render_area: false,
        }
    }
}

pub struct CameraController {
    // Control parameters
    speed: f32,
    sensitivity: f32,
    zoom_speed: f32,

    // Input state
    input: InputState,

    // Orbit camera parameters
    pub target: glam::Vec3, // Center point to orbit around
    pub distance: f32,      // Distance from the target
    pub theta: f32,         // Horizontal angle (yaw)
    pub phi: f32,           // Vertical angle (pitch)
    
    // Window and GUI layout tracking
    window_size: glam::Vec2,
    gui_width: f32,
}

impl CameraController {
    pub fn new(speed: f32, target: glam::Vec3, initial_distance: f32) -> Self {
        Self {
            speed,
            sensitivity: 0.005,
            zoom_speed: 0.1,
            input: InputState::default(),
            target,
            distance: initial_distance,
            theta: INITIAL_THETA,
            phi: INITIAL_PHI,
            window_size: glam::Vec2::new(1024.0, 768.0), // Default window size
            gui_width: 300.0, // Default GUI panel width
        }
    }

    // Update window size and GUI width
    pub fn update_layout(&mut self, window_size: glam::Vec2, gui_width: f32) {
        self.window_size = window_size;
        self.gui_width = gui_width;
    }

    // Check if a mouse position is within the 3D render area (not GUI area)
    fn is_in_render_area(&self, mouse_pos: glam::Vec2) -> bool {
        const PANEL_RESIZE_PADDING: f32 = 20.0; // Padding in pixels to avoid accidental camera movement
        
        let render_area_width = self.window_size.x - self.gui_width - PANEL_RESIZE_PADDING;
        mouse_pos.x >= 0.0 && mouse_pos.x < render_area_width && 
        mouse_pos.y >= 0.0 && mouse_pos.y < self.window_size.y
    }

    // keyboard input handler
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
        
        // When mouse is pressed, check if it's in the render area
        if is_pressed {
            self.input.mouse_started_in_render_area = self.is_in_render_area(self.input.last_mouse_pos);
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

    fn handle_cursor_moved(&mut self, new_pos: glam::Vec2) -> bool {
        // Only process mouse movement if the interaction started in the render area
        if !self.input.mouse_started_in_render_area {
            self.input.last_mouse_pos = new_pos;
            return false;
        }
        
        if self.input.is_mouse_pressed {
            // Left mouse: orbit camera around target
            let delta = new_pos - self.input.last_mouse_pos;
            self.theta -= delta.x * self.sensitivity;
            self.phi -= delta.y * self.sensitivity;
            self.phi = self.phi.clamp(
                -std::f32::consts::FRAC_PI_2 + 0.1,
                std::f32::consts::FRAC_PI_2 - 0.1,
            );
            self.input.last_mouse_pos = new_pos;
            true
        } else if self.input.is_right_mouse_pressed {
            // Right mouse: zoom in/out
            let delta = new_pos - self.input.last_mouse_pos;
            let zoom_amount = -delta.y * self.sensitivity * self.distance * 0.5;
            self.distance = (self.distance + zoom_amount).clamp(0.1, f32::MAX);
            self.input.last_mouse_pos = new_pos;
            true
        } else if self.input.is_middle_mouse_pressed {
            // Middle mouse: pan target (RViz-style)
            let delta = new_pos - self.input.last_mouse_pos;

            // Calculate camera vectors for target panning
            let camera_vectors = self.calculate_camera_vectors();

            // Scale the panning based on distance (closer = smaller movements, farther = larger movements)
            let pan_scale = self.distance * 0.001;

            // Move target:
            // - Mouse right = view pans right (target moves left relative to camera)
            // - Mouse up = view pans up (target moves down relative to camera)
            let right_movement = camera_vectors.right * (-delta.x * pan_scale);
            let up_movement = camera_vectors.up * (delta.y * pan_scale);

            self.target += right_movement + up_movement;
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
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key: key,
                        state,
                        ..
                    },
                ..
            } => self.handle_keyboard_input(key, *state == ElementState::Pressed),
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

    fn calc_distance(&mut self, dt: f32) {
        if self.input.is_up_pressed {
            self.distance -= self.zoom_speed * dt * 10.0;
        }
        if self.input.is_down_pressed {
            self.distance += self.zoom_speed * dt * 10.0;
        }
        self.distance = self.distance.clamp(0.1, f32::MAX);
    }

    fn calc_target(&mut self, dt: f32) {
        let camera_vectors = self.calculate_camera_vectors();
        let mut target_movement = glam::Vec3::ZERO;

        if self.input.is_right_pressed {
            target_movement -= camera_vectors.right;
        }
        if self.input.is_left_pressed {
            target_movement += camera_vectors.right;
        }
        if self.input.is_pan_up_pressed {
            target_movement += camera_vectors.up;
        }
        if self.input.is_pan_down_pressed {
            target_movement -= camera_vectors.up;
        }

        self.target += target_movement * self.speed * dt;
    }

    fn calculate_camera_vectors(&self) -> CameraVectors {
        let right = glam::Vec3::new(-self.theta.cos(), -self.theta.sin(), 0.0).normalize();
        let up = glam::Vec3::new(
            -self.phi.sin() * self.theta.sin(),
            self.phi.sin() * self.theta.cos(),
            self.phi.cos(),
        )
        .normalize();
        CameraVectors { right, up }
    }

    pub fn update(&mut self, camera: &mut OrbitCamera, dt: std::time::Duration) {
        let dt = dt.as_secs_f32();
        self.calc_distance(dt);
        self.calc_target(dt);
        camera.update_view(self.target, self.distance, self.theta, self.phi);
    }
}

struct CameraVectors {
    right: glam::Vec3,
    up: glam::Vec3,
}
