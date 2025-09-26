// Up vector
const UP: glam::Vec3 = glam::Vec3::Z;

pub struct OrbitCamera {
    eye: glam::Vec3,
    target: glam::Vec3,
    aspect: f32,
    fov: f32,
    znear: f32,
    zfar: f32,
}

impl OrbitCamera {
    pub fn new(aspect: f32, fov: f32, znear: f32, zfar: f32) -> Self {
        Self {
            eye: glam::Vec3::ZERO,
            target: glam::Vec3::ZERO,
            aspect,
            fov,
            znear,
            zfar,
        }
    }

    pub fn update_view(&mut self, target: glam::Vec3, distance: f32, theta: f32, phi: f32) {
        let base_forward = glam::Vec3::new(0.0, -1.0, 0.0);
        let yaw_rotation = glam::Mat3::from_rotation_z(theta);
        let yawed_forward = yaw_rotation * base_forward;
        let right_vector = yawed_forward.cross(UP).normalize();
        let pitch_rotation = glam::Mat3::from_axis_angle(right_vector, phi);
        let final_forward = pitch_rotation * yawed_forward;

        self.eye = target - final_forward * distance;
        self.target = target;
    }

    pub fn update_proj(&mut self, fov: f32, aspect: f32, znear: f32, zfar: f32) {
        self.fov = fov;
        self.aspect = aspect;
        self.znear = znear;
        self.zfar = zfar;
    }

    pub fn get_vp_matrix(&self) -> glam::Mat4 {
        let view = glam::Mat4::look_at_rh(self.eye, self.target, UP);
        let proj = glam::Mat4::perspective_rh(self.fov, self.aspect, self.znear, self.zfar);
        proj * view
    }

    pub fn get_fov(&self) -> f32 {
        self.fov
    }

    pub fn get_eye(&self) -> glam::Vec3 {
        self.eye
    }

    pub fn get_target(&self) -> glam::Vec3 {
        self.target
    }

    pub fn get_distance(&self) -> f32 {
        self.eye.distance(self.target)
    }

    /// Convert screen coordinates to world ray
    pub fn screen_to_world_ray(&self, screen_pos: (f32, f32), screen_size: (f32, f32)) -> (glam::Vec3, glam::Vec3) {
        // Convert screen coordinates to normalized device coordinates (-1 to 1)
        let ndc_x = (2.0 * screen_pos.0 / screen_size.0) - 1.0;
        let ndc_y = 1.0 - (2.0 * screen_pos.1 / screen_size.1); // Flip Y axis

        // Create projection matrix
        let proj = glam::Mat4::perspective_rh(self.fov, self.aspect, self.znear, self.zfar);
        let view = glam::Mat4::look_at_rh(self.eye, self.target, UP);
        let proj_view = proj * view;

        // Get inverse of projection-view matrix
        let inv_proj_view = proj_view.inverse();

        // Convert NDC points to world space
        let near_point = inv_proj_view * glam::Vec4::new(ndc_x, ndc_y, -1.0, 1.0);
        let far_point = inv_proj_view * glam::Vec4::new(ndc_x, ndc_y, 1.0, 1.0);

        // Perspective divide
        let near_world = glam::Vec3::new(
            near_point.x / near_point.w,
            near_point.y / near_point.w,
            near_point.z / near_point.w,
        );
        let far_world = glam::Vec3::new(
            far_point.x / far_point.w,
            far_point.y / far_point.w,
            far_point.z / far_point.w,
        );

        // Ray origin is camera position, direction is normalized vector from near to far
        let ray_origin = self.eye;
        let ray_direction = (far_world - near_world).normalize();

        (ray_origin, ray_direction)
    }
}
