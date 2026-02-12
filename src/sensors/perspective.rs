// Copyright @yucwang 2026

use crate::core::sensor::Sensor;
use crate::math::bitmap::Bitmap;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::ray::Ray3f;

pub struct PerspectiveCamera {
    origin: Vector3f,
    forward: Vector3f,
    right: Vector3f,
    up: Vector3f,
    tan_half_fov_y: Float,
    aspect: Float,
    near_clip: Float,
    far_clip: Float,
    bitmap: Bitmap,
}

impl PerspectiveCamera {
    pub fn new(origin: Vector3f,
               target: Vector3f,
               up: Vector3f,
               fov_y_radians: Float,
               aspect: Float,
               width: usize,
               height: usize,
               near_clip: Float,
               far_clip: Float) -> Self {
        let forward = (target - origin).normalize();
        let right = forward.cross(&up).normalize();
        let up = right.cross(&forward).normalize();

        Self {
            origin,
            forward,
            right,
            up,
            tan_half_fov_y: (0.5 * fov_y_radians).tan(),
            aspect,
            near_clip,
            far_clip,
            bitmap: Bitmap::new(width, height),
        }
    }
}

impl PerspectiveCamera {
    pub fn width(&self) -> usize {
        self.bitmap.width()
    }

    pub fn height(&self) -> usize {
        self.bitmap.height()
    }
}

impl Sensor for PerspectiveCamera {
    fn sample_ray(&self, u: &Vector2f) -> Ray3f {
        let px = (2.0 * u.x - 1.0) * self.aspect * self.tan_half_fov_y;
        let py = (1.0 - 2.0 * u.y) * self.tan_half_fov_y;

        let d_camera = Vector3f::new(px, py, 1.0).normalize();
        let dir = (self.right * d_camera.x + self.up * d_camera.y + self.forward * d_camera.z).normalize();

        let inv_z = if d_camera.z != 0.0 { 1.0 / d_camera.z } else { std::f32::MAX };
        let near_t = self.near_clip * inv_z;
        let far_t = self.far_clip * inv_z;
        let origin = self.origin + dir * near_t;
        let max_t = far_t - near_t;
        Ray3f::new(origin, dir, Some(0.0), Some(max_t))
    }

    fn bitmap(&self) -> &Bitmap {
        &self.bitmap
    }

    fn bitmap_mut(&mut self) -> &mut Bitmap {
        &mut self.bitmap
    }

    fn describe(&self) -> String {
        String::from("PerspectiveCamera\n  origin: Vector3f\n  forward: Vector3f\n  right: Vector3f\n  up: Vector3f\n  tan_half_fov_y: Float\n  aspect: Float\n  near_clip: Float\n  far_clip: Float")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perspective_camera_center_ray() {
        let origin = Vector3f::new(0.0, 0.0, 0.0);
        let target = Vector3f::new(0.0, 0.0, -1.0);
        let up = Vector3f::new(0.0, 1.0, 0.0);
        let fov_y = std::f32::consts::FRAC_PI_2;
        let aspect = 1.0;
        let cam = PerspectiveCamera::new(origin, target, up, fov_y, aspect, 4, 4, 0.0, std::f32::MAX);

        let ray = cam.sample_ray(&Vector2f::new(0.5, 0.5));
        let dir = ray.dir();

        assert!((dir.x - 0.0).abs() < 1e-6);
        assert!((dir.y - 0.0).abs() < 1e-6);
        assert!((dir.z + 1.0).abs() < 1e-6);
    }
}
