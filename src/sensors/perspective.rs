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
    bitmap: Bitmap,
}

impl PerspectiveCamera {
    pub fn new(origin: Vector3f,
               target: Vector3f,
               up: Vector3f,
               fov_y_radians: Float,
               aspect: Float,
               width: usize,
               height: usize) -> Self {
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
            bitmap: Bitmap::new(width, height),
        }
    }
}

impl Sensor for PerspectiveCamera {
    fn sample_ray(&self, u: &Vector2f) -> Ray3f {
        let px = (2.0 * u.x - 1.0) * self.aspect * self.tan_half_fov_y;
        let py = (1.0 - 2.0 * u.y) * self.tan_half_fov_y;

        let dir = (self.forward + self.right * px + self.up * py).normalize();
        Ray3f::new(self.origin, dir, None, None)
    }

    fn bitmap(&self) -> &Bitmap {
        &self.bitmap
    }

    fn bitmap_mut(&mut self) -> &mut Bitmap {
        &mut self.bitmap
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
        let cam = PerspectiveCamera::new(origin, target, up, fov_y, aspect, 4, 4);

        let ray = cam.sample_ray(&Vector2f::new(0.5, 0.5));
        let dir = ray.dir();

        assert!((dir.x - 0.0).abs() < 1e-6);
        assert!((dir.y - 0.0).abs() < 1e-6);
        assert!((dir.z + 1.0).abs() < 1e-6);
    }
}
