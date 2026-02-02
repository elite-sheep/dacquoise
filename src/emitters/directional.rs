// Copyright @yucwang 2026

use crate::core::emitter::{Emitter, EmitterFlag};
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::math::constants::{Float, Vector2f, Vector3f, EPSILON};
use crate::math::spectrum::RGBSpectrum;

pub struct DirectionalEmitter {
    pub direction: Vector3f,
    pub irradiance: RGBSpectrum,
}

impl DirectionalEmitter {
    pub fn new_with(direction: Vector3f, irradiance: RGBSpectrum) -> Self {
        Self { direction, irradiance }
    }
}

impl Emitter for DirectionalEmitter {
    fn new() -> Self {
        Self {
            direction: Vector3f::new(0.0, 0.0, -1.0),
            irradiance: RGBSpectrum::default(),
        }
    }

    fn get_flag(&self) -> EmitterFlag {
        EmitterFlag::DIRECTION | EmitterFlag::DELTA
    }

    fn sample_position(&self, _u: &Vector2f) -> SurfaceSampleRecord {
        let p = Vector3f::zeros();
        let n = Vector3f::new(0.0, 1.0, 0.0);
        let uv = Vector2f::new(0.0, 0.0);
        let intersection = SurfaceIntersection::new(p, n, n, uv, 0.0, self.irradiance, None, None);
        SurfaceSampleRecord::new(intersection, 0.0)
    }

    fn sample_direction(&self, _u: &Vector2f, _position: &SurfaceIntersection) -> Vector3f {
        let len = self.direction.norm();
        if len <= 0.0 {
            Vector3f::zeros()
        } else {
            self.direction / len
        }
    }

    fn pdf_position(&self, _position: &SurfaceIntersection) -> Float {
        0.0
    }

    fn pdf_direction(&self, _position: &SurfaceIntersection, direction: &Vector3f) -> Float {
        let len = self.direction.norm();
        if len <= 0.0 {
            return 0.0;
        }
        let d = self.direction / len;
        let dir_len = direction.norm();
        if dir_len <= 0.0 {
            return 0.0;
        }
        let dir = direction / dir_len;
        if (d - dir).norm() <= EPSILON {
            1.0
        } else {
            0.0
        }
    }
}
