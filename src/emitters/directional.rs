// Copyright @yucwang 2026

use crate::core::computation_node::ComputationNode;
use crate::core::emitter::{Emitter, EmitterFlag};
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::tangent_frame::build_tangent_frame;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::sample_uniform_disk_concentric;

pub struct DirectionalEmitter {
    pub direction: Vector3f,
    pub irradiance: RGBSpectrum,
    bsphere_center: Vector3f,
    bsphere_radius: Float,
}

impl DirectionalEmitter {
    pub fn new_with(direction: Vector3f, irradiance: RGBSpectrum) -> Self {
        Self {
            direction,
            irradiance,
            bsphere_center: Vector3f::zeros(),
            bsphere_radius: 1.0,
        }
    }
}

impl ComputationNode for DirectionalEmitter {
    fn to_string(&self) -> String {
        String::from("DirectionalEmitter")
    }
}

impl Emitter for DirectionalEmitter {
    fn new() -> Self {
        Self {
            direction: Vector3f::new(0.0, 0.0, -1.0),
            irradiance: RGBSpectrum::default(),
            bsphere_center: Vector3f::zeros(),
            bsphere_radius: 1.0,
        }
    }

    fn get_flag(&self) -> EmitterFlag {
        EmitterFlag::DIRECTION | EmitterFlag::DELTA
    }

    fn set_scene_bounds(&mut self, bounds: &AABB) {
        if bounds.is_valid() {
            let center = bounds.center();
            let radius = (bounds.p_max - center).norm().max(1e-6);
            self.bsphere_center = center;
            self.bsphere_radius = radius;
        } else {
            self.bsphere_center = Vector3f::zeros();
            self.bsphere_radius = 1e-6;
        }
    }

    fn sample_position(&self, u: &Vector2f) -> SurfaceSampleRecord {
        let len = self.direction.norm();
        let emit_dir = if len > 0.0 {
            self.direction / len
        } else {
            Vector3f::new(0.0, 0.0, 1.0)
        };
        let (tangent, bitangent) = build_tangent_frame(&emit_dir);
        let disk = sample_uniform_disk_concentric(u);
        let perp_offset = tangent * disk.x + bitangent * disk.y;
        let p = self.bsphere_center + (perp_offset - emit_dir) * self.bsphere_radius;
        let n = emit_dir;
        let uv = Vector2f::new(0.0, 0.0);
        let intersection = SurfaceIntersection::new(p, n, n, uv, 0.0, self.irradiance, None, None);
        let pdf = 1.0 / (std::f32::consts::PI * self.bsphere_radius * self.bsphere_radius).max(1e-6);
        SurfaceSampleRecord::new(intersection, pdf)
    }

    fn sample_direction(&self, _u: &Vector2f, _position: &SurfaceIntersection) -> Vector3f {
        let len = self.direction.norm();
        if len <= 0.0 {
            Vector3f::zeros()
        } else {
            -(self.direction / len)
        }
    }

    fn pdf_position(&self, _position: &SurfaceIntersection) -> Float {
        0.0
    }

    fn pdf_direction(&self, _position: &SurfaceIntersection, direction: &Vector3f) -> Float {
        let _ = direction;
        0.0
    }
}
