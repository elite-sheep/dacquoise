// Copyright @yucwang 2026

use crate::core::computation_node::ComputationNode;
use crate::core::emitter::{Emitter, EmitterFlag};
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::shape::Shape;
use crate::core::tangent_frame::{build_tangent_frame, local_to_world};
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::{sample_cosine_hemisphere, sample_cosine_hemisphere_pdf};
use crate::shapes::triangle::Triangle;
use std::sync::Arc;

pub struct AreaEmitter {
    shape: Arc<dyn Shape>,
    radiance: RGBSpectrum,
}

impl AreaEmitter {
    pub fn from_shape(shape: Arc<dyn Shape>, radiance: RGBSpectrum) -> Self {
        Self { shape, radiance }
    }
}

impl ComputationNode for AreaEmitter {
    fn to_string(&self) -> String {
        String::from("AreaEmitter")
    }
}

impl Emitter for AreaEmitter {
    fn new() -> Self {
        Self {
            shape: Arc::new(Triangle::new(
                Vector3f::new(0.0, 0.0, 0.0),
                Vector3f::new(1.0, 0.0, 0.0),
                Vector3f::new(0.0, 1.0, 0.0),
            )),
            radiance: RGBSpectrum::default(),
        }
    }

    fn get_flag(&self) -> EmitterFlag {
        EmitterFlag::SURFACE
    }

    fn sample_position(&self, u: &Vector2f) -> SurfaceSampleRecord {
        let sample = self.shape.sample(u);
        let intersection = sample.intersection().with_le(self.radiance);
        SurfaceSampleRecord::new(intersection, sample.pdf())
    }

    fn sample_direction(&self, u: &Vector2f, position: &SurfaceIntersection) -> Vector3f {
        let local_dir = sample_cosine_hemisphere(u);
        let n = position.geo_normal();
        let (tangent, bitangent) = build_tangent_frame(&n);
        local_to_world(&local_dir, &tangent, &bitangent, &n)
    }

    fn pdf_position(&self, _position: &SurfaceIntersection) -> Float {
        let area = self.shape.surface_area();
        if area > 0.0 {
            1.0 / area.max(1e-6)
        } else {
            0.0
        }
    }

    fn pdf_direction(&self, _position: &SurfaceIntersection, _direction: &Vector3f) -> Float {
        let dir_len = _direction.norm();
        if dir_len <= 0.0 {
            return 0.0;
        }
        let dir = _direction / dir_len;
        let cos_theta = _position.geo_normal().dot(&dir).max(0.0);
        if cos_theta > 0.0 {
            sample_cosine_hemisphere_pdf(cos_theta)
        } else {
            0.0
        }
    }
}
