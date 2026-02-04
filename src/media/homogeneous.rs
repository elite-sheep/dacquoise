// Copyright @yucwang 2026

use crate::core::medium::Medium;
use crate::core::volume::Volume;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use std::sync::Arc;

pub struct HomogeneousMedium {
    sigma_t: RGBSpectrum,
    albedo: RGBSpectrum,
    albedo_volume: Option<Arc<dyn Volume>>,
    scale: Float,
    bbox: Option<AABB>,
}

impl HomogeneousMedium {
    pub fn new(sigma_t: RGBSpectrum, albedo: RGBSpectrum) -> Self {
        Self {
            sigma_t,
            albedo,
            albedo_volume: None,
            scale: 1.0,
            bbox: None,
        }
    }

    pub fn with_albedo_volume(mut self, volume: Arc<dyn Volume>) -> Self {
        self.albedo_volume = Some(volume);
        self
    }

    pub fn with_scale(mut self, scale: Float) -> Self {
        self.scale = scale;
        self
    }

    pub fn with_bbox(mut self, bbox: Option<AABB>) -> Self {
        self.bbox = bbox;
        self
    }
}

impl Medium for HomogeneousMedium {
    fn sigma_t(&self, _p_world: Vector3f) -> RGBSpectrum {
        self.sigma_t * self.scale
    }

    fn albedo(&self, p_world: Vector3f) -> RGBSpectrum {
        let raw = if let Some(volume) = &self.albedo_volume {
            let v = volume.eval(p_world);
            RGBSpectrum::new(v.x, v.y, v.z)
        } else {
            self.albedo
        };
        clamp_spectrum(raw)
    }

    fn bbox(&self) -> Option<AABB> {
        self.bbox
    }
}

fn clamp_spectrum(value: RGBSpectrum) -> RGBSpectrum {
    RGBSpectrum::new(
        value[0].clamp(0.0, 1.0),
        value[1].clamp(0.0, 1.0),
        value[2].clamp(0.0, 1.0),
    )
}
