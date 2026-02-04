// Copyright @yucwang 2026

use crate::core::medium::Medium;
use crate::core::volume::Volume;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use std::sync::Arc;

pub struct HeterogeneousMedium {
    sigma_t_volume: Arc<dyn Volume>,
    albedo_volume: Arc<dyn Volume>,
    scale: Float,
    bbox: Option<AABB>,
}

impl HeterogeneousMedium {
    pub fn new(sigma_t_volume: Arc<dyn Volume>, albedo_volume: Arc<dyn Volume>) -> Self {
        let bbox = union_bbox(sigma_t_volume.bbox(), albedo_volume.bbox());
        Self {
            sigma_t_volume,
            albedo_volume,
            scale: 1.0,
            bbox,
        }
    }

    pub fn with_scale(mut self, scale: Float) -> Self {
        self.scale = scale;
        self
    }
}

impl Medium for HeterogeneousMedium {
    fn sigma_t(&self, p_world: Vector3f) -> RGBSpectrum {
        let v = self.sigma_t_volume.eval(p_world);
        let rgb = RGBSpectrum::new(v.x, v.y, v.z);
        rgb * self.scale
    }

    fn albedo(&self, p_world: Vector3f) -> RGBSpectrum {
        let v = self.albedo_volume.eval(p_world);
        clamp_spectrum(RGBSpectrum::new(v.x, v.y, v.z))
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

fn union_bbox(a: Option<AABB>, b: Option<AABB>) -> Option<AABB> {
    match (a, b) {
        (Some(mut aabb), Some(other)) => {
            aabb.expand_by_aabb(&other);
            Some(aabb)
        }
        (Some(aabb), None) => Some(aabb),
        (None, Some(aabb)) => Some(aabb),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::volumes::const_volume::ConstantVolume;

    #[test]
    fn heterogeneous_medium_scale_and_clamp() {
        let sigma_t = Arc::new(ConstantVolume::new_scalar(0.5));
        let albedo = Arc::new(ConstantVolume::new_rgb(Vector3f::new(2.0, -1.0, 0.5)));
        let medium = HeterogeneousMedium::new(sigma_t, albedo).with_scale(2.0);

        let sigma = medium.sigma_t(Vector3f::new(0.0, 0.0, 0.0));
        assert_eq!(sigma, RGBSpectrum::new(1.0, 1.0, 1.0));

        let alb = medium.albedo(Vector3f::new(0.0, 0.0, 0.0));
        assert_eq!(alb, RGBSpectrum::new(1.0, 0.0, 0.5));
    }

    #[test]
    fn heterogeneous_medium_bbox_union() {
        let bbox_a = AABB::new(Vector3f::new(-1.0, 0.0, 2.0), Vector3f::new(1.0, 1.0, 3.0));
        let bbox_b = AABB::new(Vector3f::new(-2.0, -1.0, 0.0), Vector3f::new(0.0, 2.0, 4.0));
        let sigma_t = Arc::new(ConstantVolume::new_scalar(1.0).with_bbox(Some(bbox_a)));
        let albedo = Arc::new(ConstantVolume::new_scalar(1.0).with_bbox(Some(bbox_b)));
        let medium = HeterogeneousMedium::new(sigma_t, albedo);

        let out = medium.bbox().expect("bbox");
        assert_eq!(out.p_min, Vector3f::new(-2.0, -1.0, 0.0));
        assert_eq!(out.p_max, Vector3f::new(1.0, 2.0, 4.0));
    }
}
