// Copyright @yucwang 2026

use crate::math::aabb::AABB;
use crate::math::constants::Vector3f;
use crate::math::spectrum::RGBSpectrum;

pub trait Medium: Send + Sync {
    fn sigma_t(&self, p_world: Vector3f) -> RGBSpectrum;
    fn albedo(&self, p_world: Vector3f) -> RGBSpectrum;

    fn sigma_s(&self, p_world: Vector3f) -> RGBSpectrum {
        self.sigma_t(p_world) * self.albedo(p_world)
    }

    fn sigma_a(&self, p_world: Vector3f) -> RGBSpectrum {
        let one = RGBSpectrum::new(1.0, 1.0, 1.0);
        self.sigma_t(p_world) * (one - self.albedo(p_world))
    }

    fn bbox(&self) -> Option<AABB> {
        None
    }
}
