// Copyright @yucwang 2026

use crate::core::scene::Scene;
use crate::core::rng::LcgRng;
use crate::core::sensor::Sensor;
use crate::math::constants::Vector2f;
use crate::math::spectrum::RGBSpectrum;

pub trait Integrator: Sync {
    fn trace_ray_forward(&self, scene: &Scene, sensor: &dyn Sensor, pixel: Vector2f, rng: &mut LcgRng) -> RGBSpectrum;
    fn samples_per_pixel(&self) -> u32;
}
