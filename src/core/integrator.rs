// Copyright @yucwang 2026

use crate::core::scene::Scene;
use crate::core::sensor::Sensor;

pub trait Integrator {
    fn render_forward(&self, scene: &Scene, sensor: &mut dyn Sensor, seed: u64);
}
