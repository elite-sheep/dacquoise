// Copyright @yucwang 2021

use crate::core::computation_node::ComputationNode;
use crate::core::integrator::Integrator;
use crate::core::scene::Scene;
use crate::math::bitmap::Bitmap;

pub use super::renderer::Renderer;

pub struct SimpleRenderer {
    integrator: Box<dyn Integrator>,
    camera_id: usize,
    seed: u64,
}

impl ComputationNode for SimpleRenderer {
    fn to_string(&self) -> String {
        String::from("SimpleRenderer: {}")
    }
}

impl Renderer for SimpleRenderer {
    fn render(&self, scene: &mut Scene) -> Bitmap {
        let mut sensor = match scene.take_sensor(self.camera_id) {
            Some(sensor) => sensor,
            None => return Bitmap::new(0, 0),
        };

        self.integrator.render_forward(scene, sensor.as_mut(), self.seed);
        let bitmap = sensor.bitmap().clone();
        scene.insert_sensor(self.camera_id, sensor);
        bitmap
    }
}

impl SimpleRenderer {
    pub fn new(integrator: Box<dyn Integrator>, camera_id: usize, seed: u64) -> Self {
        Self {
            integrator,
            camera_id,
            seed,
        }
    }
}
