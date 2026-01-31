// Copyright @yucwang 2021

use crate::core::computation_node::ComputationNode;
use crate::core::integrator::Integrator;
use crate::core::scene::Scene;
use crate::math::bitmap::Bitmap;
use crate::math::constants::{Float, Vector2f, Vector3f};
use indicatif::{ProgressBar, ProgressStyle};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;

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

        let (width, height) = {
            let bmp = sensor.bitmap();
            (bmp.width(), bmp.height())
        };
        if width == 0 || height == 0 {
            return Bitmap::new(0, 0);
        }
        let spp = match self.integrator.samples_per_pixel() {
            0 => 1,
            v => v,
        };
        let inv_spp = 1.0 / (spp as Float);

        let block_size = 128usize;
        let blocks_x = (width + block_size - 1) / block_size;
        let blocks_y = (height + block_size - 1) / block_size;
        let total_blocks = blocks_x * blocks_y;
        let scene_ref: &Scene = scene;
        let sensor_ref: &dyn crate::core::sensor::Sensor = sensor.as_ref();
        let integrator_ref: &dyn Integrator = self.integrator.as_ref();

        let progress = ProgressBar::new(total_blocks as u64);
        progress.set_style(
            ProgressStyle::with_template("[{elapsed_precise}] {bar:40.cyan/blue} {pos}/{len} blocks")
                .unwrap_or_else(|_| ProgressStyle::default_bar()),
        );

        let output = Arc::new(Mutex::new(Bitmap::new(width, height)));
        let next_block = Arc::new(AtomicUsize::new(0));
        let thread_count = thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);

        thread::scope(|scope| {
            for _ in 0..thread_count {
                let output = Arc::clone(&output);
                let next_block = Arc::clone(&next_block);
                let progress = progress.clone();
                scope.spawn(move || {
                    loop {
                        let block_index = next_block.fetch_add(1, Ordering::Relaxed);
                        if block_index >= total_blocks {
                            break;
                        }

                        let bx = block_index % blocks_x;
                        let by = block_index / blocks_x;
                        let x0 = bx * block_size;
                        let y0 = by * block_size;
                        let x1 = (x0 + block_size).min(width);
                        let y1 = (y0 + block_size).min(height);

                        let mut block = vec![Vector3f::zeros(); (x1 - x0) * (y1 - y0)];
                        for y in y0..y1 {
                            for x in x0..x1 {
                                let mut color = Vector3f::zeros();
                                let pixel = Vector2f::new(x as Float, y as Float);
                                for sample in 0..spp {
                                    let seed = self.seed
                                        .wrapping_add((y as u64).wrapping_mul(width as u64))
                                        .wrapping_add(x as u64)
                                        .wrapping_mul(sample as u64);
                                    let rgb = integrator_ref.trace_ray_forward(scene_ref, sensor_ref, pixel, seed);
                                    color += Vector3f::new(rgb[0], rgb[1], rgb[2]);
                                }
                                let local_x = x - x0;
                                let local_y = y - y0;
                                block[local_x + (x1 - x0) * local_y] = color * inv_spp;
                            }
                        }

                        let mut bitmap = match output.lock() {
                            Ok(guard) => guard,
                            Err(poisoned) => poisoned.into_inner(),
                        };
                        for y in y0..y1 {
                            for x in x0..x1 {
                                let local_x = x - x0;
                                let local_y = y - y0;
                                bitmap[(x, y)] = block[local_x + (x1 - x0) * local_y];
                            }
                        }
                        progress.inc(1);
                    }
                });
            }
        });
        progress.finish_and_clear();
        let bitmap = match Arc::try_unwrap(output) {
            Ok(mutex) => match mutex.into_inner() {
                Ok(bmp) => bmp,
                Err(poisoned) => poisoned.into_inner(),
            },
            Err(arc) => arc.lock().map(|bmp| bmp.clone()).unwrap_or_else(|e| e.into_inner().clone()),
        };
        *sensor.bitmap_mut() = bitmap.clone();
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
