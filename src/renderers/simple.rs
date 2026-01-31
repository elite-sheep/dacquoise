// Copyright @yucwang 2021

use crate::core::computation_node::ComputationNode;
use crate::core::integrator::Integrator;
use crate::core::rng::LcgRng;
use crate::core::scene::Scene;
use crate::math::bitmap::Bitmap;
use crate::math::constants::{Float, Vector2f, Vector3f};
use indicatif::{ProgressBar, ProgressStyle};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{mpsc, Arc};
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

        let next_block = Arc::new(AtomicUsize::new(0));
        let thread_count = thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        let (tx, rx) = mpsc::channel::<(usize, usize, usize, usize, Vec<Vector3f>)>();
        let mut output = vec![Vector3f::zeros(); width * height];

        thread::scope(|scope| {
            for _ in 0..thread_count {
                let next_block = Arc::clone(&next_block);
                let tx = tx.clone();
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
                                let seed = ((self.seed & 0xFFF) << 32)
                                    | (((y as u64) & 0xFFFF) << 16)
                                    | ((x as u64) & 0xFFFF);
                                let mut rng = LcgRng::new(seed);
                                for _sample in 0..spp {
                                    let rgb = integrator_ref.trace_ray_forward(scene_ref, sensor_ref, pixel, &mut rng);
                                    color += Vector3f::new(rgb[0], rgb[1], rgb[2]);
                                }
                                let local_x = x - x0;
                                let local_y = y - y0;
                                block[local_x + (x1 - x0) * local_y] = color * inv_spp;
                            }
                        }
                        if tx.send((x0, y0, x1, y1, block)).is_err() {
                            break;
                        }
                    }
                });
            }

            drop(tx);
            for _ in 0..total_blocks {
                if let Ok((x0, y0, x1, y1, block)) = rx.recv() {
                    for y in y0..y1 {
                        for x in x0..x1 {
                            let local_x = x - x0;
                            let local_y = y - y0;
                            output[x + width * y] = block[local_x + (x1 - x0) * local_y];
                        }
                    }
                    progress.inc(1);
                }
            }
        });
        progress.finish_and_clear();
        let bitmap = sensor.bitmap_mut();
        for y in 0..height {
            for x in 0..width {
                bitmap[(x, y)] = output[x + width * y];
            }
        }
        let bitmap = bitmap.clone();
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
