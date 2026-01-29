// Copyright @yucwang 2026

use crate::core::integrator::Integrator;
use crate::core::scene::Scene;
use crate::core::sensor::Sensor;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::Spectrum;

struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u32(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (self.state >> 32) as u32
    }

    fn next_f32(&mut self) -> Float {
        (self.next_u32() as Float) / (u32::MAX as Float)
    }
}

pub struct PathIntegrator {
    pub max_depth: u32,
    pub samples_per_pixel: u32,
}

impl PathIntegrator {
    pub fn new(max_depth: u32, samples_per_pixel: u32) -> Self {
        Self { max_depth, samples_per_pixel }
    }
}

impl Integrator for PathIntegrator {
    fn render_forward(&self, scene: &Scene, sensor: &mut dyn Sensor, seed: u64) {
        let (width, height) = {
            let bmp = sensor.bitmap();
            (bmp.width(), bmp.height())
        };

        let spp = if self.samples_per_pixel == 0 { 1 } else { self.samples_per_pixel };
        let inv_spp = 1.0 / (spp as Float);
        let mut rng = LcgRng::new(seed);

        for y in 0..height {
            for x in 0..width {
                let mut color = Vector3f::zeros();
                for _ in 0..spp {
                    let u = (x as Float + rng.next_f32()) / (width as Float);
                    let v = (y as Float + rng.next_f32()) / (height as Float);
                    let ray = sensor.sample_ray(&Vector2f::new(u, v));
                    color += self.trace_path(scene, ray, &mut rng);
                }
                sensor.bitmap_mut()[(x, y)] = color * inv_spp;
            }
        }
    }
}

impl PathIntegrator {
    fn trace_path(&self, scene: &Scene, mut ray: crate::math::ray::Ray3f, rng: &mut LcgRng) -> Vector3f {
        let mut radiance = Vector3f::zeros();
        let mut throughput = Vector3f::new(1.0, 1.0, 1.0);
        let mut prev_bsdf_pdf: Float = 0.0;

        for bounce in 0..self.max_depth {
            let intersection = match scene.ray_intersection(&ray) {
                Some(h) => h,
                None => break,
            };

            let le = intersection.le();
            if le.is_black() == false {
                if bounce == 0 {
                    radiance += throughput.component_mul(&Vector3f::new(le[0], le[1], le[2]));
                } else if let Some(light_pdf_area) = intersection.light_pdf_area() {
                    let p = intersection.p();
                    let dist2 = (p - ray.origin()).dot(&(p - ray.origin()));
                    let cos_light = intersection.geo_normal().dot(&(-ray.dir())).max(0.0);
                    if cos_light > 0.0 && dist2 > 0.0 {
                        let light_pdf = light_pdf_area * dist2 / cos_light;
                        let weight = power_heuristic(prev_bsdf_pdf, light_pdf);
                        radiance += throughput.component_mul(&Vector3f::new(le[0], le[1], le[2])) * weight;
                    }
                }
            }

            let n = intersection.geo_normal();
            let (tangent, bitangent) = build_tangent_frame(&n);

            let wo_world = -ray.dir();
            let wo_local = world_to_local(&wo_world, &tangent, &bitangent, &n);

            let material = match intersection.material() {
                Some(m) => m,
                None => break,
            };

            // Next Event Estimation (direct lighting), skip if this is the last bounce
            if bounce + 1 < self.max_depth {
                if let Some(light_sample) = scene.sample_emitter(rng.next_f32(), &Vector2f::new(rng.next_f32(), rng.next_f32())) {
                let light_intersection = light_sample.intersection();
                let light_le = light_intersection.le();
                if light_le.is_black() == false {
                    let p = intersection.p();
                    let p_light = light_intersection.p();
                    let to_light = p_light - p;
                    let dist2 = to_light.dot(&to_light);
                    if dist2 > 0.0 {
                        let dist = dist2.sqrt();
                        let wi_world = to_light / dist;
                        let cos_light = light_intersection.geo_normal().dot(&(-wi_world)).max(0.0);

                        if cos_light > 0.0 {
                            let shadow_ray = crate::math::ray::Ray3f::new(
                                p + n * 1e-3,
                                wi_world,
                                Some(1e-3),
                                None,
                            );

                            let shadow_hit = scene.ray_intersection(&shadow_ray);
                            let is_occluded = match shadow_hit {
                                Some(ref h) => {
                                    let hit_p = h.p();
                                    let near_light = (hit_p - p_light).norm() < 1e-3;
                                    !near_light && h.t() < dist - 1e-3
                                }
                                None => false,
                            };

                            if !is_occluded {
                                let wi_local = world_to_local(&wi_world, &tangent, &bitangent, &n);
                                let mut eval_record = crate::core::bsdf::BSDFSampleRecord::default();
                                eval_record.wi = wi_local;
                                eval_record.wo = wo_local;
                                eval_record.pdf = 0.0;
                                let eval = material.eval(eval_record);
                                let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
                                let cos_theta = wi_local.z.abs();

                                let light_pdf_area = light_sample.pdf();
                                let light_pdf = light_pdf_area * dist2 / cos_light;
                                if light_pdf > 0.0 {
                                    let bsdf_pdf = eval.pdf.max(0.0);
                                    let weight = power_heuristic(light_pdf, bsdf_pdf);
                                    let le_vec = Vector3f::new(light_le[0], light_le[1], light_le[2]);
                                    let contrib = throughput.component_mul(&f) * (cos_theta / light_pdf) * weight;
                                    let direct = contrib.component_mul(&le_vec);
                                    radiance += direct;
                                }
                            }
                        }
                    }
                }
            }
            }

            let u1 = Vector2f::new(rng.next_f32(), rng.next_f32());
            let u2 = Vector2f::new(rng.next_f32(), rng.next_f32());
            let sample = material.sample(u1, u2, wo_local);
            let wi_local = sample.wi;
            let pdf = sample.pdf;

            if pdf <= 0.0 {
                break;
            }

            let eval = material.eval(sample);
            let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
            let cos_theta = wi_local.z.abs();
            let bsdf_weight = cos_theta / pdf;
            throughput = throughput.component_mul(&f) * bsdf_weight;

            if throughput.x <= 0.0 && throughput.y <= 0.0 && throughput.z <= 0.0 {
                break;
            }

            prev_bsdf_pdf = pdf;
            let wi_world = local_to_world(&wi_local, &tangent, &bitangent, &n);
            let origin = intersection.p() + n * 1e-3;
            ray = crate::math::ray::Ray3f::new(origin, wi_world, Some(1e-3), None);
        }

        radiance
    }
}

fn build_tangent_frame(n: &Vector3f) -> (Vector3f, Vector3f) {
    let up = if n.z.abs() < 0.999 {
        Vector3f::new(0.0, 0.0, 1.0)
    } else {
        Vector3f::new(1.0, 0.0, 0.0)
    };
    let tangent = n.cross(&up).normalize();
    let bitangent = n.cross(&tangent).normalize();
    (tangent, bitangent)
}

fn world_to_local(v: &Vector3f, t: &Vector3f, b: &Vector3f, n: &Vector3f) -> Vector3f {
    Vector3f::new(v.dot(t), v.dot(b), v.dot(n))
}

fn local_to_world(v: &Vector3f, t: &Vector3f, b: &Vector3f, n: &Vector3f) -> Vector3f {
    t * v.x + b * v.y + n * v.z
}

fn power_heuristic(pdf_a: Float, pdf_b: Float) -> Float {
    let a2 = pdf_a * pdf_a;
    let b2 = pdf_b * pdf_b;
    if a2 + b2 == 0.0 {
        0.0
    } else {
        a2 / (a2 + b2)
    }
}
