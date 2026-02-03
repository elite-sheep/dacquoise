// Copyright @yucwang 2026

use crate::core::integrator::Integrator;
use crate::core::rng::LcgRng;
use crate::core::scene::Scene;
use crate::core::sensor::Sensor;
use crate::core::tangent_frame::{build_tangent_frame, local_to_world, world_to_local};
use crate::core::bsdf::BSDFSampleRecord;
use crate::core::emitter::EmitterSample;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::ray::Ray3f;
use crate::math::spectrum::{RGBSpectrum, Spectrum};

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
    fn trace_ray_forward(&self, scene: &Scene, sensor: &dyn Sensor, pixel: Vector2f, rng: &mut LcgRng) -> RGBSpectrum {
        let (width, height) = {
            let bmp = sensor.bitmap();
            (bmp.width(), bmp.height())
        };
        if width == 0 || height == 0 {
            return RGBSpectrum::default();
        }

        let px = pixel.x;
        let py = pixel.y;
        let u = (px + rng.next_f32()) / (width as Float);
        let v = (py + rng.next_f32()) / (height as Float);
        let mut ray = sensor.sample_ray(&Vector2f::new(u, v));
        let mut radiance = Vector3f::zeros();
        let mut throughput = Vector3f::new(1.0, 1.0, 1.0);
        let mut prev_bsdf_pdf: Float = 0.0;

        for bounce in 0..self.max_depth {
            let intersection = match scene.ray_intersection(&ray) {
                Some(h) => h,
                None => {
                    let mut env = RGBSpectrum::default();
                    for emitter in scene.emitters() {
                        env += emitter.eval_direction(&ray.dir());
                    }
                    if env.is_black() == false {
                        radiance += throughput.component_mul(&Vector3f::new(env[0], env[1], env[2]));
                    }
                    break;
                }
            };

            let le = intersection.le();
            if le.is_black() == false {
                let cos_light_hit = intersection.geo_normal().dot(&(-ray.dir()));
                if cos_light_hit <= 0.0 {
                    // One-sided emitters: ignore emission from the back face.
                    // Continue path, since the light might still be hittable by BSDF sampling.
                } else
                if bounce == 0 {
                    radiance += throughput.component_mul(&Vector3f::new(le[0], le[1], le[2]));
                } else if let Some(light_pdf_area) = intersection.light_pdf_area() {
                    let p = intersection.p();
                    let dist2 = (p - ray.origin()).dot(&(p - ray.origin()));
                    let cos_light = cos_light_hit.max(0.0);
                    if cos_light > 0.0 && dist2 > 0.0 {
                        let light_pdf = light_pdf_area * dist2 / cos_light;
                        let weight = power_heuristic(prev_bsdf_pdf, light_pdf);
                        radiance += throughput.component_mul(&Vector3f::new(le[0], le[1], le[2])) * weight;
                    }
                }
            }

            let n_geo = intersection.geo_normal();
            let n_sh = intersection.sh_normal();

            let wi_world = -ray.dir();
            let (tangent, bitangent) = build_tangent_frame(&n_sh);
            let wi_local = world_to_local(&wi_world, &tangent, &bitangent, &n_sh);

            let material = match intersection.material() {
                Some(m) => m,
                None => break,
            };


            // Next Event Estimation (direct lighting), skip if this is the last bounce
            if bounce + 1 < self.max_depth {
                if let Some(light_sample) = scene.sample_emitter(
                    rng.next_f32(),
                    &Vector2f::new(rng.next_f32(), rng.next_f32()),
                ) {
                    match light_sample {
                        EmitterSample::Surface(light_sample) => {
                            let light_intersection = light_sample.intersection();
                            let light_le = light_intersection.le();
                            if light_le.is_black() == false {
                                let p = intersection.p();
                                let p_light = light_intersection.p();
                                let to_light = p_light - p;
                                let dist2 = to_light.dot(&to_light);
                                if dist2 > 0.0 {
                                    let dist = dist2.sqrt();
                                    let wo_world = to_light / dist;
                                    let cos_light = light_intersection.geo_normal().dot(&(-wo_world)).max(0.0);

                                    if cos_light > 0.0 {
                                        let shadow_ray = Ray3f::new(
                                            p + n_geo * 1e-3,
                                            wo_world,
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
                                            let wo_local = world_to_local(&wo_world, &tangent, &bitangent, &n_sh);
                                            let mut eval_record = BSDFSampleRecord::default();
                                            eval_record.wi = wi_local;
                                            eval_record.wo = wo_local;
                                            eval_record.pdf = 0.0;
                                            eval_record.uv = intersection.uv();
                                            let eval = material.eval(eval_record);
                                            let mut f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
                                            let cos_theta = wo_local.z.abs();
                                            let corr = shading_normal_correction(
                                                wi_world,
                                                wo_world,
                                                wi_local,
                                                wo_local,
                                                n_geo,
                                            );
                                            if let Some(corr) = corr {
                                                f *= corr;
                                            } else {
                                                continue;
                                            }

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
                        EmitterSample::Direction { direction, irradiance, pdf, is_delta } => {
                            let dir_len = direction.norm();
                            if dir_len > 0.0 && pdf > 0.0 && !irradiance.is_black() {
                                let wo_world = direction / dir_len;
                                    let shadow_ray = Ray3f::new(
                                        intersection.p() + n_geo * 1e-3,
                                        wo_world,
                                        Some(1e-3),
                                        None,
                                    );
                                if !scene.ray_intersection_t(&shadow_ray) {
                                    let wo_local = world_to_local(&wo_world, &tangent, &bitangent, &n_sh);
                                    let mut eval_record = BSDFSampleRecord::default();
                                    eval_record.wi = wi_local;
                                    eval_record.wo = wo_local;
                                    eval_record.pdf = 0.0;
                                    eval_record.uv = intersection.uv();
                                    let eval = material.eval(eval_record);
                                    let mut f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
                                    let cos_theta = wo_local.z.abs();
                                    let corr = shading_normal_correction(
                                        wi_world,
                                        wo_world,
                                        wi_local,
                                        wo_local,
                                        n_geo,
                                    );
                                    if let Some(corr) = corr {
                                        f *= corr;
                                    } else {
                                        continue;
                                    }
                                    if cos_theta > 0.0 {
                                        let bsdf_pdf = eval.pdf.max(0.0);
                                        let weight = if is_delta { 1.0 } else { power_heuristic(pdf, bsdf_pdf) };
                                        let le_vec = Vector3f::new(irradiance[0], irradiance[1], irradiance[2]);
                                        let contrib = throughput.component_mul(&f) * (cos_theta / pdf) * weight;
                                        radiance += contrib.component_mul(&le_vec);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            let u1 = Vector2f::new(rng.next_f32(), rng.next_f32());
            let u2 = Vector2f::new(rng.next_f32(), rng.next_f32());
            let (next_ray, bsdf_weight, bsdf_pdf) = match compute_scatter_ray(
                material,
                u1,
                u2,
                wi_local,
                intersection.uv(),
                intersection.p(),
                n_sh,
                n_geo,
                &tangent,
                &bitangent,
            ) {
                Some(v) => v,
                None => break,
            };
            throughput = throughput.component_mul(&bsdf_weight);

            if throughput.x <= 0.0 && throughput.y <= 0.0 && throughput.z <= 0.0 {
                break;
            }

            let rr_depth: u32 = 5;
            if bounce >= rr_depth {
                let max_comp = throughput.x.max(throughput.y).max(throughput.z);
                let survival = max_comp.min(0.95).max(0.05);
                if rng.next_f32() > survival {
                    break;
                }
                throughput /= survival;
            }

            prev_bsdf_pdf = bsdf_pdf;
            ray = next_ray;
        }

        RGBSpectrum::new(radiance[0], radiance[1], radiance[2])
    }

    fn samples_per_pixel(&self) -> u32 {
        self.samples_per_pixel
    }
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

fn shading_normal_correction(
    wi_world: Vector3f,
    wo_world: Vector3f,
    wi_local: Vector3f,
    wo_local: Vector3f,
    n_geo: Vector3f,
) -> Option<Float> {
    let wi_dot_geo = wi_world.dot(&n_geo);
    let wo_dot_geo = wo_world.dot(&n_geo);
    let wi_cos = wi_local.z;
    let wo_cos = wo_local.z;
    if wi_dot_geo * wi_cos <= 0.0 || wo_dot_geo * wo_cos <= 0.0 {
        return None;
    }
    let denom = wo_cos * wi_dot_geo;
    if denom.abs() <= 1e-8 {
        return None;
    }
    Some(((wi_cos * wo_dot_geo) / denom).abs())
}

fn compute_scatter_ray(
    material: &dyn crate::core::bsdf::BSDF,
    u1: Vector2f,
    u2: Vector2f,
    wi_local: Vector3f,
    uv: Vector2f,
    p: Vector3f,
    n_sh: Vector3f,
    n_geo: Vector3f,
    tangent: &Vector3f,
    bitangent: &Vector3f,
) -> Option<(Ray3f, Vector3f, Float)> {
    let mut sample = material.sample(u1, u2, wi_local);
    sample.uv = uv;
    let pdf = sample.pdf;
    if pdf <= 0.0 {
        return None;
    }

    let wo_local = sample.wo;
    let eval = material.eval(sample);
    let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
    let cos_theta = wo_local.z.abs();
    let mut bsdf_weight = f * (cos_theta / pdf);

    let wi_world = local_to_world(&wi_local, tangent, bitangent, &n_sh);
    let wo_world = local_to_world(&wo_local, tangent, bitangent, &n_sh);

    let wi_dot_geo = wi_world.dot(&n_geo);
    let wo_dot_geo = wo_world.dot(&n_geo);
    let wi_cos = wi_local.z;
    let wo_cos = wo_local.z;
    if wi_dot_geo * wi_cos <= 0.0 || wo_dot_geo * wo_cos <= 0.0 {
        return None;
    }

    let denom = wo_cos * wi_dot_geo;
    if denom.abs() > 1e-8 {
        let correction = (wi_cos * wo_dot_geo / denom).abs();
        bsdf_weight *= correction;
    }

    let offset_dir = if wo_world.dot(&n_geo) >= 0.0 { n_geo } else { -n_geo };
    let origin = p + offset_dir * 1e-6;
    Some((Ray3f::new(origin, wo_world, Some(1e-4), None), bsdf_weight, pdf))
}
