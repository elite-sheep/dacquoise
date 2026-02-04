// Copyright @yucwang 2026

use crate::core::bsdf::BSDFSampleRecord;
use crate::core::emitter::EmitterSample;
use crate::core::integrator::Integrator;
use crate::core::medium::Medium;
use crate::core::rng::LcgRng;
use crate::core::scene::Scene;
use crate::core::sensor::Sensor;
use crate::core::tangent_frame::{build_tangent_frame, local_to_world, world_to_local};
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::ray::Ray3f;
use crate::math::spectrum::{RGBSpectrum, Spectrum};
use std::sync::Arc;

pub struct RaymarchingIntegrator {
    pub max_depth: u32,
    pub samples_per_pixel: u32,
    pub step_size: Option<Float>,
}

impl RaymarchingIntegrator {
    pub fn new(max_depth: u32, samples_per_pixel: u32, step_size: Option<Float>) -> Self {
        Self { max_depth, samples_per_pixel, step_size }
    }
}

impl Integrator for RaymarchingIntegrator {
    fn trace_ray_forward(&self, scene: &Scene, sensor: &dyn Sensor, pixel: Vector2f, rng: &mut LcgRng) -> RGBSpectrum {
        let (width, height) = {
            let bmp = sensor.bitmap();
            (bmp.width(), bmp.height())
        };
        if width == 0 || height == 0 {
            return RGBSpectrum::default();
        }

        let u = (pixel.x + rng.next_f32()) / (width as Float);
        let v = (pixel.y + rng.next_f32()) / (height as Float);
        let ray = sensor.sample_ray(&Vector2f::new(u, v));

        // No global medium: only handle participating media bound to shapes.
        match scene.ray_intersection(&ray) {
            None => rgb_from_vec(env_radiance(scene, &ray)),
            Some(hit) => {
                let material = match hit.material() {
                    Some(m) => m,
                    None => return RGBSpectrum::default(),
                };

                if material.is_null() {
                    if let Some((medium, t_entry, t_exit)) = interior_medium_segment(scene, &ray, &hit) {
                        let (vol_radiance, beta) = march_nerf_segment(
                            &ray,
                            t_entry,
                            t_exit,
                            medium.as_ref(),
                            rng,
                            self.max_depth,
                            self.step_size,
                        );
                        let mut color = vol_radiance;

                        // Continue tracing after the medium boundary.
                        let new_origin = ray.at(t_exit + 1e-4);
                        let exit_ray = Ray3f::new(new_origin, ray.dir(), Some(0.0), None);
                        if let Some(hit2) = scene.ray_intersection(&exit_ray) {
                            let surf = shade_surface_basic(scene, &exit_ray, &hit2, rng);
                            color += beta.component_mul(&surf);
                        } else {
                            let env = env_radiance(scene, &exit_ray);
                            color += beta.component_mul(&env);
                        }

                        return rgb_from_vec(color);
                    }
                }

                rgb_from_vec(shade_surface_basic(scene, &ray, &hit, rng))
            }
        }
    }

    fn samples_per_pixel(&self) -> u32 {
        self.samples_per_pixel
    }
}

fn march_nerf_segment(
    ray: &Ray3f,
    t0: Float,
    t1: Float,
    medium: &dyn Medium,
    rng: &mut LcgRng,
    max_depth: u32,
    step_size: Option<Float>,
) -> (Vector3f, Vector3f) {
    let max_steps = max_depth.max(1) as usize;
    let length = t1 - t0;
    if length <= 0.0 || max_steps == 0 {
        return (Vector3f::zeros(), Vector3f::new(1.0, 1.0, 1.0));
    }

    let (dt, steps) = if let Some(step) = step_size {
        if step > 0.0 {
            let needed = (length / step).ceil() as usize;
            let steps = needed.max(1).min(max_steps);
            (step, steps)
        } else {
            (length / (max_steps as Float), max_steps)
        }
    } else {
        (length / (max_steps as Float), max_steps)
    };
    let mut t = t0;
    let mut color = Vector3f::zeros();
    let mut beta = Vector3f::new(1.0, 1.0, 1.0);

    for _ in 0..steps {
        let jitter = rng.next_f32();
        let t_sample = (t + jitter * dt).min(t1);
        let p = ray.at(t_sample);

        let (sigma, c) = field_eval(medium, p, ray.dir());
        let alpha = 1.0 - (-sigma * dt).exp();
        let w = beta * alpha;
        color += w.component_mul(&c);
        beta *= 1.0 - alpha;

        if beta.x.max(beta.y).max(beta.z) < 1e-4 {
            break;
        }

        t += dt;
        if t >= t1 {
            break;
        }
    }

    (color, beta)
}

fn field_eval(medium: &dyn Medium, p: Vector3f, _dir: Vector3f) -> (Float, Vector3f) {
    let sigma_rgb = medium.sigma_t(p);
    let sigma = ((sigma_rgb[0] + sigma_rgb[1] + sigma_rgb[2]) / 3.0).max(0.0);
    let c = medium.albedo(p);
    (sigma, Vector3f::new(c[0], c[1], c[2]))
}

fn shade_surface_basic(
    scene: &Scene,
    ray: &Ray3f,
    hit: &crate::core::interaction::SurfaceIntersection,
    rng: &mut LcgRng,
) -> Vector3f {
    let mut radiance = Vector3f::zeros();

    let le = hit.le();
    if !le.is_black() {
        let cos_light_hit = hit.geo_normal().dot(&(-ray.dir()));
        if cos_light_hit > 0.0 {
            radiance += Vector3f::new(le[0], le[1], le[2]);
        }
    }

    let material = match hit.material() {
        Some(m) => m,
        None => return radiance,
    };
    if material.is_null() {
        return radiance;
    }

    let n_geo = hit.geo_normal();
    let n_sh = hit.sh_normal();
    let wi_world = -ray.dir();
    let (tangent, bitangent) = build_tangent_frame(&n_sh);
    let wi_local = world_to_local(&wi_world, &tangent, &bitangent, &n_sh);

    if let Some(light_sample) = scene.sample_emitter(
        rng.next_f32(),
        &Vector2f::new(rng.next_f32(), rng.next_f32()),
    ) {
        match light_sample {
            EmitterSample::Surface(light_sample) => {
                let light_intersection = light_sample.intersection();
                let light_le = light_intersection.le();
                if !light_le.is_black() {
                    let p = hit.p();
                    let p_light = light_intersection.p();
                    let to_light = p_light - p;
                    let dist2 = to_light.dot(&to_light);
                    if dist2 > 0.0 {
                        let dist = dist2.sqrt();
                        let wo_world = to_light / dist;
                        let cos_light = light_intersection.geo_normal().dot(&(-wo_world)).max(0.0);
                        if cos_light > 0.0 {
                            let shadow_origin = p + n_geo * 1e-3;
                            if shadow_visible_through_nulls(
                                scene,
                                shadow_origin,
                                wo_world,
                                Some(dist - 1e-3),
                                Some(p_light),
                            ) {
                                let wo_local = world_to_local(&wo_world, &tangent, &bitangent, &n_sh);
                                let mut eval_record = BSDFSampleRecord::default();
                                eval_record.wi = wi_local;
                                eval_record.wo = wo_local;
                                eval_record.pdf = 0.0;
                                eval_record.uv = hit.uv();
                                let eval = material.eval(eval_record);
                                let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
                                let cos_theta = wo_local.z.abs();
                                let light_pdf_area = light_sample.pdf();
                                let light_pdf = light_pdf_area * dist2 / cos_light;
                                if light_pdf > 0.0 {
                                    let le_vec = Vector3f::new(light_le[0], light_le[1], light_le[2]);
                                    let contrib = f * (cos_theta / light_pdf);
                                    radiance += contrib.component_mul(&le_vec);
                                }
                            }
                        }
                    }
                }
            }
            EmitterSample::Direction { direction, irradiance, pdf, .. } => {
                let dir_len = direction.norm();
                if dir_len > 0.0 && pdf > 0.0 && !irradiance.is_black() {
                    let wo_world = direction / dir_len;
                    let shadow_origin = hit.p() + n_geo * 1e-3;
                    if shadow_visible_through_nulls(scene, shadow_origin, wo_world, None, None) {
                        let wo_local = world_to_local(&wo_world, &tangent, &bitangent, &n_sh);
                        let mut eval_record = BSDFSampleRecord::default();
                        eval_record.wi = wi_local;
                        eval_record.wo = wo_local;
                        eval_record.pdf = 0.0;
                        eval_record.uv = hit.uv();
                        let eval = material.eval(eval_record);
                        let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
                        let cos_theta = wo_local.z.abs();
                        if cos_theta > 0.0 {
                            let le_vec = Vector3f::new(irradiance[0], irradiance[1], irradiance[2]);
                            let contrib = f * (cos_theta / pdf);
                            radiance += contrib.component_mul(&le_vec);
                        }
                    }
                }
            }
        }
    }

    radiance
}

fn env_radiance(scene: &Scene, ray: &Ray3f) -> Vector3f {
    let mut env = RGBSpectrum::default();
    for emitter in scene.emitters() {
        env += emitter.eval_direction(&ray.dir());
    }
    Vector3f::new(env[0], env[1], env[2])
}

fn interior_medium_segment(
    scene: &Scene,
    ray: &Ray3f,
    hit: &crate::core::interaction::SurfaceIntersection,
) -> Option<(Arc<dyn Medium>, Float, Float)> {
    let object_index = hit.object_index()?;
    let object = scene.objects().get(object_index)?;
    let medium = object.interior_medium()?;
    let bbox = object.shape().bounding_box();
    let (_t0, t1) = bbox.ray_intersect_range(ray)?;

    let origin = ray.origin();
    let inside = origin.x >= bbox.p_min.x - 1e-4
        && origin.x <= bbox.p_max.x + 1e-4
        && origin.y >= bbox.p_min.y - 1e-4
        && origin.y <= bbox.p_max.y + 1e-4
        && origin.z >= bbox.p_min.z - 1e-4
        && origin.z <= bbox.p_max.z + 1e-4;

    let entry = if inside { ray.min_t } else { hit.t() };
    let exit = if inside { hit.t() } else { t1.min(ray.max_t) };

    if exit <= entry {
        return None;
    }

    Some((medium, entry, exit))
}

fn shadow_visible_through_nulls(
    scene: &Scene,
    origin: Vector3f,
    dir: Vector3f,
    max_dist: Option<Float>,
    target_p: Option<Vector3f>,
) -> bool {
    let mut current_origin = origin;
    let mut remaining = max_dist.unwrap_or(std::f32::MAX);
    let max_steps = 64;

    for _ in 0..max_steps {
        let max_t = if remaining.is_finite() {
            if remaining <= 0.0 {
                return true;
            }
            Some(remaining)
        } else {
            None
        };
        let ray = Ray3f::new(current_origin, dir, Some(1e-4), max_t);
        let hit = match scene.ray_intersection(&ray) {
            Some(h) => h,
            None => return true,
        };

        let material = match hit.material() {
            Some(m) => m,
            None => return false,
        };

        if material.is_null() {
            let advance = hit.t();
            if remaining.is_finite() {
                if advance >= remaining - 1e-4 {
                    return true;
                }
                remaining = (remaining - advance).max(0.0);
            }
            current_origin = hit.p() + dir * 1e-4;
            continue;
        }

        if let Some(target) = target_p {
            if (hit.p() - target).norm() < 1e-3 {
                return true;
            }
        }

        if remaining.is_finite() && hit.t() >= remaining - 1e-4 {
            return true;
        }

        return false;
    }

    true
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
    let bsdf_weight = f * (cos_theta / pdf);
    let wo_world = local_to_world(&wo_local, tangent, bitangent, &n_sh);

    let offset_dir = if wo_world.dot(&n_geo) >= 0.0 { n_geo } else { -n_geo };
    let origin = p + offset_dir * 1e-6;
    Some((Ray3f::new(origin, wo_world, Some(1e-4), None), bsdf_weight, pdf))
}

fn rgb_from_vec(v: Vector3f) -> RGBSpectrum {
    RGBSpectrum::new(v[0], v[1], v[2])
}
