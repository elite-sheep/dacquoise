// Copyright @yucwang 2026

use crate::core::integrator::Integrator;
use crate::core::medium::Medium;
use crate::core::rng::LcgRng;
use crate::core::scene::Scene;
use crate::core::sensor::Sensor;
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
                            let surf = shade_surface_basic(&hit2);
                            color += beta.component_mul(&surf);
                        } else {
                            let env = env_radiance(scene, &exit_ray);
                            color += beta.component_mul(&env);
                        }

                        return rgb_from_vec(color);
                    }
                }

                rgb_from_vec(shade_surface_basic(&hit))
            }
        }
    }

    fn samples_per_pixel(&self) -> u32 {
        self.samples_per_pixel
    }

    fn describe(&self) -> String {
        String::from("RaymarchingIntegrator")
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

fn shade_surface_basic(hit: &crate::core::interaction::SurfaceIntersection) -> Vector3f {
    let le = hit.le();
    if le.is_black() {
        Vector3f::zeros()
    } else {
        Vector3f::new(le[0], le[1], le[2])
    }
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

fn rgb_from_vec(v: Vector3f) -> RGBSpectrum {
    RGBSpectrum::new(v[0], v[1], v[2])
}
