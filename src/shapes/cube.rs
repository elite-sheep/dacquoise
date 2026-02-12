// Copyright @yucwang 2026

use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::shape::Shape;
use crate::math::aabb::AABB;
use crate::math::constants::{EPSILON, Float, Vector2f, Vector3f};
use crate::math::ray::Ray3f;
use crate::math::spectrum::RGBSpectrum;
use crate::math::transform::Transform;

pub struct Cube {
    id: String,
    to_world: Transform,
    face_areas: [Float; 6],
    area: Float,
    inv_area: Float,
}

impl Cube {
    pub fn new(to_world: Transform, id: Option<String>) -> Self {
        let dx = to_world.apply_vector(Vector3f::new(2.0, 0.0, 0.0));
        let dy = to_world.apply_vector(Vector3f::new(0.0, 2.0, 0.0));
        let dz = to_world.apply_vector(Vector3f::new(0.0, 0.0, 2.0));
        let area_xy = dx.cross(&dy).norm();
        let area_xz = dx.cross(&dz).norm();
        let area_yz = dy.cross(&dz).norm();
        let face_areas = [area_xy, area_xy, area_xz, area_xz, area_yz, area_yz];
        let area = 2.0 * (area_xy + area_xz + area_yz);
        let inv_area = if area > 0.0 { 1.0 / area } else { 0.0 };
        Self { id: id.unwrap_or_else(|| generate_node_id("Cube")), to_world, face_areas, area, inv_area }
    }

    fn intersect_local(&self, ray: &Ray3f) -> Option<(Vector3f, Vector3f)> {
        let ray_local = self.to_world.inv_apply_ray(ray);
        let o = ray_local.origin();
        let d = ray_local.dir();

        let mut t_min = ray_local.min_t;
        let mut t_max = ray_local.max_t;

        for axis in 0..3 {
            let dir = d[axis];
            if dir.abs() < EPSILON {
                if o[axis] < -1.0 || o[axis] > 1.0 {
                    return None;
                }
                continue;
            }

            let inv = 1.0 / dir;
            let mut t0 = (-1.0 - o[axis]) * inv;
            let mut t1 = (1.0 - o[axis]) * inv;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
            }

            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if t_max < t_min {
                return None;
            }
        }

        let t_hit = if t_min >= ray_local.min_t { t_min } else { t_max };
        if t_hit < ray_local.min_t || t_hit > ray_local.max_t {
            return None;
        }
        let p_local = ray_local.at(t_hit);
        let n_local = cube_normal(p_local);
        Some((p_local, n_local))
    }

    fn face_uv(p_local: Vector3f, n_local: Vector3f) -> Vector2f {
        let (u, v) = if n_local.z.abs() > 0.5 {
            (0.5 * (p_local.x + 1.0), 0.5 * (p_local.y + 1.0))
        } else if n_local.y.abs() > 0.5 {
            (0.5 * (p_local.x + 1.0), 0.5 * (p_local.z + 1.0))
        } else {
            (0.5 * (p_local.y + 1.0), 0.5 * (p_local.z + 1.0))
        };
        Vector2f::new(u, v)
    }

    fn sample_face(&self, face: usize, u: Vector2f) -> (Vector3f, Vector3f, Vector2f) {
        let a = 2.0 * u.x - 1.0;
        let b = 2.0 * u.y - 1.0;
        let (p_local, n_local, uv) = match face {
            0 => (Vector3f::new(a, b, 1.0), Vector3f::new(0.0, 0.0, 1.0), Vector2f::new(u.x, u.y)),
            1 => (Vector3f::new(a, b, -1.0), Vector3f::new(0.0, 0.0, -1.0), Vector2f::new(u.x, u.y)),
            2 => (Vector3f::new(a, 1.0, b), Vector3f::new(0.0, 1.0, 0.0), Vector2f::new(u.x, u.y)),
            3 => (Vector3f::new(a, -1.0, b), Vector3f::new(0.0, -1.0, 0.0), Vector2f::new(u.x, u.y)),
            4 => (Vector3f::new(1.0, a, b), Vector3f::new(1.0, 0.0, 0.0), Vector2f::new(u.x, u.y)),
            _ => (Vector3f::new(-1.0, a, b), Vector3f::new(-1.0, 0.0, 0.0), Vector2f::new(u.x, u.y)),
        };
        (p_local, n_local, uv)
    }
}

impl ComputationNode for Cube {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        String::from("Cube")
    }
}

impl Shape for Cube {
    fn bounding_box(&self) -> AABB {
        let mut bbox = AABB::default();
        let corners = [
            Vector3f::new(-1.0, -1.0, -1.0),
            Vector3f::new(-1.0, -1.0, 1.0),
            Vector3f::new(-1.0, 1.0, -1.0),
            Vector3f::new(-1.0, 1.0, 1.0),
            Vector3f::new(1.0, -1.0, -1.0),
            Vector3f::new(1.0, -1.0, 1.0),
            Vector3f::new(1.0, 1.0, -1.0),
            Vector3f::new(1.0, 1.0, 1.0),
        ];
        for corner in &corners {
            let p = self.to_world.apply_point(*corner);
            bbox.expand_by_point(&p);
        }
        bbox
    }

    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let (p_local, n_local) = self.intersect_local(ray)?;
        let p_world = self.to_world.apply_point(p_local);
        let t_world = (p_world - ray.origin()).dot(&ray.dir());
        if !ray.test_segment(t_world) {
            return None;
        }
        let n_world = self.to_world.apply_normal(n_local).normalize();
        let uv = Self::face_uv(p_local, n_local);
        Some(SurfaceIntersection::new(
            p_world,
            n_world,
            n_world,
            uv,
            t_world,
            RGBSpectrum::default(),
            None,
            None,
        ))
    }

    fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        let (p_local, _) = match self.intersect_local(ray) {
            Some(hit) => hit,
            None => return false,
        };
        let p_world = self.to_world.apply_point(p_local);
        let t_world = (p_world - ray.origin()).dot(&ray.dir());
        ray.test_segment(t_world)
    }

    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord {
        if self.area <= 0.0 {
            let intersection = SurfaceIntersection::new(
                Vector3f::zeros(),
                Vector3f::new(0.0, 0.0, 1.0),
                Vector3f::new(0.0, 0.0, 1.0),
                Vector2f::new(0.0, 0.0),
                0.0,
                RGBSpectrum::default(),
                None,
                None,
            );
            return SurfaceSampleRecord::new(intersection, 0.0);
        }

        let target = u.x * self.area;
        let mut accum = 0.0;
        let mut face = 0usize;
        for i in 0..6 {
            accum += self.face_areas[i];
            if target <= accum {
                face = i;
                break;
            }
        }
        let prev = accum - self.face_areas[face];
        let local_u = if self.face_areas[face] > 0.0 {
            ((target - prev) / self.face_areas[face]).clamp(0.0, 1.0)
        } else {
            0.5
        };
        let local_v = u.y;

        let (p_local, n_local, uv) = self.sample_face(face, Vector2f::new(local_u, local_v));
        let p_world = self.to_world.apply_point(p_local);
        let n_world = self.to_world.apply_normal(n_local).normalize();
        let intersection = SurfaceIntersection::new(
            p_world,
            n_world,
            n_world,
            uv,
            0.0,
            RGBSpectrum::default(),
            None,
            None,
        );
        SurfaceSampleRecord::new(intersection, self.inv_area)
    }

    fn surface_area(&self) -> Float {
        self.area
    }
}

fn cube_normal(p: Vector3f) -> Vector3f {
    let ax = p.x.abs();
    let ay = p.y.abs();
    let az = p.z.abs();
    if ax >= ay && ax >= az {
        Vector3f::new(p.x.signum(), 0.0, 0.0)
    } else if ay >= az {
        Vector3f::new(0.0, p.y.signum(), 0.0)
    } else {
        Vector3f::new(0.0, 0.0, p.z.signum())
    }
}
