// Copyright @yucwang 2026

use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::shape::Shape;
use crate::math::aabb::AABB;
use crate::math::constants::{EPSILON, Float, Vector2f, Vector3f};
use crate::math::ray::Ray3f;
use crate::math::spectrum::RGBSpectrum;
use crate::math::transform::Transform;

pub struct Rectangle {
    to_world: Transform,
    normal: Vector3f,
    dp_du: Vector3f,
    dp_dv: Vector3f,
    area: Float,
    inv_area: Float,
}

impl Rectangle {
    pub fn new(to_world: Transform) -> Self {
        let dp_du = to_world.apply_vector(Vector3f::new(2.0, 0.0, 0.0));
        let dp_dv = to_world.apply_vector(Vector3f::new(0.0, 2.0, 0.0));
        let area = dp_du.cross(&dp_dv).norm();
        let inv_area = if area > 0.0 { 1.0 / area } else { 0.0 };
        let mut normal = dp_du.cross(&dp_dv);
        if normal.norm() > 0.0 {
            normal = normal.normalize();
        } else {
            normal = to_world.apply_normal(Vector3f::new(0.0, 0.0, 1.0));
            if normal.norm() > 0.0 {
                normal = normal.normalize();
            }
        }

        Self { to_world, normal, dp_du, dp_dv, area, inv_area }
    }

    fn intersect_local(&self, ray: &Ray3f) -> Option<(Vector3f, Vector2f)> {
        let ray_local = self.to_world.inv_apply_ray(ray);
        let dir = ray_local.dir();
        if dir.z.abs() < EPSILON {
            return None;
        }

        let t_local = -ray_local.origin().z / dir.z;
        let p_local = ray_local.at(t_local);
        if p_local.x.abs() > 1.0 || p_local.y.abs() > 1.0 {
            return None;
        }

        let uv = Vector2f::new(0.5 * (p_local.x + 1.0), 0.5 * (p_local.y + 1.0));
        Some((p_local, uv))
    }
}

impl Shape for Rectangle {
    fn bounding_box(&self) -> AABB {
        let mut bbox = AABB::default();
        let corners = [
            Vector3f::new(-1.0, -1.0, 0.0),
            Vector3f::new(-1.0,  1.0, 0.0),
            Vector3f::new( 1.0, -1.0, 0.0),
            Vector3f::new( 1.0,  1.0, 0.0),
        ];
        for corner in &corners {
            let p = self.to_world.apply_point(*corner);
            bbox.expand_by_point(&p);
        }
        bbox
    }

    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let (p_local, uv) = self.intersect_local(ray)?;
        let p_world = self.to_world.apply_point(p_local);
        let t_world = (p_world - ray.origin()).dot(&ray.dir());
        if !ray.test_segment(t_world) {
            return None;
        }

        let intersection = SurfaceIntersection::new(
            p_world,
            self.normal,
            self.normal,
            uv,
            t_world,
            RGBSpectrum::default(),
            None,
            None,
        );
        Some(intersection)
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
        let p_local = Vector3f::new(2.0 * u.x - 1.0, 2.0 * u.y - 1.0, 0.0);
        let p_world = self.to_world.apply_point(p_local);
        let uv = Vector2f::new(u.x, u.y);
        let intersection = SurfaceIntersection::new(
            p_world,
            self.normal,
            self.normal,
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
