// Copyright @yucwang 2023


use crate::core::computation_node::ComputationNode;
use crate::core::shape::Shape;
use crate::core::interaction::{ SurfaceIntersection, SurfaceSampleRecord };
use crate::math::aabb::AABB;
use crate::math::constants:: { EPSILON, Float, Vector2f, Vector3f };
use crate::math::ray::Ray3f;
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::square_to_triangle;

use std::option::Option;

pub struct Triangle {
    p0: Vector3f,
    p1: Vector3f,
    p2: Vector3f
}

impl ComputationNode for Triangle {
    fn to_string(&self) -> String {
        String::from("Triangle: {}")
    }
}

impl Shape for Triangle {
    fn bounding_box(&self) -> AABB {
        let mut bound = AABB::new(self.p0, self.p1);
        bound.expand_by_point(&self.p2);

        bound
    }

    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let edge0 = self.p1 - self.p0;
        let edge1 = self.p2 - self.p0;
        let geo_normal = edge0.cross(&edge1).normalize();

        let n_dot_dir = geo_normal.dot(&ray.dir());

        if n_dot_dir > -EPSILON && n_dot_dir < EPSILON {
            return None;
        }

        let plane_d = geo_normal.dot(&self.p0);
        let t = (plane_d - geo_normal.dot(&ray.origin())) / n_dot_dir;

        let intesection_p = ray.origin() + t * ray.dir();

        // TODO: Compute uv, please check PBRT.
        if self.is_in_trangle(&intesection_p) && t >= ray.min_t && t <= ray.max_t {
            let uv = Vector2f::new(0.5, 0.5);
            let intersection = SurfaceIntersection::new(intesection_p, geo_normal, geo_normal, uv, t, RGBSpectrum::default(), None);
            return Some(intersection);
        } else {
            return None;
        }
    }

    fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        let edge0 = self.p1 - self.p0;
        let edge1 = self.p2 - self.p0;
        let geo_normal = edge0.cross(&edge1).normalize();

        let n_dot_dir = geo_normal.dot(&ray.dir());
        
        if n_dot_dir > -EPSILON && n_dot_dir < EPSILON {
            return false;
        }

        let plane_d = geo_normal.dot(&self.p0);
        let t = (plane_d - geo_normal.dot(&ray.origin())) / n_dot_dir;

        let intersection_p = ray.origin() + t * ray.dir();

        return self.is_in_trangle(&intersection_p) & (t >= ray.min_t) & (t <= ray.max_t);
    }

    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord {
        let uv = square_to_triangle(u);
        let p = self.p0 * uv.x + self.p1 * uv.y + self.p2 * uv.z;

        // Compute geoNoraml
        // TODO: Compute shading normal
        let edge0 = self.p1 - self.p0;
        let edge1 = self.p2 - self.p0;
        let n = edge0.cross(&edge1).normalize();

        let interaction = SurfaceIntersection::new(p, n, n, uv.xy(), 0.0, RGBSpectrum::default(), None);

        SurfaceSampleRecord::new(interaction, 1.0 / self.surface_area())
    }

    fn surface_area(&self) -> Float {
        let length = 0.5 * ((self.p1 - self.p0).cross(&(self.p2 - self.p0))).norm();
        length
    }
}

impl Triangle {
    fn new(new_p0: Vector3f, new_p1: Vector3f, new_p2: Vector3f) -> Self {
        Triangle {
            p0: new_p0, 
            p1: new_p1,
            p2: new_p2,
        }
    }

    fn is_in_trangle(&self, p: &Vector3f) -> bool {
        // Compute geoNoraml
        let edge0 = self.p1 - self.p0;
        let edge1 = self.p2 - self.p0;
        let geo_normal = edge0.cross(&edge1);

        let n0 = (self.p1 - self.p0).cross(&(p - self.p0));
        let n1 = (self.p2 - self.p1).cross(&(p - self.p1));
        let n2 = (self.p0 - self.p2).cross(&(p - self.p2));

        return (n0.dot(&geo_normal) >= 0.0) & (n1.dot(&geo_normal) >= 0.0) & (n2.dot(&geo_normal) >= 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounding_box1() {
        let p0 = Vector3f::new(1.0, 1.0, 1.0);
        let p1 = Vector3f::new(1.5, 4.0, -1.0);
        let p2 = Vector3f::new(-1.0, 2.0, 2.5);

        let triangle = Triangle::new(p0, p1, p2);
        let bounding_box = triangle.bounding_box();

        assert_eq!(bounding_box.p_min, Vector3f::new(-1.0, 1.0, -1.0));
        assert_eq!(bounding_box.p_max, Vector3f::new(1.5, 4.0, 2.5));
    }

    #[test]
    fn test_is_in_triangle1() {
        let triangle = Triangle::new(Vector3f::new(1.0, 1.0, 0.0),
                                     Vector3f::new(2.0, 2.0, 0.0),
                                     Vector3f::new(2.0, 1.0, 0.0));

        let p0 = Vector3f::new(1.5, 1.1, 0.0);
        let p1 = Vector3f::new(1.5, 2.0, 0.0);

        let result1 = triangle.is_in_trangle(&p0);
        let result2 = triangle.is_in_trangle(&p1);

        assert_eq!(result1, true);
        assert_eq!(result2, false);
    }

    #[test]
    fn test_ray_intersection_t() {
        let triangle = Triangle::new(Vector3f::new(1.0, 1.0, 0.0),
                                     Vector3f::new(2.0, 2.0, 0.0),
                                     Vector3f::new(2.0, 1.0, 0.0));

        let ray1 = Ray3f::new(Vector3f::new(1.5, 1.1, 3.0), 
            Vector3f::new(0.0, 0.0, -1.0),
            None,
            None);
        let ray2 = Ray3f::new(Vector3f::new(1.5, 1.1, 3.0), 
            Vector3f::new(0.0, 0.0, 1.0),
            None,
            None);

        let result1 = triangle.ray_intersection_t(&ray1);
        let result2 = triangle.ray_intersection_t(&ray2);
        assert_eq!(result1, true);
        assert_eq!(result2, false);
    }
}
