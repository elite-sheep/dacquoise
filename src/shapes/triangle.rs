// Copyright @yucwang 2023

use crate::core::computation_node::ComputationNode;
use crate::core::shape::Shape;
use crate::core::interaction::{ SurfaceIntersection, SurfaceSampleRecord };
use crate::math::aabb::AABB;
use crate::math::constants:: { EPSILON, Float, Vector2f, Vector3f };
use crate::math::ray::Ray3f;
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
        let edge1 = self.p1 - self.p0;
        let edge2 = self.p2 - self.p0;

        let p_vec = ray.dir().cross(&edge2);
        let det = edge1.dot(&p_vec);

        if det > -EPSILON && det < EPSILON {
            ()
        }

        let inv_det = 1.0 / det;
        let t_vec = ray.origin() - self.p0;

        let u = t_vec.dot(&p_vec) * inv_det;
        if u < 0.0 || u > 1.0 {
            ()
        }

        let q_vec = t_vec.cross(&edge1);
        let v = ray.dir().dot(&q_vec) * inv_det;
        if v < 0.0 || v > 1.0 {
            ()
        }

        let t = edge2.dot(&q_vec) * inv_det;

        let p = self.p0 * u + self.p1 * v + self.p2 * (1.0 - u - v);

        Some(SurfaceIntersection::new(p, p, p, Vector2f::new(u, v), t))
    }

    fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        let edge1 = self.p1 - self.p0;
        let edge2 = self.p2 - self.p0;

        let p_vec = ray.dir().cross(&edge2);
        let det = edge1.dot(&p_vec);

        if det > -EPSILON && det < EPSILON {
            return false;
        }

        let inv_det = 1.0 / det;
        let t_vec = ray.origin() - self.p0;

        let u = t_vec.dot(&p_vec) * inv_det;
        if u < 0.0 || u > 1.0 {
            return false;
        }

        let q_vec = t_vec.cross(&edge1);
        let v = ray.dir().dot(&q_vec) * inv_det;
        if v < 0.0 || v > 1.0 {
            return false;
        }

        let t = edge2.dot(&q_vec) * inv_det;

        return t > ray.min_t && t < ray.max_t;
    }

    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord {
        let uv = square_to_triangle(u);
        let p = self.p0 * uv.x + self.p1 * uv.y + self.p2 * uv.z;
        let interaction = SurfaceIntersection::new(p, p, p, uv.xy(), 0.0);

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
}
