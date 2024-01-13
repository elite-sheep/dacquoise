// Copyright @yucwang 2023

use crate::core::computation_node::ComputationNode;
use crate::core::shape::Shape;
use crate::math::constants:: { Float, Vector2f, Vector3f };

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
    }

    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
    }

    fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
    }

    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord {
    }

    fn surface_area(&self) -> Float {
        0.5 * ((p1 - p0).cross(p2 - p0)).length()
    }
}
