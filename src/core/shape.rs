// Copyright @yucwang 2023

use crate::core::interaction::{ SurfaceIntersection, SurfaceSampleRecord };
use crate::math::aabb::AABB;
use crate::math::constants::{ Float, Vector2f };
use crate::math::ray::Ray3f;
pub trait Shape: crate::core::computation_node::ComputationNode + Send + Sync {
    fn bounding_box(&self) -> AABB;
    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection>;
    fn ray_intersection_t(&self, ray: &Ray3f) -> bool;
    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord;
    fn surface_area(&self) -> Float;
}
