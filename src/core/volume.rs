// Copyright @yucwang 2026

use crate::math::aabb::AABB;
use crate::math::constants::Vector3f;

pub trait Volume: Send + Sync {
    fn bbox(&self) -> Option<AABB> {
        None
    }
    fn channels(&self) -> usize;
    fn eval(&self, p_world: Vector3f) -> Vector3f;
}
