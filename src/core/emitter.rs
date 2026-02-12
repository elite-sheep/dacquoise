// Copyright @yucwang 2026

use crate::core::computation_node::ComputationNode;
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::aabb::AABB;
use crate::math::spectrum::RGBSpectrum;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmitterFlag(u8);

impl EmitterFlag {
    pub const NONE: Self = Self(0);
    pub const DIRECTION: Self = Self(1 << 0);
    pub const SURFACE: Self = Self(1 << 1);
    pub const DELTA: Self = Self(1 << 2);

    pub fn contains(self, other: Self) -> bool {
        (self.0 & other.0) != 0
    }
}

impl std::ops::BitOr for EmitterFlag {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for EmitterFlag {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

pub enum EmitterSample {
    Surface(SurfaceSampleRecord),
    Direction {
        direction: Vector3f,
        irradiance: RGBSpectrum,
        pdf: Float,
        is_delta: bool,
    },
}

pub trait Emitter: ComputationNode + Send + Sync {
    fn new() -> Self where Self: Sized;
    fn get_flag(&self) -> EmitterFlag;
    fn set_scene_bounds(&mut self, _bounds: &AABB) {}
    fn eval_direction(&self, _direction: &Vector3f) -> RGBSpectrum {
        RGBSpectrum::default()
    }
    fn sample_position(&self, u: &Vector2f) -> SurfaceSampleRecord;
    fn sample_direction(&self, u: &Vector2f, position: &SurfaceIntersection) -> Vector3f;
    fn pdf_position(&self, position: &SurfaceIntersection) -> Float;
    fn pdf_direction(&self, position: &SurfaceIntersection, direction: &Vector3f) -> Float;
}
