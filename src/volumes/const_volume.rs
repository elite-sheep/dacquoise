// Copyright @yucwang 2026

use crate::core::volume::Volume;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector3f};

pub struct ConstantVolume {
    value: Vector3f,
    channels: usize,
    bbox: Option<AABB>,
}

impl ConstantVolume {
    pub fn new_scalar(value: Float) -> Self {
        Self {
            value: Vector3f::new(value, value, value),
            channels: 1,
            bbox: None,
        }
    }

    pub fn new_rgb(value: Vector3f) -> Self {
        Self {
            value,
            channels: 3,
            bbox: None,
        }
    }

    pub fn with_bbox(mut self, bbox: Option<AABB>) -> Self {
        self.bbox = bbox;
        self
    }
}

impl Volume for ConstantVolume {
    fn bbox(&self) -> Option<AABB> {
        self.bbox
    }

    fn channels(&self) -> usize {
        self.channels
    }

    fn eval(&self, _p_world: Vector3f) -> Vector3f {
        self.value
    }
}
