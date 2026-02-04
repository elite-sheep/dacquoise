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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_volume_scalar() {
        let vol = ConstantVolume::new_scalar(2.0);
        assert_eq!(vol.channels(), 1);
        assert!(vol.bbox().is_none());
        let v = vol.eval(Vector3f::new(0.1, 0.2, 0.3));
        assert_eq!(v, Vector3f::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn constant_volume_rgb_bbox() {
        let bbox = AABB::new(Vector3f::new(-1.0, 0.0, 1.0), Vector3f::new(2.0, 3.0, 4.0));
        let vol = ConstantVolume::new_rgb(Vector3f::new(1.0, 2.0, 3.0)).with_bbox(Some(bbox));
        assert_eq!(vol.channels(), 3);
        let v = vol.eval(Vector3f::new(-0.5, 1.0, 2.0));
        assert_eq!(v, Vector3f::new(1.0, 2.0, 3.0));
        let out = vol.bbox().unwrap();
        assert_eq!(out.p_min, Vector3f::new(-1.0, 0.0, 1.0));
        assert_eq!(out.p_max, Vector3f::new(2.0, 3.0, 4.0));
    }
}
