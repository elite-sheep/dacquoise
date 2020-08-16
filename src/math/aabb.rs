// Copyright 2020 @TwoCookingMice

use super::constants::{Float, Vector3f, FLOAT_MIN, FLOAT_MAX};
use super::ray::{Ray3f};

pub struct AABB {
    pub p_min: Vector3f,
    pub p_max: Vector3f
}

impl Default for AABB {
    fn default() -> Self {
        Self { p_min: Vector3f::new(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX),
               p_max: Vector3f::new(FLOAT_MIN, FLOAT_MIN, FLOAT_MIN) }
    }
}

impl AABB {
    pub fn new(p_min: Vector3f, p_max: Vector3f) -> Self {
        Self { p_min: p_min, p_max: p_max }
    }

    pub fn center(&self) -> Vector3f {
        0.5f32 * self.p_min + 0.5f32 * self.p_max
    }

    pub fn surface_area(&self) -> Float {
        let a = self.p_max[0] - self.p_min[0];
        let b = self.p_max[1] - self.p_min[1];
        let c = self.p_max[2] - self.p_min[2];

        2.0f32 * (a*b + a*c + b*c)
    }
}

/* Test for AABB */

#[cfg(test)]
mod tests {
    use super::Vector3f;
    use super::AABB;

    #[test]
    fn test_aabb_geometry() {
        let min = Vector3f::new(1.0, 4.0, 3.0);
        let max = Vector3f::new(4.0, 7.0, 4.0);
        let bbox: AABB = AABB::new(min, max);

        let center = bbox.center();
        assert_ne!((center[0] - 2.5f32).abs(), 0.000001);
        assert_ne!((center[1] - 5.5f32).abs(), 0.000001);
        assert_ne!((center[2] - 3.5f32).abs(), 0.000001);

        let surface_area = bbox.surface_area();
        assert_ne!((surface_area - 30.0f32), 0.000001);
    }
}
