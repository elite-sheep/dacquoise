// Copyright 2020 @TwoCookingMice

use super::constants::{Int, Float, Vector2f, Vector3f, 
                       FLOAT_MIN, FLOAT_MAX};
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
        let mut min = Vector3f::new(0.0, 0.0, 0.0);
        let mut max = Vector3f::new(0.0, 0.0, 0.0);
        for idx in 0..3 {
            min[idx] = p_min[idx].min(p_max[idx]);
            max[idx] = p_max[idx].max(p_min[idx]);
        }
        Self { p_min: min, p_max: max }
    }

    pub fn center(&self) -> Vector3f {
        0.5f32 * self.p_min + 0.5f32 * self.p_max
    }

    pub fn expand_by_point(&mut self, p: &Vector3f) {
        for idx in 0..3 {
            self.p_min[idx] = self.p_min[idx].min(p[idx]);
            self.p_max[idx] = self.p_max[idx].max(p[idx]);
        }
    }

    pub fn expand_by_aabb(&mut self, other: &AABB) {
        for idx in 0..3 {
            self.p_min[idx] = self.p_min[idx].min(other.p_min[idx]);
            self.p_max[idx] = self.p_max[idx].max(other.p_max[idx]);
        }
    }

    pub fn aabb_intersect(&mut self, other: &AABB) {
        for idx in 0..3 {
            self.p_min[idx] = self.p_min[idx].max(other.p_min[idx]);
            self.p_max[idx] = self.p_max[idx].min(other.p_max[idx]);
        }
    }

    pub fn ray_intersect(&self, ray: &Ray3f) -> bool {
        let mut result = true;

        if self.is_valid() {
            let o = ray.origin();
            let d = ray.dir();
            let mut t = Vector2f::new(FLOAT_MIN, FLOAT_MAX);
            for idx in 0..3 {
                let t1 = (self.p_min[idx] - o[idx]) / d[idx];
                let t2 = (self.p_max[idx] - o[idx]) / d[idx];
    
                if t1 < 0.0 && t2 < 0.0 {
                    result = false;
                    break;
                }
    
                t[0] = t[0].max(t1);
                t[1] = t[1].min(t2);
            }
    
            if result && t[0] <= t[1] {
                result &= ray.test_segment(t[0]) | ray.test_segment(t[1]);
            } else {
                result = false;
            }
        } else {
            result = false;
        }

        result
    }

    pub fn surface_area(&self) -> Float {
        let a = self.p_max[0] - self.p_min[0];
        let b = self.p_max[1] - self.p_min[1];
        let c = self.p_max[2] - self.p_min[2];

        2.0f32 * (a*b + a*c + b*c)
    }

    pub fn volume(&self) -> Float {
        let a = self.p_max[0] - self.p_min[0];
        let b = self.p_max[1] - self.p_min[1];
        let c = self.p_max[2] - self.p_min[2];

        a * b * c
    }

    pub fn diagnal(&self) -> Vector3f {
        self.p_max - self.p_min
    }

    pub fn max_extent(&self) -> Int {
        let diagnal = self.diagnal();
        let ans: Int;
        if diagnal[0] > diagnal[1] && diagnal[0] > diagnal[2] {
            ans = 0;
        } else if diagnal[1] > diagnal[2] {
            ans = 1;
        } else {
            ans = 2;
        }

        ans
    }

    pub fn is_valid(&self) -> bool {
        let mut result = true;
        for idx in 0..3 {
            if self.p_min[idx] > self.p_max[idx] {
                result = false;
                break;
            }
        }

        result
    }
}

/* Test for AABB */
#[cfg(test)]
mod tests {
    use super::AABB;
    use super::Ray3f;
    use super::Vector3f;

    #[test]
    fn test_aabb_geometry() {
        let min = Vector3f::new(1.0, 7.0, 3.0);
        let max = Vector3f::new(4.0, 4.0, 4.0);
        let mut bbox: AABB = AABB::new(min, max);

        let center = bbox.center();
        assert_ne!((center[0] - 2.5f32).abs(), 0.000001);
        assert_ne!((center[1] - 5.5f32).abs(), 0.000001);
        assert_ne!((center[2] - 3.5f32).abs(), 0.000001);

        let surface_area = bbox.surface_area();
        assert_ne!((surface_area - 30.0f32), 0.000001);

        let volume = bbox.volume();
        assert_ne!((volume - 9.0f32), 0.000001);

        bbox.expand_by_point(&Vector3f::new(-1.0, 5.0, 6.0));
        assert_ne!((bbox.p_min[0] + 1.0f32), 0.000001);
        assert_ne!((bbox.p_max[0] - 6.0f32), 0.000001);
        assert_eq!(bbox.max_extent(), 0);

        let mut bbox1: AABB = AABB::default();
        bbox1.expand_by_aabb(&bbox);
        assert_ne!((bbox1.p_min[0] + 1.0f32), 0.000001);
        assert_ne!((bbox1.p_max[0] - 6.0f32), 0.000001);
    }

    #[test]
    fn test_aabb_intersect() {
        let o1 = Vector3f::new(0.0, 0.0, 0.0);
        let d1 = Vector3f::new(1.0, 1.0, 1.0);

        let bbox = AABB::new(Vector3f::new(-1.0, -1.0, -1.0),
                             Vector3f::new(1.0, 1.0, 1.0));

        let r1 = Ray3f::new(o1, d1, Some(0.0), Some(1.0));
        let r2 = Ray3f::new(o1, d1, Some(0.0), Some(10.0));
        assert_eq!(bbox.ray_intersect(&r1), false);
        assert_eq!(bbox.ray_intersect(&r2), true);

        let o2 = Vector3f::new(-1.1, 0.0, 0.0);
        let d2 = Vector3f::new(-0.1, 10.0, 10.0);
        let r3 = Ray3f::new(o2, d2, None, None);
        assert_eq!(bbox.ray_intersect(&r3), false);
    }
}
