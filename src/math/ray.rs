// Copyright 2020 @TwoCookingMice

use super::constants::{Float, Vector3f};

pub struct Ray3f {
    origin: Vector3f,
    dir: Vector3f,
    pub min_t: Float,
    pub max_t: Float
}

impl Ray3f {
    pub fn new(o: Vector3f, d: Vector3f, 
               min_t: Option<Float>, max_t: Option<Float>) -> Self {
        Self { origin: o, dir: d.normalize(), 
               min_t: min_t.unwrap_or(0.0),
               max_t: max_t.unwrap_or(std::f32::MAX)}
    }

    pub fn origin(&self) -> Vector3f {
        self.origin
    }

    pub fn dir(&self) -> Vector3f {
        self.dir
    }

    pub fn at(&self, t: Float) -> Vector3f {
        self.origin + self.dir * t
    }

    pub fn update(&mut self, t: Float) -> bool {
        if t < self.min_t || t > self.max_t {
            false
        } else {
            self.max_t = t;
            true
        }
    }

    pub fn test_segment(&self, t: Float) -> bool {
        if t >= self.min_t && t <= self.max_t {
            true
        } else {
            false
        }
    }
}

/* Tests for Ray */

#[cfg(test)]
mod tests {
    use super::Vector3f;
    use super::{Ray3f};

    #[test]
    fn test_ray3f() {
        let o = Vector3f::new(0.0, 0.0, 0.0);
        let d = Vector3f::new(1.0, 0.0, 1.0);
        let mut ray = Ray3f::new(o, d, None, None);
        assert_eq!(o, ray.origin());

        let v1 = ray.at(2.0);
        assert_ne!((v1[0]-std::f32::consts::SQRT_2).abs(), 0.000001);
        assert_ne!((v1[1]-0.0).abs(), 0.000001);
        assert_ne!((v1[2]-std::f32::consts::SQRT_2).abs(), 0.000001);

        let status1 = ray.update(100.0);
        let status2 = ray.update(105.0);
        assert_eq!(status1, true);
        assert_eq!(status2, false);
    }
}
