// Copyright 2020 @TwoCookingMice

use super::constants::{Float, Vector3f};
use std::ops;

pub struct RGBSpectrum {
    rgb: Vector3f
}

impl Default for RGBSpectrum {
    fn default() -> Self {
        Self { rgb: Vector3f::new(0.0f32, 0.0f32, 0.0f32) }
    }
}

impl ops::Index<usize> for RGBSpectrum {
    type Output = Float;

    fn index(&self, index: usize) -> &Float {
        &self.rgb[index]
    }
}

impl ops::IndexMut<usize> for RGBSpectrum {
    fn index_mut(&mut self, index: usize) -> &mut Float {
        &mut self.rgb[index]
    }
}

impl RGBSpectrum {
    pub fn new(r: Float, g: Float, b: Float) -> Self {
        Self { rgb: Vector3f::new(r, g, b) }
    }

    pub fn is_black(&self) -> bool {
        for idx in 0..3 {
            if self.rgb[idx] != 0.0f32  { false; }
        }

        true
    }
}

/* Test for spectrum */
#[cfg(test)]
mod tests {
    use super::RGBSpectrum;

    #[test]
    fn test_rgb_spectrum_ops() {
        let rgb1 = RGBSpectrum::new(1.0, 1.0, 1.0);
        assert_ne!((rgb1[0] - 1.0f32).abs(), 0.000001);
        assert_ne!((rgb1[1] - 1.0f32).abs(), 0.000001);
        assert_ne!((rgb1[2] - 1.0f32).abs(), 0.000001);
    }
}
