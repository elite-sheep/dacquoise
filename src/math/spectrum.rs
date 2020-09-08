// Copyright 2020 @TwoCookingMice

use super::constants::{Float, Vector3f};
use std::ops;

// Trait definition
pub trait Spectrum {
    fn is_black(&self) -> bool;
    fn value(&self) -> Float;
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RGBSpectrum {
    rgb: Vector3f
}

impl Default for RGBSpectrum {
    fn default() -> Self {
        Self { rgb: Vector3f::new(0.0f32, 0.0f32, 0.0f32) }
    }
}

impl ops::Add<Float> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn add(self, value: Float) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] + value,
                         self.rgb[1] + value,
                         self.rgb[2] + value)
    }
}

impl ops::Add<RGBSpectrum> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn add(self, other: RGBSpectrum) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] + other[0],
                         self.rgb[1] + other[1],
                         self.rgb[2] + other[2])
    }
}

impl ops::AddAssign for RGBSpectrum {
    fn add_assign(&mut self, other: Self) {
        *self = RGBSpectrum::new(self.rgb[0] + other[0], 
                                 self.rgb[1] + other[1], 
                                 self.rgb[2] + other[2])
    }
}

impl ops::Div<Float> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn div(self, value: Float) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] / value,
                         self.rgb[1] / value,
                         self.rgb[2] / value)
    }
}

impl ops::Div<RGBSpectrum> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn div(self, other: RGBSpectrum) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] / other[0],
                         self.rgb[1] / other[1],
                         self.rgb[2] / other[2])
    }
}

impl ops::DivAssign for RGBSpectrum {
    fn div_assign(&mut self, other: Self) {
        *self = RGBSpectrum::new(self.rgb[0] / other[0], 
                                 self.rgb[1] / other[1], 
                                 self.rgb[2] / other[2])
    }
}

impl ops::Mul<Float> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn mul(self, value: Float) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] * value,
                         self.rgb[1] * value,
                         self.rgb[2] * value)
    }
}

impl ops::Mul<RGBSpectrum> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn mul(self, other: RGBSpectrum) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] * other[0],
                         self.rgb[1] * other[1],
                         self.rgb[2] * other[2])
    }
}

impl ops::MulAssign for RGBSpectrum {
    fn mul_assign(&mut self, other: Self) {
        *self = RGBSpectrum::new(self.rgb[0] * other[0], 
                                 self.rgb[1] * other[1], 
                                 self.rgb[2] * other[2])
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

impl ops::Sub<Float> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn sub(self, value: Float) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] - value,
                         self.rgb[1] - value,
                         self.rgb[2] - value)
    }
}

impl ops::Sub<RGBSpectrum> for RGBSpectrum {
    type Output = RGBSpectrum;

    fn sub(self, other: RGBSpectrum) -> Self::Output {
        RGBSpectrum::new(self.rgb[0] - other[0],
                         self.rgb[1] - other[1],
                         self.rgb[2] - other[2])
    }
}

impl ops::SubAssign for RGBSpectrum {
    fn sub_assign(&mut self, other: Self) {
        *self = RGBSpectrum::new(self.rgb[0] - other[0], 
                                 self.rgb[1] - other[1], 
                                 self.rgb[2] - other[2])
    }
}

impl Spectrum for RGBSpectrum {
    fn is_black(&self) -> bool {
        let mut result = true;
        for idx in 0..3 {
            if self.rgb[idx] != 0.0f32 {
                result = false;
                break;
            }
        }

        result
    }

    fn value(&self) -> Float {
        0.212671f32 * self.rgb[0] + 0.715160f32 * self.rgb[1] + 0.072169 * self.rgb[2]
    }
}

impl RGBSpectrum {
    pub fn new(r: Float, g: Float, b: Float) -> Self {
        Self { rgb: Vector3f::new(r, g, b) }
    }
}

/* Test for spectrum */
#[cfg(test)]
mod tests {
    use super::RGBSpectrum;
    use super::Spectrum;

    #[test]
    fn test_rgb_spectrum_ops() {
        let rgb1 = RGBSpectrum::new(1.0, 1.0, 1.0);
        assert_ne!((rgb1[0] - 1.0f32).abs(), 0.000001);
        assert_ne!((rgb1[1] - 1.0f32).abs(), 0.000001);
        assert_ne!((rgb1[2] - 1.0f32).abs(), 0.000001);
        assert_eq!(rgb1.is_black(), false);

        let rgb2 = RGBSpectrum::new(0.5, 0.5, 1.4);

        let mut rgb3 = rgb1 + rgb2;
        assert_ne!((rgb3[2] - 2.4f32).abs(), 0.000001);

        rgb3 -= rgb2;
        assert_ne!((rgb3[2] - 1.0f32).abs(), 0.000001);

        rgb3 = rgb3 * 2.0f32;
        assert_ne!((rgb3[0] - 2.0f32).abs(), 0.000001);
    }
}
