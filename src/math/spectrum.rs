// Copyright 2020 @TwoCookingMice

use super::constants::{Float, Vector3f};

pub struct RGBSpectrum {
    rgb: Vector3f
}

impl Default for RGBSpectrum {
    fn default() -> Self {
        Self { rgb: Vector3f::new(0.0f32, 0.0f32, 0.0f32) }
    }
}

impl RGBSpectrum {
    pub fn is_black(&self) -> bool {
        for idx in 0..3 {
            if self.rgb[idx] != 0.0f32  { false; }
        }

        true
    }
}
