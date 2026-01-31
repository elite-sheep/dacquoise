// Copyright @yucwang 2026

use crate::math::constants::Vector2f;
use crate::math::spectrum::RGBSpectrum;

pub trait Texture: Send + Sync {
    fn eval(&self, uv: Vector2f) -> RGBSpectrum;
}
