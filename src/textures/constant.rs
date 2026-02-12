// Copyright @yucwang 2026

use crate::core::texture::Texture;
use crate::math::constants::Vector2f;
use crate::math::spectrum::RGBSpectrum;

pub struct ConstantTexture {
    value: RGBSpectrum,
}

impl ConstantTexture {
    pub fn new(value: RGBSpectrum) -> Self {
        Self { value }
    }
}

impl Texture for ConstantTexture {
    fn eval(&self, _uv: Vector2f) -> RGBSpectrum {
        self.value
    }

    fn describe(&self) -> String {
        String::from("ConstantTexture")
    }
}

#[cfg(test)]
mod tests {
    use super::ConstantTexture;
    use crate::core::texture::Texture;
    use crate::math::constants::Vector2f;
    use crate::math::spectrum::RGBSpectrum;

    #[test]
    fn test_constant_texture_eval() {
        let value = RGBSpectrum::new(0.25, 0.5, 0.75);
        let tex = ConstantTexture::new(value);
        let result = tex.eval(Vector2f::new(0.1, 0.9));
        assert_eq!(result, value);
    }
}
