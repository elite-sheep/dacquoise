// Copyright @yucwang 2021

use crate::core::scene::Scene;
use crate::math::bitmap::Bitmap;

pub trait Renderer {
    fn render(&self, scene: &mut Scene) -> Bitmap;
}
