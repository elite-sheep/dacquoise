// Copyright @yucwang 2026

use crate::math::bitmap::Bitmap;
use crate::math::constants::Vector2f;
use crate::math::ray::Ray3f;

pub trait Sensor: Sync {
    fn sample_ray(&self, u: &Vector2f) -> Ray3f;
    fn bitmap(&self) -> &Bitmap;
    fn bitmap_mut(&mut self) -> &mut Bitmap;
    fn describe(&self) -> String {
        String::from("Sensor")
    }
}
