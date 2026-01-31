// Copyright @yucwang 2026

use crate::core::texture::Texture;
use crate::math::constants::{Float, Vector2f};
use crate::math::spectrum::RGBSpectrum;
use exr::prelude::*;
use image::io::Reader as ImageReader;
use image::GenericImageView;
use std::path::Path;

pub struct ImageTexture {
    width: usize,
    height: usize,
    data: Vec<(Float, Float, Float)>,
}

impl ImageTexture {
    pub fn from_exr(path: &str) -> std::result::Result<Self, String> {
        let image = read()
            .no_deep_data()
            .largest_resolution_level()
            .rgba_channels(
                |resolution, _| ImageTexture {
                    width: resolution.width(),
                    height: resolution.height(),
                    data: vec![(0.0, 0.0, 0.0); resolution.width() * resolution.height()],
                },
                |image, position, (r, g, b, _a): (f32, f32, f32, f32)| {
                    let idx = position.y() * image.width + position.x();
                    image.data[idx] = (r, g, b);
                },
            )
            .first_valid_layer()
            .all_attributes()
            .from_file(path)
            .map_err(|e| format!("failed to read exr {}: {}", path, e))?;

        Ok(image.layer_data.channel_data.pixels)
    }

    pub fn from_jpg(path: &str) -> std::result::Result<Self, String> {
        let img = ImageReader::open(path)
            .map_err(|e| format!("failed to open image {}: {}", path, e))?
            .decode()
            .map_err(|e| format!("failed to decode image {}: {}", path, e))?;

        let (width, height) = img.dimensions();
        let rgb = img.to_rgb32f();
        let mut data = Vec::with_capacity((width * height) as usize);
        for y in 0..height {
            for x in 0..width {
                let p = rgb.get_pixel(x, y);
                data.push((p[0], p[1], p[2]));
            }
        }

        Ok(Self {
            width: width as usize,
            height: height as usize,
            data,
        })
    }

    pub fn from_file(path: &str) -> std::result::Result<Self, String> {
        let ext = Path::new(path)
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();

        match ext.as_str() {
            "exr" => Self::from_exr(path),
            "jpg" | "jpeg" => Self::from_jpg(path),
            _ => Err(format!("unsupported texture format: {}", ext)),
        }
    }

    fn sample_nearest(&self, uv: Vector2f) -> RGBSpectrum {
        if self.width == 0 || self.height == 0 {
            return RGBSpectrum::default();
        }

        let mut u = uv.x.fract();
        let mut v = uv.y.fract();
        if u < 0.0 { u += 1.0; }
        if v < 0.0 { v += 1.0; }

        let x = (u * (self.width as Float - 1.0)).round() as usize;
        let y = ((1.0 - v) * (self.height as Float - 1.0)).round() as usize;
        let idx = y * self.width + x;
        let (r, g, b) = self.data[idx];
        RGBSpectrum::new(r, g, b)
    }
}

impl Texture for ImageTexture {
    fn eval(&self, uv: Vector2f) -> RGBSpectrum {
        self.sample_nearest(uv)
    }
}
