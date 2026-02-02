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

fn srgb_to_linear(v: Float) -> Float {
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

impl ImageTexture {
    pub fn from_rgb(r: Float, g: Float, b: Float) -> Self {
        Self {
            width: 1,
            height: 1,
            data: vec![(r, g, b)],
        }
    }

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

    pub fn from_image(path: &str, srgb: bool) -> std::result::Result<Self, String> {
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
                let (mut r, mut g, mut b) = (p[0], p[1], p[2]);
                if srgb {
                    r = srgb_to_linear(r);
                    g = srgb_to_linear(g);
                    b = srgb_to_linear(b);
                }
                data.push((r, g, b));
            }
        }

        Ok(Self {
            width: width as usize,
            height: height as usize,
            data,
        })
    }

    pub fn from_file(path: &str) -> std::result::Result<Self, String> {
        Self::from_file_with_srgb(path, true)
    }

    pub fn from_file_with_srgb(path: &str, srgb: bool) -> std::result::Result<Self, String> {
        let ext = Path::new(path)
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();

        match ext.as_str() {
            "exr" => Self::from_exr(path),
            "jpg" | "jpeg" | "png" => Self::from_image(path, srgb),
            _ => Err(format!("unsupported texture format: {}", ext)),
        }
    }

    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    fn sample_bilinear(&self, uv: Vector2f) -> RGBSpectrum {
        if self.width == 0 || self.height == 0 {
            return RGBSpectrum::default();
        }

        let mut u = uv.x.fract();
        let mut v = uv.y.fract();
        if u < 0.0 { u += 1.0; }
        if v < 0.0 { v += 1.0; }

        let x = u * (self.width as Float - 1.0);
        let y = (1.0 - v) * (self.height as Float - 1.0);

        let x0 = x.floor() as usize;
        let y0 = y.floor() as usize;
        let x1 = (x0 + 1) % self.width;
        let y1 = (y0 + 1) % self.height;

        let tx = x - x0 as Float;
        let ty = y - y0 as Float;

        let idx00 = y0 * self.width + x0;
        let idx10 = y0 * self.width + x1;
        let idx01 = y1 * self.width + x0;
        let idx11 = y1 * self.width + x1;

        let (r00, g00, b00) = self.data[idx00];
        let (r10, g10, b10) = self.data[idx10];
        let (r01, g01, b01) = self.data[idx01];
        let (r11, g11, b11) = self.data[idx11];

        let r0 = r00 * (1.0 - tx) + r10 * tx;
        let g0 = g00 * (1.0 - tx) + g10 * tx;
        let b0 = b00 * (1.0 - tx) + b10 * tx;

        let r1 = r01 * (1.0 - tx) + r11 * tx;
        let g1 = g01 * (1.0 - tx) + g11 * tx;
        let b1 = b01 * (1.0 - tx) + b11 * tx;

        let r = r0 * (1.0 - ty) + r1 * ty;
        let g = g0 * (1.0 - ty) + g1 * ty;
        let b = b0 * (1.0 - ty) + b1 * ty;

        RGBSpectrum::new(r, g, b)
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
        self.sample_bilinear(uv)
    }
}
