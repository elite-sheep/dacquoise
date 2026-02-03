// Copyright @yucwang 2026

use crate::core::texture::Texture;
use crate::math::constants::{Float, Matrix3f, Vector2f};
use crate::math::spectrum::RGBSpectrum;
use exr::prelude::*;
use image::io::Reader as ImageReader;
use image::GenericImageView;
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterMode {
    Bilinear,
    Nearest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WrapMode {
    Repeat,
    Mirror,
    Clamp,
}

pub struct ImageTexture {
    width: usize,
    height: usize,
    data: Vec<(Float, Float, Float)>,
    filter_mode: FilterMode,
    wrap_mode: WrapMode,
    to_uv: Matrix3f,
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
        let (width, height, data) = ensure_min_size(1, 1, vec![(r, g, b)]);
        Self::from_data(width, height, data)
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
                    filter_mode: FilterMode::Bilinear,
                    wrap_mode: WrapMode::Repeat,
                    to_uv: Matrix3f::identity(),
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

        let pixels = image.layer_data.channel_data.pixels;
        let (width, height) = pixels.dimensions();
        let (width, height, data) = ensure_min_size(width, height, pixels.data);
        Ok(Self::from_data(width, height, data))
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

        let (width, height, data) = ensure_min_size(width as usize, height as usize, data);
        Ok(Self::from_data(width, height, data))
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

    fn from_data(width: usize, height: usize, data: Vec<(Float, Float, Float)>) -> Self {
        Self {
            width,
            height,
            data,
            filter_mode: FilterMode::Bilinear,
            wrap_mode: WrapMode::Repeat,
            to_uv: Matrix3f::identity(),
        }
    }

    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    pub fn set_filter_mode(&mut self, filter_mode: FilterMode) {
        self.filter_mode = filter_mode;
    }

    pub fn set_wrap_mode(&mut self, wrap_mode: WrapMode) {
        self.wrap_mode = wrap_mode;
    }

    pub fn set_uv_transform(&mut self, transform: Matrix3f) {
        self.to_uv = transform;
    }

    fn sample_bilinear(&self, uv: Vector2f) -> RGBSpectrum {
        if self.width == 0 || self.height == 0 {
            return RGBSpectrum::default();
        }

        let uv = self.wrap_uv(uv);
        let u = uv.x;
        let v = uv.y;

        let x = u * (self.width as Float) - 0.5;
        let y = (1.0 - v) * (self.height as Float) - 0.5;

        let x0 = x.floor() as isize;
        let y0 = y.floor() as isize;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let x0u = self.wrap_index(x0, self.width);
        let y0u = self.wrap_index(y0, self.height);
        let x1u = self.wrap_index(x1, self.width);
        let y1u = self.wrap_index(y1, self.height);

        let tx = x - x0 as Float;
        let ty = y - y0 as Float;

        let idx00 = y0u * self.width + x0u;
        let idx10 = y0u * self.width + x1u;
        let idx01 = y1u * self.width + x0u;
        let idx11 = y1u * self.width + x1u;

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

        let uv = self.wrap_uv(uv);
        let u = uv.x;
        let v = uv.y;

        let x = (u * (self.width as Float) - 0.5).round() as isize;
        let y = ((1.0 - v) * (self.height as Float) - 0.5).round() as isize;
        let xi = self.wrap_index(x, self.width);
        let yi = self.wrap_index(y, self.height);
        let idx = yi * self.width + xi;
        let (r, g, b) = self.data[idx];
        RGBSpectrum::new(r, g, b)
    }

    fn apply_uv_transform(&self, uv: Vector2f) -> Vector2f {
        let u = uv.x;
        let v = uv.y;
        let x = self.to_uv[(0, 0)] * u + self.to_uv[(0, 1)] * v + self.to_uv[(0, 2)];
        let y = self.to_uv[(1, 0)] * u + self.to_uv[(1, 1)] * v + self.to_uv[(1, 2)];
        let w = self.to_uv[(2, 0)] * u + self.to_uv[(2, 1)] * v + self.to_uv[(2, 2)];
        if w.abs() > 1e-8 {
            Vector2f::new(x / w, y / w)
        } else {
            Vector2f::new(x, y)
        }
    }

    fn wrap_uv(&self, uv: Vector2f) -> Vector2f {
        let u = self.wrap_coord(uv.x);
        let v = self.wrap_coord(uv.y);
        Vector2f::new(u, v)
    }

    fn wrap_coord(&self, value: Float) -> Float {
        match self.wrap_mode {
            WrapMode::Repeat => value.rem_euclid(1.0),
            WrapMode::Mirror => {
                let mut v = value.rem_euclid(2.0);
                if v > 1.0 {
                    v = 2.0 - v;
                }
                v
            }
            WrapMode::Clamp => value.clamp(0.0, 1.0),
        }
    }

    fn wrap_index(&self, idx: isize, size: usize) -> usize {
        let n = size as isize;
        if n <= 0 {
            return 0;
        }
        match self.wrap_mode {
            WrapMode::Repeat => idx.rem_euclid(n) as usize,
            WrapMode::Clamp => idx.clamp(0, n - 1) as usize,
            WrapMode::Mirror => {
                if n == 1 {
                    return 0;
                }
                let period = 2 * (n - 1);
                let mut i = idx.rem_euclid(period);
                if i >= n {
                    i = period - i;
                }
                i as usize
            }
        }
    }
}

impl Texture for ImageTexture {
    fn eval(&self, uv: Vector2f) -> RGBSpectrum {
        let uv = self.apply_uv_transform(uv);
        match self.filter_mode {
            FilterMode::Bilinear => self.sample_bilinear(uv),
            FilterMode::Nearest => self.sample_nearest(uv),
        }
    }
}

fn ensure_min_size(
    width: usize,
    height: usize,
    data: Vec<(Float, Float, Float)>,
) -> (usize, usize, Vec<(Float, Float, Float)>) {
    let new_width = width.max(2);
    let new_height = height.max(2);
    if new_width == width && new_height == height {
        return (width, height, data);
    }

    let mut out = vec![(0.0, 0.0, 0.0); new_width * new_height];
    for y in 0..new_height {
        let src_y = if height > 1 {
            (y as Float / (new_height as Float - 1.0)) * (height as Float - 1.0)
        } else {
            0.0
        };
        let y0 = src_y.floor() as usize;
        let y1 = (y0 + 1).min(height.saturating_sub(1));
        let ty = src_y - y0 as Float;

        for x in 0..new_width {
            let src_x = if width > 1 {
                (x as Float / (new_width as Float - 1.0)) * (width as Float - 1.0)
            } else {
                0.0
            };
            let x0 = src_x.floor() as usize;
            let x1 = (x0 + 1).min(width.saturating_sub(1));
            let tx = src_x - x0 as Float;

            let idx00 = y0 * width + x0;
            let idx10 = y0 * width + x1;
            let idx01 = y1 * width + x0;
            let idx11 = y1 * width + x1;

            let (r00, g00, b00) = data[idx00];
            let (r10, g10, b10) = data[idx10];
            let (r01, g01, b01) = data[idx01];
            let (r11, g11, b11) = data[idx11];

            let r0 = r00 * (1.0 - tx) + r10 * tx;
            let g0 = g00 * (1.0 - tx) + g10 * tx;
            let b0 = b00 * (1.0 - tx) + b10 * tx;

            let r1 = r01 * (1.0 - tx) + r11 * tx;
            let g1 = g01 * (1.0 - tx) + g11 * tx;
            let b1 = b01 * (1.0 - tx) + b11 * tx;

            let r = r0 * (1.0 - ty) + r1 * ty;
            let g = g0 * (1.0 - ty) + g1 * ty;
            let b = b0 * (1.0 - ty) + b1 * ty;

            out[y * new_width + x] = (r, g, b);
        }
    }

    (new_width, new_height, out)
}
