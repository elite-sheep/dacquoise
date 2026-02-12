// Copyright @yucwang 2026

use crate::core::texture::Texture;
use crate::core::scene::RawDataView;
use crate::math::constants::{Float, Matrix3f, MatrixXF, Vector2f};
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
    data: MatrixXF,
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
        let mut data = MatrixXF::zeros(1, 3);
        data[(0, 0)] = r;
        data[(0, 1)] = g;
        data[(0, 2)] = b;
        let (width, height, data) = ensure_min_size(1, 1, data);
        Self::from_data(width, height, data)
    }

    pub fn from_exr(path: &str) -> std::result::Result<Self, String> {
        let image = read()
            .no_deep_data()
            .largest_resolution_level()
            .rgba_channels(
                |resolution, _| {
                    let width = resolution.width() as usize;
                    let height = resolution.height() as usize;
                    let columns = width * 3;
                    ImageTexture {
                        width,
                        height,
                        data: MatrixXF::zeros(height, columns),
                        filter_mode: FilterMode::Bilinear,
                        wrap_mode: WrapMode::Repeat,
                        to_uv: Matrix3f::identity(),
                    }
                },
                |image, position, (r, g, b, _a): (f32, f32, f32, f32)| {
                    let x = position.x() as usize;
                    let y = position.y() as usize;
                    let base = x * 3;
                    image.data[(y, base)] = r;
                    image.data[(y, base + 1)] = g;
                    image.data[(y, base + 2)] = b;
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
        let width = width as usize;
        let height = height as usize;
        let rgb = img.to_rgb32f();
        let columns = width * 3;
        let mut data = MatrixXF::zeros(height, columns);
        for y in 0..height {
            for x in 0..width {
                let p = rgb.get_pixel(x as u32, y as u32);
                let (mut r, mut g, mut b) = (p[0], p[1], p[2]);
                if srgb {
                    r = srgb_to_linear(r);
                    g = srgb_to_linear(g);
                    b = srgb_to_linear(b);
                }
                let base = x * 3;
                data[(y, base)] = r;
                data[(y, base + 1)] = g;
                data[(y, base + 2)] = b;
            }
        }

        let (width, height, data) = ensure_min_size(width, height, data);
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

    fn from_data(width: usize, height: usize, data: MatrixXF) -> Self {
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

    pub fn raw_data_view(&self) -> RawDataView {
        RawDataView::from_matrix(&self.data)
    }

    pub fn raw_matrix(&self) -> &MatrixXF {
        &self.data
    }

    fn pixel_at(&self, x: usize, y: usize) -> (Float, Float, Float) {
        let base = x * 3;
        (
            self.data[(y, base)],
            self.data[(y, base + 1)],
            self.data[(y, base + 2)],
        )
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

        let (r00, g00, b00) = self.pixel_at(x0u, y0u);
        let (r10, g10, b10) = self.pixel_at(x1u, y0u);
        let (r01, g01, b01) = self.pixel_at(x0u, y1u);
        let (r11, g11, b11) = self.pixel_at(x1u, y1u);

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
        let (r, g, b) = self.pixel_at(xi, yi);
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
    fn describe(&self) -> String {
        String::from("ImageTexture")
    }

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
    data: MatrixXF,
) -> (usize, usize, MatrixXF) {
    let new_width = width.max(2);
    let new_height = height.max(2);
    if new_width == width && new_height == height {
        return (width, height, data);
    }

    if width == 0 || height == 0 {
        let out = MatrixXF::zeros(new_height, new_width * 3);
        return (new_width, new_height, out);
    }

    let mut out = MatrixXF::zeros(new_height, new_width * 3);
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

            let base00 = x0 * 3;
            let base10 = x1 * 3;
            let base01 = x0 * 3;
            let base11 = x1 * 3;

            let (r00, g00, b00) = (
                data[(y0, base00)],
                data[(y0, base00 + 1)],
                data[(y0, base00 + 2)],
            );
            let (r10, g10, b10) = (
                data[(y0, base10)],
                data[(y0, base10 + 1)],
                data[(y0, base10 + 2)],
            );
            let (r01, g01, b01) = (
                data[(y1, base01)],
                data[(y1, base01 + 1)],
                data[(y1, base01 + 2)],
            );
            let (r11, g11, b11) = (
                data[(y1, base11)],
                data[(y1, base11 + 1)],
                data[(y1, base11 + 2)],
            );

            let r0 = r00 * (1.0 - tx) + r10 * tx;
            let g0 = g00 * (1.0 - tx) + g10 * tx;
            let b0 = b00 * (1.0 - tx) + b10 * tx;

            let r1 = r01 * (1.0 - tx) + r11 * tx;
            let g1 = g01 * (1.0 - tx) + g11 * tx;
            let b1 = b01 * (1.0 - tx) + b11 * tx;

            let r = r0 * (1.0 - ty) + r1 * ty;
            let g = g0 * (1.0 - ty) + g1 * ty;
            let b = b0 * (1.0 - ty) + b1 * ty;

            let base = x * 3;
            out[(y, base)] = r;
            out[(y, base + 1)] = g;
            out[(y, base + 2)] = b;
        }
    }

    (new_width, new_height, out)
}
