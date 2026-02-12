// Copyright @yucwang 2026

use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::core::emitter::{Emitter, EmitterFlag};
use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::tangent_frame::build_tangent_frame;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, Vector2f, Vector3f, PI};
use crate::math::spectrum::{RGBSpectrum, Spectrum};
use crate::math::warp::sample_uniform_disk_concentric;
use crate::math::transform::Transform;
use crate::core::texture::Texture;
use crate::textures::image::ImageTexture;

pub struct EnvMap {
    id: String,
    texture: ImageTexture,
    scale: Float,
    to_world: Transform,
    width: usize,
    height: usize,
    cdf_width: usize,
    row_cdf: Vec<Float>,
    col_cdf: Vec<Vec<Float>>,
    total_weight: Float,
    bsphere_center: Vector3f,
    bsphere_radius: Float,
}

impl EnvMap {
    pub fn from_file(path: &str, scale: Float) -> std::result::Result<Self, String> {
        Self::from_file_with_id(path, scale, None)
    }

    pub fn from_file_with_id(path: &str, scale: Float, id: Option<String>) -> std::result::Result<Self, String> {
        let texture = ImageTexture::from_file(path)?;
        let (width, height) = texture.dimensions();
        if width == 0 || height == 0 {
            return Err(format!("envmap has invalid resolution: {}x{}", width, height));
        }

        let mut emitter = Self {
            id: id.unwrap_or_else(|| generate_node_id("EnvMap")),
            texture,
            scale,
            to_world: Transform::default(),
            width,
            height,
            cdf_width: width + 1,
            row_cdf: vec![0.0; height + 1],
            col_cdf: vec![vec![0.0; width + 2]; height],
            total_weight: 0.0,
            bsphere_center: Vector3f::zeros(),
            bsphere_radius: 1.0,
        };
        emitter.build_distribution();
        Ok(emitter)
    }

    pub fn set_transform(&mut self, to_world: Transform) {
        self.to_world = to_world;
    }

    fn build_distribution(&mut self) {
        let mut total = 0.0;
        let offset = 0.5 / (self.width as Float);
        for y in 0..self.height {
            let v = (y as Float + 0.5) / (self.height as Float);
            let theta = v * PI;
            let sin_theta = theta.sin();
            let mut row_sum = 0.0;
            for x in 0..self.cdf_width {
                let u_warp = (x as Float + 0.5) / (self.cdf_width as Float);
                let mut u_eval = u_warp + offset;
                if u_eval >= 1.0 {
                    u_eval -= 1.0;
                }
                let rgb = self.eval_radiance(Vector2f::new(u_eval, v));
                let lum = rgb.value();
                let weight = lum * sin_theta;
                row_sum += weight;
                self.col_cdf[y][x + 1] = row_sum;
            }
            total += row_sum;
            self.row_cdf[y + 1] = total;
        }
        self.total_weight = total.max(1e-6);
    }

    fn sample_uv(&self, u: &Vector2f) -> (Vector2f, Float) {
        if self.total_weight <= 0.0 {
            return (Vector2f::new(0.0, 0.0), 0.0);
        }

        let target_row = u.x * self.total_weight;
        let mut y = 0usize;
        for i in 0..self.height {
            if self.row_cdf[i + 1] >= target_row {
                y = i;
                break;
            }
        }

        let row_weight = (self.row_cdf[y + 1] - self.row_cdf[y]).max(1e-8);
        let target_col = u.y * row_weight;
        let mut x = 0usize;
        for i in 0..self.cdf_width {
            if self.col_cdf[y][i + 1] >= target_col {
                x = i;
                break;
            }
        }

        let weight = (self.col_cdf[y][x + 1] - self.col_cdf[y][x]).max(0.0);
        let mut u_coord = (x as Float + 0.5) / (self.cdf_width as Float);
        u_coord += 0.5 / (self.width as Float);
        if u_coord >= 1.0 {
            u_coord -= 1.0;
        }
        let v_coord = (y as Float + 0.5) / (self.height as Float);

        let theta = v_coord * PI;
        let sin_theta = theta.sin().max(1e-8);
        let pixel_area = 1.0 / (self.cdf_width as Float * self.height as Float);
        let pdf_uv = if weight > 0.0 {
            (weight / self.total_weight) / pixel_area
        } else {
            0.0
        };
        let pdf_dir = if pdf_uv > 0.0 {
            pdf_uv / (2.0 * PI * PI * sin_theta)
        } else {
            0.0
        };

        (Vector2f::new(u_coord, v_coord), pdf_dir)
    }

    fn direction_from_uv(&self, uv: Vector2f) -> Vector3f {
        let theta = uv.y * PI;
        let phi = uv.x * 2.0 * PI;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        let d = Vector3f::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
        Vector3f::new(d.y, d.z, -d.x)
    }

    fn uv_from_direction(&self, d: &Vector3f) -> Vector2f {
        let inv_two_pi = 1.0 / (2.0 * PI);
        let raw_u = d.x.atan2(-d.z) * inv_two_pi;
        let u = raw_u - raw_u.floor();
        let v = d.y.clamp(-1.0, 1.0).acos() * (1.0 / PI);
        Vector2f::new(u, v)
    }

    fn eval_radiance(&self, uv: Vector2f) -> RGBSpectrum {
        let uv_tex = Vector2f::new(uv.x, 1.0 - uv.y);
        self.texture.eval(uv_tex) * self.scale
    }

    fn world_dir_from_local(&self, local: Vector3f) -> Vector3f {
        let dir = self.to_world.apply_vector(local);
        let len = dir.norm();
        if len > 0.0 {
            dir / len
        } else {
            dir
        }
    }

    fn local_dir_from_world(&self, world: &Vector3f) -> Option<Vector3f> {
        let dir = self.to_world.inv_apply_vector(*world);
        let len = dir.norm();
        if len > 0.0 {
            Some(dir / len)
        } else {
            None
        }
    }
}

impl ComputationNode for EnvMap {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        String::from("EnvMap")
    }
}

impl Emitter for EnvMap {
    fn new() -> Self {
        let texture = ImageTexture::from_rgb(0.0, 0.0, 0.0);
        Self {
            id: generate_node_id("EnvMap"),
            texture,
            scale: 1.0,
            to_world: Transform::default(),
            width: 1,
            height: 1,
            cdf_width: 2,
            row_cdf: vec![0.0, 1.0],
            col_cdf: vec![vec![0.0, 1.0, 1.0]],
            total_weight: 1.0,
            bsphere_center: Vector3f::zeros(),
            bsphere_radius: 1.0,
        }
    }

    fn get_flag(&self) -> EmitterFlag {
        EmitterFlag::DIRECTION
    }

    fn set_scene_bounds(&mut self, bounds: &AABB) {
        if bounds.is_valid() {
            let center = bounds.center();
            let radius = (bounds.p_max - center).norm().max(1e-6);
            self.bsphere_center = center;
            self.bsphere_radius = radius;
        } else {
            self.bsphere_center = Vector3f::zeros();
            self.bsphere_radius = 1e-6;
        }
    }

    fn eval_direction(&self, direction: &Vector3f) -> RGBSpectrum {
        let dir = match self.local_dir_from_world(direction) {
            Some(dir) => dir,
            None => return RGBSpectrum::default(),
        };
        let uv = self.uv_from_direction(&dir);
        self.eval_radiance(uv)
    }

    fn sample_position(&self, u: &Vector2f) -> SurfaceSampleRecord {
        let (uv, _pdf) = self.sample_uv(u);
        let le = self.eval_radiance(uv);

        let dir_local = self.direction_from_uv(uv);
        let dir = self.world_dir_from_local(dir_local);
        let (tangent, bitangent) = build_tangent_frame(&dir);
        let disk = sample_uniform_disk_concentric(u);
        let perp_offset = tangent * disk.x + bitangent * disk.y;
        let p = self.bsphere_center + (perp_offset - dir) * self.bsphere_radius;
        let n = dir;
        let intersection = SurfaceIntersection::new(p, n, n, uv, 0.0, le, None, None);
        SurfaceSampleRecord::new(intersection, 0.0)
    }

    fn sample_direction(&self, u: &Vector2f, _position: &SurfaceIntersection) -> Vector3f {
        let (uv, _pdf) = self.sample_uv(u);
        let dir_local = self.direction_from_uv(uv);
        self.world_dir_from_local(dir_local)
    }

    fn pdf_position(&self, _position: &SurfaceIntersection) -> Float {
        0.0
    }

    fn pdf_direction(&self, _position: &SurfaceIntersection, direction: &Vector3f) -> Float {
        if self.total_weight <= 0.0 {
            return 0.0;
        }
        let dir = match self.local_dir_from_world(direction) {
            Some(dir) => dir,
            None => return 0.0,
        };
        let uv = self.uv_from_direction(&dir);
        let mut u_warp = uv.x - 0.5 / (self.width as Float);
        u_warp = u_warp - u_warp.floor();

        let mut x = (u_warp * self.cdf_width as Float).floor() as usize;
        let mut y = (uv.y * self.height as Float).floor() as usize;
        if x >= self.cdf_width {
            x = self.cdf_width - 1;
        }
        if y >= self.height {
            y = self.height - 1;
        }

        let weight = (self.col_cdf[y][x + 1] - self.col_cdf[y][x]).max(0.0);
        let pixel_area = 1.0 / (self.cdf_width as Float * self.height as Float);
        let pdf_uv = if weight > 0.0 {
            (weight / self.total_weight) / pixel_area
        } else {
            0.0
        };

        let theta = uv.y * PI;
        let sin_theta = theta.sin();
        if sin_theta <= 0.0 {
            return 0.0;
        }
        let pdf = pdf_uv / (2.0 * PI * PI * sin_theta);
        pdf.max(0.0)
    }
}
