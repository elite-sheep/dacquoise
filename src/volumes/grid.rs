// Copyright @yucwang 2026

use crate::core::volume::Volume;
use crate::core::scene::RawDataView;
use crate::math::aabb::AABB;
use crate::math::constants::{Float, MatrixXF, Vector3f};
use crate::math::transform::Transform;
use crate::volumes::{wrap_coord3, VolumeFilterMode, VolumeWrapMode};
use std::fs;

pub struct GridVolume {
    data: MatrixXF,
    xres: usize,
    yres: usize,
    zres: usize,
    channels: usize,
    bbox: AABB,
    use_grid_bbox: bool,
    to_world: Transform,
    filter_mode: VolumeFilterMode,
    wrap_mode: VolumeWrapMode,
}

impl GridVolume {
    pub fn from_file(path: &str) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|e| format!("failed to read {}: {}", path, e))?;
        let mut cursor = 4usize;

        if bytes.len() < 4 {
            return Err("vol file too small".to_string());
        }
        if &bytes[0..3] != b"VOL" {
            return Err("invalid vol header".to_string());
        }
        let version = bytes[3];
        if version != 3 {
            return Err(format!("unsupported vol version: {}", version));
        }

        let encoding = read_i32(&bytes, &mut cursor)?;
        let xres = read_i32(&bytes, &mut cursor)?;
        let yres = read_i32(&bytes, &mut cursor)?;
        let zres = read_i32(&bytes, &mut cursor)?;
        let channels = read_i32(&bytes, &mut cursor)?;

        if encoding != 1 {
            return Err(format!("unsupported vol encoding: {}", encoding));
        }
        if xres <= 0 || yres <= 0 || zres <= 0 || channels <= 0 {
            return Err("vol dimensions must be positive".to_string());
        }
        let channels = channels as usize;
        if channels != 1 && channels != 3 && channels != 6 {
            return Err(format!("unsupported vol channels: {}", channels));
        }

        let min_x = read_f32(&bytes, &mut cursor)?;
        let min_y = read_f32(&bytes, &mut cursor)?;
        let min_z = read_f32(&bytes, &mut cursor)?;
        let max_x = read_f32(&bytes, &mut cursor)?;
        let max_y = read_f32(&bytes, &mut cursor)?;
        let max_z = read_f32(&bytes, &mut cursor)?;
        let bbox = AABB::new(
            Vector3f::new(min_x, min_y, min_z),
            Vector3f::new(max_x, max_y, max_z),
        );

        let xres = xres as usize;
        let yres = yres as usize;
        let zres = zres as usize;
        let rows = yres
            .checked_mul(zres)
            .ok_or_else(|| "vol dimensions overflow".to_string())?;
        let cols = xres
            .checked_mul(channels)
            .ok_or_else(|| "vol dimensions overflow".to_string())?;
        let mut data = MatrixXF::zeros(rows, cols);

        for z in 0..zres {
            for y in 0..yres {
                let row = z * yres + y;
                for x in 0..xres {
                    let base = x * channels;
                    for c in 0..channels {
                        let v = read_f32(&bytes, &mut cursor)?;
                        data[(row, base + c)] = v;
                    }
                }
            }
        }

        Ok(Self {
            data,
            xres,
            yres,
            zres,
            channels,
            bbox,
            use_grid_bbox: true,
            to_world: Transform::default(),
            filter_mode: VolumeFilterMode::Trilinear,
            wrap_mode: VolumeWrapMode::Clamp,
        })
    }

    pub fn set_transform(&mut self, transform: Transform) {
        self.to_world = transform;
    }

    pub fn set_filter_mode(&mut self, filter_mode: VolumeFilterMode) {
        self.filter_mode = filter_mode;
    }

    pub fn set_wrap_mode(&mut self, wrap_mode: VolumeWrapMode) {
        self.wrap_mode = wrap_mode;
    }

    pub fn set_use_grid_bbox(&mut self, use_grid_bbox: bool) {
        self.use_grid_bbox = use_grid_bbox;
    }

    pub fn raw_data_view(&self) -> RawDataView {
        RawDataView::from_matrix(&self.data)
    }

    fn fetch(&self, x: usize, y: usize, z: usize, channel: usize) -> Float {
        let row = z * self.yres + y;
        let col = x * self.channels + channel;
        self.data[(row, col)]
    }

    fn sample_nearest(&self, p: Vector3f) -> Vector3f {
        if self.xres == 0 || self.yres == 0 || self.zres == 0 {
            return Vector3f::zeros();
        }

        let x = p.x * (self.xres as Float - 1.0);
        let y = p.y * (self.yres as Float - 1.0);
        let z = p.z * (self.zres as Float - 1.0);

        let xi = (x + 0.5).floor() as isize;
        let yi = (y + 0.5).floor() as isize;
        let zi = (z + 0.5).floor() as isize;

        let x0 = xi.clamp(0, self.xres as isize - 1) as usize;
        let y0 = yi.clamp(0, self.yres as isize - 1) as usize;
        let z0 = zi.clamp(0, self.zres as isize - 1) as usize;

        self.sample_channels(x0, y0, z0)
    }

    fn sample_trilinear(&self, p: Vector3f) -> Vector3f {
        if self.xres == 0 || self.yres == 0 || self.zres == 0 {
            return Vector3f::zeros();
        }

        let x = p.x * (self.xres as Float - 1.0);
        let y = p.y * (self.yres as Float - 1.0);
        let z = p.z * (self.zres as Float - 1.0);

        let x0 = x.floor() as isize;
        let y0 = y.floor() as isize;
        let z0 = z.floor() as isize;
        let x1 = x0 + 1;
        let y1 = y0 + 1;
        let z1 = z0 + 1;

        let tx = x - x0 as Float;
        let ty = y - y0 as Float;
        let tz = z - z0 as Float;

        let x0u = x0.clamp(0, self.xres as isize - 1) as usize;
        let y0u = y0.clamp(0, self.yres as isize - 1) as usize;
        let z0u = z0.clamp(0, self.zres as isize - 1) as usize;
        let x1u = x1.clamp(0, self.xres as isize - 1) as usize;
        let y1u = y1.clamp(0, self.yres as isize - 1) as usize;
        let z1u = z1.clamp(0, self.zres as isize - 1) as usize;

        let c000 = self.sample_channels(x0u, y0u, z0u);
        let c100 = self.sample_channels(x1u, y0u, z0u);
        let c010 = self.sample_channels(x0u, y1u, z0u);
        let c110 = self.sample_channels(x1u, y1u, z0u);
        let c001 = self.sample_channels(x0u, y0u, z1u);
        let c101 = self.sample_channels(x1u, y0u, z1u);
        let c011 = self.sample_channels(x0u, y1u, z1u);
        let c111 = self.sample_channels(x1u, y1u, z1u);

        let c00 = c000 * (1.0 - tx) + c100 * tx;
        let c10 = c010 * (1.0 - tx) + c110 * tx;
        let c01 = c001 * (1.0 - tx) + c101 * tx;
        let c11 = c011 * (1.0 - tx) + c111 * tx;

        let c0 = c00 * (1.0 - ty) + c10 * ty;
        let c1 = c01 * (1.0 - ty) + c11 * ty;

        c0 * (1.0 - tz) + c1 * tz
    }

    fn sample_channels(&self, x: usize, y: usize, z: usize) -> Vector3f {
        if self.channels == 1 {
            let v = self.fetch(x, y, z, 0);
            Vector3f::new(v, v, v)
        } else {
            let r = self.fetch(x, y, z, 0);
            let g = self.fetch(x, y, z, 1);
            let b = self.fetch(x, y, z, 2);
            Vector3f::new(r, g, b)
        }
    }

    fn local_bbox(&self) -> AABB {
        if self.use_grid_bbox {
            self.bbox
        } else {
            AABB::new(Vector3f::new(0.0, 0.0, 0.0), Vector3f::new(1.0, 1.0, 1.0))
        }
    }
}

impl Volume for GridVolume {
    fn bbox(&self) -> Option<AABB> {
        let local = self.local_bbox();
        let mut out = AABB::default();
        let min = local.p_min;
        let max = local.p_max;
        let corners = [
            Vector3f::new(min.x, min.y, min.z),
            Vector3f::new(max.x, min.y, min.z),
            Vector3f::new(min.x, max.y, min.z),
            Vector3f::new(max.x, max.y, min.z),
            Vector3f::new(min.x, min.y, max.z),
            Vector3f::new(max.x, min.y, max.z),
            Vector3f::new(min.x, max.y, max.z),
            Vector3f::new(max.x, max.y, max.z),
        ];
        for corner in corners {
            out.expand_by_point(&self.to_world.apply_point(corner));
        }
        Some(out)
    }

    fn channels(&self) -> usize {
        self.channels
    }

    fn eval(&self, p_world: Vector3f) -> Vector3f {
        if self.data.nrows() == 0 || self.data.ncols() == 0 {
            return Vector3f::zeros();
        }

        let mut p = self.to_world.inv_apply_point(p_world);
        if self.use_grid_bbox {
            let diag = self.bbox.diagnal();
            if diag.x.abs() < 1e-8 || diag.y.abs() < 1e-8 || diag.z.abs() < 1e-8 {
                return Vector3f::zeros();
            }
            p = Vector3f::new(
                (p.x - self.bbox.p_min.x) / diag.x,
                (p.y - self.bbox.p_min.y) / diag.y,
                (p.z - self.bbox.p_min.z) / diag.z,
            );
        }

        let p = wrap_coord3(p, self.wrap_mode);

        match self.filter_mode {
            VolumeFilterMode::Nearest => self.sample_nearest(p),
            VolumeFilterMode::Trilinear => self.sample_trilinear(p),
        }
    }
}

fn read_i32(bytes: &[u8], cursor: &mut usize) -> Result<i32, String> {
    if *cursor + 4 > bytes.len() {
        return Err("unexpected eof while reading i32".to_string());
    }
    let mut buf = [0u8; 4];
    buf.copy_from_slice(&bytes[*cursor..*cursor + 4]);
    *cursor += 4;
    Ok(i32::from_le_bytes(buf))
}

fn read_f32(bytes: &[u8], cursor: &mut usize) -> Result<Float, String> {
    if *cursor + 4 > bytes.len() {
        return Err("unexpected eof while reading f32".to_string());
    }
    let mut buf = [0u8; 4];
    buf.copy_from_slice(&bytes[*cursor..*cursor + 4]);
    *cursor += 4;
    Ok(Float::from_le_bytes(buf))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn write_test_vol(path: &PathBuf, data: &[f32], xres: i32, yres: i32, zres: i32, channels: i32) {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"VOL");
        bytes.push(3u8);
        bytes.extend_from_slice(&1i32.to_le_bytes()); // encoding
        bytes.extend_from_slice(&xres.to_le_bytes());
        bytes.extend_from_slice(&yres.to_le_bytes());
        bytes.extend_from_slice(&zres.to_le_bytes());
        bytes.extend_from_slice(&channels.to_le_bytes());
        // bbox 0..1
        for v in [0.0f32, 0.0, 0.0, 1.0, 1.0, 1.0] {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        for v in data {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        std::fs::write(path, bytes).expect("write vol");
    }

    fn approx_eq(a: Vector3f, b: Vector3f) -> bool {
        let eps = 1e-4;
        (a.x - b.x).abs() < eps && (a.y - b.y).abs() < eps && (a.z - b.z).abs() < eps
    }

    #[test]
    fn grid_volume_trilinear_center() {
        let mut path = std::env::temp_dir();
        path.push("grid_volume_trilinear_center.vol");
        let data: Vec<f32> = (0..8).map(|v| v as f32).collect();
        write_test_vol(&path, &data, 2, 2, 2, 1);

        let vol = GridVolume::from_file(path.to_str().unwrap()).expect("load vol");
        assert_eq!(vol.channels(), 1);
        let v = vol.eval(Vector3f::new(0.5, 0.5, 0.5));
        assert!(approx_eq(v, Vector3f::new(3.5, 3.5, 3.5)));
    }

    #[test]
    fn grid_volume_nearest_corner() {
        let mut path = std::env::temp_dir();
        path.push("grid_volume_nearest_corner.vol");
        let data: Vec<f32> = (0..8).map(|v| v as f32).collect();
        write_test_vol(&path, &data, 2, 2, 2, 1);

        let mut vol = GridVolume::from_file(path.to_str().unwrap()).expect("load vol");
        vol.set_filter_mode(VolumeFilterMode::Nearest);
        let v = vol.eval(Vector3f::new(0.1, 0.1, 0.1));
        assert!(approx_eq(v, Vector3f::new(0.0, 0.0, 0.0)));
    }
}
