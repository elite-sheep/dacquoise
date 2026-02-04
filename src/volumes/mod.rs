// Copyright @yucwang 2026

pub mod const_volume;
pub mod grid_volume;

use crate::math::constants::Float;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VolumeFilterMode {
    Nearest,
    Trilinear,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VolumeWrapMode {
    Repeat,
    Mirror,
    Clamp,
}

fn wrap_coord(value: Float, mode: VolumeWrapMode) -> Float {
    match mode {
        VolumeWrapMode::Clamp => value.clamp(0.0, 1.0),
        VolumeWrapMode::Repeat => {
            let mut v = value - value.floor();
            if v < 0.0 {
                v += 1.0;
            }
            v
        }
        VolumeWrapMode::Mirror => {
            let mut v = value % 2.0;
            if v < 0.0 {
                v += 2.0;
            }
            if v > 1.0 {
                2.0 - v
            } else {
                v
            }
        }
    }
}

fn wrap_index(idx: isize, size: usize, mode: VolumeWrapMode) -> usize {
    let size_i = size as isize;
    match mode {
        VolumeWrapMode::Clamp => idx.clamp(0, size_i - 1) as usize,
        VolumeWrapMode::Repeat => {
            let mut v = idx % size_i;
            if v < 0 {
                v += size_i;
            }
            v as usize
        }
        VolumeWrapMode::Mirror => {
            let mut v = idx % (2 * size_i);
            if v < 0 {
                v += 2 * size_i;
            }
            if v >= size_i {
                (2 * size_i - v - 1) as usize
            } else {
                v as usize
            }
        }
    }
}

pub(crate) fn wrap_coord3(value: crate::math::constants::Vector3f, mode: VolumeWrapMode) -> crate::math::constants::Vector3f {
    crate::math::constants::Vector3f::new(
        wrap_coord(value.x, mode),
        wrap_coord(value.y, mode),
        wrap_coord(value.z, mode),
    )
}

pub(crate) fn wrap_index3(idx: (isize, isize, isize), size: (usize, usize, usize), mode: VolumeWrapMode) -> (usize, usize, usize) {
    (
        wrap_index(idx.0, size.0, mode),
        wrap_index(idx.1, size.1, mode),
        wrap_index(idx.2, size.2, mode),
    )
}
