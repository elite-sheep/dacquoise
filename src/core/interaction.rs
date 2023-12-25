// Copyright @yucwang 2023

use crate::math::constants::Float;

pub struct SurfaceIntersection {
}

pub struct SurfaceSampleRecord {
    intersection: SurfaceIntersection,
    pdf: Float,
}
