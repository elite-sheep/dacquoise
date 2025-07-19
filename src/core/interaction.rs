// Copyright @yucwang 2023

use crate::math::constants::{ Float, Vector2f, Vector3f };

pub struct SurfaceIntersection {
    p: Vector3f,
    geo_normal: Vector3f,
    sh_normal:  Vector3f,
    uv: Vector2f,
    t: Float
}

pub struct SurfaceSampleRecord {
    intersection: SurfaceIntersection,
    pdf: Float,
}

impl SurfaceIntersection {
    pub fn new(new_p: Vector3f, 
               new_geo_normal: Vector3f, 
               new_sh_normal: Vector3f, 
               new_uv: Vector2f,
               new_t: Float) -> Self {
        Self { p: new_p, geo_normal: new_geo_normal, sh_normal: new_sh_normal,
               uv: new_uv, t: new_t}
    }
}

impl SurfaceSampleRecord {
    pub fn new(new_intersection: SurfaceIntersection,
               new_pdf: Float) -> Self {
        Self { intersection: new_intersection, pdf: new_pdf }
    }
}
