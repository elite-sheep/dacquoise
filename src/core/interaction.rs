// Copyright @yucwang 2023

use crate::core::bsdf::BSDF;
use crate::math::constants::{ Float, Vector2f, Vector3f };
use crate::math::spectrum::RGBSpectrum;
use std::sync::Arc;

pub struct SurfaceIntersection {
    p: Vector3f,
    geo_normal: Vector3f,
    sh_normal:  Vector3f,
    uv: Vector2f,
    t: Float,
    le: RGBSpectrum,
    material: Option<Arc<dyn BSDF>>,
    light_pdf_area: Option<Float>,
    tri_index: Option<usize>,
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
               new_t: Float,
               new_le: RGBSpectrum,
               new_material: Option<Arc<dyn BSDF>>,
               new_light_pdf_area: Option<Float>) -> Self {
        Self { p: new_p, geo_normal: new_geo_normal, sh_normal: new_sh_normal,
               uv: new_uv, t: new_t, le: new_le, material: new_material, light_pdf_area: new_light_pdf_area,
               tri_index: None }
    }

    pub fn t(&self) -> Float {
        self.t
    }

    pub fn le(&self) -> RGBSpectrum {
        self.le
    }

    pub fn p(&self) -> Vector3f {
        self.p
    }

    pub fn uv(&self) -> Vector2f {
        self.uv
    }

    pub fn geo_normal(&self) -> Vector3f {
        self.geo_normal
    }

    pub fn sh_normal(&self) -> Vector3f {
        self.sh_normal
    }

    pub fn triangle_index(&self) -> Option<usize> {
        self.tri_index
    }

    pub fn with_triangle_index(&self, tri_index: Option<usize>) -> Self {
        Self {
            p: self.p.clone(),
            geo_normal: self.geo_normal.clone(),
            sh_normal: self.sh_normal.clone(),
            uv: self.uv.clone(),
            t: self.t,
            le: self.le,
            material: self.material.clone(),
            light_pdf_area: self.light_pdf_area,
            tri_index,
        }
    }

    pub fn with_le(&self, new_le: RGBSpectrum) -> Self {
        Self {
            p: self.p.clone(),
            geo_normal: self.geo_normal.clone(),
            sh_normal: self.sh_normal.clone(),
            uv: self.uv.clone(),
            t: self.t,
            le: new_le,
            material: self.material.clone(),
            light_pdf_area: self.light_pdf_area,
            tri_index: self.tri_index,
        }
    }

    pub fn material(&self) -> Option<&dyn BSDF> {
        self.material.as_deref()
    }

    pub fn with_material(&self, new_material: Arc<dyn BSDF>) -> Self {
        Self {
            p: self.p.clone(),
            geo_normal: self.geo_normal.clone(),
            sh_normal: self.sh_normal.clone(),
            uv: self.uv.clone(),
            t: self.t,
            le: self.le,
            material: Some(new_material),
            light_pdf_area: self.light_pdf_area,
            tri_index: self.tri_index,
        }
    }

    pub fn light_pdf_area(&self) -> Option<Float> {
        self.light_pdf_area
    }

    pub fn with_light_pdf_area(&self, new_light_pdf_area: Option<Float>) -> Self {
        Self {
            p: self.p.clone(),
            geo_normal: self.geo_normal.clone(),
            sh_normal: self.sh_normal.clone(),
            uv: self.uv.clone(),
            t: self.t,
            le: self.le,
            material: self.material.clone(),
            light_pdf_area: new_light_pdf_area,
            tri_index: self.tri_index,
        }
    }
}

impl SurfaceSampleRecord {
    pub fn new(new_intersection: SurfaceIntersection,
               new_pdf: Float) -> Self {
        Self { intersection: new_intersection, pdf: new_pdf }
    }

    pub fn intersection(&self) -> &SurfaceIntersection {
        &self.intersection
    }

    pub fn pdf(&self) -> Float {
        self.pdf
    }

    pub fn set_pdf(&mut self, pdf: Float) {
        self.pdf = pdf;
    }
}
