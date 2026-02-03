// Copyright @yucwang 2023

use crate::math::constants::{ Float, Vector2f, Vector3f };
use crate::math::spectrum::RGBSpectrum;

// Definitions of types used in BSDF sampling and eval 
// processes
pub type BSDFValue = RGBSpectrum;

#[derive(Debug, PartialEq)]
pub struct BSDFSampleRecord {
    pub wi: Vector3f,
    pub wo: Vector3f,
    pub pdf: Float,
    pub uv: Vector2f,
}

#[derive(Debug, PartialEq)]
pub struct BSDFEvalResult {
    pub value: BSDFValue,
    pub pdf: Float,
}

pub trait BSDF: Send + Sync {
    fn name(&self) -> &'static str {
        std::any::type_name::<Self>()
    }
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult;
    fn sample(&self, u1: Vector2f, 
                     u2: Vector2f,
                     wi: Vector3f) -> BSDFSampleRecord;
    fn sample_and_eval(&self, u1: Vector2f, 
                              u2: Vector2f,
                              wi: Vector3f) -> BSDFEvalResult;
}

impl Default for BSDFSampleRecord {
    fn default() -> Self {
        Self {
            wi: Vector3f::zeros(),
            wo: Vector3f::zeros(),
            pdf: 0.0,
            uv: Vector2f::new(0.0, 0.0),
        }
    }
}

impl BSDFSampleRecord {
    pub fn new(_wi: Vector3f, _wo: Vector3f, _pdf: Float, _uv: Vector2f) -> Self {
        Self {
            wi: _wi,
            wo: _wo,
            pdf: _pdf,
            uv: _uv,
        }
    }
}

impl Default for BSDFEvalResult {
    fn default() -> Self {
        Self {
            value: RGBSpectrum::default(),
            pdf: 0.0,
        }
    }
}
