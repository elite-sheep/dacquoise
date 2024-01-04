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
}

#[derive(Debug, PartialEq)]
pub struct BSDFEvalResult {
    pub value: BSDFValue,
    pub pdf: Float,
}

pub trait BSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult;
    fn sample(&self, u1: Vector2f, 
                     u2: Vector2f,
                     wi: Vector3f) -> BSDFSampleRecord;
    fn sample_and_eval(&self, u1: Vector2f, 
                              u2: Vector2f,
                              wi: Vector3f) -> BSDFEvalResult;
}
