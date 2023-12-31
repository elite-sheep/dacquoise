// Copyright @yucwang 2023

use crate::math::constants::{ Float, Vector2f, Vector3f };

// Definitions of types used in BSDF sampling and eval 
// processes
pub type BSDFValue = Vector3f;

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
    fn sample(&self, u1: Float, u2: Vector2f) -> BSDFSampleRecord;
    fn sample_and_eval(&self) -> BSDFEvalResult;
}
