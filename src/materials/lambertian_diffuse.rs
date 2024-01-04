// Copyright @yucwang 2023

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::Computation_Node;
use crate::math::constants::{ Float, Vector2f, Vector3f };
use crate::math::spectrum::RGBSpectrum;

pub struct LambertianDiffuseBSDF {
    color: RGBSpectrum
}

impl Computation_Node for LambertianDiffuseBSDF {
    fn to_string(&self) -> String {
        String::from("LambertianDiffuseBSDF: {}")
    }
}

impl BSDF for LambertianDiffuseBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
    }

    fn sample(&self, 
              u1: Vector2f, 
              u2: Vector2f,
              wi: Vector3f) -> BSDFSampleRecord {
    }

    fn sample_and_eval(&self, 
                       u1: Vector2f, 
                       u2: Vector2f,
                       wi: Vector3f) -> BSDFEvalResult {
    }
}

impl LambertianDiffuseBSDF {
    pub fn new(rgb: RGBSpectrum) -> Self {
        Self {
            color: rgb,
        }
    }
}
