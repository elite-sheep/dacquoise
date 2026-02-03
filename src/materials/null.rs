// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::ComputationNode;
use crate::math::constants::{Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;

pub struct NullBSDF;

impl NullBSDF {
    pub fn new() -> Self {
        Self
    }
}

impl ComputationNode for NullBSDF {
    fn to_string(&self) -> String {
        String::from("NullBSDF")
    }
}

impl BSDF for NullBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        let wi = sample_record.wi;
        let wo = sample_record.wo;
        let delta = (wo + wi).norm();
        if delta > 1e-6 {
            return eval_result;
        }

        let cos_theta = wo.z.abs();
        if cos_theta <= 1e-6 {
            return eval_result;
        }
        let value = 1.0 / cos_theta;
        eval_result.value = RGBSpectrum::new(value, value, value);
        eval_result.pdf = 1.0;
        eval_result
    }

    fn sample(&self, _u1: Vector2f, _u2: Vector2f, wi: Vector3f) -> BSDFSampleRecord {
        let mut record = BSDFSampleRecord::default();
        record.wi = wi;
        record.wo = -wi;
        record.pdf = 1.0;
        record
    }

    fn sample_and_eval(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFEvalResult {
        let sample = self.sample(u1, u2, wi);
        self.eval(sample)
    }
}
