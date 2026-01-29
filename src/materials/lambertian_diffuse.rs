// Copyright @yucwang 2023

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::ComputationNode;
use crate::math::constants::{ INV_PI, Vector2f, Vector3f };
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::{ sample_cosine_hemisphere, sample_cosine_hemisphere_pdf };

pub struct LambertianDiffuseBSDF {
    color: RGBSpectrum
}

impl ComputationNode for LambertianDiffuseBSDF {
    fn to_string(&self) -> String {
        String::from("LambertianDiffuseBSDF: {}")
    }
}

impl BSDF for LambertianDiffuseBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        eval_result.value = self.color * INV_PI;
        eval_result.pdf = sample_record.pdf;

        return eval_result
    }

    fn sample(&self, 
              u1: Vector2f, 
              _u2: Vector2f,
              wo: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = sample_cosine_hemisphere(&u1);
        if wo.z < 0.0 {
            sampling_record.wi.z *= -1.0;
        }
        sampling_record.wo = wo;
        sampling_record.pdf = sample_cosine_hemisphere_pdf(wo.dot(&sampling_record.wi).abs());

        return sampling_record
    }

    fn sample_and_eval(&self, 
                       u1: Vector2f, 
                       u2: Vector2f,
                       wo: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wo);
        let eval_result = self.eval(sampling_record);

        return eval_result
    }
}

impl LambertianDiffuseBSDF {
    pub fn new(rgb: RGBSpectrum) -> Self {
        Self {
            color: rgb,
        }
    }
}
