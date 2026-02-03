// Copyright @yucwang 2023

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::ComputationNode;
use crate::core::texture::Texture;
use crate::math::constants::{ INV_PI, Vector2f, Vector3f };
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::{ sample_cosine_hemisphere, sample_cosine_hemisphere_pdf };
use std::sync::Arc;

pub struct LambertianDiffuseBSDF {
    texture: Arc<dyn Texture>,
}

impl ComputationNode for LambertianDiffuseBSDF {
    fn to_string(&self) -> String {
        String::from("LambertianDiffuseBSDF: {}")
    }
}

impl BSDF for LambertianDiffuseBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        if sample_record.wi.z <= 0.0 || sample_record.wo.z <= 0.0 {
            eval_result.value = RGBSpectrum::default();
            eval_result.pdf = 0.0;
            return eval_result;
        }

        let color = self.texture.eval(sample_record.uv);
        eval_result.value = color * INV_PI;
        eval_result.pdf = sample_cosine_hemisphere_pdf(sample_record.wo.z.abs());

        return eval_result
    }

    fn sample(&self, 
              u1: Vector2f, 
              _u2: Vector2f,
              wi: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = wi;
        if wi.z <= 0.0 {
            return sampling_record;
        }
        sampling_record.wo = sample_cosine_hemisphere(&u1);
        sampling_record.pdf = sample_cosine_hemisphere_pdf(sampling_record.wo.z.abs());

        return sampling_record
    }

    fn sample_and_eval(&self, 
                       u1: Vector2f, 
                       u2: Vector2f,
                       wi: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wi);
        let eval_result = self.eval(sampling_record);

        return eval_result
    }
}

impl LambertianDiffuseBSDF {
    pub fn new(texture: Arc<dyn Texture>) -> Self {
        Self {
            texture,
        }
    }
}
