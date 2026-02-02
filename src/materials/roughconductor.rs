// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::ComputationNode;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::materials::microfacet::{fresnel_schlick, ggx_d, ggx_g, pdf_ggx_vndf, sample_ggx_vndf, reflect};

pub struct RoughConductorBSDF {
    alpha: Float,
    specular_reflectance: RGBSpectrum,
}

impl ComputationNode for RoughConductorBSDF {
    fn to_string(&self) -> String {
        String::from("RoughConductorBSDF")
    }
}

impl RoughConductorBSDF {
    pub fn new(alpha: Float, specular_reflectance: RGBSpectrum) -> Self {
        Self { alpha, specular_reflectance }
    }
}

impl BSDF for RoughConductorBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        let wi = sample_record.wi;
        let wo = sample_record.wo;
        if wi.z * wo.z <= 0.0 {
            return eval_result;
        }

        let mut m = wi + wo;
        if m.norm_squared() <= 0.0 {
            return eval_result;
        }
        m = m.normalize();
        if m.z <= 0.0 {
            return eval_result;
        }

        let cos_i = wi.z.abs();
        let cos_o = wo.z.abs();
        if cos_i <= 1e-6 || cos_o <= 1e-6 {
            return eval_result;
        }
        let cos_i_m = wi.dot(&m);
        let cos_o_m = wo.dot(&m);
        if cos_i <= 0.0 || cos_o <= 0.0 || cos_i_m <= 0.0 || cos_o_m <= 0.0 {
            return eval_result;
        }

        let d = ggx_d(m.z, self.alpha);
        let g = ggx_g(cos_i, cos_o, self.alpha);
        let f = fresnel_schlick(self.specular_reflectance, cos_i_m.abs());
        let denom = 4.0 * cos_i * cos_o;
        if denom <= 1e-6 {
            return eval_result;
        }
        let value = f * (d * g / denom);

        let pdf_m = pdf_ggx_vndf(&wi, &m, self.alpha);
        let pdf_denom = 4.0 * cos_o_m.abs();
        if pdf_denom <= 1e-6 {
            return eval_result;
        }
        let pdf = pdf_m / pdf_denom;

        eval_result.value = value;
        eval_result.pdf = pdf;
        eval_result
    }

    fn sample(&self, u1: Vector2f, _u2: Vector2f, wi: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = wi;
        if wi.z <= 0.0 {
            return sampling_record;
        }

        let wi_pos = wi;
        let m = sample_ggx_vndf(&wi_pos, &u1, self.alpha);
        let cos_i_m = wi.dot(&m);
        if cos_i_m <= 0.0 {
            return sampling_record;
        }

        let wo = reflect(&wi, &m);
        if wo.z * wi.z <= 0.0 {
            return sampling_record;
        }

        let pdf_m = pdf_ggx_vndf(&wi_pos, &m, self.alpha);
        let pdf_denom = 4.0 * wo.dot(&m).abs();
        if pdf_denom <= 1e-6 {
            return sampling_record;
        }
        let pdf = pdf_m / pdf_denom;

        sampling_record.wo = wo;
        sampling_record.pdf = pdf;
        sampling_record
    }

    fn sample_and_eval(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wi);
        self.eval(sampling_record)
    }
}
