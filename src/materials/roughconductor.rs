// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::materials::microfacet::{
    fresnel_conductor,
    MicrofacetDistribution,
    MicrofacetType,
    reflect,
};

pub struct RoughConductorBSDF {
    id: String,
    distribution: MicrofacetDistribution,
    eta: RGBSpectrum,
    k: RGBSpectrum,
    specular_reflectance: RGBSpectrum,
}

impl ComputationNode for RoughConductorBSDF {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        String::from("RoughConductorBSDF")
    }
}

impl RoughConductorBSDF {
    pub fn new(
        m_type: MicrofacetType,
        alpha_u: Float,
        alpha_v: Float,
        sample_visible: bool,
        eta: RGBSpectrum,
        k: RGBSpectrum,
        specular_reflectance: RGBSpectrum,
        id: Option<String>,
    ) -> Self {
        Self {
            id: id.unwrap_or_else(|| generate_node_id("RoughConductorBSDF")),
            distribution: MicrofacetDistribution::new(m_type, alpha_u, alpha_v, sample_visible),
            eta,
            k,
            specular_reflectance,
        }
    }
}

impl BSDF for RoughConductorBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        let wi = sample_record.wi;
        let wo = sample_record.wo;
        if wi.z <= 0.0 || wo.z <= 0.0 {
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

        if wi.dot(&m) <= 0.0 || wo.dot(&m) <= 0.0 {
            return eval_result;
        }

        let d = self.distribution.eval(&m);
        if d <= 0.0 {
            return eval_result;
        }
        let g = self.distribution.g(&wi, &wo, &m);
        let denom = 4.0 * wi.z;
        if denom <= 1e-6 {
            return eval_result;
        }
        let f = fresnel_conductor(wi.dot(&m), self.eta, self.k);
        let mut value = f * (d * g / denom);
        value *= self.specular_reflectance;

        let pdf_m = self.distribution.pdf(&wi, &m);
        let pdf_denom = 4.0 * wo.dot(&m).abs();
        if pdf_denom <= 1e-6 {
            return eval_result;
        }

        eval_result.value = value;
        eval_result.pdf = pdf_m / pdf_denom;
        eval_result
    }

    fn sample(&self, u1: Vector2f, _u2: Vector2f, wi: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = wi;
        if wi.z <= 0.0 {
            return sampling_record;
        }

        let (m, pdf_m) = self.distribution.sample(&wi, &u1);
        if pdf_m <= 0.0 {
            return sampling_record;
        }

        let wo = reflect(&wi, &m);
        if wo.z <= 0.0 {
            return sampling_record;
        }
        if wi.dot(&m) <= 0.0 || wo.dot(&m) <= 0.0 {
            return sampling_record;
        }

        sampling_record.wo = wo;
        sampling_record.pdf = pdf_m / (4.0 * wo.dot(&m).abs()).max(1e-6);
        sampling_record
    }

    fn sample_and_eval(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wi);
        self.eval(sampling_record)
    }
}
