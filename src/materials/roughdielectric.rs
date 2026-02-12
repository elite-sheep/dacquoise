// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::materials::microfacet::{
    fresnel_dielectric_full,
    MicrofacetDistribution,
    MicrofacetType,
    reflect,
    refract_with_cos,
};

pub struct RoughDielectricBSDF {
    id: String,
    int_ior: Float,
    ext_ior: Float,
    eta: Float,
    inv_eta: Float,
    distribution: MicrofacetDistribution,
    specular_reflectance: RGBSpectrum,
    specular_transmittance: RGBSpectrum,
}

impl ComputationNode for RoughDielectricBSDF {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        String::from("RoughDielectricBSDF")
    }
}

impl RoughDielectricBSDF {
    pub fn new(
        m_type: MicrofacetType,
        alpha_u: Float,
        alpha_v: Float,
        sample_visible: bool,
        int_ior: Float,
        ext_ior: Float,
        specular_reflectance: RGBSpectrum,
        specular_transmittance: RGBSpectrum,
        id: Option<String>,
    ) -> Self {
        let eta = int_ior / ext_ior;
        let inv_eta = ext_ior / int_ior;
        Self {
            id: id.unwrap_or_else(|| generate_node_id("RoughDielectricBSDF")),
            int_ior,
            ext_ior,
            eta,
            inv_eta,
            distribution: MicrofacetDistribution::new(m_type, alpha_u, alpha_v, sample_visible),
            specular_reflectance,
            specular_transmittance,
        }
    }
}

impl BSDF for RoughDielectricBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        let wi = sample_record.wi;
        let wo = sample_record.wo;
        let cos_i = wi.z;
        let cos_o = wo.z;
        let reflect = cos_i * cos_o > 0.0;
        let eta = if cos_i > 0.0 { self.eta } else { self.inv_eta };
        let wi_align = if cos_i > 0.0 { wi } else { -wi };
        let mut m = (wi + wo * if reflect { 1.0 } else { eta }).normalize();
        if m.z < 0.0 {
            m = -m;
        }

        if wi.dot(&m) * cos_i <= 0.0 || wo.dot(&m) * cos_o <= 0.0 {
            return eval_result;
        }

        let d = self.distribution.eval(&m);
        let g = self.distribution.g(&wi, &wo, &m);
        let (f, _cos_t, _eta_it, _eta_ti) = fresnel_dielectric_full(wi.dot(&m), self.eta);

        if reflect {
            let denom = 4.0 * cos_i.abs() * cos_o.abs();
            if denom <= 1e-6 {
                return eval_result;
            }
            let value = self.specular_reflectance * (f * d * g / denom);
            let dwh_dwo = 1.0 / (4.0 * wo.dot(&m)).abs();
            let pdf = self.distribution.pdf(&wi_align, &m) * f * dwh_dwo;
            eval_result.value = value;
            eval_result.pdf = pdf;
            return eval_result;
        }

        let denom = wi.dot(&m) + eta * wo.dot(&m);
        if denom.abs() <= 1e-6 {
            return eval_result;
        }
        let scale = 1.0 / (eta * eta);
        let numer = (1.0 - f) * d * g * (eta * eta) * wi.dot(&m) * wo.dot(&m);
        let denom_cos = cos_i * cos_o.abs();
        if denom_cos.abs() <= 1e-6 {
            return eval_result;
        }
        let value = self.specular_transmittance
            * (scale * (numer / (denom_cos * denom * denom)).abs());

        let dwh_dwo = (eta * eta * wo.dot(&m)) / (denom * denom);
        let pdf = self.distribution.pdf(&wi_align, &m) * (1.0 - f) * dwh_dwo.abs();

        eval_result.value = value;
        eval_result.pdf = pdf;
        eval_result
    }

    fn sample(&self, u1: Vector2f, u2: Vector2f, wi_in: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = wi_in;
        let wi = wi_in;
        let cos_i = wi.z;
        if cos_i == 0.0 {
            return sampling_record;
        }

        let eta = if cos_i > 0.0 { self.eta } else { self.inv_eta };
        let wi_align = if cos_i > 0.0 { wi } else { -wi };

        let (m, pdf_m) = self.distribution.sample(&wi_align, &u2);
        if pdf_m <= 0.0 {
            return sampling_record;
        }

        let cos_i_m = wi.dot(&m);
        let (f, cos_t, _eta_it, eta_ti) = fresnel_dielectric_full(cos_i_m, self.eta);
        let wo;
        let pdf;

        if u1.x < f {
            wo = reflect(&wi, &m);
            if wo.z * wi.z <= 0.0 {
                return sampling_record;
            }
            let dwh_dwo = 1.0 / (4.0 * wo.dot(&m)).abs();
            pdf = pdf_m * f * dwh_dwo;
        } else {
            if cos_t == 0.0 {
                wo = reflect(&wi, &m);
                if wo.z * wi.z <= 0.0 {
                    return sampling_record;
                }
                let dwh_dwo = 1.0 / (4.0 * wo.dot(&m)).abs();
                pdf = pdf_m * dwh_dwo;
            } else {
                wo = refract_with_cos(&wi, &m, cos_t, eta_ti);
                if wo.z * wi.z >= 0.0 {
                    return sampling_record;
                }
                let denom = cos_i_m + eta * wo.dot(&m);
                if denom.abs() <= 1e-6 {
                    return sampling_record;
                }
                let dwh_dwo = (eta * eta * wo.dot(&m)) / (denom * denom);
                pdf = pdf_m * (1.0 - f) * dwh_dwo.abs();
            }
        }

        if !pdf.is_finite() || pdf <= 0.0 {
            return sampling_record;
        }
        sampling_record.wo = wo;
        sampling_record.pdf = pdf;
        sampling_record
    }

    fn sample_and_eval(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wi);
        self.eval(sampling_record)
    }
}
