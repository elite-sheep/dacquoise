// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::ComputationNode;
use crate::math::constants::{Float, Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;
use crate::materials::microfacet::{fresnel_dielectric, ggx_d, ggx_g, pdf_ggx_vndf, sample_ggx_vndf, reflect, refract};

pub struct RoughDielectricBSDF {
    alpha: Float,
    int_ior: Float,
    ext_ior: Float,
    specular_reflectance: RGBSpectrum,
    specular_transmittance: RGBSpectrum,
}

impl ComputationNode for RoughDielectricBSDF {
    fn to_string(&self) -> String {
        String::from("RoughDielectricBSDF")
    }
}

impl RoughDielectricBSDF {
    pub fn new(alpha: Float, int_ior: Float, ext_ior: Float, specular_reflectance: RGBSpectrum, specular_transmittance: RGBSpectrum) -> Self {
        Self {
            alpha,
            int_ior,
            ext_ior,
            specular_reflectance,
            specular_transmittance,
        }
    }
}

impl BSDF for RoughDielectricBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        let mut eval_result = BSDFEvalResult::default();
        let mut wi = sample_record.wi;
        let mut wo = sample_record.wo;
        if wi.z == 0.0 {
            return eval_result;
        }

        let mut flip = 1.0;
        if wi.z < 0.0 {
            wi = -wi;
            wo = -wo;
            flip = -1.0;
        }

        let (eta_i, eta_t) = if flip > 0.0 {
            (self.ext_ior, self.int_ior)
        } else {
            (self.int_ior, self.ext_ior)
        };

        let cos_i = wi.z;
        let cos_o = wo.z;
        if cos_i.abs() <= 1e-6 || cos_o.abs() <= 1e-6 {
            return eval_result;
        }
        let alpha = self.alpha.max(1e-4);

        let is_reflection = cos_i * cos_o > 0.0;
        if is_reflection {
            let mut m = wi + wo;
            if m.norm_squared() <= 0.0 {
                return eval_result;
            }
            m = m.normalize();
            if m.z <= 0.0 {
                return eval_result;
            }
            let cos_i_m = wi.dot(&m);
            let cos_o_m = wo.dot(&m);
            if cos_i_m <= 0.0 || cos_o_m <= 0.0 {
                return eval_result;
            }

            let d = ggx_d(m.z, alpha);
            let g = ggx_g(cos_i.abs(), cos_o.abs(), alpha);
            let f = fresnel_dielectric(cos_i_m, eta_i, eta_t);
            let denom = 4.0 * cos_i.abs() * cos_o.abs();
            if denom <= 1e-6 {
                return eval_result;
            }
            let value = self.specular_reflectance * (f * d * g / denom);

            let pdf_m = pdf_ggx_vndf(&wi, &m, alpha);
            let pdf_denom = 4.0 * cos_o_m.abs();
            if pdf_denom <= 1e-6 {
                return eval_result;
            }
            let pdf = f * pdf_m / pdf_denom;

            eval_result.value = value;
            eval_result.pdf = pdf;
            return eval_result;
        }

        let eta = eta_t / eta_i;
        let mut m = wi + wo * eta;
        if m.norm_squared() <= 0.0 {
            return eval_result;
        }
        m = m.normalize();
        if m.z <= 0.0 {
            m = -m;
        }

        let cos_i_m = wi.dot(&m);
        let cos_o_m = wo.dot(&m);
        if cos_i_m <= 0.0 || cos_o_m >= 0.0 {
            return eval_result;
        }

        let d = ggx_d(m.z, alpha);
        let g = ggx_g(cos_i.abs(), cos_o.abs(), alpha);
        let f = fresnel_dielectric(cos_i_m, eta_i, eta_t);
        let denom = cos_i_m + eta * cos_o_m;
        if denom.abs() <= 1e-6 {
            return eval_result;
        }

        let cos_i_abs = cos_i.abs();
        if cos_i_abs <= 1e-6 {
            return eval_result;
        }
        let scale = 1.0 / (eta * eta);
        let numer = (1.0 - f) * d * g * (eta * eta) * cos_i_m * cos_o_m;
        let value = self.specular_transmittance * (scale * (numer / (cos_i_abs * denom * denom)).abs());

        let pdf_m = pdf_ggx_vndf(&wi, &m, alpha);
        let pdf_denom = (denom * denom).abs();
        if pdf_denom <= 1e-6 {
            return eval_result;
        }
        let pdf = (1.0 - f) * pdf_m * (eta * eta) * cos_o_m.abs() / pdf_denom;

        eval_result.value = value;
        eval_result.pdf = pdf;
        eval_result
    }

    fn sample(&self, u1: Vector2f, u2: Vector2f, wi_in: Vector3f) -> BSDFSampleRecord {
        let mut sampling_record = BSDFSampleRecord::default();
        sampling_record.wi = wi_in;
        if wi_in.z == 0.0 {
            return sampling_record;
        }

        let mut wi = wi_in;
        let mut flip = 1.0;
        if wi.z < 0.0 {
            wi = -wi;
            flip = -1.0;
        }

        let (eta_i, eta_t) = if flip > 0.0 {
            (self.ext_ior, self.int_ior)
        } else {
            (self.int_ior, self.ext_ior)
        };

        let alpha = self.alpha.max(1e-4);
        let m = sample_ggx_vndf(&wi, &u1, alpha);
        let cos_i_m = wi.dot(&m);
        if cos_i_m <= 0.0 {
            return sampling_record;
        }

        let f = fresnel_dielectric(cos_i_m, eta_i, eta_t);
        let mut wo;
        let pdf;

        if u2.x < f {
            wo = reflect(&wi, &m);
            if wo.z <= 0.0 {
                return sampling_record;
            }
            let pdf_m = pdf_ggx_vndf(&wi, &m, alpha);
            let pdf_denom = 4.0 * wo.dot(&m).abs();
            if pdf_denom <= 1e-6 {
                return sampling_record;
            }
            pdf = f * pdf_m / pdf_denom;
        } else {
            let eta_i_over_t = eta_i / eta_t;
            let eta_t_over_i = eta_t / eta_i;
            let refracted = refract(&wi, &m, eta_i_over_t);
            if refracted.is_none() {
                wo = reflect(&wi, &m);
                if wo.z <= 0.0 {
                    return sampling_record;
                }
                let pdf_m = pdf_ggx_vndf(&wi, &m, alpha);
                let pdf_denom = 4.0 * wo.dot(&m).abs();
                if pdf_denom <= 1e-6 {
                    return sampling_record;
                }
                pdf = f * pdf_m / pdf_denom;
            } else {
                wo = refracted.unwrap();
                if wo.z >= 0.0 {
                    return sampling_record;
                }
                if wo.dot(&m) >= 0.0 {
                    return sampling_record;
                }
                let pdf_m = pdf_ggx_vndf(&wi, &m, alpha);
                let denom = cos_i_m + eta_t_over_i * wo.dot(&m);
                if denom.abs() <= 1e-6 {
                    return sampling_record;
                }
                let pdf_denom = (denom * denom).abs();
                if pdf_denom <= 1e-6 {
                    return sampling_record;
                }
                pdf = (1.0 - f) * pdf_m * (eta_t_over_i * eta_t_over_i) * wo.dot(&m).abs() / pdf_denom;
            }
        }

        if flip < 0.0 {
            wo = -wo;
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
