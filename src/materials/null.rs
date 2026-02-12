// Copyright @yucwang 2026

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::math::constants::{Vector2f, Vector3f};
use crate::math::spectrum::RGBSpectrum;

pub struct NullBSDF {
    id: String,
}

impl NullBSDF {
    pub fn new(id: Option<String>) -> Self {
        Self {
            id: id.unwrap_or_else(|| generate_node_id("NullBSDF")),
        }
    }
}

impl ComputationNode for NullBSDF {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        format!("NullBSDF [id={}]", self.id)
    }
}

impl BSDF for NullBSDF {
    fn is_null(&self) -> bool {
        true
    }

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

#[cfg(test)]
mod tests {
    use super::NullBSDF;
    use crate::core::bsdf::{BSDF, BSDFSampleRecord};
    use crate::math::constants::{Vector2f, Vector3f};

    fn assert_close(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5, "expected {} ≈ {}", a, b);
    }

    #[test]
    fn test_sample_opposite_direction() {
        let bsdf = NullBSDF::new(None);
        let wi = Vector3f::new(0.1, -0.2, 0.97).normalize();
        let sample = bsdf.sample(Vector2f::new(0.3, 0.7), Vector2f::new(0.1, 0.9), wi);
        assert_close(sample.wo.x, -wi.x);
        assert_close(sample.wo.y, -wi.y);
        assert_close(sample.wo.z, -wi.z);
        assert_close(sample.pdf, 1.0);
    }

    #[test]
    fn test_eval_delta_response() {
        let bsdf = NullBSDF::new(None);
        let wi = Vector3f::new(0.0, 0.0, 1.0);
        let wo = -wi;
        let mut record = BSDFSampleRecord::default();
        record.wi = wi;
        record.wo = wo;
        let result = bsdf.eval(record);
        assert_close(result.pdf, 1.0);
        assert_close(result.value[0], 1.0);
        assert_close(result.value[1], 1.0);
        assert_close(result.value[2], 1.0);
    }

    #[test]
    fn test_eval_non_delta_is_zero() {
        let bsdf = NullBSDF::new(None);
        let mut record = BSDFSampleRecord::default();
        record.wi = Vector3f::new(0.0, 0.0, 1.0);
        record.wo = Vector3f::new(0.0, 1.0, 0.0);
        let result = bsdf.eval(record);
        assert_close(result.pdf, 0.0);
        assert_close(result.value[0], 0.0);
        assert_close(result.value[1], 0.0);
        assert_close(result.value[2], 0.0);
    }

    #[test]
    fn test_sample_and_eval_matches_eval() {
        let bsdf = NullBSDF::new(None);
        let wi = Vector3f::new(0.0, 0.0, 1.0);
        let result = bsdf.sample_and_eval(Vector2f::new(0.2, 0.8), Vector2f::new(0.4, 0.6), wi);
        assert_close(result.pdf, 1.0);
        assert_close(result.value[0], 1.0);
        assert_close(result.value[1], 1.0);
        assert_close(result.value[2], 1.0);
    }
}
