// Copyright @yucwang 2026

use std::sync::Arc;

use crate::core::bsdf::{BSDFSampleRecord, BSDFEvalResult, BSDF};
use crate::core::computation_node::{ComputationNode, generate_node_id};
use crate::math::constants::{Float, Vector2f, Vector3f};

pub struct BlendBSDF {
    id: String,
    bsdf_a: Arc<dyn BSDF>,
    bsdf_b: Arc<dyn BSDF>,
    weight: Float,
}

impl ComputationNode for BlendBSDF {
    fn id(&self) -> &str {
        &self.id
    }

    fn to_string(&self) -> String {
        String::from("BlendBSDF")
    }
}

impl BlendBSDF {
    pub fn new(bsdf_a: Arc<dyn BSDF>, bsdf_b: Arc<dyn BSDF>, weight: Float, id: Option<String>) -> Self {
        let weight = weight.max(0.0).min(1.0);
        Self { id: id.unwrap_or_else(|| generate_node_id("BlendBSDF")), bsdf_a, bsdf_b, weight }
    }
}

impl BSDF for BlendBSDF {
    fn eval(&self, sample_record: BSDFSampleRecord) -> BSDFEvalResult {
        if sample_record.wi.z <= 0.0 {
            return BSDFEvalResult::default();
        }
        let rec_a = BSDFSampleRecord::new(sample_record.wi, sample_record.wo, 0.0, sample_record.uv);
        let rec_b = BSDFSampleRecord::new(sample_record.wi, sample_record.wo, 0.0, sample_record.uv);

        let eval_a = self.bsdf_a.eval(rec_a);
        let eval_b = self.bsdf_b.eval(rec_b);

        let value = eval_a.value * self.weight + eval_b.value * (1.0 - self.weight);
        let pdf = eval_a.pdf * self.weight + eval_b.pdf * (1.0 - self.weight);

        BSDFEvalResult { value, pdf }
    }

    fn sample(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFSampleRecord {
        if wi.z <= 0.0 {
            return BSDFSampleRecord::default();
        }
        let choose_a = u1.x < self.weight;
        let mut remapped = u1;
        if self.weight > 0.0 && self.weight < 1.0 {
            if choose_a {
                remapped.x = u1.x / self.weight;
            } else {
                remapped.x = (u1.x - self.weight) / (1.0 - self.weight);
            }
        }

        let sample = if choose_a {
            self.bsdf_a.sample(remapped, u2, wi)
        } else {
            self.bsdf_b.sample(remapped, u2, wi)
        };

        let eval_a = self.bsdf_a.eval(BSDFSampleRecord::new(wi, sample.wo, 0.0, sample.uv));
        let eval_b = self.bsdf_b.eval(BSDFSampleRecord::new(wi, sample.wo, 0.0, sample.uv));
        let pdf = eval_a.pdf * self.weight + eval_b.pdf * (1.0 - self.weight);

        BSDFSampleRecord::new(wi, sample.wo, pdf, sample.uv)
    }

    fn sample_and_eval(&self, u1: Vector2f, u2: Vector2f, wi: Vector3f) -> BSDFEvalResult {
        let sampling_record = self.sample(u1, u2, wi);
        self.eval(sampling_record)
    }
}
