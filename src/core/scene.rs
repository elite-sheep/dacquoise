// Copyright @yucwang 2026

use crate::core::bsdf::BSDF;
use crate::core::bvh::BVH;
use crate::core::interaction::SurfaceIntersection;
use crate::core::sensor::Sensor;
use crate::core::shape::Shape;
use crate::math::ray::Ray3f;
use crate::math::spectrum::{RGBSpectrum, Spectrum};
use crate::math::constants::{ Float, Vector2f };
use std::sync::Arc;

pub struct SceneObject {
    pub shape: Box<dyn Shape>,
    pub material: Arc<dyn BSDF>,
    pub emission: RGBSpectrum,
    pub name: Option<String>,
}

impl SceneObject {
    pub fn new(shape: Box<dyn Shape>, material: Arc<dyn BSDF>) -> Self {
        Self { shape, material, emission: RGBSpectrum::default(), name: None }
    }

    pub fn with_emission(shape: Box<dyn Shape>, material: Arc<dyn BSDF>, emission: RGBSpectrum) -> Self {
        Self { shape, material, emission, name: None }
    }

    pub fn with_name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }
}

pub struct Scene {
    objects: Vec<SceneObject>,
    sensors: Vec<Box<dyn Sensor>>,
    base_dir: std::path::PathBuf,
    bvh: Option<BVH>,
}

impl Scene {
    pub fn new() -> Self {
        Self {
            objects: Vec::new(),
            sensors: Vec::new(),
            base_dir: std::path::PathBuf::new(),
            bvh: None,
        }
    }

    pub fn with_objects(objects: Vec<SceneObject>) -> Self {
        Self { objects, sensors: Vec::new(), base_dir: std::path::PathBuf::new(), bvh: None }
    }

    pub fn with_objects_and_sensors(objects: Vec<SceneObject>, sensors: Vec<Box<dyn Sensor>>) -> Self {
        Self { objects, sensors, base_dir: std::path::PathBuf::new(), bvh: None }
    }

    pub fn add_object(&mut self, object: SceneObject) {
        self.objects.push(object);
        self.bvh = None;
    }

    pub fn objects(&self) -> &Vec<SceneObject> {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut Vec<SceneObject> {
        self.bvh = None;
        &mut self.objects
    }

    pub fn sensors(&self) -> &Vec<Box<dyn Sensor>> {
        &self.sensors
    }

    pub fn sensors_mut(&mut self) -> &mut Vec<Box<dyn Sensor>> {
        &mut self.sensors
    }

    pub fn add_sensor(&mut self, sensor: Box<dyn Sensor>) {
        self.sensors.push(sensor);
    }

    pub fn take_sensor(&mut self, camera_id: usize) -> Option<Box<dyn Sensor>> {
        if camera_id < self.sensors.len() {
            Some(self.sensors.remove(camera_id))
        } else {
            None
        }
    }

    pub fn insert_sensor(&mut self, camera_id: usize, sensor: Box<dyn Sensor>) {
        if camera_id <= self.sensors.len() {
            self.sensors.insert(camera_id, sensor);
        } else {
            self.sensors.push(sensor);
        }
    }

    pub fn set_base_dir(&mut self, base_dir: std::path::PathBuf) {
        self.base_dir = base_dir;
    }

    pub fn base_dir(&self) -> &std::path::Path {
        &self.base_dir
    }

    pub fn camera(&self, camera_id: usize) -> Option<&dyn Sensor> {
        self.sensors.get(camera_id).map(|s| s.as_ref())
    }

    pub fn camera_mut(&mut self, camera_id: usize) -> Option<&mut (dyn Sensor + '_)> {
        match self.sensors.get_mut(camera_id) {
            Some(sensor) => Some(sensor.as_mut()),
            None => None,
        }
    }

    pub fn len(&self) -> usize {
        self.objects.len()
    }

    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    pub fn build_bvh(&mut self) {
        if self.objects.is_empty() {
            self.bvh = None;
            return;
        }

        let mut prim_bounds = Vec::with_capacity(self.objects.len());
        let mut prim_centroids = Vec::with_capacity(self.objects.len());
        for obj in &self.objects {
            let bounds = obj.shape.bounding_box();
            prim_centroids.push(bounds.center());
            prim_bounds.push(bounds);
        }

        self.bvh = Some(BVH::new(prim_bounds, prim_centroids));
    }

    pub fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let mut closest_hit: Option<SurfaceIntersection> = None;
        let mut closest_t = std::f32::MAX;

        let emitter_count = self.objects.iter().filter(|o| !o.emission.is_black()).count() as Float;
        if let Some(bvh) = &self.bvh {
            if let Some((idx, hit)) = bvh.ray_intersection(ray, |prim_idx, ray| {
                self.objects[prim_idx].shape.ray_intersection(ray).map(|h| {
                    let t = h.t();
                    (h, t)
                })
            }) {
                let object = &self.objects[idx];
                let mut hit = hit.with_le(object.emission).with_material(object.material.clone());
                if !object.emission.is_black() && emitter_count > 0.0 {
                    let area = object.shape.surface_area().max(1e-6);
                    let pdf_area = 1.0 / area;
                    let pdf_select = 1.0 / emitter_count;
                    hit = hit.with_light_pdf_area(Some(pdf_area * pdf_select));
                }
                return Some(hit);
            }
        } else {
            for object in &self.objects {
                if let Some(hit) = object.shape.ray_intersection(ray) {
                    let hit_t = hit.t();
                    if hit_t < closest_t {
                        closest_t = hit_t;
                        let mut hit = hit.with_le(object.emission).with_material(object.material.clone());
                        if !object.emission.is_black() && emitter_count > 0.0 {
                            let area = object.shape.surface_area().max(1e-6);
                            let pdf_area = 1.0 / area;
                            let pdf_select = 1.0 / emitter_count;
                            hit = hit.with_light_pdf_area(Some(pdf_area * pdf_select));
                        }
                        closest_hit = Some(hit);
                    }
                }
            }
        }

        closest_hit
    }

    pub fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        if let Some(bvh) = &self.bvh {
            return bvh.ray_intersection_t(ray, |prim_idx, ray| {
                self.objects[prim_idx].shape.ray_intersection_t(ray)
            });
        }

        for object in &self.objects {
            if object.shape.ray_intersection_t(ray) {
                return true;
            }
        }
        false
    }

    pub fn ray_intersection_with_object_name(&self, ray: &Ray3f) -> Option<(SurfaceIntersection, Option<&str>)> {
        let mut closest_hit: Option<SurfaceIntersection> = None;
        let mut closest_name: Option<&str> = None;
        let mut closest_t = std::f32::MAX;

        let emitter_count = self.objects.iter().filter(|o| !o.emission.is_black()).count() as Float;
        if let Some(bvh) = &self.bvh {
            if let Some((idx, hit)) = bvh.ray_intersection(ray, |prim_idx, ray| {
                self.objects[prim_idx].shape.ray_intersection(ray).map(|h| {
                    let t = h.t();
                    (h, t)
                })
            }) {
                let object = &self.objects[idx];
                let mut hit = hit.with_le(object.emission).with_material(object.material.clone());
                if !object.emission.is_black() && emitter_count > 0.0 {
                    let area = object.shape.surface_area().max(1e-6);
                    let pdf_area = 1.0 / area;
                    let pdf_select = 1.0 / emitter_count;
                    hit = hit.with_light_pdf_area(Some(pdf_area * pdf_select));
                }
                return Some((hit, object.name.as_deref()));
            }
        } else {
            for object in &self.objects {
                if let Some(hit) = object.shape.ray_intersection(ray) {
                    let hit_t = hit.t();
                    if hit_t < closest_t {
                        closest_t = hit_t;
                        let mut hit = hit.with_le(object.emission).with_material(object.material.clone());
                        if !object.emission.is_black() && emitter_count > 0.0 {
                            let area = object.shape.surface_area().max(1e-6);
                            let pdf_area = 1.0 / area;
                            let pdf_select = 1.0 / emitter_count;
                            hit = hit.with_light_pdf_area(Some(pdf_area * pdf_select));
                        }
                        closest_hit = Some(hit);
                        closest_name = object.name.as_deref();
                    }
                }
            }
        }

        closest_hit.map(|hit| (hit, closest_name))
    }

    pub fn sample_emitter(&self, u1: Float, u2: &Vector2f) -> Option<crate::core::interaction::SurfaceSampleRecord> {
        let emitter_indices: Vec<usize> = self.objects.iter()
            .enumerate()
            .filter(|(_, obj)| !obj.emission.is_black())
            .map(|(idx, _)| idx)
            .collect();

        if emitter_indices.is_empty() {
            return None;
        }

        let emitter_count = emitter_indices.len() as Float;
        let mut emitter_index = (u1 * emitter_count) as usize;
        if emitter_index >= emitter_indices.len() {
            emitter_index = emitter_indices.len() - 1;
        }

        let object = &self.objects[emitter_indices[emitter_index]];
        let sample = object.shape.sample(u2);
        let intersection = sample.intersection().with_le(object.emission);
        let pdf = sample.pdf() / emitter_count;

        Some(crate::core::interaction::SurfaceSampleRecord::new(intersection, pdf))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::interaction::SurfaceSampleRecord;
    use crate::math::aabb::AABB;
    use crate::math::constants::{Float, Vector2f, Vector3f};
    use crate::math::bitmap::Bitmap;
    use crate::math::ray::Ray3f;

    struct TestShape {
        t: Float,
    }

    impl TestShape {
        fn new(t: Float) -> Self {
            Self { t }
        }
    }

    impl Shape for TestShape {
        fn bounding_box(&self) -> AABB {
            AABB::new(Vector3f::zeros(), Vector3f::zeros())
        }

        fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
            if self.t < ray.min_t || self.t > ray.max_t {
                return None;
            }

            let p = ray.at(self.t);
            let n = Vector3f::new(0.0, 0.0, 1.0);
            let uv = Vector2f::new(0.0, 0.0);
            Some(SurfaceIntersection::new(p, n, n, uv, self.t, RGBSpectrum::default(), None, None))
        }

        fn ray_intersection_t(&self, _ray: &Ray3f) -> bool {
            true
        }

        fn sample(&self, _u: &Vector2f) -> SurfaceSampleRecord {
            let p = Vector3f::zeros();
            let n = Vector3f::new(0.0, 0.0, 1.0);
            let uv = Vector2f::new(0.0, 0.0);
            let intersection = SurfaceIntersection::new(p, n, n, uv, self.t, RGBSpectrum::default(), None, None);
            SurfaceSampleRecord::new(intersection, 1.0)
        }

        fn surface_area(&self) -> Float {
            1.0
        }
    }

    struct TestBSDF;

    impl BSDF for TestBSDF {
        fn eval(&self, _sample_record: crate::core::bsdf::BSDFSampleRecord) -> crate::core::bsdf::BSDFEvalResult {
            crate::core::bsdf::BSDFEvalResult::default()
        }

        fn sample(&self,
                  _u1: Vector2f,
                  _u2: Vector2f,
                  _wi: Vector3f) -> crate::core::bsdf::BSDFSampleRecord {
            crate::core::bsdf::BSDFSampleRecord::default()
        }

        fn sample_and_eval(&self,
                           _u1: Vector2f,
                           _u2: Vector2f,
                           _wi: Vector3f) -> crate::core::bsdf::BSDFEvalResult {
            crate::core::bsdf::BSDFEvalResult::default()
        }
    }

    #[test]
    fn test_scene_ray_intersection_closest_hit() {
        let mut scene = Scene::new();
        scene.add_object(SceneObject::new(Box::new(TestShape::new(5.0)), Arc::new(TestBSDF)));
        scene.add_object(SceneObject::new(Box::new(TestShape::new(2.0)), Arc::new(TestBSDF)));
        scene.add_object(SceneObject::new(Box::new(TestShape::new(10.0)), Arc::new(TestBSDF)));

        let ray = Ray3f::new(Vector3f::zeros(), Vector3f::new(0.0, 0.0, 1.0), None, None);
        let hit = scene.ray_intersection(&ray).expect("expected intersection");

        assert_eq!(hit.t(), 2.0);
    }

    struct TestSensor {
        bitmap: Bitmap,
    }

    impl TestSensor {
        fn new() -> Self {
            Self { bitmap: Bitmap::new(2, 2) }
        }
    }

    impl Sensor for TestSensor {
        fn sample_ray(&self, _u: &Vector2f) -> Ray3f {
            Ray3f::new(Vector3f::zeros(), Vector3f::new(0.0, 0.0, 1.0), None, None)
        }

        fn bitmap(&self) -> &Bitmap {
            &self.bitmap
        }

        fn bitmap_mut(&mut self) -> &mut Bitmap {
            &mut self.bitmap
        }
    }

    #[test]
    fn test_scene_camera_access() {
        let mut scene = Scene::new();
        assert!(scene.camera(0).is_none());

        scene.add_sensor(Box::new(TestSensor::new()));
        scene.add_sensor(Box::new(TestSensor::new()));

        assert!(scene.camera(0).is_some());
        assert!(scene.camera(1).is_some());
        assert!(scene.camera(2).is_none());
    }
}
