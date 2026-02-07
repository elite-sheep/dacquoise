// Copyright @yucwang 2026

use crate::core::bsdf::BSDF;
use crate::core::bvh::BVH;
use crate::core::emitter::{Emitter, EmitterFlag, EmitterSample};
use crate::core::interaction::SurfaceIntersection;
use crate::core::medium::Medium;
use crate::core::sensor::Sensor;
use crate::core::shape::Shape;
use crate::core::volume::Volume;
use crate::emitters::area::AreaEmitter;
use crate::math::aabb::AABB;
use crate::math::constants::{ Float, MatrixXF, Vector2f, Vector3f };
use crate::math::ray::Ray3f;
use crate::math::spectrum::{RGBSpectrum, Spectrum};
use std::collections::HashMap;
use std::sync::Arc;

pub struct SceneObject {
    pub shape: Arc<dyn Shape>,
    pub material: Arc<dyn BSDF>,
    pub emission: RGBSpectrum,
    pub name: Option<String>,
    pub interior_medium: Option<Arc<dyn Medium>>,
}

impl SceneObject {
    pub fn new(shape: Arc<dyn Shape>, material: Arc<dyn BSDF>) -> Self {
        Self { shape, material, emission: RGBSpectrum::default(), name: None, interior_medium: None }
    }

    pub fn with_emission(shape: Arc<dyn Shape>, material: Arc<dyn BSDF>, emission: RGBSpectrum) -> Self {
        Self { shape, material, emission, name: None, interior_medium: None }
    }

    pub fn with_name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }

    pub fn with_interior_medium(mut self, medium: Option<Arc<dyn Medium>>) -> Self {
        self.interior_medium = medium;
        self
    }

    pub fn interior_medium(&self) -> Option<Arc<dyn Medium>> {
        self.interior_medium.clone()
    }

    pub fn shape(&self) -> &Arc<dyn Shape> {
        &self.shape
    }
}

#[derive(Clone, Copy)]
pub struct RawDataView {
    pub ptr: *const Float,
    pub rows: usize,
    pub cols: usize,
}

unsafe impl Send for RawDataView {}
unsafe impl Sync for RawDataView {}

impl RawDataView {
    pub fn from_matrix(matrix: &MatrixXF) -> Self {
        Self {
            ptr: matrix.as_slice().as_ptr(),
            rows: matrix.nrows(),
            cols: matrix.ncols(),
        }
    }
}

pub struct Scene {
    objects: Vec<SceneObject>,
    sensors: Vec<Box<dyn Sensor>>,
    emitters: Vec<Box<dyn Emitter>>,
    volumes: HashMap<String, Arc<dyn Volume>>,
    media: HashMap<String, Arc<dyn Medium>>,
    raw_data: HashMap<String, RawDataView>,
    scene_bounds: AABB,
    base_dir: std::path::PathBuf,
    bvh: Option<BVH>,
}

impl Scene {
    pub fn new() -> Self {
        Self {
            objects: Vec::new(),
            sensors: Vec::new(),
            emitters: Vec::new(),
            volumes: HashMap::new(),
            media: HashMap::new(),
            raw_data: HashMap::new(),
            scene_bounds: AABB::default(),
            base_dir: std::path::PathBuf::new(),
            bvh: None,
        }
    }

    pub fn with_objects(objects: Vec<SceneObject>) -> Self {
        let emitters = Self::emitters_from_objects(&objects);
        Self {
            objects,
            sensors: Vec::new(),
            emitters,
            volumes: HashMap::new(),
            media: HashMap::new(),
            raw_data: HashMap::new(),
            scene_bounds: AABB::default(),
            base_dir: std::path::PathBuf::new(),
            bvh: None,
        }
    }

    pub fn with_objects_and_sensors(objects: Vec<SceneObject>, sensors: Vec<Box<dyn Sensor>>) -> Self {
        let emitters = Self::emitters_from_objects(&objects);
        Self {
            objects,
            sensors,
            emitters,
            volumes: HashMap::new(),
            media: HashMap::new(),
            raw_data: HashMap::new(),
            scene_bounds: AABB::default(),
            base_dir: std::path::PathBuf::new(),
            bvh: None,
        }
    }

    pub fn add_object(&mut self, object: SceneObject) {
        let emitter = if !object.emission.is_black() {
            Some(AreaEmitter::from_shape(object.shape.clone(), object.emission))
        } else {
            None
        };
        self.objects.push(object);
        if let Some(emitter) = emitter {
            self.emitters.push(Box::new(emitter));
        }
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

    pub fn add_emitter(&mut self, emitter: Box<dyn Emitter>) {
        self.emitters.push(emitter);
    }

    pub fn emitters(&self) -> &Vec<Box<dyn Emitter>> {
        &self.emitters
    }

    pub fn emitters_mut(&mut self) -> &mut Vec<Box<dyn Emitter>> {
        &mut self.emitters
    }

    pub fn add_volume(&mut self, id: String, volume: Arc<dyn Volume>) {
        self.volumes.insert(id, volume);
    }

    pub fn volume(&self, id: &str) -> Option<Arc<dyn Volume>> {
        self.volumes.get(id).cloned()
    }

    pub fn volumes(&self) -> &HashMap<String, Arc<dyn Volume>> {
        &self.volumes
    }

    pub fn add_medium(&mut self, id: String, medium: Arc<dyn Medium>) {
        self.media.insert(id, medium);
    }

    pub fn medium(&self, id: &str) -> Option<Arc<dyn Medium>> {
        self.media.get(id).cloned()
    }

    pub fn media(&self) -> &HashMap<String, Arc<dyn Medium>> {
        &self.media
    }

    pub fn add_raw_data(&mut self, key: String, view: RawDataView) {
        self.raw_data.insert(key, view);
    }

    pub fn set_raw_data(&mut self, raw_data: HashMap<String, RawDataView>) {
        self.raw_data = raw_data;
    }

    pub fn raw_data(&self) -> &HashMap<String, RawDataView> {
        &self.raw_data
    }

    pub fn raw_data_view(&self, key: &str) -> Option<RawDataView> {
        self.raw_data.get(key).copied()
    }

    pub fn scene_bounds(&self) -> &AABB {
        &self.scene_bounds
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
        let mut prim_bounds = Vec::with_capacity(self.objects.len());
        let mut prim_centroids = Vec::with_capacity(self.objects.len());
        let mut scene_bounds = crate::math::aabb::AABB::default();
        for obj in &self.objects {
            let bounds = obj.shape.bounding_box();
            prim_centroids.push(bounds.center());
            prim_bounds.push(bounds);
            scene_bounds.expand_by_aabb(&bounds);
        }

        self.bvh = Some(BVH::new(prim_bounds, prim_centroids));

        self.scene_bounds = scene_bounds;

        for emitter in &mut self.emitters {
            emitter.set_scene_bounds(&scene_bounds);
        }
    }

    fn emitters_from_objects(objects: &[SceneObject]) -> Vec<Box<dyn Emitter>> {
        let mut emitters: Vec<Box<dyn Emitter>> = Vec::new();
        for object in objects {
            if !object.emission.is_black() {
                emitters.push(Box::new(AreaEmitter::from_shape(
                    object.shape.clone(),
                    object.emission,
                )));
            }
        }
        emitters
    }

    pub fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let bvh = self.bvh.as_ref().expect("BVH must be built before ray_intersection");
        if let Some((idx, hit)) = bvh.ray_intersection(ray, |prim_idx, ray| {
            self.objects[prim_idx].shape.ray_intersection(ray).map(|h| {
                let t = h.t();
                (h, t)
            })
        }) {
            let object = &self.objects[idx];
            let result = hit
                .with_le(object.emission)
                .with_material(object.material.clone())
                .with_object_index(Some(idx));
            Some(result)
        } else {
            None
        }
    }

    pub fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        let bvh = self.bvh.as_ref().expect("BVH must be built before ray_intersection_t");
        bvh.ray_intersection_t(ray, |prim_idx, ray| {
            self.objects[prim_idx].shape.ray_intersection_t(ray)
        })
    }

    pub fn pdf_light(&self, intersection: &SurfaceIntersection, ref_p: &Vector3f) -> Option<Float> {
        let obj_idx = intersection.object_index()?;
        if self.emitters.is_empty() {
            return None;
        }
        let object = self.objects.get(obj_idx)?;
        if object.emission.is_black() {
            return None;
        }
        let area = object.shape.surface_area();
        if area <= 0.0 {
            return None;
        }

        let to_light = intersection.p() - *ref_p;
        let dist2 = to_light.dot(&to_light);
        if dist2 <= 0.0 {
            return None;
        }
        let dir = to_light / dist2.sqrt();
        let cos_light = intersection.geo_normal().dot(&(-dir)).max(0.0);
        if cos_light <= 0.0 {
            return None;
        }

        let select_pdf = 1.0 / (self.emitters.len() as Float);
        let area_pdf = 1.0 / area.max(1e-6);
        Some(area_pdf * select_pdf * dist2 / cos_light)
    }

    pub fn sample_emitter(&self, u1: Float, u2: &Vector2f) -> Option<EmitterSample> {
        if self.emitters.is_empty() {
            return None;
        }

        let emitter_count = self.emitters.len() as Float;
        let mut emitter_index = (u1 * emitter_count) as usize;
        if emitter_index >= self.emitters.len() {
            emitter_index = self.emitters.len() - 1;
        }

        let emitter = &self.emitters[emitter_index];
        let select_pdf = 1.0 / emitter_count;
        let flag = emitter.get_flag();
        if flag.contains(EmitterFlag::DIRECTION) {
            let sample = emitter.sample_position(u2);
            let direction = emitter.sample_direction(u2, sample.intersection());
            let pdf = if flag.contains(EmitterFlag::DELTA) {
                select_pdf
            } else {
                emitter.pdf_direction(sample.intersection(), &direction) * select_pdf
            };
            Some(EmitterSample::Direction {
                direction,
                irradiance: sample.intersection().le(),
                pdf,
                is_delta: flag.contains(EmitterFlag::DELTA),
            })
        } else if flag.contains(EmitterFlag::SURFACE) {
            let mut sample = emitter.sample_position(u2);
            let pdf = sample.pdf() * select_pdf;
            sample.set_pdf(pdf);
            Some(EmitterSample::Surface(sample))
        } else {
            None
        }
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
        scene.add_object(SceneObject::new(Arc::new(TestShape::new(5.0)), Arc::new(TestBSDF)));
        scene.add_object(SceneObject::new(Arc::new(TestShape::new(2.0)), Arc::new(TestBSDF)));
        scene.add_object(SceneObject::new(Arc::new(TestShape::new(10.0)), Arc::new(TestBSDF)));
        scene.build_bvh();

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
