// Copyright @yucwang 2026

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use quick_xml::events::Event;
use quick_xml::Reader;

use crate::core::scene::{RawDataView, Scene};
use crate::core::medium::Medium;
use crate::core::volume::Volume;
use crate::math::constants::{Float, Matrix3f, Matrix4f, Vector3f};
use crate::math::transform::Transform;
use crate::sensors::perspective::PerspectiveCamera;
use crate::materials::lambertian_diffuse::LambertianDiffuseBSDF;
use crate::materials::null::NullBSDF;
use crate::materials::roughconductor::RoughConductorBSDF;
use crate::materials::roughdielectric::RoughDielectricBSDF;
use crate::materials::blend::BlendBSDF;
use crate::math::spectrum::RGBSpectrum;
use crate::shapes::rectangle::Rectangle;
use crate::shapes::cube::Cube;
use crate::shapes::triangle_mesh::TriangleMesh;
use crate::textures::constant::ConstantTexture;
use crate::textures::image::{FilterMode, ImageTexture, WrapMode};
use crate::emitters::directional::DirectionalEmitter;
use crate::emitters::envmap::EnvMap;
use crate::core::scene::SceneObject;
use crate::core::bsdf::BSDF;
use crate::core::integrator::Integrator;
use crate::integrators::path::PathIntegrator;
use crate::integrators::raymarching::RaymarchingIntegrator;
use crate::media::homogeneous::HomogeneousMedium;
use crate::media::heterogeneous::HeterogeneousMedium;
use crate::volumes::constant::ConstantVolume;
use crate::volumes::grid::GridVolume;
use crate::volumes::{VolumeFilterMode, VolumeWrapMode};
use std::sync::Arc;
use nalgebra as na;

#[derive(Debug)]
struct BsdfTextureState {
    tex_type: Option<String>,
    tex_name: Option<String>,
    tex_filename: Option<String>,
    raw: bool,
    filter_type: Option<String>,
    wrap_mode: Option<String>,
    to_uv: Matrix3f,
}

struct BsdfState {
    bsdf_type: String,
    id: Option<String>,
    reflectance: Option<RGBSpectrum>,
    specular_reflectance: Option<RGBSpectrum>,
    specular_transmittance: Option<RGBSpectrum>,
    eta: Option<RGBSpectrum>,
    k: Option<RGBSpectrum>,
    alpha: Option<Float>,
    alpha_u: Option<Float>,
    alpha_v: Option<Float>,
    distribution: Option<String>,
    int_ior: Option<Float>,
    ext_ior: Option<Float>,
    weight: Option<Float>,
    sample_visible: bool,
    texture: Option<BsdfTextureState>,
    texture_active: bool,
    children: Vec<Arc<dyn BSDF>>,
}

impl BsdfState {
    fn new(bsdf_type: String, id: Option<String>) -> Self {
        Self {
            bsdf_type,
            id,
            reflectance: None,
            specular_reflectance: None,
            specular_transmittance: None,
            eta: None,
            k: None,
            alpha: None,
            alpha_u: None,
            alpha_v: None,
            distribution: None,
            int_ior: None,
            ext_ior: None,
            weight: None,
            sample_visible: true,
            texture: None,
            texture_active: false,
            children: Vec::new(),
        }
    }
}

#[derive(Debug)]
struct VolumeState {
    volume_type: String,
    id: Option<String>,
    value_rgb: Option<RGBSpectrum>,
    value_scalar: Option<Float>,
    filename: Option<String>,
    filter_type: Option<String>,
    wrap_mode: Option<String>,
    to_world: Matrix4f,
    use_grid_bbox: Option<bool>,
}

impl VolumeState {
    fn new(volume_type: String, id: Option<String>) -> Self {
        Self {
            volume_type,
            id,
            value_rgb: None,
            value_scalar: None,
            filename: None,
            filter_type: None,
            wrap_mode: None,
            to_world: Matrix4f::identity(),
            use_grid_bbox: None,
        }
    }
}

#[derive(Debug)]
struct MediumState {
    medium_type: String,
    id: Option<String>,
    name: Option<String>,
    sigma_t: Option<RGBSpectrum>,
    sigma_t_scalar: Option<Float>,
    sigma_t_ref: Option<String>,
    albedo: Option<RGBSpectrum>,
    albedo_scalar: Option<Float>,
    albedo_ref: Option<String>,
    scale: Option<Float>,
    phase_function: Option<String>,
}

impl MediumState {
    fn new(medium_type: String, id: Option<String>) -> Self {
        Self {
            medium_type,
            id,
            name: None,
            sigma_t: None,
            sigma_t_scalar: None,
            sigma_t_ref: None,
            albedo: None,
            albedo_scalar: None,
            albedo_ref: None,
            scale: None,
            phase_function: None,
        }
    }
}

#[derive(Debug)]
pub enum SceneLoadError {
    Io(std::io::Error),
    Parse(String),
    MissingField(&'static str),
}

impl From<std::io::Error> for SceneLoadError {
    fn from(err: std::io::Error) -> Self {
        SceneLoadError::Io(err)
    }
}

pub fn load_scene<P: AsRef<Path>>(path: P) -> Result<Scene, SceneLoadError> {
    let result = load_scene_with_settings(path)?;
    Ok(result.scene)
}

pub struct SceneLoadResult {
    pub scene: Scene,
    pub integrator: Option<Box<dyn Integrator>>,
    pub integrator_type: Option<String>,
    pub samples_per_pixel: Option<u32>,
    pub max_depth: Option<u32>,
}

pub fn load_scene_with_settings<P: AsRef<Path>>(path: P) -> Result<SceneLoadResult, SceneLoadError> {
    let path = path.as_ref();
    let xml = fs::read_to_string(path)?;
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    parse_scene(&xml, base_dir)
}

fn parse_scene(xml: &str, base_dir: &Path) -> Result<SceneLoadResult, SceneLoadError> {
    let mut reader = Reader::from_str(xml);
    reader.trim_text(true);
    let mut buf = Vec::new();

    let mut defaults: HashMap<String, String> = HashMap::new();

    let mut in_sensor = false;
    let mut in_film = false;
    let mut in_transform = false;
    let mut in_sensor_transform = false;
    let mut in_shape_transform = false;
    let mut in_emitter_transform = false;
    let mut in_texture_transform = false;
    let mut in_integrator = false;
    let mut in_volume_transform = false;
    let mut bsdf_stack: Vec<BsdfState> = Vec::new();
    let mut in_shape = false;
    let mut in_emitter = false;
    let mut in_volume = false;
    let mut in_medium = false;
    let mut in_shape_medium = false;
    let mut in_medium_volume = false;

    let mut fov_deg: Option<Float> = None;
    let mut origin: Option<Vector3f> = None;
    let mut target: Option<Vector3f> = None;
    let mut up: Option<Vector3f> = None;
    let mut near_clip: Option<Float> = None;
    let mut far_clip: Option<Float> = None;
    let mut fov_axis: Option<String> = None;
    let mut width: Option<usize> = None;
    let mut height: Option<usize> = None;
    let mut max_depth: Option<u32> = None;
    let mut spp: Option<u32> = None;
    let mut integrator_type: Option<String> = None;
    let mut raymarch_step_size: Option<Float> = None;

    let mut bsdfs: HashMap<String, Arc<dyn BSDF>> = HashMap::new();
    let mut raw_data: HashMap<String, RawDataView> = HashMap::new();
    let mut current_shape_filename: Option<String> = None;
    let mut current_shape_bsdf_ref: Option<String> = None;
    let mut current_shape_bsdf_inline: Option<Arc<dyn BSDF>> = None;
    let mut current_shape_type: Option<String> = None;
    let mut current_emitter_radiance: Option<RGBSpectrum> = None;
    let mut current_emitter_type: Option<String> = None;
    let mut current_emitter_direction: Option<Vector3f> = None;
    let mut current_emitter_irradiance: Option<RGBSpectrum> = None;
    let mut current_envmap_filename: Option<String> = None;
    let mut current_envmap_scale: Option<Float> = None;
    let mut current_emitter_transform = Matrix4f::identity();
    let mut current_sensor_transform = Matrix4f::identity();
    let mut current_shape_emissive: bool = false;
    let mut current_shape_id: Option<String> = None;
    let mut current_shape_transform = Matrix4f::identity();
    let mut current_shape_face_normals = false;
    let mut current_volume: Option<VolumeState> = None;
    let mut current_medium: Option<MediumState> = None;
    let mut current_shape_medium: Option<MediumState> = None;
    let mut current_medium_volume: Option<VolumeState> = None;
    let mut current_medium_volume_name: Option<String> = None;
    let mut pending_media: Vec<MediumState> = Vec::new();

    let mut scene = Scene::new();
    scene.set_base_dir(base_dir.to_path_buf());

    macro_rules! handle_start {
        ($e:expr, $is_empty:expr) => {{
            let e = $e;
                match e.name().as_ref() {
                    b"default" => {
                        let mut key: Option<String> = None;
                        let mut value: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => key = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                _ => {}
                            }
                        }
                        if let (Some(k), Some(v)) = (key, value) {
                            defaults.insert(k, v);
                        }
                    }
                    b"sensor" => {
                        let mut sensor_type: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"type" {
                                sensor_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                            }
                        }
                        in_sensor = matches!(sensor_type.as_deref(), Some("perspective"));
                    }
                    b"integrator" => {
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"type" {
                                let integrator_value = resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults);
                                let integrator_value = integrator_value.trim().to_lowercase();
                                integrator_type = Some(integrator_value.clone());
                                if integrator_value != "path" && integrator_value != "raymarching" {
                                    return Err(SceneLoadError::Parse(format!("unsupported integrator: {}", integrator_value)));
                                }
                            }
                        }
                        in_integrator = true;
                    }
                    b"film" => {
                        in_film = true;
                    }
                    b"transform" => {
                        if let Some(current_bsdf) = bsdf_stack.last() {
                            if current_bsdf.texture_active {
                                for attr in e.attributes().flatten() {
                                    if attr.key.as_ref() == b"name" {
                                        let name = attr.unescape_value().unwrap_or_default();
                                        in_texture_transform = name.as_ref() == "to_uv";
                                    }
                                }
                            }
                        }
                        if in_sensor {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_transform = name.as_ref() == "to_world";
                                    in_sensor_transform = in_transform;
                                }
                            }
                        } else if in_shape {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_shape_transform = name.as_ref() == "to_world";
                                }
                            }
                        } else if in_emitter {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_emitter_transform = name.as_ref() == "to_world";
                                }
                            }
                        } else if in_volume {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_volume_transform = name.as_ref() == "to_world";
                                }
                            }
                        }
                    }
                    b"lookat" => {
                        if in_sensor && in_transform {
                            let mut o: Option<Vector3f> = None;
                            let mut t: Option<Vector3f> = None;
                            let mut u: Option<Vector3f> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"origin" => o = Some(parse_vec3(&attr.unescape_value().unwrap_or_default())?),
                                    b"target" => t = Some(parse_vec3(&attr.unescape_value().unwrap_or_default())?),
                                    b"up" => u = Some(parse_vec3(&attr.unescape_value().unwrap_or_default())?),
                                    _ => {}
                                }
                            }
                            origin = o;
                            target = t;
                            up = u;
                        }
                    }
                    b"translate" => {
                        if (in_shape && in_shape_transform)
                            || (in_emitter && in_emitter_transform)
                            || (in_sensor && in_sensor_transform)
                            || (in_volume && in_volume_transform)
                        {
                            let mut x: Float = 0.0;
                            let mut y: Float = 0.0;
                            let mut z: Float = 0.0;
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => x = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"y" => y = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"z" => z = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    _ => {}
                                }
                            }
                            if let Some(value_attr) = value_attr {
                                let v = parse_vec3(&value_attr)?;
                                x = v.x;
                                y = v.y;
                                z = v.z;
                            }
                            let mut t = Matrix4f::identity();
                            t[(0, 3)] = x;
                            t[(1, 3)] = y;
                            t[(2, 3)] = z;
                            if in_shape && in_shape_transform {
                                current_shape_transform = t * current_shape_transform;
                            }
                            if in_emitter && in_emitter_transform {
                                current_emitter_transform = t * current_emitter_transform;
                            }
                            if in_sensor && in_sensor_transform {
                                current_sensor_transform = t * current_sensor_transform;
                            }
                            if in_volume && in_volume_transform {
                                if let Some(ref mut volume) = current_volume {
                                    volume.to_world = t * volume.to_world;
                                }
                            }
                        }
                        if in_texture_transform {
                            let mut x: Float = 0.0;
                            let mut y: Float = 0.0;
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => x = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"y" => y = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    _ => {}
                                }
                            }
                            if let Some(value_attr) = value_attr {
                                let v = parse_vec3(&value_attr)?;
                                x = v.x;
                                y = v.y;
                            }
                            let mut t = Matrix3f::identity();
                            t[(0, 2)] = x;
                            t[(1, 2)] = y;
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if current_bsdf.texture_active {
                                    if let Some(ref mut tex) = current_bsdf.texture {
                                        tex.to_uv = t * tex.to_uv;
                                    }
                                }
                            }
                        }
                    }
                    b"scale" => {
                        if (in_shape && in_shape_transform)
                            || (in_emitter && in_emitter_transform)
                            || (in_sensor && in_sensor_transform)
                            || (in_volume && in_volume_transform)
                        {
                            let mut sx: Option<Float> = None;
                            let mut sy: Option<Float> = None;
                            let mut sz: Option<Float> = None;
                            let mut uniform: Option<Float> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => sx = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    b"y" => sy = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    b"z" => sz = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    b"value" => uniform = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    _ => {}
                                }
                            }
                            let s = if let Some(u) = uniform {
                                Vector3f::new(u, u, u)
                            } else {
                                Vector3f::new(sx.unwrap_or(1.0), sy.unwrap_or(1.0), sz.unwrap_or(1.0))
                            };

                            let mut t = Matrix4f::identity();
                            t[(0, 0)] = s.x;
                            t[(1, 1)] = s.y;
                            t[(2, 2)] = s.z;
                            if in_shape && in_shape_transform {
                                current_shape_transform = t * current_shape_transform;
                            }
                            if in_emitter && in_emitter_transform {
                                current_emitter_transform = t * current_emitter_transform;
                            }
                            if in_sensor && in_sensor_transform {
                                current_sensor_transform = t * current_sensor_transform;
                            }
                            if in_volume && in_volume_transform {
                                if let Some(ref mut volume) = current_volume {
                                    volume.to_world = t * volume.to_world;
                                }
                            }
                        }
                        if in_texture_transform {
                            let mut sx: Option<Float> = None;
                            let mut sy: Option<Float> = None;
                            let mut uniform: Option<Float> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => sx = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    b"y" => sy = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    b"value" => uniform = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    _ => {}
                                }
                            }
                            let s = if let Some(u) = uniform {
                                Vector3f::new(u, u, u)
                            } else {
                                Vector3f::new(sx.unwrap_or(1.0), sy.unwrap_or(1.0), 1.0)
                            };
                            let mut t = Matrix3f::identity();
                            t[(0, 0)] = s.x;
                            t[(1, 1)] = s.y;
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if current_bsdf.texture_active {
                                    if let Some(ref mut tex) = current_bsdf.texture {
                                        tex.to_uv = t * tex.to_uv;
                                    }
                                }
                            }
                        }
                    }
                    b"rotate" => {
                        if (in_shape && in_shape_transform)
                            || (in_emitter && in_emitter_transform)
                            || (in_sensor && in_sensor_transform)
                            || (in_volume && in_volume_transform)
                        {
                            let mut axis = Vector3f::new(0.0, 0.0, 0.0);
                            let mut angle: Option<Float> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => axis.x = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"y" => axis.y = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"z" => axis.z = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"angle" => angle = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    _ => {}
                                }
                            }
                            if let Some(angle) = angle {
                                if axis.norm() > 0.0 {
                                    let angle_rad = angle * std::f32::consts::PI / 180.0;
                                    let unit = na::Unit::new_normalize(axis);
                                    let rot = na::Rotation3::from_axis_angle(&unit, angle_rad);
                                    let t = rot.to_homogeneous();
                                    if in_shape && in_shape_transform {
                                        current_shape_transform = t * current_shape_transform;
                                    }
                                    if in_emitter && in_emitter_transform {
                                        current_emitter_transform = t * current_emitter_transform;
                                    }
                                    if in_sensor && in_sensor_transform {
                                        current_sensor_transform = t * current_sensor_transform;
                                    }
                                    if in_volume && in_volume_transform {
                                        if let Some(ref mut volume) = current_volume {
                                            volume.to_world = t * volume.to_world;
                                        }
                                    }
                                }
                            }
                        }
                        if in_texture_transform {
                            let mut axis = Vector3f::new(0.0, 0.0, 0.0);
                            let mut angle: Option<Float> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"x" => axis.x = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"y" => axis.y = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"z" => axis.z = parse_float(&attr.unescape_value().unwrap_or_default())?,
                                    b"angle" => angle = Some(parse_float(&attr.unescape_value().unwrap_or_default())?),
                                    _ => {}
                                }
                            }
                            if let Some(angle) = angle {
                                let mut angle_rad = angle * std::f32::consts::PI / 180.0;
                                if axis.z < 0.0 {
                                    angle_rad = -angle_rad;
                                }
                                if axis.norm() > 0.0 || axis.z == 0.0 {
                                    let (c, s) = (angle_rad.cos(), angle_rad.sin());
                                    let mut t = Matrix3f::identity();
                                    t[(0, 0)] = c;
                                    t[(0, 1)] = -s;
                                    t[(1, 0)] = s;
                                    t[(1, 1)] = c;
                                    if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                        if current_bsdf.texture_active {
                                            if let Some(ref mut tex) = current_bsdf.texture {
                                                tex.to_uv = t * tex.to_uv;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    b"matrix" => {
                        if (in_shape && in_shape_transform)
                            || (in_emitter && in_emitter_transform)
                            || (in_sensor && in_sensor_transform)
                            || (in_volume && in_volume_transform)
                        {
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"value" {
                                    value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                                }
                            }
                            if let Some(value_attr) = value_attr {
                                let t = parse_matrix4(&value_attr)?;
                                if in_shape && in_shape_transform {
                                    current_shape_transform = t * current_shape_transform;
                                }
                                if in_emitter && in_emitter_transform {
                                    current_emitter_transform = t * current_emitter_transform;
                                }
                                if in_sensor && in_sensor_transform {
                                    current_sensor_transform = t * current_sensor_transform;
                                }
                                if in_volume && in_volume_transform {
                                    if let Some(ref mut volume) = current_volume {
                                        volume.to_world = t * volume.to_world;
                                    }
                                }
                            }
                        }
                        if in_texture_transform {
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"value" {
                                    value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                                }
                            }
                            if let Some(value_attr) = value_attr {
                                let t = parse_matrix3_or_4(&value_attr)?;
                                if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                    if current_bsdf.texture_active {
                                        if let Some(ref mut tex) = current_bsdf.texture {
                                            tex.to_uv = t * tex.to_uv;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    b"float" => {
                        let mut name_attr: Option<String> = None;
                        let mut value_attr: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                _ => {}
                            }
                        }
                        if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                            if in_integrator && name_attr == "step_size" {
                                raymarch_step_size = Some(parse_float(&value_attr)?);
                            }
                            if in_sensor {
                                if name_attr == "fov" {
                                    fov_deg = Some(parse_float(&value_attr)?);
                                }
                                if name_attr == "near_clip" {
                                    near_clip = Some(parse_float(&value_attr)?);
                                }
                                if name_attr == "far_clip" {
                                    far_clip = Some(parse_float(&value_attr)?);
                                }
                            }
                            if in_emitter {
                                if name_attr == "radiance" {
                                    current_emitter_radiance = Some(parse_vec3_spectrum(&value_attr)?);
                                }
                                if current_emitter_type.as_deref() == Some("directional") && name_attr == "irradiance" {
                                    let v = parse_float(&value_attr)?;
                                    current_emitter_irradiance = Some(RGBSpectrum::new(v, v, v));
                                }
                                if current_emitter_type.as_deref() == Some("envmap") && name_attr == "scale" {
                                    current_envmap_scale = Some(parse_float(&value_attr)?);
                                }
                            }
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if name_attr == "alpha" {
                                    current_bsdf.alpha = Some(parse_float(&value_attr)?);
                                } else if name_attr == "alpha_u" {
                                    current_bsdf.alpha_u = Some(parse_float(&value_attr)?);
                                } else if name_attr == "alpha_v" {
                                    current_bsdf.alpha_v = Some(parse_float(&value_attr)?);
                                } else if name_attr == "weight" {
                                    current_bsdf.weight = Some(parse_float(&value_attr)?);
                                } else if name_attr == "int_ior" {
                                    current_bsdf.int_ior = Some(parse_ior(&value_attr)?);
                                } else if name_attr == "ext_ior" {
                                    current_bsdf.ext_ior = Some(parse_ior(&value_attr)?);
                                } else if name_attr == "specular_reflectance" {
                                    let v = parse_float(&value_attr)?;
                                    current_bsdf.specular_reflectance = Some(RGBSpectrum::new(v, v, v));
                                } else if name_attr == "specular_transmittance" {
                                    let v = parse_float(&value_attr)?;
                                    current_bsdf.specular_transmittance = Some(RGBSpectrum::new(v, v, v));
                                } else if name_attr == "eta" {
                                    let v = parse_float(&value_attr)?;
                                    current_bsdf.eta = Some(RGBSpectrum::new(v, v, v));
                                } else if name_attr == "k" {
                                    let v = parse_float(&value_attr)?;
                                    current_bsdf.k = Some(RGBSpectrum::new(v, v, v));
                                } else if name_attr == "reflectance" {
                                    let v = parse_float(&value_attr)?;
                                    current_bsdf.reflectance = Some(RGBSpectrum::new(v, v, v));
                                }
                            }
                            if in_volume && name_attr == "value" {
                                if let Some(ref mut volume) = current_volume {
                                    volume.value_scalar = Some(parse_float(&value_attr)?);
                                }
                            }
                            if in_medium_volume && name_attr == "value" {
                                if let Some(ref mut volume) = current_medium_volume {
                                    volume.value_scalar = Some(parse_float(&value_attr)?);
                                }
                            }
                            if in_shape_medium {
                                if let Some(ref mut medium) = current_shape_medium {
                                    if name_attr == "sigma_t" {
                                        medium.sigma_t_scalar = Some(parse_float(&value_attr)?);
                                    } else if name_attr == "albedo" {
                                        medium.albedo_scalar = Some(parse_float(&value_attr)?);
                                    } else if name_attr == "scale" {
                                        medium.scale = Some(parse_float(&value_attr)?);
                                    }
                                }
                            } else if in_medium {
                                if let Some(ref mut medium) = current_medium {
                                    if name_attr == "sigma_t" {
                                        medium.sigma_t_scalar = Some(parse_float(&value_attr)?);
                                    } else if name_attr == "albedo" {
                                        medium.albedo_scalar = Some(parse_float(&value_attr)?);
                                    } else if name_attr == "scale" {
                                        medium.scale = Some(parse_float(&value_attr)?);
                                    }
                                }
                            }
                        }
                    }
                    b"integer" => {
                        if in_film {
                            let mut name_attr: Option<String> = None;
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    _ => {}
                                }
                            }
                            if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                                let value = parse_usize(&value_attr)?;
                                if name_attr == "width" {
                                    width = Some(value);
                                } else if name_attr == "height" {
                                    height = Some(value);
                                }
                            }
                        }
                        let mut name_attr: Option<String> = None;
                        let mut value_attr: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                _ => {}
                            }
                        }
                        if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                            if name_attr == "max_depth" {
                                max_depth = Some(parse_u32(&value_attr)?);
                            }
                            if name_attr == "sample_count" {
                                spp = Some(parse_u32(&value_attr)?);
                            }
                        }
                    }
                    b"boolean" => {
                        let mut name_attr: Option<String> = None;
                        let mut value_attr: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                _ => {}
                            }
                        }
                        if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if current_bsdf.texture_active && name_attr == "raw" {
                                    if let Some(ref mut tex) = current_bsdf.texture {
                                        tex.raw = parse_bool(&value_attr)?;
                                    }
                                } else if name_attr == "sample_visible" {
                                    current_bsdf.sample_visible = parse_bool(&value_attr)?;
                                }
                            }
                            if in_shape && name_attr == "face_normals" {
                                current_shape_face_normals = parse_bool(&value_attr)?;
                            }
                            if in_volume && name_attr == "use_grid_bbox" {
                                if let Some(ref mut volume) = current_volume {
                                    volume.use_grid_bbox = Some(parse_bool(&value_attr)?);
                                }
                            }
                        }
                    }
                    b"bsdf" => {
                        let mut bsdf_type: Option<String> = None;
                        let mut bsdf_id: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"type" => bsdf_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                b"id" => bsdf_id = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                _ => {}
                            }
                        }
                        if let Some(bsdf_type) = bsdf_type {
                            bsdf_stack.push(BsdfState::new(bsdf_type, bsdf_id));
                        }
                    }
                    b"volume" => {
                        let mut volume_type: Option<String> = None;
                        let mut volume_id: Option<String> = None;
                        let mut volume_name: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"type" => volume_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                b"id" => volume_id = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"name" => volume_name = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                _ => {}
                            }
                        }
                        if let Some(volume_type) = volume_type {
                            let volume_type = volume_type.trim().to_lowercase();
                            if in_medium || in_shape_medium {
                                let inline_type = match volume_type.as_str() {
                                    "const" | "constvolume" => Some("const".to_string()),
                                    "grid" => Some("grid".to_string()),
                                    _ => None,
                                };
                                if let Some(inline_type) = inline_type {
                                    in_medium_volume = true;
                                    current_medium_volume_name = volume_name;
                                    current_medium_volume = Some(VolumeState::new(inline_type, None));
                                }
                            } else if volume_type == "const" || volume_type == "grid" {
                                in_volume = true;
                                current_volume = Some(VolumeState::new(volume_type, volume_id));
                            } else {
                                in_volume = false;
                                current_volume = None;
                            }
                        }
                    }
                    b"medium" => {
                        let mut medium_type: Option<String> = None;
                        let mut medium_id: Option<String> = None;
                        let mut medium_name: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"type" => medium_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                b"id" => medium_id = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"name" => medium_name = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                _ => {}
                            }
                        }
                        if let Some(medium_type) = medium_type {
                            let medium_type = medium_type.trim().to_lowercase();
                            if medium_type == "homogeneous" || medium_type == "heterogeneous" {
                                if in_shape {
                                    in_shape_medium = true;
                                    let mut state = MediumState::new(medium_type, medium_id);
                                    state.name = medium_name;
                                    current_shape_medium = Some(state);
                                } else {
                                    in_medium = true;
                                    let mut state = MediumState::new(medium_type, medium_id);
                                    state.name = medium_name;
                                    current_medium = Some(state);
                                }
                            } else {
                                if in_shape {
                                    in_shape_medium = false;
                                    current_shape_medium = None;
                                } else {
                                    in_medium = false;
                                    current_medium = None;
                                }
                            }
                        }
                    }
                    b"texture" => {
                        if let Some(current_bsdf) = bsdf_stack.last_mut() {
                            let mut tex_type: Option<String> = None;
                            let mut tex_name: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"type" => tex_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    b"name" => tex_name = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    _ => {}
                                }
                            }
                            current_bsdf.texture_active = true;
                            current_bsdf.texture = Some(BsdfTextureState {
                                tex_type,
                                tex_name,
                                tex_filename: None,
                                raw: false,
                                filter_type: None,
                                wrap_mode: None,
                                to_uv: Matrix3f::identity(),
                            });
                        }
                    }
                    b"string" => {
                        let mut name_attr: Option<String> = None;
                        let mut value_attr: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                _ => {}
                            }
                        }
                        if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                            if in_sensor && name_attr == "fov_axis" {
                                fov_axis = Some(value_attr.clone());
                            }
                            if in_emitter && current_emitter_type.as_deref() == Some("envmap") && name_attr == "filename" {
                                current_envmap_filename = Some(value_attr.clone());
                            }
                            if in_shape && name_attr == "filename" {
                                current_shape_filename = Some(value_attr.clone());
                            }
                            if in_volume {
                                if let Some(ref mut volume) = current_volume {
                                    if name_attr == "filename" {
                                        volume.filename = Some(value_attr.clone());
                                    } else if name_attr == "filter_type" {
                                        volume.filter_type = Some(value_attr.clone());
                                    } else if name_attr == "wrap_mode" {
                                        volume.wrap_mode = Some(value_attr.clone());
                                    }
                                }
                            }
                            if in_medium_volume {
                                if let Some(ref mut volume) = current_medium_volume {
                                    if name_attr == "filename" {
                                        volume.filename = Some(value_attr.clone());
                                    } else if name_attr == "filter_type" {
                                        volume.filter_type = Some(value_attr.clone());
                                    } else if name_attr == "wrap_mode" {
                                        volume.wrap_mode = Some(value_attr.clone());
                                    }
                                }
                            }
                            if in_shape_medium {
                                if let Some(ref mut medium) = current_shape_medium {
                                    if name_attr == "phase_function" {
                                        medium.phase_function = Some(value_attr.clone());
                                    }
                                }
                            } else if in_medium {
                                if let Some(ref mut medium) = current_medium {
                                    if name_attr == "phase_function" {
                                        medium.phase_function = Some(value_attr.clone());
                                    }
                                }
                            }
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if current_bsdf.texture_active {
                                    if let Some(ref mut tex) = current_bsdf.texture {
                                        if name_attr == "filename" {
                                            tex.tex_filename = Some(value_attr.clone());
                                        } else if name_attr == "filter_type" {
                                            tex.filter_type = Some(value_attr.clone());
                                        } else if name_attr == "wrap_mode" {
                                            tex.wrap_mode = Some(value_attr.clone());
                                        }
                                    }
                                } else if name_attr == "distribution" {
                                    current_bsdf.distribution = Some(value_attr.clone());
                                } else if name_attr == "int_ior" {
                                    current_bsdf.int_ior = Some(parse_ior(&value_attr)?);
                                } else if name_attr == "ext_ior" {
                                    current_bsdf.ext_ior = Some(parse_ior(&value_attr)?);
                                }
                            }
                        }
                    }
                    b"rgb" => {
                        let mut name_attr: Option<String> = None;
                        let mut value_attr: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.as_ref() {
                                b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                _ => {}
                            }
                        }
                        if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                            if let Some(current_bsdf) = bsdf_stack.last_mut() {
                                if name_attr == "reflectance" {
                                    current_bsdf.reflectance = Some(parse_vec3_spectrum(&value_attr)?);
                                } else if name_attr == "specular_reflectance" {
                                    current_bsdf.specular_reflectance = Some(parse_vec3_spectrum(&value_attr)?);
                                } else if name_attr == "specular_transmittance" {
                                    current_bsdf.specular_transmittance = Some(parse_vec3_spectrum(&value_attr)?);
                                } else if name_attr == "eta" {
                                    current_bsdf.eta = Some(parse_vec3_spectrum(&value_attr)?);
                                } else if name_attr == "k" {
                                    current_bsdf.k = Some(parse_vec3_spectrum(&value_attr)?);
                                }
                            }
                            if in_emitter && name_attr == "radiance" {
                                current_emitter_radiance = Some(parse_vec3_spectrum(&value_attr)?);
                            }
                            if in_emitter && current_emitter_type.as_deref() == Some("directional") && name_attr == "irradiance" {
                                current_emitter_irradiance = Some(parse_vec3_spectrum(&value_attr)?);
                            }
                            if in_volume && name_attr == "value" {
                                if let Some(ref mut volume) = current_volume {
                                    volume.value_rgb = Some(parse_vec3_spectrum(&value_attr)?);
                                }
                            }
                            if in_medium_volume && name_attr == "value" {
                                if let Some(ref mut volume) = current_medium_volume {
                                    volume.value_rgb = Some(parse_vec3_spectrum(&value_attr)?);
                                }
                            }
                            if in_shape_medium {
                                if let Some(ref mut medium) = current_shape_medium {
                                    if name_attr == "sigma_t" {
                                        medium.sigma_t = Some(parse_vec3_spectrum(&value_attr)?);
                                    } else if name_attr == "albedo" {
                                        medium.albedo = Some(parse_vec3_spectrum(&value_attr)?);
                                    }
                                }
                            } else if in_medium {
                                if let Some(ref mut medium) = current_medium {
                                    if name_attr == "sigma_t" {
                                        medium.sigma_t = Some(parse_vec3_spectrum(&value_attr)?);
                                    } else if name_attr == "albedo" {
                                        medium.albedo = Some(parse_vec3_spectrum(&value_attr)?);
                                    }
                                }
                            }
                        }
                    }
                    b"vector" => {
                        if in_emitter {
                            let mut name_attr: Option<String> = None;
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    b"value" => value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    _ => {}
                                }
                            }
                            if let (Some(name_attr), Some(value_attr)) = (name_attr, value_attr) {
                                if current_emitter_type.as_deref() == Some("directional") && name_attr == "direction" {
                                    current_emitter_direction = Some(parse_vec3(&value_attr)?);
                                }
                            }
                        }
                    }
                    b"shape" => {
                        let mut shape_type: Option<String> = None;
                        let mut shape_id: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"type" {
                                shape_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                            } else if attr.key.as_ref() == b"id" {
                                shape_id = Some(attr.unescape_value().unwrap_or_default().to_string());
                            }
                        }
                        if matches!(shape_type.as_deref(), Some("obj") | Some("ply") | Some("rectangle") | Some("cube")) {
                            in_shape = true;
                            current_shape_type = shape_type;
                            current_shape_filename = None;
                            current_shape_bsdf_ref = None;
                            current_shape_bsdf_inline = None;
                            current_shape_emissive = false;
                            current_emitter_radiance = None;
                            current_shape_id = shape_id;
                            current_shape_transform = Matrix4f::identity();
                            current_shape_face_normals = false;
                            current_shape_medium = None;
                            in_shape_medium = false;
                        } else {
                            in_shape = false;
                            current_shape_type = None;
                        }
                    }
                    b"ref" => {
                        if in_shape_medium || in_medium {
                            let mut name_attr: Option<String> = None;
                            let mut id_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"name" => name_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    b"id" => id_attr = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    _ => {}
                                }
                            }
                            if let (Some(name_attr), Some(id_attr)) = (name_attr, id_attr) {
                                let target = if in_shape_medium {
                                    current_shape_medium.as_mut()
                                } else {
                                    current_medium.as_mut()
                                };
                                if let Some(medium) = target {
                                    if name_attr == "sigma_t" {
                                        medium.sigma_t_ref = Some(id_attr);
                                    } else if name_attr == "albedo" {
                                        medium.albedo_ref = Some(id_attr);
                                    }
                                }
                            }
                        } else if in_shape {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"id" {
                                    current_shape_bsdf_ref = Some(attr.unescape_value().unwrap_or_default().to_string());
                                }
                            }
                        }
                    }
                    b"emitter" => {
                        let mut emitter_type: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"type" {
                                emitter_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                            }
                        }
                        current_emitter_type = emitter_type;
                        current_emitter_radiance = None;
                        if in_shape {
                            in_emitter = true;
                            current_shape_emissive = true;
                        } else {
                            in_emitter = true;
                            current_emitter_direction = None;
                            current_emitter_irradiance = None;
                            current_envmap_filename = None;
                            current_envmap_scale = None;
                            current_emitter_transform = Matrix4f::identity();
                        }
                    }
                    _ => {}
                };
        if $is_empty {
            match e.name().as_ref() {
                b"bsdf" => {
                    if let Some(state) = bsdf_stack.pop() {
                        let bsdf_id = state.id.clone();
                        let bsdf = build_bsdf(state, base_dir, &mut raw_data)?;

                        if let Some(parent) = bsdf_stack.last_mut() {
                            parent.children.push(bsdf);
                        } else if in_shape {
                            if let Some(id) = bsdf_id {
                                bsdfs.insert(id, bsdf.clone());
                            }
                            current_shape_bsdf_inline = Some(bsdf);
                        } else if let Some(id) = bsdf_id {
                            bsdfs.insert(id, bsdf);
                        }
                    }
                }
                b"volume" => {
                    if in_medium_volume {
                        if let Some(state) = current_medium_volume.take() {
                            commit_inline_medium_volume(state, &mut current_shape_medium, &mut current_medium, &current_medium_volume_name)?;
                        }
                        current_medium_volume_name = None;
                        in_medium_volume = false;
                    } else {
                        if let Some(state) = current_volume.take() {
                            let (id, volume) = build_volume(state, base_dir, &mut raw_data)?;
                            scene.add_volume(id, volume);
                        }
                        in_volume = false;
                        in_volume_transform = false;
                    }
                }
                b"medium" => {
                    if in_shape_medium {
                        in_shape_medium = false;
                    } else {
                        if let Some(state) = current_medium.take() {
                            pending_media.push(state);
                        }
                        in_medium = false;
                    }
                }
                _ => {}
            }
        }
        Ok::<(), SceneLoadError>(())
        }};
    }

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) => {
                handle_start!(e, false)?;
            }
            Ok(Event::Empty(e)) => {
                handle_start!(e, true)?;
            }
            Ok(Event::End(e)) => {
                match e.name().as_ref() {
                    b"integrator" => {
                        in_integrator = false;
                    }
                    b"sensor" => {
                        if in_sensor {
                            let fov_deg = fov_deg.ok_or(SceneLoadError::MissingField("sensor.fov"))?;
                            let (origin, target, up) = if origin.is_some() && target.is_some() && up.is_some() {
                                (origin.unwrap(), target.unwrap(), up.unwrap())
                            } else {
                                let origin = Vector3f::new(
                                    current_sensor_transform[(0, 3)],
                                    current_sensor_transform[(1, 3)],
                                    current_sensor_transform[(2, 3)],
                                );
                                let forward = Vector3f::new(
                                    current_sensor_transform[(0, 2)],
                                    current_sensor_transform[(1, 2)],
                                    current_sensor_transform[(2, 2)],
                                )
                                .normalize();
                                let up = Vector3f::new(
                                    current_sensor_transform[(0, 1)],
                                    current_sensor_transform[(1, 1)],
                                    current_sensor_transform[(2, 1)],
                                )
                                .normalize();
                                let target = origin + forward;
                                (origin, target, up)
                            };
                            let width = width.ok_or(SceneLoadError::MissingField("film.width"))?;
                            let height = height.ok_or(SceneLoadError::MissingField("film.height"))?;

                            let aspect = width as Float / height as Float;
                            let axis = fov_axis.as_deref().unwrap_or("x");
                            let fov_rad = fov_y_from_axis(fov_deg, axis, width, height)?;
                            let camera = PerspectiveCamera::new(
                                origin,
                                target,
                                up,
                                fov_rad,
                                aspect,
                                width,
                                height,
                                near_clip.unwrap_or(1e-2),
                                far_clip.unwrap_or(1e4),
                            );
                            scene.add_sensor(Box::new(camera));
                        }

                        in_sensor = false;
                        in_film = false;
                        in_transform = false;
                        in_sensor_transform = false;
                        fov_deg = None;
                        origin = None;
                        target = None;
                        up = None;
                        near_clip = None;
                        far_clip = None;
                        fov_axis = None;
                        width = None;
                        height = None;
                        current_sensor_transform = Matrix4f::identity();
                    }
                    b"film" => {
                        in_film = false;
                    }
                    b"transform" => {
                        in_transform = false;
                        in_sensor_transform = false;
                        in_shape_transform = false;
                        in_emitter_transform = false;
                        in_texture_transform = false;
                        in_volume_transform = false;
                    }
                    b"texture" => {
                        if let Some(current_bsdf) = bsdf_stack.last_mut() {
                            current_bsdf.texture_active = false;
                        }
                    }
                    b"bsdf" => {
                        if let Some(state) = bsdf_stack.pop() {
                            let bsdf_id = state.id.clone();
                            let bsdf = build_bsdf(state, base_dir, &mut raw_data)?;

                            if let Some(parent) = bsdf_stack.last_mut() {
                                parent.children.push(bsdf);
                            } else if in_shape {
                                if let Some(id) = bsdf_id {
                                    bsdfs.insert(id, bsdf.clone());
                                }
                                current_shape_bsdf_inline = Some(bsdf);
                            } else if let Some(id) = bsdf_id {
                                bsdfs.insert(id, bsdf);
                            }
                        }
                    }
                    b"volume" => {
                        if in_medium_volume {
                            if let Some(state) = current_medium_volume.take() {
                                commit_inline_medium_volume(state, &mut current_shape_medium, &mut current_medium, &current_medium_volume_name)?;
                            }
                            current_medium_volume_name = None;
                            in_medium_volume = false;
                        } else {
                            if let Some(state) = current_volume.take() {
                            let (id, volume) = build_volume(state, base_dir, &mut raw_data)?;
                            scene.add_volume(id, volume);
                        }
                            in_volume = false;
                            in_volume_transform = false;
                        }
                    }
                    b"medium" => {
                        if in_shape_medium {
                            in_shape_medium = false;
                        } else {
                            if let Some(state) = current_medium.take() {
                                pending_media.push(state);
                            }
                            in_medium = false;
                        }
                    }
                    b"emitter" => {
                        if in_emitter && !in_shape {
                            if current_emitter_type.as_deref() == Some("directional") {
                                let direction = current_emitter_direction
                                    .ok_or(SceneLoadError::MissingField("emitter.direction"))?;
                                let irradiance = current_emitter_irradiance
                                    .ok_or(SceneLoadError::MissingField("emitter.irradiance"))?;
                                let emitter = DirectionalEmitter::new_with(
                                    direction,
                                    irradiance,
                                );
                                scene.add_emitter(Box::new(emitter));
                            } else if current_emitter_type.as_deref() == Some("envmap") {
                                let filename = current_envmap_filename
                                    .ok_or(SceneLoadError::MissingField("emitter.filename"))?;
                                let filename = if Path::new(&filename).is_absolute() {
                                    filename
                                } else {
                                    base_dir.join(filename).to_string_lossy().to_string()
                                };
                                let scale = current_envmap_scale.unwrap_or(1.0);
                                let mut emitter = EnvMap::from_file(&filename, scale)
                                    .map_err(|e| SceneLoadError::Parse(e))?;
                                emitter.set_transform(Transform::new(current_emitter_transform));
                                scene.add_emitter(Box::new(emitter));
                            }
                        }
                        in_emitter = false;
                        current_emitter_type = None;
                        current_emitter_direction = None;
                        current_emitter_irradiance = None;
                        current_envmap_filename = None;
                        current_envmap_scale = None;
                        current_emitter_transform = Matrix4f::identity();
                    }
                    b"shape" => {
                        if in_shape {
                            let material = if let Some(bsdf) = current_shape_bsdf_inline.take() {
                                bsdf
                            } else if let Some(bsdf_id) = current_shape_bsdf_ref.take() {
                                bsdfs.get(&bsdf_id)
                                    .ok_or_else(|| SceneLoadError::Parse(format!("missing bsdf ref: {}", bsdf_id)))?
                                    .clone()
                            } else {
                                return Err(SceneLoadError::MissingField("shape.bsdf_ref"));
                            };

                            let shape: Arc<dyn crate::core::shape::Shape> = match current_shape_type.as_deref() {
                                Some("obj") | Some("ply") => {
                                    let filename = current_shape_filename.take().ok_or(SceneLoadError::MissingField("shape.filename"))?;
                                    let filename = if Path::new(&filename).is_absolute() {
                                        filename
                                    } else {
                                        base_dir.join(filename).to_string_lossy().to_string()
                                    };

                                    let mesh = match current_shape_type.as_deref() {
                                        Some("obj") => TriangleMesh::from_obj(&filename)
                                            .map_err(|e| SceneLoadError::Parse(format!("obj load failed: {}", e)))?,
                                        Some("ply") => TriangleMesh::from_ply(&filename)
                                            .map_err(|e| SceneLoadError::Parse(format!("ply load failed: {}", e)))?,
                                        _ => unreachable!(),
                                    };

                                    let mut mesh = mesh;
                                    mesh.set_face_normals(current_shape_face_normals);
                                    let transform = Transform::new(current_shape_transform);
                                    mesh.apply_transform_matrix(&transform);
                                    Arc::new(mesh)
                                }
                                Some("rectangle") => {
                                    let transform = Transform::new(current_shape_transform);
                                    Arc::new(Rectangle::new(transform))
                                }
                                Some("cube") => {
                                    let transform = Transform::new(current_shape_transform);
                                    Arc::new(Cube::new(transform))
                                }
                                Some(other) => {
                                    return Err(SceneLoadError::Parse(format!("unsupported shape type: {}", other)));
                                }
                                None => {
                                    return Err(SceneLoadError::Parse("missing shape type".to_string()));
                                }
                            };

                            let mut object = SceneObject::new(shape.clone(), material);
                            if let Some(id) = current_shape_id.take() {
                                object = object.with_name(id);
                            }
                            if let Some(state) = current_shape_medium.take() {
                                let is_interior = state.name.as_deref().unwrap_or("interior") == "interior";
                                if is_interior {
                                    let (id, medium) = build_medium(state, &scene)?;
                                    if let Some(id) = id {
                                        scene.add_medium(id, medium.clone());
                                    }
                                    object = object.with_interior_medium(Some(medium));
                                }
                            }
                            if current_shape_emissive {
                                let radiance = current_emitter_radiance.unwrap_or(RGBSpectrum::new(1.0, 1.0, 1.0));
                                object.emission = radiance;
                            }
                            scene.add_object(object);
                        }
                        in_shape = false;
                        current_shape_type = None;
                        current_shape_filename = None;
                        current_shape_bsdf_ref = None;
                        current_shape_bsdf_inline = None;
                        current_shape_emissive = false;
                        current_emitter_radiance = None;
                        current_shape_id = None;
                        current_shape_transform = Matrix4f::identity();
                        current_shape_face_normals = false;
                        current_shape_medium = None;
                        in_shape_medium = false;
                    }
                    _ => {}
                }
            }
            Err(e) => {
                return Err(SceneLoadError::Parse(e.to_string()));
            }
            _ => {}
        }

        buf.clear();
    }

    scene.build_bvh();
    scene.set_raw_data(raw_data);

    if !pending_media.is_empty() {
        for state in pending_media {
            let (id, medium) = build_medium(state, &scene)?;
            if let Some(id) = id {
                scene.add_medium(id, medium);
            }
        }
    }

    let integrator = if let Some(depth) = max_depth {
        let integrator_name = integrator_type.as_deref().unwrap_or("path");
        let spp = spp.unwrap_or(1);
        let integrator: Box<dyn Integrator> = match integrator_name {
            "path" => Box::new(PathIntegrator::new(depth, spp)),
            "raymarching" => Box::new(RaymarchingIntegrator::new(depth, spp, raymarch_step_size)),
            other => {
                return Err(SceneLoadError::Parse(format!("unsupported integrator: {}", other)));
            }
        };
        Some(integrator)
    } else {
        None
    };

    Ok(SceneLoadResult {
        scene,
        integrator,
        integrator_type,
        samples_per_pixel: spp,
        max_depth,
    })
}

fn build_bsdf(
    state: BsdfState,
    base_dir: &Path,
    raw_data: &mut HashMap<String, RawDataView>,
) -> Result<Arc<dyn BSDF>, SceneLoadError> {
    let bsdf_id = state.id.clone();
    match state.bsdf_type.as_str() {
        "diffuse" => {
            let texture = if let Some(tex) = state.texture {
                let tex_type = tex.tex_type.unwrap_or_else(|| "bitmap".to_string()).to_lowercase();
                if matches!(tex_type.as_str(), "image" | "bitmap") {
                    let filename = tex.tex_filename.ok_or(SceneLoadError::MissingField("texture.filename"))?;
                    let filename = if Path::new(&filename).is_absolute() {
                        filename
                    } else {
                        base_dir.join(filename).to_string_lossy().to_string()
                    };
                    let mut image = ImageTexture::from_file_with_srgb(&filename, !tex.raw)
                        .map_err(|e| SceneLoadError::Parse(e))?;
                    let raw_view = bsdf_id.as_ref().map(|id| {
                        let key = format!("{}.reflectance.data", id);
                        (key, image.raw_data_view())
                    });
                    if let Some(filter) = tex.filter_type {
                        let filter = filter.trim().to_lowercase();
                        let mode = match filter.as_str() {
                            "bilinear" => FilterMode::Bilinear,
                            "nearest" => FilterMode::Nearest,
                            other => {
                                return Err(SceneLoadError::Parse(format!("unsupported filter_type: {}", other)));
                            }
                        };
                        image.set_filter_mode(mode);
                    }
                    if let Some(wrap) = tex.wrap_mode {
                        let wrap = wrap.trim().to_lowercase();
                        let mode = match wrap.as_str() {
                            "repeat" => WrapMode::Repeat,
                            "mirror" => WrapMode::Mirror,
                            "clamp" => WrapMode::Clamp,
                            other => {
                                return Err(SceneLoadError::Parse(format!("unsupported wrap_mode: {}", other)));
                            }
                        };
                        image.set_wrap_mode(mode);
                    }
                    image.set_uv_transform(tex.to_uv);
                    if let Some((key, view)) = raw_view {
                        raw_data.insert(key, view);
                    }
                    Arc::new(image) as Arc<dyn crate::core::texture::Texture>
                } else {
                    let refl = state.reflectance.unwrap_or(RGBSpectrum::new(0.5, 0.5, 0.5));
                    Arc::new(ConstantTexture::new(refl)) as Arc<dyn crate::core::texture::Texture>
                }
            } else {
                let refl = state.reflectance.unwrap_or(RGBSpectrum::new(0.5, 0.5, 0.5));
                Arc::new(ConstantTexture::new(refl)) as Arc<dyn crate::core::texture::Texture>
            };
            Ok(Arc::new(LambertianDiffuseBSDF::new(texture)) as Arc<dyn BSDF>)
        }
        "roughconductor" | "conductor" => {
            let dist = state.distribution.as_deref().unwrap_or("beckmann").to_lowercase();
            let m_type = match dist.as_str() {
                "ggx" => crate::materials::microfacet::MicrofacetType::GGX,
                "beckmann" => crate::materials::microfacet::MicrofacetType::Beckmann,
                other => {
                    return Err(SceneLoadError::Parse(format!("unsupported roughconductor distribution: {}", other)));
                }
            };
            let (alpha_u, alpha_v) = if state.alpha_u.is_some() || state.alpha_v.is_some() {
                let au = state.alpha_u.ok_or(SceneLoadError::MissingField("bsdf.alpha_u"))?;
                let av = state.alpha_v.ok_or(SceneLoadError::MissingField("bsdf.alpha_v"))?;
                (au, av)
            } else {
                let a = if state.bsdf_type == "conductor" { 0.0 } else { state.alpha.unwrap_or(0.1) };
                (a, a)
            };
            let spec = state.specular_reflectance.unwrap_or(RGBSpectrum::new(1.0, 1.0, 1.0));
            let eta = state.eta.unwrap_or(RGBSpectrum::new(0.0, 0.0, 0.0));
            let k = state.k.unwrap_or(RGBSpectrum::new(1.0, 1.0, 1.0));
            Ok(Arc::new(RoughConductorBSDF::new(
                m_type,
                alpha_u,
                alpha_v,
                state.sample_visible,
                eta,
                k,
                spec,
            )) as Arc<dyn BSDF>)
        }
        "roughdielectric" | "dielectric" => {
            let dist = state.distribution.as_deref().unwrap_or("beckmann").to_lowercase();
            let m_type = match dist.as_str() {
                "ggx" => crate::materials::microfacet::MicrofacetType::GGX,
                "beckmann" => crate::materials::microfacet::MicrofacetType::Beckmann,
                other => {
                    return Err(SceneLoadError::Parse(format!("unsupported roughdielectric distribution: {}", other)));
                }
            };
            let (alpha_u, alpha_v) = if state.alpha_u.is_some() || state.alpha_v.is_some() {
                let au = state.alpha_u.ok_or(SceneLoadError::MissingField("bsdf.alpha_u"))?;
                let av = state.alpha_v.ok_or(SceneLoadError::MissingField("bsdf.alpha_v"))?;
                (au, av)
            } else {
                let a = if state.bsdf_type == "dielectric" { 0.0 } else { state.alpha.unwrap_or(0.1) };
                (a, a)
            };
            let int_ior = state.int_ior.unwrap_or(1.5046);
            let ext_ior = state.ext_ior.unwrap_or(1.000277);
            let spec_reflect = state.specular_reflectance.unwrap_or(RGBSpectrum::new(1.0, 1.0, 1.0));
            let spec_trans = state.specular_transmittance.unwrap_or(RGBSpectrum::new(1.0, 1.0, 1.0));
            Ok(Arc::new(RoughDielectricBSDF::new(
                m_type,
                alpha_u,
                alpha_v,
                state.sample_visible,
                int_ior,
                ext_ior,
                spec_reflect,
                spec_trans,
            )) as Arc<dyn BSDF>)
        }
        "blendbsdf" => {
            if state.children.len() != 2 {
                return Err(SceneLoadError::Parse("blendbsdf expects exactly two bsdf children".to_string()));
            }
            let weight = state.weight.unwrap_or(0.5);
            Ok(Arc::new(BlendBSDF::new(state.children[0].clone(), state.children[1].clone(), weight)) as Arc<dyn BSDF>)
        }
        "twosided" => {
            if state.children.len() != 1 {
                return Err(SceneLoadError::Parse("twosided expects exactly one bsdf child".to_string()));
            }
            Ok(state.children[0].clone())
        }
        "null" => {
            Ok(Arc::new(NullBSDF::new()) as Arc<dyn BSDF>)
        }
        other => Err(SceneLoadError::Parse(format!("unsupported bsdf type: {}", other))),
    }
}

fn build_volume(
    state: VolumeState,
    base_dir: &Path,
    raw_data: &mut HashMap<String, RawDataView>,
) -> Result<(String, Arc<dyn Volume>), SceneLoadError> {
    let id = state.id.ok_or(SceneLoadError::MissingField("volume.id"))?;
    match state.volume_type.as_str() {
        "const" => {
            let value = if let Some(rgb) = state.value_rgb {
                Vector3f::new(rgb[0], rgb[1], rgb[2])
            } else if let Some(v) = state.value_scalar {
                Vector3f::new(v, v, v)
            } else {
                return Err(SceneLoadError::MissingField("volume.value"));
            };
            let volume = if state.value_scalar.is_some() && state.value_rgb.is_none() {
                ConstantVolume::new_scalar(value.x)
            } else {
                ConstantVolume::new_rgb(value)
            };
            Ok((id, Arc::new(volume)))
        }
        "grid" => {
            let filename = state.filename.ok_or(SceneLoadError::MissingField("volume.filename"))?;
            let filename = if Path::new(&filename).is_absolute() {
                filename
            } else {
                base_dir.join(filename).to_string_lossy().to_string()
            };
            let mut volume = GridVolume::from_file(&filename).map_err(SceneLoadError::Parse)?;
            volume.set_transform(Transform::new(state.to_world));
            if let Some(filter) = state.filter_type {
                let filter = filter.trim().to_lowercase();
                let mode = match filter.as_str() {
                    "nearest" => VolumeFilterMode::Nearest,
                    "trilinear" | "linear" => VolumeFilterMode::Trilinear,
                    other => {
                        return Err(SceneLoadError::Parse(format!("unsupported volume filter_type: {}", other)));
                    }
                };
                volume.set_filter_mode(mode);
            }
            if let Some(wrap) = state.wrap_mode {
                let wrap = wrap.trim().to_lowercase();
                let mode = match wrap.as_str() {
                    "repeat" => VolumeWrapMode::Repeat,
                    "mirror" => VolumeWrapMode::Mirror,
                    "clamp" => VolumeWrapMode::Clamp,
                    other => {
                        return Err(SceneLoadError::Parse(format!("unsupported volume wrap_mode: {}", other)));
                    }
                };
                volume.set_wrap_mode(mode);
            }
            if let Some(use_grid_bbox) = state.use_grid_bbox {
                volume.set_use_grid_bbox(use_grid_bbox);
            }
            let key = format!("{}.data", id);
            raw_data.insert(key, volume.raw_data_view());
            Ok((id, Arc::new(volume)))
        }
        other => Err(SceneLoadError::Parse(format!("unsupported volume type: {}", other))),
    }
}

fn build_medium(state: MediumState, scene: &Scene) -> Result<(Option<String>, Arc<dyn Medium>), SceneLoadError> {
    if let Some(phase) = state.phase_function.as_deref() {
        let phase = phase.trim().to_lowercase();
        if phase != "isotropic" {
            return Err(SceneLoadError::Parse(format!("unsupported phase_function: {}", phase)));
        }
    }

    let scale = state.scale.unwrap_or(1.0);
    let medium = match state.medium_type.as_str() {
        "homogeneous" => {
            if state.sigma_t_ref.is_some() {
                return Err(SceneLoadError::Parse("homogeneous medium does not accept sigma_t volume refs".to_string()));
            }
            let sigma_t = resolve_spectrum(state.sigma_t, state.sigma_t_scalar, "medium.sigma_t")?;
            let albedo = if state.albedo_ref.is_some() {
                resolve_spectrum(state.albedo, state.albedo_scalar, "medium.albedo")
                    .unwrap_or_else(|_| RGBSpectrum::new(1.0, 1.0, 1.0))
            } else {
                resolve_spectrum(state.albedo, state.albedo_scalar, "medium.albedo")?
            };
            let mut medium = HomogeneousMedium::new(sigma_t, albedo).with_scale(scale);
            if let Some(albedo_id) = state.albedo_ref {
                let volume = scene
                    .volume(&albedo_id)
                    .ok_or_else(|| SceneLoadError::Parse(format!("missing albedo volume ref: {}", albedo_id)))?;
                medium = medium.with_albedo_volume(volume);
            }
            Arc::new(medium) as Arc<dyn Medium>
        }
        "heterogeneous" => {
            let sigma_t_volume = if let Some(id) = state.sigma_t_ref {
                scene
                    .volume(&id)
                    .ok_or_else(|| SceneLoadError::Parse(format!("missing sigma_t volume ref: {}", id)))?
            } else if state.sigma_t.is_some() || state.sigma_t_scalar.is_some() {
                let sigma_t = resolve_spectrum(state.sigma_t, state.sigma_t_scalar, "medium.sigma_t")?;
                let value = Vector3f::new(sigma_t[0], sigma_t[1], sigma_t[2]);
                Arc::new(ConstantVolume::new_rgb(value)) as Arc<dyn Volume>
            } else {
                return Err(SceneLoadError::MissingField("medium.sigma_t"));
            };

            let albedo_volume = if let Some(id) = state.albedo_ref {
                scene
                    .volume(&id)
                    .ok_or_else(|| SceneLoadError::Parse(format!("missing albedo volume ref: {}", id)))?
            } else if state.albedo.is_some() || state.albedo_scalar.is_some() {
                let albedo = resolve_spectrum(state.albedo, state.albedo_scalar, "medium.albedo")?;
                let value = Vector3f::new(albedo[0], albedo[1], albedo[2]);
                Arc::new(ConstantVolume::new_rgb(value)) as Arc<dyn Volume>
            } else {
                return Err(SceneLoadError::MissingField("medium.albedo"));
            };

            Arc::new(HeterogeneousMedium::new(sigma_t_volume, albedo_volume).with_scale(scale)) as Arc<dyn Medium>
        }
        other => {
            return Err(SceneLoadError::Parse(format!("unsupported medium type: {}", other)));
        }
    };

    Ok((state.id, medium))
}

fn commit_inline_medium_volume(
    state: VolumeState,
    current_shape_medium: &mut Option<MediumState>,
    current_medium: &mut Option<MediumState>,
    volume_name: &Option<String>,
) -> Result<(), SceneLoadError> {
    let target = if let Some(medium) = current_shape_medium.as_mut() {
        medium
    } else if let Some(medium) = current_medium.as_mut() {
        medium
    } else {
        return Err(SceneLoadError::Parse("inline volume missing parent medium".to_string()));
    };

    let name = volume_name
        .as_deref()
        .ok_or(SceneLoadError::MissingField("volume.name"))?;

    if state.volume_type != "const" {
        return Err(SceneLoadError::Parse(format!("unsupported inline volume type: {}", state.volume_type)));
    }

    let (rgb, scalar) = (state.value_rgb, state.value_scalar);
    match name {
        "sigma_t" => {
            if let Some(rgb) = rgb {
                target.sigma_t = Some(rgb);
            } else if let Some(v) = scalar {
                target.sigma_t_scalar = Some(v);
            } else {
                return Err(SceneLoadError::MissingField("volume.value"));
            }
        }
        "albedo" => {
            if let Some(rgb) = rgb {
                target.albedo = Some(rgb);
            } else if let Some(v) = scalar {
                target.albedo_scalar = Some(v);
            } else {
                return Err(SceneLoadError::MissingField("volume.value"));
            }
        }
        other => {
            return Err(SceneLoadError::Parse(format!("unsupported inline volume name: {}", other)));
        }
    }

    Ok(())
}

fn resolve_spectrum(
    rgb: Option<RGBSpectrum>,
    scalar: Option<Float>,
    field: &'static str,
) -> Result<RGBSpectrum, SceneLoadError> {
    if let Some(rgb) = rgb {
        Ok(rgb)
    } else if let Some(v) = scalar {
        Ok(RGBSpectrum::new(v, v, v))
    } else {
        Err(SceneLoadError::MissingField(field))
    }
}

fn resolve_value(raw: &str, defaults: &HashMap<String, String>) -> String {
    let mut out = raw.to_string();
    for (k, v) in defaults {
        out = out.replace(&format!("${}", k), v);
    }
    out
}

fn parse_float(value: &str) -> Result<Float, SceneLoadError> {
    value.parse::<Float>().map_err(|_| SceneLoadError::Parse(format!("invalid float: {}", value)))
}

fn parse_ior(value: &str) -> Result<Float, SceneLoadError> {
    if let Ok(v) = value.parse::<Float>() {
        return Ok(v);
    }
    let name = value.trim().to_lowercase();
    let ior = match name.as_str() {
        "vacuum" => 1.0,
        "air" => 1.000277,
        "water" => 1.3330,
        "bk7" => 1.5046,
        "diamond" => 2.419,
        _ => {
            return Err(SceneLoadError::Parse(format!("invalid ior: {}", value)));
        }
    };
    Ok(ior)
}

fn parse_bool(value: &str) -> Result<bool, SceneLoadError> {
    let v = value.trim().to_lowercase();
    match v.as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(SceneLoadError::Parse(format!("invalid bool: {}", value))),
    }
}

fn parse_u32(value: &str) -> Result<u32, SceneLoadError> {
    value.parse::<u32>().map_err(|_| SceneLoadError::Parse(format!("invalid integer: {}", value)))
}

fn parse_usize(value: &str) -> Result<usize, SceneLoadError> {
    value.parse::<usize>().map_err(|_| SceneLoadError::Parse(format!("invalid integer: {}", value)))
}

fn parse_vec3(value: &str) -> Result<Vector3f, SceneLoadError> {
    let parts: Vec<&str> = value
        .split(|c| c == ',' || c == ' ' || c == '\t')
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();

    match parts.len() {
        1 => {
            let v = parse_float(parts[0])?;
            Ok(Vector3f::new(v, v, v))
        }
        3 | 4 => {
            let x = parse_float(parts[0])?;
            let y = parse_float(parts[1])?;
            let z = parse_float(parts[2])?;
            Ok(Vector3f::new(x, y, z))
        }
        _ => Err(SceneLoadError::Parse("invalid vec3".to_string())),
    }
}

fn parse_matrix4(value: &str) -> Result<Matrix4f, SceneLoadError> {
    let parts: Vec<&str> = value
        .split(|c| c == ',' || c == ' ' || c == '\t')
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();
    if parts.len() != 16 {
        return Err(SceneLoadError::Parse("invalid matrix4".to_string()));
    }
    let mut m = Matrix4f::identity();
    for r in 0..4 {
        for c in 0..4 {
            let idx = r * 4 + c;
            m[(r, c)] = parse_float(parts[idx])?;
        }
    }
    Ok(m)
}

fn parse_matrix3_or_4(value: &str) -> Result<Matrix3f, SceneLoadError> {
    let parts: Vec<&str> = value
        .split(|c| c == ',' || c == ' ' || c == '\t')
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();
    match parts.len() {
        9 => {
            let mut m = Matrix3f::identity();
            for r in 0..3 {
                for c in 0..3 {
                    let idx = r * 3 + c;
                    m[(r, c)] = parse_float(parts[idx])?;
                }
            }
            Ok(m)
        }
        16 => {
            let m4 = parse_matrix4(value)?;
            let mut m = Matrix3f::identity();
            m[(0, 0)] = m4[(0, 0)];
            m[(0, 1)] = m4[(0, 1)];
            m[(0, 2)] = m4[(0, 3)];
            m[(1, 0)] = m4[(1, 0)];
            m[(1, 1)] = m4[(1, 1)];
            m[(1, 2)] = m4[(1, 3)];
            m[(2, 0)] = m4[(3, 0)];
            m[(2, 1)] = m4[(3, 1)];
            m[(2, 2)] = m4[(3, 3)];
            Ok(m)
        }
        _ => Err(SceneLoadError::Parse("invalid matrix3 or matrix4".to_string())),
    }
}

fn parse_vec3_spectrum(value: &str) -> Result<RGBSpectrum, SceneLoadError> {
    let v = parse_vec3(value)?;
    Ok(RGBSpectrum::new(v.x, v.y, v.z))
}

fn fov_y_from_axis(fov_deg: Float, axis: &str, width: usize, height: usize) -> Result<Float, SceneLoadError> {
    let fov_rad = fov_deg * std::f32::consts::PI / 180.0;
    let aspect = width as Float / height as Float;
    let tan_half = (0.5 * fov_rad).tan();
    let axis = axis.trim().to_lowercase();
    let tan_half_y = match axis.as_str() {
        "x" => tan_half / aspect,
        "y" => tan_half,
        "diagonal" => tan_half / (aspect * aspect + 1.0).sqrt(),
        "smaller" => {
            if width < height {
                tan_half / aspect
            } else {
                tan_half
            }
        }
        "larger" => {
            if width < height {
                tan_half
            } else {
                tan_half / aspect
            }
        }
        _ => {
            return Err(SceneLoadError::Parse(format!("invalid fov_axis: {}", axis)));
        }
    };
    Ok(2.0 * tan_half_y.atan())
}
