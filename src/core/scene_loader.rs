// Copyright @yucwang 2026

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use quick_xml::events::Event;
use quick_xml::Reader;

use crate::core::scene::Scene;
use crate::math::constants::{Float, Matrix4f, Vector3f};
use crate::math::transform::Transform;
use crate::sensors::perspective::PerspectiveCamera;
use crate::materials::lambertian_diffuse::LambertianDiffuseBSDF;
use crate::math::spectrum::RGBSpectrum;
use crate::shapes::rectangle::Rectangle;
use crate::shapes::triangle_mesh::TriangleMesh;
use crate::textures::constant::ConstantTexture;
use crate::textures::image::ImageTexture;
use crate::emitters::directional::DirectionalEmitter;
use crate::emitters::envmap::EnvMap;
use crate::core::scene::SceneObject;
use crate::core::bsdf::BSDF;
use crate::core::integrator::Integrator;
use crate::integrators::path::PathIntegrator;
use std::sync::Arc;
use nalgebra as na;

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
    let mut in_shape_transform = false;
    let mut in_bsdf = false;
    let mut in_shape = false;
    let mut in_emitter = false;

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

    let mut bsdfs: HashMap<String, Arc<dyn BSDF>> = HashMap::new();
    let mut current_bsdf_id: Option<String> = None;
    let mut current_bsdf_reflectance: Option<RGBSpectrum> = None;

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
    let mut current_shape_emissive: bool = false;
    let mut current_shape_id: Option<String> = None;
    let mut current_shape_translate = Vector3f::new(0.0, 0.0, 0.0);
    let mut current_shape_scale = Vector3f::new(1.0, 1.0, 1.0);
    let mut current_shape_transform = Matrix4f::identity();

    let mut in_texture = false;
    let mut current_texture_name: Option<String> = None;
    let mut current_texture_type: Option<String> = None;
    let mut current_texture_filename: Option<String> = None;

    let mut scene = Scene::new();
    scene.set_base_dir(base_dir.to_path_buf());

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
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
                                let integrator_type = resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults);
                                if integrator_type != "path" {
                                    return Err(SceneLoadError::Parse(format!("unsupported integrator: {}", integrator_type)));
                                }
                            }
                        }
                    }
                    b"film" => {
                        in_film = true;
                    }
                    b"transform" => {
                        if in_sensor {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_transform = name.as_ref() == "to_world";
                                }
                            }
                        } else if in_shape {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"name" {
                                    let name = attr.unescape_value().unwrap_or_default();
                                    in_shape_transform = name.as_ref() == "to_world";
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
                        if in_shape && in_shape_transform {
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
                            current_shape_translate += Vector3f::new(x, y, z);
                            let mut t = Matrix4f::identity();
                            t[(0, 3)] = x;
                            t[(1, 3)] = y;
                            t[(2, 3)] = z;
                            current_shape_transform = t * current_shape_transform;
                        }
                    }
                    b"scale" => {
                        if in_shape && in_shape_transform {
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
                            current_shape_scale = current_shape_scale.component_mul(&s);
                            let mut t = Matrix4f::identity();
                            t[(0, 0)] = s.x;
                            t[(1, 1)] = s.y;
                            t[(2, 2)] = s.z;
                            current_shape_transform = t * current_shape_transform;
                        }
                    }
                    b"rotate" => {
                        if in_shape && in_shape_transform {
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
                                    current_shape_transform = t * current_shape_transform;
                                }
                            }
                        }
                    }
                    b"matrix" => {
                        if in_shape && in_shape_transform {
                            let mut value_attr: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"value" {
                                    value_attr = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults));
                                }
                            }
                            if let Some(value_attr) = value_attr {
                                let t = parse_matrix4(&value_attr)?;
                                current_shape_transform = t * current_shape_transform;
                            }
                        }
                    }
                    b"float" => {
                        if in_sensor {
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
                        }
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
                        if bsdf_type.as_deref() != Some("diffuse") {
                            in_bsdf = false;
                            current_bsdf_id = None;
                            current_bsdf_reflectance = None;
                            in_texture = false;
                            current_texture_name = None;
                            current_texture_type = None;
                            current_texture_filename = None;
                        } else {
                            in_bsdf = true;
                            current_bsdf_id = bsdf_id;
                            current_bsdf_reflectance = None;
                            in_texture = false;
                            current_texture_name = None;
                            current_texture_type = None;
                            current_texture_filename = None;
                        }
                    }
                    b"texture" => {
                        if in_bsdf {
                            let mut tex_type: Option<String> = None;
                            let mut tex_name: Option<String> = None;
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"type" => tex_type = Some(resolve_value(&attr.unescape_value().unwrap_or_default(), &defaults)),
                                    b"name" => tex_name = Some(attr.unescape_value().unwrap_or_default().to_string()),
                                    _ => {}
                                }
                            }
                            in_texture = true;
                            current_texture_type = tex_type;
                            current_texture_name = tex_name;
                            current_texture_filename = None;
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
                            if in_texture && name_attr == "filename" {
                                current_texture_filename = Some(value_attr.clone());
                            }
                            if in_shape && name_attr == "filename" {
                                current_shape_filename = Some(value_attr);
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
                            let reflectance_ok = if in_texture {
                                current_texture_name.as_deref() == Some("reflectance")
                            } else {
                                true
                            };
                            if in_bsdf && name_attr == "reflectance" && reflectance_ok {
                                current_bsdf_reflectance = Some(parse_vec3_spectrum(&value_attr)?);
                            }
                            if in_emitter && name_attr == "radiance" {
                                current_emitter_radiance = Some(parse_vec3_spectrum(&value_attr)?);
                            }
                            if in_emitter && current_emitter_type.as_deref() == Some("directional") && name_attr == "irradiance" {
                                current_emitter_irradiance = Some(parse_vec3_spectrum(&value_attr)?);
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
                        if matches!(shape_type.as_deref(), Some("obj") | Some("ply") | Some("rectangle")) {
                            in_shape = true;
                            current_shape_type = shape_type;
                            current_shape_filename = None;
                            current_shape_bsdf_ref = None;
                            current_shape_bsdf_inline = None;
                            current_shape_emissive = false;
                            current_emitter_radiance = None;
                            current_shape_id = shape_id;
                            current_shape_translate = Vector3f::new(0.0, 0.0, 0.0);
                            current_shape_scale = Vector3f::new(1.0, 1.0, 1.0);
                            current_shape_transform = Matrix4f::identity();
                        } else {
                            in_shape = false;
                            current_shape_type = None;
                        }
                    }
                    b"ref" => {
                        if in_shape {
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
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(e)) => {
                match e.name().as_ref() {
                    b"sensor" => {
                        if in_sensor {
                            let fov_deg = fov_deg.ok_or(SceneLoadError::MissingField("sensor.fov"))?;
                            let origin = origin.ok_or(SceneLoadError::MissingField("sensor.origin"))?;
                            let target = target.ok_or(SceneLoadError::MissingField("sensor.target"))?;
                            let up = up.ok_or(SceneLoadError::MissingField("sensor.up"))?;
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
                        fov_deg = None;
                        origin = None;
                        target = None;
                        up = None;
                        near_clip = None;
                        far_clip = None;
                        fov_axis = None;
                        width = None;
                        height = None;
                    }
                    b"film" => {
                        in_film = false;
                    }
                    b"transform" => {
                        in_transform = false;
                        in_shape_transform = false;
                    }
                    b"texture" => {
                        in_texture = false;
                    }
                    b"bsdf" => {
                        if in_bsdf {
                            if let Some(id) = current_bsdf_id.take() {
                                let texture = match current_texture_type.as_deref() {
                                    Some("image") => {
                                        let filename = current_texture_filename.clone().ok_or(SceneLoadError::MissingField("texture.filename"))?;
                                        let filename = if Path::new(&filename).is_absolute() {
                                            filename
                                        } else {
                                            base_dir.join(filename).to_string_lossy().to_string()
                                        };
                                        let image = ImageTexture::from_file(&filename)
                                            .map_err(|e| SceneLoadError::Parse(e))?;
                                        Arc::new(image) as Arc<dyn crate::core::texture::Texture>
                                    }
                                    _ => {
                                        let refl = current_bsdf_reflectance.unwrap_or(RGBSpectrum::new(0.5, 0.5, 0.5));
                                        Arc::new(ConstantTexture::new(refl)) as Arc<dyn crate::core::texture::Texture>
                                    }
                                };
                                let bsdf = Arc::new(LambertianDiffuseBSDF::new(texture)) as Arc<dyn BSDF>;
                                bsdfs.insert(id, bsdf);
                            } else if in_shape {
                                let texture = match current_texture_type.as_deref() {
                                    Some("image") => {
                                        let filename = current_texture_filename.clone().ok_or(SceneLoadError::MissingField("texture.filename"))?;
                                        let filename = if Path::new(&filename).is_absolute() {
                                            filename
                                        } else {
                                            base_dir.join(filename).to_string_lossy().to_string()
                                        };
                                        let image = ImageTexture::from_file(&filename)
                                            .map_err(|e| SceneLoadError::Parse(e))?;
                                        Arc::new(image) as Arc<dyn crate::core::texture::Texture>
                                    }
                                    _ => {
                                        let refl = current_bsdf_reflectance.unwrap_or(RGBSpectrum::new(0.5, 0.5, 0.5));
                                        Arc::new(ConstantTexture::new(refl)) as Arc<dyn crate::core::texture::Texture>
                                    }
                                };
                                let bsdf = Arc::new(LambertianDiffuseBSDF::new(texture)) as Arc<dyn BSDF>;
                                current_shape_bsdf_inline = Some(bsdf);
                            }
                        }
                        in_bsdf = false;
                        in_texture = false;
                        current_texture_name = None;
                        current_texture_type = None;
                        current_texture_filename = None;
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
                                let emitter = EnvMap::from_file(&filename, scale)
                                    .map_err(|e| SceneLoadError::Parse(e))?;
                                scene.add_emitter(Box::new(emitter));
                            }
                        }
                        in_emitter = false;
                        current_emitter_type = None;
                        current_emitter_direction = None;
                        current_emitter_irradiance = None;
                        current_envmap_filename = None;
                        current_envmap_scale = None;
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
                                    let transform = Transform::new(current_shape_transform);
                                    mesh.apply_transform_matrix(&transform);
                                    Arc::new(mesh)
                                }
                                Some("rectangle") => {
                                    let transform = Transform::new(current_shape_transform);
                                    Arc::new(Rectangle::new(transform))
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
                        current_shape_translate = Vector3f::new(0.0, 0.0, 0.0);
                        current_shape_scale = Vector3f::new(1.0, 1.0, 1.0);
                        current_shape_transform = Matrix4f::identity();
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

    let integrator = if let Some(depth) = max_depth {
        Some(Box::new(PathIntegrator::new(depth, spp.unwrap_or(1))) as Box<dyn Integrator>)
    } else {
        None
    };

    Ok(SceneLoadResult {
        scene,
        integrator,
        samples_per_pixel: spp,
        max_depth,
    })
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
