use dacquoise::core::interaction::SurfaceIntersection;
use dacquoise::core::scene::Scene;
use dacquoise::core::scene_loader::load_scene_with_settings;
use dacquoise::math::constants::{Float, Vector2f};
use dacquoise::math::ray::Ray3f;
use exr::prelude::*;
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

#[derive(Debug)]
struct Pixels {
    width: usize,
    height: usize,
    data: Vec<(f32, f32, f32, f32)>,
}

fn read_rgba(path: &str) -> Pixels {
    let image = read()
        .no_deep_data()
        .largest_resolution_level()
        .rgba_channels(
            |resolution, _| Pixels {
                width: resolution.width(),
                height: resolution.height(),
                data: vec![(0.0, 0.0, 0.0, 0.0); resolution.width() * resolution.height()],
            },
            |image, position, (r, g, b, a): (f32, f32, f32, f32)| {
                let idx = position.y() * image.width + position.x();
                image.data[idx] = (r, g, b, a);
            },
        )
        .first_valid_layer()
        .all_attributes()
        .from_file(path)
        .unwrap_or_else(|e| panic!("failed to read {}: {}", path, e));

    image.layer_data.channel_data.pixels
}

fn primary_hit(scene: &Scene, ray: &Ray3f) -> Option<(usize, SurfaceIntersection)> {
    let mut best_t = std::f32::MAX;
    let mut best_idx = None;
    let mut best_hit = None;
    for (idx, object) in scene.objects().iter().enumerate() {
        if let Some(hit) = object.shape.ray_intersection(ray) {
            let t = hit.t();
            if t > 0.0 && t < best_t {
                best_t = t;
                best_idx = Some(idx);
                best_hit = Some(hit);
            }
        }
    }
    match (best_idx, best_hit) {
        (Some(idx), Some(hit)) => Some((idx, hit)),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug)]
enum PlyType {
    Char,
    UChar,
    Short,
    UShort,
    Int,
    UInt,
    Float,
    Double,
}

impl PlyType {
    fn from_str(s: &str) -> Option<Self> {
        match s {
            "char" | "int8" => Some(Self::Char),
            "uchar" | "uint8" => Some(Self::UChar),
            "short" | "int16" => Some(Self::Short),
            "ushort" | "uint16" => Some(Self::UShort),
            "int" | "int32" => Some(Self::Int),
            "uint" | "uint32" => Some(Self::UInt),
            "float" | "float32" => Some(Self::Float),
            "double" | "float64" => Some(Self::Double),
            _ => None,
        }
    }

    fn size(self) -> usize {
        match self {
            Self::Char | Self::UChar => 1,
            Self::Short | Self::UShort => 2,
            Self::Int | Self::UInt | Self::Float => 4,
            Self::Double => 8,
        }
    }
}

#[derive(Clone, Debug)]
enum PlyProperty {
    Scalar { ty: PlyType, name: String },
    List { count_ty: PlyType, item_ty: PlyType, name: String },
}

#[derive(Clone, Debug)]
struct PlyElement {
    name: String,
    count: usize,
    properties: Vec<PlyProperty>,
}

#[derive(Debug)]
struct PlyHeader {
    raw: Vec<u8>,
    format: String,
    elements: Vec<PlyElement>,
}

fn read_ply_header<R: BufRead>(reader: &mut R) -> std::result::Result<PlyHeader, String> {
    let mut raw = Vec::new();
    let mut elements = Vec::new();
    let mut current: Option<PlyElement> = None;
    let mut format = None;

    loop {
        let mut line = String::new();
        let read = reader.read_line(&mut line).map_err(|e| e.to_string())?;
        if read == 0 {
            return Err("unexpected EOF while reading header".to_string());
        }
        raw.extend_from_slice(line.as_bytes());
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("comment") {
            if trimmed == "end_header" {
                break;
            }
            continue;
        }
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }
        match parts[0] {
            "format" => {
                if parts.len() >= 2 {
                    format = Some(parts[1].to_string());
                }
            }
            "element" => {
                if let Some(elem) = current.take() {
                    elements.push(elem);
                }
                if parts.len() >= 3 {
                    let count = parts[2].parse::<usize>().map_err(|_| "invalid element count".to_string())?;
                    current = Some(PlyElement {
                        name: parts[1].to_string(),
                        count,
                        properties: Vec::new(),
                    });
                }
            }
            "property" => {
                if let Some(ref mut elem) = current {
                    if parts.len() >= 3 && parts[1] != "list" {
                        let ty = PlyType::from_str(parts[1])
                            .ok_or_else(|| format!("unsupported ply property type {}", parts[1]))?;
                        elem.properties.push(PlyProperty::Scalar {
                            ty,
                            name: parts[2].to_string(),
                        });
                    } else if parts.len() >= 5 && parts[1] == "list" {
                        let count_ty = PlyType::from_str(parts[2])
                            .ok_or_else(|| format!("unsupported ply list count type {}", parts[2]))?;
                        let item_ty = PlyType::from_str(parts[3])
                            .ok_or_else(|| format!("unsupported ply list item type {}", parts[3]))?;
                        elem.properties.push(PlyProperty::List {
                            count_ty,
                            item_ty,
                            name: parts[4].to_string(),
                        });
                    }
                }
            }
            "end_header" => {
                break;
            }
            _ => {}
        }

        if trimmed == "end_header" {
            break;
        }
    }

    if let Some(elem) = current {
        elements.push(elem);
    }

    let format = format.ok_or_else(|| "missing ply format".to_string())?;
    Ok(PlyHeader { raw, format, elements })
}

fn read_scalar<R: Read>(reader: &mut R, ty: PlyType) -> std::result::Result<i64, String> {
    let mut buf = [0u8; 8];
    let size = ty.size();
    reader.read_exact(&mut buf[..size]).map_err(|e| e.to_string())?;
    let val = match ty {
        PlyType::Char => i8::from_le_bytes([buf[0]]) as i64,
        PlyType::UChar => u8::from_le_bytes([buf[0]]) as i64,
        PlyType::Short => i16::from_le_bytes([buf[0], buf[1]]) as i64,
        PlyType::UShort => u16::from_le_bytes([buf[0], buf[1]]) as i64,
        PlyType::Int => i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as i64,
        PlyType::UInt => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as i64,
        PlyType::Float => f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as i64,
        PlyType::Double => f64::from_le_bytes([buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]]) as i64,
    };
    Ok(val)
}

fn write_scalar<W: Write>(writer: &mut W, ty: PlyType, value: i64) -> std::result::Result<(), String> {
    match ty {
        PlyType::Char => writer.write_all(&(value as i8).to_le_bytes()),
        PlyType::UChar => writer.write_all(&(value as u8).to_le_bytes()),
        PlyType::Short => writer.write_all(&(value as i16).to_le_bytes()),
        PlyType::UShort => writer.write_all(&(value as u16).to_le_bytes()),
        PlyType::Int => writer.write_all(&(value as i32).to_le_bytes()),
        PlyType::UInt => writer.write_all(&(value as u32).to_le_bytes()),
        PlyType::Float => writer.write_all(&(value as f32).to_le_bytes()),
        PlyType::Double => writer.write_all(&(value as f64).to_le_bytes()),
    }
    .map_err(|e| e.to_string())
}

fn flip_ply_faces(path: &Path, triangles_to_flip: &HashSet<usize>) -> std::result::Result<(usize, usize), String> {
    if triangles_to_flip.is_empty() {
        return Ok((0, 0));
    }

    let file = File::open(path).map_err(|e| e.to_string())?;
    let mut reader = BufReader::new(file);
    let header = read_ply_header(&mut reader)?;
    if header.format != "binary_little_endian" {
        return Err(format!("unsupported ply format: {}", header.format));
    }

    let mut vertex_elem = None;
    let mut face_elem = None;
    for elem in &header.elements {
        match elem.name.as_str() {
            "vertex" => vertex_elem = Some(elem.clone()),
            "face" => face_elem = Some(elem.clone()),
            _ => return Err(format!("unsupported extra ply element: {}", elem.name)),
        }
    }

    let vertex_elem = vertex_elem.ok_or_else(|| "missing vertex element".to_string())?;
    let face_elem = face_elem.ok_or_else(|| "missing face element".to_string())?;

    let mut vertex_stride = 0usize;
    for prop in &vertex_elem.properties {
        match prop {
            PlyProperty::Scalar { ty, .. } => vertex_stride += ty.size(),
            PlyProperty::List { .. } => return Err("list property in vertex not supported".to_string()),
        }
    }

    let (count_ty, index_ty) = match face_elem.properties.as_slice() {
        [PlyProperty::List { count_ty, item_ty, .. }] => (*count_ty, *item_ty),
        _ => return Err("face element must have a single list property".to_string()),
    };

    let vertex_bytes_len = vertex_stride * vertex_elem.count;
    let mut vertex_bytes = vec![0u8; vertex_bytes_len];
    reader.read_exact(&mut vertex_bytes).map_err(|e| e.to_string())?;

    let tmp_path = path.with_extension("tmp");
    let tmp_file = File::create(&tmp_path).map_err(|e| e.to_string())?;
    let mut writer = BufWriter::new(tmp_file);

    writer.write_all(&header.raw).map_err(|e| e.to_string())?;
    writer.write_all(&vertex_bytes).map_err(|e| e.to_string())?;

    let mut flipped = 0usize;
    let mut tri_cursor = 0usize;
    for _face_idx in 0..face_elem.count {
        let count = read_scalar(&mut reader, count_ty)? as usize;
        let mut indices = Vec::with_capacity(count);
        for _ in 0..count {
            let idx = read_scalar(&mut reader, index_ty)?;
            indices.push(idx);
        }
        let mut flip = false;
        if count >= 3 {
            let tri_count = count - 2;
            for local in 0..tri_count {
                if triangles_to_flip.contains(&(tri_cursor + local)) {
                    flip = true;
                    break;
                }
            }
            tri_cursor += tri_count;
        }
        if flip {
            indices.reverse();
            flipped += 1;
        }
        write_scalar(&mut writer, count_ty, count as i64)?;
        for idx in indices {
            write_scalar(&mut writer, index_ty, idx)?;
        }
    }

    writer.flush().map_err(|e| e.to_string())?;
    std::fs::rename(&tmp_path, path).map_err(|e| e.to_string())?;
    Ok((face_elem.count, flipped))
}

fn extract_attr(line: &str, key: &str) -> Option<String> {
    let needle = format!("{}=\"", key);
    let start = line.find(&needle)? + needle.len();
    let rest = &line[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

fn load_shape_filenames(scene_path: &str) -> std::result::Result<HashMap<String, String>, String> {
    let file = File::open(scene_path).map_err(|e| e.to_string())?;
    let reader = BufReader::new(file);
    let mut map = HashMap::new();
    let mut in_shape = false;
    let mut current_id: Option<String> = None;
    let mut current_filename: Option<String> = None;

    for line in reader.lines() {
        let line = line.map_err(|e| e.to_string())?;
        let trimmed = line.trim();
        if trimmed.starts_with("<shape") {
            in_shape = true;
            current_id = extract_attr(trimmed, "id");
            current_filename = None;
        }
        if in_shape {
            if trimmed.contains("name=\"filename\"") {
                if let Some(value) = extract_attr(trimmed, "value") {
                    current_filename = Some(value);
                }
            }
            if trimmed.starts_with("</shape") {
                if let (Some(id), Some(filename)) = (current_id.take(), current_filename.take()) {
                    map.insert(id, filename);
                }
                in_shape = false;
            }
        }
    }

    Ok(map)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: {} <scene.xml> <image.exr> [threshold]", args[0]);
        std::process::exit(1);
    }

    let scene_path = &args[1];
    let exr_path = &args[2];
    let threshold = args.get(3).and_then(|v| v.parse::<Float>().ok()).unwrap_or(1e-6);

    let load = load_scene_with_settings(scene_path)
        .unwrap_or_else(|e| panic!("failed to load scene {}: {:?}", scene_path, e));
    let scene = load.scene;
    let shape_files = load_shape_filenames(scene_path)
        .unwrap_or_else(|e| panic!("failed to parse {}: {}", scene_path, e));
    let base_dir = scene.base_dir().to_path_buf();
    let sensor = scene.camera(0).expect("camera not found");

    let img = read_rgba(exr_path);
    let width = img.width;
    let height = img.height;

    let mut black_pixels = 0usize;
    let mut backfaced_hits = 0usize;
    let mut tri_map: HashMap<String, HashSet<usize>> = HashMap::new();

    for y in 0..height {
        for x in 0..width {
            let (r, g, b, _) = img.data[y * width + x];
            if !r.is_finite() || !g.is_finite() || !b.is_finite() {
                continue;
            }
            if r <= threshold && g <= threshold && b <= threshold {
                black_pixels += 1;
                let u = (x as Float + 0.5) / (width as Float);
                let v = (y as Float + 0.5) / (height as Float);
                let ray = sensor.sample_ray(&Vector2f::new(u, v));
                if let Some((obj_idx, hit)) = primary_hit(&scene, &ray) {
                    let view = -ray.dir();
                    if view.dot(&hit.geo_normal()) < 0.0 {
                        backfaced_hits += 1;
                        let object = &scene.objects()[obj_idx];
                        let name = match object.name.as_deref() {
                            Some(n) => n,
                            None => continue,
                        };
                        let filename = match shape_files.get(name) {
                            Some(f) => f,
                            None => continue,
                        };
                        let path = if Path::new(filename).is_absolute() {
                            PathBuf::from(filename)
                        } else {
                            base_dir.join(filename)
                        };
                        if let Some(tri_idx) = hit.triangle_index() {
                            tri_map.entry(path.to_string_lossy().to_string()).or_default().insert(tri_idx);
                        }
                    }
                }
            }
        }
    }

    println!(
        "black pixels: {}, backfaced hits: {}, meshes to update: {}",
        black_pixels,
        backfaced_hits,
        tri_map.len()
    );

    for (path_str, triangles) in tri_map {
        let path = PathBuf::from(path_str);
        let (total, flipped) = flip_ply_faces(&path, &triangles)
            .unwrap_or_else(|e| panic!("failed to update {}: {}", path.display(), e));
        println!(
            "updated {}: flipped {}/{} faces",
            path.display(),
            flipped,
            total
        );
    }
}
