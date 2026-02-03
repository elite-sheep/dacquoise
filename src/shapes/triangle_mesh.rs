// Copyright @yucwang 2023

use super::triangle::Triangle;

use crate::core::interaction::{SurfaceIntersection, SurfaceSampleRecord};
use crate::core::shape::Shape;
use crate::core::bvh::BVH;
use crate::io::obj_utils;
use crate::io::obj_utils::ObjLoadError;
use crate::math::aabb::AABB;
use crate::math::constants::{ Float, Vector2f, Vector3f };
use crate::math::ray::Ray3f;
use crate::math::spectrum::RGBSpectrum;
use crate::math::transform::Transform;

use ply_rs_bw::parser::Parser;
use ply_rs_bw::ply::{DefaultElement, Property};

use std::fmt;
use std::fs::File;
use std::io::BufReader;
use std::vec::Vec;

#[derive(Debug)]
pub enum PlyLoadError {
    Io(std::io::Error),
    Parse(String),
    MissingElement(&'static str),
}

impl From<std::io::Error> for PlyLoadError {
    fn from(err: std::io::Error) -> Self {
        PlyLoadError::Io(err)
    }
}

impl fmt::Display for PlyLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlyLoadError::Io(err) => write!(f, "io error: {}", err),
            PlyLoadError::Parse(err) => write!(f, "parse error: {}", err),
            PlyLoadError::MissingElement(name) => write!(f, "missing element: {}", name),
        }
    }
}

impl std::error::Error for PlyLoadError {}

pub struct TriangleMesh {
    vertices: Vec<Vector3f>,
    normals:  Vec<Vector3f>,
    uvs:      Vec<Vector2f>,
    triangles:Vec<Triangle>,
    tri_areas: Vec<Float>,
    total_area: Float,
    tri_normals: Vec<Vector3f>,
    tri_uv_indices: Vec<[Option<usize>; 3]>,
    bvh: Option<BVH>,
    use_face_normals: bool,
}

impl TriangleMesh {
    pub fn from_obj(path: &str) -> Result<Self, ObjLoadError> {
        let obj_set = obj_utils::load_obj_from_file(path)?;
        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut uvs = Vec::new();
        let mut triangles = Vec::new();
        let mut tri_areas = Vec::new();
        let mut total_area = 0.0;
        let mut tri_normals = Vec::new();
        let mut tri_uv_indices = Vec::new();

        for object in obj_set.objects {
            for v in object.vertices {
                vertices.push(Vector3f::new(v.x as f32, v.y as f32, v.z as f32));
            }
            for vn in object.normals {
                normals.push(Vector3f::new(vn.x as f32, vn.y as f32, vn.z as f32));
            }
            for vt in object.tex_vertices {
                uvs.push(Vector2f::new(vt.u as f32, vt.v as f32));
            }
            for geom in object.geometry {
                for shape in geom.shapes {
                    if let wavefront_obj::obj::Primitive::Triangle(a, b, c) = shape.primitive {
                        let p0 = vertices[(a.0) as usize];
                        let p1 = vertices[(b.0) as usize];
                        let p2 = vertices[(c.0) as usize];
                        let tri = Triangle::new(p0, p1, p2);
                        let area = tri.surface_area();
                        total_area += area;
                        tri_areas.push(area);
                        triangles.push(tri);

                        let n0 = a.2.map(|i| i as usize).and_then(|i| normals.get(i)).cloned();
                        let n1 = b.2.map(|i| i as usize).and_then(|i| normals.get(i)).cloned();
                        let n2 = c.2.map(|i| i as usize).and_then(|i| normals.get(i)).cloned();
                        if let (Some(n0), Some(n1), Some(n2)) = (n0, n1, n2) {
                            let n = (n0 + n1 + n2).normalize();
                            tri_normals.push(n);
                        } else {
                            let n = (p1 - p0).cross(&(p2 - p0)).normalize();
                            tri_normals.push(n);
                        }

                        let uv0 = a.1.map(|i| i as usize);
                        let uv1 = b.1.map(|i| i as usize);
                        let uv2 = c.1.map(|i| i as usize);
                        tri_uv_indices.push([uv0, uv1, uv2]);
                    }
                }
            }
        }

        let mut mesh = Self {
            vertices,
            normals,
            uvs,
            triangles,
            tri_areas,
            total_area,
            tri_normals,
            tri_uv_indices,
            bvh: None,
            use_face_normals: false,
        };
        mesh.build_bvh();
        Ok(mesh)
    }

    pub fn from_ply(path: &str) -> Result<Self, PlyLoadError> {
        let file = File::open(path)?;
        let parser = Parser::<DefaultElement>::new();
        let ply = parser
            .read_ply(&mut BufReader::new(file))
            .map_err(|e| PlyLoadError::Parse(e.to_string()))?;

        let vertices_payload = ply
            .payload
            .get("vertex")
            .ok_or(PlyLoadError::MissingElement("vertex"))?;

        let mut vertices = Vec::with_capacity(vertices_payload.len());
        let mut normals = Vec::with_capacity(vertices_payload.len());
        let mut uvs = Vec::with_capacity(vertices_payload.len());
        let mut has_uv = true;
        let mut has_normals = true;

        for v in vertices_payload {
            let x = ply_prop_f32(v, "x").ok_or(PlyLoadError::MissingElement("vertex.x"))?;
            let y = ply_prop_f32(v, "y").ok_or(PlyLoadError::MissingElement("vertex.y"))?;
            let z = ply_prop_f32(v, "z").ok_or(PlyLoadError::MissingElement("vertex.z"))?;
            vertices.push(Vector3f::new(x, y, z));

            let u = ply_prop_f32(v, "u").or_else(|| ply_prop_f32(v, "s"));
            let vv = ply_prop_f32(v, "v").or_else(|| ply_prop_f32(v, "t"));
            if let (Some(u), Some(vv)) = (u, vv) {
                uvs.push(Vector2f::new(u, vv));
            } else {
                has_uv = false;
                uvs.push(Vector2f::new(0.0, 0.0));
            }

            let nx = ply_prop_f32(v, "nx");
            let ny = ply_prop_f32(v, "ny");
            let nz = ply_prop_f32(v, "nz");
            if let (Some(nx), Some(ny), Some(nz)) = (nx, ny, nz) {
                normals.push(Vector3f::new(nx, ny, nz));
            } else {
                has_normals = false;
                normals.push(Vector3f::new(0.0, 0.0, 0.0));
            }
        }

        let faces_payload = ply
            .payload
            .get("face")
            .ok_or(PlyLoadError::MissingElement("face"))?;

        let mut triangles = Vec::new();
        let mut tri_areas = Vec::new();
        let mut total_area = 0.0;
        let mut tri_normals = Vec::new();
        let mut tri_uv_indices = Vec::new();

        for face in faces_payload {
            let indices = ply_prop_indices(face, "vertex_indices")
                .or_else(|| ply_prop_indices(face, "vertex_index"))
                .ok_or(PlyLoadError::MissingElement("face.vertex_indices"))?;

            if indices.len() < 3 {
                continue;
            }

            for i in 1..(indices.len() - 1) {
                let i0 = indices[0];
                let i1 = indices[i];
                let i2 = indices[i + 1];
                if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
                    continue;
                }

                let p0 = vertices[i0];
                let p1 = vertices[i1];
                let p2 = vertices[i2];
                let tri = Triangle::new(p0, p1, p2);
                let area = tri.surface_area();
                total_area += area;
                tri_areas.push(area);
                triangles.push(tri);

                let mut n = if has_normals {
                    normals[i0] + normals[i1] + normals[i2]
                } else {
                    Vector3f::new(0.0, 0.0, 0.0)
                };
                if n.norm() > 0.0 {
                    n = n.normalize();
                } else {
                    n = (p1 - p0).cross(&(p2 - p0)).normalize();
                }
                tri_normals.push(n);

                if has_uv {
                    tri_uv_indices.push([Some(i0), Some(i1), Some(i2)]);
                } else {
                    tri_uv_indices.push([None, None, None]);
                }
            }
        }

        let mut mesh = Self {
            vertices,
            normals,
            uvs,
            triangles,
            tri_areas,
            total_area,
            tri_normals,
            tri_uv_indices,
            bvh: None,
            use_face_normals: false,
        };
        mesh.build_bvh();
        Ok(mesh)
    }

    pub fn apply_transform(&mut self, scale: &Vector3f, translate: &Vector3f) {
        for v in &mut self.vertices {
            *v = v.component_mul(scale) + translate;
        }

        for tri in &mut self.triangles {
            tri.apply_transform(scale, translate);
        }

        self.tri_normals.clear();
        self.tri_areas.clear();
        self.total_area = 0.0;
        for tri in &self.triangles {
            let n = tri.geometric_normal();
            self.tri_normals.push(n);
            let area = tri.surface_area();
            self.tri_areas.push(area);
            self.total_area += area;
        }

        self.build_bvh();
    }

    pub fn apply_transform_matrix(&mut self, transform: &Transform) {
        for v in &mut self.vertices {
            *v = transform.apply_point(*v);
        }

        for tri in &mut self.triangles {
            tri.apply_transform_matrix(transform);
        }

        self.tri_normals.clear();
        self.tri_areas.clear();
        self.total_area = 0.0;
        for tri in &self.triangles {
            let n = tri.geometric_normal();
            self.tri_normals.push(n);
            let area = tri.surface_area();
            self.tri_areas.push(area);
            self.total_area += area;
        }

        self.build_bvh();
    }

    fn build_bvh(&mut self) {
        if self.triangles.is_empty() {
            self.bvh = None;
            return;
        }

        let mut prim_bounds = Vec::with_capacity(self.triangles.len());
        let mut prim_centroids = Vec::with_capacity(self.triangles.len());
        for tri in &self.triangles {
            let bounds = tri.bounding_box();
            prim_centroids.push(bounds.center());
            prim_bounds.push(bounds);
        }

        self.bvh = Some(BVH::new(prim_bounds, prim_centroids));
    }

    fn tri_uv(&self, idx: usize, bary: Vector3f) -> Vector2f {
        let indices = self.tri_uv_indices.get(idx).cloned().unwrap_or([None, None, None]);
        let uv0 = indices[0].and_then(|i| self.uvs.get(i)).cloned().unwrap_or(Vector2f::new(0.0, 0.0));
        let uv1 = indices[1].and_then(|i| self.uvs.get(i)).cloned().unwrap_or(Vector2f::new(0.0, 0.0));
        let uv2 = indices[2].and_then(|i| self.uvs.get(i)).cloned().unwrap_or(Vector2f::new(0.0, 0.0));
        uv0 * bary.x + uv1 * bary.y + uv2 * bary.z
    }

    pub fn set_face_normals(&mut self, use_face_normals: bool) {
        self.use_face_normals = use_face_normals;
    }
}

fn ply_prop_f32(elem: &DefaultElement, name: &str) -> Option<Float> {
    elem.get(name).and_then(ply_property_f32)
}

fn ply_property_f32(prop: &Property) -> Option<Float> {
    match prop {
        Property::Float(v) => Some(*v),
        Property::Double(v) => Some(*v as Float),
        Property::Int(v) => Some(*v as Float),
        Property::UInt(v) => Some(*v as Float),
        Property::Short(v) => Some(*v as Float),
        Property::UShort(v) => Some(*v as Float),
        Property::Char(v) => Some(*v as Float),
        Property::UChar(v) => Some(*v as Float),
        _ => None,
    }
}

fn ply_prop_indices(elem: &DefaultElement, name: &str) -> Option<Vec<usize>> {
    elem.get(name).and_then(ply_property_indices)
}

fn ply_property_indices(prop: &Property) -> Option<Vec<usize>> {
    match prop {
        Property::ListInt(v) => Some(v.iter().filter_map(|x| if *x >= 0 { Some(*x as usize) } else { None }).collect()),
        Property::ListUInt(v) => Some(v.iter().map(|x| *x as usize).collect()),
        Property::ListShort(v) => Some(v.iter().filter_map(|x| if *x >= 0 { Some(*x as usize) } else { None }).collect()),
        Property::ListUShort(v) => Some(v.iter().map(|x| *x as usize).collect()),
        Property::ListChar(v) => Some(v.iter().filter_map(|x| if *x >= 0 { Some(*x as usize) } else { None }).collect()),
        Property::ListUChar(v) => Some(v.iter().map(|x| *x as usize).collect()),
        Property::ListFloat(v) => Some(v.iter().map(|x| *x as usize).collect()),
        Property::ListDouble(v) => Some(v.iter().map(|x| *x as usize).collect()),
        _ => None,
    }
}

impl Shape for TriangleMesh {
    fn bounding_box(&self) -> AABB {
        let mut bound = AABB::default();
        for tri in &self.triangles {
            bound.expand_by_aabb(&tri.bounding_box());
        }
        bound
    }

    fn ray_intersection(&self, ray: &Ray3f) -> Option<SurfaceIntersection> {
        let mut closest_hit: Option<SurfaceIntersection> = None;
        let mut closest_t = std::f32::MAX;

        if let Some(bvh) = &self.bvh {
            if let Some((idx, hit)) = bvh.ray_intersection(ray, |prim_idx, ray| {
                self.triangles[prim_idx].ray_intersection(ray).map(|h| {
                    let t = h.t();
                    (h, t)
                })
            }) {
                let geo_n = hit.geo_normal();
                let mut sh_n = if self.use_face_normals {
                    geo_n
                } else {
                    self.tri_normals.get(idx).cloned().unwrap_or(geo_n)
                };
                if sh_n.dot(&geo_n) < 0.0 {
                    sh_n = -sh_n;
                }
                let bary = self.triangles[idx].barycentric(&hit.p());
                let uv = self.tri_uv(idx, bary);
                let intersection = SurfaceIntersection::new(
                    hit.p(),
                    geo_n,
                    sh_n,
                    uv,
                    hit.t(),
                    RGBSpectrum::default(),
                    None,
                    None,
                )
                .with_triangle_index(Some(idx));
                return Some(intersection);
            }
        } else {
            for (idx, tri) in self.triangles.iter().enumerate() {
                if let Some(hit) = tri.ray_intersection(ray) {
                    let hit_t = hit.t();
                    if hit_t < closest_t {
                        let geo_n = hit.geo_normal();
                        let mut sh_n = if self.use_face_normals {
                            geo_n
                        } else {
                            self.tri_normals.get(idx).cloned().unwrap_or(geo_n)
                        };
                        if sh_n.dot(&geo_n) < 0.0 {
                            sh_n = -sh_n;
                        }
                        let bary = tri.barycentric(&hit.p());
                        let uv = self.tri_uv(idx, bary);
                        let hit = SurfaceIntersection::new(
                            hit.p(),
                            geo_n,
                            sh_n,
                            uv,
                            hit_t,
                            RGBSpectrum::default(),
                            None,
                            None,
                        )
                        .with_triangle_index(Some(idx));
                        closest_t = hit_t;
                        closest_hit = Some(hit);
                    }
                }
            }
        }

        closest_hit
    }

    fn ray_intersection_t(&self, ray: &Ray3f) -> bool {
        if let Some(bvh) = &self.bvh {
            return bvh.ray_intersection_t(ray, |prim_idx, ray| {
                self.triangles[prim_idx].ray_intersection_t(ray)
            });
        }

        for tri in &self.triangles {
            if tri.ray_intersection_t(ray) {
                return true;
            }
        }
        false
    }

    fn sample(&self, u: &Vector2f) -> SurfaceSampleRecord {
        if self.triangles.is_empty() || self.total_area <= 0.0 {
            let p = Vector3f::zeros();
            let n = Vector3f::new(0.0, 0.0, 1.0);
            let uv = Vector2f::new(0.0, 0.0);
            let intersection = SurfaceIntersection::new(p, n, n, uv, 0.0, RGBSpectrum::default(), None, None);
            return SurfaceSampleRecord::new(intersection, 0.0);
        }

        let target = u.x * self.total_area;
        let mut accum = 0.0;
        let mut idx = 0;
        for (i, area) in self.tri_areas.iter().enumerate() {
            accum += *area;
            if target <= accum {
                idx = i;
                break;
            }
        }

        let tri_area = self.tri_areas[idx].max(1e-6);
        let local_u = Vector2f::new(((target - (accum - tri_area)) / tri_area).min(0.999999), u.y);

        let bary = crate::math::warp::square_to_triangle(&local_u);
        let (p0, p1, p2) = self.triangles[idx].vertices();
        let p = p0 * bary.x + p1 * bary.y + p2 * bary.z;

        let geo_n = self.triangles[idx].geometric_normal();
        let mut sh_n = if self.use_face_normals {
            geo_n
        } else {
            self.tri_normals.get(idx).cloned().unwrap_or(geo_n)
        };
        if sh_n.dot(&geo_n) < 0.0 {
            sh_n = -sh_n;
        }

        let uv = self.tri_uv(idx, bary);
        let intersection = SurfaceIntersection::new(
            p,
            geo_n,
            sh_n,
            uv,
            0.0,
            RGBSpectrum::default(),
            None,
            None,
        );
        SurfaceSampleRecord::new(intersection, 1.0 / self.total_area)
    }

    fn surface_area(&self) -> Float {
        self.total_area
    }

}
