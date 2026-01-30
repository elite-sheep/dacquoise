// Copyright @yucwang 2026

use crate::math::constants::Vector3f;

pub fn build_tangent_frame(n: &Vector3f) -> (Vector3f, Vector3f) {
    let up = if n.z.abs() < 0.999 {
        Vector3f::new(0.0, 0.0, 1.0)
    } else {
        Vector3f::new(1.0, 0.0, 0.0)
    };
    let tangent = n.cross(&up).normalize();
    let bitangent = n.cross(&tangent).normalize();
    (tangent, bitangent)
}

pub fn world_to_local(v: &Vector3f, t: &Vector3f, b: &Vector3f, n: &Vector3f) -> Vector3f {
    Vector3f::new(v.dot(t), v.dot(b), v.dot(n))
}

pub fn local_to_world(v: &Vector3f, t: &Vector3f, b: &Vector3f, n: &Vector3f) -> Vector3f {
    t * v.x + b * v.y + n * v.z
}
