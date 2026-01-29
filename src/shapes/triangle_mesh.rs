// Copyright @yucwang 2023

use super::triangle::Triangle;

use crate::math::constants::{ Vector2f, Vector3f };

use std::vec::Vec;

pub struct TriangleMesh {
    vertices: Vec<Vector3f>,
    normals:  Vec<Vector3f>,
    uvs:      Vec<Vector2f>,
    triangles:Vec<Triangle>
}
