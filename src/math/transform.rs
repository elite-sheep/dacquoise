// Copyright 2020 @TwoCookingMice

use super::constants::{ Vector3f, Matrix4f };
use super::ray::Ray3f;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Transform {
    matrix: Matrix4f,
    inv_matrix: Matrix4f
}

impl Default for Transform {
    fn default() -> Self {
        Self { matrix: Matrix4f::identity(),
               inv_matrix: Matrix4f::identity() }
    }
}

impl Transform {
    pub fn new(matrix: Matrix4f) -> Self {
        Self { matrix: matrix,
               inv_matrix: matrix.try_inverse().unwrap_or(Matrix4f::identity())}
    }

    pub fn apply_point(&self, p: Vector3f) -> Vector3f {
        let x = p[0] * self.matrix[(0, 0)] + p[1] * self.matrix[(0, 1)] + 
            p[2] * self.matrix[(0, 2)] + self.matrix[(0, 3)];
        let y = p[0] * self.matrix[(1, 0)] + p[1] * self.matrix[(1, 1)] + 
            p[2] * self.matrix[(1, 2)] + self.matrix[(1, 3)];
        let z = p[0] * self.matrix[(2, 0)] + p[1] * self.matrix[(2, 1)] + 
            p[2] * self.matrix[(2, 2)] + self.matrix[(2, 3)];
        let w = p[0] * self.matrix[(3, 0)] + p[1] * self.matrix[(3, 1)] + 
            p[2] * self.matrix[(3, 2)] + self.matrix[(3, 3)];

        Vector3f::new(x / w, y / w, z / w)
    }

    pub fn apply_vector(&self, v: Vector3f) -> Vector3f {
        let x = v[0] * self.matrix[(0, 0)] + v[1] * self.matrix[(0, 1)] + v[2] * self.matrix[(0, 2)];
        let y = v[0] * self.matrix[(1, 0)] + v[1] * self.matrix[(1, 1)] + v[2] * self.matrix[(1, 2)];
        let z = v[0] * self.matrix[(2, 0)] + v[1] * self.matrix[(2, 1)] + v[2] * self.matrix[(2, 2)];

        Vector3f::new(x, y, z)
    }

    // Normal transformation is different from point transformation.
    // Before transformation, we have n^Tx = 0
    // After transformation, we have (Sn)^T(Mx) = 0
    // Then, we will get: S = (M^{-1})^T
    pub fn apply_normal(&self, n: Vector3f) -> Vector3f {
        let transpose_inv = self.inv_matrix.transpose();
        let x = n[0] * transpose_inv[(0, 0)] + n[1] * transpose_inv[(0, 1)] + n[2] * transpose_inv[(0, 2)];
        let y = n[0] * transpose_inv[(1, 0)] + n[1] * transpose_inv[(1, 1)] + n[2] * transpose_inv[(1, 2)];
        let z = n[0] * transpose_inv[(2, 0)] + n[1] * transpose_inv[(2, 1)] + n[2] * transpose_inv[(2, 2)];

        Vector3f::new(x, y, z)
    }

    pub fn apply_ray(&self, ray: &Ray3f) -> Ray3f {
        let new_p = self.apply_point(ray.origin());
        let new_d = self.apply_vector(ray.dir());

        Ray3f::new(new_p, new_d, Some(ray.min_t), Some(ray.max_t))
    }

    pub fn inv_apply_point(&self, p: Vector3f) -> Vector3f {
        let x = p[0] * self.inv_matrix[(0, 0)] + p[1] * self.inv_matrix[(0, 1)] + 
            p[2] * self.inv_matrix[(0, 2)] + self.inv_matrix[(0, 3)];
        let y = p[0] * self.inv_matrix[(1, 0)] + p[1] * self.inv_matrix[(1, 1)] + 
            p[2] * self.inv_matrix[(1, 2)] + self.inv_matrix[(1, 3)];
        let z = p[0] * self.inv_matrix[(2, 0)] + p[1] * self.inv_matrix[(2, 1)] + 
            p[2] * self.inv_matrix[(2, 2)] + self.inv_matrix[(2, 3)];
        let w = p[0] * self.inv_matrix[(3, 0)] + p[1] * self.inv_matrix[(3, 1)] + 
            p[2] * self.inv_matrix[(3, 2)] + self.inv_matrix[(3, 3)];

        Vector3f::new(x / w, y / w, z / w)
    }

    pub fn inv_apply_vector(&self, v: Vector3f) -> Vector3f {
        let x = v[0] * self.inv_matrix[(0, 0)] + v[1] * self.inv_matrix[(0, 1)] + v[2] * self.inv_matrix[(0, 2)];
        let y = v[0] * self.inv_matrix[(1, 0)] + v[1] * self.inv_matrix[(1, 1)] + v[2] * self.inv_matrix[(1, 2)];
        let z = v[0] * self.inv_matrix[(2, 0)] + v[1] * self.inv_matrix[(2, 1)] + v[2] * self.inv_matrix[(2, 2)];

        Vector3f::new(x, y, z)
    }

    pub fn inv_apply_normal(&self, n: Vector3f) -> Vector3f {
        let transpose_inv = self.matrix.transpose();
        let x = n[0] * transpose_inv[(0, 0)] + n[1] * transpose_inv[(0, 1)] + n[2] * transpose_inv[(0, 2)];
        let y = n[0] * transpose_inv[(1, 0)] + n[1] * transpose_inv[(1, 1)] + n[2] * transpose_inv[(1, 2)];
        let z = n[0] * transpose_inv[(2, 0)] + n[1] * transpose_inv[(2, 1)] + n[2] * transpose_inv[(2, 2)];

        Vector3f::new(x, y, z)
    }

    pub fn inv_apply_ray(&self, ray: &Ray3f) -> Ray3f {
        let new_p = self.inv_apply_point(ray.origin());
        let new_d = self.inv_apply_vector(ray.dir());

        Ray3f::new(new_p, new_d, Some(ray.min_t), Some(ray.max_t))
    }
}
