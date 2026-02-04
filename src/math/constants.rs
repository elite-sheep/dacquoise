// Copyright 2020 @TwoCookingMice

extern crate nalgebra as na;

// Scalar definitions.
pub type Float = f32;
pub type Int = i32;
pub type UInt = u32;

// Vector definitions.
pub type Vector2i = na::base::Vector2<Int>;
pub type Vector2f = na::base::Vector2<Float>;
pub type Vector3i = na::base::Vector3<Int>;
pub type Vector3f = na::base::Vector3<Float>;
pub type Vector4i = na::base::Vector4<Int>;
pub type Vector4f = na::base::Vector4<Float>;
pub type Matrix2i = na::base::Matrix2<Int>;
pub type Matrix2f = na::base::Matrix2<Float>;
pub type Matrix3i = na::base::Matrix3<Int>;
pub type Matrix3f = na::base::Matrix3<Float>;
pub type Matrix4i = na::base::Matrix4<Int>;
pub type Matrix4f = na::base::Matrix4<Float>;
pub type MatrixXF = na::base::DMatrix<Float>;

// Constants.
pub const EPSILON: Float = 1e-8;
pub const PI: Float = std::f32::consts::PI;
pub const INV_PI: Float = 1.0 / std::f32::consts::PI;
pub const SQUARE_2: Float = std::f32::consts::SQRT_2;
pub const INV_SQUARE_2: Float = SQUARE_2 / 2.0;
pub const FLOAT_MIN: Float = std::f32::MIN;
pub const FLOAT_MAX: Float = std::f32::MAX;

#[cfg(test)]
mod tests {
    use super::{Vector2i, Vector3i, Vector4i};
    use super::{Vector2f, Vector3f, Vector4f};

    #[test]
    fn test_vectorni() {
        let v21 = Vector2i::new(1, 5);
        let v22 = Vector2i::new(2, 4);
        let v23 = v21 + v22;
        assert_eq!(v23[0], 3);
        assert_eq!(v23[1], 9);

        let v31 = Vector3i::new(1, 5, 4);
        let v32 = Vector3i::new(3, 7, 6);
        let v33 = v31 - v32;
        assert_eq!(v33[0], -2);
        assert_eq!(v33[1], -2);
        assert_eq!(v33[2], -2);

        let v41 = Vector4i::new(2, 6, 4, 3);
        let v42 = Vector4i::new(5, 7, 1, 1);
        let v43 = v41 - v42;
        assert_eq!(v43[0], -3);
        assert_eq!(v43[1], -1);
        assert_eq!(v43[2], 3);
        assert_eq!(v43[3], 2);
    }

    #[test]
    fn test_vectornf() {
        let v21 = Vector2f::new(1.0, 2.0);
        let v22 = Vector2f::new(3.0, 4.0);
        let v23 = v21 + v22;
        assert_eq!(v23[0], 4.0);
        assert_eq!(v23[1], 6.0);

        let v31 = Vector3f::new(1.0, 5.0, 4.0);
        let v32 = Vector3f::new(3.0, 7.0, 6.0);
        let v33 = v31 - v32;
        assert_eq!(v33[0], -2.0);
        assert_eq!(v33[1], -2.0);
        assert_eq!(v33[2], -2.0);

        let v41 = Vector4f::new(2.0, 6.0, 4.0, 3.0);
        let v42 = Vector4f::new(5.0, 7.0, 1.0, 1.0);
        let v43 = v41 - v42;
        assert_eq!(v43[0], -3.0);
        assert_eq!(v43[1], -1.0);
        assert_eq!(v43[2], 3.0);
        assert_eq!(v43[3], 2.0);
    }
}
