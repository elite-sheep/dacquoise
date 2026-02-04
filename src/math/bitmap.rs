// Copyright 2020 @TwoCookingMice

use super::constants::{Float, MatrixXF, Vector3f};
use std::vec::Vec;

#[derive(Debug, Clone)]
pub struct Bitmap {
    data: MatrixXF,
    height: usize,
    width: usize,
}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        let columns = width * 3;
        let data = MatrixXF::zeros(height, columns);
        Self {
            data,
            width,
            height,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn pixel(&self, x: usize, y: usize) -> Vector3f {
        assert!(x < self.width && y < self.height);
        let base = x * 3;
        Vector3f::new(
            self.data[(y, base)],
            self.data[(y, base + 1)],
            self.data[(y, base + 2)],
        )
    }

    pub fn set_pixel(&mut self, x: usize, y: usize, value: Vector3f) {
        assert!(x < self.width && y < self.height);
        let base = x * 3;
        self.data[(y, base)] = value[0];
        self.data[(y, base + 1)] = value[1];
        self.data[(y, base + 2)] = value[2];
    }

    pub fn matrix(&self) -> &MatrixXF {
        &self.data
    }

    pub fn raw_copy(&self) -> Vec<(Float, Float, Float)> {
        let mut raw_copy = vec!((0.0, 0.0, 0.0); (self.width * self.height) as usize);
        for i in 0..self.height {
            for j in 0..self.width {
                let index = (i * self.width + j) as usize;
                let base = j * 3;
                raw_copy[index].0 = self.data[(i, base)];
                raw_copy[index].1 = self.data[(i, base + 1)];
                raw_copy[index].2 = self.data[(i, base + 2)];
            }
        }

        raw_copy
    }
}

/* Test for Bitmap */
#[cfg(test)]
mod tests {
    use super::{Bitmap, Vector3f};

    #[test]
    fn test_bitmap_basic_functions() {
        let mut bitmap = Bitmap::new(256usize, 256usize);
        assert_eq!(bitmap.width(), 256);
        assert_eq!(bitmap.height(), 256);

        bitmap.set_pixel(5, 6, Vector3f::new(1.0, 0.5, 0.6));
        assert_ne!((bitmap.pixel(5, 6)[0] - 1.0).abs(), 0.000001);
        assert_ne!((bitmap.pixel(2, 6)[0] - 0.0).abs(), 0.000001);
    }
}
