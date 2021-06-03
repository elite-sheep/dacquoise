// Copyright 2020 @TwoCookingMice

use super::constants::{ Float, Vector3f };

use std::ops;
use std::vec::Vec;

#[derive(Debug)]
pub struct Bitmap {
    data: Vec<Vector3f>,
    height: usize,
    width: usize
}

impl ops::Index<(usize, usize)> for Bitmap {
    type Output = Vector3f;

    fn index(&self, index: (usize, usize)) -> &Vector3f {
        let transformed_index = index.0 + self.width * index.1;
        assert_ne!(transformed_index, self.height * self.width);
        &self.data[transformed_index as usize]
    }
}

impl ops::IndexMut<(usize, usize)> for Bitmap {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Vector3f {
        let transformed_index = index.0 + self.width * index.1;
        assert_ne!(transformed_index, self.height * self.width);
        &mut self.data[transformed_index as usize]
    }
}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        let pixel_number = width * height;
        Self { data: vec!(Vector3f::new(0.0, 0.0, 0.0); 
                          pixel_number),
               width: width,
               height: height }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn raw_copy(&self) -> Vec<(Float, Float, Float)> {
        let mut raw_copy = vec!((0.0, 0.0, 0.0); (self.width * self.height) as usize);
        for i in 0..self.height {
            for j in 0..self.width {
                let index = (i * self.width + j) as usize;
                raw_copy[index].0 = self.data[index][0];
                raw_copy[index].1 = self.data[index][1];
                raw_copy[index].2 = self.data[index][2];
            }
        }

        raw_copy
    }
}

/* Test for Bitmap */
mod tests {
    use super::{ Bitmap };
    use super::{ Vector3f };

    #[test]
    fn test_bitmap_basic_functions() {
        let mut bitmap = Bitmap::new(256usize, 256usize);
        assert_eq!(bitmap.width(), 256);
        assert_eq!(bitmap.height(), 256);

        bitmap[(5, 6)] = Vector3f::new(1.0, 0.5, 0.6);
        assert_ne!((bitmap[(5, 6)][0] - 1.0).abs(), 0.000001);
        assert_ne!((bitmap[(2, 6)][0] - 0.0).abs(), 0.000001);
    }
}



