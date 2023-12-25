// Copyright @yucwang 2021

use crate::core::computation_node::Computation_Node;
use crate::io::exr_utils;
use crate::math::bitmap::Bitmap;
use crate::math::constants::Vector3f;
use crate::math::spectrum::RGBSpectrum;

pub use super::renderer::Renderer;

pub struct SimpleRenderer {
    colors: [RGBSpectrum; 4]
}

impl Computation_Node for SimpleRenderer {
    fn to_string(&self) -> String {
        String::from("SimpleRenderer: {}")
    }
}

impl Renderer for SimpleRenderer {
    fn render(&self) {
        let mut tmp_bitmap = Bitmap::new(256, 256);
        for i in 0..256 {
            for j in 0..256 {
                let mut color_index = 0;
                if i >= 128 {
                    color_index += 1;
                }
                if j >= 128 {
                    color_index += 2;
                }
                tmp_bitmap[(i, j)] = Vector3f::new(self.colors[color_index][0], 
                                                   self.colors[color_index][1],
                                                   self.colors[color_index][2]);
            }
        }

        exr_utils::write_exr_to_file(&tmp_bitmap.raw_copy(), 
                                     256 as usize, 
                                     256 as usize, 
                                     "./test.exr");
    }
}

impl SimpleRenderer {
    pub fn new(new_colors: [RGBSpectrum; 4]) -> Self {
        Self {
            colors: new_colors,
        }
    }
}
