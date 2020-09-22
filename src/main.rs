// Copyright 2020 TwoCookingMice

pub extern crate nalgebra as na;

mod io;
mod math;

use self::io::exr_utils;
use self::math::constants::{ Vector3f };
use self::math::bitmap::Bitmap;

use std::env;

fn main() {
    env::set_var("RUST_LOG", "info");
    env_logger::init();

    let test_exr_name = String::from("/Users/apple/Desktop/test.exr");
    let mut bitmap = Bitmap::new(256, 256);
    for i in 0..256 {
        for j in 0..256 {
            if i > 128 {
                bitmap[(i, j)] = Vector3f::new(0.5, 0.0, 0.0);
            } else {
                bitmap[(i, j)] = Vector3f::new(0.0, 0.5, 0.0);
            }
        }
    }

    exr_utils::write_exr_to_file(&bitmap.raw_copy(), bitmap.width() as u32, bitmap.height() as u32, &test_exr_name);
}
