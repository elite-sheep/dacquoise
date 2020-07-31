// Copyright 2020 TwoCookingMice

mod io;
mod math;

use self::io::exr_utils;

use std::env;

fn main() {
    env::set_var("RUST_LOG", "info");
    env_logger::init();

    let test_exr_name = String::from("/Users/apple/Desktop/cbox-path.exr");
    exr_utils::read_exr_from_file(&test_exr_name);
    println!("Hello, world!");
}
