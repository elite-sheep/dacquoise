// Copyright 2020 TwoCookingMice

#![allow(dead_code)]

pub extern crate nalgebra as na;

mod core;
mod io;
mod integrators;
mod materials;
mod math;
mod emitters;
mod renderers;
mod sensors;
mod shapes;
mod textures;

use self::core::scene_loader::load_scene_with_settings;
use self::integrators::path::PathIntegrator;
use self::io::exr_utils;
use self::renderers::simple::{ SimpleRenderer, Renderer };

use std::env;

fn main() {
    env::set_var("RUST_LOG", "info");
    env_logger::init();

    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: {} <scene.xml> <output.exr> [--spp N] [--max-depth N] [--seed N] [--camera N]", args[0]);
        std::process::exit(1);
    }

    let input_path = &args[1];
    let output_path = &args[2];
    let mut spp_override: Option<u32> = None;
    let mut max_depth_override: Option<u32> = None;
    let mut seed: u64 = 0;
    let mut camera_id: usize = 0;

    let mut i = 3;
    while i < args.len() {
        match args[i].as_str() {
            "--spp" => {
                i += 1;
                spp_override = args.get(i).and_then(|v| v.parse::<u32>().ok());
            }
            "--max-depth" => {
                i += 1;
                max_depth_override = args.get(i).and_then(|v| v.parse::<u32>().ok());
            }
            "--seed" => {
                i += 1;
                seed = args.get(i).and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
            }
            "--camera" => {
                i += 1;
                camera_id = args.get(i).and_then(|v| v.parse::<usize>().ok()).unwrap_or(0);
            }
            _ => {}
        }
        i += 1;
    }

    let load_result = load_scene_with_settings(input_path)
        .expect("failed to load scene");

    let mut scene = load_result.scene;
    let spp = spp_override.or(load_result.samples_per_pixel).unwrap_or(1);
    let max_depth = max_depth_override.or(load_result.max_depth).unwrap_or(1);
    let integrator = Box::new(PathIntegrator::new(max_depth, spp));

    let renderer: SimpleRenderer = SimpleRenderer::new(integrator, camera_id, seed);
    let image = renderer.render(&mut scene);
    exr_utils::write_exr_to_file(&image.raw_copy(), image.width(), image.height(), output_path);
}
