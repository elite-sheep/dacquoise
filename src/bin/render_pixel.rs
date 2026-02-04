use dacquoise::core::integrator::Integrator;
use dacquoise::core::rng::LcgRng;
use dacquoise::core::scene_loader::load_scene_with_settings;
use dacquoise::integrators::path::PathIntegrator;
use dacquoise::integrators::raymarching::RaymarchingIntegrator;
use dacquoise::math::constants::{Float, Vector2f, Vector3f};
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        eprintln!("Usage: {} <scene.xml> <x> <y> [--spp N] [--max-depth N] [--seed N] [--camera N]", args[0]);
        std::process::exit(1);
    }

    let scene_path = &args[1];
    let x: usize = args[2].parse().unwrap_or(0);
    let y: usize = args[3].parse().unwrap_or(0);

    let mut spp: u32 = 64;
    let mut max_depth: u32 = 8;
    let mut seed: u64 = 0;
    let mut camera_id: usize = 0;

    let mut i = 4;
    while i < args.len() {
        match args[i].as_str() {
            "--spp" => {
                i += 1;
                spp = args.get(i).and_then(|v| v.parse::<u32>().ok()).unwrap_or(spp);
            }
            "--max-depth" => {
                i += 1;
                max_depth = args.get(i).and_then(|v| v.parse::<u32>().ok()).unwrap_or(max_depth);
            }
            "--seed" => {
                i += 1;
                seed = args.get(i).and_then(|v| v.parse::<u64>().ok()).unwrap_or(seed);
            }
            "--camera" => {
                i += 1;
                camera_id = args.get(i).and_then(|v| v.parse::<usize>().ok()).unwrap_or(camera_id);
            }
            _ => {}
        }
        i += 1;
    }

    let load_result = load_scene_with_settings(scene_path)
        .unwrap_or_else(|e| panic!("failed to load scene: {:?}", e));
    let mut scene = load_result.scene;
    scene.build_bvh();
    let sensor = scene.camera(camera_id).expect("camera not found");
    let (width, height) = {
        let bmp = sensor.bitmap();
        (bmp.width(), bmp.height())
    };
    if x >= width || y >= height {
        eprintln!("Pixel out of bounds: ({}, {}) for size {}x{}", x, y, width, height);
        std::process::exit(2);
    }

    let integrator_name = load_result.integrator_type.as_deref().unwrap_or("path");
    let integrator: Box<dyn Integrator> = match integrator_name {
        "path" => Box::new(PathIntegrator::new(max_depth, spp)),
        "raymarching" => Box::new(RaymarchingIntegrator::new(max_depth, spp, None)),
        other => {
            eprintln!("Unsupported integrator '{}', falling back to path.", other);
            Box::new(PathIntegrator::new(max_depth, spp))
        }
    };
    let pixel = Vector2f::new(x as Float, y as Float);
    let pixel_seed = ((seed & 0xFFF) << 32) | (((y as u64) & 0xFFFF) << 16) | ((x as u64) & 0xFFFF);
    let mut rng = LcgRng::new(pixel_seed);

    let mut accum = Vector3f::zeros();
    for _ in 0..spp {
        let rgb = integrator.trace_ray_forward(&scene, sensor, pixel, &mut rng);
        accum += Vector3f::new(rgb[0], rgb[1], rgb[2]);
    }

    let inv_spp = 1.0 / (spp as Float);
    let avg = accum * inv_spp;
    println!(
        "pixel ({}, {}) spp={} depth={} -> R {:.6}, G {:.6}, B {:.6}",
        x, y, spp, max_depth, avg.x, avg.y, avg.z
    );
}
