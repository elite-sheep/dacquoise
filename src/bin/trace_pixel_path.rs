use dacquoise::core::rng::LcgRng;
use dacquoise::core::scene_loader::load_scene_with_settings;
use dacquoise::core::tangent_frame::{build_tangent_frame, local_to_world, world_to_local};
use dacquoise::math::constants::{Float, Vector2f, Vector3f};
use dacquoise::math::ray::Ray3f;
use dacquoise::math::spectrum::Spectrum;
use std::env;

fn brute_force_primary_hit(
    scene: &dacquoise::core::scene::Scene,
    ray: &Ray3f,
) -> Option<(usize, dacquoise::core::interaction::SurfaceIntersection)> {
    let mut best_t = std::f32::MAX;
    let mut best_idx: Option<usize> = None;
    let mut best_hit: Option<dacquoise::core::interaction::SurfaceIntersection> = None;
    for (idx, object) in scene.objects().iter().enumerate() {
        if let Some(hit) = object.shape.ray_intersection(ray) {
            let t = hit.t();
            if t > 0.0 && t < best_t {
                best_t = t;
                best_idx = Some(idx);
                best_hit = Some(hit);
            }
        }
    }
    match (best_idx, best_hit) {
        (Some(idx), Some(hit)) => Some((idx, hit)),
        _ => None,
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        eprintln!("Usage: {} <scene.xml> <x> <y> [--max-depth N] [--seed N] [--camera N]", args[0]);
        std::process::exit(1);
    }

    let scene_path = &args[1];
    let x: usize = args[2].parse().unwrap_or(0);
    let y: usize = args[3].parse().unwrap_or(0);

    let mut max_depth: u32 = 8;
    let mut seed: u64 = 0;
    let mut camera_id: usize = 0;

    let mut i = 4;
    while i < args.len() {
        match args[i].as_str() {
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
    let scene = load_result.scene;
    let sensor = scene.camera(camera_id).expect("camera not found");
    let (width, height) = {
        let bmp = sensor.bitmap();
        (bmp.width(), bmp.height())
    };

    if x >= width || y >= height {
        eprintln!("Pixel out of bounds: ({}, {}) for size {}x{}", x, y, width, height);
        std::process::exit(2);
    }

    let pixel_seed = ((seed & 0xFFF) << 32) | (((y as u64) & 0xFFFF) << 16) | ((x as u64) & 0xFFFF);
    let mut rng = LcgRng::new(pixel_seed);
    let u = (x as Float + rng.next_f32()) / (width as Float);
    let v = (y as Float + rng.next_f32()) / (height as Float);
    let mut ray = sensor.sample_ray(&Vector2f::new(u, v));
    let mut throughput = Vector3f::new(1.0, 1.0, 1.0);

    println!("trace_pixel_path: scene={} pixel=({}, {}) u={:.6} v={:.6}", scene_path, x, y, u, v);

    if let Some((idx, hit)) = brute_force_primary_hit(&scene, &ray) {
        let object = &scene.objects()[idx];
        let name = object.name.as_deref().unwrap_or("<unnamed>");
        println!(
            "primary object: {} bsdf={} t={:.6} p=({:.5}, {:.5}, {:.5})",
            name,
            object.material.name(),
            hit.t(),
            hit.p().x, hit.p().y, hit.p().z
        );
    } else {
        println!("primary object: miss");
    }

    for bounce in 0..max_depth {
        let intersection = match scene.ray_intersection(&ray) {
            Some(hit) => hit,
            None => {
                let mut env = Vector3f::zeros();
                for emitter in scene.emitters() {
                    let le = emitter.eval_direction(&ray.dir());
                    env += Vector3f::new(le[0], le[1], le[2]);
                }
                println!(
                    "bounce {}: miss, env=({:.6}, {:.6}, {:.6}) throughput=({:.6}, {:.6}, {:.6})",
                    bounce, env.x, env.y, env.z, throughput.x, throughput.y, throughput.z
                );
                break;
            }
        };

        let material = match intersection.material() {
            Some(m) => m,
            None => {
                println!("bounce {}: hit with no material", bounce);
                break;
            }
        };

        let le = intersection.le();
        if !le.is_black() {
            println!(
                "bounce {}: hit emissive le=({:.6}, {:.6}, {:.6})",
                bounce, le[0], le[1], le[2]
            );
        }

        println!(
            "bounce {}: bsdf={} p=({:.5}, {:.5}, {:.5}) n=({:.5}, {:.5}, {:.5}) uv=({:.5}, {:.5})",
            bounce,
            material.name(),
            intersection.p().x, intersection.p().y, intersection.p().z,
            intersection.geo_normal().x, intersection.geo_normal().y, intersection.geo_normal().z,
            intersection.uv().x, intersection.uv().y
        );

        let n_geo = intersection.geo_normal();
        let n_sh = intersection.sh_normal();
        let wi_world = -ray.dir();
        let (tangent, bitangent) = build_tangent_frame(&n_sh);
        let wi_local = world_to_local(&wi_world, &tangent, &bitangent, &n_sh);
        let wi_dot_geo = wi_world.dot(&n_geo);
        println!(
            "bounce {}: wi_local.z={:.6} wi_dot_geo={:.6} (sign={})",
            bounce,
            wi_local.z,
            wi_dot_geo,
            if wi_local.z * wi_dot_geo > 0.0 { "ok" } else { "mismatch" }
        );

        let u1 = Vector2f::new(rng.next_f32(), rng.next_f32());
        let u2 = Vector2f::new(rng.next_f32(), rng.next_f32());
        let mut sample = material.sample(u1, u2, wi_local);
        sample.uv = intersection.uv();
        let pdf = sample.pdf;
        let wo_local = sample.wo;
        if pdf <= 0.0 {
            println!("bounce {}: bsdf pdf <= 0, terminate", bounce);
            break;
        }

        let cos_theta = wo_local.z.abs();
        let eval = material.eval(sample);
        let f = Vector3f::new(eval.value[0], eval.value[1], eval.value[2]);
        let bsdf_weight = f * (cos_theta / pdf);
        throughput = throughput.component_mul(&bsdf_weight);

        println!(
            "bounce {}: wo_local=({:.5}, {:.5}, {:.5}) pdf={:.6} f=({:.6}, {:.6}, {:.6}) weight=({:.6}, {:.6}, {:.6}) throughput=({:.6}, {:.6}, {:.6})",
            bounce,
            wo_local.x, wo_local.y, wo_local.z,
            pdf,
            f.x, f.y, f.z,
            bsdf_weight.x, bsdf_weight.y, bsdf_weight.z,
            throughput.x, throughput.y, throughput.z
        );

        let wo_world = local_to_world(&wo_local, &tangent, &bitangent, &n_sh);
        let offset_dir = if wo_world.dot(&n_geo) >= 0.0 { n_geo } else { -n_geo };
        let origin = intersection.p() + offset_dir * 1e-6;
        ray = Ray3f::new(origin, wo_world, Some(1e-4), None);
    }
}
