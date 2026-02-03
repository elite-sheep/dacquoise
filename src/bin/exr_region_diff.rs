use exr::prelude::*;

#[derive(Debug)]
struct Pixels {
    width: usize,
    height: usize,
    data: Vec<(f32, f32, f32, f32)>,
}

fn read_rgba(path: &str) -> Pixels {
    let image = read()
        .no_deep_data()
        .largest_resolution_level()
        .rgba_channels(
            |resolution, _| Pixels {
                width: resolution.width(),
                height: resolution.height(),
                data: vec![(0.0, 0.0, 0.0, 0.0); resolution.width() * resolution.height()],
            },
            |image, position, (r, g, b, a): (f32, f32, f32, f32)| {
                let idx = position.y() * image.width + position.x();
                image.data[idx] = (r, g, b, a);
            },
        )
        .first_valid_layer()
        .all_attributes()
        .from_file(path)
        .unwrap_or_else(|e| panic!("failed to read {}: {}", path, e));

    image.layer_data.channel_data.pixels
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 7 {
        eprintln!("Usage: {} <image_a.exr> <image_b.exr> <x0> <y0> <x1> <y1>", args[0]);
        std::process::exit(1);
    }

    let x0: usize = args[3].parse().unwrap_or(0);
    let y0: usize = args[4].parse().unwrap_or(0);
    let x1: usize = args[5].parse().unwrap_or(0);
    let y1: usize = args[6].parse().unwrap_or(0);

    let a = read_rgba(&args[1]);
    let b = read_rgba(&args[2]);

    if a.width != b.width || a.height != b.height {
        eprintln!("Size mismatch: {}x{} vs {}x{}", a.width, a.height, b.width, b.height);
        std::process::exit(2);
    }

    let x1 = x1.min(a.width - 1);
    let y1 = y1.min(a.height - 1);
    let mut sum_abs = (0.0f64, 0.0f64, 0.0f64);
    let mut max_abs = (0.0f32, 0.0f32, 0.0f32);
    let mut count = 0usize;
    let mut nan_count = 0usize;
    let mut inf_count = 0usize;

    for y in y0..=y1 {
        for x in x0..=x1 {
            let idx = y * a.width + x;
            let (ar, ag, ab, _) = a.data[idx];
            let (br, bg, bb, _) = b.data[idx];
            if !ar.is_finite() || !ag.is_finite() || !ab.is_finite() {
                if ar.is_nan() || ag.is_nan() || ab.is_nan() {
                    nan_count += 1;
                } else {
                    inf_count += 1;
                }
                continue;
            }
            let dr = (ar - br).abs();
            let dg = (ag - bg).abs();
            let db = (ab - bb).abs();
            sum_abs.0 += dr as f64;
            sum_abs.1 += dg as f64;
            sum_abs.2 += db as f64;
            if dr > max_abs.0 { max_abs.0 = dr; }
            if dg > max_abs.1 { max_abs.1 = dg; }
            if db > max_abs.2 { max_abs.2 = db; }
            count += 1;
        }
    }

    let n = count.max(1) as f64;
    println!("Region ({}, {})-({}, {})", x0, y0, x1, y1);
    println!("Mean abs diff: R {:.6}, G {:.6}, B {:.6}", sum_abs.0 / n, sum_abs.1 / n, sum_abs.2 / n);
    println!("Max abs diff:  R {:.6}, G {:.6}, B {:.6}", max_abs.0, max_abs.1, max_abs.2);
    if nan_count > 0 || inf_count > 0 {
        println!("Non-finite in region: NaN {}, Inf {}", nan_count, inf_count);
    }
}
