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
    if args.len() < 2 {
        eprintln!("Usage: {} <image.exr>", args[0]);
        std::process::exit(1);
    }

    let img = read_rgba(&args[1]);
    let mut min = (f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let mut max = (f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    let mut sum = (0.0f64, 0.0f64, 0.0f64);

    for (r, g, b, _) in &img.data {
        if *r < min.0 { min.0 = *r; }
        if *g < min.1 { min.1 = *g; }
        if *b < min.2 { min.2 = *b; }
        if *r > max.0 { max.0 = *r; }
        if *g > max.1 { max.1 = *g; }
        if *b > max.2 { max.2 = *b; }
        sum.0 += *r as f64;
        sum.1 += *g as f64;
        sum.2 += *b as f64;
    }

    let n = img.data.len() as f64;
    println!("Size: {}x{} ({} pixels)", img.width, img.height, img.data.len());
    println!("Min RGB: {:.6}, {:.6}, {:.6}", min.0, min.1, min.2);
    println!("Max RGB: {:.6}, {:.6}, {:.6}", max.0, max.1, max.2);
    println!("Mean RGB: {:.6}, {:.6}, {:.6}", sum.0 / n, sum.1 / n, sum.2 / n);
}
