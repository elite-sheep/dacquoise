use exr::prelude::*;

#[derive(Debug)]
struct Pixels {
    width: usize,
    height: usize,
    data: Vec<(f32, f32, f32, f32)>,
}

fn read_rgba(path: &str) -> Pixels {
    let image = read().no_deep_data()
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

fn luminance(rgb: (f32, f32, f32)) -> f32 {
    0.2126 * rgb.0 + 0.7152 * rgb.1 + 0.0722 * rgb.2
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: {} <image_a.exr> <image_b.exr>", args[0]);
        std::process::exit(1);
    }

    let a = read_rgba(&args[1]);
    let b = read_rgba(&args[2]);

    if a.width != b.width || a.height != b.height {
        eprintln!("Size mismatch: {}x{} vs {}x{}", a.width, a.height, b.width, b.height);
        std::process::exit(2);
    }

    let mut sum_abs = (0.0f64, 0.0f64, 0.0f64);
    let mut max_abs = (0.0f32, 0.0f32, 0.0f32);
    let mut sum_luma_a = 0.0f64;
    let mut sum_luma_b = 0.0f64;
    let mut sum_luma_abs = 0.0f64;

    for i in 0..a.data.len() {
        let (ar, ag, ab, _) = a.data[i];
        let (br, bg, bb, _) = b.data[i];
        let dr = (ar - br).abs();
        let dg = (ag - bg).abs();
        let db = (ab - bb).abs();
        sum_abs.0 += dr as f64;
        sum_abs.1 += dg as f64;
        sum_abs.2 += db as f64;
        if dr > max_abs.0 { max_abs.0 = dr; }
        if dg > max_abs.1 { max_abs.1 = dg; }
        if db > max_abs.2 { max_abs.2 = db; }

        let la = luminance((ar, ag, ab)) as f64;
        let lb = luminance((br, bg, bb)) as f64;
        sum_luma_a += la;
        sum_luma_b += lb;
        sum_luma_abs += (la - lb).abs();
    }

    let n = a.data.len() as f64;
    println!("Mean abs diff: R {:.6}, G {:.6}, B {:.6}",
        sum_abs.0 / n, sum_abs.1 / n, sum_abs.2 / n);
    println!("Max abs diff:  R {:.6}, G {:.6}, B {:.6}",
        max_abs.0, max_abs.1, max_abs.2);
    println!("Mean luminance: A {:.6}, B {:.6}, ratio {:.6}",
        sum_luma_a / n, sum_luma_b / n, (sum_luma_a / n) / (sum_luma_b / n));
    println!("Mean abs luminance diff: {:.6}", sum_luma_abs / n);
}
