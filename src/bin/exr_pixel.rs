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
    if args.len() < 5 {
        eprintln!("Usage: {} <image_a.exr> <image_b.exr> <x> <y>", args[0]);
        std::process::exit(1);
    }

    let x: usize = args[3].parse().unwrap_or(0);
    let y: usize = args[4].parse().unwrap_or(0);

    let a = read_rgba(&args[1]);
    let b = read_rgba(&args[2]);

    if a.width != b.width || a.height != b.height {
        eprintln!("Size mismatch: {}x{} vs {}x{}", a.width, a.height, b.width, b.height);
        std::process::exit(2);
    }
    if x >= a.width || y >= a.height {
        eprintln!("Out of bounds: ({}, {}) for size {}x{}", x, y, a.width, a.height);
        std::process::exit(3);
    }

    let idx = y * a.width + x;
    let (ar, ag, ab, aa) = a.data[idx];
    let (br, bg, bb, ba) = b.data[idx];

    println!("Pixel ({}, {})", x, y);
    println!("A: R {:.6}, G {:.6}, B {:.6}, A {:.6}", ar, ag, ab, aa);
    println!("B: R {:.6}, G {:.6}, B {:.6}, A {:.6}", br, bg, bb, ba);
    println!("Diff: R {:.6}, G {:.6}, B {:.6}, A {:.6}", (ar - br), (ag - bg), (ab - bb), (aa - ba));
}
