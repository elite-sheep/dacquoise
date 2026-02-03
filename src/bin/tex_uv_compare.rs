use dacquoise::math::constants::Vector2f;
use dacquoise::textures::image::ImageTexture;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 4 {
        eprintln!("Usage: {} <texture.exr/png/jpg> <u> <v>", args[0]);
        std::process::exit(1);
    }

    let path = &args[1];
    let u: f32 = args[2].parse().unwrap_or(0.0);
    let v: f32 = args[3].parse().unwrap_or(0.0);

    let tex = ImageTexture::from_file(path)
        .unwrap_or_else(|e| panic!("failed to load texture {}: {}", path, e));

    let uv = Vector2f::new(u, v);
    let uv_unflipped = Vector2f::new(u, 1.0 - v);
    let c = tex.eval(uv);
    let c_unflipped = tex.eval(uv_unflipped);

    println!("texture: {}", path);
    println!("uv = ({:.6}, {:.6})", u, v);
    println!("eval (current, v flipped internally): R {:.6}, G {:.6}, B {:.6}", c[0], c[1], c[2]);
    println!("eval (simulate no v flip):          R {:.6}, G {:.6}, B {:.6}", c_unflipped[0], c_unflipped[1], c_unflipped[2]);
}
use dacquoise::core::texture::Texture;
