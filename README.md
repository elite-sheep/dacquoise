# Dacquoise ![Building Status](https://github.com/Aaron19960821/Dacquoise/workflows/build_check/badge.svg)

![Veach Ajar teaser](assets/veach-ajar.png)

**Dacquoise** is a compact offline renderer focused on physically-based path tracing in Rust.  
It targets clarity and extensibility for research and learning, while remaining practical for rendering real scenes.  

## Features

Emitters:
- Area
- Directional
- Environment map

Materials:
- Lambertian diffuse
- Rough conductor (GGX/Beckmann microfacet)
- Rough dielectric (GGX/Beckmann microfacet)
- Blend BSDF
- Two-sided wrapper
- Null BSDF

Integrators:
- Path tracer
- NerF-Like Ray marching

Media:
- Homogeneous medium
- Heterogeneous medium

Volumes:
- Constant volume (albedo/sigma_t)
- Grid volume (.vol/.vdb)

## Build

Prerequisites:
- Rust toolchain (stable), install via `rustup`

Build:
```
cargo build
```

Run tests:
```
cargo test
```

Conda setup (optional):
```
conda create -n dacquoise python=3.11
conda activate dacquoise
python -m pip install -r requirements.txt
```

Python library (optional):
```
python -m pip install maturin
maturin develop -m Cargo.toml --features python --release
```

Render a scene:
```
cargo run --release --bin dacquoise -- scenes/cbox/cbox.xml output/cbox.exr --spp 128 --max-depth 3
```

## Rendering

Use the CLI to render a scene file:
```
cargo run --release --bin dacquoise -- <scene.xml> <output.exr> --spp <samples> --max-depth <depth>
```

Python rendering:
```
python python/render.py <scene.xml> --spp <samples> --max-depth <depth> --out <output.exr> --progress
```

Python texture access (requires `--features python` build):
```
import dacquoise as dq

# Returns a NumPy matrix of shape (height, width * 3) in RGB order.
raw = dq.texture_raw("assets/texture.exr", True)
height, columns = raw.shape
width = columns // 3
```

Sample scenes download [link](https://drive.google.com/drive/folders/1CVsNjM_GvmVP8oyHzRgteWlzTVGmjJnl?usp=sharing).

## Developer Notes

Texture data layout:
- `ImageTexture` stores raw RGB data in a `MatrixXF` sized `height x (width * 3)`.
- Columns are `x * 3 + channel` (R, G, B) and rows are `y`.

Grid volume data layout:
- `GridVolume` stores raw values in a `MatrixXF` sized `(zres * yres) x (xres * channels)`.
- Rows are `z * yres + y`; columns are `x * channels + channel`.

Scene raw data dictionary:
- `Scene` keeps raw data pointers in a dictionary keyed by IDs.
- Diffuse image textures use `<bsdf_id>.reflectance.data`.
- Grid volumes use `<volume_id>.data`.
- `RawDataView` points to nalgebra's column-major buffer; use `rows`/`cols` for shape.

## Debug Tools

We keep small debug utilities in `src/bin` for inspecting pixels, EXRs, textures, and paths. Run any of them with `cargo run --bin <tool> -- ...`.

Pixel/path inspection:
- Trace a single pixel path with per-bounce details.
```
cargo run --bin trace_pixel_path -- <scene.xml> <x> <y> [--max-depth N] [--seed N] [--camera N]
```
- Render a single pixel with the path tracer (quick spot checks).
```
cargo run --bin render_pixel -- <scene.xml> <x> <y> [--spp N] [--max-depth N] [--seed N] [--camera N]
```

EXR analysis:
- Basic stats (min/max/mean, NaN/Inf counts).
```
cargo run --bin exr_stats -- <image.exr>
```
- Full-image diff (mean/max absolute RGB, luminance stats).
```
cargo run --bin exr_diff -- <image_a.exr> <image_b.exr>
```
- Region diff for a bounding box.
```
cargo run --bin exr_region_diff -- <image_a.exr> <image_b.exr> <x0> <y0> <x1> <y1>
```
- Per-pixel diff between two images.
```
cargo run --bin exr_pixel -- <image_a.exr> <image_b.exr> <x> <y>
```

Texture debugging:
- Compare texture eval at (u, v) vs (u, 1-v) to check V-flip issues.
```
cargo run --bin tex_uv_compare -- <texture.exr/png/jpg> <u> <v>
```

Geometry/backface fixer:
- Flip backfaced PLY faces for black primary-hit pixels (threshold optional).
```
cargo run --bin fix_black_backfaces -- <scene.xml> <image.exr> [threshold]
```

## Gallery

![Cornell Box](assets/cbox.png)
![Dragon](assets/dragon.png)
![Lego](assets/lego.png)
![Rover](assets/rover.png)
![Veach Ajar](assets/veach-ajar.png)
