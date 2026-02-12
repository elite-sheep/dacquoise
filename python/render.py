#!/usr/bin/env python3
import argparse
import os
import sys

import numpy as np
import pyexr

try:
    import dacquoise
except Exception as exc:
    print(f"Failed to import dacquoise module: {exc}", file=sys.stderr)
    print("Build the Python extension first, e.g.:", file=sys.stderr)
    print("  maturin develop -m Cargo.toml --features python", file=sys.stderr)
    sys.exit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a scene with dacquoise.")
    parser.add_argument(
        "scene",
        nargs="?",
        default=os.path.join("scenes", "cbox", "cbox.xml"),
        help="Path to the scene XML (default: scenes/cbox/cbox.xml).",
    )
    parser.add_argument("--spp", type=int, default=8, help="Samples per pixel.")
    parser.add_argument("--max-depth", type=int, default=4, help="Max path depth.")
    parser.add_argument(
        "--progress",
        action="store_true",
        help="Force a progress bar even when stdout/stderr is not a TTY.",
    )
    parser.add_argument(
        "--out",
        default=os.path.join("output", "cbox.exr"),
        help="Output EXR path.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.progress:
        os.environ["DACQUOISE_PROGRESS"] = "1"

    scene = dacquoise.load_scene(args.scene)
    matrix = dacquoise.render(scene, args.spp, args.max_depth)
    matrix = np.asarray(matrix)
    height, width_times_three = matrix.shape

    if width_times_three % 3 != 0:
        print(
            f"Unexpected matrix shape {matrix.shape}; expected width * 3 columns.",
            file=sys.stderr,
        )
        return 1

    image = matrix.reshape(height, width_times_three // 3, 3)

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    pyexr.write(args.out, image.astype(np.float32, copy=False))
    print(f"Wrote {args.out} with shape {image.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
