// Copyright @yucwang 2023

use super::constants::{ INV_PI, PI, Float, Vector2f, Vector3f };

fn sample_uniform_hemisphere(u: &Vector2f) -> Vector3f {
    let z: Float = u.x;
    let r: Float = (1. - z * z).sqrt();
    let phi: Float = 2. * PI * u.y;

    return Vector3f::new(r * phi.cos(), r * phi.sin(), z)
}

fn sample_uniform_hemisphere_pdf() -> Float {
    return INV_PI / 2.
}

fn invert_uniform_hemisphere(v: &Vector3f) -> Vector2f {
    let mut phi = v.y.atan2(v.x);
    if phi < 0.0 {
        phi += 2. * PI;
    }

    return Vector2f::new(v.z, phi / (2. * PI))
}

fn sample_uniform_disk_concentric(u: &Vector2f) -> Vector2f {
    let r1: Float = 2.0 * u.x - 1.0;
    let r2: Float = 2.0 * u.y - 1.0;

    let phi: Float;
    let r:   Float;

    if r1 == 0. && r2 == 0. {
        r = 0.0;
        phi = 0.0;
    } else if r1 * r1 > r2 * r2 {
        r = r1;
        phi = (PI / 4.0) * (r2 / r1);
    } else {
        r = r2;
        phi = (PI / 2.0) - (r1 / r2) * (PI / 4.0);
    }

    let (sin_phi, cos_phi) = phi.sin_cos();

    return Vector2f::new(r * cos_phi, r * sin_phi)
}

fn sample_cosine_hemisphere(u: &Vector2f) -> Vector3f {
    let p = sample_uniform_disk_concentric(&u);
    let z = (1. - p.x * p.x - p.y * p.y).sqrt();

    return Vector3f::new(p.x, p.y, z)
}

fn sample_cosine_hemisphere_pdf(cos_theta: Float) -> Float {
    return cos_theta * INV_PI;
}
