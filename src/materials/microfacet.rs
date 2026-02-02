// Copyright @yucwang 2026

use crate::math::constants::{Float, Vector2f, Vector3f, PI};
use crate::math::spectrum::RGBSpectrum;

fn clamp01(v: Float) -> Float {
    if v < 0.0 {
        0.0
    } else if v > 1.0 {
        1.0
    } else {
        v
    }
}

pub fn ggx_d(cos_theta: Float, alpha: Float) -> Float {
    if cos_theta <= 0.0 {
        return 0.0;
    }
    let a = alpha.max(1e-4);
    let a2 = a * a;
    let cos2 = cos_theta * cos_theta;
    let denom = cos2 * (a2 - 1.0) + 1.0;
    a2 / (PI * denom * denom)
}

pub fn ggx_g1(cos_theta: Float, alpha: Float) -> Float {
    if cos_theta <= 0.0 {
        return 0.0;
    }
    let a = alpha.max(1e-4);
    let cos2 = cos_theta * cos_theta;
    let sin2 = (1.0 - cos2).max(0.0);
    if sin2 <= 0.0 {
        return 1.0;
    }
    let tan2 = sin2 / cos2.max(1e-6);
    let root = (1.0 + a * a * tan2).sqrt();
    2.0 / (1.0 + root)
}

pub fn ggx_g(cos_i: Float, cos_o: Float, alpha: Float) -> Float {
    ggx_g1(cos_i.abs(), alpha) * ggx_g1(cos_o.abs(), alpha)
}

pub fn pdf_ggx_vndf(wi: &Vector3f, m: &Vector3f, alpha: Float) -> Float {
    if wi.z <= 0.0 || m.z <= 0.0 {
        return 0.0;
    }
    let d = ggx_d(m.z, alpha);
    let g1 = ggx_g1(wi.z, alpha);
    let dot = wi.dot(m).abs();
    if wi.z.abs() <= 1e-6 {
        return 0.0;
    }
    d * g1 * dot / wi.z.abs()
}

pub fn sample_ggx(u: &Vector2f, alpha: Float) -> Vector3f {
    let a = alpha.max(1e-4);
    let u1 = clamp01(u.x);
    let u2 = clamp01(u.y.min(1.0 - 1e-6));
    let phi = 2.0 * PI * u1;
    let tan2 = (a * a) * u2 / (1.0 - u2).max(1e-6);
    let cos_theta = 1.0 / (1.0 + tan2).sqrt();
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    Vector3f::new(sin_theta * phi.cos(), sin_theta * phi.sin(), cos_theta)
}

pub fn sample_ggx_vndf(wi: &Vector3f, u: &Vector2f, alpha: Float) -> Vector3f {
    let a = alpha.max(1e-4);
    let wi = Vector3f::new(a * wi.x, a * wi.y, wi.z).normalize();

    let mut t1 = Vector3f::new(1.0, 0.0, 0.0);
    if wi.z < 0.9999 {
        t1 = Vector3f::new(0.0, 0.0, 1.0).cross(&wi).normalize();
    }
    let t2 = wi.cross(&t1);

    let u1 = clamp01(u.x);
    let u2 = clamp01(u.y);
    let r = u1.sqrt();
    let phi = 2.0 * PI * u2;
    let t1p = r * phi.cos();
    let mut t2p = r * phi.sin();
    let s = 0.5 * (1.0 + wi.z);
    t2p = (1.0 - s) * (1.0 - t1p * t1p).max(0.0).sqrt() + s * t2p;

    let nh = t1 * t1p + t2 * t2p + wi * (1.0 - t1p * t1p - t2p * t2p).max(0.0).sqrt();
    Vector3f::new(a * nh.x, a * nh.y, nh.z.max(0.0)).normalize()
}

pub fn reflect(wi: &Vector3f, m: &Vector3f) -> Vector3f {
    2.0 * wi.dot(m) * m - wi
}

pub fn refract(wi: &Vector3f, m: &Vector3f, eta: Float) -> Option<Vector3f> {
    let cos_i = wi.dot(m).max(-1.0).min(1.0);
    let sin2_i = (1.0 - cos_i * cos_i).max(0.0);
    let sin2_t = eta * eta * sin2_i;
    if sin2_t >= 1.0 {
        return None;
    }
    let cos_t = (1.0 - sin2_t).sqrt();
    let wt = -eta * wi + (eta * cos_i - cos_t) * m;
    Some(wt)
}

pub fn fresnel_dielectric(cos_i: Float, eta_i: Float, eta_t: Float) -> Float {
    let mut cos_i = cos_i.max(-1.0).min(1.0);
    let entering = cos_i > 0.0;
    let (eta_i, eta_t) = if entering { (eta_i, eta_t) } else { (eta_t, eta_i) };
    cos_i = cos_i.abs();

    let sin2_i = (1.0 - cos_i * cos_i).max(0.0);
    let eta = eta_i / eta_t;
    let sin2_t = eta * eta * sin2_i;
    if sin2_t >= 1.0 {
        return 1.0;
    }
    let cos_t = (1.0 - sin2_t).sqrt();
    let r_parl = (eta_t * cos_i - eta_i * cos_t) / (eta_t * cos_i + eta_i * cos_t);
    let r_perp = (eta_i * cos_i - eta_t * cos_t) / (eta_i * cos_i + eta_t * cos_t);
    0.5 * (r_parl * r_parl + r_perp * r_perp)
}

pub fn fresnel_schlick(f0: RGBSpectrum, cos_theta: Float) -> RGBSpectrum {
    let cos_theta = cos_theta.max(0.0).min(1.0);
    let one_minus = (1.0 - cos_theta).powi(5);
    f0 + (RGBSpectrum::new(1.0, 1.0, 1.0) - f0) * one_minus
}
