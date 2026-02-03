// Copyright @yucwang 2026

use crate::math::constants::{Float, Vector2f, Vector3f, PI};
use crate::math::spectrum::RGBSpectrum;
use crate::math::warp::sample_uniform_disk_concentric;

fn clamp01(v: Float) -> Float {
    if v < 0.0 {
        0.0
    } else if v > 1.0 {
        1.0
    } else {
        v
    }
}

fn safe_sqrt(v: Float) -> Float {
    v.max(0.0).sqrt()
}

fn erf_approx(x: Float) -> Float {
    // Abramowitz-Stegun 7.1.26
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let y = 1.0 - (((((1.061405429 * t - 1.453152027) * t + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t) * (-x * x).exp();
    sign * y
}

fn erfinv_approx(x: Float) -> Float {
    // Winitzki approximation with one Newton refinement
    let x = x.max(-0.999_999).min(0.999_999);
    let a = 0.147;
    let ln = (1.0 - x * x).ln();
    let term = 2.0 / (PI * a) + ln / 2.0;
    let inside = term * term - ln / a;
    let mut y = (inside.sqrt() - term).sqrt();
    if x < 0.0 {
        y = -y;
    }
    // One Newton iteration to improve accuracy
    let err = erf_approx(y) - x;
    let deriv = (2.0 / PI).sqrt() * (-y * y).exp();
    y - err / deriv
}

fn sincos_phi(v: &Vector3f) -> (Float, Float) {
    let len = (v.x * v.x + v.y * v.y).sqrt();
    if len > 0.0 {
        (v.y / len, v.x / len)
    } else {
        (0.0, 1.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MicrofacetType {
    Beckmann,
    GGX,
}

#[derive(Clone, Copy, Debug)]
pub struct MicrofacetDistribution {
    m_type: MicrofacetType,
    alpha_u: Float,
    alpha_v: Float,
    sample_visible: bool,
}

impl MicrofacetDistribution {
    pub fn new(m_type: MicrofacetType, alpha_u: Float, alpha_v: Float, sample_visible: bool) -> Self {
        let alpha_u = alpha_u.max(1e-4);
        let alpha_v = alpha_v.max(1e-4);
        Self {
            m_type,
            alpha_u,
            alpha_v,
            sample_visible,
        }
    }

    pub fn eval(&self, m: &Vector3f) -> Float {
        let cos_theta = m.z;
        if cos_theta <= 0.0 {
            return 0.0;
        }
        let alpha_uv = self.alpha_u * self.alpha_v;
        let cos_theta_2 = cos_theta * cos_theta;
        let result = match self.m_type {
            MicrofacetType::Beckmann => {
                let term = (m.x / self.alpha_u) * (m.x / self.alpha_u)
                    + (m.y / self.alpha_v) * (m.y / self.alpha_v);
                (-term / cos_theta_2).exp() / (PI * alpha_uv * cos_theta_2 * cos_theta_2)
            }
            MicrofacetType::GGX => {
                let term = (m.x / self.alpha_u) * (m.x / self.alpha_u)
                    + (m.y / self.alpha_v) * (m.y / self.alpha_v)
                    + m.z * m.z;
                1.0 / (PI * alpha_uv * term * term)
            }
        };

        if result * cos_theta > 1e-20 { result } else { 0.0 }
    }

    pub fn smith_g1(&self, v: &Vector3f, m: &Vector3f) -> Float {
        let xy_alpha_2 = (self.alpha_u * v.x) * (self.alpha_u * v.x)
            + (self.alpha_v * v.y) * (self.alpha_v * v.y);
        let v_z2 = v.z * v.z;
        if v_z2 <= 0.0 {
            return 0.0;
        }
        let tan_theta_alpha_2 = xy_alpha_2 / v_z2;
        let mut result = match self.m_type {
            MicrofacetType::Beckmann => {
                let a = 1.0 / tan_theta_alpha_2.sqrt();
                if a >= 1.6 {
                    1.0
                } else {
                    let a2 = a * a;
                    (3.535 * a + 2.181 * a2) / (1.0 + 2.276 * a + 2.577 * a2)
                }
            }
            MicrofacetType::GGX => 2.0 / (1.0 + (1.0 + tan_theta_alpha_2).sqrt()),
        };

        if xy_alpha_2 == 0.0 {
            result = 1.0;
        }
        if v.dot(m) * v.z <= 0.0 {
            return 0.0;
        }
        result
    }

    pub fn g(&self, wi: &Vector3f, wo: &Vector3f, m: &Vector3f) -> Float {
        self.smith_g1(wi, m) * self.smith_g1(wo, m)
    }

    pub fn pdf(&self, wi: &Vector3f, m: &Vector3f) -> Float {
        let mut result = self.eval(m);
        if self.sample_visible {
            if wi.z.abs() <= 1e-6 {
                return 0.0;
            }
            result *= self.smith_g1(wi, m) * wi.dot(m).abs() / wi.z.abs();
        } else {
            result *= m.z.max(0.0);
        }
        result
    }

    pub fn sample(&self, wi: &Vector3f, sample: &Vector2f) -> (Vector3f, Float) {
        if !self.sample_visible {
            return self.sample_non_visible(sample);
        }

        // Visible normal sampling
        let wi_p = Vector3f::new(self.alpha_u * wi.x, self.alpha_v * wi.y, wi.z).normalize();
        let (sin_phi, cos_phi) = sincos_phi(&wi_p);
        let cos_theta = wi_p.z;
        if !cos_theta.is_finite() || cos_theta <= 1e-6 {
            return self.sample_non_visible(sample);
        }
        let mut slope = self.sample_visible_11(cos_theta, sample);

        // rotate & unstretch
        let slope_x = cos_phi * slope.x - sin_phi * slope.y;
        let slope_y = sin_phi * slope.x + cos_phi * slope.y;
        slope.x = slope_x * self.alpha_u;
        slope.y = slope_y * self.alpha_v;

        let m = Vector3f::new(-slope.x, -slope.y, 1.0).normalize();
        let pdf = self.eval(&m) * self.smith_g1(wi, &m) * wi.dot(&m).abs() / wi.z.abs().max(1e-6);
        (m, pdf)
    }

    fn sample_non_visible(&self, sample: &Vector2f) -> (Vector3f, Float) {
        let (sin_phi, cos_phi, alpha_2) = if (self.alpha_u - self.alpha_v).abs() < 1e-6 {
            let phi = 2.0 * PI * sample.y;
            (phi.sin(), phi.cos(), self.alpha_u * self.alpha_u)
        } else {
            let ratio = self.alpha_v / self.alpha_u;
            let tmp = ratio * (2.0 * PI * sample.y).tan();
            let mut cos_phi = 1.0 / (tmp * tmp + 1.0).sqrt();
            if (sample.y - 0.5).abs() > 0.25 {
                cos_phi = -cos_phi;
            }
            let sin_phi = cos_phi * tmp;
            let alpha_2 = 1.0 / ((cos_phi / self.alpha_u) * (cos_phi / self.alpha_u)
                + (sin_phi / self.alpha_v) * (sin_phi / self.alpha_v));
            (sin_phi, cos_phi, alpha_2)
        };

        let (cos_theta, pdf) = match self.m_type {
            MicrofacetType::Beckmann => {
                let cos_theta = 1.0 / (1.0 - alpha_2 * (1.0 - sample.x).ln()).sqrt();
                let cos_theta_2 = cos_theta * cos_theta;
                let cos_theta_3 = (cos_theta_2 * cos_theta).max(1e-20);
                let pdf = (1.0 - sample.x) / (PI * self.alpha_u * self.alpha_v * cos_theta_3);
                (cos_theta, pdf)
            }
            MicrofacetType::GGX => {
                let tan_theta_m_2 = alpha_2 * sample.x / (1.0 - sample.x).max(1e-6);
                let cos_theta = 1.0 / (1.0 + tan_theta_m_2).sqrt();
                let cos_theta_2 = cos_theta * cos_theta;
                let temp = 1.0 + tan_theta_m_2 / alpha_2;
                let cos_theta_3 = (cos_theta_2 * cos_theta).max(1e-20);
                let pdf = 1.0 / (PI * self.alpha_u * self.alpha_v * cos_theta_3 * temp * temp);
                (cos_theta, pdf)
            }
        };

        let sin_theta = safe_sqrt(1.0 - cos_theta * cos_theta);
        let m = Vector3f::new(cos_phi * sin_theta, sin_phi * sin_theta, cos_theta);
        (m, pdf)
    }

    fn sample_visible_11(&self, cos_theta_i: Float, sample: &Vector2f) -> Vector2f {
        if let MicrofacetType::Beckmann = self.m_type {
            let cos_theta_i = cos_theta_i.max(1e-6);
            let tan_theta_i = safe_sqrt(1.0 - cos_theta_i * cos_theta_i) / cos_theta_i;
            let cot_theta_i = 1.0 / tan_theta_i.max(1e-6);
            let maxval = erf_approx(cot_theta_i);

            let mut u1 = sample.x.max(1e-6).min(1.0 - 1e-6);
            let mut x = maxval - (maxval + 1.0) * erf_approx((-u1.ln()).sqrt());
            u1 *= 1.0 + maxval + (1.0 / PI).sqrt() * tan_theta_i * (-cot_theta_i * cot_theta_i).exp();

            for _ in 0..3 {
                let slope = erfinv_approx(x);
                let value = 1.0 + x + (1.0 / PI).sqrt() * tan_theta_i * (-slope * slope).exp() - u1;
                let derivative = 1.0 - slope * tan_theta_i;
                x -= value / derivative.max(1e-6);
            }

            Vector2f::new(erfinv_approx(x), erfinv_approx(2.0 * sample.y - 1.0))
        } else {
            let mut p = sample_uniform_disk_concentric(sample);
            let s = 0.5 * (1.0 + cos_theta_i);
            let y = safe_sqrt(1.0 - p.x * p.x);
            p.y = y + s * (p.y - y);

            let x = p.x;
            let y = p.y;
            let z = safe_sqrt(1.0 - x * x - y * y);
            let sin_theta_i = safe_sqrt(1.0 - cos_theta_i * cos_theta_i);
            let norm = 1.0 / (sin_theta_i * y + cos_theta_i * z).max(1e-6);
            Vector2f::new((cos_theta_i * y - sin_theta_i * z) * norm, x * norm)
        }
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

pub fn fresnel_dielectric_full(cos_i: Float, eta: Float) -> (Float, Float, Float, Float) {
    let outside = cos_i >= 0.0;
    let rcp_eta = 1.0 / eta;
    let eta_it = if outside { eta } else { rcp_eta };
    let eta_ti = if outside { rcp_eta } else { eta };

    let cos_i2 = cos_i * cos_i;
    let cos_t2 = 1.0 - (1.0 - cos_i2) * eta_ti * eta_ti;
    let cos_i_abs = cos_i.abs();
    let cos_t_abs = cos_t2.max(0.0).sqrt();

    let index_matched = (eta - 1.0).abs() < 1e-6;
    let special_case = index_matched || cos_i_abs == 0.0;
    let r_sc = if index_matched { 0.0 } else { 1.0 };

    let a_s = (eta_it * cos_t_abs - cos_i_abs) / (eta_it * cos_t_abs + cos_i_abs).max(1e-6);
    let a_p = (eta_it * cos_i_abs - cos_t_abs) / (eta_it * cos_i_abs + cos_t_abs).max(1e-6);
    let mut r = 0.5 * (a_s * a_s + a_p * a_p);
    if special_case {
        r = r_sc;
    }

    let cos_t = if outside { -cos_t_abs } else { cos_t_abs };
    (r, cos_t, eta_it, eta_ti)
}

pub fn refract_with_cos(wi: &Vector3f, m: &Vector3f, cos_t: Float, eta_ti: Float) -> Vector3f {
    m * (wi.dot(m) * eta_ti + cos_t) - wi * eta_ti
}

pub fn fresnel_schlick(f0: RGBSpectrum, cos_theta: Float) -> RGBSpectrum {
    let cos_theta = cos_theta.max(0.0).min(1.0);
    let one_minus = (1.0 - cos_theta).powi(5);
    f0 + (RGBSpectrum::new(1.0, 1.0, 1.0) - f0) * one_minus
}

pub fn fresnel_conductor(cos_i: Float, eta: RGBSpectrum, k: RGBSpectrum) -> RGBSpectrum {
    let cos_i = cos_i.abs().min(1.0);
    let cos2 = cos_i * cos_i;
    let sin2 = (1.0 - cos2).max(0.0);

    fn channel(cos_i: Float, cos2: Float, sin2: Float, eta: Float, k: Float) -> Float {
        let eta2 = eta * eta;
        let k2 = k * k;
        let t0 = eta2 - k2 - sin2;
        let a2plusb2 = (t0 * t0 + 4.0 * eta2 * k2).max(0.0).sqrt();
        let t1 = a2plusb2 + cos2;
        let a = (0.5 * (a2plusb2 + t0)).max(0.0).sqrt();
        let t2 = 2.0 * cos_i * a;
        let rs = (t1 - t2) / (t1 + t2).max(1e-6);
        let t3 = cos2 * a2plusb2 + sin2 * sin2;
        let t4 = t2 * sin2;
        let rp = rs * (t3 - t4) / (t3 + t4).max(1e-6);
        0.5 * (rp + rs)
    }

    RGBSpectrum::new(
        channel(cos_i, cos2, sin2, eta[0], k[0]),
        channel(cos_i, cos2, sin2, eta[1], k[1]),
        channel(cos_i, cos2, sin2, eta[2], k[2]),
    )
}
