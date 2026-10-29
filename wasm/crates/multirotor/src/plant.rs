//! The rotor plant: `stepRotorPlant` from `src/rotorPlant.ts`, operation for
//! operation. Each rotor's speed follows its command with one time constant
//! (an ideal motor and ESC); blade element theory with momentum inflow gives
//! its thrust, H-force, torque and hub moment, with ground effect and the
//! vortex ring state; the rotors' reaction and gyroscopic moments go on the
//! body. The layouts are `PLANT` in `src/layout.ts`.

use core::f64::consts::PI;

pub mod config {
    pub const DT: usize = 0;
    pub const DECAY: usize = 1;
    pub const AVERAGE: usize = 2;
    pub const OMEGA_MAX: usize = 3;
    pub const RADIUS: usize = 4;
    pub const SIGMA_A_HALF: usize = 5;
    pub const THETA_0: usize = 6;
    pub const THETA_TW: usize = 7;
    pub const SIGMA_CD0: usize = 8;
    pub const HUB_GAIN: usize = 9;
    pub const ROTOR_INERTIA: usize = 10;
    pub const VRS_AMPLITUDE: usize = 11;
    pub const NOISE_DECAY: usize = 12;
    pub const NOISE_GAIN: usize = 13;
    pub const FOLD_MU: usize = 14;
    pub const GE_SPACING: usize = 15;
    pub const GE_BODY: usize = 16;
    pub const GE_BODY_GAIN: usize = 17;
    pub const MIN_TIP_SPEED: usize = 18;
    pub const GLOBALS: usize = 19;
    pub const X: usize = 0;
    pub const Y: usize = 1;
    pub const Z: usize = 2;
    pub const SPIN: usize = 3;
    pub const STRIDE: usize = 4;
}

pub mod input {
    pub const U: usize = 0;
    pub const V: usize = 1;
    pub const W: usize = 2;
    pub const P: usize = 3;
    pub const Q: usize = 4;
    pub const R: usize = 5;
    pub const RHO: usize = 6;
    pub const C31: usize = 7;
    pub const C32: usize = 8;
    pub const C33: usize = 9;
    /// NaN for no ground.
    pub const AGL: usize = 10;
    pub const GLOBALS: usize = 11;
    pub const OMEGA_CMD: usize = 0;
    pub const STRIDE: usize = 1;
}

pub mod state {
    pub const RNG: usize = 0;
    pub const GLOBALS: usize = 1;
    pub const OMEGA: usize = 0;
    pub const NOISE: usize = 1;
    pub const STRIDE: usize = 2;
}

pub mod output {
    pub const FX: usize = 0;
    pub const FY: usize = 1;
    pub const FZ: usize = 2;
    pub const L: usize = 3;
    pub const M: usize = 4;
    pub const N: usize = 5;
    pub const GLOBALS: usize = 6;
    pub const THRUST: usize = 0;
    pub const H_FORCE: usize = 1;
    pub const TORQUE: usize = 2;
    pub const OMEGA: usize = 3;
    pub const OMEGA_MEAN: usize = 4;
    pub const INDUCED: usize = 5;
    pub const GROUND_EFFECT: usize = 6;
    pub const VRS: usize = 7;
    pub const STRIDE: usize = 8;
}

pub const MAX_ROTORS: usize = 8;

const ONE_THIRD: f64 = 1.0 / 3.0;
const INFLOW_STEPS: usize = 20;
const NEWTON_STEPS: usize = 8;
const BISECTIONS: usize = 48;
const MIN_CT: f64 = 1e-14;
const K1: f64 = -1.112;
const K2: f64 = -1.372;
const K3: f64 = -1.718;
const K4: f64 = -0.655;

/// xorshift32.
#[inline]
pub fn xorshift(value: u32) -> u32 {
    let mut x = value;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    x
}

/// Induced velocity over hover's in pure axial descent.
pub fn axial_inflow(x: f64) -> f64 {
    if x <= -2.0 {
        return -0.5 * x - (0.25 * x * x - 1.0).sqrt();
    }
    1.0 + x * (K1 + x * (K2 + x * (K3 + x * K4)))
}

/// Glauert's w for x >= 0, by Newton from an upper bound.
pub fn glauert_climb(x: f64, m: f64) -> f64 {
    let m2 = m * m;
    let mut w = 1.0 / (m2 + x * x).sqrt();
    if !(w < 1.0) {
        w = 1.0;
    }
    for _ in 0..NEWTON_STEPS {
        let s = x + w;
        let a = m2 + s * s;
        w = w - (w * w * a - 1.0) / (2.0 * w * a + 2.0 * w * w * s);
    }
    w
}

/// Glauert's w for x < 0 at or above the fold, by bisection on [0, 1/m].
pub fn glauert_descent(x: f64, m: f64) -> f64 {
    let m2 = m * m;
    let mut lo = 0.0;
    let mut hi = 1.0 / m;
    for _ in 0..BISECTIONS {
        let w = 0.5 * (lo + hi);
        let s = x + w;
        if w * w * (m2 + s * s) - 1.0 > 0.0 {
            hi = w;
        } else {
            lo = w;
        }
    }
    0.5 * (lo + hi)
}

pub fn inflow_ratio(x: f64, m: f64, fold: f64) -> f64 {
    if x >= 0.0 {
        return glauert_climb(x, m);
    }
    if m >= fold {
        return glauert_descent(x, m);
    }
    let t = 1.0 - m / fold;
    let weight = t * t;
    let m2 = m * m;
    let level = (0.5 * ((m2 * m2 + 4.0).sqrt() - m2)).sqrt();
    let f2 = fold * fold;
    let fold_level = (0.5 * ((f2 * f2 + 4.0).sqrt() - f2)).sqrt();
    level + weight * (axial_inflow(x) - 1.0) + (1.0 - weight) * (glauert_descent(x, fold) - fold_level)
}

pub fn vrs_intensity(x: f64, m: f64, fold: f64) -> f64 {
    if !(x < 0.0 && x > -2.0 && m < fold) {
        return 0.0;
    }
    let t = 1.0 - m / fold;
    let r = x * (x + 2.0);
    t * t * r * r
}

#[allow(clippy::too_many_arguments)]
pub fn ground_effect(agl: f64, c31: f64, c32: f64, c33: f64, x: f64, y: f64, z: f64, radius: f64, spacing: f64, body: f64, body_gain: f64) -> f64 {
    if agl.is_nan() || !(c33 > 0.5) {
        return 1.0;
    }
    let mut h = (agl - (c31 * x + c32 * y + c33 * z)) / c33;
    let lowest = 0.5 * radius;
    if !(h > lowest) {
        h = lowest;
    }
    let r2 = radius * radius;
    let q = radius / (4.0 * h);
    let h4 = 4.0 * h * h;
    let s1 = spacing * spacing + h4;
    let s2 = 2.0 * spacing * spacing + h4;
    let s3 = body * body + h4;
    let k = 1.0 - q * q - r2 * h / (s1 * s1.sqrt()) - 0.5 * r2 * h / (s2 * s2.sqrt()) - 2.0 * body_gain * r2 * h / (s3 * s3.sqrt());
    k.clamp(0.5, 1.0)
}

#[inline]
fn thrust_coefficient(sigma_a_half: f64, pitch: f64, lambda: f64) -> f64 {
    let ct = sigma_a_half * (pitch - 0.5 * lambda);
    if ct > 0.0 { ct } else { 0.0 }
}

#[allow(clippy::too_many_arguments)]
fn induced_for(li: f64, lc: f64, mu: f64, ge: f64, swing: f64, fold: f64, sigma_a_half: f64, pitch: f64) -> f64 {
    let ct = thrust_coefficient(sigma_a_half, pitch, lc + li);
    if !(ct > MIN_CT) {
        return 0.0;
    }
    let lh = (0.5 * ct).sqrt();
    let x = lc / lh;
    let m = mu / lh;
    let mut f = 1.0 + swing * vrs_intensity(x, m, fold);
    if f < 0.2 {
        f = 0.2;
    }
    ge * lh * inflow_ratio(x, m, fold) * f
}

pub fn pitch_term(theta0: f64, theta_tw: f64, mu: f64) -> f64 {
    let mu2 = mu * mu;
    theta0 * (ONE_THIRD + 0.5 * mu2) + 0.25 * theta_tw * (1.0 + mu2)
}

/// Illinois regula falsi for the induced inflow ratio.
#[allow(clippy::too_many_arguments)]
pub fn solve_inflow(lc: f64, mu: f64, ge: f64, swing: f64, fold: f64, sigma_a_half: f64, theta0: f64, theta_tw: f64) -> f64 {
    let pitch = pitch_term(theta0, theta_tw, mu);
    let mut hi = 2.0 * pitch - lc;
    if !(hi > 0.0) {
        return 0.0;
    }
    let mut lo = 0.0;
    let mut flo = -induced_for(0.0, lc, mu, ge, swing, fold, sigma_a_half, pitch);
    if !(flo < 0.0) {
        return 0.0;
    }
    let mut fhi = hi - induced_for(hi, lc, mu, ge, swing, fold, sigma_a_half, pitch);
    if !(fhi > 0.0) {
        return hi;
    }
    let mut side = 0;
    let mut li = 0.0;
    for _ in 0..INFLOW_STEPS {
        li = (lo * fhi - hi * flo) / (fhi - flo);
        let f = li - induced_for(li, lc, mu, ge, swing, fold, sigma_a_half, pitch);
        if f > 0.0 {
            hi = li;
            fhi = f;
            if side == 1 {
                flo *= 0.5;
            }
            side = 1;
        } else if f < 0.0 {
            lo = li;
            flo = f;
            if side == -1 {
                fhi *= 0.5;
            }
            side = -1;
        } else {
            break;
        }
    }
    li
}

/// Rotor count of buffers that fit together, or `None`.
pub fn rotors(config: usize, input: usize, state: usize, output: usize) -> Option<usize> {
    if config <= config::GLOBALS || !(config - config::GLOBALS).is_multiple_of(config::STRIDE) {
        return None;
    }
    let n = (config - config::GLOBALS) / config::STRIDE;
    let fits = n <= MAX_ROTORS
        && input == input::GLOBALS + n * input::STRIDE
        && state == state::GLOBALS + n * state::STRIDE
        && output == output::GLOBALS + n * output::STRIDE;
    fits.then_some(n)
}

/// `checkPlant`'s value checks: false for bad input, before anything is written.
pub fn valid(config: &[f64], input: &[f64], state: &[f64]) -> bool {
    let rng = state[state::RNG];
    config.iter().all(|v| v.is_finite())
        && input.iter().enumerate().all(|(i, v)| if i == input::AGL { !v.is_infinite() } else { v.is_finite() })
        && state.iter().all(|v| v.is_finite())
        && (1.0..4_294_967_296.0).contains(&rng)
        && rng.floor() == rng
        && config[config::DT] > 0.0
        && input[input::RHO] >= 0.0
}

/// One step on buffers [`rotors`] and [`valid`] accepted.
pub fn step(config: &[f64], input: &[f64], state: &mut [f64], output: &mut [f64], n: usize) {
    use self::{config as c, input as i, output as o, state as s};
    let dt = config[c::DT];
    let decay = config[c::DECAY];
    let average = config[c::AVERAGE];
    let omega_max = config[c::OMEGA_MAX];
    let radius = config[c::RADIUS];
    let sigma_a_half = config[c::SIGMA_A_HALF];
    let theta0 = config[c::THETA_0];
    let theta_tw = config[c::THETA_TW];
    let sigma_cd0 = config[c::SIGMA_CD0];
    let hub_gain = config[c::HUB_GAIN];
    let inertia = config[c::ROTOR_INERTIA];
    let amplitude = config[c::VRS_AMPLITUDE];
    let noise_decay = config[c::NOISE_DECAY];
    let noise_gain = config[c::NOISE_GAIN];
    let fold = config[c::FOLD_MU];
    let spacing = config[c::GE_SPACING];
    let body = config[c::GE_BODY];
    let body_gain = config[c::GE_BODY_GAIN];
    let min_tip = config[c::MIN_TIP_SPEED];
    let (u, v, w) = (input[i::U], input[i::V], input[i::W]);
    let (p, q, r) = (input[i::P], input[i::Q], input[i::R]);
    let rho = input[i::RHO];
    let (c31, c32, c33) = (input[i::C31], input[i::C32], input[i::C33]);
    let agl = input[i::AGL];
    let area = rho * PI * radius * radius;
    let (mut fx, mut fy, mut fz, mut l, mut m, mut yaw, mut momentum) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let mut rng = state[s::RNG] as u32;
    for k in 0..n {
        let cr = c::GLOBALS + k * c::STRIDE;
        let sr = s::GLOBALS + k * s::STRIDE;
        let or = o::GLOBALS + k * o::STRIDE;
        let (x, y, z, spin) = (config[cr + c::X], config[cr + c::Y], config[cr + c::Z], config[cr + c::SPIN]);
        let mut command = input[i::GLOBALS + k * i::STRIDE + i::OMEGA_CMD];
        if command < 0.0 {
            command = 0.0;
        } else if command > omega_max {
            command = omega_max;
        }
        let omega0 = state[sr + s::OMEGA];
        let omega1 = command + (omega0 - command) * decay;
        let omega_mean = command + (omega0 - command) * average;
        let omega_dot = (omega1 - omega0) / dt;
        rng = xorshift(rng);
        let noise = state[sr + s::NOISE] * noise_decay + noise_gain * (rng as f64 / 2_147_483_648.0 - 1.0);
        let hx = u + (q * z - r * y);
        let hy = v + (r * x - p * z);
        let hz = w + (p * y - q * x);
        let tip = omega_mean * radius;
        let (mut thrust, mut h_force, mut torque, mut induced, mut ge, mut vrs) = (0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        if tip > min_tip {
            let edge = (hx * hx + hy * hy).sqrt();
            let mu = edge / tip;
            let lc = -hz / tip;
            ge = ground_effect(agl, c31, c32, c33, x, y, z, radius, spacing, body, body_gain);
            let li = solve_inflow(lc, mu, ge, amplitude * noise, fold, sigma_a_half, theta0, theta_tw);
            let lambda = lc + li;
            let mu2 = mu * mu;
            let ct = thrust_coefficient(sigma_a_half, pitch_term(theta0, theta_tw, mu), lambda);
            let ch = 0.25 * sigma_cd0 * mu + 0.5 * sigma_a_half * lambda * mu * (theta0 + 0.5 * theta_tw);
            let cq = lambda * ct + 0.125 * sigma_cd0 * (1.0 + 4.65 * mu2);
            let cm = hub_gain * sigma_a_half * mu * (ONE_THIRD * theta0 + 0.25 * theta_tw - 0.25 * lambda);
            let scale = area * tip * tip;
            thrust = ct * scale;
            h_force = ch * scale;
            torque = cq * scale * radius;
            let hub = cm * scale * radius;
            induced = li * tip;
            if ct > MIN_CT {
                let lh = (0.5 * ct).sqrt();
                vrs = vrs_intensity(lc / lh, mu / lh, fold);
            }
            let (mut ex, mut ey) = (0.0, 0.0);
            if edge > 0.0 {
                ex = hx / edge;
                ey = hy / edge;
            }
            let rx = -h_force * ex;
            let ry = -h_force * ey;
            let rz = -thrust;
            fx += rx;
            fy += ry;
            fz += rz;
            l += y * rz - z * ry + spin * hub * ex;
            m += z * rx - x * rz + spin * hub * ey;
            yaw += x * ry - y * rx;
        }
        yaw -= spin * (torque + inertia * omega_dot);
        momentum += spin * inertia * omega_mean;
        state[sr + s::OMEGA] = omega1;
        state[sr + s::NOISE] = noise;
        output[or + o::THRUST] = thrust;
        output[or + o::H_FORCE] = h_force;
        output[or + o::TORQUE] = torque;
        output[or + o::OMEGA] = omega1;
        output[or + o::OMEGA_MEAN] = omega_mean;
        output[or + o::INDUCED] = induced;
        output[or + o::GROUND_EFFECT] = ge;
        output[or + o::VRS] = vrs;
    }
    state[s::RNG] = rng as f64;
    output[o::FX] = fx;
    output[o::FY] = fy;
    output[o::FZ] = fz;
    output[o::L] = l - q * momentum;
    output[o::M] = m + p * momentum;
    output[o::N] = yaw;
}
