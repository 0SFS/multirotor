//! The flight controller: `stepFlightController` from
//! `src/flightController.ts`, operation for operation. A shaped velocity
//! setpoint, a PI velocity loop, a tilt-prioritised attitude loop with a
//! heading loop, a PI rate loop, and allocation to rotor speeds. The layouts
//! are `CONTROL` in `src/layout.ts`.

use core::f64::consts::PI;

pub mod config {
    pub const DT: usize = 0;
    pub const G: usize = 1;
    pub const K_V: usize = 2;
    pub const K_VI: usize = 3;
    pub const VI_LIMIT_H: usize = 4;
    pub const VI_LIMIT_V: usize = 5;
    pub const SHAPE_GAIN: usize = 6;
    pub const ACCEL_MAX_H: usize = 7;
    pub const ACCEL_MAX_V: usize = 8;
    pub const JERK_MAX: usize = 9;
    pub const K_ATT: usize = 10;
    pub const RATE_MAX_RP: usize = 11;
    pub const K_PSI: usize = 12;
    pub const YAW_LEAD: usize = 13;
    pub const K_RATE: usize = 14;
    pub const K_RI: usize = 15;
    pub const RI_LIMIT: usize = 16;
    pub const THRUST_COEFF: usize = 17;
    pub const OMEGA_IDLE: usize = 18;
    pub const OMEGA_MAX: usize = 19;
    pub const GLOBALS: usize = 20;
    pub const A_THRUST: usize = 0;
    pub const A_ROLL: usize = 1;
    pub const A_PITCH: usize = 2;
    pub const A_YAW: usize = 3;
    pub const STRIDE: usize = 4;
}

pub mod input {
    pub const VN: usize = 0;
    pub const VE: usize = 1;
    pub const VD: usize = 2;
    pub const P: usize = 3;
    pub const Q: usize = 4;
    pub const R: usize = 5;
    pub const C11: usize = 6;
    pub const C12: usize = 7;
    pub const C13: usize = 8;
    pub const C21: usize = 9;
    pub const C22: usize = 10;
    pub const C23: usize = 11;
    pub const C31: usize = 12;
    pub const C32: usize = 13;
    pub const C33: usize = 14;
    pub const PSI: usize = 15;
    pub const MASS: usize = 16;
    pub const IXX: usize = 17;
    pub const IYY: usize = 18;
    pub const IZZ: usize = 19;
    pub const RHO: usize = 20;
    pub const TARGET_VN: usize = 21;
    pub const TARGET_VE: usize = 22;
    pub const TARGET_VD: usize = 23;
    pub const YAW_MODE: usize = 24;
    pub const YAW: usize = 25;
    pub const MODE: usize = 26;
    pub const TAN_TILT_MAX: usize = 27;
    pub const YAW_RATE_MAX: usize = 28;
    pub const THRUST_EFF: usize = 29;
    pub const LENGTH: usize = 30;
}

pub mod state {
    pub const VSP_N: usize = 0;
    pub const VSP_E: usize = 1;
    pub const VSP_D: usize = 2;
    pub const ASP_N: usize = 3;
    pub const ASP_E: usize = 4;
    pub const ASP_D: usize = 5;
    pub const VI_N: usize = 6;
    pub const VI_E: usize = 7;
    pub const VI_D: usize = 8;
    pub const PSI_D: usize = 9;
    pub const RI_P: usize = 10;
    pub const RI_Q: usize = 11;
    pub const RI_R: usize = 12;
    pub const SATURATED: usize = 13;
    pub const ACTIVE: usize = 14;
    pub const LENGTH: usize = 15;
}

pub mod output {
    pub const THRUST: usize = 0;
    pub const AN: usize = 1;
    pub const AE: usize = 2;
    pub const AD: usize = 3;
    pub const P_CMD: usize = 4;
    pub const Q_CMD: usize = 5;
    pub const R_CMD: usize = 6;
    pub const SATURATED: usize = 7;
    pub const GLOBALS: usize = 8;
    pub const OMEGA_CMD: usize = 0;
    pub const STRIDE: usize = 1;
}

pub const MODE_STOPPED: f64 = 0.0;
pub const MODE_IDLE: f64 = 1.0;
pub const MODE_FLYING: f64 = 2.0;
pub const YAW_RATE: f64 = 0.0;
pub const YAW_HEADING: f64 = 1.0;
const MAX_ROTORS: usize = crate::plant::MAX_ROTORS;
const TAU: f64 = 2.0 * PI;

/// `wrapAngle`: into [-pi, pi).
#[inline]
pub fn wrap_angle(angle: f64) -> f64 {
    angle - TAU * ((angle + PI) / TAU).floor()
}

#[inline]
fn limit(value: f64, bound: f64) -> f64 {
    if value > bound {
        bound
    } else if value < -bound {
        -bound
    } else {
        value
    }
}

/// Rotor count of buffers that fit together, or `None`.
pub fn rotors(config: usize, input: usize, state: usize, output: usize) -> Option<usize> {
    if config <= config::GLOBALS || !(config - config::GLOBALS).is_multiple_of(config::STRIDE) {
        return None;
    }
    let n = (config - config::GLOBALS) / config::STRIDE;
    let fits = n <= MAX_ROTORS && input == input::LENGTH && state == state::LENGTH && output == output::GLOBALS + n * output::STRIDE;
    fits.then_some(n)
}

/// `checkController`'s value checks: false for bad input, before anything is written.
pub fn valid(config: &[f64], input: &[f64], state: &[f64]) -> bool {
    let mode = input[input::MODE];
    let yaw_mode = input[input::YAW_MODE];
    config.iter().all(|v| v.is_finite())
        && input.iter().all(|v| v.is_finite())
        && state.iter().all(|v| v.is_finite())
        && config[config::DT] > 0.0
        && config[config::THRUST_COEFF] > 0.0
        && config[config::OMEGA_IDLE] > 0.0
        && config[config::OMEGA_MAX] > config[config::OMEGA_IDLE]
        && input[input::MASS] > 0.0
        && input[input::RHO] > 0.0
        && input[input::THRUST_EFF] > 0.0
        && (mode == MODE_STOPPED || mode == MODE_IDLE || mode == MODE_FLYING)
        && (yaw_mode == YAW_RATE || yaw_mode == YAW_HEADING)
}

/// One step on buffers [`rotors`] and [`valid`] accepted.
pub fn step(config: &[f64], input: &[f64], state: &mut [f64], output: &mut [f64], n: usize) {
    use self::{config as c, input as i, output as o, state as s};
    let dt = config[c::DT];
    let mode = input[i::MODE];
    let rho = input[i::RHO];
    let thrust_scale = config[c::THRUST_COEFF] * rho * input[i::THRUST_EFF];
    let omega_idle = config[c::OMEGA_IDLE];
    let omega_max = config[c::OMEGA_MAX];
    let rotor_min = thrust_scale * omega_idle * omega_idle;
    let rotor_max = thrust_scale * omega_max * omega_max;
    let (vn, ve, vd) = (input[i::VN], input[i::VE], input[i::VD]);
    let psi = input[i::PSI];
    if mode != MODE_FLYING {
        state[s::VSP_N] = vn;
        state[s::VSP_E] = ve;
        state[s::VSP_D] = vd;
        state[s::ASP_N] = 0.0;
        state[s::ASP_E] = 0.0;
        state[s::ASP_D] = 0.0;
        state[s::VI_N] = 0.0;
        state[s::VI_E] = 0.0;
        state[s::VI_D] = 0.0;
        state[s::PSI_D] = psi;
        state[s::RI_P] = 0.0;
        state[s::RI_Q] = 0.0;
        state[s::RI_R] = 0.0;
        state[s::SATURATED] = 0.0;
        state[s::ACTIVE] = 0.0;
        let omega = if mode == MODE_IDLE { omega_idle } else { 0.0 };
        output[..o::GLOBALS].fill(0.0);
        for k in 0..n {
            output[o::GLOBALS + k * o::STRIDE + o::OMEGA_CMD] = omega;
        }
        return;
    }
    if state[s::ACTIVE] != 1.0 {
        state[s::VSP_N] = vn;
        state[s::VSP_E] = ve;
        state[s::VSP_D] = vd;
        state[s::ASP_N] = 0.0;
        state[s::ASP_E] = 0.0;
        state[s::ASP_D] = 0.0;
        state[s::PSI_D] = psi;
        state[s::ACTIVE] = 1.0;
    }
    let held = state[s::SATURATED] == 1.0;
    let g = config[c::G];
    let mass = input[i::MASS];
    let (c11, c12, c13) = (input[i::C11], input[i::C12], input[i::C13]);
    let (c21, c22, c23) = (input[i::C21], input[i::C22], input[i::C23]);
    let (c31, c32, c33) = (input[i::C31], input[i::C32], input[i::C33]);
    let (p, q, r) = (input[i::P], input[i::Q], input[i::R]);

    // 1. Shaped setpoint.
    let shape = config[c::SHAPE_GAIN];
    let jerk = config[c::JERK_MAX] * dt;
    let mut dn = shape * (input[i::TARGET_VN] - state[s::VSP_N]);
    let mut de = shape * (input[i::TARGET_VE] - state[s::VSP_E]);
    let mut dd = shape * (input[i::TARGET_VD] - state[s::VSP_D]);
    let accel_h = config[c::ACCEL_MAX_H];
    let length_h = (dn * dn + de * de).sqrt();
    if length_h > accel_h {
        dn = dn * accel_h / length_h;
        de = de * accel_h / length_h;
    }
    dd = limit(dd, config[c::ACCEL_MAX_V]);
    let mut jn = dn - state[s::ASP_N];
    let mut je = de - state[s::ASP_E];
    let mut jd = dd - state[s::ASP_D];
    let length_j = (jn * jn + je * je + jd * jd).sqrt();
    if length_j > jerk {
        jn = jn * jerk / length_j;
        je = je * jerk / length_j;
        jd = jd * jerk / length_j;
    }
    let asp_n = state[s::ASP_N] + jn;
    let asp_e = state[s::ASP_E] + je;
    let asp_d = state[s::ASP_D] + jd;
    let vsp_n = state[s::VSP_N] + asp_n * dt;
    let vsp_e = state[s::VSP_E] + asp_e * dt;
    let vsp_d = state[s::VSP_D] + asp_d * dt;
    state[s::ASP_N] = asp_n;
    state[s::ASP_E] = asp_e;
    state[s::ASP_D] = asp_d;
    state[s::VSP_N] = vsp_n;
    state[s::VSP_E] = vsp_e;
    state[s::VSP_D] = vsp_d;

    // 2. Velocity PI.
    let kv = config[c::K_V];
    let (en, ee, ed) = (vsp_n - vn, vsp_e - ve, vsp_d - vd);
    let (mut vin, mut vie, mut vid) = (state[s::VI_N], state[s::VI_E], state[s::VI_D]);
    if !held {
        let kvi = config[c::K_VI] * dt;
        vin += kvi * en;
        vie += kvi * ee;
        vid += kvi * ed;
        let bound_h = config[c::VI_LIMIT_H];
        let length_i = (vin * vin + vie * vie).sqrt();
        if length_i > bound_h {
            vin = vin * bound_h / length_i;
            vie = vie * bound_h / length_i;
        }
        vid = limit(vid, config[c::VI_LIMIT_V]);
        state[s::VI_N] = vin;
        state[s::VI_E] = vie;
        state[s::VI_D] = vid;
    }
    let an = asp_n + kv * en + vin;
    let ae = asp_e + kv * ee + vie;
    let ad = asp_d + kv * ed + vid;
    let mut saturated = 0.0;
    let mut f_n = mass * an;
    let mut f_e = mass * ae;
    let mut f_d = mass * (ad - g);
    let total_min = n as f64 * rotor_min;
    let total_max = n as f64 * rotor_max;
    if f_d > -total_min {
        f_d = -total_min;
    }
    if f_d < -0.9 * total_max {
        f_d = -0.9 * total_max;
        saturated = 1.0;
    }
    let tilt_limit = input[i::TAN_TILT_MAX] * -f_d;
    let mut horizontal = (f_n * f_n + f_e * f_e).sqrt();
    if horizontal > tilt_limit {
        f_n = f_n * tilt_limit / horizontal;
        f_e = f_e * tilt_limit / horizontal;
        horizontal = tilt_limit;
        saturated = 1.0;
    }
    let room = (total_max * total_max - f_d * f_d).sqrt();
    if horizontal > room {
        f_n = f_n * room / horizontal;
        f_e = f_e * room / horizontal;
        saturated = 1.0;
    }
    let force = (f_n * f_n + f_e * f_e + f_d * f_d).sqrt();
    let (bn, be, bd) = (-f_n / force, -f_e / force, -f_d / force);

    // 3. Attitude.
    // `clamp` is the TypeScript's two comparisons, NaN through.
    let cosine = (c13 * bn + c23 * be + c33 * bd).clamp(-1.0, 1.0);
    let xn = c23 * bd - c33 * be;
    let xe = c33 * bn - c13 * bd;
    let xd = c13 * be - c23 * bn;
    let ex = c11 * xn + c21 * xe + c31 * xd;
    let ey = c12 * xn + c22 * xe + c32 * xd;
    let ez = c13 * xn + c23 * xe + c33 * xd;
    let k_att = config[c::K_ATT];
    let sin_half = (0.5 * (1.0 - cosine)).sqrt();
    let axis = (ex * ex + ey * ey + ez * ez).sqrt();
    let (mut wx, mut wy, mut wz) = (0.0, 0.0, 0.0);
    if axis > 1e-12 {
        let k = 2.0 * k_att * sin_half / axis;
        wx = k * ex;
        wy = k * ey;
        wz = k * ez;
    } else if cosine < 0.0 {
        wx = 2.0 * k_att;
    }
    let rate_max = config[c::RATE_MAX_RP];
    wx = limit(wx, rate_max);
    wy = limit(wy, rate_max);

    // Heading.
    let yaw_rate_max = input[i::YAW_RATE_MAX];
    let mut psi_d = state[s::PSI_D];
    let feed_forward;
    if input[i::YAW_MODE] == YAW_HEADING {
        let step = limit(wrap_angle(input[i::YAW] - psi_d), yaw_rate_max * dt);
        psi_d = wrap_angle(psi_d + step);
        feed_forward = step / dt;
    } else {
        feed_forward = limit(input[i::YAW], yaw_rate_max);
        psi_d = wrap_angle(psi_d + feed_forward * dt);
    }
    let lead_max = config[c::YAW_LEAD];
    let mut lead = wrap_angle(psi_d - psi);
    if lead > lead_max || lead < -lead_max {
        lead = limit(lead, lead_max);
        psi_d = wrap_angle(psi + lead);
    }
    state[s::PSI_D] = psi_d;
    let yaw_rate = config[c::K_PSI] * lead + feed_forward;
    wx += yaw_rate * c31;
    wy += yaw_rate * c32;
    wz += yaw_rate * c33;

    // 4. Rate PI with the gyroscopic term.
    let (ixx, iyy, izz) = (input[i::IXX], input[i::IYY], input[i::IZZ]);
    let k_rate = config[c::K_RATE];
    let (ep, eq, er) = (wx - p, wy - q, wz - r);
    let (mut rip, mut riq, mut rir) = (state[s::RI_P], state[s::RI_Q], state[s::RI_R]);
    if !held {
        let kri = config[c::K_RI] * dt;
        let bound = config[c::RI_LIMIT];
        rip = limit(rip + kri * ep, bound);
        riq = limit(riq + kri * eq, bound);
        rir = limit(rir + kri * er, bound);
        state[s::RI_P] = rip;
        state[s::RI_Q] = riq;
        state[s::RI_R] = rir;
    }
    let tx = ixx * (k_rate * ep + rip) + (q * izz * r - r * iyy * q);
    let ty = iyy * (k_rate * eq + riq) + (r * ixx * p - p * izz * r);
    let tz = izz * (k_rate * er + rir) + (p * iyy * q - q * ixx * p);

    // 5. Allocation.
    let mut collective = -(f_n * c13 + f_e * c23 + f_d * c33);
    if collective < total_min {
        collective = total_min;
    } else if collective > total_max {
        collective = total_max;
    }
    let (mut lowest, mut highest) = (0.0, 0.0);
    for k in 0..n {
        let a = c::GLOBALS + k * c::STRIDE;
        let t = config[a + c::A_ROLL] * tx + config[a + c::A_PITCH] * ty;
        if k == 0 || t < lowest {
            lowest = t;
        }
        if k == 0 || t > highest {
            highest = t;
        }
    }
    let mut scale = 1.0;
    let spread = highest - lowest;
    if spread > rotor_max - rotor_min {
        scale = (rotor_max - rotor_min) / spread;
        saturated = 1.0;
    }
    lowest = 0.0;
    highest = 0.0;
    for k in 0..n {
        let a = c::GLOBALS + k * c::STRIDE;
        let t = config[a + c::A_THRUST] * collective + scale * (config[a + c::A_ROLL] * tx + config[a + c::A_PITCH] * ty);
        output[o::GLOBALS + k * o::STRIDE + o::OMEGA_CMD] = t;
        if k == 0 || t < lowest {
            lowest = t;
        }
        if k == 0 || t > highest {
            highest = t;
        }
    }
    let mut shift = 0.0;
    if highest > rotor_max {
        shift = rotor_max - highest;
    } else if lowest < rotor_min {
        shift = rotor_min - lowest;
    }
    if shift != 0.0 {
        saturated = 1.0;
    }
    let mut share = 1.0;
    for k in 0..n {
        let a = c::GLOBALS + k * c::STRIDE;
        let t = output[o::GLOBALS + k * o::STRIDE + o::OMEGA_CMD] + shift;
        let y = config[a + c::A_YAW] * tz;
        let mut room = 1.0;
        if y > 0.0 {
            room = (rotor_max - t) / y;
        } else if y < 0.0 {
            room = (rotor_min - t) / y;
        }
        if room < share {
            share = room;
        }
    }
    if share < 0.0 {
        share = 0.0;
    }
    if share < 1.0 {
        saturated = 1.0;
    }
    let mut total = 0.0;
    for k in 0..n {
        let a = c::GLOBALS + k * c::STRIDE;
        let slot = o::GLOBALS + k * o::STRIDE + o::OMEGA_CMD;
        let mut t = output[slot] + shift + share * config[a + c::A_YAW] * tz;
        if t < rotor_min {
            t = rotor_min;
        } else if t > rotor_max {
            t = rotor_max;
        }
        total += t;
        output[slot] = (t / thrust_scale).sqrt();
    }
    state[s::SATURATED] = saturated;
    output[o::THRUST] = total;
    output[o::AN] = an;
    output[o::AE] = ae;
    output[o::AD] = ad;
    output[o::P_CMD] = wx;
    output[o::Q_CMD] = wy;
    output[o::R_CMD] = wz;
    output[o::SATURATED] = saturated;
}
