import { CONTROL, rotorCount } from "./layout";
import { INPUT, INVALID, KernelError } from "./rotorPlant";

/**
 * The flight controller, from a commanded velocity to rotor speeds:
 *
 * 1. The velocity setpoint follows the command with acceleration and jerk limits.
 * 2. A PI velocity loop gives the acceleration, and with gravity the force.
 *    Vertical comes first: the tilt and thrust limits trim the horizontal.
 * 3. A tilt-prioritised attitude loop turns the thrust axis toward that force
 *    and a heading loop turns about the vertical.
 * 4. A PI rate loop with the gyroscopic term gives the moments.
 * 5. Allocation to rotor thrusts keeps thrust and tilt and lets yaw give
 *    first; static thrust per rotor speed squared, scaled by the thrust
 *    efficiency measured last, gives the speeds.
 *
 * Arithmetic, sqrt and floor only, in the Rust twin's order.
 */

const C = CONTROL.config;
const IN = CONTROL.input;
const S = CONTROL.state;
const O = CONTROL.output;
const TAU = 2 * Math.PI;

/** An angle wrapped into [-pi, pi). */
export function wrapAngle(angle: number): number {
  return angle - TAU * Math.floor((angle + Math.PI) / TAU);
}

function limit(value: number, bound: number): number {
  return value > bound ? bound : value < -bound ? -bound : value;
}

/** Rotor count of a controller whose buffers fit together; throws otherwise. */
export function checkController(config: Float64Array, input: Float64Array, state: Float64Array, output: Float64Array): number {
  const n = rotorCount(config.length, C.globals, C.stride);
  if (!n || input.length !== IN.length || state.length !== S.length || output.length !== O.globals + n * O.stride) {
    throw new KernelError("Flight controller buffers do not fit together", INVALID);
  }
  for (let i = 0; i < config.length; i++) if (!Number.isFinite(config[i])) throw new KernelError("Flight controller config is not finite", INPUT);
  for (let i = 0; i < input.length; i++) if (!Number.isFinite(input[i])) throw new KernelError("Flight controller input is not finite", INPUT);
  for (let i = 0; i < state.length; i++) if (!Number.isFinite(state[i])) throw new KernelError("Flight controller state is not finite", INPUT);
  const mode = input[IN.MODE];
  const yawMode = input[IN.YAW_MODE];
  if (!(config[C.DT] > 0 && config[C.THRUST_COEFF] > 0 && config[C.OMEGA_IDLE] > 0 && config[C.OMEGA_MAX] > config[C.OMEGA_IDLE]
    && input[IN.MASS] > 0 && input[IN.RHO] > 0 && input[IN.THRUST_EFF] > 0)
    || !(mode === CONTROL.MODE_STOPPED || mode === CONTROL.MODE_IDLE || mode === CONTROL.MODE_FLYING)
    || !(yawMode === CONTROL.YAW_RATE || yawMode === CONTROL.YAW_HEADING)) {
    throw new KernelError("Flight controller input is out of range", INPUT);
  }
  return n;
}

/** One step: reads `config` and `input`, advances `state` and writes `output`. Allocates nothing. */
export function stepFlightController(config: Float64Array, input: Float64Array, state: Float64Array, output: Float64Array): void {
  const n = checkController(config, input, state, output);
  const dt = config[C.DT];
  const mode = input[IN.MODE];
  const rho = input[IN.RHO];
  const thrustScale = config[C.THRUST_COEFF] * rho * input[IN.THRUST_EFF];
  const omegaIdle = config[C.OMEGA_IDLE];
  const omegaMax = config[C.OMEGA_MAX];
  const rotorMin = thrustScale * omegaIdle * omegaIdle;
  const rotorMax = thrustScale * omegaMax * omegaMax;
  const vn = input[IN.VN], ve = input[IN.VE], vd = input[IN.VD];
  const psi = input[IN.PSI];
  if (mode !== CONTROL.MODE_FLYING) {
    // On the ground the setpoints follow the aircraft and the integrators rest.
    state[S.VSP_N] = vn; state[S.VSP_E] = ve; state[S.VSP_D] = vd;
    state[S.ASP_N] = 0; state[S.ASP_E] = 0; state[S.ASP_D] = 0;
    state[S.VI_N] = 0; state[S.VI_E] = 0; state[S.VI_D] = 0;
    state[S.PSI_D] = psi;
    state[S.RI_P] = 0; state[S.RI_Q] = 0; state[S.RI_R] = 0;
    state[S.SATURATED] = 0;
    state[S.ACTIVE] = 0;
    const omega = mode === CONTROL.MODE_IDLE ? omegaIdle : 0;
    for (let k = 0; k < O.globals; k++) output[k] = 0;
    for (let i = 0; i < n; i++) output[O.globals + i * O.stride + O.OMEGA_CMD] = omega;
    return;
  }
  if (state[S.ACTIVE] !== 1) {
    state[S.VSP_N] = vn; state[S.VSP_E] = ve; state[S.VSP_D] = vd;
    state[S.ASP_N] = 0; state[S.ASP_E] = 0; state[S.ASP_D] = 0;
    state[S.PSI_D] = psi;
    state[S.ACTIVE] = 1;
  }
  const held = state[S.SATURATED] === 1;
  const g = config[C.G];
  const mass = input[IN.MASS];
  const c11 = input[IN.C11], c12 = input[IN.C12], c13 = input[IN.C13];
  const c21 = input[IN.C21], c22 = input[IN.C22], c23 = input[IN.C23];
  const c31 = input[IN.C31], c32 = input[IN.C32], c33 = input[IN.C33];
  const p = input[IN.P], q = input[IN.Q], r = input[IN.R];

  // 1. Shaped setpoint: horizontal and vertical acceleration limits, one jerk limit.
  const shape = config[C.SHAPE_GAIN];
  const jerk = config[C.JERK_MAX] * dt;
  let dn = shape * (input[IN.TARGET_VN] - state[S.VSP_N]);
  let de = shape * (input[IN.TARGET_VE] - state[S.VSP_E]);
  let dd = shape * (input[IN.TARGET_VD] - state[S.VSP_D]);
  const accelH = config[C.ACCEL_MAX_H];
  const lengthH = Math.sqrt(dn * dn + de * de);
  if (lengthH > accelH) { dn = dn * accelH / lengthH; de = de * accelH / lengthH; }
  dd = limit(dd, config[C.ACCEL_MAX_V]);
  let jn = dn - state[S.ASP_N];
  let je = de - state[S.ASP_E];
  let jd = dd - state[S.ASP_D];
  const lengthJ = Math.sqrt(jn * jn + je * je + jd * jd);
  if (lengthJ > jerk) { jn = jn * jerk / lengthJ; je = je * jerk / lengthJ; jd = jd * jerk / lengthJ; }
  const aspN = state[S.ASP_N] + jn;
  const aspE = state[S.ASP_E] + je;
  const aspD = state[S.ASP_D] + jd;
  const vspN = state[S.VSP_N] + aspN * dt;
  const vspE = state[S.VSP_E] + aspE * dt;
  const vspD = state[S.VSP_D] + aspD * dt;
  state[S.ASP_N] = aspN; state[S.ASP_E] = aspE; state[S.ASP_D] = aspD;
  state[S.VSP_N] = vspN; state[S.VSP_E] = vspE; state[S.VSP_D] = vspD;

  // 2. Velocity PI. The integral holds while the last step was limited.
  const kv = config[C.K_V];
  const en = vspN - vn, ee = vspE - ve, ed = vspD - vd;
  let vin = state[S.VI_N], vie = state[S.VI_E], vid = state[S.VI_D];
  if (!held) {
    const kvi = config[C.K_VI] * dt;
    vin += kvi * en; vie += kvi * ee; vid += kvi * ed;
    const boundH = config[C.VI_LIMIT_H];
    const lengthI = Math.sqrt(vin * vin + vie * vie);
    if (lengthI > boundH) { vin = vin * boundH / lengthI; vie = vie * boundH / lengthI; }
    vid = limit(vid, config[C.VI_LIMIT_V]);
    state[S.VI_N] = vin; state[S.VI_E] = vie; state[S.VI_D] = vid;
  }
  const an = aspN + kv * en + vin;
  const ae = aspE + kv * ee + vie;
  const ad = aspD + kv * ed + vid;
  let saturated = 0;
  let fn = mass * an;
  let fe = mass * ae;
  let fd = mass * (ad - g);
  const totalMin = n * rotorMin;
  const totalMax = n * rotorMax;
  if (fd > -totalMin) fd = -totalMin;
  if (fd < -0.9 * totalMax) { fd = -0.9 * totalMax; saturated = 1; }
  const tiltLimit = input[IN.TAN_TILT_MAX] * -fd;
  let horizontal = Math.sqrt(fn * fn + fe * fe);
  if (horizontal > tiltLimit) {
    fn = fn * tiltLimit / horizontal; fe = fe * tiltLimit / horizontal;
    horizontal = tiltLimit; saturated = 1;
  }
  const room = Math.sqrt(totalMax * totalMax - fd * fd);
  if (horizontal > room) { fn = fn * room / horizontal; fe = fe * room / horizontal; saturated = 1; }
  const force = Math.sqrt(fn * fn + fe * fe + fd * fd);
  // Thrust axis wanted (body z, down) is against the force.
  const bn = -fn / force, be = -fe / force, bd = -fd / force;

  // 3. Attitude: rotate body z (column 3 of the body-to-NED matrix) onto it.
  let cosine = c13 * bn + c23 * be + c33 * bd;
  if (cosine > 1) cosine = 1;
  else if (cosine < -1) cosine = -1;
  const xn = c23 * bd - c33 * be;
  const xe = c33 * bn - c13 * bd;
  const xd = c13 * be - c23 * bn;
  const ex = c11 * xn + c21 * xe + c31 * xd;
  const ey = c12 * xn + c22 * xe + c32 * xd;
  const ez = c13 * xn + c23 * xe + c33 * xd;
  const kAtt = config[C.K_ATT];
  const sinHalf = Math.sqrt(0.5 * (1 - cosine));
  const axis = Math.sqrt(ex * ex + ey * ey + ez * ez);
  let wx = 0, wy = 0, wz = 0;
  if (axis > 1e-12) {
    const k = 2 * kAtt * sinHalf / axis;
    wx = k * ex; wy = k * ey; wz = k * ez;
  } else if (cosine < 0) wx = 2 * kAtt;
  const rateMax = config[C.RATE_MAX_RP];
  wx = limit(wx, rateMax);
  wy = limit(wy, rateMax);

  // Heading: a rate integrates the setpoint, a heading slews it; either way it
  // may lead the aircraft by YAW_LEAD at most.
  const yawRateMax = input[IN.YAW_RATE_MAX];
  let psiD = state[S.PSI_D];
  let feedForward: number;
  if (input[IN.YAW_MODE] === CONTROL.YAW_HEADING) {
    const step = limit(wrapAngle(input[IN.YAW] - psiD), yawRateMax * dt);
    psiD = wrapAngle(psiD + step);
    feedForward = step / dt;
  } else {
    feedForward = limit(input[IN.YAW], yawRateMax);
    psiD = wrapAngle(psiD + feedForward * dt);
  }
  const leadMax = config[C.YAW_LEAD];
  let lead = wrapAngle(psiD - psi);
  if (lead > leadMax || lead < -leadMax) {
    lead = limit(lead, leadMax);
    psiD = wrapAngle(psi + lead);
  }
  state[S.PSI_D] = psiD;
  const yawRate = config[C.K_PSI] * lead + feedForward;
  // About the vertical, in body axes: the body-to-NED matrix's down row.
  wx += yawRate * c31;
  wy += yawRate * c32;
  wz += yawRate * c33;

  // 4. Rate PI with the gyroscopic term.
  const ixx = input[IN.IXX], iyy = input[IN.IYY], izz = input[IN.IZZ];
  const kRate = config[C.K_RATE];
  const ep = wx - p, eq = wy - q, er = wz - r;
  let rip = state[S.RI_P], riq = state[S.RI_Q], rir = state[S.RI_R];
  if (!held) {
    const kri = config[C.K_RI] * dt;
    const bound = config[C.RI_LIMIT];
    rip = limit(rip + kri * ep, bound);
    riq = limit(riq + kri * eq, bound);
    rir = limit(rir + kri * er, bound);
    state[S.RI_P] = rip; state[S.RI_Q] = riq; state[S.RI_R] = rir;
  }
  const tx = ixx * (kRate * ep + rip) + (q * izz * r - r * iyy * q);
  const ty = iyy * (kRate * eq + riq) + (r * ixx * p - p * izz * r);
  const tz = izz * (kRate * er + rir) + (p * iyy * q - q * ixx * p);

  // 5. Allocation. Collective is the force along the thrust axis there is.
  let collective = -(fn * c13 + fe * c23 + fd * c33);
  if (collective < totalMin) collective = totalMin;
  else if (collective > totalMax) collective = totalMax;
  let lowest = 0, highest = 0;
  for (let i = 0; i < n; i++) {
    const a = C.globals + i * C.stride;
    const t = config[a + C.A_ROLL] * tx + config[a + C.A_PITCH] * ty;
    if (i === 0 || t < lowest) lowest = t;
    if (i === 0 || t > highest) highest = t;
  }
  // Too much roll and pitch for the rotors' range: scale them down.
  let scale = 1;
  const spread = highest - lowest;
  if (spread > rotorMax - rotorMin) { scale = (rotorMax - rotorMin) / spread; saturated = 1; }
  lowest = 0; highest = 0;
  for (let i = 0; i < n; i++) {
    const a = C.globals + i * C.stride;
    const t = config[a + C.A_THRUST] * collective + scale * (config[a + C.A_ROLL] * tx + config[a + C.A_PITCH] * ty);
    output[O.globals + i * O.stride + O.OMEGA_CMD] = t;
    if (i === 0 || t < lowest) lowest = t;
    if (i === 0 || t > highest) highest = t;
  }
  // Then shift collective to fit, attitude before height.
  let shift = 0;
  if (highest > rotorMax) shift = rotorMax - highest;
  else if (lowest < rotorMin) shift = rotorMin - lowest;
  if (shift !== 0) saturated = 1;
  // Yaw gets what room is left.
  let share = 1;
  for (let i = 0; i < n; i++) {
    const a = C.globals + i * C.stride;
    const t = output[O.globals + i * O.stride + O.OMEGA_CMD] + shift;
    const y = config[a + C.A_YAW] * tz;
    let room = 1;
    if (y > 0) room = (rotorMax - t) / y;
    else if (y < 0) room = (rotorMin - t) / y;
    if (room < share) share = room;
  }
  if (share < 0) share = 0;
  if (share < 1) saturated = 1;
  let total = 0;
  for (let i = 0; i < n; i++) {
    const a = C.globals + i * C.stride;
    const slot = O.globals + i * O.stride + O.OMEGA_CMD;
    let t = output[slot] + shift + share * config[a + C.A_YAW] * tz;
    if (t < rotorMin) t = rotorMin;
    else if (t > rotorMax) t = rotorMax;
    total += t;
    output[slot] = Math.sqrt(t / thrustScale);
  }
  state[S.SATURATED] = saturated;
  output[O.THRUST] = total;
  output[O.AN] = an;
  output[O.AE] = ae;
  output[O.AD] = ad;
  output[O.P_CMD] = wx;
  output[O.Q_CMD] = wy;
  output[O.R_CMD] = wz;
  output[O.SATURATED] = saturated;
}
