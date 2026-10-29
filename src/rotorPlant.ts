import { PLANT, rotorCount } from "./layout";

/**
 * The rotor plant, down to the spinning propeller: each rotor's speed follows
 * its command with one time constant (an ideal motor and ESC), and blade
 * element theory with momentum inflow gives its thrust, H-force, torque and
 * hub moment from the air the hub moves through. Ground effect, the vortex
 * ring state and its fluctuation change the inflow. The rotors' gyroscopic
 * and reaction moments go on the body too.
 *
 * Only + - * /, sqrt, floor and comparisons, in an order the Rust twin keeps,
 * so both give the same bits. The host computes every exp, sin and cos.
 */

const C = PLANT.config;
const IN = PLANT.input;
const S = PLANT.state;
const O = PLANT.output;
const ONE_THIRD = 1 / 3;
/** Regula falsi (Illinois) steps for the inflow, Newton steps and bisections for momentum theory. */
const INFLOW_STEPS = 20;
const NEWTON_STEPS = 8;
const BISECTIONS = 48;
/** Below this thrust coefficient the rotor has no induced inflow. */
const MIN_CT = 1e-14;
/** Leishman's axial induced-velocity curve through the vortex ring state, with kappa = 1 and 0.013 added to the first coefficient so it meets the windmill branch at x = -2. */
const K1 = -1.112;
const K2 = -1.372;
const K3 = -1.718;
const K4 = -0.655;

/** A kernel failure, with the WASM core's status: 1 invalid buffers, 3 bad input. */
export class KernelError extends Error {
  readonly status: number;
  constructor(message: string, status: number) {
    super(message);
    this.status = status;
  }
}
export const INVALID = 1;
export const INPUT = 3;

/** xorshift32 on an integer held in an f64. */
export function xorshift(value: number): number {
  let x = value >>> 0;
  x ^= x << 13;
  x >>>= 0;
  x ^= x >>> 17;
  x ^= x << 5;
  return x >>> 0;
}

/**
 * Induced velocity over hover's, w = lambda_i / lambda_h, in pure axial flight
 * (x = lambda_c / lambda_h < 0): the empirical curve through the vortex ring
 * state down to x = -2, then the windmill brake state.
 */
export function axialInflow(x: number): number {
  if (x <= -2) return -0.5 * x - Math.sqrt(0.25 * x * x - 1);
  return 1 + x * (K1 + x * (K2 + x * (K3 + x * K4)));
}

/** w for x >= 0 from Glauert's w^2 (m^2 + (x + w)^2) = 1: Newton from an upper bound, where the curve is convex. */
export function glauertClimb(x: number, m: number): number {
  const m2 = m * m;
  let w = 1 / Math.sqrt(m2 + x * x);
  if (!(w < 1)) w = 1;
  for (let i = 0; i < NEWTON_STEPS; i++) {
    const s = x + w;
    const a = m2 + s * s;
    w = w - (w * w * a - 1) / (2 * w * a + 2 * w * w * s);
  }
  return w;
}

/** w for x < 0 where Glauert's equation has one root (m at or above the fold), by bisection on [0, 1/m]. */
export function glauertDescent(x: number, m: number): number {
  const m2 = m * m;
  let lo = 0;
  let hi = 1 / m;
  for (let i = 0; i < BISECTIONS; i++) {
    const w = 0.5 * (lo + hi);
    const s = x + w;
    if (w * w * (m2 + s * s) - 1 > 0) hi = w;
    else lo = w;
  }
  return 0.5 * (lo + hi);
}

/**
 * w for any climb ratio x and edgewise ratio m = mu / lambda_h. Above the
 * fold momentum theory has one answer. Below it, in descent, the deviation
 * from x = 0 blends the axial curve (at m = 0) into Glauert's at the fold, so
 * w is continuous across x = 0, the fold and the windmill branch.
 */
export function inflowRatio(x: number, m: number, fold: number): number {
  if (x >= 0) return glauertClimb(x, m);
  if (m >= fold) return glauertDescent(x, m);
  const t = 1 - m / fold;
  const weight = t * t;
  const m2 = m * m;
  const level = Math.sqrt(0.5 * (Math.sqrt(m2 * m2 + 4) - m2));
  const f2 = fold * fold;
  const foldLevel = Math.sqrt(0.5 * (Math.sqrt(f2 * f2 + 4) - f2));
  return level + weight * (axialInflow(x) - 1) + (1 - weight) * (glauertDescent(x, fold) - foldLevel);
}

/** Vortex ring intensity from 0 to 1: strongest at x = -1 in axial descent, gone at the fold. */
export function vrsIntensity(x: number, m: number, fold: number): number {
  if (!(x < 0 && x > -2 && m < fold)) return 0;
  const t = 1 - m / fold;
  const r = x * (x + 2);
  return t * t * r * r;
}

/**
 * Induced inflow multiplier near the ground (Sanchez-Cuevas et al. 2017,
 * the multirotor form): the rotor's own image, its neighbours' and the body's.
 * 1 without a ground or banked past 60 degrees.
 */
export function groundEffect(agl: number, c31: number, c32: number, c33: number, x: number, y: number, z: number,
  radius: number, spacing: number, body: number, bodyGain: number): number {
  if (!(agl === agl) || !(c33 > 0.5)) return 1;
  let h = (agl - (c31 * x + c32 * y + c33 * z)) / c33;
  const lowest = 0.5 * radius;
  if (!(h > lowest)) h = lowest;
  const r2 = radius * radius;
  const q = radius / (4 * h);
  const h4 = 4 * h * h;
  const s1 = spacing * spacing + h4;
  const s2 = 2 * spacing * spacing + h4;
  const s3 = body * body + h4;
  const k = 1 - q * q - r2 * h / (s1 * Math.sqrt(s1)) - 0.5 * r2 * h / (s2 * Math.sqrt(s2))
    - 2 * bodyGain * r2 * h / (s3 * Math.sqrt(s3));
  return k < 0.5 ? 0.5 : k > 1 ? 1 : k;
}

/** Blade-element thrust coefficient at total inflow `lambda`, clamped at 0. */
function thrustCoefficient(sigmaAHalf: number, pitch: number, lambda: number): number {
  const ct = sigmaAHalf * (pitch - 0.5 * lambda);
  return ct > 0 ? ct : 0;
}

/** The induced inflow momentum theory gives for the thrust blade elements make at induced inflow `li`. */
function inducedFor(li: number, lc: number, mu: number, ge: number, swing: number, fold: number,
  sigmaAHalf: number, pitch: number): number {
  const ct = thrustCoefficient(sigmaAHalf, pitch, lc + li);
  if (!(ct > MIN_CT)) return 0;
  const lh = Math.sqrt(0.5 * ct);
  const x = lc / lh;
  const m = mu / lh;
  let f = 1 + swing * vrsIntensity(x, m, fold);
  if (f < 0.2) f = 0.2;
  return ge * lh * inflowRatio(x, m, fold) * f;
}

/**
 * The pitch term of the thrust coefficient over sigma a / 2:
 * theta0 (1/3 + mu^2/2) + thetaTw (1 + mu^2) / 4.
 */
export function pitchTerm(theta0: number, thetaTw: number, mu: number): number {
  const mu2 = mu * mu;
  return theta0 * (ONE_THIRD + 0.5 * mu2) + 0.25 * thetaTw * (1 + mu2);
}

/**
 * The induced inflow ratio where blade elements and momentum agree, by
 * Illinois regula falsi on [0, the inflow that zeroes thrust]: the residual
 * is negative at 0 and positive there. `swing` is the fluctuation amplitude
 * times its noise.
 */
export function solveInflow(lc: number, mu: number, ge: number, swing: number, fold: number,
  sigmaAHalf: number, theta0: number, thetaTw: number): number {
  const pitch = pitchTerm(theta0, thetaTw, mu);
  let hi = 2 * pitch - lc;
  if (!(hi > 0)) return 0;
  let lo = 0;
  let flo = -inducedFor(0, lc, mu, ge, swing, fold, sigmaAHalf, pitch);
  if (!(flo < 0)) return 0;
  let fhi = hi - inducedFor(hi, lc, mu, ge, swing, fold, sigmaAHalf, pitch);
  if (!(fhi > 0)) return hi;
  let side = 0;
  let li = 0;
  for (let i = 0; i < INFLOW_STEPS; i++) {
    li = (lo * fhi - hi * flo) / (fhi - flo);
    const f = li - inducedFor(li, lc, mu, ge, swing, fold, sigmaAHalf, pitch);
    if (f > 0) {
      hi = li; fhi = f;
      if (side === 1) flo *= 0.5;
      side = 1;
    } else if (f < 0) {
      lo = li; flo = f;
      if (side === -1) fhi *= 0.5;
      side = -1;
    } else break;
  }
  return li;
}

/** Rotor count of a plant whose buffers fit together; throws otherwise. */
export function checkPlant(config: Float64Array, input: Float64Array, state: Float64Array, output: Float64Array): number {
  const n = rotorCount(config.length, C.globals, C.stride);
  if (!n || input.length !== IN.globals + n * IN.stride || state.length !== S.globals + n * S.stride
    || output.length !== O.globals + n * O.stride) throw new KernelError("Rotor plant buffers do not fit together", INVALID);
  for (let i = 0; i < config.length; i++) if (!Number.isFinite(config[i])) throw new KernelError("Rotor plant config is not finite", INPUT);
  // AGL may be NaN, for no ground, but not infinite.
  for (let i = 0; i < input.length; i++) {
    const value = input[i];
    if (i === IN.AGL ? value === Infinity || value === -Infinity : !Number.isFinite(value)) throw new KernelError("Rotor plant input is not finite", INPUT);
  }
  for (let i = 0; i < state.length; i++) if (!Number.isFinite(state[i])) throw new KernelError("Rotor plant state is not finite", INPUT);
  const rng = state[S.RNG];
  if (!(rng >= 1 && rng < 4294967296 && Math.floor(rng) === rng) || !(config[C.DT] > 0) || !(input[IN.RHO] >= 0)) {
    throw new KernelError("Rotor plant input is out of range", INPUT);
  }
  return n;
}

/**
 * One step. Reads `config` and `input`, advances `state` and writes `output`:
 * the rotor force (body axes, N) and moment about the CG (N m) to hold for
 * the coming step, and per rotor what it did. Allocates nothing.
 */
export function stepRotorPlant(config: Float64Array, input: Float64Array, state: Float64Array, output: Float64Array): void {
  const n = checkPlant(config, input, state, output);
  const dt = config[C.DT];
  const decay = config[C.DECAY];
  const average = config[C.AVERAGE];
  const omegaMax = config[C.OMEGA_MAX];
  const radius = config[C.RADIUS];
  const sigmaAHalf = config[C.SIGMA_A_HALF];
  const theta0 = config[C.THETA_0];
  const thetaTw = config[C.THETA_TW];
  const sigmaCd0 = config[C.SIGMA_CD0];
  const hubGain = config[C.HUB_GAIN];
  const inertia = config[C.ROTOR_INERTIA];
  const amplitude = config[C.VRS_AMPLITUDE];
  const noiseDecay = config[C.NOISE_DECAY];
  const noiseGain = config[C.NOISE_GAIN];
  const fold = config[C.FOLD_MU];
  const spacing = config[C.GE_SPACING];
  const body = config[C.GE_BODY];
  const bodyGain = config[C.GE_BODY_GAIN];
  const minTip = config[C.MIN_TIP_SPEED];
  const u = input[IN.U], v = input[IN.V], w = input[IN.W];
  const p = input[IN.P], q = input[IN.Q], r = input[IN.R];
  const rho = input[IN.RHO];
  const c31 = input[IN.C31], c32 = input[IN.C32], c33 = input[IN.C33];
  const agl = input[IN.AGL];
  const area = rho * Math.PI * radius * radius;
  let fx = 0, fy = 0, fz = 0, l = 0, m = 0, yaw = 0, momentum = 0;
  let rng = state[S.RNG];
  for (let i = 0; i < n; i++) {
    const c = C.globals + i * C.stride;
    const s = S.globals + i * S.stride;
    const o = O.globals + i * O.stride;
    const x = config[c + C.X], y = config[c + C.Y], z = config[c + C.Z], spin = config[c + C.SPIN];
    let command = input[IN.globals + i * IN.stride + IN.OMEGA_CMD];
    if (command < 0) command = 0;
    else if (command > omegaMax) command = omegaMax;
    const omega0 = state[s + S.OMEGA];
    const omega1 = command + (omega0 - command) * decay;
    const omegaMean = command + (omega0 - command) * average;
    const omegaDot = (omega1 - omega0) / dt;
    rng = xorshift(rng);
    const noise = state[s + S.NOISE] * noiseDecay + noiseGain * (rng / 2147483648 - 1);
    // The hub's velocity through the air, body axes: the CG's plus omega x r.
    const hx = u + (q * z - r * y);
    const hy = v + (r * x - p * z);
    const hz = w + (p * y - q * x);
    const tip = omegaMean * radius;
    let thrust = 0, hForce = 0, torque = 0, induced = 0, ge = 1, vrs = 0;
    if (tip > minTip) {
      const edge = Math.sqrt(hx * hx + hy * hy);
      const mu = edge / tip;
      const lc = -hz / tip;
      ge = groundEffect(agl, c31, c32, c33, x, y, z, radius, spacing, body, bodyGain);
      const li = solveInflow(lc, mu, ge, amplitude * noise, fold, sigmaAHalf, theta0, thetaTw);
      const lambda = lc + li;
      const mu2 = mu * mu;
      const ct = thrustCoefficient(sigmaAHalf, pitchTerm(theta0, thetaTw, mu), lambda);
      const ch = 0.25 * sigmaCd0 * mu + 0.5 * sigmaAHalf * lambda * mu * (theta0 + 0.5 * thetaTw);
      const cq = lambda * ct + 0.125 * sigmaCd0 * (1 + 4.65 * mu2);
      const cm = hubGain * sigmaAHalf * mu * (ONE_THIRD * theta0 + 0.25 * thetaTw - 0.25 * lambda);
      const scale = area * tip * tip;
      thrust = ct * scale;
      hForce = ch * scale;
      torque = cq * scale * radius;
      const hub = cm * scale * radius;
      induced = li * tip;
      if (ct > MIN_CT) {
        const lh = Math.sqrt(0.5 * ct);
        vrs = vrsIntensity(lc / lh, mu / lh, fold);
      }
      let ex = 0, ey = 0;
      if (edge > 0) { ex = hx / edge; ey = hy / edge; }
      // H opposes the hub's edgewise motion; thrust is up the shaft (-z). The
      // advancing blade's extra lift rolls the hub toward the retreating side.
      const rx = -hForce * ex;
      const ry = -hForce * ey;
      const rz = -thrust;
      fx += rx; fy += ry; fz += rz;
      l += y * rz - z * ry + spin * hub * ex;
      m += z * rx - x * rz + spin * hub * ey;
      yaw += x * ry - y * rx;
    }
    // The motor's reaction: drag torque and spin-up, against the spin.
    yaw -= spin * (torque + inertia * omegaDot);
    momentum += spin * inertia * omegaMean;
    state[s + S.OMEGA] = omega1;
    state[s + S.NOISE] = noise;
    output[o + O.THRUST] = thrust;
    output[o + O.H_FORCE] = hForce;
    output[o + O.TORQUE] = torque;
    output[o + O.OMEGA] = omega1;
    output[o + O.OMEGA_MEAN] = omegaMean;
    output[o + O.INDUCED] = induced;
    output[o + O.GROUND_EFFECT] = ge;
    output[o + O.VRS] = vrs;
  }
  state[S.RNG] = rng;
  // Gyroscopic: the body turning the rotors' angular momentum (along body z).
  output[O.FX] = fx;
  output[O.FY] = fy;
  output[O.FZ] = fz;
  output[O.L] = l - q * momentum;
  output[O.M] = m + p * momentum;
  output[O.N] = yaw;
}
