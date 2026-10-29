import { CONTROL, PLANT, controlLengths, plantLengths } from "./layout";
import { solveInflow } from "./rotorPlant";

/** One propeller model, shared by every rotor of an airframe. */
export interface PropellerSpec {
  /** Blade tip radius, m. */
  radiusM: number;
  blades: number;
  /** Mean blade chord, m. */
  chordM: number;
  /** Geometric pitch, m per turn. */
  pitchM: number;
  /** Zero-lift angle of the blade section, degrees; negative for a cambered section. */
  zeroLiftDeg: number;
  /** Section lift slope, per radian. */
  liftSlope: number;
  /** Section profile drag coefficient. */
  profileDrag: number;
  /** Propeller and motor bell polar inertia, kg m^2. */
  inertiaKgM2: number;
  /** Rotor speed response time constant, s: the ideal motor and ESC. */
  timeConstantS: number;
  maxRpm: number;
  /** Armed on the ground. */
  idleRpm: number;
}

/** One rotor: where its hub is and which way it turns. */
export interface RotorSpec {
  /** For the display: "front-left", "front-right", "rear-left" or "rear-right" on a quad. */
  position: string;
  /** Hub, JSBSim structural metres: x aft, y right, z up. */
  hubM: readonly [number, number, number];
  /** +1 clockwise seen from above, -1 counter-clockwise. */
  spin: 1 | -1;
}

/** Shared gains and limits of the flight controller. */
export interface FlightControlSpec {
  /** Velocity loop, 1/s and 1/s^2; integral limits, m/s^2. */
  kv: number;
  kvi: number;
  viLimitH: number;
  viLimitV: number;
  /** Setpoint shaping, 1/s; acceleration limits m/s^2; jerk limit m/s^3. */
  shapeGain: number;
  accelMaxH: number;
  accelMaxV: number;
  jerkMax: number;
  /** Tilt loop, 1/s; roll and pitch rate limit, deg/s. */
  kAtt: number;
  rateMaxDeg: number;
  /** Heading loop, 1/s; how far the heading setpoint may lead, deg. */
  kPsi: number;
  yawLeadDeg: number;
  /** Rate loop, 1/s and 1/s^2; integral limit, rad/s^2. */
  kRate: number;
  kRi: number;
  riLimit: number;
}

/** Inflow effects. Every value has a default. */
export interface RotorAeroSpec {
  /** Fraction of the rigid-blade hub rolling moment the blades pass on. */
  hubMomentGain?: number;
  /** Vortex ring inflow fluctuation at full intensity, as a fraction, and its correlation time, s. */
  vrsAmplitude?: number;
  vrsCorrelationS?: number;
  /** Ground effect body term gain (Sanchez-Cuevas K_b). */
  groundBodyGain?: number;
}

export interface AirframeSpec {
  propeller: PropellerSpec;
  rotors: readonly RotorSpec[];
  /** Centre of gravity, structural metres; the JSBSim model's must be the same. */
  cgM: readonly [number, number, number];
  control: FlightControlSpec;
  aero?: RotorAeroSpec;
}

/** The kernels' constant buffers and what the host needs to drive them. */
export interface Airframe {
  rotors: number;
  dt: number;
  positions: readonly string[];
  spins: readonly (1 | -1)[];
  /** Hubs from the CG in body axes (x forward, y right, z down), m. */
  arms: readonly (readonly [number, number, number])[];
  plantConfig: Float64Array;
  controlConfig: Float64Array;
  /** Static thrust per rotor = thrustCoefficient * rho * omega^2 (N), out of ground effect. */
  thrustCoefficient: number;
  /** Shaft torque over thrust at static thrust, m. */
  torqueRatio: number;
  omegaIdle: number;
  omegaMax: number;
  /** Rotor speed (rad/s) that holds `massKg` up at density `rho`, static and out of ground effect. */
  hoverOmega(massKg: number, rho: number): number;
}

export const G = 9.80665;
const RPM = Math.PI / 30;
const DEG = Math.PI / 180;
/** Edgewise ratio above which momentum theory has one descent solution, kept clear of the cusp near 0.61. */
export const FOLD_MU = 0.7;
const MIN_TIP_SPEED = 1;
const AERO_DEFAULTS: Required<RotorAeroSpec> = { hubMomentGain: 0.6, vrsAmplitude: 0.03, vrsCorrelationS: 0.05, groundBodyGain: 1 };

/** Blade pitch at the axis and its twist to the tip, rad, for a constant-pitch propeller fitted through 50% and 100% radius. */
export function bladePitch(propeller: PropellerSpec): { theta0: number; thetaTw: number; theta75: number } {
  const at = (fraction: number) => Math.atan(propeller.pitchM / (2 * Math.PI * fraction * propeller.radiusM)) - propeller.zeroLiftDeg * DEG;
  const theta75 = at(0.75);
  const thetaTw = 2 * (at(1) - at(0.5));
  return { theta0: theta75 - 0.75 * thetaTw, thetaTw, theta75 };
}

export function solidity(propeller: PropellerSpec): number {
  return propeller.blades * propeller.chordM / (Math.PI * propeller.radiusM);
}

/** Static thrust and torque coefficients, C_T and C_Q, from the plant's own inflow solution. */
export function staticCoefficients(propeller: PropellerSpec): { ct: number; cq: number; inflow: number } {
  const sigma = solidity(propeller);
  const sigmaAHalf = 0.5 * sigma * propeller.liftSlope;
  const { theta0, thetaTw } = bladePitch(propeller);
  const inflow = solveInflow(0, 0, 1, 0, FOLD_MU, sigmaAHalf, theta0, thetaTw);
  const ct = sigmaAHalf * ((theta0 / 3 + thetaTw / 4) - 0.5 * inflow);
  return { ct, cq: inflow * ct + 0.125 * sigma * propeller.profileDrag, inflow };
}

/** Solves A x = b in place for a small dense system; throws if it is singular. */
function solve(matrix: number[][], rhs: number[][]): number[][] {
  const size = matrix.length;
  const a = matrix.map(row => [...row]);
  const b = rhs.map(row => [...row]);
  for (let col = 0; col < size; col++) {
    let pivot = col;
    for (let row = col + 1; row < size; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    if (!(Math.abs(a[pivot][col]) > 1e-12)) throw new Error("Rotor layout cannot control thrust, roll, pitch and yaw");
    [a[col], a[pivot]] = [a[pivot], a[col]];
    [b[col], b[pivot]] = [b[pivot], b[col]];
    for (let row = 0; row < size; row++) {
      if (row === col) continue;
      const factor = a[row][col] / a[col][col];
      for (let k = 0; k < size; k++) a[row][k] -= factor * a[col][k];
      for (let k = 0; k < b[row].length; k++) b[row][k] -= factor * b[col][k];
    }
  }
  return b.map((row, index) => row.map(value => value / a[index][index]));
}

/**
 * Allocation: each rotor's thrust from [thrust, roll, pitch, yaw moment], the
 * minimum-norm inverse of the effectiveness rows [1, -y, x, -spin * kappa].
 */
export function allocation(arms: readonly (readonly [number, number, number])[], spins: readonly number[], torqueRatio: number): number[][] {
  const effectiveness = [arms.map(() => 1), arms.map(arm => -arm[1]), arms.map(arm => arm[0]), spins.map(spin => -spin * torqueRatio)];
  const gram = effectiveness.map(a => effectiveness.map(b => a.reduce((sum, value, i) => sum + value * b[i], 0)));
  // A = B^T (B B^T)^-1: solve (B B^T) X = B, then A = X^T.
  const x = solve(gram, effectiveness);
  return arms.map((_, i) => x.map(row => row[i]));
}

function positive(value: number, what: string): number {
  if (!(Number.isFinite(value) && value > 0)) throw new Error(`${what} must be a positive number`);
  return value;
}

/** Builds the kernels' constant buffers for a fixed step of `dt` seconds. */
export function createAirframe(spec: AirframeSpec, dt: number): Airframe {
  const { propeller, control } = spec;
  const aero = { ...AERO_DEFAULTS, ...spec.aero };
  positive(dt, "The step");
  for (const key of ["radiusM", "blades", "chordM", "pitchM", "liftSlope", "profileDrag", "inertiaKgM2", "timeConstantS", "maxRpm", "idleRpm"] as const) {
    positive(propeller[key], `propeller.${key}`);
  }
  if (!(propeller.idleRpm < propeller.maxRpm)) throw new Error("Idle RPM must be below the maximum");
  const n = spec.rotors.length;
  if (n < 4 || n > PLANT.maxRotors) throw new Error(`A multirotor has 4 to ${PLANT.maxRotors} rotors`);
  const [cgX, cgY, cgZ] = spec.cgM;
  const arms = spec.rotors.map(rotor => [-(rotor.hubM[0] - cgX), rotor.hubM[1] - cgY, -(rotor.hubM[2] - cgZ)] as const);
  const spins = spec.rotors.map(rotor => rotor.spin);
  const { ct, cq } = staticCoefficients(propeller);
  const radius = propeller.radiusM;
  const thrustCoefficient = ct * Math.PI * radius ** 4;
  const torqueRatio = cq * radius / ct;
  const omegaIdle = propeller.idleRpm * RPM;
  const omegaMax = propeller.maxRpm * RPM;
  const { theta0, thetaTw } = bladePitch(propeller);
  const sigma = solidity(propeller);

  // Ground effect: the mean spacing of rotors adjacent around the CG, and the mean hub distance from it.
  const around = arms.map(arm => ({ arm, angle: Math.atan2(arm[1], arm[0]) })).sort((a, b) => a.angle - b.angle);
  let spacing = 0;
  for (let i = 0; i < n; i++) {
    const a = around[i].arm, b = around[(i + 1) % n].arm;
    spacing += Math.hypot(a[0] - b[0], a[1] - b[1]) / n;
  }
  const body = arms.reduce((sum, arm) => sum + Math.hypot(arm[0], arm[1]), 0) / n;

  const plantConfig = new Float64Array(plantLengths(n).config);
  const P = PLANT.config;
  const decay = Math.exp(-dt / propeller.timeConstantS);
  const noiseDecay = Math.exp(-dt / positive(aero.vrsCorrelationS, "aero.vrsCorrelationS"));
  plantConfig[P.DT] = dt;
  plantConfig[P.DECAY] = decay;
  plantConfig[P.AVERAGE] = propeller.timeConstantS / dt * (1 - decay);
  plantConfig[P.OMEGA_MAX] = omegaMax;
  plantConfig[P.RADIUS] = radius;
  plantConfig[P.SIGMA_A_HALF] = 0.5 * sigma * propeller.liftSlope;
  plantConfig[P.THETA_0] = theta0;
  plantConfig[P.THETA_TW] = thetaTw;
  plantConfig[P.SIGMA_CD0] = sigma * propeller.profileDrag;
  plantConfig[P.HUB_GAIN] = aero.hubMomentGain;
  plantConfig[P.ROTOR_INERTIA] = propeller.inertiaKgM2;
  plantConfig[P.VRS_AMPLITUDE] = aero.vrsAmplitude;
  plantConfig[P.NOISE_DECAY] = noiseDecay;
  // Uniform innovations on [-1, 1) have variance 1/3; this gain makes the noise's variance 1.
  plantConfig[P.NOISE_GAIN] = Math.sqrt(3 * (1 - noiseDecay * noiseDecay));
  plantConfig[P.FOLD_MU] = FOLD_MU;
  plantConfig[P.GE_SPACING] = spacing;
  plantConfig[P.GE_BODY] = body;
  plantConfig[P.GE_BODY_GAIN] = aero.groundBodyGain;
  plantConfig[P.MIN_TIP_SPEED] = MIN_TIP_SPEED;
  arms.forEach((arm, i) => {
    const at = P.globals + i * P.stride;
    plantConfig[at + P.X] = arm[0];
    plantConfig[at + P.Y] = arm[1];
    plantConfig[at + P.Z] = arm[2];
    plantConfig[at + P.SPIN] = spins[i];
  });

  const controlConfig = new Float64Array(controlLengths(n).config);
  const K = CONTROL.config;
  controlConfig[K.DT] = dt;
  controlConfig[K.G] = G;
  controlConfig[K.K_V] = control.kv;
  controlConfig[K.K_VI] = control.kvi;
  controlConfig[K.VI_LIMIT_H] = control.viLimitH;
  controlConfig[K.VI_LIMIT_V] = control.viLimitV;
  controlConfig[K.SHAPE_GAIN] = control.shapeGain;
  controlConfig[K.ACCEL_MAX_H] = control.accelMaxH;
  controlConfig[K.ACCEL_MAX_V] = control.accelMaxV;
  controlConfig[K.JERK_MAX] = control.jerkMax;
  controlConfig[K.K_ATT] = control.kAtt;
  controlConfig[K.RATE_MAX_RP] = control.rateMaxDeg * DEG;
  controlConfig[K.K_PSI] = control.kPsi;
  controlConfig[K.YAW_LEAD] = control.yawLeadDeg * DEG;
  controlConfig[K.K_RATE] = control.kRate;
  controlConfig[K.K_RI] = control.kRi;
  controlConfig[K.RI_LIMIT] = control.riLimit;
  controlConfig[K.THRUST_COEFF] = thrustCoefficient;
  controlConfig[K.OMEGA_IDLE] = omegaIdle;
  controlConfig[K.OMEGA_MAX] = omegaMax;
  allocation(arms, spins, torqueRatio).forEach((row, i) => {
    const at = K.globals + i * K.stride;
    controlConfig[at + K.A_THRUST] = row[0];
    controlConfig[at + K.A_ROLL] = row[1];
    controlConfig[at + K.A_PITCH] = row[2];
    controlConfig[at + K.A_YAW] = row[3];
  });
  for (const [name, values] of [["plant", plantConfig], ["controller", controlConfig]] as const) {
    if (!values.every(Number.isFinite)) throw new Error(`The ${name} config is not finite`);
  }
  return {
    rotors: n,
    dt,
    positions: spec.rotors.map(rotor => rotor.position),
    spins,
    arms,
    plantConfig,
    controlConfig,
    thrustCoefficient,
    torqueRatio,
    omegaIdle,
    omegaMax,
    hoverOmega: (massKg, rho) => Math.sqrt(massKg * G / (n * thrustCoefficient * rho)),
  };
}
