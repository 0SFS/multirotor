import { PLANT } from "../src/layout";
import type { AircraftSample, FlightCommand, Multirotor } from "../src/multirotor";

/**
 * A minimal rigid body for the tests, in place of JSBSim: flat earth, NED,
 * a diagonal inertia, quadratic body drag and a flat ground at d = 0 the CG
 * rests `restM` above. Semi-implicit Euler at the multirotor's step.
 */
export interface Body {
  n: number; e: number; d: number;
  vn: number; ve: number; vd: number;
  /** Body to NED, row major. */
  c: number[];
  p: number; q: number; r: number;
}

export interface World {
  massKg: number; ixx: number; iyy: number; izz: number;
  rho: number;
  /** Drag area per body axis, m^2. */
  cdA: [number, number, number];
  /** Wind NED, m/s. */
  wind: [number, number, number];
  /** CG height on the ground, m; NaN for no ground. */
  restM: number;
}

export const G = 9.80665;

export function level(heading = 0, heightM = 10): Body {
  const c = Math.cos(heading), s = Math.sin(heading);
  return { n: 0, e: 0, d: -heightM, vn: 0, ve: 0, vd: 0, c: [c, -s, 0, s, c, 0, 0, 0, 1], p: 0, q: 0, r: 0 };
}

export function euler(body: Body): { phi: number; theta: number; psi: number } {
  const c = body.c;
  return { phi: Math.atan2(c[7], c[8]), theta: -Math.asin(Math.max(-1, Math.min(1, c[6]))), psi: Math.atan2(c[3], c[0]) };
}

/** Tilt of body z from the vertical, rad. */
export function tilt(body: Body): number {
  return Math.acos(Math.max(-1, Math.min(1, body.c[8])));
}

export function sample(body: Body, world: World): AircraftSample {
  const c = body.c;
  const an = body.vn - world.wind[0], ae = body.ve - world.wind[1], ad = body.vd - world.wind[2];
  const { phi, theta, psi } = euler(body);
  return {
    vn: body.vn, ve: body.ve, vd: body.vd,
    u: c[0] * an + c[3] * ae + c[6] * ad,
    v: c[1] * an + c[4] * ae + c[7] * ad,
    w: c[2] * an + c[5] * ae + c[8] * ad,
    p: body.p, q: body.q, r: body.r,
    phi, theta, psi,
    massKg: world.massKg, ixx: world.ixx, iyy: world.iyy, izz: world.izz,
    rho: world.rho,
    aglM: Number.isNaN(world.restM) ? NaN : -body.d,
  };
}

/** Steps the multirotor and then the body. */
export function advance(body: Body, world: World, multirotor: Multirotor, command: FlightCommand): void {
  const dt = multirotor.airframe.dt;
  const s = sample(body, world);
  multirotor.step(s, command);
  const out = multirotor.plant.output;
  const O = PLANT.output;
  const half = 0.5 * world.rho;
  const air = Math.hypot(s.u, s.v, s.w);
  const fx = out[O.FX] - half * world.cdA[0] * air * s.u;
  const fy = out[O.FY] - half * world.cdA[1] * air * s.v;
  const fz = out[O.FZ] - half * world.cdA[2] * air * s.w;
  const c = body.c;
  let fn = c[0] * fx + c[1] * fy + c[2] * fz;
  let fe = c[3] * fx + c[4] * fy + c[5] * fz;
  let fd = c[6] * fx + c[7] * fy + c[8] * fz + world.massKg * G;
  let { p, q, r } = body;
  // The ground: stiff, damped, with friction; it stops rotation too.
  const below = Number.isNaN(world.restM) ? -Infinity : body.d + world.restM;
  if (below > 0) {
    const normal = Math.max(0, 4000 * world.massKg * below + 120 * world.massKg * body.vd);
    fd -= normal;
    fn -= Math.min(normal * 0.6, world.massKg * Math.abs(body.vn) / dt) * Math.sign(body.vn);
    fe -= Math.min(normal * 0.6, world.massKg * Math.abs(body.ve) / dt) * Math.sign(body.ve);
  }
  const l = out[O.L], m = out[O.M], nn = out[O.N];
  p += dt * (l - (world.izz - world.iyy) * q * r) / world.ixx;
  q += dt * (m - (world.ixx - world.izz) * r * p) / world.iyy;
  r += dt * (nn - (world.iyy - world.ixx) * p * q) / world.izz;
  if (below > 0) {
    // Feet on the ground: level the body and stop it turning.
    const keep = Math.max(0, 1 - 40 * dt);
    p *= keep; q *= keep; r *= keep;
  }
  body.vn += dt * fn / world.massKg;
  body.ve += dt * fe / world.massKg;
  body.vd += dt * fd / world.massKg;
  body.n += dt * body.vn;
  body.e += dt * body.ve;
  body.d += dt * body.vd;
  body.p = p; body.q = q; body.r = r;
  // C' = C [w]x, then re-orthonormalise.
  const k = [c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8]];
  for (let i = 0; i < 3; i++) {
    const a = k[3 * i], b = k[3 * i + 1], d = k[3 * i + 2];
    c[3 * i] = a + dt * (b * r - d * q);
    c[3 * i + 1] = b + dt * (d * p - a * r);
    c[3 * i + 2] = d + dt * (a * q - b * p);
  }
  orthonormalise(c);
}

function orthonormalise(c: number[]): void {
  // Gram-Schmidt on the columns.
  const x = [c[0], c[3], c[6]], y = [c[1], c[4], c[7]];
  const nx = Math.hypot(x[0], x[1], x[2]);
  for (let i = 0; i < 3; i++) x[i] /= nx;
  const dot = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
  for (let i = 0; i < 3; i++) y[i] -= dot * x[i];
  const ny = Math.hypot(y[0], y[1], y[2]);
  for (let i = 0; i < 3; i++) y[i] /= ny;
  const z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
  for (let i = 0; i < 3; i++) { c[3 * i] = x[i]; c[3 * i + 1] = y[i]; c[3 * i + 2] = z[i]; }
}
