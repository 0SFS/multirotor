import { describe, expect, it } from "vitest";
import { createAirframe, FOLD_MU, staticCoefficients } from "../src/airframe";
import { PLANT, plantLengths } from "../src/layout";
import {
  axialInflow, checkPlant, glauertClimb, glauertDescent, groundEffect, inflowRatio, KernelError, solveInflow, stepRotorPlant, vrsIntensity, xorshift,
} from "../src/rotorPlant";
import { AIR_2S, AIR_2S_MASS, MINI_2, MINI_2_MASS } from "./airframes";

const RPM = 30 / Math.PI;
const I = PLANT.input, O = PLANT.output, S = PLANT.state;

/** A plant for `spec` with every rotor at `omega` and commanded there, level, still air. */
function plant(spec = MINI_2, omega = 940) {
  const airframe = createAirframe(spec, 1 / 120);
  const lengths = plantLengths(airframe.rotors);
  const buffers = {
    config: Float64Array.from(airframe.plantConfig),
    input: new Float64Array(lengths.input),
    state: new Float64Array(lengths.state),
    output: new Float64Array(lengths.output),
  };
  buffers.input[I.RHO] = 1.225;
  buffers.input[I.C33] = 1;
  buffers.input[I.AGL] = NaN;
  buffers.state[S.RNG] = 1;
  for (let i = 0; i < airframe.rotors; i++) {
    buffers.input[I.globals + i] = omega;
    buffers.state[S.globals + i * S.stride + S.OMEGA] = omega;
  }
  const step = () => stepRotorPlant(buffers.config, buffers.input, buffers.state, buffers.output);
  const rotor = (i: number, field: number) => buffers.output[O.globals + i * O.stride + field];
  return { airframe, ...buffers, step, rotor };
}

describe("momentum inflow", () => {
  it("solves Glauert's equation in climb and in descent above the fold", () => {
    for (const x of [0, 0.3, 1, 4]) for (const m of [0, 0.2, 1, 3]) {
      const w = glauertClimb(x, m);
      expect(Math.abs(w * w * (m * m + (x + w) ** 2) - 1)).toBeLessThan(1e-12);
    }
    for (const x of [-0.5, -1, -3]) for (const m of [FOLD_MU, 1, 2]) {
      const w = glauertDescent(x, m);
      expect(Math.abs(w * w * (m * m + (x + w) ** 2) - 1)).toBeLessThan(1e-9);
    }
  });

  it("is continuous across hover, the fold and the windmill brake state", () => {
    const step = 1e-6;
    for (const m of [0, 0.3, 0.6]) expect(Math.abs(inflowRatio(-step, m, FOLD_MU) - inflowRatio(step, m, FOLD_MU))).toBeLessThan(1e-5);
    for (const x of [-0.5, -1, -1.8]) expect(Math.abs(inflowRatio(x, FOLD_MU - step, FOLD_MU) - inflowRatio(x, FOLD_MU, FOLD_MU))).toBeLessThan(1e-5);
    expect(Math.abs(axialInflow(-2 - step) - axialInflow(-2 + step))).toBeLessThan(0.03);
    expect(axialInflow(0)).toBe(1);
  });

  it("puts the vortex ring at x = -1 in axial descent and nowhere else", () => {
    expect(vrsIntensity(-1, 0, FOLD_MU)).toBe(1);
    for (const [x, m] of [[0, 0], [-2, 0], [0.5, 0], [-1, FOLD_MU], [-1, 1]]) expect(vrsIntensity(x, m, FOLD_MU)).toBe(0);
  });

  it("balances blade elements and momentum at its answer", () => {
    const sigmaAHalf = 0.16, theta0 = 0.42, thetaTw = -0.33;
    for (const [lc, mu] of [[0, 0], [0.05, 0.1], [-0.04, 0.05], [0.02, 0.4]]) {
      const li = solveInflow(lc, mu, 1, 0, FOLD_MU, sigmaAHalf, theta0, thetaTw);
      const pitch = theta0 * (1 / 3 + mu * mu / 2) + thetaTw * (1 + mu * mu) / 4;
      const ct = sigmaAHalf * (pitch - 0.5 * (lc + li));
      const lh = Math.sqrt(ct / 2);
      expect(Math.abs(li - lh * inflowRatio(lc / lh, mu / lh, FOLD_MU))).toBeLessThan(1e-6);
    }
  });
});

describe("rotor plant", () => {
  it("hovers the Mini 2 near 9,000 RPM and the Air 2S near 6,200 RPM", () => {
    const mini = createAirframe(MINI_2, 1 / 120).hoverOmega(MINI_2_MASS.massKg, 1.225) * RPM;
    const air = createAirframe(AIR_2S, 1 / 120).hoverOmega(AIR_2S_MASS.massKg, 1.225) * RPM;
    expect(mini).toBeGreaterThan(8500);
    expect(mini).toBeLessThan(9500);
    expect(air).toBeGreaterThan(5800);
    expect(air).toBeLessThan(6600);
    const { ct, cq } = staticCoefficients(MINI_2.propeller);
    expect(ct).toBeGreaterThan(0.01);
    expect(ct).toBeLessThan(0.02);
    // Figure of merit = C_T^1.5 / (sqrt 2 C_Q): a small propeller's 0.5 to 0.7.
    const merit = ct ** 1.5 / (Math.SQRT2 * cq);
    expect(merit).toBeGreaterThan(0.45);
    expect(merit).toBeLessThan(0.75);
  });

  it("makes the static thrust the airframe promises, up the shaft", () => {
    const p = plant();
    p.step();
    const promised = p.airframe.thrustCoefficient * 1.225 * 940 * 940;
    expect(p.rotor(0, O.THRUST)).toBeCloseTo(promised, 10);
    expect(p.output[O.FZ]).toBeCloseTo(-4 * promised, 10);
    expect(p.output[O.FX]).toBe(0);
  });

  it("follows the speed command with the rotor's time constant", () => {
    const p = plant(MINI_2, 0);
    for (let i = 0; i < 4; i++) p.input[I.globals + i] = 1000;
    // 0.025 s at 120 Hz: three steps reach 1 - e^-1.
    for (let i = 0; i < 3; i++) p.step();
    expect(p.rotor(0, O.OMEGA)).toBeCloseTo(1000 * (1 - Math.exp(-1)), 6);
    p.input[I.globals] = 1e6;
    p.step();
    expect(p.rotor(0, O.OMEGA)).toBeLessThanOrEqual(MINI_2.propeller.maxRpm / RPM);
  });

  it("turns the body against clockwise rotors spinning up, and holds yaw at a steady trim", () => {
    const p = plant(MINI_2, 900);
    p.step();
    expect(Math.abs(p.output[O.N])).toBeLessThan(1e-12);
    for (const i of [0, 3]) p.input[I.globals + i] = 1100;
    p.step();
    // Front-left and rear-right turn clockwise: their torque and spin-up yaw the body left.
    expect(p.output[O.N]).toBeLessThan(0);
  });

  it("adds the gyroscopic moment of the rotors' net angular momentum", () => {
    const moments = (inertia: number, p: number, q: number) => {
      const plantNow = plant(MINI_2, 900);
      plantNow.config[PLANT.config.ROTOR_INERTIA] = inertia;
      for (const i of [1, 2]) { plantNow.input[I.globals + i] = 700; plantNow.state[S.globals + i * S.stride + S.OMEGA] = 700; }
      plantNow.input[I.P] = p;
      plantNow.input[I.Q] = q;
      plantNow.step();
      return [plantNow.output[O.L], plantNow.output[O.M]];
    };
    // Net clockwise momentum (body +z): pitching up rolls the body left, -q H; rolling right pitches it up, +p H.
    const momentum = 2 * MINI_2.propeller.inertiaKgM2 * (900 - 700);
    const pitching = moments(MINI_2.propeller.inertiaKgM2, 0, 1), still = moments(0, 0, 1);
    expect(pitching[0] - still[0]).toBeCloseTo(-momentum, 12);
    const rolling = moments(MINI_2.propeller.inertiaKgM2, 1, 0), stillRolling = moments(0, 1, 0);
    expect(rolling[1] - stillRolling[1]).toBeCloseTo(momentum, 12);
  });

  it("drags the hub against edgewise motion, linearly at low advance ratio", () => {
    const p = plant(MINI_2, 940);
    const h = (u: number) => { p.input[I.U] = u; p.state[S.RNG] = 1; p.step(); return { h: p.rotor(0, O.H_FORCE), fx: p.output[O.FX], l: p.output[O.L] }; };
    const slow = h(0.5), double = h(1);
    expect(slow.fx).toBeLessThan(0);
    expect(double.h / slow.h).toBeGreaterThan(1.9);
    expect(double.h / slow.h).toBeLessThan(2.1);
    // Clockwise and counter-clockwise hub moments cancel in roll on a left-right symmetric quad.
    expect(double.l).toBeCloseTo(0, 12);
  });

  it("makes more thrust near the ground, and none of it without one", () => {
    const thrust = (agl: number) => { const p = plant(MINI_2, 940); p.input[I.AGL] = agl; p.step(); return { t: p.rotor(0, O.THRUST), ge: p.rotor(0, O.GROUND_EFFECT) }; };
    const free = thrust(NaN), high = thrust(20), low = thrust(0.06);
    expect(Math.abs(high.t / free.t - 1)).toBeLessThan(1e-3);
    expect(low.ge).toBeLessThan(0.9);
    expect(low.t / free.t).toBeGreaterThan(1.05);
    expect(groundEffect(0.1, 0, 0.87, 0.49, 0, 0, 0, 0.06, 0.17, 0.11, 1)).toBe(1);
    // Below half a radius the image model is held where it is.
    expect(groundEffect(-5, 0, 0, 1, 0, 0, 0, 0.06, 0.17, 0.11, 1)).toBe(groundEffect(0.03, 0, 0, 1, 0, 0, 0, 0.06, 0.17, 0.11, 1));
  });

  it("fluctuates in the vortex ring state and is steady out of it", () => {
    const spread = (w: number) => {
      const p = plant(MINI_2, 940);
      p.input[I.W] = w;
      const values: number[] = [];
      for (let i = 0; i < 600; i++) { p.step(); values.push(p.rotor(0, O.THRUST)); }
      const mean = values.reduce((a, b) => a + b) / values.length;
      return { sd: Math.sqrt(values.reduce((a, b) => a + (b - mean) ** 2, 0) / values.length) / mean, vrs: p.rotor(0, O.VRS) };
    };
    const hover = spread(0), ring = spread(4);
    expect(hover.sd).toBeLessThan(1e-12);
    expect(ring.vrs).toBeGreaterThan(0.5);
    expect(ring.sd).toBeGreaterThan(0.005);
    expect(ring.sd).toBeLessThan(0.1);
  });

  it("refuses buffers that do not fit and values out of range, before writing", () => {
    const p = plant();
    expect(() => checkPlant(p.config, p.input.subarray(1), p.state, p.output)).toThrow(KernelError);
    p.state[S.RNG] = 0;
    const before = Float64Array.from(p.output);
    let status = 0;
    try { p.step(); } catch (error) { status = (error as KernelError).status; }
    expect(status).toBe(3);
    expect(p.output).toEqual(before);
    expect(xorshift(1)).toBe(270369);
  });
});
