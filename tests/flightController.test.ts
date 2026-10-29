import { describe, expect, it } from "vitest";
import { allocation, createAirframe } from "../src/airframe";
import { CONTROL, PLANT } from "../src/layout";
import { createMultirotor, type CoreLike, type FlightCommand } from "../src/multirotor";
import { AIR_2S, AIR_2S_MASS, MINI_2, MINI_2_MASS } from "./airframes";
import { advance, euler, level, tilt, type World } from "./rigidBody";

const DEG = Math.PI / 180;
const MINI_WORLD: World = { ...MINI_2_MASS, rho: 1.225, cdA: [0.005, 0.006, 0.013], wind: [0, 0, 0], restM: NaN };
const AIR_WORLD: World = { ...AIR_2S_MASS, rho: 1.225, cdA: [0.0085, 0.01, 0.022], wind: [0, 0, 0], restM: NaN };
const hold = (overrides: Partial<FlightCommand> = {}): FlightCommand => ({
  vn: 0, ve: 0, vd: 0, yawMode: CONTROL.YAW_RATE, yaw: 0, mode: CONTROL.MODE_FLYING,
  tanTiltMax: Math.tan(25 * DEG), yawRateMaxRad: 60 * DEG, ...overrides,
});

function fly(seconds: number, command: FlightCommand | ((t: number) => FlightCommand), spec = MINI_2, world = MINI_WORLD, heightM = 50) {
  const multirotor = createMultirotor(createAirframe(spec, 1 / 120));
  const body = level(0, heightM);
  let maxTilt = 0;
  for (let i = 0; i < seconds * 120; i++) {
    advance(body, world, multirotor, typeof command === "function" ? command(i / 120) : command);
    maxTilt = Math.max(maxTilt, tilt(body));
  }
  return { body, multirotor, maxTilt };
}

describe("allocation", () => {
  it("inverts the effectiveness of the rotor layout", () => {
    const airframe = createAirframe(MINI_2, 1 / 120);
    const rows = allocation(airframe.arms, airframe.spins, airframe.torqueRatio);
    const effectiveness = [airframe.arms.map(() => 1), airframe.arms.map(arm => -arm[1]), airframe.arms.map(arm => arm[0]), airframe.spins.map(s => -s * airframe.torqueRatio)];
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
      const value = effectiveness[r].reduce((sum, e, i) => sum + e * rows[i][c], 0);
      expect(value).toBeCloseTo(r === c ? 1 : 0, 9);
    }
  });

  it("refuses a layout that cannot yaw", () => {
    const same = { ...MINI_2, rotors: MINI_2.rotors.map(rotor => ({ ...rotor, spin: 1 as const })) };
    expect(() => createAirframe(same, 1 / 120)).toThrow(/cannot control/);
  });
});

describe("flight controller", () => {
  it("stops or idles the rotors on the ground", () => {
    const multirotor = createMultirotor(createAirframe(MINI_2, 1 / 120));
    const body = level(0, 0);
    advance(body, MINI_WORLD, multirotor, hold({ mode: CONTROL.MODE_IDLE }));
    const out = multirotor.control.output;
    for (let i = 0; i < 4; i++) expect(out[CONTROL.output.globals + i]).toBe(multirotor.airframe.omegaIdle);
    advance(body, MINI_WORLD, multirotor, hold({ mode: CONTROL.MODE_STOPPED }));
    for (let i = 0; i < 4; i++) expect(out[CONTROL.output.globals + i]).toBe(0);
  });

  it("holds a hover still and level", () => {
    const { body } = fly(5, hold());
    expect(Math.abs(body.d + 50)).toBeLessThan(0.1);
    expect(Math.hypot(body.vn, body.ve, body.vd)).toBeLessThan(0.02);
    expect(tilt(body)).toBeLessThan(0.01);
  });

  it("reaches the commanded speed within the tilt limit and holds height", () => {
    for (const [spec, world, top, tiltDeg] of [[MINI_2, MINI_WORLD, 10, 25], [MINI_2, MINI_WORLD, 16, 40], [AIR_2S, AIR_WORLD, 19, 35]] as const) {
      const { body, maxTilt } = fly(10, hold({ vn: top, tanTiltMax: Math.tan(tiltDeg * DEG) }), spec, world);
      expect(body.vn).toBeGreaterThan(0.97 * top);
      expect(body.vn).toBeLessThan(1.02 * top);
      expect(maxTilt).toBeLessThan((tiltDeg + 1.5) * DEG);
      expect(Math.abs(body.d + 50)).toBeLessThan(1.5);
    }
  });

  it("climbs at 5 m/s within 2 s and descends through the vortex ring under control", () => {
    const climb = fly(2, hold({ vd: -5 }));
    expect(climb.body.vd).toBeLessThan(-4.5);
    const descent = fly(8, hold({ vd: 3 }), MINI_2, MINI_WORLD, 100);
    expect(descent.body.vd).toBeGreaterThan(2.8);
    expect(descent.body.vd).toBeLessThan(3.2);
    expect(descent.maxTilt).toBeLessThan(8 * DEG);
    expect(Math.hypot(descent.body.vn, descent.body.ve)).toBeLessThan(0.5);
  });

  it("turns at the commanded yaw rate and onto a commanded heading", () => {
    const rate = fly(2, hold({ yaw: 60 * DEG }));
    expect(rate.body.r).toBeCloseTo(60 * DEG, 2);
    const heading = fly(3, hold({ yawMode: CONTROL.YAW_HEADING, yaw: 120 * DEG, yawRateMaxRad: 130 * DEG }));
    expect(euler(heading.body).psi).toBeCloseTo(120 * DEG, 2);
  });

  it("leans into a 10 m/s wind and holds still in it", () => {
    const { body } = fly(10, hold(), MINI_2, { ...MINI_WORLD, wind: [10, 0, 0] });
    expect(Math.hypot(body.vn, body.ve)).toBeLessThan(0.05);
    // The air moves north: the aircraft tilts back, south, into it.
    expect(euler(body).theta).toBeGreaterThan(5 * DEG);
    expect(euler(body).theta).toBeLessThan(20 * DEG);
  });

  it("measures the thrust lost in a climb and makes it up", () => {
    const { multirotor } = fly(3, hold({ vd: -5 }));
    expect(multirotor.control.input[CONTROL.input.THRUST_EFF]).toBeLessThan(0.9);
    expect(multirotor.control.input[CONTROL.input.THRUST_EFF]).toBeGreaterThan(0.5);
  });
});

describe("runtime", () => {
  it("falls back to the TypeScript with the state when a WASM kernel fails, and says why", () => {
    const airframe = createAirframe(MINI_2, 1 / 120);
    const multirotor = createMultirotor(airframe);
    const kernel = (lengths: { config: number; input: number; state: number; output: number }, fail: boolean) => ({
      config: new Float64Array(lengths.config), input: new Float64Array(lengths.input), state: new Float64Array(lengths.state), output: new Float64Array(lengths.output),
      run() { if (fail) throw new Error("memory access out of bounds"); }, dispose() {},
    });
    const core: CoreLike = {
      createPlant: () => kernel({ config: 19 + 16, input: 11 + 4, state: 1 + 8, output: 6 + 32 }, true),
      createController: () => kernel({ config: 20 + 16, input: 30, state: 15, output: 8 + 4 }, false),
    };
    multirotor.useCore(core);
    expect(multirotor.status()).toEqual({ backend: "wasm", reason: null });
    const body = level(0, 10);
    advance(body, MINI_WORLD, multirotor, hold());
    expect(multirotor.status().backend).toBe("typescript");
    expect(multirotor.status().reason).toMatch(/out of bounds/);
    for (let i = 0; i < 12; i++) advance(body, MINI_WORLD, multirotor, hold());
    expect(multirotor.plant.output[PLANT.output.FZ]).toBeLessThan(0);
    // Once failed, the core is not taken again.
    multirotor.useCore(core);
    expect(multirotor.status().backend).toBe("typescript");
  });

  it("passes bad input through rather than falling back", () => {
    const multirotor = createMultirotor(createAirframe(MINI_2, 1 / 120));
    const body = level(0, 10);
    expect(() => advance(body, { ...MINI_WORLD, massKg: 0 }, multirotor, hold())).toThrow(/out of range/);
    expect(multirotor.status().reason).toMatch(/not loaded/);
  });
});
