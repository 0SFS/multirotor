import { readFileSync } from "node:fs";
import { afterEach, beforeEach, describe, expect, it } from "vitest";
import { createAirframe } from "../src/airframe";
import { CONTROL } from "../src/layout";
import { createMultirotor } from "../src/multirotor";
import { instantiateMultirotor, type MultirotorCore } from "../wasm/index.js";
import { AIR_2S, AIR_2S_MASS, MINI_2, MINI_2_MASS } from "./airframes";
import { advance, level, type World } from "./rigidBody";

let core: MultirotorCore;
beforeEach(async () => { core = await instantiateMultirotor(readFileSync(new URL("../wasm/single/multirotor.wasm", import.meta.url))); });
afterEach(() => core.dispose());

describe("WASM multirotor core", () => {
  it("flies the same flight as the TypeScript, bit for bit, without allocating", () => {
    for (const [spec, mass, cdA] of [[MINI_2, MINI_2_MASS, [0.005, 0.006, 0.013]], [AIR_2S, AIR_2S_MASS, [0.0085, 0.01, 0.022]]] as const) {
      const airframe = createAirframe(spec, 1 / 120);
      const typescript = createMultirotor(airframe);
      const wasm = createMultirotor(airframe);
      wasm.useCore(core);
      expect(wasm.status().backend).toBe("wasm");
      const world: World = { ...mass, rho: 1.2, cdA: [...cdA], wind: [6, 3, 0], restM: 0.04 };
      const bodies = [level(1, 0.04), level(1, 0.04)];
      const before = core.heapAllocations();
      for (let i = 0; i < 120 * 12; i++) {
        const t = i / 120;
        const command = {
          vn: t > 3 && t < 8 ? 12 : 0, ve: t > 5 && t < 9 ? -6 : 0, vd: t < 0.5 ? 0 : t < 3 ? -3 : t < 9 ? 0 : 2.5,
          yawMode: t < 6 ? CONTROL.YAW_RATE : CONTROL.YAW_HEADING, yaw: t < 6 ? 1 : -2,
          mode: t < 0.5 ? CONTROL.MODE_IDLE : CONTROL.MODE_FLYING, tanTiltMax: Math.tan(0.6), yawRateMaxRad: 2.2,
        };
        advance(bodies[0], world, typescript, command);
        advance(bodies[1], world, wasm, command);
        expect(Array.from(wasm.plant.output)).toEqual(Array.from(typescript.plant.output));
        expect(Array.from(wasm.control.output)).toEqual(Array.from(typescript.control.output));
      }
      expect(core.heapAllocations()).toBe(before);
      expect(wasm.status().backend).toBe("wasm");
      expect(bodies[1]).toEqual(bodies[0]);
      wasm.dispose();
    }
  });

  it("refuses bad input with status 3 and leaves the runtime on the core", () => {
    const multirotor = createMultirotor(createAirframe(MINI_2, 1 / 120));
    multirotor.useCore(core);
    const world: World = { ...MINI_2_MASS, rho: 1.2, cdA: [0.005, 0.006, 0.013], wind: [0, 0, 0], restM: NaN };
    expect(() => advance(level(0, 10), { ...world, rho: NaN }, multirotor, {
      vn: 0, ve: 0, vd: 0, yawMode: 0, yaw: 0, mode: 2, tanTiltMax: 0.5, yawRateMaxRad: 1,
    })).toThrow(expect.objectContaining({ status: 3 }));
    expect(multirotor.status().backend).toBe("wasm");
    multirotor.dispose();
  });
});
