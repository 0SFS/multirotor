// Times one 120 Hz multirotor step (controller and rotor plant, four rotors,
// the host's input writes included) for the Mini 2 on the TypeScript kernels
// and on single/multirotor.wasm: hovering, in sport forward flight, and in a
// vortex ring descent, where the inflow solve works hardest. Prints the median
// and fastest time per step. A laptop's numbers move with power source,
// thermal state and load; compare runs only on the same machine in the same
// state. Correctness is scripts/check.mjs's job, not this script's.
import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { registerHooks } from "node:module";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

registerHooks({
  resolve(specifier, context, next) {
    try { return next(specifier, context); } catch (error) {
      if (error?.code !== "ERR_MODULE_NOT_FOUND" || !/^\.\.?\//.test(specifier) || specifier.endsWith(".ts")) throw error;
      return next(`${specifier}.ts`, context);
    }
  },
});
const root = fileURLToPath(new URL("../", import.meta.url));
const load = file => import(pathToFileURL(path.join(root, "..", file)).href);
const { instantiateMultirotor } = await import("../index.js");
const { createAirframe } = await load("src/airframe.ts");
const { createMultirotor } = await load("src/multirotor.ts");
const { MINI_2, MINI_2_MASS } = await load("tests/airframes.ts");
const core = await instantiateMultirotor(readFileSync(path.join(root, "single/multirotor.wasm")));
const quiet = (command, args) => { try { return execFileSync(command, args, { encoding: "utf8" }).trim(); } catch { return "unknown"; } };

const airframe = createAirframe(MINI_2, 1 / 120);
const hover = airframe.hoverOmega(MINI_2_MASS.massKg, 1.225);
const sample = (u, w, theta) => ({ vn: u, ve: 0, vd: w, u, v: 0, w, p: 0.01, q: -0.02, r: 0.03, phi: 0.01, theta, psi: 0.2,
  massKg: MINI_2_MASS.massKg, ixx: MINI_2_MASS.ixx, iyy: MINI_2_MASS.iyy, izz: MINI_2_MASS.izz, rho: 1.225, aglM: 20 });
const command = (vn, vd) => ({ vn, ve: 0, vd, yawMode: 0, yaw: 0.2, mode: 2, tanTiltMax: 0.84, yawRateMaxRad: 2.3 });
const cases = { hover: [sample(0, 0, 0), command(0, 0)], sport: [sample(16, 3, -0.5), command(16, 0)], vortexRing: [sample(0, 3, 0), command(0, 3)] };
const ROUNDS = 25, STEPS = 2000;
const results = {};
for (const backend of ["typescript", "wasm"]) {
  for (const [name, [input, wanted]] of Object.entries(cases)) {
    const runtime = createMultirotor(airframe);
    if (backend === "wasm") runtime.useCore(core);
    const state = runtime.plant.state;
    for (let i = 0; i < 4; i++) state[1 + 2 * i] = hover;
    const times = [];
    for (let round = 0; round < ROUNDS; round++) {
      const start = process.hrtime.bigint();
      for (let i = 0; i < STEPS; i++) runtime.step(input, wanted);
      times.push(Number(process.hrtime.bigint() - start) / 1000 / STEPS);
    }
    times.sort((a, b) => a - b);
    results[`${backend} ${name}`] = { medianMicroseconds: +times[ROUNDS >> 1].toFixed(2), fastestMicroseconds: +times[0].toFixed(2), backend: runtime.status().backend };
    runtime.dispose();
  }
}
core.dispose();
console.log(JSON.stringify({
  host: { cpu: quiet("sysctl", ["-n", "machdep.cpu.brand_string"]), node: process.versions.node, power: quiet("pmset", ["-g", "batt"]).split("\n")[0] },
  stepsPerRound: STEPS, rounds: ROUNDS, results,
}, null, 2));
