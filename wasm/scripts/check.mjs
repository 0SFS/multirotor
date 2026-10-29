// Checks single/multirotor.wasm against its build receipt and the live
// TypeScript in ../src (read only): the rotor plant and the flight controller
// bit for bit, on random buffers and closed loop through a rigid body (the
// TypeScript's createMultirotor, once on its own kernels and once given this
// core, which must never fall back); zero heap allocations while stepping;
// and the error codes, with nothing written on error. Correctness only:
// timings come from scripts/bench.mjs.
import { createHash } from "node:crypto";
import { existsSync, readFileSync, readdirSync } from "node:fs";
import { registerHooks } from "node:module";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const root = fileURLToPath(new URL("../", import.meta.url));
const packageRoot = path.join(root, "..");

// Receipt: every input and the binary are what build.mjs recorded.
const sha = file => createHash("sha256").update(readFileSync(path.join(root, file))).digest("hex");
const metadata = JSON.parse(readFileSync(path.join(root, "build-metadata.json"), "utf8"));
const crateFiles = directory => readdirSync(path.join(root, directory), { withFileTypes: true })
  .flatMap(entry => entry.isDirectory() ? crateFiles(path.posix.join(directory, entry.name)) : [path.posix.join(directory, entry.name)]);
const stale = [...Object.entries(metadata.inputs).filter(([file, hash]) => !existsSync(path.join(root, file)) || sha(file) !== hash).map(([file]) => file),
  ...crateFiles("crates").filter(file => !(file in metadata.inputs))];
const wasmFile = "single/multirotor.wasm";
if (stale.length || sha(wasmFile) !== metadata.outputs[wasmFile]?.sha256) {
  throw new Error(`Build receipt is stale (${stale.join(", ") || wasmFile}); run scripts/build.mjs`);
}

// The TypeScript imports its siblings without an extension.
registerHooks({
  resolve(specifier, context, next) {
    try { return next(specifier, context); } catch (error) {
      if (error?.code !== "ERR_MODULE_NOT_FOUND" || !/^\.\.?\//.test(specifier) || specifier.endsWith(".ts")) throw error;
      return next(`${specifier}.ts`, context);
    }
  },
});
const load = file => import(pathToFileURL(path.join(packageRoot, file)).href);
const { ERRORS, createMultirotorCore } = await import("../index.js");
const { CONTROL, PLANT, controlLengths, plantLengths } = await load("src/layout.ts");
const { stepRotorPlant, KernelError } = await load("src/rotorPlant.ts");
const { stepFlightController } = await load("src/flightController.ts");
const { createAirframe } = await load("src/airframe.ts");
const { createMultirotor } = await load("src/multirotor.ts");
const { MINI_2, MINI_2_MASS, AIR_2S, AIR_2S_MASS } = await load("tests/airframes.ts");
const { advance, level } = await load("tests/rigidBody.ts");

const { instance } = await WebAssembly.instantiate(readFileSync(path.join(root, wasmFile)), {});
const raw = instance.exports;
const core = createMultirotorCore(raw);

const failures = [];
let failureCount = 0;
function expect(condition, message) {
  if (!condition) { failureCount++; if (failures.length < 25) failures.push(message); }
  return condition;
}
const compared = { plant: 0, controller: 0, closedLoop: 0 };
function same(kind, label, expected, actual) {
  for (let i = 0; i < expected.length; i++) {
    compared[kind]++;
    if (!expect(Object.is(expected[i], actual[i]), `${kind} ${label}[${i}]: TypeScript ${expected[i]} vs WASM ${actual[i]}`)) return;
  }
}

// Deterministic inputs.
let seed = 0x4d494e32;
function rand() {
  seed = (seed + 0x6d2b79f5) >>> 0;
  let t = seed;
  t = Math.imul(t ^ (t >>> 15), t | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}
const uniform = (low, high) => low + (high - low) * rand();
const chance = p => rand() < p;
const pick = values => values[Math.floor(rand() * values.length)];

// A quad, the Air 2S, and a hexacopter and octocopter on a ring.
function ring(count, radius, propeller) {
  return {
    ...MINI_2,
    propeller,
    rotors: Array.from({ length: count }, (_, i) => {
      const angle = (i + 0.5) * 2 * Math.PI / count;
      return { position: `rotor-${i}`, hubM: [-radius * Math.cos(angle), radius * Math.sin(angle), 0.02], spin: i % 2 ? -1 : 1 };
    }),
  };
}
const airframes = [
  ["Mini 2", createAirframe(MINI_2, 1 / 120)],
  ["Air 2S", createAirframe(AIR_2S, 1 / 120)],
  ["hexacopter", createAirframe(ring(6, 0.25, { ...AIR_2S.propeller, radiusM: 0.12, chordM: 0.02, pitchM: 0.11 }), 1 / 120)],
  ["octocopter", createAirframe(ring(8, 0.4, { ...AIR_2S.propeller, radiusM: 0.15, chordM: 0.025, pitchM: 0.13, maxRpm: 7000, idleRpm: 1500 }), 1 / 100)],
];

function eulerMatrix(phi, theta, psi) {
  const sf = Math.sin(phi), cf = Math.cos(phi), st = Math.sin(theta), ct = Math.cos(theta), sp = Math.sin(psi), cp = Math.cos(psi);
  return [ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp, ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp, -st, sf * ct, cf * ct];
}

function plantCase(airframe, buffers) {
  const n = airframe.rotors;
  const I = PLANT.input, S = PLANT.state;
  buffers.config.set(airframe.plantConfig);
  const input = buffers.input;
  // Hover to fast forward flight, steep climbs and descents through the vortex ring and the windmill state.
  const scale = pick([0.5, 3, 10, 30]);
  input[I.U] = uniform(-scale, scale); input[I.V] = uniform(-scale, scale); input[I.W] = uniform(-scale, scale);
  if (chance(0.25)) { input[I.U] = uniform(-0.3, 0.3); input[I.V] = uniform(-0.3, 0.3); }
  input[I.P] = uniform(-6, 6); input[I.Q] = uniform(-6, 6); input[I.R] = uniform(-6, 6);
  input[I.RHO] = chance(0.05) ? 0 : uniform(0.6, 1.4);
  const c = eulerMatrix(uniform(-1.4, 1.4), uniform(-1.4, 1.4), uniform(-3, 3));
  input[I.C31] = c[6]; input[I.C32] = c[7]; input[I.C33] = c[8];
  input[I.AGL] = pick([NaN, uniform(-0.1, 0.1), uniform(0, 0.5), uniform(0, 3), uniform(0, 100)]);
  const omegaMax = airframe.omegaMax;
  for (let i = 0; i < n; i++) input[I.globals + i] = pick([0, uniform(-50, omegaMax + 200), uniform(0.8, 1.2) * airframe.hoverOmega(1, 1.2)]);
  const state = buffers.state;
  state[S.RNG] = 1 + Math.floor(rand() * 4294967295);
  for (let i = 0; i < n; i++) {
    state[S.globals + i * S.stride + S.OMEGA] = pick([0, uniform(0, omegaMax)]);
    state[S.globals + i * S.stride + S.NOISE] = uniform(-3, 3);
  }
}

function controllerCase(airframe, buffers) {
  const I = CONTROL.input, S = CONTROL.state;
  buffers.config.set(airframe.controlConfig);
  const input = buffers.input;
  const speed = pick([0.2, 5, 20]);
  input[I.VN] = uniform(-speed, speed); input[I.VE] = uniform(-speed, speed); input[I.VD] = uniform(-speed, speed);
  input[I.P] = uniform(-5, 5); input[I.Q] = uniform(-5, 5); input[I.R] = uniform(-5, 5);
  const psi = uniform(-Math.PI, Math.PI);
  const tiltRange = pick([0.05, 0.7, 3]);
  const c = eulerMatrix(uniform(-tiltRange, tiltRange), uniform(-Math.min(tiltRange, 1.5), Math.min(tiltRange, 1.5)), psi);
  for (let k = 0; k < 9; k++) input[I.C11 + k] = c[k];
  input[I.PSI] = psi;
  input[I.MASS] = uniform(0.2, 8); input[I.IXX] = uniform(1e-4, 0.2); input[I.IYY] = uniform(1e-4, 0.2); input[I.IZZ] = uniform(1e-4, 0.3);
  input[I.RHO] = uniform(0.6, 1.4);
  input[I.TARGET_VN] = uniform(-20, 20); input[I.TARGET_VE] = uniform(-20, 20); input[I.TARGET_VD] = uniform(-6, 6);
  input[I.YAW_MODE] = pick([CONTROL.YAW_RATE, CONTROL.YAW_HEADING]);
  input[I.YAW] = input[I.YAW_MODE] === CONTROL.YAW_HEADING ? uniform(-10, 10) : uniform(-6, 6);
  input[I.MODE] = pick([CONTROL.MODE_STOPPED, CONTROL.MODE_IDLE, CONTROL.MODE_FLYING, CONTROL.MODE_FLYING, CONTROL.MODE_FLYING]);
  input[I.TAN_TILT_MAX] = Math.tan(uniform(0.1, 0.8));
  input[I.YAW_RATE_MAX] = uniform(0.5, 5);
  input[I.THRUST_EFF] = uniform(0.3, 2);
  const state = buffers.state;
  for (let k = 0; k < S.length; k++) state[k] = uniform(-5, 5);
  state[S.PSI_D] = uniform(-12, 12);
  state[S.SATURATED] = pick([0, 1]);
  state[S.ACTIVE] = pick([0, 1, 1, 1]);
}

const copyOf = buffers => ({ config: Float64Array.from(buffers.config), input: Float64Array.from(buffers.input), state: Float64Array.from(buffers.state), output: Float64Array.from(buffers.output) });
const fresh = lengths => ({ config: new Float64Array(lengths.config), input: new Float64Array(lengths.input), state: new Float64Array(lengths.state), output: new Float64Array(lengths.output) });
function load4(kernel, buffers) {
  kernel.config.set(buffers.config); kernel.input.set(buffers.input); kernel.state.set(buffers.state); kernel.output.set(buffers.output);
}

// 1. Random buffers, several steps each so the state carries.
const CASES = 4000;
const STEPS = 4;
const coverage = { vortexRing: 0, groundEffect: 0, saturated: 0, idle: 0 };
const kernels = airframes.map(([, airframe]) => ({ plant: core.createPlant(airframe.rotors), control: core.createController(airframe.rotors) }));
for (let c = 0; c < CASES; c++) {
  const index = c % airframes.length;
  const [name, airframe] = airframes[index];
  const { plant, control } = kernels[index];
  const p = fresh(plantLengths(airframe.rotors));
  plantCase(airframe, p);
  load4(plant, p);
  const k = fresh(controlLengths(airframe.rotors));
  controllerCase(airframe, k);
  load4(control, k);
  for (let s = 0; s < STEPS; s++) {
    stepRotorPlant(p.config, p.input, p.state, p.output);
    plant.run();
    same("plant", `${name} case ${c} step ${s} state`, p.state, plant.state);
    same("plant", `${name} case ${c} step ${s} output`, p.output, plant.output);
    stepFlightController(k.config, k.input, k.state, k.output);
    control.run();
    same("controller", `${name} case ${c} step ${s} state`, k.state, control.state);
    same("controller", `${name} case ${c} step ${s} output`, k.output, control.output);
  }
  for (let i = 0; i < airframe.rotors; i++) {
    if (p.output[PLANT.output.globals + i * PLANT.output.stride + PLANT.output.VRS] > 0.1) coverage.vortexRing++;
    if (p.output[PLANT.output.globals + i * PLANT.output.stride + PLANT.output.GROUND_EFFECT] < 0.99) coverage.groundEffect++;
  }
  if (k.output[CONTROL.output.SATURATED] === 1) coverage.saturated++;
  if (k.input[CONTROL.input.MODE] !== CONTROL.MODE_FLYING) coverage.idle++;
}

// 2. Closed loop: the same flight on the TypeScript kernels and on this core.
const flights = [
  ["Mini 2 sport circuit", MINI_2, MINI_2_MASS, [0.005, 0.006, 0.013], [0, 0, 0]],
  ["Air 2S in wind", AIR_2S, AIR_2S_MASS, [0.0085, 0.01, 0.022], [8, -4, 0]],
];
const closedLoop = { steps: 0 };
const runtimes = flights.map(([, spec]) => {
  const airframe = createAirframe(spec, 1 / 120);
  const wasm = createMultirotor(airframe);
  wasm.useCore(core);
  return { typescript: createMultirotor(airframe), wasm };
});
// The runtimes' kernels exist before the count; stepping must add none.
const allocationsBefore = core.heapAllocations();
for (const [index, [name, , mass, cdA, wind]] of flights.entries()) {
  const { typescript, wasm } = runtimes[index];
  expect(wasm.status().backend === "wasm", `${name}: the core was refused (${wasm.status().reason})`);
  const world = { ...mass, rho: 1.18, cdA, wind, restM: 0.04 };
  const bodies = [level(0.3, 0.04), level(0.3, 0.04)];
  for (let i = 0; i < 120 * 40; i++) {
    const t = i / 120;
    // Idle on the ground, take off, sport circuit with yaw, descend through the vortex ring, land.
    const command = {
      vn: t > 4 && t < 14 ? 16 * Math.cos(t / 2) : 0, ve: t > 4 && t < 14 ? 16 * Math.sin(t / 2) : 0,
      vd: t < 1 ? 0 : t < 4 ? -4 : t < 14 ? 0 : t < 30 ? 3 : 1,
      yawMode: t < 20 ? CONTROL.YAW_RATE : CONTROL.YAW_HEADING, yaw: t < 20 ? Math.sin(t) * 2 : 2.5,
      mode: t < 1 || t > 38 ? CONTROL.MODE_IDLE : CONTROL.MODE_FLYING,
      tanTiltMax: Math.tan(40 * Math.PI / 180), yawRateMaxRad: 4.4,
    };
    advance(bodies[0], world, typescript, command);
    advance(bodies[1], world, wasm, command);
    same("closedLoop", `${name} t=${t.toFixed(3)} plant`, typescript.plant.output, wasm.plant.output);
    same("closedLoop", `${name} t=${t.toFixed(3)} controller`, typescript.control.output, wasm.control.output);
    closedLoop.steps++;
  }
  expect(wasm.status().backend === "wasm", `${name}: fell back (${wasm.status().reason})`);
  expect(Object.is(bodies[0].n, bodies[1].n) && Object.is(bodies[0].d, bodies[1].d), `${name}: the flights parted`);
  closedLoop[name] = { north: bodies[1].n, east: bodies[1].e, down: bodies[1].d };
}
const allocationsAfter = core.heapAllocations();
expect(allocationsAfter === allocationsBefore, `allocations while stepping: ${allocationsAfter - allocationsBefore}`);

// 3. Error codes, and nothing written on error.
const errors = {};
function status(expected, actual, label) {
  errors[label] = actual;
  expect(actual === expected, `${label}: status ${actual}, expected ${expected}`);
}
function tsStatus(run) {
  try { run(); return 0; } catch (error) { return error instanceof KernelError ? error.status : -1; }
}
{
  const airframe = airframes[0][1];
  const plant = core.createPlant(4);
  const p = fresh(plantLengths(4));
  plantCase(airframe, p);
  load4(plant, p);
  const args = [plant.config, plant.input, plant.state, plant.output].map(view => [view.byteOffset, view.length]);
  status(ERRORS.INVALID, raw.mr_plant(...args[0], ...args[1], ...args[2], args[3][0], args[3][1] - 1), "plant: short output");
  status(ERRORS.INVALID, raw.mr_plant(...args[0], ...args[1], ...args[2], ...args[2]), "plant: state passed as the output");
  status(ERRORS.INVALID, raw.mr_plant(args[0][0] + 8, args[0][1], ...args[1], ...args[2], ...args[3]), "plant: foreign pointer");
  status(ERRORS.INVALID, raw.mr_plant(...args[1], ...args[1], ...args[2], ...args[3]), "plant: input passed as the config");
  for (const [label, edit] of [
    ["plant: NaN velocity", () => { plant.input[PLANT.input.U] = NaN; }],
    ["plant: infinite AGL", () => { plant.input[PLANT.input.AGL] = Infinity; }],
    ["plant: rng 0", () => { plant.state[PLANT.state.RNG] = 0; }],
    ["plant: rng 2^32", () => { plant.state[PLANT.state.RNG] = 4294967296; }],
    ["plant: rng 1.5", () => { plant.state[PLANT.state.RNG] = 1.5; }],
    ["plant: negative density", () => { plant.input[PLANT.input.RHO] = -1; }],
  ]) {
    load4(plant, p);
    edit();
    const before = copyOf(plant);
    status(ERRORS.INPUT, raw.mr_plant(...args[0], ...args[1], ...args[2], ...args[3]), label);
    same("plant", `${label} state untouched`, before.state, plant.state);
    same("plant", `${label} output untouched`, before.output, plant.output);
    const ts = copyOf(plant);
    status(ERRORS.INPUT, tsStatus(() => stepRotorPlant(ts.config, ts.input, ts.state, ts.output)), `${label} (TypeScript)`);
  }
  plant.dispose();
  const control = core.createController(4);
  const k = fresh(controlLengths(4));
  controllerCase(airframe, k);
  const cargs = [control.config, control.input, control.state, control.output].map(view => [view.byteOffset, view.length]);
  status(ERRORS.INVALID, raw.mr_control(...cargs[0], ...cargs[1], cargs[2][0], cargs[2][1], ...cargs[1]), "controller: input passed as the output");
  for (const [label, edit] of [
    ["controller: mode 3", () => { control.input[CONTROL.input.MODE] = 3; }],
    ["controller: yaw mode 0.5", () => { control.input[CONTROL.input.YAW_MODE] = 0.5; }],
    ["controller: zero mass", () => { control.input[CONTROL.input.MASS] = 0; }],
    ["controller: zero thrust efficiency", () => { control.input[CONTROL.input.THRUST_EFF] = 0; }],
    ["controller: infinite integral", () => { control.state[CONTROL.state.VI_N] = -Infinity; }],
  ]) {
    load4(control, k);
    edit();
    const before = copyOf(control);
    status(ERRORS.INPUT, raw.mr_control(...cargs[0], ...cargs[1], ...cargs[2], ...cargs[3]), label);
    same("controller", `${label} state untouched`, before.state, control.state);
    same("controller", `${label} output untouched`, before.output, control.output);
    const ts = copyOf(control);
    status(ERRORS.INPUT, tsStatus(() => stepFlightController(ts.config, ts.input, ts.state, ts.output)), `${label} (TypeScript)`);
  }
  control.dispose();
  status(ERRORS.ALLOCATION, raw.mr_alloc(0) === 0 ? raw.mr_last_error() : -1, "zero-length allocation");
  status(ERRORS.ALLOCATION, raw.mr_alloc(3_000_000) === 0 ? raw.mr_last_error() : -1, "allocation past 16 MiB");
}

// 4. The runtime falls back, with the state, when a kernel fails for any reason but its input.
{
  const airframe = airframes[0][1];
  const runtime = createMultirotor(airframe);
  const kernelCore = {
    createPlant: rotors => {
      const kernel = core.createPlant(rotors);
      return {
        get config() { return kernel.config; }, get input() { return kernel.input; },
        get state() { return kernel.state; }, get output() { return kernel.output; },
        run() { throw new Error("unreachable trap"); },
        dispose: () => kernel.dispose(),
      };
    },
    createController: rotors => core.createController(rotors),
  };
  runtime.useCore(kernelCore);
  const world = { ...MINI_2_MASS, rho: 1.2, cdA: [0.005, 0.006, 0.013], wind: [0, 0, 0], restM: NaN };
  const body = level(0, 10);
  advance(body, world, runtime, { vn: 0, ve: 0, vd: 0, yawMode: 0, yaw: 0, mode: CONTROL.MODE_FLYING, tanTiltMax: 0.5, yawRateMaxRad: 1 });
  const status = runtime.status();
  expect(status.backend === "typescript" && /unreachable trap/.test(status.reason ?? ""), `a trapping kernel did not fall back: ${JSON.stringify(status)}`);
  expect(runtime.plant.output[PLANT.output.FZ] < 0, "the fallback step made no thrust");
  errors["runtime fallback"] = status.reason;
  runtime.dispose();
}

for (const { wasm } of runtimes) wasm.dispose();
for (const { plant, control } of kernels) { plant.dispose(); control.dispose(); }
core.dispose();

const summary = {
  wasm: { file: wasmFile, sha256: metadata.outputs[wasmFile].sha256, bytes: metadata.outputs[wasmFile].bytes, node: process.versions.node },
  bitForBit: { values: compared, cases: CASES, stepsPerCase: STEPS, airframes: airframes.map(([name]) => name), coverage, closedLoop },
  allocations: { whileStepping: allocationsAfter - allocationsBefore, closedLoopSteps: closedLoop.steps },
  errors,
  failures: failureCount,
};
console.log(JSON.stringify(summary, null, 2));
if (failureCount) {
  for (const failure of failures) console.error(`FAIL ${failure}`);
  process.exitCode = 1;
}
