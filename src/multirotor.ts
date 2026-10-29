import type { Airframe } from "./airframe";
import { stepFlightController } from "./flightController";
import { CONTROL, PLANT, controlLengths, plantLengths } from "./layout";
import { INPUT, stepRotorPlant } from "./rotorPlant";

export interface KernelBuffers {
  readonly config: Float64Array;
  readonly input: Float64Array;
  readonly state: Float64Array;
  readonly output: Float64Array;
}

/** A WASM kernel as `createMultirotorCore` in `wasm/index.js` makes it. */
export interface CoreKernel extends KernelBuffers {
  run(): void;
  dispose(): void;
}

/** The parts of the WASM core this module uses, so it imports no WASM itself. */
export interface CoreLike {
  createPlant(rotors: number): CoreKernel;
  createController(rotors: number): CoreKernel;
}

export interface MultirotorStatus {
  backend: "wasm" | "typescript";
  /** Why the TypeScript runs: not loaded yet, failed to load, or the kernel failed. */
  reason: string | null;
}

/** What the flight model says the aircraft is doing, SI units. */
export interface AircraftSample {
  /** Inertial velocity NED, m/s. */
  vn: number; ve: number; vd: number;
  /** Body velocity relative to the air, m/s. */
  u: number; v: number; w: number;
  /** Body rates, rad/s. */
  p: number; q: number; r: number;
  /** Euler angles, rad. */
  phi: number; theta: number; psi: number;
  massKg: number;
  /** Principal inertia, kg m^2. */
  ixx: number; iyy: number; izz: number;
  /** Air density, kg/m^3. */
  rho: number;
  /** CG height above the ground, m; NaN for none. */
  aglM: number;
}

/** What the pilot asks for, after the stick mapping. */
export interface FlightCommand {
  /** Velocity NED, m/s. */
  vn: number; ve: number; vd: number;
  /** CONTROL.YAW_RATE with `yaw` in rad/s, or CONTROL.YAW_HEADING with `yaw` in rad. */
  yawMode: number;
  yaw: number;
  /** CONTROL.MODE_STOPPED, MODE_IDLE or MODE_FLYING. */
  mode: number;
  tanTiltMax: number;
  yawRateMaxRad: number;
}

export interface Multirotor {
  readonly airframe: Airframe;
  /**
   * One step: the sample and command into the controller, its rotor speed
   * commands into the plant. Read the force and moment for the coming step
   * from `plant.output`. Allocates nothing.
   */
  step(sample: AircraftSample, command: FlightCommand): void;
  /** The live buffers. Re-read after `useCore`, which swaps them. */
  readonly plant: KernelBuffers;
  readonly control: KernelBuffers;
  /** Runs the controller on `control`: input to rotor speed commands. */
  runControl(): void;
  /** Runs the plant on `plant`: rotor speed commands to force and moment. */
  runPlant(): void;
  /** Rotors stopped, setpoints and integrators cleared. */
  reset(): void;
  /** Moves the buffers into the WASM core and runs there from now on. */
  useCore(core: CoreLike): void;
  /** Keeps the TypeScript for good, with the reason. */
  refuseCore(reason: string): void;
  status(): MultirotorStatus;
  dispose(): void;
}

export const DEFAULT_SEED = 0x9e3779b9;
/** The thrust efficiency estimate's time constant, s, and its bounds. */
const EFFICIENCY_TIME_S = 0.1;
const EFFICIENCY_MIN = 0.3;
const EFFICIENCY_MAX = 3;

function typescriptBuffers(lengths: { config: number; input: number; state: number; output: number }): KernelBuffers {
  return {
    config: new Float64Array(lengths.config),
    input: new Float64Array(lengths.input),
    state: new Float64Array(lengths.state),
    output: new Float64Array(lengths.output),
  };
}

function copy(from: KernelBuffers, to: KernelBuffers): void {
  to.config.set(from.config);
  to.input.set(from.input);
  to.state.set(from.state);
  to.output.set(from.output);
}

/**
 * The rotor plant and controller for one airframe on the TypeScript kernels,
 * moved to the WASM core once it is given. If a WASM kernel fails for any
 * reason but its input, the buffers come back and the TypeScript runs for
 * good; `status()` says why.
 */
export function createMultirotor(airframe: Airframe, seed = DEFAULT_SEED): Multirotor {
  const n = airframe.rotors;
  const local = { plant: typescriptBuffers(plantLengths(n)), control: typescriptBuffers(controlLengths(n)) };
  local.plant.config.set(airframe.plantConfig);
  local.control.config.set(airframe.controlConfig);
  let wasm: { plant: CoreKernel; control: CoreKernel } | null = null;
  let reason: string | null = "The WASM core has not loaded yet.";
  // Thrust over static thrust at the rotors' speed, as an accelerometer would
  // measure it, filtered: the controller's incremental dynamic inversion.
  let efficiency = 1;
  const efficiencyGain = 1 - Math.exp(-airframe.dt / EFFICIENCY_TIME_S);
  function reset(buffers: { plant: KernelBuffers; control: KernelBuffers }) {
    efficiency = 1;
    buffers.plant.state.fill(0);
    buffers.plant.state[PLANT.state.RNG] = seed >>> 0 || DEFAULT_SEED;
    buffers.plant.output.fill(0);
    buffers.control.state.fill(0);
    buffers.control.output.fill(0);
  }
  reset(local);
  function fallBack(error: unknown) {
    if (!wasm) return;
    copy(wasm.plant, local.plant);
    copy(wasm.control, local.control);
    const failed = wasm;
    wasm = null;
    reason = `The WASM kernel failed: ${error instanceof Error ? error.message : String(error)}`;
    try { failed.plant.dispose(); failed.control.dispose(); } catch { /* the TypeScript already has the buffers */ }
  }
  function run(kernel: "plant" | "control", typescript: () => void) {
    if (wasm) {
      try { wasm[kernel].run(); return; } catch (error) {
        if ((error as { status?: number }).status === INPUT) throw error;
        fallBack(error);
      }
    }
    typescript();
  }
  const runControl = () => run("control", () => stepFlightController(local.control.config, local.control.input, local.control.state, local.control.output));
  const runPlant = () => run("plant", () => stepRotorPlant(local.plant.config, local.plant.input, local.plant.state, local.plant.output));
  const K = CONTROL.input;
  const P = PLANT.input;
  return {
    airframe,
    get plant() { return wasm ? wasm.plant : local.plant; },
    get control() { return wasm ? wasm.control : local.control; },
    step(sample, command) {
      const sphi = Math.sin(sample.phi), cphi = Math.cos(sample.phi);
      const stheta = Math.sin(sample.theta), ctheta = Math.cos(sample.theta);
      const spsi = Math.sin(sample.psi), cpsi = Math.cos(sample.psi);
      // Body to NED, Rz(psi) Ry(theta) Rx(phi).
      const c31 = -stheta, c32 = sphi * ctheta, c33 = cphi * ctheta;
      const last = (wasm ? wasm.plant : local.plant).output;
      let thrust = 0, staticThrust = 0;
      for (let i = 0; i < n; i++) {
        const o = PLANT.output.globals + i * PLANT.output.stride;
        const omega = last[o + PLANT.output.OMEGA_MEAN];
        thrust += last[o + PLANT.output.THRUST];
        staticThrust += airframe.thrustCoefficient * sample.rho * omega * omega;
      }
      if (staticThrust > 0.05 * sample.massKg) {
        const measured = thrust / staticThrust;
        efficiency += efficiencyGain * ((measured < EFFICIENCY_MIN ? EFFICIENCY_MIN : measured > EFFICIENCY_MAX ? EFFICIENCY_MAX : measured) - efficiency);
      }
      const control = wasm ? wasm.control.input : local.control.input;
      control[K.VN] = sample.vn; control[K.VE] = sample.ve; control[K.VD] = sample.vd;
      control[K.P] = sample.p; control[K.Q] = sample.q; control[K.R] = sample.r;
      control[K.C11] = ctheta * cpsi;
      control[K.C12] = sphi * stheta * cpsi - cphi * spsi;
      control[K.C13] = cphi * stheta * cpsi + sphi * spsi;
      control[K.C21] = ctheta * spsi;
      control[K.C22] = sphi * stheta * spsi + cphi * cpsi;
      control[K.C23] = cphi * stheta * spsi - sphi * cpsi;
      control[K.C31] = c31; control[K.C32] = c32; control[K.C33] = c33;
      control[K.PSI] = sample.psi;
      control[K.MASS] = sample.massKg;
      control[K.IXX] = sample.ixx; control[K.IYY] = sample.iyy; control[K.IZZ] = sample.izz;
      control[K.RHO] = sample.rho;
      control[K.TARGET_VN] = command.vn; control[K.TARGET_VE] = command.ve; control[K.TARGET_VD] = command.vd;
      control[K.YAW_MODE] = command.yawMode;
      control[K.YAW] = command.yaw;
      control[K.MODE] = command.mode;
      control[K.TAN_TILT_MAX] = command.tanTiltMax;
      control[K.YAW_RATE_MAX] = command.yawRateMaxRad;
      control[K.THRUST_EFF] = efficiency;
      runControl();
      // The run may have fallen back to the TypeScript: read the buffers again.
      const commands = (wasm ? wasm.control : local.control).output;
      const plant = (wasm ? wasm.plant : local.plant).input;
      plant[P.U] = sample.u; plant[P.V] = sample.v; plant[P.W] = sample.w;
      plant[P.P] = sample.p; plant[P.Q] = sample.q; plant[P.R] = sample.r;
      plant[P.RHO] = sample.rho;
      plant[P.C31] = c31; plant[P.C32] = c32; plant[P.C33] = c33;
      plant[P.AGL] = sample.aglM;
      for (let i = 0; i < n; i++) plant[P.globals + i * P.stride + P.OMEGA_CMD] = commands[CONTROL.output.globals + i * CONTROL.output.stride + CONTROL.output.OMEGA_CMD];
      runPlant();
    },
    runControl,
    runPlant,
    reset() { reset(wasm ?? local); },
    useCore(core) {
      if (wasm || reason?.startsWith("The WASM kernel failed")) return;
      let plant: CoreKernel | null = null;
      try {
        plant = core.createPlant(n);
        const control = core.createController(n);
        copy(local.plant, plant);
        copy(local.control, control);
        wasm = { plant, control };
        reason = null;
      } catch (error) {
        plant?.dispose();
        reason = `The WASM core could not take the airframe: ${error instanceof Error ? error.message : String(error)}`;
      }
    },
    refuseCore(why) { if (!wasm) reason = why; },
    status: () => ({ backend: wasm ? "wasm" : "typescript", reason }),
    dispose() {
      const live = wasm;
      wasm = null;
      live?.plant.dispose();
      live?.control.dispose();
    },
  };
}
