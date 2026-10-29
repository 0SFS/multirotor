import type { CONTROL as ControlLayout, PLANT as PlantLayout } from "../src/layout.ts";

export const PLANT: typeof PlantLayout;
export const CONTROL: typeof ControlLayout;
export const ERRORS: Readonly<{ INVALID: 1; ALLOCATION: 2; INPUT: 3 }>;

/**
 * One kernel's four buffers in WASM memory. The views stay valid until
 * dispose(); re-read the getters after creating another kernel.
 */
export interface MultirotorKernel {
  readonly config: Float64Array;
  readonly input: Float64Array;
  readonly state: Float64Array;
  readonly output: Float64Array;
  /** Throws an Error with `.status` (ERRORS) if the kernel refuses the buffers. */
  run(): void;
  dispose(): void;
}

export interface MultirotorCore {
  readonly backend: "single";
  readonly abiVersion: 1;
  heapAllocations(): number;
  createPlant(rotors: number): MultirotorKernel;
  createController(rotors: number): MultirotorKernel;
  dispose(): void;
}

export function loadMultirotor(options?: { wasmUrl?: URL | string; fetch?: typeof fetch }): Promise<MultirotorCore>;
export function instantiateMultirotor(bytes: BufferSource): Promise<MultirotorCore>;
export function createMultirotorCore(exports: WebAssembly.Exports): MultirotorCore;
