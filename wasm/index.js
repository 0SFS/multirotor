import { CONTROL, PLANT, controlLengths, plantLengths } from "../src/layout.ts";

const MAX_MEMORY = 32 * 1024 * 1024;
const LITTLE_ENDIAN = new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;
const ABI_VERSION = 1;
const functions = ["mr_abi_version", "mr_last_error", "mr_heap_allocations", "mr_alloc", "mr_free", "mr_plant", "mr_control"];

/** Row layouts in f64 values, shared with the TypeScript kernels. */
export { CONTROL, PLANT };
export const ERRORS = Object.freeze({ INVALID: 1, ALLOCATION: 2, INPUT: 3 });

/** No binary fetch or WASM work happens on import. No hidden JS fallback. */
export async function loadMultirotor({ wasmUrl = new URL("./single/multirotor.wasm", import.meta.url), fetch: fetchBytes = globalThis.fetch } = {}) {
  const response = await fetchBytes(wasmUrl);
  if (!response.ok) throw new Error(`Multirotor WASM request failed (${response.status})`);
  return instantiateMultirotor(await response.arrayBuffer());
}

export async function instantiateMultirotor(bytes) {
  const result = await WebAssembly.instantiate(bytes, {});
  return createMultirotorCore((result.instance ?? result).exports);
}

function count(value) {
  if (!Number.isSafeInteger(value) || value < 1 || value > PLANT.maxRotors) throw new Error(`Rotors must be an integer from 1 to ${PLANT.maxRotors}`);
  return value;
}

/**
 * Raw ABI adapter. Each kernel owns four buffers in WASM memory, [config,
 * input, state, output], from creation to dispose(); its views are cached
 * Float64Arrays replaced only if the memory grows, which stepping never does.
 * Fill the views, call run(), read the output: no step allocates in JS or Rust.
 */
export function createMultirotorCore(exports) {
  if (!LITTLE_ENDIAN) throw new Error("WASM multirotor views need a little-endian host");
  for (const name of functions) if (typeof exports?.[name] !== "function") throw new Error(`Invalid multirotor ABI: ${name}`);
  if (!(exports.memory instanceof WebAssembly.Memory) || exports.mr_abi_version() !== ABI_VERSION) throw new Error("Unsupported multirotor ABI");
  let closed = false;
  const kernels = new Set();
  function buffer() {
    const value = exports.memory.buffer;
    if (!(value instanceof ArrayBuffer) || value.byteLength === 0 || value.byteLength > MAX_MEMORY) throw new Error("Invalid WASM multirotor memory");
    return value;
  }
  function active() { if (closed) throw new Error("WASM multirotor core is disposed"); }
  function failure(operation, status = exports.mr_last_error()) {
    const error = new Error(`WASM multirotor ${operation} failed (${status})`);
    error.status = status;
    return error;
  }
  function region(length, owned) {
    const pointer = exports.mr_alloc(length);
    if (!pointer) throw failure("allocation");
    const entry = { pointer, length, memory: null, view: null };
    owned.push(entry);
    const bytes = buffer().byteLength;
    if (!Number.isSafeInteger(pointer) || pointer % 8 || pointer > bytes || length * 8 > bytes - pointer) throw new Error("Invalid WASM multirotor allocation");
    for (const other of owned) if (other !== entry && pointer < other.pointer + other.length * 8 && other.pointer < pointer + length * 8) throw new Error("Overlapping WASM multirotor allocations");
    return entry;
  }
  function view(entry) {
    const memory = exports.memory.buffer;
    if (entry.memory !== memory) { buffer(); entry.view = new Float64Array(memory, entry.pointer, entry.length); entry.memory = memory; }
    return entry.view;
  }
  function release(owned) {
    const errors = [];
    for (const { pointer, length } of owned.reverse()) {
      try { const status = exports.mr_free(pointer, length); if (status !== 0) throw new Error(`Multirotor buffer release failed (${status})`); }
      catch (error) { errors.push(error); }
    }
    if (errors.length) throw new AggregateError(errors, "WASM multirotor cleanup failed");
  }
  function kernel(lengths, step, operation) {
    active();
    const owned = [];
    let regions;
    try { regions = [lengths.config, lengths.input, lengths.state, lengths.output].map(length => region(length, owned)); }
    catch (error) { try { release(owned); } catch (cleanup) { throw new AggregateError([error, cleanup], error.message); } throw error; }
    const [config, input, state, output] = regions;
    let disposed = false;
    const result = {
      get config() { return view(config); },
      get input() { return view(input); },
      get state() { return view(state); },
      get output() { return view(output); },
      run() {
        active();
        if (disposed) throw new Error("WASM multirotor kernel is disposed");
        const status = step(config.pointer, config.length, input.pointer, input.length, state.pointer, state.length, output.pointer, output.length);
        if (status !== 0) throw failure(operation, status);
      },
      dispose() {
        if (disposed) return;
        disposed = true; kernels.delete(result);
        release(owned);
      },
    };
    kernels.add(result);
    return result;
  }
  buffer();
  return {
    backend: "single",
    abiVersion: ABI_VERSION,
    /** Heap allocations made in WASM since instantiation. */
    heapAllocations: () => exports.mr_heap_allocations(),
    /** rotorPlant.ts's stepRotorPlant on buffers of PLANT's layout. */
    createPlant(rotors) { return kernel(plantLengths(count(rotors)), exports.mr_plant, "rotor plant"); },
    /** flightController.ts's stepFlightController on buffers of CONTROL's layout. */
    createController(rotors) { return kernel(controlLengths(count(rotors)), exports.mr_control, "flight controller"); },
    dispose() {
      if (closed) return;
      const errors = [];
      for (const live of [...kernels]) { try { live.dispose(); } catch (error) { errors.push(error); } }
      closed = true;
      if (errors.length) throw new AggregateError(errors, "WASM multirotor cleanup failed");
    },
  };
}
