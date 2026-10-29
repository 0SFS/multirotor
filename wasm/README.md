# multirotor/wasm

The rotor plant and flight controller in Rust, built for
`wasm32-unknown-unknown` with `simd128` and no wasm-bindgen. Each kernel is
the arithmetic of its TypeScript function in `../src`, in the same order, so
the two agree bit for bit. The host does all trigonometry and exponentials
and passes the results in.

| export | does | reference |
| --- | --- | --- |
| `mr_plant` | rotor speed, inflow, forces and moments for 1 to 8 rotors | `rotorPlant.ts` `stepRotorPlant` |
| `mr_control` | velocity, tilt, rate loops and allocation | `flightController.ts` `stepFlightController` |
| `mr_alloc`, `mr_free`, `mr_last_error`, `mr_heap_allocations`, `mr_abi_version` | buffers, errors, the allocation count, ABI 1 | |

Both kernels take `(config, config_len, input, input_len, state, state_len,
output, output_len)` as f64 pointers and lengths. The layouts are in
`../src/layout.ts`, which `index.js` re-exports as `PLANT` and `CONTROL`.
A kernel checks everything before it writes anything. Its status is
0 for success, 1 for buffers that do not fit together, 2 for an allocation
failure, and 3 for a non-finite or out-of-range input.

## Use

```js
import { loadMultirotor } from "@0sfs/multirotor/wasm";

const core = await loadMultirotor();            // or instantiateMultirotor(bytes)
multirotor.useCore(core);                       // a runtime from ../src/multirotor.ts
```

`useCore` copies the runtime's buffers into WASM memory. From then on a step
writes the inputs through `Float64Array` views, runs the kernel and reads the
outputs. Neither JS nor Rust allocates, and `heapAllocations()` lets a test
prove it. If a kernel fails for any reason but its input, the runtime copies
the buffers back and runs the TypeScript for good. `status()` says why.

There is no worker. The kernels are synchronous, bounded and take
microseconds, so they run wherever JSBSim's fixed step runs.

## Build and check

```sh
node scripts/build.mjs   # offline, --locked; writes single/multirotor.wasm and build-metadata.json
node scripts/check.mjs   # the receipt, then parity with the TypeScript
node scripts/bench.mjs   # a step's cost on both backends
```

`check.mjs` compares the two backends bit for bit with `Object.is`:

- **Random cases:** 4,000 cases of 4 steps each, on a Mini 2, an Air 2S, a
  hexacopter and an octocopter at 100 Hz. Together they cover the vortex
  ring, ground effect, saturation and idle.
- **Closed loop:** two 40-second closed-loop flights through
  `tests/rigidBody.ts`.
- **Error codes:** each one, and that nothing is written on an error.
- **Allocations:** zero while stepping.
- **Fallback:** a trapping kernel makes the runtime fall back.

The last run matched 992,282 plant values, 456,135 controller values and
480,000 closed-loop values, with no failures.

## Performance

`bench.mjs` times one full runtime step for the Mini 2. Node 26 on an Apple
M5, on battery:

| case | TypeScript | WASM |
| --- | --- | --- |
| hover | 4.7 µs | 9.4 µs |
| sport, 16 m/s | 3.7 µs | 12.1 µs |
| vortex ring | 10.9 µs | 16.8 µs |

V8's optimised TypeScript is faster than this WASM. Building without
`simd128`, turning off Liftoff, or calling the kernel directly makes no
difference, so the cost is the kernel itself rather than the call. Both
backends use less than 0.2% of the 8.3 ms a 120 Hz step allows. The WASM
build stays for its fixed cost, which no JIT can deoptimise, and for moving
the plant off the main thread later.
