# Multirotor

Rotor aerodynamics and the flight controller for 0SFS drones. JSBSim flies
the rigid body. This package gives it the rotors' force and moment each step,
and the flight controller turns stick commands into rotor speeds.

The simulation goes down to the spinning propeller and no further. Each
rotor's speed follows its command with a first-order lag, which stands in
for the ESC, the motor and the battery. Per rotor and per step, the plant
computes:

- **Blade element and momentum.** Thrust, H-force, shaft torque and the hub's
  rolling moment, from a twisted constant-pitch blade. The inflow is solved
  so blade elements and momentum agree.
- **Inflow in every state.** Glauert's momentum inflow in climb and forward
  flight, the empirical axial curve through the vortex ring state, then the
  windmill brake state. The pieces blend so the inflow has no jumps.
- **Vortex ring buffet.** A correlated fluctuation of the inflow, strongest at
  a descent rate equal to the hover induced velocity and gone in forward
  flight.
- **Ground effect.** Sanchez-Cuevas et al.'s multirotor ground effect: each
  rotor's image, its neighbours' images and the body's.
- **Reaction torque and gyroscopic moment.** The rotor's drag torque and
  spin-up react on the body. The body turning the rotors' net angular
  momentum makes a gyroscopic moment.

The flight controller has four loops:

- **Velocity**, with jerk-limited setpoint shaping and an integral.
- **Attitude**, which tilts the thrust axis first and then turns the heading,
  from a heading or a yaw-rate command.
- **Body rate**, with an integral and the gyroscopic term.
- **Allocation**, the minimum-norm inverse of the rotor layout. Thrust and
  tilt come first and yaw gives way.

It scales its static-thrust model by the thrust it measured last, as an
accelerometer would see it. This is incremental dynamic inversion, so climbs
and fast flight need no gain schedule.

| Module | Does |
| --- | --- |
| `src/airframe.ts` | An airframe spec (propeller, rotor layout, CG, gains) into the kernels' constant buffers |
| `src/rotorPlant.ts` | The rotor plant kernel |
| `src/flightController.ts` | The flight controller kernel |
| `src/layout.ts` | The buffer layouts both kernels and their Rust twins share |
| `src/multirotor.ts` | The runtime: one airframe's plant and controller on TypeScript, moved to the WASM core once it loads |
| `wasm/` | The kernels in Rust, identical bit for bit ([README](wasm/README.md)) |

## Use

This is the component mod `0sfs/multirotor` (`public/0sfs-mod.json`). The
`0SFS/Quadcopter` mod depends on it, so every drone in the default modpack
brings it in. The local package is `@0sfs/multirotor`; the drone feature
imports it lazily after a drone is applied.

```ts
import { createAirframe } from "@0sfs/multirotor/airframe";
import { createMultirotor } from "@0sfs/multirotor/multirotor";
import { loadMultirotor } from "@0sfs/multirotor/wasm";

const multirotor = createMultirotor(createAirframe(spec, 1 / 120));
loadMultirotor().then(core => multirotor.useCore(core), error => multirotor.refuseCore(String(error)));
// Each step: sample the aircraft, command it, read the wrench to apply.
multirotor.step(sample, command);
const wrench = multirotor.plant.output; // PLANT.output.FX to N: body axes, N and N m
```

Body axes are x forward, y right and z down. Spin +1 is clockwise seen from
above. The wrench acts at the CG for the coming step.

## Tests

The fly app depends on this package and runs its tests. From the `fly`
directory, `npm test` includes them. `tests/rigidBody.ts` is a minimal flat-earth
rigid body that stands in for JSBSim, so the closed loop can be tested here.

Topics: `0sfs-mod` and `0sfs-component-mod`. [Modding guide](https://github.com/0SFS/docs/blob/main/modding.md) describes discovery and installation.

## Community

- [Code of conduct](https://github.com/0SFS/.github/blob/main/CODE_OF_CONDUCT.md)
- [Contributing](https://github.com/0SFS/.github/blob/main/CONTRIBUTING.md)
- [License](LICENSE)
- [Security](https://github.com/0SFS/.github/blob/main/SECURITY.md)
