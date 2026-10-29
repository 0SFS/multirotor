import type { AirframeSpec, FlightControlSpec } from "../src/airframe";

/** Test airframes: the DJI Mini 2 and Air 2S as their 0SFS manifests give them. */
export const CONTROL: FlightControlSpec = {
  kv: 2.5, kvi: 2, viLimitH: 8, viLimitV: 4,
  shapeGain: 2, accelMaxH: 6, accelMaxV: 4, jerkMax: 30,
  kAtt: 7, rateMaxDeg: 250,
  kPsi: 3, yawLeadDeg: 30,
  kRate: 18, kRi: 5, riLimit: 10,
};

export const MINI_2: AirframeSpec = {
  propeller: {
    radiusM: 0.0597, blades: 2, chordM: 0.011, pitchM: 0.06604, zeroLiftDeg: -1.5, liftSlope: 5.5, profileDrag: 0.035,
    inertiaKgM2: 8e-7, timeConstantS: 0.025, maxRpm: 13300, idleRpm: 3100,
  },
  rotors: [
    { position: "front-left", hubM: [-0.0648, -0.0877, 0.027], spin: 1 },
    { position: "front-right", hubM: [-0.0648, 0.0877, 0.027], spin: -1 },
    { position: "rear-left", hubM: [0.0604, -0.0844, 0.005], spin: -1 },
    { position: "rear-right", hubM: [0.0604, 0.0844, 0.005], spin: 1 },
  ],
  cgM: [-0.0022, 0, -0.005],
  control: CONTROL,
};
export const MINI_2_MASS = { massKg: 0.249, ixx: 3.2e-4, iyy: 4.341e-4, izz: 6.932e-4 };

export const AIR_2S: AirframeSpec = {
  propeller: {
    radiusM: 0.0917, blades: 2, chordM: 0.0155, pitchM: 0.09652, zeroLiftDeg: -1.5, liftSlope: 5.5, profileDrag: 0.035,
    inertiaKgM2: 4.6e-6, timeConstantS: 0.035, maxRpm: 9200, idleRpm: 2200,
  },
  rotors: [
    { position: "front-left", hubM: [-0.0871, -0.1278, 0.0348], spin: 1 },
    { position: "front-right", hubM: [-0.0871, 0.1278, 0.0348], spin: -1 },
    { position: "rear-left", hubM: [0.093, -0.1135, 0.0135], spin: -1 },
    { position: "rear-right", hubM: [0.093, 0.1135, 0.0135], spin: 1 },
  ],
  cgM: [0.003, 0, -0.005],
  control: CONTROL,
};
export const AIR_2S_MASS = { massKg: 0.595, ixx: 1.643e-3, iyy: 2.246e-3, izz: 3.599e-3 };
