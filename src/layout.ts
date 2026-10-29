/**
 * Buffer layouts shared by the TypeScript kernels and their Rust twins, in
 * f64 slots. Every buffer is globals first, then one row per rotor.
 */

/** Rotor plant: rotor speed, blade-element aerodynamics and the wrench on the body. */
export const PLANT = Object.freeze({
  config: Object.freeze({
    /** Step, s. */
    DT: 0,
    /** exp(-dt / tau), the rotor speed decay over one step. */
    DECAY: 1,
    /** tau / dt * (1 - DECAY): the mean of that decay over the step. */
    AVERAGE: 2,
    /** Rotor speed limit, rad/s. */
    OMEGA_MAX: 3,
    /** Blade tip radius, m. */
    RADIUS: 4,
    /** Solidity times lift slope over two, sigma * a / 2. */
    SIGMA_A_HALF: 5,
    /** Blade pitch at the axis and its linear twist to the tip, rad. */
    THETA_0: 6,
    THETA_TW: 7,
    /** Solidity times profile drag coefficient. */
    SIGMA_CD0: 8,
    /** Fraction of the rigid-blade hub rolling moment the flexible blades pass on. */
    HUB_GAIN: 9,
    /** Rotor (propeller and motor bell) polar inertia, kg m^2. */
    ROTOR_INERTIA: 10,
    /** Vortex ring inflow fluctuation at full intensity, fraction of the induced velocity. */
    VRS_AMPLITUDE: 11,
    /** The fluctuation's first-order noise: exp(-dt / correlation time), and the gain for unit variance. */
    NOISE_DECAY: 12,
    NOISE_GAIN: 13,
    /** Edgewise advance ratio over hover inflow above which momentum theory has one descent solution. */
    FOLD_MU: 14,
    /** Ground effect: adjacent rotor spacing and hub distance from the centre, m, and the body gain. */
    GE_SPACING: 15,
    GE_BODY: 16,
    GE_BODY_GAIN: 17,
    /** Tip speed below which the rotor makes no aerodynamic force, m/s. */
    MIN_TIP_SPEED: 18,
    globals: 19,
    /** Per rotor: hub position from the CG in body axes (x forward, y right, z down), m, and spin (+1 clockwise seen from above). */
    X: 0, Y: 1, Z: 2, SPIN: 3,
    stride: 4,
  }),
  input: Object.freeze({
    /** Body velocity relative to the air, m/s. */
    U: 0, V: 1, W: 2,
    /** Body rates, rad/s. */
    P: 3, Q: 4, R: 5,
    /** Air density, kg/m^3. */
    RHO: 6,
    /** The body-to-NED matrix's down row: each body axis's downward component. */
    C31: 7, C32: 8, C33: 9,
    /** CG height above the ground, m; NaN for no ground. */
    AGL: 10,
    globals: 11,
    /** Per rotor: commanded speed, rad/s. */
    OMEGA_CMD: 0,
    stride: 1,
  }),
  state: Object.freeze({
    /** xorshift32 state, an integer in [1, 2^32). */
    RNG: 0,
    globals: 1,
    /** Per rotor: speed (rad/s) and the vortex ring fluctuation's noise. */
    OMEGA: 0, NOISE: 1,
    stride: 2,
  }),
  output: Object.freeze({
    /** Total rotor force in body axes, N, and moment about the CG, N m. */
    FX: 0, FY: 1, FZ: 2, L: 3, M: 4, N: 5,
    globals: 6,
    /** Per rotor: thrust, H-force (N), shaft torque (N m), speed at the end and mean over the step (rad/s), induced velocity (m/s), ground effect inflow factor, vortex ring intensity. */
    THRUST: 0, H_FORCE: 1, TORQUE: 2, OMEGA: 3, OMEGA_MEAN: 4, INDUCED: 5, GROUND_EFFECT: 6, VRS: 7,
    stride: 8,
  }),
  maxRotors: 8,
});

/** Flight controller: velocity, attitude and rate loops, then allocation to rotor speeds. */
export const CONTROL = Object.freeze({
  config: Object.freeze({
    DT: 0,
    G: 1,
    /** Velocity loop: proportional (1/s) and integral (1/s^2) gains, integral limits (m/s^2). */
    K_V: 2, K_VI: 3, VI_LIMIT_H: 4, VI_LIMIT_V: 5,
    /** Setpoint shaping: gain (1/s), acceleration limits (m/s^2), jerk limit (m/s^3). */
    SHAPE_GAIN: 6, ACCEL_MAX_H: 7, ACCEL_MAX_V: 8, JERK_MAX: 9,
    /** Tilt loop gain (1/s) and roll/pitch rate limit (rad/s). */
    K_ATT: 10, RATE_MAX_RP: 11,
    /** Heading gain (1/s) and how far the heading setpoint may lead the aircraft (rad). */
    K_PSI: 12, YAW_LEAD: 13,
    /** Rate loop: proportional (1/s) and integral (1/s^2) gains, integral limit (rad/s^2). */
    K_RATE: 14, K_RI: 15, RI_LIMIT: 16,
    /** Static thrust per rotor = THRUST_COEFF * rho * omega^2; idle and maximum speed, rad/s. */
    THRUST_COEFF: 17, OMEGA_IDLE: 18, OMEGA_MAX: 19,
    globals: 20,
    /** Per rotor: the allocation row mapping [thrust, roll, pitch, yaw moment] to this rotor's thrust. */
    A_THRUST: 0, A_ROLL: 1, A_PITCH: 2, A_YAW: 3,
    stride: 4,
  }),
  input: Object.freeze({
    /** Inertial velocity NED, m/s. */
    VN: 0, VE: 1, VD: 2,
    P: 3, Q: 4, R: 5,
    /** Body-to-NED matrix, row major. */
    C11: 6, C12: 7, C13: 8, C21: 9, C22: 10, C23: 11, C31: 12, C32: 13, C33: 14,
    /** Heading, rad. */
    PSI: 15,
    /** Mass (kg), principal inertia (kg m^2) and air density (kg/m^3), live from the flight model. */
    MASS: 16, IXX: 17, IYY: 18, IZZ: 19, RHO: 20,
    /** Commanded velocity NED, m/s. */
    TARGET_VN: 21, TARGET_VE: 22, TARGET_VD: 23,
    /** 0: YAW is a rate (rad/s); 1: YAW is a heading (rad). */
    YAW_MODE: 24, YAW: 25,
    /** 0 motors stopped, 1 ground idle, 2 flying. */
    MODE: 26,
    /** tan of the tilt limit, and the yaw rate limit (rad/s), for the flight mode. */
    TAN_TILT_MAX: 27, YAW_RATE_MAX: 28,
    /**
     * Thrust the rotors make over what static thrust would at their speed, as
     * an accelerometer sees it (incremental dynamic inversion): about 0.7 in a
     * fast climb, above 1 in forward flight. 1 for none.
     */
    THRUST_EFF: 29,
    length: 30,
  }),
  state: Object.freeze({
    /** Shaped velocity setpoint and its acceleration, NED. */
    VSP_N: 0, VSP_E: 1, VSP_D: 2, ASP_N: 3, ASP_E: 4, ASP_D: 5,
    /** Velocity integral, m/s^2. */
    VI_N: 6, VI_E: 7, VI_D: 8,
    /** Heading setpoint, rad. */
    PSI_D: 9,
    /** Rate integral, rad/s^2. */
    RI_P: 10, RI_Q: 11, RI_R: 12,
    /** 1 when the last step was limited, which holds the integrators. */
    SATURATED: 13,
    /** 1 once the setpoints have been taken from the aircraft. */
    ACTIVE: 14,
    length: 15,
  }),
  output: Object.freeze({
    /** Collective thrust (N), commanded acceleration NED (m/s^2), commanded body rates (rad/s), saturation. */
    THRUST: 0, AN: 1, AE: 2, AD: 3, P_CMD: 4, Q_CMD: 5, R_CMD: 6, SATURATED: 7,
    globals: 8,
    /** Per rotor: commanded speed, rad/s. */
    OMEGA_CMD: 0,
    stride: 1,
  }),
  MODE_STOPPED: 0,
  MODE_IDLE: 1,
  MODE_FLYING: 2,
  YAW_RATE: 0,
  YAW_HEADING: 1,
});

/** Buffer lengths for `rotors` rotors. */
export function plantLengths(rotors: number) {
  return {
    config: PLANT.config.globals + rotors * PLANT.config.stride,
    input: PLANT.input.globals + rotors * PLANT.input.stride,
    state: PLANT.state.globals + rotors * PLANT.state.stride,
    output: PLANT.output.globals + rotors * PLANT.output.stride,
  };
}

export function controlLengths(rotors: number) {
  return {
    config: CONTROL.config.globals + rotors * CONTROL.config.stride,
    input: CONTROL.input.length,
    state: CONTROL.state.length,
    output: CONTROL.output.globals + rotors * CONTROL.output.stride,
  };
}

/** The rotor count a plant config of `length` slots holds, or 0 if none fits. */
export function rotorCount(length: number, globals: number, stride: number, max = PLANT.maxRotors): number {
  const count = (length - globals) / stride;
  return Number.isInteger(count) && count >= 1 && count <= max ? count : 0;
}
