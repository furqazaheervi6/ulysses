# Ulysses Firmware for Flight Controller: documentation

**As of 22 Nov 2026.** Branch `dev/furqazaheervi6/controller-handle` (head `4a174fa`), PR #17 on `furqazaheervi6/ulysses`, merged into `main` as `23f7b1b`. This page is in issue order and matches the code on that branch. For status, open questions and who to ask, see the status report.

## Where the code lives

| File | What it holds |
|---|---|
| `include/ulysses/lib/controls/flight_controller.h` | The handle, the state, reference and command structs, the fault bits, the frame notes and the three API functions |
| `lib/controls/flight_controller.c` | `flight_controller_init`, `flight_controller_set_gains`, `flight_controller_step` (the control law is a TODO) |
| `include/ulysses/lib/controls/flight_config.h` | `fc_config_t`: vehicle, environment, torque gains and gimbal gains |
| `include/ulysses/lib/controls/pid.h`, `lib/controls/pid.c` | The single-axis and 3-axis PID with integrator clamping |
| `include/ulysses/lib/controls/gimbal.h`, `lib/controls/gimbal.c` | The conversions between the gimbal angles and the thrust direction, with the sign convention |
| `include/ulysses/lib/math/quaternion.h`, `lib/math/quaternion.c` | The quaternion helpers. The `.c` file only includes the header, so the library build compiles it |

All of these are in `ulysses_lib`, so CI compiles them for the ARM target. CI does not run any tests.

## Conventions

- Quaternions are Hamilton, scalar-first `(w, v)`, and `q` maps body coordinates to world coordinates.
- A rotation in the world frame left-multiplies (`q' = dq * q`). A rotation in the body frame right-multiplies (`q' = q * dq`).
- Normalize a quaternion after every nominal state update.
- Attitude error: `q_err = q_des * conj(q_meas)`, a rotation in the world frame. Torque command: `tau_cmd = tau_gyro + phi`, where `phi` is the rotation-vector form of `q_err` and `tau_gyro = (MMOI * omega) x omega`.
- Units are SI, with angles in radians.

## #1 Replace hidden static state with a `flight_controller_t` handle

**Problem.** `init` took only a config and `step` took no handle, so the integrator and derivative state had to be file-level statics. That means one controller per image, tests that leak into each other (an integrator wound up by test A changes test B), and a `step` that looks pure but is not. Two RTOS tasks running the same image would also share unprotected statics, which is a race.

**Change.** The caller owns a `flight_controller_t`, and `init` and `step` take it:

```c
typedef struct flight_controller {
    fc_config_t config;        ///< Copy of the configuration given to flight_controller_init()
    pid3_state_t torque_pid;   ///< Torque controller integrators
    pid_state_t gimbal_pid[2]; ///< Gimbal controller integrators (x, y)

    atomic_uint faults;        ///< Sticky FC_FAULT_* bits, cleared by init

    // Triple buffer handing gains from the writer (set_gains) to the reader (step) without locks
    fc_gains_t gains_slots[3];
    atomic_uint gains_latest;
    unsigned gains_write_idx;
    unsigned gains_read_idx;
} flight_controller_t;
```

**Status.** Done and closed. Since #10 the handle holds only the PID integrators. The old `torque_prev_err` and `gimbal_prev_err` arrays are gone, because the derivative now comes from a caller-supplied error rate.

## #2 Config ownership, and tuning gains live

**Problem.** The header did not say whether `init` copies the config or keeps the pointer. A kept pointer can dangle (the caller built the config as a local variable) or change mid-step (a debug or telemetry task edits the caller's struct), so a step could use half-old, half-new gains.

**Change.** `init` copies the config (about 240 bytes), so the controller is self-contained. Changing gains later does not go through `init`, because that would zero the integrators and jolt the output. Instead `flight_controller_set_gains` queues new gains, and `step` applies them at the start of its next call. Only the gains change. The mass, inertia, pivot and limits are not tuned live, and the integrators are kept.

**Threading.** If one task calls `step` while another calls `set_gains`, a half-written gain matrix could be read. The fix is a lock-free pending-gains buffer (three slots and an atomic index), so the control loop never blocks:

```c
void flight_controller_set_gains(flight_controller_t *fc,
                                 const fc_torque_controller_config_t *torque,
                                 const fc_gimbal_controller_config_t *gimbal)
{
    fc_gains_t *slot = &fc->gains_slots[fc->gains_write_idx];
    slot->torque = *torque;
    slot->gimbal = *gimbal;

    // Publish the written slot and take ownership of the slot it replaces
    unsigned prev = atomic_exchange(&fc->gains_latest, fc->gains_write_idx | FC_GAINS_DIRTY);
    fc->gains_write_idx = prev & FC_GAINS_INDEX_MASK;
}
```

In `step`:

```c
    if (atomic_load(&fc->gains_latest) & FC_GAINS_DIRTY) {
        unsigned prev = atomic_exchange(&fc->gains_latest, fc->gains_read_idx);
        fc->gains_read_idx = prev & FC_GAINS_INDEX_MASK;

        const fc_gains_t *gains = &fc->gains_slots[fc->gains_read_idx];
        fc->config.torque_gains = gains->torque;
        fc->config.gimbal_gains = gains->gimbal;
    }
```

If `set_gains` is called twice before a step, the later gains win.

**Known gap.** `set_gains` checks only for NULL pointers. It does not check that the gains are finite or within limits.

**Status.** Done and closed.

## #3 Reset all controller state in `init`

`init` zeroes the whole struct with one `memset` and then copies the config in. A field added later is cleared automatically, so a forgotten reset can no longer make a test depend on run order.

```c
    memset(fc, 0, sizeof(*fc));
    fc->config = *config;
    atomic_init(&fc->faults, 0);
    /* layout check, see #4 */
    fc->gains_write_idx = 0;
    fc->gains_read_idx = 1;
    atomic_init(&fc->gains_latest, 2);
```

The `atomic_uint` is zeroed by `memset` and then set with `atomic_init`. That works on this platform, and the `atomic_init` call is the part that follows the rules. Re-running `init` while another task uses the controller is unsafe (see #5).

**Status.** Done and closed.

## #4 Guard against a bad `dt_s`

**Problem.** The integral term accumulates `error * dt_s` and the derivative term divides by `dt_s`. With `dt_s = 0` there is no accumulation or a divide by zero, with `dt_s < 0` the integrator runs backwards and the derivative flips sign, and a tiny `dt_s` gives a huge derivative spike.

**Decision.** A bad `dt_s` is a programming or timing fault that should never happen, so it is a runtime assert plus a sticky fault flag. A static assert cannot check a measured value, so static asserts are only for compile-time constants.

```c
    // Written so NaN also fails. Flag first: the assert handler does not return.
    if (!(dt_s > 0.0f)) {
        atomic_fetch_or(&fc->faults, FC_FAULT_BAD_DT);
    }
    ULYSSES_ASSERT(dt_s > 0.0f, "Time step must be positive");

    // A late tick is recoverable: flag it and cap dt so one slow step cannot jump the integrator
    float dt = dt_s;
    if (dt > FC_DT_MAX_S) {
        atomic_fetch_or(&fc->faults, FC_FAULT_MISSED_TICK);
        dt = FC_DT_MAX_S;
    }
```

- `FC_FAULT_BAD_DT` (bit 0): `dt_s` was zero, negative or NaN.
- `FC_FAULT_MISSED_TICK` (bit 1): `dt_s` was above `FC_DT_MAX_US`, which is 3 seconds. The control law uses the capped `dt`, not `dt_s`.
- `FC_FAULT_BAD_GAINS_LAYOUT` (bit 2): `init` found the gains buffer constants invalid. This is a runtime check that replaced two static asserts.
- `faults` is sticky: only `init` clears it. If a live "currently faulting" indicator is wanted, that is a different step.
- A bad `dt_s` still halts the flight computer: on the target the assert handler disables interrupts and spins. Whether that is acceptable in flight, or the controller should hold its last command, is a question for the team.
- The 3 s limit is loose. It needs the real loop rate.
- Since #10 the derivative no longer divides by `dt`, so the "no lower bound" concern is moot for the PID.

**Status.** Done and closed.

## #5 Which task owns the controller

- `init` and `step` must be called from a single control task and never run concurrently with each other or themselves.
- `set_gains` may be called from one other task. Only one task may call it.
- `faults` is an `atomic_uint`, so a telemetry task can read it while the control loop runs. The three places that set a bit use `atomic_fetch_or`.

**Status.** Done and closed. Reopen, or file a new issue, when the control-task wiring exists.

## #6 Take the shortest rotation

`q` and `-q` are the same rotation, but only the one with `w >= 0` is the short way round (`w = 0` is a half turn, 180 degrees). `quaternion_shortest_path` returns `-q` if `w < 0` and `q` otherwise:

```c
static inline quaternion_t quaternion_shortest_path(quaternion_t q)
{
    if (q.w < 0.0f) {
        for (int i = 0; i < 4; i++) {
            q.q[i] = -q.q[i];
        }
    }

    return q;
}
```

**Status.** Done and closed. Nothing calls it from the controller yet.

## #7 Frames and gimbal sign conventions

The header comment in `flight_controller.h` states only what the team docs define: Hamilton scalar-first, body to world, the attitude error, the torque command, and the gimbal angle formulas `theta_x = -atan(t_des.y / t_des.z)` and `theta_y = asin(t_des.x)`.

**Sign convention.** The architecture page's `t_hat = [sin(theta_y), -sin(theta_x)*cos(theta_y), cos(theta_x)*cos(theta_y)]` means that with both angles at zero the thrust points along +z, a positive `theta_x` tilts it toward -y, and a positive `theta_y` tilts it toward +x, in the frame `t_hat` is expressed in. The header's "not defined" list used to say the tilt direction was undefined. That was wrong and is fixed (`4a174fa`): the header now states the convention, and the list names the physical servo direction and mounting, and which servo is X or Y, as the open items.

**Gimbal conversion code (`gimbal.h`).** The docs' two formulas are now code, so the convention is executable and tested. Angles are in radians, and neither function applies the gimbal limits.

```c
void gimbal_thrust_direction(float theta_x_rad, float theta_y_rad, float t_hat_out[3]);
void gimbal_angles_from_thrust(const float t_des_hat[3], float *theta_x_rad, float *theta_y_rad);
```

- `gimbal_thrust_direction` returns `[sin(theta_y), -sin(theta_x)*cos(theta_y), cos(theta_x)*cos(theta_y)]`, a unit vector.
- `gimbal_angles_from_thrust` returns `theta_x = -atan(t_y / t_z)` and `theta_y = asin(t_x)` for a desired unit direction. It asserts on a NaN or infinite input, on a non-unit x component, and on a thrust that points backwards (`t_z <= 0`). An x component just past 1 through rounding is clamped.
- The two are inverses of each other for angles strictly between -pi/2 and pi/2.
- **Checked on a host:** the direction is always a unit vector over a grid of angles, the round trip recovers the angles, 18 degrees gives `t_y = -0.3090`, `t_z = 0.9511` (and `t_x = 0.3090` for `theta_y`), and every assert fires. CI compiles the code but does not run the tests.

**Still open**
- The world and body frame axes, handedness, and the direction of gravity.
- The thrust axis. The docs' thrust vector has a positive z component, but the ESKF branch's body frame is forward/right/down, where +z points down.
- The frame of omega and of the inertia. The handwritten notes assume the body frame, with `q_err` in the world frame, and cite the Rocket Proposal PDF as saying to rotate at the junctions. That PDF has not been read yet.
- The physical servo direction and mounting, which servo is X and which is Y, and which motor is the top one. Ask firmware first.

**Status.** Open, waiting on the controls team and firmware.

## #8 Renormalize after integrating gyro rates

A valid rotation quaternion has length 1. Float integration slowly changes the length, so the quaternion is divided by its norm after each update. The rotation is unchanged. In a standalone test the norm drifted to 1.0169 after 100,000 first-order steps without normalizing and stayed at 1.000000 with it.

```c
static inline float quaternion_norm(quaternion_t q)
{
    return sqrtf(q.q[0] * q.q[0] + q.q[1] * q.q[1] + q.q[2] * q.q[2] + q.q[3] * q.q[3]);
}

static inline quaternion_t quaternion_normalize(quaternion_t q)
{
    float norm = quaternion_norm(q);

    // Written so NaN also fails
    ULYSSES_ASSERT(norm > FLT_MIN, "Cannot normalize a zero or NaN quaternion");

    float inv_norm = 1.0f / norm;
    for (int i = 0; i < 4; i++) {
        q.q[i] *= inv_norm;
    }

    return q;
}
```

- It divides once by `1/norm`, not four times.
- `q` is passed and returned by value, so the caller's original is untouched.
- `norm > FLT_MIN` is false for zero, for NaN, and for values so small their squares underflow. A comparison with NaN is always false, so the check is written as "must be greater than".

**Call site pending.** The call goes in `eskf_predict` after the orientation is propagated, and also after the error state is injected following a measurement update. `eskf_predict` is still an empty stub on the ESKF branch and its owner is still working on it, so this is left as a flag.

**Status.** Open, waiting on the ESKF owner.

## #9 Quaternion helpers

All four helpers exist as `static inline` functions in `quaternion.h`:

- `quaternion_multiply(a, b)`: the Hamilton product. A rotation `dq` in the world frame is `dq * q`, and in the body frame it is `q * dq`.
- `quaternion_conjugate(q)`: `(w, -v)`. For a unit quaternion it is the inverse.
- `quaternion_normalize(q)`: see #8.
- `quaternion_to_rotation_vector(q, out)`: the docs' phi formula, `phi = 2 * atan(|v| / w) * v / |v|`. A rotation by angle theta about axis n gives `theta * n`, in radians, in the frame `q` is in. For `q_err` that is the world frame.

The rotation vector uses `atan2`, so `w = 0` is defined, and it first flips `q` to `w >= 0`, so the angle is at most pi and `q` and `-q` give the same result. The `sign(...)` factor in the docs' MATLAB is not needed, because the flip does that job. The input does not have to be unit length. The identity gives the zero vector. A zero, infinite or NaN quaternion, or a NULL output, hits the assert handler.

**Checked on a host.** 2,000 random unit quaternions compared against rotation matrices computed separately, Hamilton's rules, associativity, the identity, `conj(p*q) = conj(q)*conj(p)`, non-commutativity, axis-angle cases from 0 to 2 pi, and the end-to-end case: a 30 degree offset about z gives `phi` of 0.5236 rad about z, and swapping desired and measured flips the sign. CI only compiles the header, through `quaternion.c`.

**Status.** Done and closed. Nothing calls the helpers from the controller yet.

## #10 PID with anti-windup

Two functions, both with integrator clamping:

- `pid_step(state, kp, ki, kd, integral_limit, error, error_rate, dt_s)`: a single-axis PID, used for the gimbal.
- `pid3_step(state, kp, ki, kd, integral_limit, error, error_rate, dt_s, out)`: a 3-axis PID that applies the full 3x3 `Kp`, `Ki` and `Kd` matrices, so an error on one axis can drive another axis's output. Used for torque.

```c
float pid_step(pid_state_t *state, float kp, float ki, float kd, float integral_limit, float error,
               float error_rate, float dt_s)
{
    /* asserts: state non-NULL, dt_s > 0, integral_limit >= 0 */

    state->integral = clamp_symmetric(state->integral + error * dt_s, integral_limit);

    return kp * error + ki * state->integral + kd * error_rate;
}
```

- **Anti-windup:** each integrator is clamped to plus or minus `integral_limit`, in units of error times seconds. The integral term can contribute at most `Ki * integral_limit`. The limits are stored with the gains (`integral_max` for torque, `integral_max_x` and `integral_max_y` for the gimbal), so they can be tuned live.
- **Derivative:** the caller supplies `error_rate`, the time derivative of the error. When the reference is constant that is `-omega`, the negative of the measured rate. This avoids a startup spike and noise amplification, and keeps the PID independent of the frame omega is in.
- **Gains are passed on every call and never stored**, so changing them does not disturb the integrator.
- **Not checked:** a NaN error reaches the integrator and stays there until `pid_reset` or `pid3_reset`.

**Known gap.** The gimbal output is meant to be clamped to `gimbal_max_x_rad` and `gimbal_max_y_rad`, with torque unlimited until the CAD values exist. Only the integrator clamp is implemented, and the functions in `gimbal.h` do not apply the limits either. The output clamp belongs in the control law.

**Status.** Done and closed. Nothing calls the PIDs from the controller yet.

## #11 Thrust to motor command layer

**On hold.** Decided so far: the tables output throttle in percent, not RPM, and each contra-rotating motor gets its own 2D table, indexed by thrust magnitude and roll torque. A bilinear lookup and a DShot encoder were written and then reverted (`35a90f9`, reverted in `e7c217b`), and the code is still in the history.

**Blocked.** The motor outputs are `TIM1` CH1 (PE9, `ESC1_PWM`) and CH2 (PE11, `ESC2_PWM`) on the CM7, with a 240 MHz timer clock. `TIM1` is initialized but never started, and no DMA is configured. The CM7's only data RAM is 128K at `0x20000000`, which DMA cannot reach, so a DMA-driven output needs a linker-script change and a CubeMX DMA setup. Which ESC flies and its protocol and speed are open: the pin sheet points at the custom ESC V1.0, while the DShot and AM32 pages describe a commercial one. Random121's branch `dev/random121/add-bdshot-lib` adds a bidirectional DShot library that overlaps this work.

**Needed.** Firmware: the ESC, protocol and speed, the linker and DMA change, the fault output, the minimum spin throttle, and which servo is X or Y and which motor is on top. Controls: the table input units. Mechanical and simulation: the characterization data and grid.

## #12 `pos_ref` and `vel_ref`

They stay. They are the desired position and velocity, hard-coded for the hop test (straight up) and set from the console later. The measured values come from the estimator through `fc_state_t` (#15). Still open: how a position or velocity error becomes a thrust magnitude demand, which no doc defines. #11's tables need that thrust as an input.

**Status.** Open, waiting on the controls team for the thrust law.

## #13 `fc_config_t` from CAD

**On hold.** There are no CAD values in the repo or the team docs, and `fc_config_t` is not instantiated anywhere. The only number so far is the gimbal limit: about plus or minus 18 degrees, which is 0.3142 rad, a mechanical estimate that is not fully tested and might reach 20 degrees. The earlier simulation used 14 degrees. The limits are now named `gimbal_max_x_rad` and `gimbal_max_y_rad` (`4a174fa`), so the unit is in the name, and 18 degrees would be entered as 0.3142.

**Needed.** Mass, inertia tensor, centre of mass and gimbal pivot from CAD, with the reference point, the axes and the CAD revision. Also the body frame (#7) and the PID gains. The TVR is electric, so its mass is constant in flight and the config is set once per flight at `init`. The docs list `MMOI` as a scalar, but the struct holds a 3x3 tensor, and with a scalar `tau_gyro` would always be zero, so a tensor needs confirming. A zeroed config means zero mass and zero gains with no error. For now this is only flagged: `init` has a TODO listing what a validity check should cover (mass above zero, a symmetric positive-definite inertia, positive gimbal limits, finite gains), and the header lists it under known limitations.

## #14 Include guard

Checked, already correct. `flight_controller.h` opens with `#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER` and `#define`, and its last line is `#endif /* ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER */`. The only other conditional is the nested `#ifndef FC_DT_MAX_US ... #endif`. Convention: the guard is the header path in upper case with underscores, and the closing `#endif` stays the last line.

**Status.** Closed. No change was needed.

## #15 The state input, and the missing `eskf.h`

**Decision.** The state estimator is a black box to the controller. `flight_controller_step` takes its own struct, and the caller converts whatever the estimator outputs into it:

```c
typedef struct fc_state {
    quaternion_t orientation; ///< Measured orientation q_meas, same convention as attitude_ref
    float omega[3];           ///< Angular velocity in radians per second
    float position[3];        ///< Position in metres
    float velocity[3];        ///< Velocity in metres per second
} fc_state_t;

void flight_controller_step(flight_controller_t *fc, const fc_state_t *state, const fc_ref_t *ref,
                            fc_command_t *cmd_out, float dt_s);
```

The `eskf.h` include is gone, so the controller no longer depends on the filter's types. Omega is stubbed until the estimator provides it, because `eskf_state_t` on the ESKF branch has no angular velocity. `assert.h` already existed. `flight_controller.c` is in `lib/CMakeLists.txt`, and the ARM Debug and Release builds pass in CI. The whole run, including lint, is green on `5fd9cc2`. The frames are still undefined, so each field is in whichever frame the caller and the control law agree on.

**Status.** Done and closed. Pushed as `a9cd632`. Later: an adapter from the estimator's output to `fc_state_t`, once the ESKF branch lands.

## #16 Joseph-form covariance update, and EKF doc typos

**On hold.** `eskf_update_position` and `eskf_predict` are empty stubs on the ESKF branch, so there is no covariance update to convert yet. The "EKF Full Derivation" page already boxes the Joseph form (eq. 8a) and gives the Cholesky solve for the gain (Appendix B). CMSIS-DSP is not in the repo, and for the 3x3 position update a small hand-written Cholesky is enough.

**Docs typos.** Seven fixes are committed in a local clone of `UBC-Rocket/docs` (`039cd30`), not pushed and no PR opened: the `P_{k|k}` versus `P_{k|k-1}` subscripts in prediction step 2, a reversed sentence about the innovation, "equation F and G" (should be F and equation 7), the `delta_{k|k}` subscript in the Joseph derivation, `k_k` for `K_k`, "semi-definite" for "definite", and a missing transpose in Appendix A. The second typo named in the issue, `u_k` versus `u_{k-1}`, is unresolved: `h` is defined as `h(x_k)` with no input, but eqs. 4, D and E write `h(x_hat, u_k)`, and the author needs to say which is right. The ESKF overview also lists a barometer bias that `eskf_state_t` lacks, and writes `omega_norm` where it defines `omega_nom`.

## Known gaps, in one list

**Open**
1. `set_gains` does not validate the gains (finite, within limits).
2. The gimbal output clamp to `gimbal_max_x_rad` and `gimbal_max_y_rad` is not implemented.

**Flagged in the code, not fixed** (in the header's known-limitations block, and as TODOs in the `.c` file)
3. A zeroed or forgotten `fc_config_t` is accepted silently.
4. The control law in `flight_controller_step` is not implemented, so the command is always zero.
5. The controller tests run only on a host, not in CI.

**Fixed in `4a174fa`**
6. The gimbal limits are named in radians (`gimbal_max_x_rad`, `gimbal_max_y_rad`), and the theta command fields say radians in their comments.
7. The "not defined" list in `flight_controller.h` no longer says the gimbal tilt direction is undefined. The convention is stated in the header and implemented in `gimbal.h`.
