#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER
#define ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER

#include <stdatomic.h>

#include "ulysses/lib/controls/flight_config.h"
#include "ulysses/lib/controls/pid.h"
#include "ulysses/lib/eskf/eskf.h"

#include "ulysses/lib/math/quaternion.h"

/**
 * @file
 * Frames and sign conventions used by the flight controller
 *
 * Source: the UBC Rocket docs (controls: Quaternions, Architecture overview, ESKF design
 * overview). Only what those pages state is written as fact here. Whatever they leave open is
 * listed at the end and must not be assumed.
 *
 * Quaternions are Hamilton, scalar-first (w, v). The orientation q maps body coordinates to world
 * coordinates. A rotation applied in the world frame left-multiplies (q' = dq * q), and a rotation
 * applied in the body frame right-multiplies (q' = q * dq). Normalize the quaternion after every
 * nominal state update.
 *
 * Attitude error: q_err = q_des * conj(q_meas). By the rule above, this is a rotation in the world
 * frame. The torque command is tau_cmd = tau_gyro + phi, where phi is the rotation-vector form of
 * q_err and tau_gyro = (MMOI * omega) x omega.
 *
 * Gimbal angles, for the desired thrust direction t_des: theta_x = -atan(t_des.y / t_des.z) and
 * theta_y = asin(t_des.x).
 *
 * Not defined by the docs, so not assumed here:
 *  - the world frame axes (NED, ENU, ...) and the direction of gravity;
 *  - the body frame axes and handedness, and which axis is the thrust axis;
 *  - the frame of omega, the sensor readings and t_des;
 *  - units;
 *  - which way a positive theta_x or theta_y tilts the thrust;
 *  - where tau_gyro, phi and tau_cmd are rotated into a common frame. q_err is a world-frame
 *    rotation, while the docs do not say which frame omega and MMOI are in.
 * Settle these with the controls team before writing the control law.
 */

typedef struct fc_ref {
    quaternion_t attitude_ref; ///< Desired orientation, same convention as q_meas
    float pos_ref[3];
    float vel_ref[3];
} fc_ref_t;

typedef struct fc_command {
    float theta_x_cmd;
    float theta_y_cmd;
    float thrust_top;
    float thrust_bottom;
} fc_command_t;

#define FC_FAULT_BAD_DT (1u << 0)           ///< step() was given a non-positive or NaN dt_s
#define FC_FAULT_MISSED_TICK (1u << 1)      ///< step() was given a dt_s above FC_DT_MAX_US
#define FC_FAULT_BAD_GAINS_LAYOUT (1u << 2) ///< init() found the gains buffer constants invalid

/// Longest dt_s step() accepts as normal (3 s).
#ifndef FC_DT_MAX_US
#define FC_DT_MAX_US (3000000u)
#endif
#define FC_DT_MAX_S ((float)FC_DT_MAX_US * 1e-6f)

_Static_assert(FC_DT_MAX_US > 0u, "FC_DT_MAX_US must be positive");
_Static_assert(FC_DT_MAX_US <= 10000000u, "FC_DT_MAX_US above 10 s is not a sane dt limit");

/** @brief Torque and gimbal gains, updated together */
typedef struct fc_gains {
    fc_torque_controller_config_t torque;
    fc_gimbal_controller_config_t gimbal;
} fc_gains_t;

/**
 * @brief Flight controller instance
 *
 * flight_controller_init() and flight_controller_step() must never run concurrently with each other
 * or with themselves, so call both from a single control task.
 */
typedef struct flight_controller {
    fc_config_t config;        ///< Copy of the configuration given to flight_controller_init()
    pid3_state_t torque_pid;   ///< Torque controller integrators
    pid_state_t gimbal_pid[2]; ///< Gimbal controller integrators (x, y)

    ///< Sticky FC_FAULT_* bits, set before the assert fires; cleared by init. Atomic, so other
    ///< tasks (e.g. telemetry) may read it while the control task runs.
    atomic_uint faults;

    // Triple buffer handing gains from the writer (set_gains) to the reader (step) without locks
    fc_gains_t gains_slots[3];
    atomic_uint gains_latest; ///< Slot index handed over, plus FC_GAINS_DIRTY if unread
    unsigned gains_write_idx; ///< Slot owned by the writer
    unsigned gains_read_idx;  ///< Slot owned by the reader
} flight_controller_t;

/**
 * @brief Initializes a flight controller
 *
 * Copies config, so config does not need to outlive this call, and clears all controller state.
 * Must not run concurrently with flight_controller_step().
 *
 * @param[out] fc Non-NULL controller to initialize
 * @param config Non-NULL configuration to copy
 */
void flight_controller_init(flight_controller_t *fc, const fc_config_t *config);

/**
 * @brief Queues new controller gains
 *
 * Copies the gains into a pending buffer. flight_controller_step() applies them at the start of its
 * next call, so this never blocks the control loop. The integrators are kept, so the output does
 * not jump when gains change.
 *
 * May be called from a different task than flight_controller_step(), but only one task may call
 * this function. If called again before the next step, the later gains replace the earlier ones.
 *
 * @param fc Non-NULL controller initialized with flight_controller_init()
 * @param torque Non-NULL torque controller gains to copy
 * @param gimbal Non-NULL gimbal controller gains to copy
 */
void flight_controller_set_gains(flight_controller_t *fc,
                                 const fc_torque_controller_config_t *torque,
                                 const fc_gimbal_controller_config_t *gimbal);

/**
 * @brief Runs one controller update
 *
 * Must not run concurrently with itself or with flight_controller_init().
 *
 * @param dt_s Time since the previous step in seconds; must be positive. Otherwise FC_FAULT_BAD_DT
 *             is set in fc->faults and the assert handler is called. If dt_s exceeds FC_DT_MAX_S
 *             (a missed tick), FC_FAULT_MISSED_TICK is set and dt_s is clamped to FC_DT_MAX_S for
 *             this step.
 */
void flight_controller_step(flight_controller_t *fc, const eskf_nominal_state_t *state,
                            const fc_ref_t *ref, fc_command_t *cmd_out, float dt_s);

#endif /* ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER */
