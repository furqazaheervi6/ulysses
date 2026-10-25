#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER
#define ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER

#include <stdatomic.h>

#include "ulysses/lib/controls/flight_config.h"
#include "ulysses/lib/eskf/eskf.h"

#include "ulysses/lib/math/quaternion.h"

typedef struct fc_ref {
    quaternion_t attitude_ref;
    float pos_ref[3];
    float vel_ref[3];
} fc_ref_t;

typedef struct fc_command {
    float theta_x_cmd;
    float theta_y_cmd;
    float thrust_top;
    float thrust_bottom;
} fc_command_t;

/** @brief Torque and gimbal gains, updated together */
typedef struct fc_gains {
    fc_torque_controller_config_t torque;
    fc_gimbal_controller_config_t gimbal;
} fc_gains_t;

typedef struct flight_controller {
    fc_config_t config;       ///< Copy of the configuration given to flight_controller_init()
    float torque_integral[3]; ///< Torque controller integrator state
    float torque_prev_err[3]; ///< Previous torque controller error, for the derivative term
    float gimbal_integral[2]; ///< Gimbal controller integrator state (x, y)
    float gimbal_prev_err[2]; ///< Previous gimbal controller error (x, y)

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
 *
 * @param[out] fc Non-NULL controller to initialize
 * @param config Non-NULL configuration to copy
 */
void flight_controller_init(flight_controller_t *fc, const fc_config_t *config);

/**
 * @brief Queues new controller gains
 *
 * Copies the gains into a pending buffer. flight_controller_step() applies them at the start of its
 * next call, so this never blocks the control loop. Integrator and previous-error state are kept,
 * so the output does not jump when gains change.
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

void flight_controller_step(flight_controller_t *fc, const eskf_nominal_state_t *state,
                            const fc_ref_t *ref, fc_command_t *cmd_out, float dt_s);

#endif /* ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER */
