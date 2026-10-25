#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER
#define ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER

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

typedef struct flight_controller {
    fc_config_t config;       ///< Copy of the configuration given to flight_controller_init()
    float torque_integral[3]; ///< Torque controller integrator state
    float torque_prev_err[3]; ///< Previous torque controller error, for the derivative term
    float gimbal_integral[2]; ///< Gimbal controller integrator state (x, y)
    float gimbal_prev_err[2]; ///< Previous gimbal controller error (x, y)
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
 * @brief Replaces the controller gains
 *
 * Copies the gains into the controller. Integrator and previous-error state are kept, so the
 * output does not jump when gains change.
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
