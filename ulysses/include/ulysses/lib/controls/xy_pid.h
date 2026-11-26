#ifndef ULYSSES_LIB_CONTROLS_XY_PID
#define ULYSSES_LIB_CONTROLS_XY_PID

/**
 * @file
 * Two-axis PID: two parallel loops (x and y), or one coupled (x, y) loop
 *
 * Parallel: the x loop and the y loop are independent single-axis PIDs, each with its own gains
 * and limits. An error on x only drives the x output, and an error on y only drives the y output.
 *
 *   u_x = kp_x * e_x + ki_x * I_x + kd_x * de_x/dt
 *   u_y = kp_y * e_y + ki_y * I_y + kd_y * de_y/dt
 *
 * Coupled: one loop on the (x, y) error vector with 2x2 matrix gains, so an error on one axis can
 * also drive the other output (u = Kp e + Ki I + Kd de/dt). The parallel loops are exactly the
 * coupled loop with diagonal gains, and xy_pid_step() is implemented that way.
 *
 * As in pid.h, the caller supplies de/dt (error_rate) instead of the controller differentiating
 * the error, the gains are passed on every call and not stored, and only the integrators are
 * state. Set the integral gains to zero for a PD controller.
 *
 * Limits, per axis:
 *  - integral_limit clamps the integrator itself (error * seconds), as in pid.h.
 *  - output_limit clamps the output. Pass INFINITY for no limit. While an output is clamped and
 *    its error pushes further into the limit, that axis's integrator is held instead of winding
 *    up (conditional integration). The coupled loop decides this with its diagonal integral gain.
 *
 * What x and y mean, and their frames and units, is the caller's choice. xy_pid_run() reads the
 * measurements and writes the commands through caller-supplied functions, so this module never
 * depends on the estimator or the actuator drivers.
 */

#define XY_PID_X (0) ///< Index of the x axis in every 2-vector
#define XY_PID_Y (1) ///< Index of the y axis in every 2-vector

#define XY_PID_SATURATED_X (1u << 0) ///< The x output was clamped to its output_limit
#define XY_PID_SATURATED_Y (1u << 1) ///< The y output was clamped to its output_limit

/** @brief Gains and limits of one axis of the parallel loops */
typedef struct xy_pid_axis_gains {
    float kp; ///< Proportional gain
    float ki; ///< Integral gain
    float kd; ///< Derivative gain

    float integral_limit; ///< Largest magnitude of the integrator (error * seconds); non-negative
    float output_limit;   ///< Largest magnitude of the output; positive, INFINITY for none
} xy_pid_axis_gains_t;

/** @brief Gains of the two parallel loops */
typedef struct xy_pid_gains {
    xy_pid_axis_gains_t x;
    xy_pid_axis_gains_t y;
} xy_pid_gains_t;

/** @brief Gains of the coupled (x, y) loop. Row i gives output i, column j reads error j. */
typedef struct xy_pid_coupled_gains {
    float kp[2][2];
    float ki[2][2];
    float kd[2][2];

    float integral_limit[2]; ///< Per axis, as in xy_pid_axis_gains_t
    float output_limit[2];   ///< Per axis, as in xy_pid_axis_gains_t
} xy_pid_coupled_gains_t;

/** @brief Integrator state, shared by both modes */
typedef struct xy_pid_state {
    float integral[2]; ///< Accumulated error * seconds per axis
} xy_pid_state_t;

/**
 * @brief Functions xy_pid_run() calls to read the measurements and write the commands
 *
 * Supplied by whoever wires the loops in (the estimator and the actuator driver). The loops treat
 * them as a black box. ctx is passed back unchanged to each function.
 */
typedef struct xy_pid_io {
    void *ctx;

    /// Writes the measured (x, y) values, e.g. the vehicle tilt in radians
    void (*get_measurement)(void *ctx, float measurement[2]);

    /// Writes the measured rate of change of (x, y), e.g. the body rates in radians per second
    void (*get_measurement_rate)(void *ctx, float rate[2]);

    /// Applies the (x, y) command, e.g. the gimbal angles in radians
    void (*set_command)(void *ctx, const float command[2]);
} xy_pid_io_t;

/**
 * @brief Clears both integrators
 *
 * Call it when switching between the parallel and coupled modes, so integral built up under one
 * gain set is not applied with another.
 *
 * @param[out] state Non-NULL state to clear
 */
void xy_pid_reset(xy_pid_state_t *state);

/**
 * @brief Runs one update of the two parallel loops
 *
 * @param state Non-NULL integrator state, updated
 * @param gains Non-NULL gains and limits for x and y
 * @param error Non-NULL error (e_x, e_y)
 * @param error_rate Non-NULL time derivative of the error, supplied by the caller
 * @param dt_s Time since the previous update in seconds; must be positive, otherwise the assert
 *             handler is called
 * @param[out] out Non-NULL output (u_x, u_y), clamped to the output limits
 * @return XY_PID_SATURATED_* bits for the axes whose output was clamped, 0 if none
 */
unsigned xy_pid_step(xy_pid_state_t *state, const xy_pid_gains_t *gains, const float error[2],
                     const float error_rate[2], float dt_s, float out[2]);

/**
 * @brief Runs one update of the coupled (x, y) loop
 *
 * Parameters and return value as xy_pid_step(), with 2x2 gain matrices.
 */
unsigned xy_pid_coupled_step(xy_pid_state_t *state, const xy_pid_coupled_gains_t *gains,
                             const float error[2], const float error_rate[2], float dt_s,
                             float out[2]);

/**
 * @brief Runs one tick of the parallel loops against a black-box measurement and actuator
 *
 * Reads the measurement and its rate through io, computes error = setpoint - measurement and
 * error_rate = -rate (the setpoint is treated as constant), runs xy_pid_step() and writes the
 * command through io.
 *
 * @param state Non-NULL integrator state, updated
 * @param gains Non-NULL gains and limits for x and y
 * @param io Non-NULL functions to call; all three must be non-NULL
 * @param setpoint Non-NULL desired (x, y), e.g. {0, 0} for upright
 * @param dt_s Time since the previous tick in seconds; must be positive
 * @return XY_PID_SATURATED_* bits, as xy_pid_step()
 */
unsigned xy_pid_run(xy_pid_state_t *state, const xy_pid_gains_t *gains, const xy_pid_io_t *io,
                    const float setpoint[2], float dt_s);

#endif /* ULYSSES_LIB_CONTROLS_XY_PID */
