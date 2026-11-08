#ifndef ULYSSES_LIB_CONTROLS_PID
#define ULYSSES_LIB_CONTROLS_PID

/**
 * @file
 * PID controllers with integrator clamping (anti-windup)
 *
 * u = Kp * e + Ki * integral(e) + Kd * de/dt
 *
 * The caller supplies de/dt (error_rate) instead of the controller differentiating the error. This
 * avoids a startup spike and noise amplification, and keeps this module independent of which frame
 * the rate is measured in. When the reference is constant, de/dt is the negative of the measured
 * rate (e.g. -omega).
 *
 * Gains are passed on every call and are not stored, so changing them never disturbs the
 * integrator. Only the integrator is state.
 *
 * Anti-windup: after each update every integrator is clamped to [-integral_limit, integral_limit].
 * The limit is on the integrator itself, in units of error * seconds. The contribution of the
 * integral term to the output is therefore at most Ki * integral_limit.
 *
 * Neither function checks that error or error_rate are finite. A NaN error reaches the integrator
 * and stays there until pid_reset() or pid3_reset().
 */

/** @brief Integrator state of a single-axis PID */
typedef struct pid_state {
    float integral; ///< Accumulated error * seconds, clamped by pid_step()
} pid_state_t;

/** @brief Integrator state of a 3-axis PID with matrix gains */
typedef struct pid3_state {
    float integral[3]; ///< Accumulated error * seconds per axis, clamped by pid3_step()
} pid3_state_t;

/**
 * @brief Clears the integrator
 *
 * @param[out] state Non-NULL state to clear
 */
void pid_reset(pid_state_t *state);

/**
 * @brief Clears all three integrators
 *
 * @param[out] state Non-NULL state to clear
 */
void pid3_reset(pid3_state_t *state);

/**
 * @brief Runs one single-axis PID update
 *
 * @param state Non-NULL integrator state, updated
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param integral_limit Largest magnitude the integrator may reach; must be non-negative
 * @param error Current error
 * @param error_rate Time derivative of the error, supplied by the caller
 * @param dt_s Time since the previous update in seconds; must be positive, otherwise the assert
 *             handler is called
 * @return The controller output
 */
float pid_step(pid_state_t *state, float kp, float ki, float kd, float integral_limit, float error,
               float error_rate, float dt_s);

/**
 * @brief Runs one 3-axis PID update with matrix gains
 *
 * Each term is a matrix times a vector, so an error on one axis can drive the output on another.
 * The integrator of each axis is clamped separately.
 *
 * @param state Non-NULL integrator state, updated
 * @param kp Proportional gain matrix
 * @param ki Integral gain matrix
 * @param kd Derivative gain matrix
 * @param integral_limit Largest magnitude each integrator may reach; must be non-negative
 * @param error Current error vector
 * @param error_rate Time derivative of the error vector, supplied by the caller
 * @param dt_s Time since the previous update in seconds; must be positive, otherwise the assert
 *             handler is called
 * @param[out] out Controller output vector
 */
void pid3_step(pid3_state_t *state, const float kp[3][3], const float ki[3][3],
               const float kd[3][3], float integral_limit, const float error[3],
               const float error_rate[3], float dt_s, float out[3]);

#endif /* ULYSSES_LIB_CONTROLS_PID */
