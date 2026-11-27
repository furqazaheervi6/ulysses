#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/xy_pid.h"

static float clamp_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void coupled_output(const xy_pid_coupled_gains_t *gains, const float error[2],
                           const float integral[2], const float error_rate[2], float out[2])
{
    for (int i = 0; i < 2; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 2; j++) {
            sum += gains->kp[i][j] * error[j] + gains->ki[i][j] * integral[j] +
                   gains->kd[i][j] * error_rate[j];
        }
        out[i] = sum;
    }
}

void xy_pid_reset(xy_pid_state_t *state)
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");

    state->integral[XY_PID_X] = 0.0f;
    state->integral[XY_PID_Y] = 0.0f;
}

unsigned xy_pid_coupled_step(xy_pid_state_t *state, const xy_pid_coupled_gains_t *gains,
                             const float error[2], const float error_rate[2], float dt_s,
                             float out[2])
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");
    ULYSSES_ASSERT(gains != NULL, "PID gains must be non NULL");
    ULYSSES_ASSERT(error != NULL && error_rate != NULL, "PID inputs must be non NULL");
    ULYSSES_ASSERT(out != NULL, "PID output must be non NULL");
    ULYSSES_ASSERT(dt_s > 0.0f, "Time step must be positive");
    for (int i = 0; i < 2; i++) {
        ULYSSES_ASSERT(gains->integral_limit[i] >= 0.0f, "Integral limit must be non-negative");
        ULYSSES_ASSERT(gains->output_limit[i] > 0.0f, "Output limit must be positive");
    }

    // Integrate, then undo it on any axis whose output saturates in the direction its error
    // pushes, so the integrator does not wind up against the limit.
    float integral[2];
    for (int i = 0; i < 2; i++) {
        integral[i] =
            clamp_symmetric(state->integral[i] + error[i] * dt_s, gains->integral_limit[i]);
    }

    float raw[2];
    coupled_output(gains, error, integral, error_rate, raw);

    unsigned saturated = 0u;
    for (int i = 0; i < 2; i++) {
        if (raw[i] > gains->output_limit[i] || raw[i] < -gains->output_limit[i]) {
            saturated |= 1u << i;
            if (gains->ki[i][i] * error[i] * raw[i] > 0.0f) {
                integral[i] = state->integral[i];
            }
        }
    }

    state->integral[XY_PID_X] = integral[XY_PID_X];
    state->integral[XY_PID_Y] = integral[XY_PID_Y];

    coupled_output(gains, error, integral, error_rate, out);
    for (int i = 0; i < 2; i++) {
        out[i] = clamp_symmetric(out[i], gains->output_limit[i]);
    }

    return saturated;
}

unsigned xy_pid_step(xy_pid_state_t *state, const xy_pid_gains_t *gains, const float error[2],
                     const float error_rate[2], float dt_s, float out[2])
{
    ULYSSES_ASSERT(gains != NULL, "PID gains must be non NULL");

    const xy_pid_coupled_gains_t diagonal = {
        .kp = {{gains->x.kp, 0.0f}, {0.0f, gains->y.kp}},
        .ki = {{gains->x.ki, 0.0f}, {0.0f, gains->y.ki}},
        .kd = {{gains->x.kd, 0.0f}, {0.0f, gains->y.kd}},
        .integral_limit = {gains->x.integral_limit, gains->y.integral_limit},
        .output_limit = {gains->x.output_limit, gains->y.output_limit},
    };

    return xy_pid_coupled_step(state, &diagonal, error, error_rate, dt_s, out);
}

unsigned xy_pid_run(xy_pid_state_t *state, const xy_pid_gains_t *gains, const xy_pid_io_t *io,
                    const float setpoint[2], float dt_s)
{
    ULYSSES_ASSERT(io != NULL, "PID io must be non NULL");
    ULYSSES_ASSERT(io->get_measurement != NULL && io->get_measurement_rate != NULL &&
                       io->set_command != NULL,
                   "PID io functions must be non NULL");
    ULYSSES_ASSERT(setpoint != NULL, "PID setpoint must be non NULL");

    float measurement[2];
    float rate[2];
    io->get_measurement(io->ctx, measurement);
    io->get_measurement_rate(io->ctx, rate);

    const float error[2] = {setpoint[XY_PID_X] - measurement[XY_PID_X],
                            setpoint[XY_PID_Y] - measurement[XY_PID_Y]};
    const float error_rate[2] = {-rate[XY_PID_X], -rate[XY_PID_Y]};

    float command[2];
    unsigned saturated = xy_pid_step(state, gains, error, error_rate, dt_s, command);
    io->set_command(io->ctx, command);

    return saturated;
}
