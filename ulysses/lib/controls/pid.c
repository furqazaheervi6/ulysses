#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/pid.h"

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

void pid_reset(pid_state_t *state)
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");

    state->integral = 0.0f;
}

void pid3_reset(pid3_state_t *state)
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");

    for (int i = 0; i < 3; i++) {
        state->integral[i] = 0.0f;
    }
}

float pid_step(pid_state_t *state, float kp, float ki, float kd, float integral_limit, float error,
               float error_rate, float dt_s)
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");
    // Written so NaN also fails
    ULYSSES_ASSERT(dt_s > 0.0f, "Time step must be positive");
    ULYSSES_ASSERT(integral_limit >= 0.0f, "Integral limit must be non-negative");

    state->integral = clamp_symmetric(state->integral + error * dt_s, integral_limit);

    return kp * error + ki * state->integral + kd * error_rate;
}

void pid3_step(pid3_state_t *state, const float kp[3][3], const float ki[3][3],
               const float kd[3][3], float integral_limit, const float error[3],
               const float error_rate[3], float dt_s, float out[3])
{
    ULYSSES_ASSERT(state != NULL, "PID state must be non NULL");
    ULYSSES_ASSERT(kp != NULL && ki != NULL && kd != NULL, "PID gains must be non NULL");
    ULYSSES_ASSERT(error != NULL && error_rate != NULL, "PID inputs must be non NULL");
    ULYSSES_ASSERT(out != NULL, "PID output must be non NULL");
    // Written so NaN also fails
    ULYSSES_ASSERT(dt_s > 0.0f, "Time step must be positive");
    ULYSSES_ASSERT(integral_limit >= 0.0f, "Integral limit must be non-negative");

    for (int i = 0; i < 3; i++) {
        state->integral[i] = clamp_symmetric(state->integral[i] + error[i] * dt_s, integral_limit);
    }

    for (int i = 0; i < 3; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 3; j++) {
            sum += kp[i][j] * error[j] + ki[i][j] * state->integral[j] + kd[i][j] * error_rate[j];
        }
        out[i] = sum;
    }
}
