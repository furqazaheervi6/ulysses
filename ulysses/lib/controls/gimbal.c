#include <math.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/gimbal.h"

/// How far past 1 the x component may be through rounding before it counts as not a unit vector
#define GIMBAL_UNIT_TOLERANCE (1e-4f)

void gimbal_thrust_direction(float theta_x_rad, float theta_y_rad, float t_hat_out[3])
{
    ULYSSES_ASSERT(t_hat_out != NULL, "Thrust direction output must be non NULL");
    ULYSSES_ASSERT(isfinite(theta_x_rad) && isfinite(theta_y_rad), "Gimbal angles must be finite");

    float sin_x = sinf(theta_x_rad);
    float cos_x = cosf(theta_x_rad);
    float sin_y = sinf(theta_y_rad);
    float cos_y = cosf(theta_y_rad);

    t_hat_out[0] = sin_y;
    t_hat_out[1] = -sin_x * cos_y;
    t_hat_out[2] = cos_x * cos_y;
}

void gimbal_angles_from_thrust(const float t_des_hat[3], float *theta_x_rad, float *theta_y_rad)
{
    ULYSSES_ASSERT(t_des_hat != NULL, "Desired thrust direction must be non NULL");
    ULYSSES_ASSERT(theta_x_rad != NULL && theta_y_rad != NULL,
                   "Gimbal angle outputs must be non NULL");
    ULYSSES_ASSERT(isfinite(t_des_hat[0]) && isfinite(t_des_hat[1]) && isfinite(t_des_hat[2]),
                   "Desired thrust direction must be finite");
    // Written so NaN also fails
    ULYSSES_ASSERT(t_des_hat[2] > 0.0f, "Desired thrust direction must point toward +z");
    ULYSSES_ASSERT(fabsf(t_des_hat[0]) <= 1.0f + GIMBAL_UNIT_TOLERANCE,
                   "Desired thrust direction must be a unit vector");

    float x = fminf(1.0f, fmaxf(-1.0f, t_des_hat[0]));

    *theta_x_rad = -atanf(t_des_hat[1] / t_des_hat[2]);
    *theta_y_rad = asinf(x);
}
