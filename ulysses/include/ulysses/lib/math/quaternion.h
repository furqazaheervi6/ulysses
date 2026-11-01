#ifndef ULYSSES_LIB_MATH_QUATERNION
#define ULYSSES_LIB_MATH_QUATERNION

#include "ulysses/lib/assert.h"

#include <float.h>
#include <math.h>

/// Hamilton quaternion, scalar-first (w, v), mapping body to world. See flight_controller.h.
typedef union quaternion {
    float q[4];

    struct {
        float w;
        float v[3];
    };
} quaternion_t;

/**
 * @brief Returns the Euclidean norm of a quaternion
 *
 * @param q Quaternion to measure
 *
 * @return sqrt(w^2 + x^2 + y^2 + z^2)
 */
static inline float quaternion_norm(quaternion_t q)
{
    return sqrtf(q.q[0] * q.q[0] + q.q[1] * q.q[1] + q.q[2] * q.q[2] + q.q[3] * q.q[3]);
}

/**
 * @brief Scales a quaternion to unit length
 *
 * Integrating gyro rates with floats slowly changes the norm of an orientation quaternion, so call
 * this after each propagation step. The rotation represented is unchanged.
 *
 * @param q Quaternion with a finite, non-zero norm
 *
 * @return q / |q|
 */
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

/**
 * @brief Returns the quaternion representing the same rotation with a non-negative scalar part
 *
 * q and -q are the same rotation, but only the one with w >= 0 is the shortest way round. Apply
 * this to an error quaternion before extracting the rotation-vector error, so the controller does
 * not rotate the long way.
 *
 * @param q Quaternion to normalize the sign of
 *
 * @return q if q.w >= 0 (or NaN), otherwise -q
 */
static inline quaternion_t quaternion_shortest_path(quaternion_t q)
{
    if (q.w < 0.0f) {
        for (int i = 0; i < 4; i++) {
            q.q[i] = -q.q[i];
        }
    }

    return q;
}

#endif /* ULYSSES_LIB_MATH_QUATERNION */
