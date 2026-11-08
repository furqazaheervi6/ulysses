#ifndef ULYSSES_LIB_MATH_QUATERNION
#define ULYSSES_LIB_MATH_QUATERNION

#include "ulysses/lib/assert.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

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

/**
 * @brief Converts a quaternion to the rotation vector of the shortest rotation it represents
 *
 * A rotation by angle theta about the unit axis n has the rotation vector theta * n. This is the
 * phi of the control architecture docs, phi = 2 * atan(|v| / w) * v / |v|, applied to q_err. It is
 * computed with atan2 so that w = 0 (a half turn) is defined, and q is first flipped to w >= 0
 * with quaternion_shortest_path(), so the angle is at most pi and the controller never turns the
 * long way. The result is in the frame q is expressed in and in radians.
 *
 * q does not need to be unit length, since only its direction matters. A quaternion with no
 * rotation (v = 0, w > 0) gives the zero vector.
 *
 * @param q Quaternion with a finite, non-zero norm
 * @param[out] out Non-NULL rotation vector
 */
static inline void quaternion_to_rotation_vector(quaternion_t q, float out[3])
{
    ULYSSES_ASSERT(out != NULL, "Rotation vector output must be non NULL");

    q = quaternion_shortest_path(q);

    float v_norm = sqrtf(q.v[0] * q.v[0] + q.v[1] * q.v[1] + q.v[2] * q.v[2]);

    // Written so NaN also fails
    ULYSSES_ASSERT(isfinite(q.w) && isfinite(v_norm) && (q.w > 0.0f || v_norm > 0.0f),
                   "Cannot convert a zero, infinite or NaN quaternion");

    if (v_norm <= FLT_MIN) {
        for (int i = 0; i < 3; i++) {
            out[i] = 0.0f;
        }
        return;
    }

    float scale = 2.0f * atan2f(v_norm, q.w) / v_norm;
    for (int i = 0; i < 3; i++) {
        out[i] = scale * q.v[i];
    }
}

#endif /* ULYSSES_LIB_MATH_QUATERNION */
