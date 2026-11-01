#ifndef ULYSSES_LIB_MATH_QUATERNION
#define ULYSSES_LIB_MATH_QUATERNION

#include "ulysses/lib/assert.h"

#include <math.h>

typedef union quaternion {
    float q[4];

    struct {
        float w;
        float v[3];
    };
} quaternion_t;

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
