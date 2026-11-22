#ifndef ULYSSES_LIB_CONTROLS_GIMBAL
#define ULYSSES_LIB_CONTROLS_GIMBAL

/**
 * @file
 * Conversions between the gimbal angles and the direction of the thrust
 *
 * Source: the control architecture page of the UBC Rocket docs. With the gimbal angles theta_x and
 * theta_y, the unit thrust direction is
 *
 *   t_hat = [sin(theta_y), -sin(theta_x) * cos(theta_y), cos(theta_x) * cos(theta_y)]
 *
 * and the angles that point the thrust along a desired unit direction t_des_hat are
 *
 *   theta_x = -atan(t_des_hat.y / t_des_hat.z)
 *   theta_y = asin(t_des_hat.x)
 *
 * Sign convention, which follows from the formula above: with both angles at zero the thrust points
 * along +z. A positive theta_x tilts the thrust toward -y, and a positive theta_y tilts it toward
 * +x. The vectors are in the frame t_hat is expressed in. The docs do not say which frame that is.
 *
 * All angles are in radians. The two conversions are inverses of each other for angles strictly
 * between -pi/2 and pi/2. These functions do not apply the gimbal limits.
 */

/**
 * @brief Returns the unit thrust direction for a pair of gimbal angles
 *
 * @param theta_x_rad Gimbal angle about x in radians, finite
 * @param theta_y_rad Gimbal angle about y in radians, finite
 * @param[out] t_hat_out Non-NULL unit thrust direction
 */
void gimbal_thrust_direction(float theta_x_rad, float theta_y_rad, float t_hat_out[3]);

/**
 * @brief Returns the gimbal angles that point the thrust along a desired direction
 *
 * The direction must be a unit vector. The z component must be positive, because the thrust cannot
 * point backwards through the gimbal. Otherwise the assert handler is called. A x component that
 * is outside [-1, 1] only through rounding is clamped.
 *
 * @param t_des_hat Non-NULL finite unit vector, the desired thrust direction
 * @param[out] theta_x_rad Non-NULL gimbal angle about x in radians
 * @param[out] theta_y_rad Non-NULL gimbal angle about y in radians
 */
void gimbal_angles_from_thrust(const float t_des_hat[3], float *theta_x_rad, float *theta_y_rad);

#endif /* ULYSSES_LIB_CONTROLS_GIMBAL */
