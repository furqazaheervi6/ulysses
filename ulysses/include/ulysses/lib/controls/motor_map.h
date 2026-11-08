#ifndef ULYSSES_LIB_CONTROLS_MOTOR_MAP
#define ULYSSES_LIB_CONTROLS_MOTOR_MAP

#include <stdbool.h>
#include <stdint.h>

/**
 * @file
 * Maps a desired thrust and roll torque to a throttle for each of the two contra-rotating motors
 *
 * Each motor has its own 2D table: throttle = table(thrust, roll torque). The tables come from the
 * propeller characterization experiment, which has not been run yet, so this file holds no data.
 * The caller provides the tables, in the same way it provides fc_config_t.
 *
 * Assumptions, not yet confirmed by the controls team:
 *  - the two table inputs are the thrust magnitude in newtons and the roll torque in newton metres;
 *  - the output is a throttle in percent (0 to 100), as in the characterization experiment, and
 *    not an RPM.
 */

/// Largest throttle a table may hold, in percent
#define MOTOR_THROTTLE_MAX_PCT (100.0f)

/**
 * @brief A 2D table of throttle values, indexed by thrust and roll torque
 *
 * The arrays are not owned, so they can be const data in flash. throttle_pct is row-major:
 * throttle_pct[i * torque_count + j] is the throttle at thrust_n[i] and roll_torque_nm[j].
 */
typedef struct motor_table {
    uint8_t thrust_count;
    uint8_t torque_count;
    const float *thrust_n;       ///< thrust_count values, strictly increasing
    const float *roll_torque_nm; ///< torque_count values, strictly increasing
    const float *throttle_pct;   ///< thrust_count * torque_count values, 0 to 100
} motor_table_t;

/// Result of a table lookup
typedef enum motor_table_status {
    MOTOR_TABLE_OK,      ///< Both inputs were inside the table
    MOTOR_TABLE_CLAMPED, ///< At least one input was outside the table and was clamped to its edge
    MOTOR_TABLE_INVALID_INPUT, ///< An input was NaN or infinite; the throttle output is 0
} motor_table_status_t;

/// The tables for both motors
typedef struct motor_map {
    motor_table_t top;
    motor_table_t bottom;
} motor_map_t;

/**
 * @brief Checks that a table can be used by motor_table_lookup()
 *
 * A valid table has non-NULL arrays, at least 2 points on each axis, finite and strictly increasing
 * axes, and finite throttle values from 0 to MOTOR_THROTTLE_MAX_PCT. Call it once when the table is
 * set up, not on every lookup.
 *
 * @param table Table to check; may be NULL
 * @return true if the table is valid
 */
bool motor_table_valid(const motor_table_t *table);

/**
 * @brief Looks up the throttle for a thrust and a roll torque
 *
 * Interpolates bilinearly between the four surrounding table points. Inputs outside the table are
 * clamped to its edge instead of extrapolated, so the result never leaves the range of the table.
 *
 * @param table Non-NULL valid table, see motor_table_valid()
 * @param thrust_n Desired thrust in newtons
 * @param roll_torque_nm Desired roll torque in newton metres
 * @param[out] throttle_pct Non-NULL; the throttle in percent. 0 if the status is
 *                          MOTOR_TABLE_INVALID_INPUT.
 * @return Whether the inputs were inside the table
 */
motor_table_status_t motor_table_lookup(const motor_table_t *table, float thrust_n,
                                        float roll_torque_nm, float *throttle_pct);

/**
 * @brief Looks up the throttle of both motors
 *
 * @param map Non-NULL map with two valid tables
 * @param thrust_n Desired thrust in newtons
 * @param roll_torque_nm Desired roll torque in newton metres
 * @param[out] top_pct Non-NULL; throttle of the top motor in percent
 * @param[out] bottom_pct Non-NULL; throttle of the bottom motor in percent
 * @return MOTOR_TABLE_INVALID_INPUT if the inputs were not finite, else MOTOR_TABLE_CLAMPED if
 *         either table clamped an input, else MOTOR_TABLE_OK
 */
motor_table_status_t motor_map_lookup(const motor_map_t *map, float thrust_n, float roll_torque_nm,
                                      float *top_pct, float *bottom_pct);

#endif /* ULYSSES_LIB_CONTROLS_MOTOR_MAP */
