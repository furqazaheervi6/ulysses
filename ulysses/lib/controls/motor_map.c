#include <math.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/motor_map.h"

static bool axis_valid(const float *axis, uint8_t count)
{
    if (axis == NULL || count < 2) {
        return false;
    }

    for (uint8_t i = 0; i < count; i++) {
        if (!isfinite(axis[i])) {
            return false;
        }
        if (i > 0 && !(axis[i] > axis[i - 1])) {
            return false;
        }
    }

    return true;
}

/// Clamps x into the axis range. Returns true if it had to move.
static bool clamp_to_axis(const float *axis, uint8_t count, float *x)
{
    if (*x < axis[0]) {
        *x = axis[0];
        return true;
    }
    if (*x > axis[count - 1]) {
        *x = axis[count - 1];
        return true;
    }
    return false;
}

/// Finds the interval [axis[i], axis[i + 1]] holding x, and how far x is along it (0 to 1).
/// x must be inside the axis range.
static uint8_t find_interval(const float *axis, uint8_t count, float x, float *fraction)
{
    uint8_t i = 0;
    while (i < count - 2 && x > axis[i + 1]) {
        i++;
    }

    *fraction = (x - axis[i]) / (axis[i + 1] - axis[i]);
    return i;
}

bool motor_table_valid(const motor_table_t *table)
{
    if (table == NULL || table->throttle_pct == NULL) {
        return false;
    }
    if (!axis_valid(table->thrust_n, table->thrust_count) ||
        !axis_valid(table->roll_torque_nm, table->torque_count)) {
        return false;
    }

    uint16_t value_count = (uint16_t)table->thrust_count * table->torque_count;
    for (uint16_t i = 0; i < value_count; i++) {
        float value = table->throttle_pct[i];
        // Written so NaN also fails
        if (!(value >= 0.0f && value <= MOTOR_THROTTLE_MAX_PCT)) {
            return false;
        }
    }

    return true;
}

motor_table_status_t motor_table_lookup(const motor_table_t *table, float thrust_n,
                                        float roll_torque_nm, float *throttle_pct)
{
    ULYSSES_ASSERT(table != NULL, "Motor table must be non NULL");
    ULYSSES_ASSERT(table->thrust_n != NULL && table->roll_torque_nm != NULL &&
                       table->throttle_pct != NULL,
                   "Motor table arrays must be non NULL");
    ULYSSES_ASSERT(table->thrust_count >= 2 && table->torque_count >= 2,
                   "Motor table needs at least 2 points on each axis");
    ULYSSES_ASSERT(throttle_pct != NULL, "Throttle output must be non NULL");

    if (!isfinite(thrust_n) || !isfinite(roll_torque_nm)) {
        *throttle_pct = 0.0f;
        return MOTOR_TABLE_INVALID_INPUT;
    }

    bool clamped = clamp_to_axis(table->thrust_n, table->thrust_count, &thrust_n);
    clamped |= clamp_to_axis(table->roll_torque_nm, table->torque_count, &roll_torque_nm);

    float thrust_fraction;
    float torque_fraction;
    uint8_t i = find_interval(table->thrust_n, table->thrust_count, thrust_n, &thrust_fraction);
    uint8_t j =
        find_interval(table->roll_torque_nm, table->torque_count, roll_torque_nm, &torque_fraction);

    const float *row0 = &table->throttle_pct[i * table->torque_count];
    const float *row1 = &table->throttle_pct[(i + 1) * table->torque_count];

    float low = row0[j] + torque_fraction * (row0[j + 1] - row0[j]);
    float high = row1[j] + torque_fraction * (row1[j + 1] - row1[j]);

    *throttle_pct = low + thrust_fraction * (high - low);

    return clamped ? MOTOR_TABLE_CLAMPED : MOTOR_TABLE_OK;
}

motor_table_status_t motor_map_lookup(const motor_map_t *map, float thrust_n, float roll_torque_nm,
                                      float *top_pct, float *bottom_pct)
{
    ULYSSES_ASSERT(map != NULL, "Motor map must be non NULL");
    ULYSSES_ASSERT(top_pct != NULL && bottom_pct != NULL, "Throttle outputs must be non NULL");

    motor_table_status_t top_status =
        motor_table_lookup(&map->top, thrust_n, roll_torque_nm, top_pct);
    motor_table_status_t bottom_status =
        motor_table_lookup(&map->bottom, thrust_n, roll_torque_nm, bottom_pct);

    if (top_status == MOTOR_TABLE_INVALID_INPUT || bottom_status == MOTOR_TABLE_INVALID_INPUT) {
        return MOTOR_TABLE_INVALID_INPUT;
    }
    if (top_status == MOTOR_TABLE_CLAMPED || bottom_status == MOTOR_TABLE_CLAMPED) {
        return MOTOR_TABLE_CLAMPED;
    }
    return MOTOR_TABLE_OK;
}
