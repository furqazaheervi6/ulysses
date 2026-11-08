#ifndef ULYSSES_LIB_DSHOT_DSHOT
#define ULYSSES_LIB_DSHOT_DSHOT

#include <stdbool.h>
#include <stdint.h>

/**
 * @file
 * DShot frame encoding, with no hardware dependency
 *
 * The team docs have no DShot design yet (the "Working with DShot" page only links an outside
 * article), so this follows the standard DShot protocol:
 *  - a frame is 16 bits, sent most significant bit first: 11 bits of value, 1 telemetry request
 *    bit and a 4 bit checksum;
 *  - value 0 means stop, 1 to 47 are special commands, and 48 to 2047 is the throttle;
 *  - a 1 bit is high for 3/4 of the bit period and a 0 bit is high for 3/8 of it.
 */

#define DSHOT_FRAME_BITS (16u)

#define DSHOT_VALUE_STOP (0u)
#define DSHOT_VALUE_THROTTLE_MIN (48u)
#define DSHOT_VALUE_MAX (2047u)

/**
 * @brief Builds a DShot frame
 *
 * @param value 0 to DSHOT_VALUE_MAX. Larger values are clamped to DSHOT_VALUE_MAX.
 * @param telemetry_request Sets the telemetry request bit
 * @return The 16 bit frame, including the checksum
 */
uint16_t dshot_encode_frame(uint16_t value, bool telemetry_request);

/**
 * @brief Converts a throttle in percent to a DShot value
 *
 * 0 % and below, and NaN, give DSHOT_VALUE_STOP, so a bad input never commands the motor. Above 0 %
 * the value runs from DSHOT_VALUE_THROTTLE_MIN up to DSHOT_VALUE_MAX at 100 %. Anything above 100 %
 * is clamped to 100 %.
 *
 * @param percent Throttle in percent
 * @return A DShot value, never a special command
 */
uint16_t dshot_value_from_percent(float percent);

/**
 * @brief Converts a frame into the high time of each bit, for a timer that drives the signal
 *
 * Entry 0 is the most significant bit, which is sent first. Use the results as timer compare
 * values with bit_period_ticks as the timer period.
 *
 * @param frame Frame from dshot_encode_frame()
 * @param bit_period_ticks Timer ticks in one bit period, e.g. the timer clock divided by the DShot
 *                         bit rate
 * @param[out] high_ticks_out Non-NULL array of DSHOT_FRAME_BITS entries
 */
void dshot_frame_to_high_ticks(uint16_t frame, uint16_t bit_period_ticks,
                               uint16_t high_ticks_out[DSHOT_FRAME_BITS]);

#endif /* ULYSSES_LIB_DSHOT_DSHOT */
