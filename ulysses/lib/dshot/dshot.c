#include <math.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/dshot/dshot.h"

uint16_t dshot_encode_frame(uint16_t value, bool telemetry_request)
{
    if (value > DSHOT_VALUE_MAX) {
        value = DSHOT_VALUE_MAX;
    }

    uint16_t payload = (uint16_t)((value << 1) | (telemetry_request ? 1u : 0u));
    uint16_t checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0Fu;

    return (uint16_t)((payload << 4) | checksum);
}

uint16_t dshot_value_from_percent(float percent)
{
    // Written so NaN also gives stop
    if (!(percent > 0.0f)) {
        return DSHOT_VALUE_STOP;
    }
    if (percent >= 100.0f) {
        return DSHOT_VALUE_MAX;
    }

    float span = (float)(DSHOT_VALUE_MAX - DSHOT_VALUE_THROTTLE_MIN);
    return (uint16_t)(DSHOT_VALUE_THROTTLE_MIN + (uint16_t)(percent * 0.01f * span + 0.5f));
}

void dshot_frame_to_high_ticks(uint16_t frame, uint16_t bit_period_ticks,
                               uint16_t high_ticks_out[DSHOT_FRAME_BITS])
{
    ULYSSES_ASSERT(high_ticks_out != NULL, "High tick output must be non NULL");

    uint16_t one_ticks = (uint16_t)((uint32_t)bit_period_ticks * 3u / 4u);
    uint16_t zero_ticks = (uint16_t)((uint32_t)bit_period_ticks * 3u / 8u);

    for (uint32_t i = 0; i < DSHOT_FRAME_BITS; i++) {
        bool bit = (frame >> (DSHOT_FRAME_BITS - 1u - i)) & 1u;
        high_ticks_out[i] = bit ? one_ticks : zero_ticks;
    }
}
