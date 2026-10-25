#include <stddef.h>
#include <string.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/flight_controller.h"

void flight_controller_init(flight_controller_t *fc, const fc_config_t *config)
{
    ULYSSES_ASSERT(fc != NULL, "Flight controller must be non NULL");
    ULYSSES_ASSERT(config != NULL, "Flight controller config must be non NULL");

    memset(fc, 0, sizeof(*fc));
    fc->config = *config;
}

void flight_controller_step(flight_controller_t *fc, const eskf_nominal_state_t *state,
                            const fc_ref_t *ref, fc_command_t *cmd_out, float dt_s)
{
    ULYSSES_ASSERT(fc != NULL, "Flight controller must be non NULL");
    ULYSSES_ASSERT(state != NULL, "State must be non NULL");
    ULYSSES_ASSERT(ref != NULL, "Reference must be non NULL");
    ULYSSES_ASSERT(cmd_out != NULL, "Command output must be non NULL");

    (void)dt_s;

    // TODO: implement the control law
    memset(cmd_out, 0, sizeof(*cmd_out));
}
