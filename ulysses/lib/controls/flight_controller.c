#include <stddef.h>
#include <string.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/controls/flight_controller.h"

#define FC_GAINS_DIRTY (0x4u)
#define FC_GAINS_INDEX_MASK (0x3u)

_Static_assert(sizeof(((flight_controller_t *)0)->gains_slots) /
                       sizeof(((flight_controller_t *)0)->gains_slots[0]) <=
                   FC_GAINS_INDEX_MASK + 1u,
               "Gains slot index does not fit in FC_GAINS_INDEX_MASK");
_Static_assert((FC_GAINS_DIRTY & FC_GAINS_INDEX_MASK) == 0u, "Dirty bit overlaps slot index");

void flight_controller_init(flight_controller_t *fc, const fc_config_t *config)
{
    ULYSSES_ASSERT(fc != NULL, "Flight controller must be non NULL");
    ULYSSES_ASSERT(config != NULL, "Flight controller config must be non NULL");

    memset(fc, 0, sizeof(*fc));
    fc->config = *config;

    fc->gains_write_idx = 0;
    fc->gains_read_idx = 1;
    atomic_init(&fc->gains_latest, 2);
}

void flight_controller_set_gains(flight_controller_t *fc,
                                 const fc_torque_controller_config_t *torque,
                                 const fc_gimbal_controller_config_t *gimbal)
{
    ULYSSES_ASSERT(fc != NULL, "Flight controller must be non NULL");
    ULYSSES_ASSERT(torque != NULL, "Torque gains must be non NULL");
    ULYSSES_ASSERT(gimbal != NULL, "Gimbal gains must be non NULL");

    fc_gains_t *slot = &fc->gains_slots[fc->gains_write_idx];
    slot->torque = *torque;
    slot->gimbal = *gimbal;

    // Publish the written slot and take ownership of the slot it replaces
    unsigned prev = atomic_exchange(&fc->gains_latest, fc->gains_write_idx | FC_GAINS_DIRTY);
    fc->gains_write_idx = prev & FC_GAINS_INDEX_MASK;
}

void flight_controller_step(flight_controller_t *fc, const eskf_nominal_state_t *state,
                            const fc_ref_t *ref, fc_command_t *cmd_out, float dt_s)
{
    ULYSSES_ASSERT(fc != NULL, "Flight controller must be non NULL");
    ULYSSES_ASSERT(state != NULL, "State must be non NULL");
    ULYSSES_ASSERT(ref != NULL, "Reference must be non NULL");
    ULYSSES_ASSERT(cmd_out != NULL, "Command output must be non NULL");

    // Written so NaN also fails. Flag first: the assert handler does not return.
    if (!(dt_s > 0.0f)) {
        fc->faults |= FC_FAULT_BAD_DT;
    }
    ULYSSES_ASSERT(dt_s > 0.0f, "Time step must be positive");

    // A late tick is recoverable: flag it and cap dt so one slow step cannot jump the integrator
    // or spike the derivative.
    float dt = dt_s;
    if (dt > FC_DT_MAX_S) {
        fc->faults |= FC_FAULT_MISSED_TICK;
        dt = FC_DT_MAX_S;
    }

    if (atomic_load(&fc->gains_latest) & FC_GAINS_DIRTY) {
        unsigned prev = atomic_exchange(&fc->gains_latest, fc->gains_read_idx);
        fc->gains_read_idx = prev & FC_GAINS_INDEX_MASK;

        const fc_gains_t *gains = &fc->gains_slots[fc->gains_read_idx];
        fc->config.torque_gains = gains->torque;
        fc->config.gimbal_gains = gains->gimbal;
    }

    // TODO: implement the control law using dt
    (void)dt;
    memset(cmd_out, 0, sizeof(*cmd_out));
}
