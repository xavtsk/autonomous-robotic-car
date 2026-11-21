/** @file line_recovery.c
 * @brief Track line observations while recovering after an obstacle detour.
 */
#include "line_recovery.h"

/** @brief Clear all recovery state, including a previous confirmation.
 * @see line_recovery.h for the non-null ownership contract.
 */
void line_recovery_reset (line_recovery_t * p_state)
{
    *p_state = (line_recovery_t){0};
}

/** @brief Track eligible contact without clearing a latched confirmation.
 * @see line_recovery.h for timing, input qualification and failure effects.
 */
bool line_recovery_observe (line_recovery_t * p_state, uint32_t now,
                            bool b_is_healthy, bool b_is_tape,
                            bool b_is_junction, bool b_is_in_corridor)
{
    if ((((!b_is_healthy) || (!b_is_tape)) || b_is_junction) ||
        (!b_is_in_corridor))
    {
        p_state->b_is_touching = false;
        return p_state->b_is_confirmed;
    }

    if (!p_state->b_is_touching)
    {
        p_state->b_is_touching = true;
        p_state->since_us      = now;
    }

    if ((now - p_state->since_us) >= 30000U)
    {
        p_state->b_is_confirmed = true;
    }

    return p_state->b_is_confirmed;
}

/* End of file. */

