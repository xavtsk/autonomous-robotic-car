/** @file line_recovery.h
 * @brief Track line observations while recovering after an obstacle detour.
 */
#ifndef BUDDY5_LINE_RECOVERY_H
#define BUDDY5_LINE_RECOVERY_H
#include "s5_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t since_us;
    unsigned b_is_touching : 1;
    unsigned b_is_confirmed : 1;
} line_recovery_t;

/** @brief Clear the contact timer and latched line confirmation.
 * @param[out] p_state Non-null, exclusively owned writable recovery state.
 * @note No null check or synchronization is performed. The avoidance state
 * owns this storage; no caller pointer is retained.
 */
void line_recovery_reset (line_recovery_t * p_state);
/* Evidence must be near the predicted original corridor, fresh, sustained,
 * non-junction tape. White-only 'between sensors' tracking is NOT evidence. */
/** @brief Update sustained tape contact and return the confirmation latch.
 * @param[in,out] p_state Non-null initialized state, exclusively caller-owned.
 * @param[in] now Timestamp in microseconds modulo 2^32, from one clock.
 * @param[in] b_is_healthy Caller has established that the sensor input is
 * usable.
 * @param[in] b_is_tape At least one sensor has positive tape evidence.
 * @param[in] b_is_junction Whether the input indicates a junction.
 * @param[in] b_is_in_corridor Whether the position is in the expected track
 * area.
 * @return The latched confirmation, which stays true until reset.
 * @details An unhealthy, white, junction or out-of-corridor observation clears
 * touching but preserves since_us and confirmed. The next eligible observation
 * starts a new contact interval. Eligible observations at least 30000 us after
 * that start latch confirmed. No freshness or inter-call gap check is made;
 * the caller supplies healthy/corridor decisions and regular observations.
 * @pre Elapsed time is less than one complete uint32_t clock cycle; unsigned
 * subtraction handles a timestamp wrap. No locking or hardware I/O occurs.
 */
bool line_recovery_observe (line_recovery_t * p_state, uint32_t now,
                            bool b_is_healthy, bool b_is_tape,
                            bool b_is_junction, bool b_is_in_corridor);
#endif

/* End of file. */

