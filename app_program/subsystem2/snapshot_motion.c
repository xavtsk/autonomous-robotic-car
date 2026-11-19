/** @file snapshot_motion.c
 * @brief Legacy Buddy 1/USB reporting view over the teammate's coherent status
 * API.
 */
/* Legacy Buddy 1/USB reporting view over the teammate's coherent status API.
 * No request, report consumption, PID input or control precision change. */
#include "snapshot_motion.h"
#include "../app_types.h"

bool motion_read_snapshot (motion_snapshot_t * p_out)
{
    motion_status_t status;
    float32_t left, right;
    bool valid;
    if (!p_out || !motion_get_status(&status))
    {
        return false;
    }
    left = status.speed_left_mps * 1000.0f;
    right = status.speed_right_mps * 1000.0f;
    valid = status.b_valid && !status.b_busy && left == left &&
            right == right && left >= -MOTION_REPORT_MAX_MMS &&
            left <= MOTION_REPORT_MAX_MMS && right >= -MOTION_REPORT_MAX_MMS &&
            right <= MOTION_REPORT_MAX_MMS;
    return motion_snapshot_pack(p_out, status.state, status.fault,
                                status.b_busy, valid, valid ? (int32_t)left : 0,
                                valid ? (int32_t)right : 0);
}

/* End of file. */

