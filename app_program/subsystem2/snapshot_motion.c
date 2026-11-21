/** @file snapshot_motion.c
 * @brief Legacy Buddy 1/USB reporting view over the teammate's coherent status
 * API.
 *
 * No request, report consumption, PID input or control precision change.
 */
#include "snapshot_motion.h"
#include "../app_types.h"

/** @brief Copy the current status into the legacy reporting packet without
 * consuming a completion. Speeds are reported in whole millimetres per second
 * when valid. Return false for a missing output, unavailable status or rejected
 * packing. The caller owns the output. */
bool snapshot_motion_read (codec_motion_snapshot_t * p_out)
{
    motion_status_t status;

    if ((0 == p_out) || (!motion_get_status(&status)))
    {
        return false;
    }

    const float32_t left  = status.speed_left_mps * 1000.0f;
    const float32_t right = status.speed_right_mps * 1000.0f;
    const bool      b_is_valid =
        (((((((status.b_is_status_valid) && (!status.b_is_busy)) &&
             (left == left)) &&
            (right == right)) &&
           (left >= -MOTION_REPORT_MAX_MMS)) &&
          (left <= MOTION_REPORT_MAX_MMS)) &&
         (right >= -MOTION_REPORT_MAX_MMS)) &&
        (right <= MOTION_REPORT_MAX_MMS);
    /* Only valid finite values within +/-510 mm/s reach the casts.
     * The unavailable branch uses zero for each wheel. */
    return codec_motion_pack(
        p_out, status.state, status.fault, status.b_is_busy, b_is_valid,
        b_is_valid ? (int32_t)left : 0, b_is_valid ? (int32_t)right : 0);
}

/* End of file. */

