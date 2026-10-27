/* Legacy Buddy 1/USB reporting view over the teammate's coherent status API.
 * No request, report consumption, PID input or control precision change. */
#include "motion_codec.h"
bool motion_read_snapshot(motion_snapshot_t *out)
{
    motion_status_t status;
    float left, right;
    bool valid;
    if (!out || !motion_get_status(&status)) { return false; }
    left = status.speed_left_mps * 1000.0f;
    right = status.speed_right_mps * 1000.0f;
    valid = status.b_valid && !status.b_busy && left == left && right == right &&
        left >= -MOTION_REPORT_MAX_MMS && left <= MOTION_REPORT_MAX_MMS &&
        right >= -MOTION_REPORT_MAX_MMS && right <= MOTION_REPORT_MAX_MMS;
    return motion_snapshot_pack(out, status.state, status.fault, status.b_busy,
        valid, valid ? (int32_t)left : 0, valid ? (int32_t)right : 0);
}
