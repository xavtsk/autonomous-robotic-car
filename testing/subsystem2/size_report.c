#include "motion_codec.h"
#include <stdio.h>

/* Exact original calibration field order, retained only for comparison. */
typedef struct {
    bool b_valid;
    float counts_per_rev, max_speed_mps;
    bool b_m1_inverted, b_m2_inverted, b_enc_l_inverted, b_enc_r_inverted;
    float counts_per_rev_l, counts_per_rev_r, top_speed_l_mps, top_speed_r_mps;
    bool b_enc_l_ok, b_enc_r_ok, b_encoders_swapped;
} original_calibration_t;

/* Equivalent decoded view, NOT an actual old queue message. */
typedef struct {
    float left_mps, right_mps;
    motion_state_t state;
    motion_fault_t fault;
    bool busy, speed_valid;
} decoded_motion_view_t;

_Static_assert(sizeof(original_calibration_t) == 36, "baseline calibration");
_Static_assert(sizeof(decoded_motion_view_t) == 12, "requires Arm short enums");
_Static_assert(sizeof(motion_snapshot_t) == 4, "copied motion result");

int main(void)
{
    printf("Current teammate calibration decoded view: %zu bytes (includes abort flag)\n",
           sizeof(motion_calibration_t));
    printf("Equivalent decoded reporting view / new packed copy: %zu / %zu bytes\n",
           sizeof(decoded_motion_view_t), sizeof(motion_snapshot_t));
    printf("Teammate status/report/command: %zu/%zu/%zu bytes; bounded report capacity %u\n",
           sizeof(uint64_t), sizeof(uint16_t), sizeof(uint32_t), MOTION_REPORT_QUEUE_LEN);
    puts("Buddy 1 MQTT motion packet remains 4 bytes including version.");
    return 0;
}
