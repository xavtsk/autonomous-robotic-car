/** @file wheel_sizes_main.c
 * @brief Report baseline and packed motion representation sizes.
 */
#include "wheel_sizes_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/build_assert.h"
#include "codec_motion.h"
#include <stdio.h>

/* Exact original calibration field order, retained only for comparison. */
typedef struct
{
    bool      b_valid;
    float32_t counts_per_rev;
    float32_t max_speed_mps;
    bool      b_is_left_motor_inverted;
    bool      b_is_right_motor_inverted;
    bool      b_enc_l_inverted;
    bool      b_enc_r_inverted;
    float32_t counts_per_rev_l;
    float32_t counts_per_rev_r;
    float32_t top_speed_l_mps;
    float32_t top_speed_r_mps;
    bool      b_enc_l_ok;
    bool      b_enc_r_ok;
    bool      b_encoders_swapped;
} original_calibration_t;

/* Equivalent decoded view, NOT an actual old queue message. */
typedef struct
{
    float32_t      left_mps;
    float32_t      right_mps;
    motion_state_t state;
    motion_fault_t fault;
    bool           b_is_busy;
    bool           b_is_speed_valid;
} decoded_motion_view_t;

BUILD_ASSERT(sizeof(original_calibration_t) == 36U, calibration_layout_t);
BUILD_ASSERT(sizeof(decoded_motion_view_t) == 12U, motion_view_layout_t);
BUILD_ASSERT(sizeof(codec_motion_snapshot_t) == 4U, motion_result_size_t);

/** @brief Print the host compiler sizes of calibration, motion feedback and
 * retained wheel state for comparison.
 */
int main (void)
{
    printf("Current teammate calibration decoded view: %zu bytes (includes "
           "abort flag)\n",
           sizeof(motion_calibration_t));
    printf("Equivalent decoded reporting view / new packed copy: %zu / %zu "
           "bytes\n",
           sizeof(decoded_motion_view_t), sizeof(codec_motion_snapshot_t));
    printf("Teammate status/report/command: %zu/%zu/%zu bytes; bounded report "
           "capacity %u\n",
           sizeof(uint64_t), sizeof(uint16_t), sizeof(uint32_t),
           MOTION_REPORT_QUEUE_LEN);
    puts("Buddy 1 MQTT motion packet remains 4 bytes including version.");
    return 0;
}

/* End of file. */

