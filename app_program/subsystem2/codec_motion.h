/** @file codec_motion.h
 * @brief Define the shared packed motion snapshot and codec interface.
 */
/* Tribunal 1 copied motion-result format. Explicit bits, not C struct layout.
 * This is a reporting snapshot, NEVER a quantised input to the PID. */
#ifndef BUDDY2_MOTION_CODEC_H
#define BUDDY2_MOTION_CODEC_H

#include "motion.h"

#define MOTION_SNAPSHOT_BYTES 4U
#define MOTION_REPORT_MAX_MMS 510

/* LSB-first: left magnitude 0..7, left reverse 8; right magnitude 9..16,
 * right reverse 17; state 18..20; primary fault 21..23; busy 24;
 * speed-valid 25; unused 26..31 must be zero. Speeds have 2 mm/s steps. */
typedef struct
{
    uint8_t bytes[MOTION_SNAPSHOT_BYTES];
} codec_motion_snapshot_t;

bool codec_motion_pack (codec_motion_snapshot_t * p_out, motion_state_t state,
                        motion_fault_t fault, bool b_is_busy,
                        bool b_is_speed_valid, int32_t left_mms,
                        int32_t right_mms);

bool           codec_motion_valid (const codec_motion_snapshot_t * p_value);
motion_state_t codec_motion_state (const codec_motion_snapshot_t * p_value);
motion_fault_t codec_motion_fault (const codec_motion_snapshot_t * p_value);
bool           codec_motion_busy (const codec_motion_snapshot_t * p_value);
bool    codec_motion_speed_valid (const codec_motion_snapshot_t * p_value);
int16_t codec_motion_left_mms (const codec_motion_snapshot_t * p_value);
int16_t codec_motion_right_mms (const codec_motion_snapshot_t * p_value);

/* Task context only. Copies under the existing motion mutex, releases it,
 * then packs. Does NOT consume MOTION_COMPLETE or change any control state.
 * False: invalid output pointer, kernel object absent, or lock failure. */
bool snapshot_motion_read (codec_motion_snapshot_t * p_out);

#endif

/* End of file. */

