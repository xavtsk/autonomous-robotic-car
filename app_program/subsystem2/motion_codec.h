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
typedef struct { uint8_t bytes[MOTION_SNAPSHOT_BYTES]; } motion_snapshot_t;

bool motion_snapshot_pack(motion_snapshot_t *out, motion_state_t state,
                          motion_fault_t fault, bool busy, bool speed_valid,
                          int32_t left_mms, int32_t right_mms);
bool motion_snapshot_valid(const motion_snapshot_t *value);
motion_state_t motion_snapshot_state(const motion_snapshot_t *value);
motion_fault_t motion_snapshot_fault(const motion_snapshot_t *value);
bool motion_snapshot_busy(const motion_snapshot_t *value);
bool motion_snapshot_speed_valid(const motion_snapshot_t *value);
int16_t motion_snapshot_left_mms(const motion_snapshot_t *value);
int16_t motion_snapshot_right_mms(const motion_snapshot_t *value);

/* Task context only. Copies under the existing motion mutex, releases it,
 * then packs. Does NOT consume MOTION_COMPLETE or change any control state.
 * False: invalid output pointer, kernel object absent, or lock failure. */
bool motion_read_snapshot(motion_snapshot_t *out);

#endif
