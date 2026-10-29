/* Observation-only controller snapshot. Separate topic/version from IR/v2.
 * Explicit bit widths; no C struct layout is transmitted. Signed quantities
 * use two's complement. No callbacks to motion/control exist in the codec. */
#ifndef CONTROL_DEBUG_H
#define CONTROL_DEBUG_H
#include <stdbool.h>
#include <stdint.h>
#define CONTROL_DEBUG_FIELDS(U, S) \
 U(timestamp_us,32) U(motion_timestamp_us,32) U(sequence,8) \
 U(motion_available,1) U(motion_valid,1) U(braked,1) U(busy,1) \
 U(motion_state,3) U(motion_fault,3) \
 S(setpoint_left,13) S(setpoint_right,13) S(measured_left,13) S(measured_right,13) \
 S(duty_left,8) S(duty_right,8) S(encoder_left,20) S(encoder_right,20) \
 U(drive_state,3) U(reason,4) U(pending,3) S(line_error,8) U(line_flags,5) \
 U(barcode_status,3) U(accepted_calibration,3) U(readiness,5) \
 U(barcode_events,16) U(junction_events,16) U(lost_events,16) \
 U(sample_gap_us,32) U(run_max_gap_us,32) \
 U(left_white,12) U(left_black,12) U(left_white_limit,10) U(left_black_limit,10) \
 U(right_white,12) U(right_black,12) U(right_white_limit,10) U(right_black_limit,10) \
 U(barcode_white,12) U(barcode_black,12) U(barcode_white_limit,10) U(barcode_black_limit,10) \
 U(cal_sensor,2) U(collecting,1) U(white_min,12) U(white_max,12) \
 U(black_min,12) U(black_max,12) U(white_count,16) U(black_count,16)
#define CD_COUNT(name,width) + width
enum { CONTROL_DEBUG_BITS = 0 CONTROL_DEBUG_FIELDS(CD_COUNT,CD_COUNT),
       CONTROL_DEBUG_PACKET_SIZE = 1 + (CONTROL_DEBUG_BITS + 7) / 8 };
#undef CD_COUNT
#define CD_U(name,width) uint32_t name;
#define CD_S(name,width) int32_t name;
typedef struct { CONTROL_DEBUG_FIELDS(CD_U,CD_S) } control_debug_sample_t;
#undef CD_U
#undef CD_S
typedef struct { uint8_t bytes[CONTROL_DEBUG_PACKET_SIZE]; } control_debug_packet_t;
bool control_debug_pack(const control_debug_sample_t *sample, control_debug_packet_t *packet);
bool control_debug_unpack(const control_debug_packet_t *packet, control_debug_sample_t *sample);
#endif
