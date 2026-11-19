/** @file s5_api.h
 * @brief Expose obstacle snapshots, scan requests and controller status
 * exchange.
 */
#ifndef S5_API_H
#define S5_API_H
#include "s5_codec.h"

typedef enum
{
    S5_SNAPSHOT_OK,
    S5_SNAPSHOT_BUSY,
    S5_SNAPSHOT_UNAVAILABLE,
    S5_SNAPSHOT_INVALID
} s5_snapshot_result_t;

/* BUSY means only a failed nonblocking lock attempt; it is not sensor data.
 * A successful read may still contain range_valid=false or no_echo=true. */
s5_snapshot_result_t subsystem5_read_snapshot (s5_observation_t * p_out);
bool subsystem5_snapshot (s5_observation_t * p_out);
bool subsystem5_request_scan (uint16_t value_id);
/* One reading at a side angle (30..60 or 120..150) while stopped; the result
 * arrives as scan_done with this id and that side's clearance updated. */
bool subsystem5_request_peek (uint16_t value_id, unsigned angle);
void subsystem5_cancel_scan (void);
/* Controller-owned bypass status for telemetry, one 16-bit word:
 * state bits 0-2, fault 3-5, route 6-7, leg 8-11, leg-3 pass step 12-14,
 * side swapped 15. Written by the controller task, read by telemetry. */
void subsystem5_set_control_status (unsigned state, unsigned fault,
                                    unsigned route, unsigned leg, unsigned pass,
                                    unsigned swapped);
uint16_t subsystem5_controller_status (void);
/* Angle the scanning servo was last commanded to, 1..180 degrees
 * (90 = straight ahead); 0 before the servo has been positioned. */
uint8_t subsystem5_servo_angle (void);
/* Raw readings of the last finished scan or side peek (for telemetry): copies
 * them to out with their number (0 before the first scan) and that scan's id;
 * peek = single side-peek reading. A failed scan ends with one "no reading"
 * entry at the angle where it stopped. False = mutex busy, try again. */
bool subsystem5_scan_samples (uint16_t * p_id, bool * p_peek,
                              s5_sample_t p_out[S5_SAMPLES_MAX],
                              unsigned * p_count);
#endif

/* End of file. */

