#ifndef S5_API_H
#define S5_API_H
#include "s5_codec.h"
typedef enum {
    S5_SNAPSHOT_OK, S5_SNAPSHOT_BUSY, S5_SNAPSHOT_UNAVAILABLE, S5_SNAPSHOT_INVALID
} s5_snapshot_result_t;
/* BUSY means only a failed nonblocking lock attempt; it is not sensor data.
 * A successful read may still contain range_valid=false or no_echo=true. */
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out);
bool subsystem5_snapshot(s5_observation_t *out);
bool subsystem5_request_scan(uint16_t id);
/* One reading at a side angle (30..60 or 120..150) while stopped; the result
 * arrives as scan_done with this id and that side's clearance updated. */
bool subsystem5_request_peek(uint16_t id,unsigned angle);
void subsystem5_cancel_scan(void);
/* Controller-owned bypass status for telemetry, one 16-bit word:
 * state bits 0-2, fault 3-5, route 6-7, leg 8-11, leg-3 pass step 12-14,
 * side swapped 15. Written by the controller task, read by telemetry. */
void subsystem5_set_controller_status(unsigned state,unsigned fault,unsigned route,
                                      unsigned leg,unsigned pass,unsigned swapped);
uint16_t subsystem5_controller_status(void);
/* Angle the scanning servo was last commanded to, 1..180 degrees
 * (90 = straight ahead); 0 before the servo has been positioned. */
uint8_t subsystem5_servo_angle(void);
#endif
