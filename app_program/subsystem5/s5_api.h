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
void subsystem5_cancel_scan(void);
void subsystem5_set_controller_status(unsigned state,unsigned fault,unsigned route);
uint8_t subsystem5_controller_status(void);
#endif
