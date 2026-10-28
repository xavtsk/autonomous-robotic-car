#ifndef S5_API_H
#define S5_API_H
#include "s5_codec.h"
bool subsystem5_snapshot(s5_observation_t *out);
bool subsystem5_request_scan(uint16_t id);
void subsystem5_cancel_scan(void);
void subsystem5_set_controller_status(unsigned state,unsigned fault,unsigned route);
uint8_t subsystem5_controller_status(void);
#endif
