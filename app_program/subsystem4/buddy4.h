#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H
#include "../app_types.h"
#include "../app.h"
#include "terr_data.h"
#define S4_PRI_MAIN 18
ER subsystem4_init (void);
ER subsystem4_start (void);
/* Task-context APIs. Bounded snapshot/event access; one event consumer.
 * Reset begins a new run; recalibrate requires stationary, level placement. */
bool subsystem4_snapshot (terrain_result_t * p_out);
bool subsystem4_event (terrain_result_t * p_out, bool * p_lost);
bool subsystem4_reset_run (bool recalibrate);
/* Sole controller reset; token is acknowledged only by a post-reset sample.
 * Read returns a coherent packet/run/count tuple, false while reset is pending.
 * Count saturates at UINT16_MAX and includes completed valid humps only. */
bool subsystem4_begin_run (uint16_t * p_token);
bool subsystem4_read (terrain_result_t * p_out, uint16_t * p_token,
                      uint16_t * p_humps);
bool subsystem4_set_mag_calibration (const float32_t p_offset[3],
                                     const float32_t p_scale[3]);
#if APP_TEST
void subsystem4_test (void);
#endif
#endif
