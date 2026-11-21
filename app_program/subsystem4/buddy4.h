/** @file buddy4.h
 * @brief Own IMU acquisition, terrain processing and protected snapshots and
 * events.
 */
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
bool buddy4_snapshot (terr_data_result_t * p_out);
bool buddy4_event (terr_data_result_t * p_out, bool * pb_has_lost_events);
bool buddy4_reset_run (bool b_should_recalibrate);
/* Sole controller reset; token is acknowledged only by a post-reset sample.
 * Read returns a coherent packet/run/count tuple, false while reset is pending.
 * Count saturates at UINT16_MAX and includes completed valid humps only. */
bool buddy4_begin_run (uint16_t * p_token);
bool buddy4_read (terr_data_result_t * p_out, uint16_t * p_token,
                  uint16_t * p_humps);
bool buddy4_set_mag_calibration (const float32_t p_offset[3],
                                 const float32_t p_scale[3]);
#if APP_TEST
void subsystem4_test (void);
#endif
#endif

/* End of file. */

