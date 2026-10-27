#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H
#include "../app.h"
#include "terrain_monitor.h"
#define S4_PRI_MAIN 18
ER subsystem4_init(void);
ER subsystem4_start(void);
/* Task-context APIs. Bounded snapshot/event access; one event consumer.
 * Reset begins a new run; recalibrate requires stationary, level placement. */
bool subsystem4_snapshot(terrain_result_t *out);
bool subsystem4_event(terrain_result_t *out, bool *lost);
bool subsystem4_reset_run(bool recalibrate);
bool subsystem4_set_magnetic_calibration(const float offset[3], const float scale[3]);
#if APP_TEST
void subsystem4_test(void);
#endif
#endif
