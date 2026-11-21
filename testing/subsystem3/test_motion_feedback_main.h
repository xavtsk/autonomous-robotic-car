/** @file test_motion_feedback_main.h
 * @brief RTOS boundary for the motion feedback fixture.
 *
 * The fixture simulates mutexes, tasks and clock registers. Its trace
 * consumers also use these declarations when compiling the fixture in their
 * own translation unit.
 */
#ifndef TESTING_SUBSYSTEM3_TEST_MOTION_FEEDBACK_MAIN_H
#define TESTING_SUBSYSTEM3_TEST_MOTION_FEEDBACK_MAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "motion_kernel_stub.h"

int motion_fixture_run (void);
#ifdef TEST_MOTION_STORAGE
bool motion_fixture_locked (void);
void motion_fixture_set_time (uint32_t timestamp_us);
void motion_fixture_enable_encoder (void);
void motion_fixture_fill_reports (void);
#endif

#endif /* TESTING_SUBSYSTEM3_TEST_MOTION_FEEDBACK_MAIN_H */

/* End of file. */

