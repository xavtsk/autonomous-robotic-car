/** @file motion_storage_main.h
 * @brief Mutex observations for the retained motion state fixture.
 *
 * The fake kernel asks this fixture whether to grant a lock attempt and
 * records each release before checking ownership.
 */
#ifndef TESTING_COMPLIANCE_MOTION_STORAGE_MAIN_H
#define TESTING_COMPLIANCE_MOTION_STORAGE_MAIN_H

#include "motion_kernel_stub.h"

#include <stdbool.h>

#ifdef MOTION_STORAGE_PROBE
#include "motion.h"
#include <stddef.h>
#include <string.h>

ID   motion_storage_test_exchange_mutex (ID h_mutex);
void motion_storage_test_seed_calibration (
    const motion_calibration_t * p_value);
void motion_storage_test_clear_calibration (void);
void motion_storage_test_seed_feedback (uint32_t stamp, uint32_t epoch,
                                        uint64_t centre, uint32_t losses,
                                        unsigned flags);
void motion_storage_test_post_to_full (void ( * p_fill)(void),
                                       const motion_report_t * p_report);
void motion_storage_test_sizes (size_t sizes[3]);
#endif

bool motion_storage_allow_lock (bool b_is_held);
void motion_storage_observe_unlock (void);

#endif /* TESTING_COMPLIANCE_MOTION_STORAGE_MAIN_H */

/* End of file. */

