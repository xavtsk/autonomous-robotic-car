/** @file firmware_main.h
 * @brief RTOS boundary for diagnostic publication tests.
 *
 * The harness supplies mutex and console callbacks. Encoder and motion
 * snapshots use their subsystem declarations.
 */
#ifndef TESTING_DIAGNOSTICS_FIRMWARE_MAIN_H
#define TESTING_DIAGNOSTICS_FIRMWARE_MAIN_H

#include "diag_kernel.h"
#include "../../app_program/app_types.h"
#include "../../app_program/diagnostics/robot_diagnostics.h"
#include <stdbool.h>
#include <stddef.h>

typedef struct
{
    uint32_t samples;
    uint32_t publish_skips;
    uint32_t gap_min_us;
    uint32_t gap_max_us;
    uint32_t exec_max_us;
    bool     b_is_gap_valid;
} robot_diagnostics_test_summary_t;

typedef struct
{
    size_t producer_bytes;
    size_t published_bytes;
    size_t total_bytes;
} robot_diagnostics_test_layout_t;

robot_diagnostics_test_summary_t
robot_diagnostics_test_published (robot_diagnostics_source_t source);
robot_diagnostics_test_layout_t robot_diagnostics_test_layout (void);
uint32_t robot_diagnostics_test_increment (uint32_t value);
bool     robot_diagnostics_test_estimate (float32_t speed_mps, int32_t * p_out);

#endif /* TESTING_DIAGNOSTICS_FIRMWARE_MAIN_H */

/* End of file. */

