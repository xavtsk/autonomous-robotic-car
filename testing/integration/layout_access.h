/** @file layout_access.h
 * @brief Preserve host observations across the two reviewed private layouts.
 */
#ifndef TESTING_INTEGRATION_STORAGE_COMPAT_H
#define TESTING_INTEGRATION_STORAGE_COMPAT_H

/* Expand these macros only inside the subject's host probes, where the private
 * state is declared. Source detection selects the corresponding layout. */
#if defined(TEST_S3_COMPACT_STORAGE) && TEST_S3_COMPACT_STORAGE
#if defined(TEST_S3_LEGACY_PREDICATES) && TEST_S3_LEGACY_PREDICATES
#define TEST_S3_RESET_WAITING (g_sampling.reset_waiting)
#define TEST_S3_RESET_REMOTE (g_sampling.reset_remote)
#define TEST_S3_COLLECTING (g_sampling.collecting)
#define TEST_S3_SENSOR (g_sampling.sensor)
#define TEST_S3_LABEL (g_sampling.label)
#define TEST_S3_CONTROL_SEEN (g_sampling.control_seen)
#define TEST_S3_IR_READY (g_shared.ir_ready)
#define TEST_S3_CONTROL_READY (g_shared.control_ready)
#else
#define TEST_S3_RESET_WAITING (g_sampling.b_waits_for_reset)
#define TEST_S3_RESET_REMOTE (g_sampling.b_is_remote_reset)
#define TEST_S3_COLLECTING (g_sampling.b_is_collecting)
#define TEST_S3_SENSOR (g_sampling.sensor)
#define TEST_S3_LABEL (g_sampling.label)
#define TEST_S3_CONTROL_SEEN (g_sampling.b_has_control_tick)
#define TEST_S3_IR_READY (g_shared.b_has_ir_debug)
#define TEST_S3_CONTROL_READY (g_shared.b_has_control_debug)
#endif
#else
#define TEST_S3_RESET_WAITING (g_reset_waiting)
#define TEST_S3_RESET_REMOTE (g_reset_remote)
#define TEST_S3_COLLECTING (g_collecting)
#define TEST_S3_SENSOR (g_sensor)
#define TEST_S3_LABEL (g_label)
#define TEST_S3_CONTROL_SEEN (g_control_seen)
#define TEST_S3_IR_READY (g_ir_ready)
#define TEST_S3_CONTROL_READY (g_control_ready)
#endif

#if defined(TEST_S3_COMPACT_STORAGE) && TEST_S3_COMPACT_STORAGE &&             \
    (!defined(TEST_S3_LEGACY_PREDICATES) || !TEST_S3_LEGACY_PREDICATES)
#define TEST_S3_SHARED_PENDING (g_shared.b_has_command)
#define TEST_S3_SHARED_STOP (g_shared.b_has_stop)
#define TEST_S3_SHARED_STATUS_READY (g_shared.b_has_status)
#define TEST_S3_SHARED_RESET_PENDING (g_shared.b_has_reset)
#else
#define TEST_S3_SHARED_PENDING (g_shared.pending)
#define TEST_S3_SHARED_STOP (g_shared.stop)
#define TEST_S3_SHARED_STATUS_READY (g_shared.status_ready)
#define TEST_S3_SHARED_RESET_PENDING (g_shared.reset_pending)
#endif

#endif

/* End of file. */

