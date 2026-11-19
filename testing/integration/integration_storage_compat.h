/** @file integration_storage_compat.h
 * @brief Preserve host observations across the two reviewed private layouts.
 */
#ifndef TESTING_INTEGRATION_STORAGE_COMPAT_H
#define TESTING_INTEGRATION_STORAGE_COMPAT_H

/* Include only after the subject implementation. Each accessor preserves the
 * original private observation's value and lvalue behavior. Inverse identifier
 * substitution recovers the original fixture assertions and observer tokens.
 * The host build selects the reviewed layout through exact source detection. */
#if defined(TEST_S3_COMPACT_STORAGE) && TEST_S3_COMPACT_STORAGE
#define TEST_S3_RESET_WAITING (g_sampling.reset_waiting)
#define TEST_S3_RESET_REMOTE (g_sampling.reset_remote)
#define TEST_S3_COLLECTING (g_sampling.collecting)
#define TEST_S3_SENSOR (g_sampling.sensor)
#define TEST_S3_LABEL (g_sampling.label)
#define TEST_S3_CONTROL_SEEN (g_sampling.control_seen)
#define TEST_S3_IR_READY (g_shared.ir_ready)
#define TEST_S3_CONTROL_READY (g_shared.control_ready)
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

#endif

/* End of file. */

