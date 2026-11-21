/** @file operator_main.h
 * @brief Kernel callbacks for the operator control fixture.
 *
 * The fixture drives button inputs and the remote mailbox. The generated
 * operator trace uses the same task and mutex declarations.
 */
#ifndef TESTING_INTEGRATION_OPERATOR_MAIN_H
#define TESTING_INTEGRATION_OPERATOR_MAIN_H

#if defined(S3_OPERATOR_TEST)
#define APP_H
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
/* Start uncalibrated with the original line and junction profile. */
#define S3_BUILTIN_CALIBRATION 0
#define S3_LINE_BETWEEN 0
#define S3_CROSS_STRAIGHT 0
#define S3_COURSE_PENDING_WAIT_US (5000000U)
#endif

#include "runtime_main.h"

#if defined(S3_OPERATOR_TEST)
#include "s3_integration.h"
#define TA_HLNG 2U
#define TSK_SELF 0

const s3_drive_t * s3_integration_operator_test_drive (void);
void               s3_integration_operator_test_cycle (void);
void               s3_integration_operator_test_sensor_fault (void);
#endif

#endif /* TESTING_INTEGRATION_OPERATOR_MAIN_H */

/* End of file. */

