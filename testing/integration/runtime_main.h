/** @file runtime_main.h
 * @brief Kernel callbacks for the controller sampling fixture.
 *
 * The sampling, command and diagnostic tests share the fake task and mutex
 * declarations. Their runtime state belongs to the fixture.
 */
#ifndef TESTING_INTEGRATION_RUNTIME_MAIN_H
#define TESTING_INTEGRATION_RUNTIME_MAIN_H

#if defined(S3_RUNTIME_TEST)
#ifndef APP_H
#define APP_H
#endif
#define APP_S3_MOTION 1
#define APP_DIAGNOSTICS 0
#define APP_TEST 0
/* Begin uncalibrated with the original line and junction profile. */
#define S3_BUILTIN_CALIBRATION 0
#define S3_LINE_BETWEEN 0
#define S3_CROSS_STRAIGHT 0
#define S3_COURSE_PENDING_WAIT_US (5000000U)
#endif

#include "../../app_program/app_types.h"
#include "ctrl_test_kernel.h"

/* The comparison harness selects the fixed baseline's former member name. */
#ifndef TEST_REMOTE_ID_MEMBER
#define TEST_REMOTE_ID_MEMBER request_id
#endif

/* Controller sampling fixtures use the same cyclic-handler descriptor. */
typedef struct
{
    unsigned cycatr;
    FP       cychdr;
    unsigned cyctim;
    unsigned cycphs;
} T_CCYC;

/** @brief Record the requested cycle and return the simulated owner handle. */
ID tk_cre_cyc (const T_CCYC * p_cycle);

#if defined(S3_RUNTIME_TEST)
#include "s3_integration.h"
#include "remote_control.h"
#include "layout_access.h"
#include <string.h>
#if APP_IR_DEBUG
#include "control_debug.h"
#endif
#define TA_HLNG 2U
#define TSK_SELF 0

/** @brief Snapshot produced only by the single-threaded host fixture. */
typedef struct
{
    uint32_t epoch;
    uint32_t last_id;
    uint32_t stop_sequence;
    uint32_t pending_id;
    uint32_t pending_epoch;
    unsigned head;
    unsigned count;
    unsigned pending_action;
    bool     b_has_command;
    bool     b_has_stop;
    bool     b_has_status;
    bool     b_has_reset;
    bool     b_waits_for_reset;
    bool     b_is_remote_reset;
    bool     b_has_pending_request;
    uint8_t  reply[REMOTE_REPLY_RING_SIZE][REMOTE_REPLY_SIZE];
#if APP_IR_DEBUG
    bool    b_has_ir_debug;
    bool    b_has_control_tick;
    bool    b_has_control_debug;
    uint8_t control_packet[CONTROL_DEBUG_PACKET_SIZE];
#endif
} runtime_main_private_view_t;

const s3_drive_t *          s3_integration_runtime_test_drive (void);
void                        s3_integration_runtime_test_cycle (void);
bool                        s3_integration_runtime_test_capture_pending (void);
runtime_main_private_view_t s3_integration_runtime_test_view (void);
void s3_integration_runtime_test_seed_status (uint32_t timestamp_us);
void s3_integration_runtime_test_reply (
    const remote_control_request_t * p_request, uint8_t result, uint8_t reason);
bool s3_integration_runtime_test_start (uint32_t now_us);
bool s3_integration_runtime_test_calibration (
    ir_acquisition_sensor_id_t         sensor,
    const ir_calibration_collector_t * p_values);
#endif

#if defined(TEST_SLOPE_LINK)
#include "../../app_program/controller/s3_integration.h"

/** @brief Read-only observations of the simulated task and its kernel fakes. */
typedef struct
{
    const s3_drive_t * p_drive;
    uint32_t           clock_us;
    unsigned           lock_depth;
    unsigned           stops;
    unsigned           velocity_requests;
    unsigned           freeze_calls;
    unsigned           cancel_calls;
    uint8_t            command_id;
} runtime_main_view_t;

/** @brief Read counters and borrow the drive until the next simulated cycle. */
runtime_main_view_t runtime_main_view (void);

/** @brief Advance the fake clock and sample the controller once. */
void runtime_main_cycle (void);

/** @brief Set the raw ADC count for subsequent reads. */
void runtime_main_set_adc (uint16_t counts);

/** @brief Return whether the selected sensor accepted the calibration. */
bool runtime_main_apply_calibration (
    ir_acquisition_sensor_id_t         sensor,
    const ir_calibration_collector_t * p_values);

/** @brief Return whether START succeeds at the current fixture time. */
bool runtime_main_start_now (void);
#endif

#endif /* TESTING_INTEGRATION_RUNTIME_MAIN_H */

/* End of file. */

