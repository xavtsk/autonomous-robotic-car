/** @file s3_integration.h
 * @brief Expose bounded controller command submission and copied runtime
 * status.
 */
#ifndef S3_INTEGRATION_H
#define S3_INTEGRATION_H
/* Sole controller runtime for the explicit Buddies 1-3 integration profile.
 * Future integration must replace/extend this owner, not run alongside it. */
#include "s3_drive.h"
#include "run_timing.h"

typedef enum
{
    S3_CONTROL_WHITE,
    S3_CONTROL_BLACK,
    S3_CONTROL_APPLY_CALIBRATION,
    S3_CONTROL_MAPPING,
    S3_CONTROL_START,
    S3_CONTROL_STOP,
    S3_CONTROL_RESET
} s3_integration_kind_t;

typedef struct
{
    s3_integration_kind_t      kind;
    ir_acquisition_sensor_id_t sensor;
    line_navigation_action_t   map[S3_BARCODE_LETTERS];
} s3_integration_request_t;

typedef struct
{
    line_navigation_output_t navigation;
    motion_feedback_t        motion;
    s3_drive_state_t         state;
    s3_drive_reason_t        reason;
    uint32_t                 samples;
    uint32_t                 max_gap_us;
    uint32_t                 above_target;
    uint32_t                 command_sequence;
    run_timing_t run_timing; /* Reset on START and freeze on STOP or halt. */
    bool         b_did_command_succeed;
    bool         b_is_collecting;

    ir_acquisition_sensor_id_t calibration_sensor;
    ir_calibration_summary_t   white;
    ir_calibration_summary_t   black;
    int32_t                    adc_status;
} s3_integration_status_t;

/* Thread-safe, bounded single pending command. STOP takes precedence and
 * cancels any queued command. Return false on a full queue/disabled profile.
 * Acceptance into the queue is not successful execution: inspect status. */
bool    s3_integration_submit (const s3_integration_request_t * p_request);
bool    s3_integration_status (s3_integration_status_t * p_status);
int32_t s3_integration_init (void);
int32_t s3_integration_start (void);
#endif

/* End of file. */

