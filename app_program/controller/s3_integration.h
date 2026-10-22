#ifndef S3_INTEGRATION_H
#define S3_INTEGRATION_H
/* Temporary controller runtime. A future controller must replace this sole
 * motion/report owner, not run alongside it. */
#include "s3_drive.h"
typedef enum
{
    S3_CONTROL_WHITE,
    S3_CONTROL_BLACK,
    S3_CONTROL_APPLY_CALIBRATION,
    S3_CONTROL_MAPPING,
    S3_CONTROL_START,
    S3_CONTROL_STOP,
    S3_CONTROL_RESET
} s3_control_kind_t;
typedef struct
{
    s3_control_kind_t kind;
    s3_sensor_id_t sensor;
    s3_nav_action_t map[26];
} s3_control_request_t;
typedef struct
{
    s3_navigation_output_t navigation;
    motion_feedback_t motion;
    s3_drive_state_t state;
    s3_drive_reason_t reason;
    uint32_t samples, max_gap_us, over_5ms, command_sequence;
    bool command_ok, collecting;
    s3_sensor_id_t calibration_sensor;
    s3_calibration_summary_t white, black;
    int32_t adc_status;
} s3_integration_status_t;
/* Thread-safe, bounded single pending command. STOP takes precedence and
 * cancels any queued command. Return false on a full queue/disabled profile.
 * Acceptance into the queue is not successful execution: inspect status. */
bool s3_integration_submit(const s3_control_request_t *request);
bool s3_integration_status(s3_integration_status_t *status);
int32_t s3_integration_init(void);
int32_t s3_integration_start(void);
#endif
