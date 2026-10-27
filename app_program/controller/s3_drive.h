#ifndef S3_DRIVE_H
#define S3_DRIVE_H
#include "../subsystem2/motion.h"
#include "../subsystem3/line_navigation.h"
typedef enum
{
    S3_DRIVE_OFF,
    S3_DRIVE_FOLLOW,
    S3_DRIVE_BRAKING,
    S3_DRIVE_MANOEUVRE,
    S3_DRIVE_REACQUIRE,
    S3_DRIVE_HALTED,
    S3_DRIVE_FAULT
} s3_drive_state_t;
typedef enum
{
    S3_DRIVE_OK,
    S3_DRIVE_SENSOR_FAULT,
    S3_DRIVE_MOTION_FAULT,
    S3_DRIVE_REJECTED,
    S3_DRIVE_LINE_LOST,
    S3_DRIVE_NO_DIRECTION,
    S3_DRIVE_UNMAPPED,
    S3_DRIVE_TIMEOUT,
    S3_DRIVE_REPORT_FAULT,
    S3_DRIVE_BAD_INPUT
} s3_drive_reason_t;
typedef enum
{
    S3_RESET_IDLE,
    S3_RESET_PENDING,
    S3_RESET_OK,
    S3_RESET_FAILED
} s3_reset_result_t;
/* Less than the operator's 1 s reply deadline; no blocking wait in the owner. */
#define S3_RESET_WAIT_US (500000U)
/* Temporary controller: sole owner of motion requests AND report consumption.
 * The future Vehicle Controller replaces this owner; the sensor core is
 * unchanged. */
typedef struct
{
    motion_result_t (*velocity)(float left, float right);
    motion_result_t (*manoeuvre)(manoeuvre_t action);
    void (*stop)(void);
    bool (*report)(motion_report_t *out);
    uint8_t (*command_id)(void);
    bool (*reset)(void);
    void (*heartbeat)(bool enabled);
} s3_drive_port_t;
typedef struct
{
    s3_navigation_t navigation;
    s3_navigation_output_t output;
    s3_drive_port_t port;
    motion_feedback_t feedback;
    s3_drive_state_t state;
    s3_drive_reason_t reason;
    s3_nav_action_t pending;
    uint32_t phase_us, pending_us, reset_us;
    uint32_t report_loss_count;
    uint8_t command_id, cancel_id;
    bool configured, waiting_stop;
    unsigned reset_result : 2;
} s3_drive_t;
bool s3_drive_init(s3_drive_t *drive, const s3_navigation_config_t *config,
                   const s3_drive_port_t *port);
bool s3_drive_start(s3_drive_t *drive, uint32_t now);
void s3_drive_stop(s3_drive_t *drive);
/* True means accepted, not completed. Completion requires a NEW braked,
 * healthy motion snapshot and healthy navigation in a later process cycle.
 * It never starts motion; STOP cancels a pending reset. */
bool s3_drive_reset(s3_drive_t *drive, uint32_t now);
s3_reset_result_t s3_drive_reset_result(const s3_drive_t *drive);
bool s3_drive_process(s3_drive_t *drive, const s3_sensor_frame_t *frame,
                      const motion_feedback_t *feedback);
#endif
