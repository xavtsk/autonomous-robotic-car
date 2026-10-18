#ifndef S3_DRIVE_H
#define S3_DRIVE_H
#include "line_navigation.h"
#include "../subsystem2/motion.h"
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
    S3_DRIVE_BAD_INPUT
} s3_drive_reason_t;
/* Only this adapter owns motion requests/poll in the integrated profile. */
typedef struct
{
    motion_result_t (*velocity)(float left, float right);
    motion_result_t (*manoeuvre)(manoeuvre_t action);
    void (*stop)(void);
    motion_state_t (*poll)(void);
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
    uint32_t phase_us, pending_us;
    bool configured;
} s3_drive_t;
bool s3_drive_init(s3_drive_t *drive, const s3_navigation_config_t *config,
                   const s3_drive_port_t *port);
bool s3_drive_start(s3_drive_t *drive, uint32_t now);
void s3_drive_stop(s3_drive_t *drive);
bool s3_drive_reset(s3_drive_t *drive, uint32_t now);
bool s3_drive_process(s3_drive_t *drive, const s3_sensor_frame_t *frame,
                      const motion_feedback_t *feedback);
#endif
