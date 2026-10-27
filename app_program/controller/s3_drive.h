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
/* Course behaviour. s3_drive_init() installs the defaults below, which keep
 * the original Buddy 3 behaviour exactly; the integration profile overrides
 * them with s3_drive_set_options(). */
typedef struct
{
    /* Junction with no pending action, or a pending STRAIGHT: keep driving
     * straight across instead of halting (default false = halt). */
    bool cross_straight;
    /* LEFT/RIGHT only: after braking at a junction, drive this far measured
     * from where the junction was detected, so the pivot (axle centre) is
     * over the crossing before the on-the-spot turn. Set to the sensor-to-axle
     * distance + ~5 mm. 0 = turn where the car stopped (original). */
    float turn_advance_m;
    float advance_mps;
    /* Drive a distance (Buddy 2 motion_request_drive); required only when
     * turn_advance_m > 0. */
    motion_result_t (*drive)(float meters, float speed_mps);
    uint32_t pending_wait_us; /* barcode -> junction limit (default 5 s) */
    /* After LEFT/RIGHT/U-TURN: a front sensor must have touched the tape at
     * most this long before the turn finished, else the new line was not
     * reached and the run halts with LINE_LOST. 0 = no check (original). */
    uint32_t turn_line_window_us;
    /* If that check fails: sweep slowly on the spot up to +/- search_deg
     * around where the turn ended (first towards the side the line most
     * likely is, then back across) and resume following as soon as a front
     * sensor touches the tape; not found -> halt LINE_LOST. Keep it below
     * 45 deg so the sweep cannot reach a neighbouring arm of the crossing.
     * 0 = no search (halt at once). Needs turn_line_window_us. */
    float search_deg;
    float search_mps; /* each wheel's speed while sweeping */
    float track_m;    /* wheel centre-to-centre: converts angle to time */
} s3_drive_options_t;
s3_drive_options_t s3_drive_default_options(void);
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
    s3_drive_options_t options;
    double junction_m;        /* odometry when the junction was detected */
    uint32_t touch_us;        /* last time a front sensor touched the tape */
    s3_nav_action_t turn;     /* manoeuvre in progress */
    bool advancing, touched;  /* advance leg running; touch_us is valid */
    bool searching;           /* sweeping for the new line after a turn */
    int8_t search_sign;       /* +1 anticlockwise, -1 clockwise */
    uint8_t search_leg;       /* 0 = first sweep, 1 = back across */
    uint32_t search_us;       /* start of the current sweep leg */
} s3_drive_t;
bool s3_drive_init(s3_drive_t *drive, const s3_navigation_config_t *config,
                   const s3_drive_port_t *port);
/* Only while OFF/HALTED/FAULT. Rejects a nonzero advance without a drive
 * port and invalid numbers; the previous options stay in force. */
bool s3_drive_set_options(s3_drive_t *drive, const s3_drive_options_t *options);
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
