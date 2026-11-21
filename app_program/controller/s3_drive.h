/** @file s3_drive.h
 * @brief Declare the drive owner state, motion boundary and course behaviour
 * interface.
 */
#ifndef S3_DRIVE_H
#define S3_DRIVE_H
#include "../app_types.h"
#include "../subsystem2/motion.h"
#include "../subsystem3/line_navigation.h"
#include "../subsystem5/s5_config.h"
#if APP_S5
#include "../subsystem5/avoid_obstacle.h"
#endif
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
    S3_DRIVE_BAD_INPUT,
    S3_DRIVE_TERRAIN_FAULT
} s3_drive_reason_t;

typedef enum
{
    S3_RESET_IDLE,
    S3_RESET_PENDING,
    S3_RESET_OK,
    S3_RESET_FAILED
} s3_drive_reset_result_t;

/* Less than the operator's 1 s reply deadline; no blocking wait in the owner.
 */
#define S3_RESET_WAIT_US (500000U)

/* Motion callbacks used by the drive controller. The integrated controller
 * task owns both requests and report consumption. */
typedef struct
{
    motion_result_t ( * p_velocity)(float32_t left, float32_t right);
    motion_result_t ( * p_manoeuvre)(motion_manoeuvre_t action);
    void            ( * p_stop)(void);
    bool            ( * p_report)(motion_report_t * p_out);
    uint8_t         ( * p_command_id)(void);
    bool            ( * p_reset)(void);
    void            ( * p_heartbeat)(bool b_is_enabled);
} s3_drive_port_t;

/* Course behaviour. s3_drive_init() installs the defaults below, which keep
 * the original Buddy 3 behaviour exactly; the integration profile overrides
 * them with s3_drive_set_options(). */
typedef struct
{
    /* Junction with no pending action, or a pending STRAIGHT: keep driving
     * straight across instead of halting (default false = halt). */
    bool b_allows_cross_straight;
    /* LEFT/RIGHT only: after braking at a junction, drive this far measured
     * from where the junction was detected, so the pivot (axle centre) is
     * over the crossing before the on-the-spot turn. Set to the sensor-to-axle
     * distance + ~5 mm. 0 = turn where the car stopped (original). */
    float32_t turn_advance_m;
    float32_t advance_mps;
    /* Drive a distance (Buddy 2 motion_request_drive); required only when
     * turn_advance_m > 0. */
    motion_result_t ( * p_drive)(float32_t meters, float32_t speed_mps);

    uint32_t pending_wait_us; /* Allow 5 s to reach a junction by default. */
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
    float32_t search_deg;
    float32_t search_mps; /* Set each wheel's sweep speed in m/s. */
    float32_t track_m;    /* Use axle track in metres to time turns. */
} s3_drive_options_t;

s3_drive_options_t s3_drive_default_options (void);

typedef struct
{
    /* The sampling owner updates these flags and the reset result together.
     * External hold permits sampling during the run-reset handshake. */
    unsigned b_has_external_hold : 1;
    unsigned b_is_configured : 1;
    unsigned b_waits_for_stop : 1;
    unsigned b_is_advancing : 1;
    unsigned b_has_touched_line : 1;
    unsigned b_is_searching : 1;
    unsigned reset_result : 2;
#if APP_S5
    unsigned b_has_obstacle_snapshot : 1;
    unsigned b_is_bypassing : 1;
#endif
    line_navigation_t        navigation;
    line_navigation_output_t output;
    s3_drive_port_t          port;
    motion_feedback_t        feedback;
    s3_drive_state_t         state;
    s3_drive_reason_t        reason;
    line_navigation_action_t pending;
    uint32_t                 phase_us;
    uint32_t                 pending_us;
    uint32_t                 reset_us;
    uint32_t                 report_loss_count;
    uint8_t                  command_id;
    uint8_t                  cancel_id;
    s3_drive_options_t       options;

    float64_t junction_m; /* Store odometry at junction detection. */
    uint32_t  touch_us;   /* Store the last front detection in microseconds. */
    line_navigation_action_t turn; /* Record the current manoeuvre. */
    int8_t   search_sign;          /* Use +1 anticlockwise or -1 clockwise. */
    uint8_t  search_leg;           /* Zero sweeps out and one sweeps back. */
    uint32_t search_us;            /* Store the sweep start in microseconds. */
#if APP_S5
    avoid_obstacle_t obstacle;
    /* Owner-local packed cache; never refresh source times on a BUSY read. */
    uint8_t  obstacle_snapshot[S5_PACKET_BYTES];
    uint32_t obstacle_snapshot_us;
    uint16_t obstacle_scan_id;
    /* Bend detection for Buddy 5: (right-left)/(right+left) of the line
     * follower's wheel speeds, averaged over ~0.5 s; latched at detection.
     * Minimum bits (Change Requirement 1): the ratio is -1..+1.
     * turn_q: filter state, 1/10000 steps, -10000..10000 -> 15 bits + sign
     *         (internal precision so the slow average does not stall);
     * obstacle_turn_pct: shared result, 1/100 steps, -100..100 -> int8_t. */
    int16_t  turn_q;
    int8_t   obstacle_turn_pct;
    uint32_t turn_us;
#endif
} s3_drive_t;

/* External sensor fault, called only by the same controller owner. */
void s3_drive_terrain_fault (s3_drive_t * p_drive);
bool s3_drive_init (s3_drive_t *                     p_drive,
                    const line_navigation_config_t * p_config,
                    const s3_drive_port_t *          p_port);
/* Only while OFF/HALTED/FAULT. Rejects a nonzero advance without a drive
 * port and invalid numbers; the previous options stay in force. */
bool s3_drive_set_options (s3_drive_t *               p_drive,
                           const s3_drive_options_t * p_options);
bool s3_drive_start (s3_drive_t * p_drive, uint32_t now);
void s3_drive_stop (s3_drive_t * p_drive);
/* True means accepted, not completed. Completion requires a NEW braked,
 * healthy motion snapshot and healthy navigation in a later process cycle.
 * It never starts motion; STOP cancels a pending reset. */
bool                    s3_drive_reset (s3_drive_t * p_drive, uint32_t now);
s3_drive_reset_result_t s3_drive_reset_result (const s3_drive_t * p_drive);

bool s3_drive_process (s3_drive_t *                   p_drive,
                       const ir_acquisition_frame_t * p_frame,
                       const motion_feedback_t *      p_feedback);
#endif

/* End of file. */

