/*
 * File: motion.h
 * Owner: Buddy 2 - Motion Control
 * Purpose: Public motion subsystem interface.
 *
 * Implements the Motion Control statechart (Week 6 Design Review, 6.1):
 *
 *   IDLE --velocity request--> VELOCITY CONTROL
 *   IDLE --distance request--> DRIVE --target reached--> COMMAND COMPLETE
 *   IDLE --turn request------> TURN  --target reached--> COMMAND COMPLETE
 *   COMMAND COMPLETE --completion reported (motion_poll)--> IDLE
 *   any active state --stop()--> IDLE
 *   any active state --feedback fault / stall / stale command--> MOTION FAULT
 *   MOTION FAULT --fault cleared AND motion_reset()--> IDLE
 *
 * Requests return immediately; the motion task (subsystem2.c) runs the motion
 * every control period (10 ms). Requests and status calls are serialised by a
 * mutex, so they are safe from any task on either core. Per the design, only
 * the Vehicle Controller sends requests; anyone may read the estimates.
 *
 * Units: metres, metres per second, degrees. Speeds are wheel speeds.
 */

#ifndef BUDDY2_MOTION_H
#define BUDDY2_MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_IDLE = 0,    /* safe outputs (motors braked), ready for a request  */
    MOTION_VELOCITY,    /* following continuous wheel-speed targets           */
    MOTION_DRIVE,       /* driving straight to a distance target              */
    MOTION_TURN,        /* turning on the spot to an angle target             */
    MOTION_COMPLETE,    /* drive/turn reached its target; reported once       */
    MOTION_FAULT        /* safe outputs, latched until motion_reset()         */
} motion_state_t;

typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_STALL,          /* driven wheel made no encoder progress     */
    MOTION_FAULT_STALE_COMMAND,  /* VELOCITY: no new target within 250 ms     */
    MOTION_FAULT_ENCODER,        /* encoder signal invalid / noisy            */
    MOTION_FAULT_DIRECTION,      /* a wheel turned against its command        */
    MOTION_FAULT_TIMEOUT         /* drive/turn took far longer than expected  */
} motion_fault_t;

typedef enum
{
    MOTION_OK = 0,               /* request accepted                          */
    MOTION_ERR_BUSY,             /* not IDLE (velocity: not IDLE or VELOCITY) */
    MOTION_ERR_FAULT,            /* in MOTION_FAULT: call motion_reset()      */
    MOTION_ERR_BAD_ARG,          /* negative / zero speed, or not a number    */
    MOTION_ERR_NOT_CALIBRATED    /* set the CALIBRATE values in motion.c      */
} motion_result_t;

typedef enum
{
    MANOEUVRE_LEFT = 0,          /* TURN 90 deg left on the spot              */
    MANOEUVRE_RIGHT,             /* TURN 90 deg right on the spot             */
    MANOEUVRE_STRAIGHT,          /* DRIVE straight MANOEUVRE_STRAIGHT_M       */
    MANOEUVRE_U_TURN             /* TURN 180 deg left on the spot             */
} manoeuvre_t;

/* Values measured by calibration; each field goes into the #define of the
 * same name (without b_) in the CALIBRATE block of motion.c */
typedef struct
{
    bool  b_valid;              /* all checks below passed                   */
    float counts_per_rev;       /* ENCODER_COUNTS_PER_REV                    */
    float max_speed_mps;        /* MOTOR_MAX_SPEED_MPS (the slower motor)    */
    bool  b_m1_inverted;        /* M1_INVERTED                               */
    bool  b_m2_inverted;        /* M2_INVERTED                               */
    bool  b_enc_l_inverted;     /* ENC_L_INVERTED                            */
    bool  b_enc_r_inverted;     /* ENC_R_INVERTED                            */
    float counts_per_rev_l;     /* per wheel, for checking                   */
    float counts_per_rev_r;
    float top_speed_l_mps;      /* per wheel, wheels off the ground          */
    float top_speed_r_mps;
    bool  b_enc_l_ok;           /* false: ENC_L did not count                */
    bool  b_enc_r_ok;           /* false: ENC_R did not count                */
    bool  b_encoders_swapped;   /* true: ENC_L / ENC_R cables swapped        */
} motion_calibration_t;

/* ---- Lifecycle (Buddy 2 internal, called by subsystem2.c only) ---------- */
int32_t motion_create(void);                 /* kernel objects; _init()      */
void    motion_attach_task(int32_t task_id); /* motion task, for stop()      */
void    motion_init(void);                   /* hardware; in the motion task */
void    motion_process(void);                /* every period, in the task    */

/* ---- Brief API (non-blocking) --------------------------------------------
 * Named as in the brief: moveForward(distance), moveBackward(distance),
 * turnLeft(angle), turnRight(angle), stop(). The moves run at the default
 * speeds in motion.c and behave like the DRIVE / TURN requests below:
 * accepted in IDLE only, finish with MOTION_COMPLETE.
 * distance in metres, angle in degrees, both >= 0. */
motion_result_t moveForward(float distance);
motion_result_t moveBackward(float distance);
motion_result_t turnLeft(float angle);      /* on the spot, anticlockwise */
motion_result_t turnRight(float angle);     /* on the spot, clockwise     */

/* Stop / cancel: any active state -> IDLE, both motors braked within one
 * control period. No effect in MOTION_FAULT (use motion_reset()). */
void stop(void);

/* ---- Requests (non-blocking) --------------------------------------------- */

/* VELOCITY CONTROL. Accepted in IDLE or VELOCITY; each call replaces the
 * targets. Re-send at least every 250 ms or the robot goes to MOTION_FAULT
 * (stale command). Negative = backward. Speeds above the limit are capped. */
motion_result_t motion_request_velocity(float left_mps, float right_mps);

/* DRIVE straight. Negative distance = backward. IDLE only. */
motion_result_t motion_request_drive(float meters, float speed_mps);

/* TURN on the spot. Positive = left (anticlockwise), negative = right.
 * speed_mps is how fast each wheel moves while turning. IDLE only. */
motion_result_t motion_request_turn(float degrees, float speed_mps);

/* TURN 180 degrees to the left */
motion_result_t motion_request_u_turn(float speed_mps);

/* Junction manoeuvre with fixed settings, so every Left (Right, Straight,
 * U-turn) is the same. IDLE only; finishes with MOTION_COMPLETE. */
motion_result_t motion_request_manoeuvre(manoeuvre_t m);

/* Leave MOTION_FAULT. True once the robot is IDLE (wheels have been still
 * for a moment); false if the fault has not cleared yet - try again later. */
bool motion_reset(void);

/* ---- Status ----------------------------------------------------------- */

/* Current state. MOTION_COMPLETE is returned exactly once (the completion
 * report); the state then moves to MOTION_IDLE. Call it regularly. */
motion_state_t motion_poll(void);

motion_fault_t motion_get_fault(void);
const char    *motion_state_name(motion_state_t s);
const char    *motion_fault_name(motion_fault_t f);
const char    *manoeuvre_name(manoeuvre_t m);

/* ---- Estimates from encoder feedback (safe from any task) -------------- */
int32_t enc_l_count(void);              /* left wheel encoder (ENC_L) count  */
int32_t enc_r_count(void);              /* right wheel encoder (ENC_R) count */
float   wheel_distance_left_m(void);    /* since start, + = forward          */
float   wheel_distance_right_m(void);
float   wheel_speed_left_mps(void);     /* + = forward                       */
float   wheel_speed_right_mps(void);
float   robot_distance_m(void);         /* centre of the robot               */
float   robot_speed_mps(void);
float   robot_heading_deg(void);        /* since start, + = anticlockwise    */

/* ---- Calibration (one-off setup, outside the statechart) ----------------
 * motion_request_calibration() is accepted in IDLE; the motion task then:
 *   1. Waits for EACH wheel to be turned by hand exactly 10 full turns in the
 *      direction that drives the robot forward, then both left still for 3 s.
 *      If an encoder has not counted within 2 minutes, it gives up.
 *   2. 5 s later runs each motor alone at full power for about 2 s, left then
 *      right - lift the wheels off the ground before then.
 * Moves are rejected (MOTION_ERR_BUSY) while it runs. No output is produced:
 * read the results with motion_get_calibration(). */
bool            motion_is_calibrated(void);   /* false until values are set */
motion_result_t motion_request_calibration(void);
bool            motion_is_calibrating(void);
/* true once calibration has finished */
bool            motion_get_calibration(motion_calibration_t *p_out);

#endif /* BUDDY2_MOTION_H */
