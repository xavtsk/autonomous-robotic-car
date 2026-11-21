/** @file avoid_obstacle.h
 * @brief Advance obstacle avoidance and line hand-back through motion
 * proposals.
 */
#ifndef BUDDY5_OBSTACLE_AVOIDANCE_H
#define BUDDY5_OBSTACLE_AVOIDANCE_H
#include "../app_types.h"
#include "avoidance_planner.h"
#include "line_recovery.h"
#include "s5_codec.h"

typedef enum
{
    S5_IDLE,
    S5_WAIT_STOP,
    S5_SCANNING,
    S5_MOVING,
    S5_FINISH,
    S5_COMPLETE,
    S5_FAULT
} avoid_obstacle_state_t;

typedef enum
{
    S5_OK,
    S5_BAD_RANGE,
    S5_NO_PATH,
    S5_SCAN_FAILED,
    S5_DEADLINE,
    S5_NO_LINE,
    S5_MOTION_ERROR,
    S5_INTERRUPTED
} avoid_obstacle_fault_t;

/* S5_ACT_PEEK: value = servo angle for one side reading while stopped. */
typedef enum
{
    S5_ACT_NONE,
    S5_ACT_STOP,
    S5_ACT_SCAN,
    S5_ACT_DRIVE,
    S5_ACT_TURN,
    S5_ACT_RESUME,
    S5_ACT_PEEK
} avoid_obstacle_action_kind_t;

/* Leg 3 (passing the obstacle) sub-steps: drive to the front face, then
 * peek / step until the side is clear, then clear the car body. */
typedef enum
{
    S5_PASS_APPROACH,
    S5_PASS_PEEK,
    S5_PASS_PEEKING,
    S5_PASS_STEP,
    S5_PASS_EXTRA
} avoid_obstacle_pass_t;

typedef struct
{
    avoid_obstacle_action_kind_t kind;
    int16_t                      value;
    uint16_t                     speed_mms;
} avoid_obstacle_action_t;

typedef struct
{
    uint32_t  now_us;
    float64_t centre_m;
    bool      b_is_stopped;
    bool      b_is_motion_done;
    bool      b_is_healthy;
    bool      b_has_tape;
    bool      b_has_junction;
    bool      b_is_line_ready;
    int8_t    turn_pct; /* The pre-stop steering average ranges from -100 to
                         * 100 percent. Positive means left.
                         */
    s5_codec_observation_t observation;
} avoid_obstacle_input_t;

typedef struct
{
    s5_config_t              config;
    avoidance_planner_plan_t plan;
    line_recovery_t          recovery;
    unsigned                 state : 3;
    unsigned                 fault : 3;
    unsigned                 leg : 4;
    unsigned                 b_has_issued : 1;
    unsigned                 pass : 3;
    unsigned                 b_has_swapped : 1;
    unsigned                 b_has_backed : 1;

    unsigned b_is_backing : 1; /* Leg zero is reversing before the turn. */
    unsigned b_has_aimed : 1;
    unsigned b_is_aiming : 1;  /* Wait for the aiming turn to settle before
                                  scanning. */
    unsigned b_is_on_bend : 1; /* Latch the bend estimate at the first scan. */
    unsigned b_is_after_tape : 1;
    unsigned b_has_extended : 1; /* Track whether the line was seen during
                                  * axle placement or the extra return drive.
                                  */
    unsigned rel_bits : 8;       /* Two's-complement heading; use
                                    avoid_obstacle_relative_get/set. */
    unsigned side_turn : 4;
    unsigned side_turns : 2;          /* Count the turns away on leg 3. */
    unsigned b_was_toward_bypass : 1; /* Last centring swerve direction. */
    unsigned centre_tries : 2;        /* Number of centring attempts. */
    unsigned blocked_reads : 2; /* Count blocked readings in a row on leg 1. */
    unsigned aim_bits : 7;      /* Two's-complement aim; use
                                   avoid_obstacle_aim_get/set. */
    uint16_t searched_mm; /* Measure travel beside the obstacle on leg 3. */
    unsigned lat_mm : 9;  /* Saturate sideways leg 3 travel after turning
                           * away at 511 mm.
                           */
    unsigned peek_lat_mm : 12; /* Store the distance from the original line
                                * in millimetres at the last leg 3 side peek.
                                */
    uint32_t  phase_us;
    uint32_t  started_us;
    float64_t leg_start_m;
} avoid_obstacle_t;

/** Read the relative heading in degrees, positive toward the bypass side. */
static inline int
avoid_obstacle_relative_get (const avoid_obstacle_t * p_avoidance)
{
    /* Each masked quantity fits int; subtraction sign-extends without a
     * signed bit-field or a bitwise operation on signed data. */
    return (int)(p_avoidance->rel_bits & 0x7fU) -
           (int)(p_avoidance->rel_bits & 0x80U);
}

/** Store the relative heading using the existing eight-bit representation. */
static inline void avoid_obstacle_relative_set (avoid_obstacle_t * p_avoidance,
                                                int                degrees)
{
    /* Conversion to unsigned is modulo 2^N, so the low eight bits preserve
     * the previous target representation, including negative angles. */
    p_avoidance->rel_bits = (unsigned int)degrees & 0xffU;
}

/** Read the aimed turn in degrees; the planner uses -40 through +40. */
static inline int avoid_obstacle_aim_get (const avoid_obstacle_t * p_avoidance)
{
    /* Both masks fit int; subtraction restores the signed seven-bit value. */
    return (int)(p_avoidance->aim_bits & 0x3fU) -
           (int)(p_avoidance->aim_bits & 0x40U);
}

/** Store the aimed turn without a signed bit-field. Zero remains zero. */
static inline void avoid_obstacle_aim_set (avoid_obstacle_t * p_avoidance,
                                           int                degrees)
{
    /* The low seven bits retain the existing signed target representation. */
    p_avoidance->aim_bits = (unsigned int)degrees & 0x7fU;
}

/** @brief Clear an avoidance controller and copy its configuration.
 * @param p_detour Nonnull writable state. The controller task serializes
 * updates.
 * @param p_config Nonnull configuration to copy, including speed in mm/s
 * and chassis dimensions in mm. The caller supplies validated values.
 */
void avoid_obstacle_init (avoid_obstacle_t *  p_detour,
                          const s5_config_t * p_config);

/** @brief Check whether the current ultrasonic range can be used.
 * @param p_range_observation Observation to check, or null.
 * @param now Current microsecond clock, using the observation's time base.
 * @return True for a valid echo from 30 through 4000 mm whose unsigned age
 * is at most S5_RANGE_FRESH_US.
 */
bool avoid_obstacle_range_ready (
    const s5_codec_observation_t * p_range_observation, uint32_t now);

/** @brief Start a new bypass by waiting for the vehicle to stop.
 * @param p_detour Nonnull initialized state. Resets the plan, recovery and
 * fault.
 * @param now Start of the run and first phase in microseconds.
 */
void avoid_obstacle_begin (avoid_obstacle_t * p_detour, uint32_t now);

/** @brief Set the controller idle and reset its line-recovery state.
 * @param p_detour Nonnull writable state. Clears the outstanding-action flag.
 */
void avoid_obstacle_abort (avoid_obstacle_t * p_detour);

/** @brief Advance one controller update and return its motion proposal.
 * @param p_detour Initialized, caller-owned state, updated in place.
 * @param p_in Current observations, including time in microseconds and
 * centre travel in metres. The caller passes a consistent snapshot.
 * @return One action. A missing argument requests STOP. When active, motion
 * or scan faults and expired deadlines can latch a fault and request STOP.
 */
avoid_obstacle_action_t
avoid_obstacle_step (avoid_obstacle_t *             p_detour,
                     const avoid_obstacle_input_t * p_in);
#ifdef S5_HOST_TEST
/** @brief Read extra clearance in millimetres from a valid test plan. */
unsigned pass_extra_test (const avoid_obstacle_t * p_detour);
#endif
/** @brief Calculate a straight-track aim from the axle to the obstacle centre.
 * @param p_scan_profile Scanned obstacle profile with edges and front range in
 * mm.
 * @return Degrees, positive left and limited to S5_AIM_MAX_DEG in magnitude.
 * Null, invalid, over-wide or below-threshold profiles produce zero.
 */
int avoid_obstacle_aim_deg (const profile_obstacle_t * p_scan_profile);

/** @brief Calculate the aiming turn using the last line-following command.
 * @param p_scan_profile Scanned profile or null, with the same rejection as
 * avoid_obstacle_aim_deg.
 * @param turn_pct Last commanded turn in percent, positive left.
 * @param pb_is_bend Nonnull output, set from turn_pct even if the scan is
 * invalid.
 * @return Signed degrees. A bend toward the detected bearing uses the bend
 * gain and cap. A bend in the opposite direction uses the straight cap.
 */
int avoid_obstacle_aim_deg_bend (const profile_obstacle_t * p_scan_profile,
                                 int turn_pct, bool * pb_is_bend);
#endif

/* End of file. */

