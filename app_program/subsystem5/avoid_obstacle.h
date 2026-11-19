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
} s5_state_t;

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
} s5_fault_t;

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
} s5_action_kind_t;

/* Leg 3 (passing the obstacle) sub-steps: drive to the front face, then
 * peek / step until the side is clear, then clear the car body. */
typedef enum
{
    S5_PASS_APPROACH,
    S5_PASS_PEEK,
    S5_PASS_PEEKING,
    S5_PASS_STEP,
    S5_PASS_EXTRA
} s5_pass_t;

typedef struct
{
    s5_action_kind_t kind;
    int16_t value;
    uint16_t speed_mms;
} s5_action_t;

typedef struct
{
    uint32_t now_us;
    float64_t centre_m;
    bool stopped, motion_done, healthy, tape, junction, line_ready;
    int8_t turn_pct; /* line follower's turning just before the stop, %, + =
                        left, -100..100 */
    s5_observation_t observation;
} s5_input_t;

typedef struct
{
    s5_config_t config;
    s5_plan_t plan;
    s5_recovery_t recovery;
    unsigned state : 3, fault : 3, leg : 4, issued : 1, pass : 3, swapped : 1;
    unsigned backed : 1,
        backing : 1; /* leg 0: reverse-before-turn checked / running */
    unsigned aimed : 1,
        aiming : 1;    /* scanning: face-the-obstacle turn done / running */
    unsigned bend : 1; /* stopped on a bend (latched at the first scan) */
    unsigned tail : 1,
        extended : 1; /* line seen, axle-onto-line drive / extra return drive */
    unsigned rel_bits : 8; /* Two's-complement heading; use s5_rel_get/set. */
    unsigned side_turn : 4,
        side_turns : 2; /* leg 3: pending turn away (deg), turns made */
    unsigned centre_dir : 1, centre_tries
        : 2; /* finish: last swerve was toward the bypass side, tries */
    unsigned blocked_reads : 2; /* leg 1: consecutive blocked readings so far */
    unsigned aim_bits : 7;      /* Two's-complement aim; use s5_aim_get/set. */
    uint16_t searched_mm;       /* leg 3 distance stepped beside the obstacle */
    unsigned lat_mm : 9; /* leg 3 sideways travel after turning away, 0..511 mm
                            (saturates) */
    unsigned peek_lat_mm
        : 12; /* leg 3: distance from the original line, last side peek, mm */
    uint32_t phase_us, started_us;
    float64_t leg_start_m;
} s5_avoidance_t;

/** Read the relative heading in degrees, positive toward the bypass side. */
static inline int s5_rel_get (const s5_avoidance_t * p_avoidance)
{
    /* Each masked quantity fits int; subtraction sign-extends without a
     * signed bit-field or a bitwise operation on signed data. */
    return (int)(p_avoidance->rel_bits & 0x7fU) -
           (int)(p_avoidance->rel_bits & 0x80U);
}

/** Store the relative heading using the existing eight-bit representation. */
static inline void s5_rel_set (s5_avoidance_t * p_avoidance, int degrees)
{
    /* Conversion to unsigned is modulo 2^N, so the low eight bits preserve
     * the previous target representation, including negative angles. */
    p_avoidance->rel_bits = (unsigned int)degrees & 0xffU;
}

/** Read the aimed turn in degrees; the planner uses -40 through +40. */
static inline int s5_aim_get (const s5_avoidance_t * p_avoidance)
{
    /* Both masks fit int; subtraction restores the signed seven-bit value. */
    return (int)(p_avoidance->aim_bits & 0x3fU) -
           (int)(p_avoidance->aim_bits & 0x40U);
}

/** Store the aimed turn without a signed bit-field. Zero remains zero. */
static inline void s5_aim_set (s5_avoidance_t * p_avoidance, int degrees)
{
    /* The low seven bits retain the existing signed target representation. */
    p_avoidance->aim_bits = (unsigned int)degrees & 0x7fU;
}

void s5_avoidance_init (s5_avoidance_t * p_a, const s5_config_t * p_config);
bool s5_range_ready (const s5_observation_t * p_o, uint32_t now);
void s5_avoidance_begin (s5_avoidance_t * p_a, uint32_t now);
void s5_avoidance_abort (s5_avoidance_t * p_a);
s5_action_t s5_avoidance_step (s5_avoidance_t * p_a,
                               const s5_input_t * p_input);
/* Turn (deg, + = left) that faces the obstacle centre seen in the profile,
 * from the wheel axle; 0 below S5_AIM_MIN_DEG, clamped to S5_AIM_MAX_DEG. */
int s5_aim_deg (const s5_profile_t * p_p);
/* Same, on a bend turning turn_pct (%, + = left): bigger aim (see s5_config.h).
 */
int s5_aim_deg_bend (const s5_profile_t * p_p, int turn_pct, bool * p_bend);
#endif
