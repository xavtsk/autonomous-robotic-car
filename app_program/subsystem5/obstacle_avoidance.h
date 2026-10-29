#ifndef BUDDY5_OBSTACLE_AVOIDANCE_H
#define BUDDY5_OBSTACLE_AVOIDANCE_H
#include "avoidance_planner.h"
#include "line_recovery.h"
#include "s5_codec.h"
typedef enum { S5_IDLE,S5_WAIT_STOP,S5_SCANNING,S5_MOVING,S5_FINISH,S5_COMPLETE,S5_FAULT } s5_state_t;
typedef enum { S5_OK,S5_BAD_RANGE,S5_NO_PATH,S5_SCAN_FAILED,S5_DEADLINE,S5_NO_LINE,S5_MOTION_ERROR,S5_INTERRUPTED } s5_fault_t;
/* S5_ACT_PEEK: value = servo angle for one side reading while stopped. */
typedef enum { S5_ACT_NONE,S5_ACT_STOP,S5_ACT_SCAN,S5_ACT_DRIVE,S5_ACT_TURN,S5_ACT_RESUME,S5_ACT_PEEK } s5_action_kind_t;
/* Leg 3 (passing the obstacle) sub-steps: drive to the front face, then
 * peek / step until the side is clear, then clear the car body. */
typedef enum { S5_PASS_APPROACH,S5_PASS_PEEK,S5_PASS_PEEKING,S5_PASS_STEP,S5_PASS_EXTRA } s5_pass_t;
typedef struct { s5_action_kind_t kind; int16_t value; uint16_t speed_mms; } s5_action_t;
typedef struct {
    uint32_t now_us; double centre_m;
    bool stopped, motion_done, healthy, tape, junction, line_ready;
    s5_observation_t observation;
} s5_input_t;
typedef struct {
    s5_config_t config; s5_plan_t plan; s5_recovery_t recovery;
    unsigned state:3, fault:3, leg:4, issued:1, pass:3, swapped:1;
    uint16_t searched_mm; /* leg 3 distance stepped beside the obstacle */
    uint32_t phase_us, started_us;
    double leg_start_m;
} s5_avoidance_t;
void s5_avoidance_init(s5_avoidance_t *a,const s5_config_t *config);
bool s5_range_ready(const s5_observation_t *o,uint32_t now);
void s5_avoidance_begin(s5_avoidance_t *a,uint32_t now);
void s5_avoidance_abort(s5_avoidance_t *a);
s5_action_t s5_avoidance_step(s5_avoidance_t *a,const s5_input_t *input);
#endif
