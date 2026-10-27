#ifndef BUDDY4_MOTION_EVENTS_H
#define BUDDY4_MOTION_EVENTS_H
#include <stdbool.h>
/* Exactly 3 bits. Flags/measurement validity are separate. */
typedef enum { TERRAIN_UNKNOWN, TERRAIN_STATIONARY, TERRAIN_MOVING, TERRAIN_ACCELERATING,
    TERRAIN_TURNING, TERRAIN_CLIMBING, TERRAIN_DESCENDING, TERRAIN_IMPACT } terrain_state_t;
terrain_state_t motion_classify(float pitch, float speed, float longitudinal_g,
                               float turn_dps, bool turn_valid, bool impact);
#endif
