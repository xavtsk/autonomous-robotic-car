#include "motion_events.h"
#include <math.h>
terrain_state_t motion_classify(float pitch,float speed,float longitudinal_g,float turn,bool turn_valid,bool impact)
{
    if(impact) { return TERRAIN_IMPACT; }
    if(turn_valid && fabsf(turn)>15) { return TERRAIN_TURNING; }
    if(fabsf(speed)>0.01f && pitch>4) { return TERRAIN_CLIMBING; }
    if(fabsf(speed)>0.01f && pitch < -4) { return TERRAIN_DESCENDING; }
    if(fabsf(longitudinal_g)>0.12f) { return TERRAIN_ACCELERATING; }
    return fabsf(speed)<0.01f?TERRAIN_STATIONARY:TERRAIN_MOVING;
}
