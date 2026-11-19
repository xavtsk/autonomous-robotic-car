/** @file profile_obstacle.h
 * @brief Pack scan samples and derive obstacle profiles and side-peek updates.
 */
#ifndef BUDDY5_OBSTACLE_PROFILE_H
#define BUDDY5_OBSTACLE_PROFILE_H
#include "s5_config.h"

/* 12 distance bits + 5 angle-index bits + validity = 18 bits / 3 bytes.
 * Angle index 0..24 means 30..150 degrees in 5 degree steps. */
typedef struct
{
    uint8_t bytes[3];
} s5_sample_t;

typedef struct
{
    uint16_t closest_mm, width_mm, left_clear_mm, right_clear_mm;
    int16_t left_edge_mm, right_edge_mm;
    uint16_t forward_mm;
    bool valid;
} s5_profile_t;

bool s5_sample_pack (unsigned angle, unsigned value_mm, bool valid,
                     s5_sample_t * p_out);
bool s5_sample_unpack (const s5_sample_t * p_sample, unsigned * p_angle,
                       unsigned * p_mm, bool * p_valid);
bool s5_profile_build (const s5_sample_t * p_samples, unsigned count,
                       const s5_config_t * p_cfg, s5_profile_t * p_out);
/* Side peek result: replaces that side's clearance in the last profile (kept
 * for telemetry); with no previous profile, a minimal valid one is made. */
bool s5_peek_merge (s5_profile_t * p_profile, const s5_sample_t * p_sample);
#endif

/* End of file. */

