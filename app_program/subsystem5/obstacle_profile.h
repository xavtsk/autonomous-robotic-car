#ifndef BUDDY5_OBSTACLE_PROFILE_H
#define BUDDY5_OBSTACLE_PROFILE_H
#include "s5_config.h"
/* 12 distance bits + 5 angle-index bits + validity = 18 bits / 3 bytes.
 * Angle index 0..24 means 30..150 degrees in 5 degree steps. */
typedef struct { uint8_t bytes[3]; } s5_sample_t;
typedef struct {
    uint16_t closest_mm, width_mm, left_clear_mm, right_clear_mm;
    int16_t left_edge_mm, right_edge_mm;
    /* Near/far longitudinal bounds of the observed sonar bearing envelopes.
     * Neither is an exact obstacle surface; hidden surfaces remain unknown. */
    uint16_t forward_mm, front_mm;
    bool valid;
} s5_profile_t;
bool s5_sample_pack(unsigned angle, unsigned mm, bool valid, s5_sample_t *out);
bool s5_sample_unpack(const s5_sample_t *sample, unsigned *angle, unsigned *mm, bool *valid);
/* A range that is not in the near-front slab is not automatically clear.
 * FAR requires its whole assumed cone to project beyond that slab. */
typedef enum { S5_FRONT_BLOCKED, S5_FRONT_NEAR, S5_FRONT_FAR } s5_front_class_t;
s5_front_class_t s5_profile_front_class(unsigned angle,unsigned mm,unsigned centre,
                                        const s5_config_t *cfg);
bool s5_profile_build(const s5_sample_t *samples, unsigned count,
                      const s5_config_t *cfg, s5_profile_t *out);
/* Side peek result: replaces that side's clearance in the last profile (kept
 * for telemetry); with no previous profile, a minimal valid one is made. */
bool s5_peek_merge(s5_profile_t *profile, const s5_sample_t *sample);
#endif
