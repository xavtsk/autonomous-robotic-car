#ifndef S5_CODEC_H
#define S5_CODEC_H
#include "obstacle_profile.h"
/* Source timestamps are preserved across copies; no refreshing stale data. */
typedef struct {
    uint32_t range_us, scan_us;
    uint16_t range_mm, scan_id;
    s5_profile_t profile;
    bool range_valid, no_echo, scan_done;
} s5_observation_t;
#define S5_PACKET_BYTES 23U
bool s5_observation_pack(const s5_observation_t *v,uint8_t out[S5_PACKET_BYTES]);
bool s5_observation_unpack(const uint8_t in[S5_PACKET_BYTES],s5_observation_t *v);
#endif
