/** @file s5_codec.h
 * @brief Encode and decode copied obstacle-observation packets.
 */
#ifndef S5_CODEC_H
#define S5_CODEC_H
#include "profile_obstacle.h"

/* Source timestamps are preserved across copies; no refreshing stale data. */
typedef struct
{
    uint32_t           range_us;
    uint32_t           scan_us;
    uint16_t           range_mm;
    uint16_t           scan_id;
    profile_obstacle_t profile;
    bool               b_is_range_valid;
    bool               b_has_no_echo;
    bool               b_is_scan_done;
} s5_codec_observation_t;

#define S5_PACKET_BYTES 26U
bool s5_codec_observation_pack (const s5_codec_observation_t * p_observation,
                                uint8_t p_out[S5_PACKET_BYTES]);
bool s5_codec_observation_unpack (const uint8_t p_in[S5_PACKET_BYTES],
                                  s5_codec_observation_t * p_observation);
#endif

/* End of file. */

