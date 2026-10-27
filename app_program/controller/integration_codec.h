/* Explicit shared copies. Decoded calculation views never cross the mutex. */
#ifndef TEAM_INTEGRATION_CODEC_H
#define TEAM_INTEGRATION_CODEC_H
#include "s3_integration.h"
#include "../subsystem3/navigation_codec.h"
typedef struct { uint8_t bytes[11]; } s3_control_packet_t; /* 83 bits */
typedef struct {
    s3_navigation_packet_t navigation; /* 479 bits, 60 bytes */
    uint8_t feedback[22];              /* 176 bits, exact binary64 odometry */
    uint8_t metadata[64];              /* 509 bits, including run timing */
} s3_status_packet_t;
bool s3_control_pack(const s3_control_request_t *value,s3_control_packet_t *out);
bool s3_control_unpack(const s3_control_packet_t *packet,s3_control_request_t *out);
bool s3_status_pack(const s3_integration_status_t *value,s3_status_packet_t *out);
bool s3_status_unpack(const s3_status_packet_t *packet,s3_integration_status_t *out);
#endif
