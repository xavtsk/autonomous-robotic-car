/* Tribunal 1 copied inter-task results. Bit order is explicit; C structure
 * layout is never sent. The decoded API stays wide for validation/calculation. */
#ifndef BUDDY3_NAVIGATION_CODEC_H
#define BUDDY3_NAVIGATION_CODEC_H
#include "line_navigation.h"

#define S3_NAVIGATION_BITS 479U
#define S3_NAVIGATION_BYTES 60U
#define S3_BARCODE_BITS 38U
#define S3_BARCODE_BYTES 5U
typedef struct { uint8_t bytes[S3_NAVIGATION_BYTES]; } s3_navigation_packet_t;
typedef struct { uint8_t bytes[S3_BARCODE_BYTES]; } s3_barcode_packet_t;
#define S3_RAW_FRAME_BYTES 34U /* frame32 + 3*(timestamp32+raw12+error32+2 flags) */
typedef struct { uint8_t bytes[S3_RAW_FRAME_BYTES]; } s3_raw_frame_packet_t;

/* False on invalid ranges/nulls; output is unchanged on failure. Floats and
 * timestamps are copied bit-for-bit, including invalid diagnostic float data.
 * Pack only task-private copies or values held under their owner's lock. */
bool s3_navigation_pack(const s3_navigation_output_t *value, s3_navigation_packet_t *out);
bool s3_navigation_unpack(const s3_navigation_packet_t *packet, s3_navigation_output_t *out);
bool s3_barcode_pack(const s3_barcode_result_t *value, s3_barcode_packet_t *out);
bool s3_barcode_unpack(const s3_barcode_packet_t *packet, s3_barcode_result_t *out);
/* For the existing stationary diagnostic snapshot. Driver error codes stay
 * signed 32-bit; an ADC failure is not collapsed into a healthy raw zero. */
bool s3_raw_frame_pack(const s3_sensor_frame_t *value, s3_raw_frame_packet_t *out);
bool s3_raw_frame_unpack(const s3_raw_frame_packet_t *packet, s3_sensor_frame_t *out);

#endif
