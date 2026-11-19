/** @file navigation_codec.h
 * @brief Define copied navigation, barcode and raw-frame packet layouts.
 */
/* Tribunal 1 copied inter-task results. Bit order is explicit; C structure
 * layout is never sent. The decoded API stays wide for validation/calculation.
 */
#ifndef BUDDY3_NAVIGATION_CODEC_H
#define BUDDY3_NAVIGATION_CODEC_H
#include "line_navigation.h"

#define S3_NAVIGATION_BITS 479U
#define S3_NAVIGATION_BYTES 60U
#define S3_BARCODE_BITS 38U
#define S3_BARCODE_BYTES 5U

typedef struct
{
    uint8_t bytes[S3_NAVIGATION_BYTES];
} s3_navigation_packet_t;

typedef struct
{
    uint8_t bytes[S3_BARCODE_BYTES];
} s3_barcode_packet_t;

#define S3_RAW_FRAME_BYTES                                                     \
    34U /* frame32 + 3*(timestamp32+raw12+error32+2 flags) */

typedef struct
{
    uint8_t bytes[S3_RAW_FRAME_BYTES];
} s3_raw_frame_packet_t;

/* False on invalid ranges/nulls; output is unchanged on failure. Floats and
 * timestamps are copied bit-for-bit, including invalid diagnostic float data.
 * Pack only task-private copies or values held under their owner's lock. */
/** @brief Encode one navigation result into its 479-bit copied packet.
 * @param[in] p_value Stable result; sensor darkness is 0..1000, line error
 * -100..100, and barcode letter is NUL or A..Z. Enum values must be within
 * their declared nonnegative ranges. Events/readiness use five bits, sensor
 * fault/calibration masks three bits, and latched faults four bits.
 * @param[out] p_out Writable 60-byte packet; unchanged on failure.
 * @return True after replacing the whole packet; false for a null pointer
 * or a rejected range, including fields whose validity flag is false.
 * @details Fields follow the implementation's fixed order, least-significant
 * bit first within each field and byte. The final unused bit is zero.
 * Timestamps/gaps remain microseconds, steering remains m/s, and capture
 * distances remain metres. Binary32 steering and binary64 distance values
 * are copied without scaling, rounding or finite-value validation.
 * @pre The caller owns the output and keeps the input stable for the call.
 * No lock is acquired and no pointer is retained.
 */
bool s3_navigation_pack (const s3_navigation_output_t * p_value,
                         s3_navigation_packet_t * p_out);
/** @brief Decode a navigation packet after checking ranges and padding.
 * @param[in] p_packet Readable, stable 60-byte packet in the packer's layout.
 * @param[out] p_out Caller-owned result; replaced only on success.
 * @return False for a null pointer, nonzero final padding bit, invalid enum,
 * barcode code, darkness or line-error code; true for an accepted packet.
 * @details Output is first assembled locally. Float/double representations
 * and integer units follow s3_navigation_pack; NaNs and infinities are not
 * rejected. Checks enforce field domains, not consistency between validity
 * flags, events and diagnostic values. No pointer is retained or lock taken.
 */
bool s3_navigation_unpack (const s3_navigation_packet_t * p_packet,
                           s3_navigation_output_t * p_out);
/** @brief Encode a barcode result as 38 bits with zero high padding.
 * @param[in] p_value Stable result with letter NUL or A..Z.
 * @param[out] p_out Caller-owned five-byte packet; unchanged on failure.
 * @return True after replacing the packet; false for nulls or other letters.
 * @details The layout is letter code 0..26 in five bits, reversed in one
 * bit, then the unchanged 32-bit microsecond timestamp, all low bit first.
 * No pointer is retained; the caller supplies synchronization if needed.
 */
bool s3_barcode_pack (const s3_barcode_result_t * p_value,
                      s3_barcode_packet_t * p_out);
/** @brief Decode a barcode result while rejecting reserved codes and bits.
 * @param[in] p_packet Readable, stable five-byte barcode packet.
 * @param[out] p_out Caller-owned result; unchanged on failure.
 * @return False for nulls, letter codes 27..31 or either high padding bit;
 * otherwise true after assigning the decoded letter, direction and timestamp.
 * @details Code zero means NUL; 1..26 mean A..Z. Timestamps are not checked
 * for age or order. The function neither locks nor retains caller pointers.
 */
bool s3_barcode_unpack (const s3_barcode_packet_t * p_packet,
                        s3_barcode_result_t * p_out);
/* For the existing stationary diagnostic snapshot. Driver error codes stay
 * signed 32-bit; an ADC failure is not collapsed into a healthy raw zero. */
/** @brief Encode a complete three-sensor frame into 266 meaningful bits.
 * @param[in] p_value Stable frame; every raw reading must be 0..4095,
 * including readings marked disabled or invalid.
 * @param[out] p_out Caller-owned 34-byte packet; unchanged on failure.
 * @return False for nulls or an out-of-range raw reading; otherwise true.
 * @details The frame timestamp precedes each sensor's timestamp, 12-bit raw
 * reading, signed 32-bit error and enabled/valid bits, in sensor-array order.
 * Fields are written low bit first; timestamps retain microsecond units and
 * all error bits are retained. Six final padding bits are zero. The caller
 * provides any required locking; no pointer is retained.
 */
bool s3_raw_frame_pack (const s3_sensor_frame_t * p_value,
                        s3_raw_frame_packet_t * p_out);
/** @brief Decode a raw frame without discarding failed ADC sample details.
 * @param[in] p_packet Readable, stable 34-byte frame packet.
 * @param[out] p_out Caller-owned frame; replaced only on success.
 * @return False for nulls or nonzero final six padding bits; otherwise true.
 * @details All 32-bit timestamps and signed error patterns are accepted.
 * Flags do not suppress raw/error values. Units and field order follow
 * s3_raw_frame_pack. There is no timestamp freshness or error-code lookup,
 * synchronization, allocation or retention of caller pointers.
 */
bool s3_raw_frame_unpack (const s3_raw_frame_packet_t * p_packet,
                          s3_sensor_frame_t * p_out);

#endif

/* End of file. */

