/** @file control_debug.h
 * @brief Observation-only controller snapshot.
 */
/* Observation-only controller snapshot. Separate topic/version from IR/v2.
 * Explicit bit widths; no C struct layout is transmitted. Signed quantities
 * use two's complement. No callbacks to motion/control exist in the codec. */
#ifndef CONTROL_DEBUG_H
#define CONTROL_DEBUG_H
#include <stdbool.h>
#include <stdint.h>
// clang-format off
#define CONTROL_DEBUG_FIELDS(U, S) \
    U(timestamp_us, 32) \
    U(motion_timestamp_us, 32) \
    U(sequence, 8) \
    U(motion_available, 1) \
    U(motion_valid, 1) \
    U(braked, 1) \
    U(busy, 1) \
    U(motion_state, 3) \
    U(motion_fault, 3) \
    S(setpoint_left, 13) \
    S(setpoint_right, 13) \
    S(measured_left, 13) \
    S(measured_right, 13) \
    S(duty_left, 8) \
    S(duty_right, 8) \
    S(encoder_left, 20) \
    S(encoder_right, 20) \
    U(drive_state, 3) \
    U(reason, 4) \
    U(pending, 3) \
    S(line_error, 8) \
    U(line_flags, 5) \
    U(barcode_status, 3) \
    U(accepted_calibration, 3) \
    U(readiness, 5) \
    U(barcode_events, 16) \
    U(junction_events, 16) \
    U(lost_events, 16) \
    U(sample_gap_us, 32) \
    U(run_max_gap_us, 32) \
    U(left_white, 12) \
    U(left_black, 12) \
    U(left_white_limit, 10) \
    U(left_black_limit, 10) \
    U(right_white, 12) \
    U(right_black, 12) \
    U(right_white_limit, 10) \
    U(right_black_limit, 10) \
    U(barcode_white, 12) \
    U(barcode_black, 12) \
    U(barcode_white_limit, 10) \
    U(barcode_black_limit, 10) \
    U(cal_sensor, 2) \
    U(collecting, 1) \
    U(white_min, 12) \
    U(white_max, 12) \
    U(black_min, 12) \
    U(black_max, 12) \
    U(white_count, 16) \
    U(black_count, 16)
// clang-format on
#define CD_COUNT(name, width) +width

enum
{
    CONTROL_DEBUG_BITS = 0 CONTROL_DEBUG_FIELDS(CD_COUNT, CD_COUNT),
    CONTROL_DEBUG_PACKET_SIZE = 1 + (CONTROL_DEBUG_BITS + 7) / 8
};

#undef CD_COUNT
#define CD_U(name, width) uint32_t name;
#define CD_S(name, width) int32_t name;

typedef struct
{
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
} control_debug_sample_t;

#undef CD_U
#undef CD_S

typedef struct
{
    uint8_t bytes[CONTROL_DEBUG_PACKET_SIZE];
} control_debug_packet_t;

/** Field selectors follow the explicit wire schema, independent of C layout. */
typedef enum
{
    CD_TIMESTAMP_US,
    CD_MOTION_TIMESTAMP_US,
    CD_SEQUENCE,
    CD_MOTION_AVAILABLE,
    CD_MOTION_VALID,
    CD_BRAKED,
    CD_BUSY,
    CD_MOTION_STATE,
    CD_MOTION_FAULT,
    CD_SETPOINT_LEFT,
    CD_SETPOINT_RIGHT,
    CD_MEASURED_LEFT,
    CD_MEASURED_RIGHT,
    CD_DUTY_LEFT,
    CD_DUTY_RIGHT,
    CD_ENCODER_LEFT,
    CD_ENCODER_RIGHT,
    CD_DRIVE_STATE,
    CD_REASON,
    CD_PENDING,
    CD_LINE_ERROR,
    CD_LINE_FLAGS,
    CD_BARCODE_STATUS,
    CD_ACCEPTED_CALIBRATION,
    CD_READINESS,
    CD_BARCODE_EVENTS,
    CD_JUNCTION_EVENTS,
    CD_LOST_EVENTS,
    CD_SAMPLE_GAP_US,
    CD_RUN_MAX_GAP_US,
    CD_LEFT_WHITE,
    CD_LEFT_BLACK,
    CD_LEFT_WHITE_LIMIT,
    CD_LEFT_BLACK_LIMIT,
    CD_RIGHT_WHITE,
    CD_RIGHT_BLACK,
    CD_RIGHT_WHITE_LIMIT,
    CD_RIGHT_BLACK_LIMIT,
    CD_BARCODE_WHITE,
    CD_BARCODE_BLACK,
    CD_BARCODE_WHITE_LIMIT,
    CD_BARCODE_BLACK_LIMIT,
    CD_CAL_SENSOR,
    CD_COLLECTING,
    CD_WHITE_MIN,
    CD_WHITE_MAX,
    CD_BLACK_MIN,
    CD_BLACK_MAX,
    CD_WHITE_COUNT,
    CD_BLACK_COUNT,
    CD_FIELD_COUNT
} control_debug_field_t;

/** Sampling-owner scratch; no pointer to this object is published to tasks. */
typedef struct
{
    control_debug_packet_t packet;
    bool valid : 1;
} control_debug_builder_t;

/** Reset the builder to the version-1 zero sample, owned by the caller. */
/** @brief Reset caller-owned scratch to a valid version-1 zero sample.
 * @param p_builder Writable builder or null; null is a no-op.
 * @post clears all prior packet bits and the previous error latch.
 * @note No locks or callbacks. The caller exclusively owns the scratch object.
 */
void control_debug_builder_init (control_debug_builder_t * p_builder);
/** Validate an unsigned public value before encoding; errors remain latched. */
/** @brief Validate and encode an unsigned wide public field value.
 * @param p_builder Initialized caller-owned builder; null is a no-op.
 * @param field Unsigned schema selector; negative/out-of-range or signed
 * selectors latch failure without writing packet bits.
 * @param value Full uint32_t input in schema units; width/domain failures
 * latch failure without narrowing or changing the selected packet bits.
 * @post Later valid writes still update fields after an error,
 * but they cannot clear the failure latch; init is required to clear it.
 * @note Caller serializes builder access. No publication or motor effect.
 */
void control_debug_builder_u (control_debug_builder_t * p_builder,
                              control_debug_field_t field, uint32_t value);
/** Validate a signed public value before encoding; errors remain latched. */
/** @brief Validate and encode a signed wide public field value.
 * @param p_builder Initialized caller-owned builder; null is a no-op.
 * @param field Signed schema selector; negative/out-of-range or unsigned
 * selectors latch failure without writing packet bits.
 * @param value Full int32_t input in schema units; no prevalidation clipping.
 * Valid values retain the public packer's low-bit two's-complement encoding.
 * @post Rejection latches failure; later valid writes cannot
 * clear that latch, although they still replace their selected field bits.
 * @note Caller serializes builder access. No locks, callbacks or publication.
 */
void control_debug_builder_s (control_debug_builder_t * p_builder,
                              control_debug_field_t field, int32_t value);
/** Publish only a valid complete sample; failure leaves p_packet unchanged. */
/** @brief Copy a valid builder packet to caller-owned output storage.
 * @param p_builder Readable initialized builder, or null for rejection.
 * @param p_packet Writable output packet, or null for rejection.
 * @return True after the full packet copy. False leaves output unchanged
 * for nulls, a latched field error, or motion valid while unavailable.
 * @note Does not change the builder. The motion-availability relationship
 * failure is not latched, so changing that field can permit a later end.
 * The caller owns synchronization; this function itself publishes nothing.
 */
bool control_debug_builder_end (const control_debug_builder_t * p_builder,
                                control_debug_packet_t * p_packet);

/** Encode a debug sample; return false for null pointers or invalid fields. */
/** @brief Encode a validated wide sample as the canonical 70-byte packet.
 * @param p_sample Readable sample; null is rejected. Widths follow the schema.
 * Clocks/gaps are microseconds, speeds mm/s, duty percent and encoders counts.
 * Raw reflectance is 0..4095 ADC; normalized limits are 0..1000.
 * @param p_packet Writable packet; null is rejected.
 * @return True on success. False for nulls, any width/domain error, or valid
 * motion marked unavailable; failure leaves the destination unchanged.
 * @note Copies no floating/native structure layout. No locks or callbacks;
 * the caller owns input/output access and supplies the already scaled units.
 */
bool control_debug_pack (const control_debug_sample_t * p_sample,
                         control_debug_packet_t * p_packet);
/** Decode a canonical packet. Use the output only when this returns true. */
/** @brief Decode and validate a canonical version-1 controller packet.
 * @param p_packet Readable complete 70-byte packet; null is rejected.
 * @param p_sample Writable wide result in the units documented by pack.
 * @return True only when decoded values re-encode to every original byte,
 * including zero padding. Nulls/bad version leave p_sample unchanged; with
 * version 1, fields are written before later semantic/canonical rejection.
 * @note Use the result only on true. No locks, callbacks or shared mutation;
 * the caller serializes access to its buffers.
 */
bool control_debug_unpack (const control_debug_packet_t * p_packet,
                           control_debug_sample_t * p_sample);
#endif

/* End of file. */

