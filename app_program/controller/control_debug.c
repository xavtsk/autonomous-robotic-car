/** @file control_debug.c
 * @brief Observation-only controller snapshot.
 * The packet schema gives every field a fixed bit span. A host can
 * decode these bytes without the compiler layout for the local sample.
 * @see docs/IR_DEBUG_PROTOCOL.md for the debug topic and format.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * Unsigned casts spell out the original modulo conversion.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "control_debug.h"
#include "../build_assert.h"
#include <string.h>

/** Explicit bit positions describe protocol bytes, never native struct layout.
 */
typedef struct
{
    const uint16_t bit : 10;
    const uint16_t width_minus_one : 5;
    const bool     b_is_signed : 1;
} control_debug_field_info_t;

BUILD_ASSERT((unsigned)CONTROL_DEBUG_BITS <= 1024U, debug_metadata_offset_t);
#if defined(__arm__) || defined(__thumb__)
BUILD_ASSERT(sizeof(control_debug_field_info_t) == 2U, debug_metadata_size_t);
#endif

/* Widths are encoded as width minus one, so all 1..32 bit fields fit five
 * stored bits. Decode to unsigned before the unchanged validation and writes.
 * The table is private constant data; no native layout is put on the wire. */
static const control_debug_field_info_t g_debug_fields[] = {
    {0U, 32U - 1U, false},   /* timestamp_us */
    {32U, 32U - 1U, false},  /* motion_timestamp_us */
    {64U, 8U - 1U, false},   /* sequence */
    {72U, 1U - 1U, false},   /* motion_available */
    {73U, 1U - 1U, false},   /* motion_valid */
    {74U, 1U - 1U, false},   /* braked */
    {75U, 1U - 1U, false},   /* busy */
    {76U, 3U - 1U, false},   /* motion_state */
    {79U, 3U - 1U, false},   /* motion_fault */
    {82U, 13U - 1U, true},   /* setpoint_left */
    {95U, 13U - 1U, true},   /* setpoint_right */
    {108U, 13U - 1U, true},  /* measured_left */
    {121U, 13U - 1U, true},  /* measured_right */
    {134U, 8U - 1U, true},   /* duty_left */
    {142U, 8U - 1U, true},   /* duty_right */
    {150U, 20U - 1U, true},  /* encoder_left */
    {170U, 20U - 1U, true},  /* encoder_right */
    {190U, 3U - 1U, false},  /* drive_state */
    {193U, 4U - 1U, false},  /* reason */
    {197U, 3U - 1U, false},  /* pending */
    {200U, 8U - 1U, true},   /* line_error */
    {208U, 5U - 1U, false},  /* line_flags */
    {213U, 3U - 1U, false},  /* barcode_status */
    {216U, 3U - 1U, false},  /* accepted_calibration */
    {219U, 5U - 1U, false},  /* readiness */
    {224U, 16U - 1U, false}, /* barcode_events */
    {240U, 16U - 1U, false}, /* junction_events */
    {256U, 16U - 1U, false}, /* lost_events */
    {272U, 32U - 1U, false}, /* sample_gap_us */
    {304U, 32U - 1U, false}, /* run_max_gap_us */
    {336U, 12U - 1U, false}, /* left_white */
    {348U, 12U - 1U, false}, /* left_black */
    {360U, 10U - 1U, false}, /* left_white_limit */
    {370U, 10U - 1U, false}, /* left_black_limit */
    {380U, 12U - 1U, false}, /* right_white */
    {392U, 12U - 1U, false}, /* right_black */
    {404U, 10U - 1U, false}, /* right_white_limit */
    {414U, 10U - 1U, false}, /* right_black_limit */
    {424U, 12U - 1U, false}, /* barcode_white */
    {436U, 12U - 1U, false}, /* barcode_black */
    {448U, 10U - 1U, false}, /* barcode_white_limit */
    {458U, 10U - 1U, false}, /* barcode_black_limit */
    {468U, 2U - 1U, false},  /* cal_sensor */
    {470U, 1U - 1U, false},  /* collecting */
    {471U, 12U - 1U, false}, /* white_min */
    {483U, 12U - 1U, false}, /* white_max */
    {495U, 12U - 1U, false}, /* black_min */
    {507U, 12U - 1U, false}, /* black_max */
    {519U, 16U - 1U, false}, /* white_count */
    {535U, 16U - 1U, false}, /* black_count */
};

BUILD_ASSERT(sizeof(g_debug_fields) / sizeof(g_debug_fields[0]) ==
                 (unsigned)CD_FIELD_COUNT,
             debug_metadata_count_t);
static void     write_bits (uint8_t * p_bytes, unsigned * p_bit, unsigned width,
                            uint32_t value);
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width);
static bool     unsigned_fits (uint32_t value, unsigned width);
static bool     signed_fits (int32_t value, unsigned width);
static bool debug_unsigned_domain (control_debug_field_t field, uint32_t value);
static bool debug_signed_domain (control_debug_field_t field, int32_t value);
static void debug_builder_write (control_debug_builder_t * p_builder,
                                 control_debug_field_t field, uint32_t value);

/** @brief Encode a validated wide sample as the canonical 70-byte packet.
 * @param p_sample Readable sample; null is rejected. Widths follow the schema.
 * Clocks/gaps are microseconds, speeds mm/s, duty percent and encoders counts.
 * Raw reflectance is 0..4095 ADC; normalized limits are 0..1000.
 * @param p_packet Writable packet; null is rejected.
 * @return True on success. False for nulls, any width/domain error, or valid
 * motion marked unavailable; failure leaves the destination unchanged.
 * @note The caller serializes buffer access and supplies already scaled
 * values for the explicit wire fields.
 */
bool control_debug_pack (const control_debug_sample_t * p_sample,
                         control_debug_packet_t *       p_packet)
{
    unsigned bit             = 0;
    bool     b_do_fields_fit = true;

    if ((((((((((((((((((((((0 == p_sample) || (0 == p_packet)) ||
                           (p_sample->motion_state > 5U)) ||
                          (p_sample->motion_fault > 5U)) ||
                         (p_sample->drive_state > 6U)) ||
                        (p_sample->reason > 10U)) ||
                       (p_sample->pending > 4U)) ||
                      (p_sample->barcode_status > 4U)) ||
                     (p_sample->cal_sensor > 2U)) ||
                    (p_sample->line_error < -100)) ||
                   (p_sample->line_error > 100)) ||
                  (p_sample->duty_left < -100)) ||
                 (p_sample->duty_left > 100)) ||
                (p_sample->duty_right < -100)) ||
               (p_sample->duty_right > 100)) ||
              (p_sample->left_white_limit > 1000U)) ||
             (p_sample->left_black_limit > 1000U)) ||
            (p_sample->right_white_limit > 1000U)) ||
           (p_sample->right_black_limit > 1000U)) ||
          (p_sample->barcode_white_limit > 1000U)) ||
         (p_sample->barcode_black_limit > 1000U)) ||
        ((p_sample->b_is_motion_valid) && (!p_sample->b_has_motion_snapshot)))
    {
        return false;
    }
#define CD_U(name, width)                                                      \
    (b_do_fields_fit =                                                         \
         b_do_fields_fit && unsigned_fits(p_sample->name, (width)));
#define CD_S(name, width)                                                      \
    (b_do_fields_fit = b_do_fields_fit && signed_fits(p_sample->name, (width)));
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    if (!b_do_fields_fit)
    {
        return false;
    }

    memset(p_packet, 0, sizeof *p_packet);
    p_packet->bytes[0] = 1;
/* The validated unsigned fields already fit uint32_t. Convert signed
 * fields modulo 2^32 and write only the declared field width. */
#define CD_U(name, width)                                                      \
    (write_bits(p_packet->bytes, &bit, (width), (uint32_t)p_sample->name));
    CONTROL_DEBUG_FIELDS(CD_U, CD_U)
#undef CD_U
    return true;
}

/** @brief Decode and validate a canonical version-1 controller packet.
 * @param p_packet Readable complete 70-byte packet; null is rejected.
 * @param p_sample Writable wide result in the units documented by pack.
 * @return True only when decoded values re-encode to every original byte,
 * including zero padding. Nulls/bad version leave p_sample unchanged; with
 * version 1, fields are written before later semantic/canonical rejection.
 * @note The caller serializes access to its buffers and uses the result
 * only on success.
 */
bool control_debug_unpack (const control_debug_packet_t * p_packet,
                           control_debug_sample_t *       p_sample)
{
    unsigned               bit = 0;
    control_debug_packet_t canonical;

    if (((0 == p_packet) || (0 == p_sample)) || (p_packet->bytes[0] != 1U))
    {
        return false;
    }
#define CD_U(name, width)                                                      \
    (p_sample->name = read_bits(p_packet->bytes, &bit, (width)));
/* Signed fields are 8, 13 or 20 bits. The masked low bits and sign
 * contribution each fit int32_t before subtraction. */
#define CD_S(name, width)                                                      \
    {                                                                          \
        const unsigned field_width = (width);                                  \
        uint32_t value_bits = read_bits(p_packet->bytes, &bit, field_width);   \
        p_sample->name =                                                       \
            (int32_t)(value_bits &                                             \
                      ((UINT32_C(1) << (field_width - 1)) - 1U)) -             \
            (int32_t)(value_bits & (UINT32_C(1) << (field_width - 1)));        \
    }
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    return (control_debug_pack(p_sample, &canonical)) &&
           (memcmp(p_packet, &canonical, sizeof *p_packet) == 0);
}

/** @brief Reset caller-owned scratch to a valid version-1 zero sample.
 * @param p_builder Writable builder or null; null is a no-op.
 * @post The payload starts at zero and the error latch clears.
 * @note The caller exclusively owns and synchronizes access to the builder.
 */
void control_debug_builder_init (control_debug_builder_t * p_builder)
{
    if (NULL != p_builder)
    {
        (void)memset(p_builder, 0, sizeof(*p_builder));
        p_builder->packet.bytes[0] = 1U;
        p_builder->b_is_valid      = true;
    }
}

/** @brief Validate and encode an unsigned wide public field value.
 * @param p_builder Initialized caller-owned builder; null is a no-op.
 * @param field Unsigned schema selector; negative/out-of-range or signed
 * selectors latch failure without writing packet bits.
 * @param value Full uint32_t input in schema units; width/domain failures
 * latch failure without narrowing or changing the selected packet bits.
 * @post Later valid writes still update fields after an error,
 * but they cannot clear the failure latch; init is required to clear it.
 * @note The caller serializes access to this private builder.
 */
void control_debug_builder_u (control_debug_builder_t * p_builder,
                              control_debug_field_t field, uint32_t value)
{
    if (NULL == p_builder)
    {
        return;
    }
    /* Converting a negative selector to unsigned also makes it out of range. */
    if (((((uint32_t)field >= (unsigned int)CD_FIELD_COUNT) ||
          (g_debug_fields[field].b_is_signed)) ||
         (!unsigned_fits(value,
                         (g_debug_fields[field].width_minus_one + 1U)))) ||
        (!debug_unsigned_domain(field, value)))
    {
        p_builder->b_is_valid = false;
    }
    else
    {
        debug_builder_write(p_builder, field, value);
    }
}

/** @brief Validate and encode a signed wide public field value.
 * @param p_builder Initialized caller-owned builder; null is a no-op.
 * @param field Signed schema selector; negative/out-of-range or unsigned
 * selectors latch failure without writing packet bits.
 * @param value Full int32_t input in schema units; no prevalidation clipping.
 * Valid values retain the public packer's low-bit two's-complement encoding.
 * @post Rejection latches failure; later valid writes cannot
 * clear that latch, although they still replace their selected field bits.
 * @note The caller serializes access to this private builder.
 */
void control_debug_builder_s (control_debug_builder_t * p_builder,
                              control_debug_field_t field, int32_t value)
{
    if (NULL == p_builder)
    {
        return;
    }
    /* Converting a negative selector to unsigned also makes it out of range. */
    if (((((uint32_t)field >= (unsigned int)CD_FIELD_COUNT) ||
          (!g_debug_fields[field].b_is_signed)) ||
         (!signed_fits(value, (g_debug_fields[field].width_minus_one + 1U)))) ||
        (!debug_signed_domain(field, value)))
    {
        p_builder->b_is_valid = false;
    }
    else
    {
        /* Conversion modulo 2^32 preserves the low-width two's-complement
         * encoding already used by the public packer for signed values. */
        debug_builder_write(p_builder, field, (uint32_t)value);
    }
}

/** @brief Copy a valid builder packet to caller-owned output storage.
 * @param p_builder Readable initialized builder, or null for rejection.
 * @param p_packet Writable output packet, or null for rejection.
 * @return True after the full packet copy. False leaves output unchanged
 * for nulls, a latched field error, or motion valid while unavailable.
 * @note Finishing leaves the builder intact. A motion-availability mismatch
 * can be corrected before another attempt because it leaves the latch clear.
 * The caller owns synchronization; this function itself publishes nothing.
 */
bool control_debug_builder_end (const control_debug_builder_t * p_builder,
                                control_debug_packet_t *        p_packet)
{
    unsigned int available_bit = g_debug_fields[CD_MOTION_AVAILABLE].bit;
    unsigned int valid_bit     = g_debug_fields[CD_MOTION_VALID].bit;

    if (((NULL == p_builder) || (NULL == p_packet)) || (!p_builder->b_is_valid))
    {
        return false;
    }

    if ((0U != read_bits(p_builder->packet.bytes, &valid_bit, 1U)) &&
        (0U == read_bits(p_builder->packet.bytes, &available_bit, 1U)))
    {
        return false;
    }

    *p_packet = p_builder->packet;
    return true;
}

/** @brief OR validated low-order value bits into the packet body.
 * @param p_bytes Writable packet of CONTROL_DEBUG_PACKET_SIZE bytes.
 * @param p_bit Non-null body-bit cursor; cursor plus width is at most 551.
 * @param width Number of bits, from 1 through 32.
 * @param value Already validated or explicitly encoded unsigned value.
 * @post The cursor advances by width. Existing one bits remain set.
 * @note The caller initializes storage and exclusively owns access to it.
 */
static void write_bits (uint8_t * p_bytes, unsigned * p_bit, unsigned width,
                        uint32_t value)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        /* Widen the existing byte for the OR. The inserted mask changes
         * only one of its eight bits, so the result still fits uint8_t. */
        p_bytes[1U + (*p_bit / 8U)] =
            (uint8_t)((unsigned)p_bytes[1U + (*p_bit / 8U)] |
                      (((value >> bit_offset) & 1U) << (*p_bit % 8U)));
        ++bit_offset;
        ++*p_bit;
    }
}

/** @brief Read a little-bit-order field following the version byte.
 * @param p_bytes Readable complete packet; this helper does not validate it.
 * @param p_bit Non-null body-bit cursor; cursor plus width is at most 551.
 * @param width Number of bits, from 1 through 32.
 * @return The unsigned field value; advances p_bit by width.
 * @note The caller synchronizes access to the packet storage.
 */
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width)
{
    uint32_t field      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        /* The extracted zero or one must be uint32_t before shifting
         * it to any position from bit 0 through bit 31. */
        field |= (uint32_t)((p_bytes[1U + (*p_bit / 8U)] >> (*p_bit % 8U)) & 1U)
                 << bit_offset;
        ++bit_offset;
        ++*p_bit;
    }

    return field;
}

/** @brief Test the unsigned schema range without overflowing at 32 bits.
 * @param value Full-width public input, including invalid sentinels.
 * @param width Schema width from 1 through 32 bits.
 * @return True exactly when value is below two to the power width.
 * @note The calculation has no side effects.
 */
static bool unsigned_fits (uint32_t value, unsigned width)
{
    /* The 64-bit limit can represent 2^32 for every schema width. */
    return (uint64_t)value < (UINT64_C(1) << width);
}

/** @brief Test the exact signed two's-complement schema interval.
 * @param value Full-width signed public input, including invalid sentinels.
 * @param width Schema width from 1 through 32 bits.
 * @return True for minus two to power(width-1) through that power minus one.
 * @note The calculation uses int64_t intermediates and has no side effects.
 */
static bool signed_fits (int32_t value, unsigned width)
{
    /* With schema widths 1..32, the unsigned power fits in int64_t. */
    const int64_t limit = (int64_t)(UINT64_C(1) << (width - 1U));
    return ((int64_t)value >= -limit) && ((int64_t)value < limit);
}

/** Apply the same additional domains as the wide public pack interface. */
/** @brief Check unsigned semantic limits beyond the encoded field width.
 * @param field Schema selector already checked by the public setter.
 * @param value Full-width unsigned input; no narrowing has occurred.
 * @return True for the field's existing domain. Other selectors return true
 * here and rely on the caller's separate selector and width validation.
 * @note Field validation has no side effects.
 */
static bool debug_unsigned_domain (control_debug_field_t field, uint32_t value)
{
    switch (field)
    {
    case CD_MOTION_STATE: /* State and fault share the same maximum. */
    case CD_MOTION_FAULT:
        return value <= 5U;
    case CD_DRIVE_STATE:
        return value <= 6U;
    case CD_REASON:
        return value <= 10U;
    case CD_PENDING: /* Both selectors use codes from zero through four. */
    case CD_BARCODE_STATUS:
        return value <= 4U;
    case CD_CAL_SENSOR:
        return value <= 2U;
    case CD_LEFT_WHITE_LIMIT: /* All six limits use the same normalized scale.
                               */
    case CD_LEFT_BLACK_LIMIT:
    case CD_RIGHT_WHITE_LIMIT:
    case CD_RIGHT_BLACK_LIMIT:
    case CD_BARCODE_WHITE_LIMIT:
    case CD_BARCODE_BLACK_LIMIT:
        return value <= 1000U;
    default:
        return true;
    }
}

/** Apply the same signed semantic limits as the wide public pack interface. */
/** @brief Check signed semantic limits beyond the encoded field width.
 * @param field Schema selector already checked by the public setter.
 * @param value Full-width input; line error and duty must be -100..100.
 * @return True for the semantic domain; other fields rely on width checks.
 * @pre The caller has already rejected invalid selectors.
 */
static bool debug_signed_domain (control_debug_field_t field, int32_t value)
{
    switch (field)
    {
    case CD_LINE_ERROR: /* Line error and both duties share a signed range. */
    case CD_DUTY_LEFT:
    case CD_DUTY_RIGHT:
        return (value >= -100) && (value <= 100);
    default:
        return true;
    }
}

/** @brief Replace exactly one validated field in the caller's packet.
 * @param p_builder Non-null, caller-owned initialized builder.
 * @param field Valid schema selector in 0..CD_FIELD_COUNT-1.
 * @param value Valid unsigned bits or explicitly encoded signed low bits.
 * @post Only the selected field changes. A zero value clears its bits.
 * @note Public setters validate the value and manage the validity latch.
 */
static void debug_builder_write (control_debug_builder_t * p_builder,
                                 control_debug_field_t field, uint32_t value)
{
    const control_debug_field_info_t * p_info = &g_debug_fields[field];
    uint32_t                           index  = 0U;

    for (index = 0U; index < (p_info->width_minus_one + 1U); ++index)
    {
        const uint32_t bit      = p_info->bit + index;
        const uint32_t byte     = 1U + (bit / 8U);
        const uint32_t mask     = UINT32_C(1) << (bit % 8U);
        const uint32_t previous = p_builder->packet.bytes[byte];
        const uint32_t next =
            (previous & ~mask) | (((value >> index) & 1U) << (bit % 8U));
        /* The mask only changes one bit of an existing byte. */
        p_builder->packet.bytes[byte] = (uint8_t)next;
    }
}

/* End of file. */

