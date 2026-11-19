/** @file control_debug.c
 * @brief Observation-only controller snapshot.
 */
#include "control_debug.h"
#include <string.h>

/** @brief OR validated low-order value bits into the packet body.
 * @param p_p Writable packet of CONTROL_DEBUG_PACKET_SIZE bytes.
 * @param p_bit Non-null body-bit cursor; cursor plus width is at most 551.
 * @param width Number of bits, from 1 through 32.
 * @param value Already validated or explicitly encoded unsigned value.
 * @post advances the cursor by width. Existing one bits remain set.
 * @note Private helper; the caller initializes storage and owns all access.
 */
static void put (uint8_t * p_p, unsigned * p_bit, unsigned width,
                 uint32_t value)
{
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        p_p[1U + *p_bit / 8U] |=
            (uint8_t)(((value >> value_i) & 1U) << (*p_bit % 8U));
    }
}

/** @brief Read a little-bit-order field following the version byte.
 * @param p_p Readable complete packet; this helper does not validate it.
 * @param p_bit Non-null body-bit cursor; cursor plus width is at most 551.
 * @param width Number of bits, from 1 through 32.
 * @return The unsigned field value; advances p_bit by width.
 * @note Private helper with caller-owned storage and no synchronization.
 */
static uint32_t get (const uint8_t * p_p, unsigned * p_bit, unsigned width)
{
    uint32_t value_v = 0;
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        value_v |= (uint32_t)((p_p[1U + *p_bit / 8U] >> (*p_bit % 8U)) & 1U)
                   << value_i;
    }
    return value_v;
}

/** @brief Test the unsigned schema range without overflowing at 32 bits.
 * @param value Full-width public input, including invalid sentinels.
 * @param width Schema width from 1 through 32 bits.
 * @return True exactly when value is below two to the power width.
 * @note Pure calculation; no caller output or shared state is changed.
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
 * @note Uses unchanged int64_t intermediates; no state is changed.
 */
static bool signed_fits (int32_t value, unsigned width)
{
    const int64_t limit = INT64_C(1) << (width - 1U);
    return ((int64_t)value >= -limit) && ((int64_t)value < limit);
}

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
                         control_debug_packet_t * p_packet)
{
    unsigned bit = 0;
    bool fields_valid = true;
    if (!p_sample || !p_packet || p_sample->motion_state > 5 ||
        p_sample->motion_fault > 5 || p_sample->drive_state > 6 ||
        p_sample->reason > 10 || p_sample->pending > 4 ||
        p_sample->barcode_status > 4 || p_sample->cal_sensor > 2 ||
        p_sample->line_error < -100 || p_sample->line_error > 100 ||
        p_sample->duty_left < -100 || p_sample->duty_left > 100 ||
        p_sample->duty_right < -100 || p_sample->duty_right > 100 ||
        p_sample->left_white_limit > 1000 ||
        p_sample->left_black_limit > 1000 ||
        p_sample->right_white_limit > 1000 ||
        p_sample->right_black_limit > 1000 ||
        p_sample->barcode_white_limit > 1000 ||
        p_sample->barcode_black_limit > 1000 ||
        (p_sample->motion_valid && !p_sample->motion_available))
    {
        return false;
    }
#define CD_U(name, width)                                                      \
    (fields_valid = fields_valid && unsigned_fits(p_sample->name, (width)));
#define CD_S(name, width)                                                      \
    (fields_valid = fields_valid && signed_fits(p_sample->name, (width)));
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    if (!fields_valid)
    {
        return false;
    }
    memset(p_packet, 0, sizeof *p_packet);
    p_packet->bytes[0] = 1;
#define CD_U(name, width)                                                      \
    put(p_packet->bytes, &bit, width, (uint32_t)p_sample->name);
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
 * @note Use the result only on true. No locks, callbacks or shared mutation;
 * the caller serializes access to its buffers.
 */
bool control_debug_unpack (const control_debug_packet_t * p_packet,
                           control_debug_sample_t * p_sample)
{
    unsigned bit = 0;
    control_debug_packet_t canonical;
    if (!p_packet || !p_sample || p_packet->bytes[0] != 1)
    {
        return false;
    }
#define CD_U(name, width) p_sample->name = get(p_packet->bytes, &bit, width);
#define CD_S(name, width)                                                      \
    {                                                                          \
        uint32_t value_bits = get(p_packet->bytes, &bit, width);               \
        p_sample->name =                                                       \
            (int32_t)(value_bits & ((UINT32_C(1) << (width - 1)) - 1U)) -      \
            (int32_t)(value_bits & (UINT32_C(1) << (width - 1)));              \
    }
    CONTROL_DEBUG_FIELDS(CD_U, CD_S)
#undef CD_U
#undef CD_S
    return control_debug_pack(p_sample, &canonical) &&
           memcmp(p_packet, &canonical, sizeof *p_packet) == 0;
}

/** Explicit bit positions describe protocol bytes, never native struct layout.
 */
typedef struct
{
    uint16_t bit;
    uint8_t width;
    bool signed_value;
} control_debug_field_info_t;

static const control_debug_field_info_t g_debug_fields[CD_FIELD_COUNT] = {
    {0U, 32U, false},   /* timestamp_us */
    {32U, 32U, false},  /* motion_timestamp_us */
    {64U, 8U, false},   /* sequence */
    {72U, 1U, false},   /* motion_available */
    {73U, 1U, false},   /* motion_valid */
    {74U, 1U, false},   /* braked */
    {75U, 1U, false},   /* busy */
    {76U, 3U, false},   /* motion_state */
    {79U, 3U, false},   /* motion_fault */
    {82U, 13U, true},   /* setpoint_left */
    {95U, 13U, true},   /* setpoint_right */
    {108U, 13U, true},  /* measured_left */
    {121U, 13U, true},  /* measured_right */
    {134U, 8U, true},   /* duty_left */
    {142U, 8U, true},   /* duty_right */
    {150U, 20U, true},  /* encoder_left */
    {170U, 20U, true},  /* encoder_right */
    {190U, 3U, false},  /* drive_state */
    {193U, 4U, false},  /* reason */
    {197U, 3U, false},  /* pending */
    {200U, 8U, true},   /* line_error */
    {208U, 5U, false},  /* line_flags */
    {213U, 3U, false},  /* barcode_status */
    {216U, 3U, false},  /* accepted_calibration */
    {219U, 5U, false},  /* readiness */
    {224U, 16U, false}, /* barcode_events */
    {240U, 16U, false}, /* junction_events */
    {256U, 16U, false}, /* lost_events */
    {272U, 32U, false}, /* sample_gap_us */
    {304U, 32U, false}, /* run_max_gap_us */
    {336U, 12U, false}, /* left_white */
    {348U, 12U, false}, /* left_black */
    {360U, 10U, false}, /* left_white_limit */
    {370U, 10U, false}, /* left_black_limit */
    {380U, 12U, false}, /* right_white */
    {392U, 12U, false}, /* right_black */
    {404U, 10U, false}, /* right_white_limit */
    {414U, 10U, false}, /* right_black_limit */
    {424U, 12U, false}, /* barcode_white */
    {436U, 12U, false}, /* barcode_black */
    {448U, 10U, false}, /* barcode_white_limit */
    {458U, 10U, false}, /* barcode_black_limit */
    {468U, 2U, false},  /* cal_sensor */
    {470U, 1U, false},  /* collecting */
    {471U, 12U, false}, /* white_min */
    {483U, 12U, false}, /* white_max */
    {495U, 12U, false}, /* black_min */
    {507U, 12U, false}, /* black_max */
    {519U, 16U, false}, /* white_count */
    {535U, 16U, false}, /* black_count */
};

/** Apply the same additional domains as the wide public pack interface. */
/** @brief Check unsigned semantic limits beyond the encoded field width.
 * @param field Schema selector already checked by the public setter.
 * @param value Full-width unsigned input; no narrowing has occurred.
 * @return True for the field's existing domain. Other selectors return true
 * here and rely on the caller's separate selector and width validation.
 * @note Pure helper; no output, builder state or shared state changes.
 */
static bool debug_unsigned_domain (control_debug_field_t field, uint32_t value)
{
    switch (field)
    {
    case CD_MOTION_STATE:
    case CD_MOTION_FAULT:
        return value <= 5U;
    case CD_DRIVE_STATE:
        return value <= 6U;
    case CD_REASON:
        return value <= 10U;
    case CD_PENDING:
    case CD_BARCODE_STATUS:
        return value <= 4U;
    case CD_CAL_SENSOR:
        return value <= 2U;
    case CD_LEFT_WHITE_LIMIT:
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
 * @note Pure helper; invalid selectors must be rejected by its caller first.
 */
static bool debug_signed_domain (control_debug_field_t field, int32_t value)
{
    switch (field)
    {
    case CD_LINE_ERROR:
    case CD_DUTY_LEFT:
    case CD_DUTY_RIGHT:
        return (value >= -100) && (value <= 100);
    default:
        return true;
    }
}

/** Replace a field, including clearing a previously populated zero value. */
/** @brief Replace exactly one validated field in the caller's packet.
 * @param p_builder Non-null, caller-owned initialized builder.
 * @param field Valid schema selector in 0..CD_FIELD_COUNT-1.
 * @param value Valid unsigned bits or explicitly encoded signed low bits.
 * @post changes only the selected bits and preserves other fields.
 * @note Unlike put, writing zero clears previous bits. Does not inspect or
 * restore the validity latch; public setters perform domain validation.
 */
static void debug_builder_write (control_debug_builder_t * p_builder,
                                 control_debug_field_t field, uint32_t value)
{
    const control_debug_field_info_t * p_info = &g_debug_fields[field];
    uint32_t index = 0U;
    for (index = 0U; index < p_info->width; ++index)
    {
        const uint32_t bit = p_info->bit + index;
        const uint32_t byte = 1U + (bit / 8U);
        const uint32_t mask = UINT32_C(1) << (bit % 8U);
        const uint32_t previous = p_builder->packet.bytes[byte];
        const uint32_t next =
            (previous & ~mask) | (((value >> index) & 1U) << (bit % 8U));
        /* The mask only changes one bit of an existing byte. */
        p_builder->packet.bytes[byte] = (uint8_t)next;
    }
}

/** @brief Reset caller-owned scratch to a valid version-1 zero sample.
 * @param p_builder Writable builder or null; null is a no-op.
 * @post clears all prior packet bits and the previous error latch.
 * @note No locks or callbacks. The caller exclusively owns the scratch object.
 */
void control_debug_builder_init (control_debug_builder_t * p_builder)
{
    if (NULL != p_builder)
    {
        (void)memset(p_builder, 0, sizeof(*p_builder));
        p_builder->packet.bytes[0] = 1U;
        p_builder->valid = true;
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
 * @note Caller serializes builder access. No publication or motor effect.
 */
void control_debug_builder_u (control_debug_builder_t * p_builder,
                              control_debug_field_t field, uint32_t value)
{
    if (NULL == p_builder)
    {
        return;
    }
    /* Converting a negative selector to unsigned also makes it out of range. */
    if (((uint32_t)field >= CD_FIELD_COUNT) ||
        g_debug_fields[field].signed_value ||
        !unsigned_fits(value, g_debug_fields[field].width) ||
        !debug_unsigned_domain(field, value))
    {
        p_builder->valid = false;
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
 * @note Caller serializes builder access. No locks, callbacks or publication.
 */
void control_debug_builder_s (control_debug_builder_t * p_builder,
                              control_debug_field_t field, int32_t value)
{
    if (NULL == p_builder)
    {
        return;
    }
    /* Converting a negative selector to unsigned also makes it out of range. */
    if (((uint32_t)field >= CD_FIELD_COUNT) ||
        !g_debug_fields[field].signed_value ||
        !signed_fits(value, g_debug_fields[field].width) ||
        !debug_signed_domain(field, value))
    {
        p_builder->valid = false;
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
 * @note Does not change the builder. The motion-availability relationship
 * failure is not latched, so changing that field can permit a later end.
 * The caller owns synchronization; this function itself publishes nothing.
 */
bool control_debug_builder_end (const control_debug_builder_t * p_builder,
                                control_debug_packet_t * p_packet)
{
    unsigned int available_bit = g_debug_fields[CD_MOTION_AVAILABLE].bit;
    unsigned int valid_bit = g_debug_fields[CD_MOTION_VALID].bit;
    if ((NULL == p_builder) || (NULL == p_packet) || !p_builder->valid)
    {
        return false;
    }
    if ((0U != get(p_builder->packet.bytes, &valid_bit, 1U)) &&
        (0U == get(p_builder->packet.bytes, &available_bit, 1U)))
    {
        return false;
    }
    *p_packet = p_builder->packet;
    return true;
}

/* End of file. */

