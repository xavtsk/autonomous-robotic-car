/** @file telemetry_codec.c
 * @brief Encode and decode bounded binary telemetry payloads.
 * @details The topic selects the packet length and field layout. A
 * version byte precedes the explicitly packed fields.
 * @note Byte and half-word operands fit the target's int and uint32_t
 * values fit int64_t. Masks and nonnegative enum bounds use unsigned
 * arithmetic. Unsigned conversions of signed wire fields keep the low
 * bits written to the packet.
 * @see app_program/subsystem1/MQTT_PROTOCOL.md for layout and validity.
 */
#include "telemetry_codec.h"
#include "../build_assert.h"

/* Compile-time guarantee: outbox topic selector is four bits. */
BUILD_ASSERT((unsigned)BUDDY1_TOPIC_COUNT <= 16U, telemetry_codec_c_check_b_t);

/* Field widths exclude the one-byte protocol version. The MQTT topic selects
 * the field layout documented in app_program/subsystem1/MQTT_PROTOCOL.md. */
/* Compile-time guarantee: fault mask and declared width must agree. */
BUILD_ASSERT(BUDDY1_FAULT_MASK == (1U << BUDDY1_FAULT_BITS) - 1U,
             telemetry_codec_c_check_a_t);
static const uint8_t g_body_bits[BUDDY1_TOPIC_COUNT] = {
    18,
    8,
    25,
    9,
    26,
    3,
    BUDDY1_FAULT_BITS,
    96U + BUDDY1_FAULT_BITS + 1U + BUDDY1_POSE_DISTANCE_BITS + 9U,
    2,
    2U + BUDDY1_FAULT_BITS,
    BUDDY1_POSE_DISTANCE_BITS + 9U,
    1U + (2U * BUDDY1_TILT_BITS) + 3U + 1U + 9U + 1U + 9U + 1U +
        BUDDY1_TURN_BITS + 2U,
    1U + 1U + 12U + 8U + 3U + 3U + 2U + 4U + 3U + 1U,
    1U + 8U + (5U * 12U) + (2U * 13U),
    8U + 1U + 4U + 4U + 5U + 1U + 12U};
static const char * const g_topics[BUDDY1_TOPIC_COUNT] = {
    "car/telemetry/motion",     "car/telemetry/barcode",
    "car/telemetry/hump",       "car/telemetry/heading",
    "car/telemetry/obstacle",   "car/telemetry/state",
    "car/telemetry/faults",     "car/reply/snapshot",
    "car/reply/barcode_map",    "car/status",
    "car/telemetry/pose",       "car/telemetry/terrain",
    "car/telemetry/avoidance",  "car/telemetry/scan",
    "car/telemetry/scan_sample"};
static void     write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                            unsigned width);
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width);
static unsigned validity (const telemetry_codec_snapshot_t * p_view);
static int      write_fields (uint8_t                            topic,
                              const telemetry_codec_snapshot_t * p_view,
                              uint8_t * p_body, unsigned * p_bit);
static int read_fields (uint8_t topic, const uint8_t * p_body, unsigned * p_bit,
                        telemetry_codec_snapshot_t * p_view);

/** @brief Include version and rounded-up body bytes in a topic's wire size.
 * @see telemetry_codec.h for selector validation and MAP_REPLY behavior.
 */
unsigned telemetry_codec_payload_size (uint8_t topic)
{
    return (int)topic < BUDDY1_TOPIC_COUNT
               ? 1U + ((g_body_bits[topic] + 7U) / 8U)
               : 0U;
}

/** @brief Include the two 12-bit face-width bounds when sizing a v4 scan.
 * @see telemetry_codec.h for supported selectors and failure behaviour.
 */
unsigned telemetry_codec_version_size (uint8_t topic, uint8_t version)
{
    const unsigned wire_version = version;
    if ((BUDDY1_TOPIC_SCAN == topic) &&
        (BUDDY1_SCAN_FACE_VERSION == wire_version))
    {
        return telemetry_codec_payload_size(topic) + 3U;
    }
    return (BUDDY1_WIRE_VERSION == wire_version)
               ? telemetry_codec_payload_size(topic)
               : 0U;
}

/** @brief Look up the meaningful body bit count without version or padding.
 * @see telemetry_codec.h for the unknown-selector return value.
 */
unsigned telemetry_codec_payload_bits (uint8_t topic)
{
    return (int)topic < BUDDY1_TOPIC_COUNT ? g_body_bits[topic] : 0U;
}

/** @brief Select the immutable MQTT topic string for one wire layout.
 * @see telemetry_codec.h for pointer lifetime and invalid selectors.
 */
const char * telemetry_codec_topic_name (uint8_t topic)
{
    return (int)topic < BUDDY1_TOPIC_COUNT ? g_topics[topic] : (const char *)0;
}

/** @brief Clear and fill a versioned packet from selected telemetry fields.
 * @param[in] topic Known selector other than MAP_REPLY.
 * @param[in] p_value Stable decoded view, disjoint from packet storage.
 * @param[out] p_packet Writable capacity-byte buffer; may be partly filled
 * when field validation fails after the initial argument checks.
 * @param[in] capacity Available byte count.
 * @return Encoded length on success or zero on failure; a zero return does
 * not promise unchanged packet bytes.
 * @see telemetry_codec.h for complete ranges, precision and validity behavior.
 */
unsigned buddy1_encode (uint8_t                            topic,
                        const telemetry_codec_snapshot_t * p_value,
                        uint8_t * p_packet, unsigned capacity)
{
    unsigned      bit = 0;
    unsigned      index;
    unsigned      flags;
    const uint8_t version =
        (((BUDDY1_TOPIC_SCAN == topic) && (0 != p_value)) &&
         (p_value->scan.face_width_min_mm || p_value->scan.face_width_max_mm))
            ? BUDDY1_SCAN_FACE_VERSION
            : BUDDY1_WIRE_VERSION;
    const unsigned length = telemetry_codec_version_size(topic, version);

    if (((((0 == p_value) || (0 == p_packet)) || (0U == length)) ||
         (capacity < length)) ||
        (BUDDY1_TOPIC_MAP_REPLY == (int)topic))
    {
        return 0;
    }

    for (index = 0; index < length; ++index)
    {
        p_packet[index] = 0;
    }

    p_packet[0] = version;

    switch (topic)
    {
    case BUDDY1_TOPIC_SNAPSHOT:
        flags = validity(p_value);
        write_bits(p_packet + 1, &bit, flags, 7);
        /* The section loop stays within the seven original topic IDs,
         * so each uint8_t selector retains the full index. Invalid sections
         * have canonical zero bits. Zero height remains a valid value. */
        for (index = 0; index <= (unsigned int)BUDDY1_TOPIC_FAULTS; ++index)
        {
            if ((flags & (1U << index)) != 0U)
            {
                if (!write_fields((uint8_t)index, p_value, p_packet + 1, &bit))
                {
                    return 0;
                }
            }
            else
            {
                write_bits(p_packet + 1, &bit, 0, g_body_bits[index]);
            }
        }
        /* 19 Nov 2026: distance/heading since START appended after the
         * original seven sections (bits 102-123), with its own validity bit. */
        write_bits(p_packet + 1, &bit, p_value->pose_valid != 0U, 1);

        if (p_value->pose_valid)
        {
            if (!write_fields(BUDDY1_TOPIC_POSE, p_value, p_packet + 1, &bit))
            {
                return 0;
            }
        }
        else
        {
            write_bits(p_packet + 1, &bit, 0, g_body_bits[BUDDY1_TOPIC_POSE]);
        }
    break;
    case BUDDY1_TOPIC_STATUS:
        if ((p_value->faults_valid) &&
            ((p_value->faults & ~BUDDY1_FAULT_MASK) != 0U))
        {
            return 0;
        }

        write_bits(p_packet + 1, &bit, 1,
                   1); /* The heartbeat confirms the sender is alive. */
        write_bits(p_packet + 1, &bit, p_value->faults_valid != 0U, 1);
        write_bits(p_packet + 1, &bit,
                   p_value->faults_valid ? p_value->faults : 0U,
                   BUDDY1_FAULT_BITS);
    break;
    default:
        if (!write_fields(topic, p_value, p_packet + 1, &bit))
        {
            return 0;
        }

    break;
    }

    /* The version byte widens exactly for comparison with wire constants. */
    if (BUDDY1_SCAN_FACE_VERSION == (unsigned)version)
    {
        const telemetry_codec_scan_t * p_scan   = &p_value->scan;
        const unsigned                 upper_mm = p_scan->face_width_max_mm;
        const unsigned                 lower_mm = p_scan->face_width_min_mm;
        if (((!p_scan->valid) || (upper_mm > 4095U)) || (lower_mm > upper_mm))
        {
            return 0;
        }
        /* The extension starts after the base scan's reserved padding bit. */
        uint8_t * p_bounds = p_packet + telemetry_codec_payload_size(topic);
        bit                = 0U;
        write_bits(p_bounds, &bit, p_scan->face_width_min_mm, 12U);
        write_bits(p_bounds, &bit, p_scan->face_width_max_mm, 12U);
    }
    return length;
}

/** @brief Validate a complete payload and publish only a fully decoded view.
 * @param[in] topic Wire layout selector in a byte, representable as int.
 * @param[in] p_packet Borrowed packet, readable for length bytes.
 * @param[in] length Exact packet length in bytes for the selected layout.
 * @param[out] p_out Writable decoded view. Rejection leaves it unchanged.
 * @details Check the version and trailing padding before reading the body.
 * A snapshot begins with seven validity bits. Decode its sections in topic
 * order and require zero bits wherever a section is absent. Pose has a
 * separate validity bit. A status packet requires the heartbeat bit and
 * may carry fault bits only when its fault-valid bit is set. Decode into
 * a local view and copy it to the caller after every check has succeeded.
 * @return One on success or zero on rejection.
 * @see telemetry_codec.h for canonical encoding and unchanged-on-failure
 * output.
 */
int buddy1_decode (uint8_t topic, const uint8_t * p_packet, unsigned length,
                   telemetry_codec_snapshot_t * p_out)
{
    telemetry_codec_snapshot_t decoded = {0};
    unsigned                   bit     = 0;
    unsigned                   topic_index;
    unsigned                   flags;

    if (((0 == p_packet) || (0 == p_out)) || (0U == length))
    {
        return 0;
    }

    if ((BUDDY1_TOPIC_MAP_REPLY == topic) ||
        (length != telemetry_codec_version_size(topic, p_packet[0])))
    {
        return 0;
    }

    /* Widen wire bytes (0..255) before unsigned arithmetic and shifting. */
    const unsigned remainder = (unsigned)g_body_bits[topic] % 8U;
    if ((0U != remainder) &&
        (0U != ((unsigned)p_packet[telemetry_codec_payload_size(topic) - 1U] >>
                remainder)))
    {
        return 0;
    }

    switch (topic)
    {
    case BUDDY1_TOPIC_SNAPSHOT:
        /* The seven-bit validity mask fits unsigned, and the section
         * indices below fit uint8_t. */
        flags = (unsigned)read_bits(p_packet + 1, &bit, 7);

        for (topic_index = 0; topic_index <= (unsigned int)BUDDY1_TOPIC_FAULTS;
             ++topic_index)
        {
            if ((flags & (1U << topic_index)) != 0U)
            {
                if (!read_fields((uint8_t)topic_index, p_packet + 1, &bit,
                                 &decoded))
                {
                    return 0;
                }
            }
            else
            {
                /* At most 34 bits, so consume one bit at a time. */
                unsigned bit_index;

                for (bit_index = 0; bit_index < g_body_bits[topic_index];
                     ++bit_index)
                {
                    if (read_bits(p_packet + 1, &bit, 1) != 0U)
                    {
                        return 0;
                    }
                }
            }
        }

        if (read_bits(p_packet + 1, &bit, 1) != 0U)
        {
            if (!read_fields(BUDDY1_TOPIC_POSE, p_packet + 1, &bit, &decoded))
            {
                return 0;
            }
        }
        else
        {
            unsigned bit_index;

            for (bit_index = 0; bit_index < g_body_bits[BUDDY1_TOPIC_POSE];
                 ++bit_index)
            {
                if (read_bits(p_packet + 1, &bit, 1) != 0U)
                {
                    return 0;
                }
            }
        }
    break;
    case BUDDY1_TOPIC_STATUS:
        if (read_bits(p_packet + 1, &bit, 1) != 1U)
        {
            return 0;
        }

        /* One validity bit and six fault bits fit their byte destinations. */
        /* clang-format off */
        decoded.faults_valid = (uint8_t)read_bits(p_packet + 1, &bit, 1);
        decoded.faults       =
            (uint8_t)read_bits(p_packet + 1, &bit, BUDDY1_FAULT_BITS);
        /* clang-format on */

        if ((!decoded.faults_valid) && (decoded.faults))
        {
            return 0;
        }
    break;
    default:
        if (!read_fields(topic, p_packet + 1, &bit, &decoded))
        {
            return 0;
        }

    break;
    }

    /* An unsigned conversion preserves every possible wire version byte. */
    if (BUDDY1_SCAN_FACE_VERSION == (unsigned)p_packet[0])
    {
        const uint8_t * p_bounds =
            p_packet + telemetry_codec_payload_size(topic);
        bit = 0U;
        /* Each 12-bit read is in 0..4095 and fits its uint16_t result. */
        decoded.scan.face_width_min_mm =
            (uint16_t)read_bits(p_bounds, &bit, 12U);
        decoded.scan.face_width_max_mm =
            (uint16_t)read_bits(p_bounds, &bit, 12U);
        if (((!decoded.scan.valid) || (!decoded.scan.face_width_max_mm)) ||
            (decoded.scan.face_width_min_mm > decoded.scan.face_width_max_mm))
        {
            return 0;
        }
    }
    *p_out = decoded;
    return 1;
}

/** @brief Validate reply bytes directly or other payloads through the decoder.
 * @see telemetry_codec.h for accepted versions, lengths and reply codes.
 */
int telemetry_codec_payload_valid (uint8_t topic, const uint8_t * p_packet,
                                   unsigned length)
{
    telemetry_codec_snapshot_t decoded;

    if (BUDDY1_TOPIC_MAP_REPLY == (int)topic)
    {
        return ((p_packet && (2U == length)) &&
                (BUDDY1_WIRE_VERSION == p_packet[0])) &&
               ((int)p_packet[1] <= BUDDY1_REPLY_INTERNAL);
    }

    return buddy1_decode(topic, p_packet, length, &decoded);
}

/** @brief Validate all four distinct letters before writing the map packet.
 * @see telemetry_codec.h for action order, bit layout and failure behavior.
 */
int buddy1_map_pack (const telemetry_codec_map_t * p_map,
                     uint8_t                       p_packet[BUDDY1_MAP_BYTES])
{
    uint8_t  letters[4];
    unsigned letter_index;
    unsigned earlier_index;
    unsigned bit = 0;

    if ((0 == p_map) || (!p_packet))
    {
        return 0;
    }

    letters[0] = p_map->forward;
    letters[1] = p_map->left;
    letters[2] = p_map->right;
    letters[3] = p_map->uturn;

    for (letter_index = 0; letter_index < sizeof letters; ++letter_index)
    {
        if (((int)letters[letter_index] < 'A') ||
            ((int)letters[letter_index] > 'Z'))
        {
            return 0;
        }

        for (earlier_index = 0; earlier_index < letter_index; ++earlier_index)
        {
            if (letters[letter_index] == letters[earlier_index])
            {
                return 0;
            }
        }
    }

    p_packet[0] = BUDDY1_WIRE_VERSION;
    p_packet[1] = p_packet[2] = p_packet[3] = 0;

    /* Validation leaves A..Z, so subtracting A gives 0..25 before
     * conversion to uint32_t and packing into five bits. */
    for (letter_index = 0; letter_index < sizeof letters; ++letter_index)
    {
        write_bits(p_packet + 1, &bit,
                   (uint32_t)((int)letters[letter_index] - 'A'), 5);
    }

    return 1;
}

/** @brief Validate padding and a local decoded map before publishing it.
 * @see telemetry_codec.h for map domains and caller ownership.
 */
int telemetry_codec_map_unpack (const uint8_t * p_packet, unsigned length,
                                telemetry_codec_map_t * p_out)
{
    telemetry_codec_map_t map;
    uint8_t               canonical[BUDDY1_MAP_BYTES];
    unsigned              bit = 0;

    if (((((0 == p_packet) || (0 == p_out)) || (length != BUDDY1_MAP_BYTES)) ||
         (p_packet[0] != BUDDY1_WIRE_VERSION)) ||
        ((p_packet[3] & 0xf0U) != 0U))
    {
        return 0;
    }

    /* A five-bit wire ordinal plus ASCII A fits a byte, including the
     * six out-of-alphabet values checked below. */
    /* clang-format off */
    map.forward =
        (uint8_t)((unsigned int)'A' + read_bits(p_packet + 1, &bit, 5));
    map.left    =
        (uint8_t)((unsigned int)'A' + read_bits(p_packet + 1, &bit, 5));
    map.right   =
        (uint8_t)((unsigned int)'A' + read_bits(p_packet + 1, &bit, 5));
    map.uturn   =
        (uint8_t)((unsigned int)'A' + read_bits(p_packet + 1, &bit, 5));
    /* clang-format on */

    if (!buddy1_map_pack(&map, canonical))
    {
        return 0;
    }

    *p_out = map;
    return 1;
}

/** @brief Append a low-bit-first field into zeroed packet storage.
 * @param[in,out] p_bytes Writable storage covering the field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] value Low 32 value bits; bits beyond 32 are treated as zero.
 * @param[in] width Number of field bits, including zero-filled wide sections.
 * @pre Pointers/capacity are valid and all destination field bits are zero.
 * @post Set bits are ORed into storage. Existing bits are never cleared.
 */
static void write_bits (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                        unsigned width)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if ((bit_offset < 32U) &&
            ((value & ((uint32_t)1U << bit_offset)) != 0U))
        {
            p_bytes[*p_bit / 8U] = (uint8_t)((unsigned)p_bytes[*p_bit / 8U] |
                                             (1U << (*p_bit % 8U)));
        }

        ++bit_offset;
        ++*p_bit;
    }
}

/** @brief Read an unsigned low-bit-first packet field.
 * @param[in] p_bytes Stable readable storage covering the field.
 * @param[in,out] p_bit Non-null cursor; advanced by width bits.
 * @param[in] width Field width, 0..32 bits.
 * @return The decoded value, or zero for a zero-width field.
 * @pre Pointers and bounds are valid; this helper does not validate them.
 */
static uint32_t read_bits (const uint8_t * p_bytes, unsigned * p_bit,
                           unsigned width)
{
    uint32_t value      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if ((p_bytes[*p_bit / 8U] & (1U << (*p_bit % 8U))) != 0U)
        {
            value |= (uint32_t)1U << bit_offset;
        }

        ++bit_offset;
        ++*p_bit;
    }

    return value;
}

/** @brief Form the original seven-section snapshot validity mask.
 * @param[in] p_view Non-null stable decoded view; each nonzero flag means
 * valid.
 * @return Bits for motion, barcode, hump, heading, obstacle, state and faults,
 * in that order. Pose has a separate bit and is not included here.
 */
static unsigned validity (const telemetry_codec_snapshot_t * p_view)
{
    return (p_view->motion_valid != 0U ? 1U : 0U) |
           (p_view->barcode_valid != 0U ? (1U << 1U) : 0U) |
           (p_view->hump_valid != 0U ? (1U << 2U) : 0U) |
           (p_view->heading_valid != 0U ? (1U << 3U) : 0U) |
           (p_view->obstacle_valid != 0U ? (1U << 4U) : 0U) |
           (p_view->robot_state_valid != 0U ? (1U << 5U) : 0U) |
           (p_view->faults_valid != 0U ? (1U << 6U) : 0U);
}

/** @brief Validate and append one telemetry section without its valid bit.
 * @param[in] topic A supported single-section selector; SNAPSHOT, STATUS,
 * MAP_REPLY and unknown selectors are rejected.
 * @param[in] p_view Non-null stable decoded values; top-level validity is
 * ignored.
 * @param[in,out] p_body Packet body with enough zeroed destination bits.
 * @param[in,out] p_bit Non-null body cursor; advanced only on success.
 * @return One after writing the whole section; zero before any section bits
 * are written if its fields or selector are invalid.
 * @details Domains and units follow buddy1_encode. Only motion is quantized:
 * nearest 2 mm/s, ties away from zero. Other fields retain integer precision.
 * @pre Caller has checked pointers, capacity and disjoint input/output storage.
 */
static int write_fields (uint8_t                            topic,
                         const telemetry_codec_snapshot_t * p_view,
                         uint8_t * p_body, unsigned * p_bit)
{
    int32_t left;
    int32_t right;

    switch (topic)
    {
    case BUDDY1_TOPIC_MOTION:
        left  = p_view->left_mms;
        right = p_view->right_mms;

        if ((((left < -BUDDY1_SPEED_MAX_MMS) ||
              (left > BUDDY1_SPEED_MAX_MMS)) ||
             (right < -BUDDY1_SPEED_MAX_MMS)) ||
            (right > BUDDY1_SPEED_MAX_MMS))
        {
            return 0;
        }
        /* Nearest 2 mm/s, half-way cases away from zero: error <= 1 mm/s.
         * Conversion affects telemetry only, never Buddy 2's control values. */
        write_bits(p_body, p_bit,
                   (uint32_t)((left < 0 ? -left : left) + 1) / 2U, 8);
        write_bits(p_body, p_bit, (uint32_t)(left < 0), 1);
        write_bits(p_body, p_bit,
                   (uint32_t)((right < 0 ? -right : right) + 1) / 2U, 8);
        write_bits(p_body, p_bit, (uint32_t)(right < 0), 1);
        return 1;
    case BUDDY1_TOPIC_BARCODE:
        if (((((int)p_view->barcode_letter < 'A') ||
              ((int)p_view->barcode_letter > 'Z')) ||
             (p_view->barcode_action < (unsigned int)BUDDY1_ACTION_UNKNOWN)) ||
            (p_view->barcode_action > (unsigned int)BUDDY1_ACTION_UTURN))
        {
            return 0;
        }
        write_bits(p_body, p_bit, (uint32_t)((int)p_view->barcode_letter - 'A'),
                   5);
        write_bits(p_body, p_bit, (uint32_t)p_view->barcode_action, 3);
        return 1;
    case BUDDY1_TOPIC_HUMP:
        if (p_view->hump_height_mm > BUDDY1_HUMP_HEIGHT_MAX_MM)
        {
            return 0;
        }
        write_bits(p_body, p_bit, p_view->hump_count, 16);
        write_bits(p_body, p_bit, p_view->hump_height_mm, 9);
        return 1;
    case BUDDY1_TOPIC_HEADING:
        if ((p_view->heading_deg < 0) || (p_view->heading_deg > 359))
        {
            return 0;
        }
        write_bits(p_body, p_bit, (uint32_t)p_view->heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_OBSTACLE:
        if (((((p_view->obstacle_distance_mm < BUDDY1_OBSTACLE_MIN_MM) ||
               (p_view->obstacle_distance_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
              (p_view->obstacle_width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM)) ||
             (p_view->obstacle_route < (unsigned int)BUDDY1_ROUTE_UNKNOWN)) ||
            (p_view->obstacle_route > (unsigned int)BUDDY1_ROUTE_RIGHT))
        {
            return 0;
        }
        write_bits(p_body, p_bit, p_view->obstacle_distance_mm, 12);
        write_bits(p_body, p_bit, p_view->obstacle_width_mm, 12);
        write_bits(p_body, p_bit, (uint32_t)p_view->obstacle_route, 2);
        return 1;
    case BUDDY1_TOPIC_STATE:
        if ((p_view->robot_state < (unsigned int)BUDDY1_STATE_STOPPED) ||
            (p_view->robot_state > (unsigned int)BUDDY1_STATE_ERROR))
        {
            return 0;
        }
        write_bits(p_body, p_bit,
                   (uint32_t)(p_view->robot_state -
                              (unsigned int)BUDDY1_STATE_STOPPED),
                   3);
        return 1;
    case BUDDY1_TOPIC_FAULTS:
        if ((p_view->faults & ~BUDDY1_FAULT_MASK) != 0U)
        {
            return 0;
        }
        write_bits(p_body, p_bit, p_view->faults, BUDDY1_FAULT_BITS);
        return 1;
    case BUDDY1_TOPIC_POSE:
        if (((p_view->pose_distance_cm >= BUDDY1_POSE_DISTANCE_MOD) ||
             (p_view->pose_heading_deg < 0)) ||
            (p_view->pose_heading_deg > 359))
        {
            return 0;
        }
        write_bits(p_body, p_bit, p_view->pose_distance_cm,
                   BUDDY1_POSE_DISTANCE_BITS);
        write_bits(p_body, p_bit, (uint32_t)p_view->pose_heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_TERRAIN:
    {
        const telemetry_codec_terrain_t * p_terrain = &p_view->terrain;

        if (((p_terrain->state >= BUDDY1_TERRAIN_STATES) ||
             (p_terrain->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM)) ||
            (p_terrain->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM))
        {
            return 0;
        }

        if (p_terrain->tilt_valid)
        {
            if ((((p_terrain->pitch_tenths_deg < -BUDDY1_TILT_LIMIT_X10) ||
                  (p_terrain->pitch_tenths_deg > BUDDY1_TILT_LIMIT_X10)) ||
                 (p_terrain->roll_tenths_deg < -BUDDY1_TILT_LIMIT_X10)) ||
                (p_terrain->roll_tenths_deg > BUDDY1_TILT_LIMIT_X10))
            {
                return 0;
            }
        }
        else if ((p_terrain->pitch_tenths_deg != 0) ||
                 (p_terrain->roll_tenths_deg != 0))
        {
            return 0; /* Represent an unavailable field with zero. */
        }
        else
        {
            /* Absent tilt has the required zero angles. */
        }

        if (((!p_terrain->on_hump) && (p_terrain->climb_mm != 0U)) ||
            ((!p_terrain->peak_valid) && (p_terrain->peak_mm != 0U)))
        {
            return 0;
        }

        if (p_terrain->turn_valid
                ? ((p_terrain->turn_dps < -BUDDY1_TURN_LIMIT_DPS) ||
                   (p_terrain->turn_dps > BUDDY1_TURN_LIMIT_DPS))
                : p_terrain->turn_dps != 0)
        {
            return 0;
        }

        if (p_terrain->safety > BUDDY1_SAFETY_OVERTURNED)
        {
            return 0;
        }

        write_bits(p_body, p_bit, p_terrain->tilt_valid != 0U, 1);
        /* Two's complement in 11 bits. */
        write_bits(p_body, p_bit,
                   (uint32_t)(int32_t)p_terrain->pitch_tenths_deg & 0x7FFU,
                   BUDDY1_TILT_BITS);
        write_bits(p_body, p_bit,
                   (uint32_t)(int32_t)p_terrain->roll_tenths_deg & 0x7FFU,
                   BUDDY1_TILT_BITS);
        write_bits(p_body, p_bit, p_terrain->state, 3);
        write_bits(p_body, p_bit, p_terrain->on_hump != 0U, 1);
        write_bits(p_body, p_bit, p_terrain->climb_mm, 9);
        write_bits(p_body, p_bit, p_terrain->peak_valid != 0U, 1);
        write_bits(p_body, p_bit, p_terrain->peak_mm, 9);
        write_bits(p_body, p_bit, p_terrain->turn_valid != 0U, 1);
        write_bits(p_body, p_bit,
                   (uint32_t)(int32_t)p_terrain->turn_dps & 0x3FFU,
                   BUDDY1_TURN_BITS);
        write_bits(p_body, p_bit, p_terrain->safety, 2);
        return 1;
    }
    case BUDDY1_TOPIC_AVOIDANCE:
    {
        const telemetry_codec_avoidance_t * p_avoidance = &p_view->avoidance;

        if ((((((((((((p_avoidance->range_mm > BUDDY1_OBSTACLE_MAX_MM) ||
                      ((p_avoidance->range_valid) &&
                       ((p_avoidance->range_mm < BUDDY1_OBSTACLE_MIN_MM) ||
                        (p_avoidance->no_echo)))) ||
                     ((!p_avoidance->range_valid) &&
                      (p_avoidance->range_mm))) ||
                    (p_avoidance->servo_deg > BUDDY1_SERVO_MAX_DEG)) ||
                   (p_avoidance->state > BUDDY1_S5_STATE_MAX)) ||
                  (p_avoidance->fault > 7U)) ||
                 ((int)p_avoidance->route > BUDDY1_ROUTE_RIGHT)) ||
                (p_avoidance->leg > 15U)) ||
               (p_avoidance->pass > BUDDY1_S5_PASS_MAX)) ||
              (p_avoidance->swapped > 1U)) ||
             (p_avoidance->range_valid > 1U)) ||
            (p_avoidance->no_echo > 1U))
        {
            return 0;
        }

        write_bits(p_body, p_bit, p_avoidance->range_valid, 1);
        write_bits(p_body, p_bit, p_avoidance->no_echo, 1);
        write_bits(p_body, p_bit, p_avoidance->range_mm, 12);
        write_bits(p_body, p_bit, p_avoidance->servo_deg, 8);
        write_bits(p_body, p_bit, p_avoidance->state, 3);
        write_bits(p_body, p_bit, p_avoidance->fault, 3);
        write_bits(p_body, p_bit, p_avoidance->route, 2);
        write_bits(p_body, p_bit, p_avoidance->leg, 4);
        write_bits(p_body, p_bit, p_avoidance->pass, 3);
        write_bits(p_body, p_bit, p_avoidance->swapped, 1);
        return 1;
    }
    case BUDDY1_TOPIC_SCAN:
    {
        const telemetry_codec_scan_t * p_scan = &p_view->scan;

        if ((((((((((p_scan->closest_mm > BUDDY1_OBSTACLE_MAX_MM) ||
                    (p_scan->width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM)) ||
                   (p_scan->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
                  (p_scan->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
                 (p_scan->forward_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
                (p_scan->left_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM)) ||
               (p_scan->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM)) ||
              (p_scan->right_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM)) ||
             (p_scan->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM)) ||
            (p_scan->valid > 1U))
        {
            return 0;
        }

        if (p_scan->valid ? ((p_scan->closest_mm < BUDDY1_OBSTACLE_MIN_MM) ||
                             (p_scan->left_edge_mm < p_scan->right_edge_mm))
                          : (((((((p_scan->closest_mm) || (p_scan->width_mm)) ||
                                 (p_scan->left_clear_mm)) ||
                                (p_scan->right_clear_mm)) ||
                               (p_scan->forward_mm)) ||
                              (p_scan->left_edge_mm)) ||
                             (p_scan->right_edge_mm)))
        {
            return 0;
        }

        write_bits(p_body, p_bit, p_scan->valid, 1);
        write_bits(p_body, p_bit, p_scan->scan_id, 8);
        write_bits(p_body, p_bit, p_scan->closest_mm, 12);
        write_bits(p_body, p_bit, p_scan->width_mm, 12);
        write_bits(p_body, p_bit, p_scan->left_clear_mm, 12);
        write_bits(p_body, p_bit, p_scan->right_clear_mm, 12);
        write_bits(p_body, p_bit, p_scan->forward_mm, 12);
        /* Checked signed edges are Ã‚Â±4000 mm. Adding the bias gives
         * unsigned codes from 0 to 8000, which fit thirteen bits. */
        write_bits(p_body, p_bit,
                   (uint32_t)(p_scan->left_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM),
                   13);
        write_bits(p_body, p_bit,
                   (uint32_t)(p_scan->right_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM),
                   13);
        return 1;
    }
    case BUDDY1_TOPIC_SCAN_SAMPLE:
    {
        const telemetry_codec_scan_sample_t * p_sample = &p_view->scan_sample;

        if (((((((((p_sample->peek > 1U) || (p_sample->valid > 1U)) ||
                  (p_sample->count < 1U)) ||
                 (p_sample->count > BUDDY1_SCAN_SAMPLES_MAX)) ||
                (p_sample->index >= p_sample->count)) ||
               (p_sample->angle_deg < 30U)) ||
              (p_sample->angle_deg > 150U)) ||
             (((p_sample->angle_deg - 30U) % 5U) != 0U)) ||
            (p_sample->valid
                 ? ((p_sample->distance_mm < BUDDY1_OBSTACLE_MIN_MM) ||
                    (p_sample->distance_mm > BUDDY1_OBSTACLE_MAX_MM))
                 : p_sample->distance_mm != 0U))
        {
            return 0;
        }

        write_bits(p_body, p_bit, p_sample->scan_id, 8);
        write_bits(p_body, p_bit, p_sample->peek, 1);
        write_bits(p_body, p_bit, p_sample->index, 4);
        /* Rebase the checked count and angle to their zero-based wire
         * ranges before writing the four- and five-bit fields. */
        write_bits(p_body, p_bit, (uint32_t)p_sample->count - 1U, 4);
        write_bits(p_body, p_bit, (uint32_t)(p_sample->angle_deg - 30U) / 5U,
                   5);
        write_bits(p_body, p_bit, p_sample->valid, 1);
        write_bits(p_body, p_bit, p_sample->distance_mm, 12);
        return 1;
    }
    default:
        return 0;
    }
}

/** @brief Decode and validate one telemetry section into temporary state.
 * @param[in] topic Supported single-section selector; wrappers are rejected.
 * @param[in] p_body Stable readable packet body containing the whole section.
 * @param[in,out] p_bit Non-null cursor; consumed bits remain advanced on error.
 * @param[in,out] p_view Non-null temporary decoded view; selected fields and
 * validity may be partially changed even when validation fails.
 * @return One for an accepted section, zero for invalid fields or selector.
 * @details Units and legal values follow buddy1_decode. The wire widths keep
 * unsigned octet fields within 0..255 and distances or counts within
 * 0..65535 before validation. Signed speeds are within -510..510 mm/s.
 * Unsigned nine-bit headings fit int16_t even if they exceed 359 degrees.
 * Negative zero in motion is rejected. The caller checks packet padding and
 * does not publish rejected input.
 * @pre Pointers and capacity are valid. This helper never commits public
 * output.
 */
static int read_fields (uint8_t topic, const uint8_t * p_body, unsigned * p_bit,
                        telemetry_codec_snapshot_t * p_view)
{
    uint32_t mag;
    uint32_t negative;

    switch (topic)
    {
    case BUDDY1_TOPIC_MOTION:
        mag      = read_bits(p_body, p_bit, 8);
        negative = read_bits(p_body, p_bit, 1);

        if ((0U == mag) && (negative != 0U))
        {
            return 0; /* Reject negative zero. */
        }

        p_view->left_mms = (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));
        mag              = read_bits(p_body, p_bit, 8);
        negative         = read_bits(p_body, p_bit, 1);

        if ((0U == mag) && (negative != 0U))
        {
            return 0;
        }

        p_view->right_mms =
            (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));

        p_view->motion_valid = 1;
        return 1;
    case BUDDY1_TOPIC_BARCODE:
        /* Adding a five-bit ordinal to ASCII A fits an octet. The check
         * below rejects the six ordinals beyond Z. */
        p_view->barcode_letter =
            (uint8_t)((unsigned int)'A' + read_bits(p_body, p_bit, 5));
        /* clang-format off */
        p_view->barcode_action =
            (telemetry_codec_action_t)read_bits(p_body, p_bit, 3);
        p_view->barcode_valid  = 1;
        /* clang-format on */
        return ((int)p_view->barcode_letter <= 'Z') &&
               (p_view->barcode_action <= (unsigned int)BUDDY1_ACTION_UTURN);
    case BUDDY1_TOPIC_HUMP:
        p_view->hump_count     = (uint16_t)read_bits(p_body, p_bit, 16);
        p_view->hump_height_mm = (uint16_t)read_bits(p_body, p_bit, 9);
        p_view->hump_valid     = 1;
        return p_view->hump_height_mm <= BUDDY1_HUMP_HEIGHT_MAX_MM;
    case BUDDY1_TOPIC_HEADING:
        p_view->heading_deg   = (int16_t)read_bits(p_body, p_bit, 9);
        p_view->heading_valid = 1;
        return p_view->heading_deg <= 359;
    case BUDDY1_TOPIC_OBSTACLE:
        p_view->obstacle_distance_mm = (uint16_t)read_bits(p_body, p_bit, 12);
        /* clang-format off */
        p_view->obstacle_width_mm = (uint16_t)read_bits(p_body, p_bit, 12);
        p_view->obstacle_route    =
            (telemetry_codec_route_t)read_bits(p_body, p_bit, 2);
        p_view->obstacle_valid    = 1;
        /* clang-format on */
        return ((p_view->obstacle_distance_mm >= BUDDY1_OBSTACLE_MIN_MM) &&
                (p_view->obstacle_distance_mm <= BUDDY1_OBSTACLE_MAX_MM)) &&
               (p_view->obstacle_route <= (unsigned int)BUDDY1_ROUTE_RIGHT);
    case BUDDY1_TOPIC_STATE:
        p_view->robot_state =
            (telemetry_codec_state_t)(read_bits(p_body, p_bit, 3) +
                                      (unsigned int)BUDDY1_STATE_STOPPED);
        p_view->robot_state_valid = 1;
        return 1;
    case BUDDY1_TOPIC_FAULTS:
        p_view->faults = (uint8_t)read_bits(p_body, p_bit, BUDDY1_FAULT_BITS);
        p_view->faults_valid = 1;
        return 1;
    case BUDDY1_TOPIC_POSE:
        p_view->pose_distance_cm =
            (uint16_t)read_bits(p_body, p_bit, BUDDY1_POSE_DISTANCE_BITS);
        p_view->pose_heading_deg = (int16_t)read_bits(p_body, p_bit, 9);
        p_view->pose_valid       = 1;
        return p_view->pose_heading_deg <= 359;
    case BUDDY1_TOPIC_TERRAIN:
    {
        telemetry_codec_terrain_t * p_terrain = &p_view->terrain;
        p_terrain->tilt_valid = (uint8_t)read_bits(p_body, p_bit, 1);
        const uint32_t pitch  = read_bits(p_body, p_bit, BUDDY1_TILT_BITS);
        const uint32_t roll   = read_bits(p_body, p_bit, BUDDY1_TILT_BITS);
        /* Eleven-bit tilt codes fit int32_t before sign decoding. The signed
         * result spans -1024 through 1023 and fits int16_t. */
        /* clang-format off */
        p_terrain->pitch_tenths_deg =
            (int16_t)((pitch & 0x400U) ? (int32_t)pitch - 0x800
                                       : (int32_t)pitch);
        p_terrain->roll_tenths_deg  =
            (int16_t)((roll & 0x400U) ? (int32_t)roll - 0x800 : (int32_t)roll);
        /* clang-format on */

        p_terrain->state      = (uint8_t)read_bits(p_body, p_bit, 3);
        p_terrain->on_hump    = (uint8_t)read_bits(p_body, p_bit, 1);
        p_terrain->climb_mm   = (uint16_t)read_bits(p_body, p_bit, 9);
        p_terrain->peak_valid = (uint8_t)read_bits(p_body, p_bit, 1);
        p_terrain->peak_mm    = (uint16_t)read_bits(p_body, p_bit, 9);
        p_terrain->turn_valid = (uint8_t)read_bits(p_body, p_bit, 1);
        const uint32_t turn   = read_bits(p_body, p_bit, BUDDY1_TURN_BITS);
        /* Ten-bit turn codes fit int32_t before sign decoding. The signed
         * result spans -512 through 511 and fits int16_t. */
        /* clang-format off */
        p_terrain->turn_dps   =
            (int16_t)((turn & 0x200U) ? (int32_t)turn - 0x400 : (int32_t)turn);
        p_terrain->safety     = (uint8_t)read_bits(p_body, p_bit, 2);
        p_view->terrain_valid = 1;
        /* clang-format on */

        if ((((p_terrain->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM) ||
              (p_terrain->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM)) ||
             ((!p_terrain->on_hump) && (p_terrain->climb_mm))) ||
            ((!p_terrain->peak_valid) && (p_terrain->peak_mm)))
        {
            return 0;
        }

        if ((p_terrain->safety > BUDDY1_SAFETY_OVERTURNED) ||
            (p_terrain->turn_valid
                 ? ((p_terrain->turn_dps < -BUDDY1_TURN_LIMIT_DPS) ||
                    (p_terrain->turn_dps > BUDDY1_TURN_LIMIT_DPS))
                 : p_terrain->turn_dps != 0))
        {
            return 0;
        }

        if (!p_terrain->tilt_valid)
        {
            return (0 == p_terrain->pitch_tenths_deg) &&
                   (0 == p_terrain->roll_tenths_deg);
        }

        return (((p_terrain->pitch_tenths_deg >= -BUDDY1_TILT_LIMIT_X10) &&
                 (p_terrain->pitch_tenths_deg <= BUDDY1_TILT_LIMIT_X10)) &&
                (p_terrain->roll_tenths_deg >= -BUDDY1_TILT_LIMIT_X10)) &&
               (p_terrain->roll_tenths_deg <= BUDDY1_TILT_LIMIT_X10);
    }
    case BUDDY1_TOPIC_AVOIDANCE:
    {
        telemetry_codec_avoidance_t * p_avoidance = &p_view->avoidance;
        p_avoidance->range_valid = (uint8_t)read_bits(p_body, p_bit, 1);
        p_avoidance->no_echo     = (uint8_t)read_bits(p_body, p_bit, 1);
        p_avoidance->range_mm    = (uint16_t)read_bits(p_body, p_bit, 12);
        p_avoidance->servo_deg   = (uint8_t)read_bits(p_body, p_bit, 8);
        p_avoidance->state       = (uint8_t)read_bits(p_body, p_bit, 3);
        p_avoidance->fault       = (uint8_t)read_bits(p_body, p_bit, 3);
        p_avoidance->route       = (uint8_t)read_bits(p_body, p_bit, 2);
        p_avoidance->leg         = (uint8_t)read_bits(p_body, p_bit, 4);
        p_avoidance->pass        = (uint8_t)read_bits(p_body, p_bit, 3);
        p_avoidance->swapped     = (uint8_t)read_bits(p_body, p_bit, 1);
        p_view->avoidance_valid  = 1;
        return (((((p_avoidance->range_mm <= BUDDY1_OBSTACLE_MAX_MM) &&
                   (p_avoidance->range_valid
                        ? ((p_avoidance->range_mm >= BUDDY1_OBSTACLE_MIN_MM) &&
                           (!p_avoidance->no_echo))
                        : !p_avoidance->range_mm)) &&
                  (p_avoidance->servo_deg <= BUDDY1_SERVO_MAX_DEG)) &&
                 (p_avoidance->state <= BUDDY1_S5_STATE_MAX)) &&
                ((int)p_avoidance->route <= BUDDY1_ROUTE_RIGHT)) &&
               (p_avoidance->pass <= BUDDY1_S5_PASS_MAX);
    }
    case BUDDY1_TOPIC_SCAN:
    {
        telemetry_codec_scan_t * p_scan = &p_view->scan;
        /* Twelve-bit millimetre fields fit uint16_t. Bias removal gives an
         * edge coordinate from -4000 to 4191 mm, within int16_t even before
         * the accepted upper bound of 4000 mm is checked. */
        /* clang-format off */
        p_scan->valid          = (uint8_t)read_bits(p_body, p_bit, 1);
        p_scan->scan_id        = (uint8_t)read_bits(p_body, p_bit, 8);
        p_scan->closest_mm     = (uint16_t)read_bits(p_body, p_bit, 12);
        p_scan->width_mm       = (uint16_t)read_bits(p_body, p_bit, 12);
        p_scan->left_clear_mm  = (uint16_t)read_bits(p_body, p_bit, 12);
        p_scan->right_clear_mm = (uint16_t)read_bits(p_body, p_bit, 12);
        p_scan->forward_mm     = (uint16_t)read_bits(p_body, p_bit, 12);
        p_scan->left_edge_mm   =
            (int16_t)((int32_t)read_bits(p_body, p_bit, 13) -
                     BUDDY1_SCAN_EDGE_MAX_MM);
        p_scan->right_edge_mm  =
            (int16_t)((int32_t)read_bits(p_body, p_bit, 13) -
                      BUDDY1_SCAN_EDGE_MAX_MM);
        p_view->scan_valid     = 1;
        /* clang-format on */

        if ((((((p_scan->closest_mm > BUDDY1_OBSTACLE_MAX_MM) ||
                (p_scan->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
               (p_scan->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
              (p_scan->forward_mm > BUDDY1_OBSTACLE_MAX_MM)) ||
             (p_scan->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM)) ||
            (p_scan->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM))
        {
            return 0;
        }

        return p_scan->valid
                   ? ((p_scan->closest_mm >= BUDDY1_OBSTACLE_MIN_MM) &&
                      (p_scan->left_edge_mm >= p_scan->right_edge_mm))
                   : !(((((((p_scan->closest_mm) || (p_scan->width_mm)) ||
                           (p_scan->left_clear_mm)) ||
                          (p_scan->right_clear_mm)) ||
                         (p_scan->forward_mm)) ||
                        (p_scan->left_edge_mm)) ||
                       (p_scan->right_edge_mm));
    }
    case BUDDY1_TOPIC_SCAN_SAMPLE:
    {
        telemetry_codec_scan_sample_t * p_sample = &p_view->scan_sample;
        /* The stored count plus one is 1..16. The angle mapping is 30..185
         * degrees before validation, so the cast preserves rejected values
         * as well as the supported 30..150-degree values. */
        /* clang-format off */
        p_sample->scan_id         = (uint8_t)read_bits(p_body, p_bit, 8);
        p_sample->peek            = (uint8_t)read_bits(p_body, p_bit, 1);
        p_sample->index           = (uint8_t)read_bits(p_body, p_bit, 4);
        p_sample->count           = (uint8_t)(read_bits(p_body, p_bit, 4) + 1U);
        p_sample->angle_deg       =
            (uint8_t)(30U + (5U * read_bits(p_body, p_bit, 5)));
        p_sample->valid           = (uint8_t)read_bits(p_body, p_bit, 1);
        p_sample->distance_mm     = (uint16_t)read_bits(p_body, p_bit, 12);
        p_view->scan_sample_valid = 1;
        /* clang-format on */
        return ((p_sample->index < p_sample->count) &&
                (p_sample->angle_deg <= 150U)) &&
               (p_sample->valid
                    ? ((p_sample->distance_mm >= BUDDY1_OBSTACLE_MIN_MM) &&
                       (p_sample->distance_mm <= BUDDY1_OBSTACLE_MAX_MM))
                    : 0U == p_sample->distance_mm);
    }
    default:
        return 0;
    }
}

/* End of file. */

