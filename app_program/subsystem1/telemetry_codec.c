#include "telemetry_codec.h"

/* Field widths exclude the one-byte protocol version. MQTT topic selects the
 * layout; no redundant field names or message type are sent in the payload. */
_Static_assert(BUDDY1_FAULT_MASK == (1U << BUDDY1_FAULT_BITS) - 1U,
               "fault mask and declared width must agree");
static const uint8_t body_bits[BUDDY1_TOPIC_COUNT] = {
    18, 8, 25, 9, 26, 3, BUDDY1_FAULT_BITS,
    96U + BUDDY1_FAULT_BITS, 2, 2U + BUDDY1_FAULT_BITS,
    BUDDY1_POSE_DISTANCE_BITS + 9U,
    1U + 2U * BUDDY1_TILT_BITS + 3U + 1U + 9U + 1U + 9U,
    1U + 1U + 12U + 8U + 3U + 3U + 2U + 4U + 3U + 1U,
    1U + 8U + 5U * 12U + 2U * 13U
};
static const char *const topics[BUDDY1_TOPIC_COUNT] = {
    "car/telemetry/motion", "car/telemetry/barcode", "car/telemetry/hump",
    "car/telemetry/heading", "car/telemetry/obstacle", "car/telemetry/state",
    "car/telemetry/faults", "car/reply/snapshot", "car/reply/barcode_map", "car/status",
    "car/telemetry/pose", "car/telemetry/terrain", "car/telemetry/avoidance",
    "car/telemetry/scan"
};
_Static_assert(BUDDY1_TOPIC_COUNT <= 16U, "outbox topic selector is four bits");

static void put(uint8_t *bytes, unsigned *bit, uint32_t value, unsigned width)
{
    unsigned i;
    for (i = 0; i < width; ++i, ++*bit) {
        if (i < 32U && (value & ((uint32_t)1U << i)) != 0U)
            bytes[*bit / 8U] |= (uint8_t)(1U << (*bit % 8U));
    }
}

static uint32_t get(const uint8_t *bytes, unsigned *bit, unsigned width)
{
    uint32_t value = 0;
    unsigned i;
    for (i = 0; i < width; ++i, ++*bit) {
        if ((bytes[*bit / 8U] & (1U << (*bit % 8U))) != 0U)
            value |= (uint32_t)1U << i;
    }
    return value;
}

unsigned buddy1_payload_size(uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? 1U + (body_bits[topic] + 7U) / 8U : 0U;
}

unsigned buddy1_payload_bits(uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? body_bits[topic] : 0U;
}

const char *buddy1_topic_name(uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? topics[topic] : (const char *)0;
}

static unsigned validity(const BUDDY1_SNAPSHOT *v)
{
    return (unsigned)((v->motion_valid != 0U) | ((v->barcode_valid != 0U) << 1) |
           ((v->hump_valid != 0U) << 2) | ((v->heading_valid != 0U) << 3) |
           ((v->obstacle_valid != 0U) << 4) | ((v->robot_state_valid != 0U) << 5) |
           ((v->faults_valid != 0U) << 6));
}

static int write_fields(uint8_t topic, const BUDDY1_SNAPSHOT *v,
                        uint8_t *p, unsigned *bit)
{
    int32_t left, right;
    switch (topic) {
    case BUDDY1_TOPIC_MOTION:
        left = v->left_mms; right = v->right_mms;
        if (left < -BUDDY1_SPEED_MAX_MMS || left > BUDDY1_SPEED_MAX_MMS ||
            right < -BUDDY1_SPEED_MAX_MMS || right > BUDDY1_SPEED_MAX_MMS) return 0;
        /* Nearest 2 mm/s, half-way cases away from zero: error <= 1 mm/s.
         * Conversion affects telemetry only, never Buddy 2's control values. */
        put(p, bit, (uint32_t)((left < 0 ? -left : left) + 1) / 2U, 8);
        put(p, bit, (uint32_t)(left < 0), 1);
        put(p, bit, (uint32_t)((right < 0 ? -right : right) + 1) / 2U, 8);
        put(p, bit, (uint32_t)(right < 0), 1);
        return 1;
    case BUDDY1_TOPIC_BARCODE:
        if (v->barcode_letter < 'A' || v->barcode_letter > 'Z' ||
            v->barcode_action < BUDDY1_ACTION_UNKNOWN ||
            v->barcode_action > BUDDY1_ACTION_UTURN) return 0;
        put(p, bit, (uint32_t)(v->barcode_letter - 'A'), 5);
        put(p, bit, (uint32_t)v->barcode_action, 3);
        return 1;
    case BUDDY1_TOPIC_HUMP:
        if (v->hump_height_mm > BUDDY1_HUMP_HEIGHT_MAX_MM) return 0;
        put(p, bit, v->hump_count, 16); put(p, bit, v->hump_height_mm, 9);
        return 1;
    case BUDDY1_TOPIC_HEADING:
        if (v->heading_deg < 0 || v->heading_deg > 359) return 0;
        put(p, bit, (uint32_t)v->heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_OBSTACLE:
        if (v->obstacle_distance_mm < BUDDY1_OBSTACLE_MIN_MM ||
            v->obstacle_distance_mm > BUDDY1_OBSTACLE_MAX_MM ||
            v->obstacle_width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM ||
            v->obstacle_route < BUDDY1_ROUTE_UNKNOWN ||
            v->obstacle_route > BUDDY1_ROUTE_RIGHT) return 0;
        put(p, bit, v->obstacle_distance_mm, 12);
        put(p, bit, v->obstacle_width_mm, 12);
        put(p, bit, (uint32_t)v->obstacle_route, 2);
        return 1;
    case BUDDY1_TOPIC_STATE:
        if (v->robot_state < BUDDY1_STATE_STOPPED ||
            v->robot_state > BUDDY1_STATE_ERROR) return 0;
        put(p, bit, (uint32_t)(v->robot_state - BUDDY1_STATE_STOPPED), 3);
        return 1;
    case BUDDY1_TOPIC_FAULTS:
        if ((v->faults & ~BUDDY1_FAULT_MASK) != 0U) return 0;
        put(p, bit, v->faults, BUDDY1_FAULT_BITS);
        return 1;
    case BUDDY1_TOPIC_POSE:
        if (v->pose_distance_cm >= BUDDY1_POSE_DISTANCE_MOD ||
            v->pose_heading_deg < 0 || v->pose_heading_deg > 359) return 0;
        put(p, bit, v->pose_distance_cm, BUDDY1_POSE_DISTANCE_BITS);
        put(p, bit, (uint32_t)v->pose_heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_TERRAIN: {
        const BUDDY1_TERRAIN *t = &v->terrain;
        if (t->state >= BUDDY1_TERRAIN_STATES || t->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM ||
            t->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM) return 0;
        if (t->tilt_valid) {
            if (t->pitch_x10 < -BUDDY1_TILT_LIMIT_X10 || t->pitch_x10 > BUDDY1_TILT_LIMIT_X10 ||
                t->roll_x10 < -BUDDY1_TILT_LIMIT_X10 || t->roll_x10 > BUDDY1_TILT_LIMIT_X10) return 0;
        } else if (t->pitch_x10 != 0 || t->roll_x10 != 0) return 0; /* canonical zero */
        if ((!t->on_hump && t->climb_mm != 0U) || (!t->peak_valid && t->peak_mm != 0U)) return 0;
        put(p, bit, t->tilt_valid != 0U, 1);
        /* Two's complement in 11 bits. */
        put(p, bit, (uint32_t)(int32_t)t->pitch_x10 & 0x7FFU, BUDDY1_TILT_BITS);
        put(p, bit, (uint32_t)(int32_t)t->roll_x10 & 0x7FFU, BUDDY1_TILT_BITS);
        put(p, bit, t->state, 3);
        put(p, bit, t->on_hump != 0U, 1); put(p, bit, t->climb_mm, 9);
        put(p, bit, t->peak_valid != 0U, 1); put(p, bit, t->peak_mm, 9);
        return 1;
    }
    case BUDDY1_TOPIC_AVOIDANCE: {
        const BUDDY1_AVOIDANCE *a = &v->avoidance;
        if (a->range_mm > BUDDY1_OBSTACLE_MAX_MM ||
            (a->range_valid && (a->range_mm < BUDDY1_OBSTACLE_MIN_MM || a->no_echo)) ||
            (!a->range_valid && a->range_mm) || a->servo_deg > BUDDY1_SERVO_MAX_DEG ||
            a->state > BUDDY1_S5_STATE_MAX || a->fault > 7U || a->route > BUDDY1_ROUTE_RIGHT ||
            a->leg > 15U || a->pass > BUDDY1_S5_PASS_MAX || a->swapped > 1U ||
            a->range_valid > 1U || a->no_echo > 1U) return 0;
        put(p, bit, a->range_valid, 1); put(p, bit, a->no_echo, 1); put(p, bit, a->range_mm, 12);
        put(p, bit, a->servo_deg, 8); put(p, bit, a->state, 3); put(p, bit, a->fault, 3);
        put(p, bit, a->route, 2); put(p, bit, a->leg, 4); put(p, bit, a->pass, 3);
        put(p, bit, a->swapped, 1);
        return 1;
    }
    case BUDDY1_TOPIC_SCAN: {
        const BUDDY1_SCAN *q = &v->scan;
        if (q->closest_mm > BUDDY1_OBSTACLE_MAX_MM || q->width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM ||
            q->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM || q->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            q->forward_mm > BUDDY1_OBSTACLE_MAX_MM ||
            q->left_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM || q->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM ||
            q->right_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM || q->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM ||
            q->valid > 1U) return 0;
        if (q->valid ? (q->closest_mm < BUDDY1_OBSTACLE_MIN_MM || q->left_edge_mm < q->right_edge_mm)
                     : (q->closest_mm || q->width_mm || q->left_clear_mm || q->right_clear_mm ||
                        q->forward_mm || q->left_edge_mm || q->right_edge_mm)) return 0;
        put(p, bit, q->valid, 1); put(p, bit, q->scan_id, 8);
        put(p, bit, q->closest_mm, 12); put(p, bit, q->width_mm, 12);
        put(p, bit, q->left_clear_mm, 12); put(p, bit, q->right_clear_mm, 12);
        put(p, bit, q->forward_mm, 12);
        put(p, bit, (uint32_t)(q->left_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM), 13);
        put(p, bit, (uint32_t)(q->right_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM), 13);
        return 1;
    }
    default: return 0;
    }
}

static int read_fields(uint8_t topic, const uint8_t *p, unsigned *bit,
                       BUDDY1_SNAPSHOT *v)
{
    uint32_t mag, negative;
    switch (topic) {
    case BUDDY1_TOPIC_MOTION:
        mag = get(p, bit, 8); negative = get(p, bit, 1);
        if (mag == 0U && negative != 0U) return 0; /* no negative zero */
        v->left_mms = (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));
        mag = get(p, bit, 8); negative = get(p, bit, 1);
        if (mag == 0U && negative != 0U) return 0;
        v->right_mms = (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));
        v->motion_valid = 1; return 1;
    case BUDDY1_TOPIC_BARCODE:
        v->barcode_letter = (uint8_t)('A' + get(p, bit, 5));
        v->barcode_action = (BUDDY1_BARCODE_ACTION)get(p, bit, 3);
        v->barcode_valid = 1;
        return v->barcode_letter <= 'Z' && v->barcode_action <= BUDDY1_ACTION_UTURN;
    case BUDDY1_TOPIC_HUMP:
        v->hump_count = (uint16_t)get(p, bit, 16);
        v->hump_height_mm = (uint16_t)get(p, bit, 9);
        v->hump_valid = 1; return v->hump_height_mm <= BUDDY1_HUMP_HEIGHT_MAX_MM;
    case BUDDY1_TOPIC_HEADING:
        v->heading_deg = (int16_t)get(p, bit, 9);
        v->heading_valid = 1; return v->heading_deg <= 359;
    case BUDDY1_TOPIC_OBSTACLE:
        v->obstacle_distance_mm = (uint16_t)get(p, bit, 12);
        v->obstacle_width_mm = (uint16_t)get(p, bit, 12);
        v->obstacle_route = (BUDDY1_OBSTACLE_ROUTE)get(p, bit, 2);
        v->obstacle_valid = 1;
        return v->obstacle_distance_mm >= BUDDY1_OBSTACLE_MIN_MM &&
               v->obstacle_distance_mm <= BUDDY1_OBSTACLE_MAX_MM &&
               v->obstacle_route <= BUDDY1_ROUTE_RIGHT;
    case BUDDY1_TOPIC_STATE:
        v->robot_state = (BUDDY1_ROBOT_STATE)(get(p, bit, 3) + BUDDY1_STATE_STOPPED);
        v->robot_state_valid = 1; return 1;
    case BUDDY1_TOPIC_FAULTS:
        v->faults = (uint8_t)get(p, bit, BUDDY1_FAULT_BITS);
        v->faults_valid = 1; return 1;
    case BUDDY1_TOPIC_POSE:
        v->pose_distance_cm = (uint16_t)get(p, bit, BUDDY1_POSE_DISTANCE_BITS);
        v->pose_heading_deg = (int16_t)get(p, bit, 9);
        v->pose_valid = 1; return v->pose_heading_deg <= 359;
    case BUDDY1_TOPIC_TERRAIN: {
        BUDDY1_TERRAIN *t = &v->terrain;
        uint32_t pitch, roll;
        t->tilt_valid = (uint8_t)get(p, bit, 1);
        pitch = get(p, bit, BUDDY1_TILT_BITS); roll = get(p, bit, BUDDY1_TILT_BITS);
        t->pitch_x10 = (int16_t)((pitch & 0x400U) ? (int32_t)pitch - 0x800 : (int32_t)pitch);
        t->roll_x10 = (int16_t)((roll & 0x400U) ? (int32_t)roll - 0x800 : (int32_t)roll);
        t->state = (uint8_t)get(p, bit, 3);
        t->on_hump = (uint8_t)get(p, bit, 1); t->climb_mm = (uint16_t)get(p, bit, 9);
        t->peak_valid = (uint8_t)get(p, bit, 1); t->peak_mm = (uint16_t)get(p, bit, 9);
        v->terrain_valid = 1;
        if (t->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM || t->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM ||
            (!t->on_hump && t->climb_mm) || (!t->peak_valid && t->peak_mm)) return 0;
        if (!t->tilt_valid) return t->pitch_x10 == 0 && t->roll_x10 == 0;
        return t->pitch_x10 >= -BUDDY1_TILT_LIMIT_X10 && t->pitch_x10 <= BUDDY1_TILT_LIMIT_X10 &&
               t->roll_x10 >= -BUDDY1_TILT_LIMIT_X10 && t->roll_x10 <= BUDDY1_TILT_LIMIT_X10;
    }
    case BUDDY1_TOPIC_AVOIDANCE: {
        BUDDY1_AVOIDANCE *a = &v->avoidance;
        a->range_valid = (uint8_t)get(p, bit, 1); a->no_echo = (uint8_t)get(p, bit, 1);
        a->range_mm = (uint16_t)get(p, bit, 12); a->servo_deg = (uint8_t)get(p, bit, 8);
        a->state = (uint8_t)get(p, bit, 3); a->fault = (uint8_t)get(p, bit, 3);
        a->route = (uint8_t)get(p, bit, 2); a->leg = (uint8_t)get(p, bit, 4);
        a->pass = (uint8_t)get(p, bit, 3); a->swapped = (uint8_t)get(p, bit, 1);
        v->avoidance_valid = 1;
        return a->range_mm <= BUDDY1_OBSTACLE_MAX_MM &&
               (a->range_valid ? (a->range_mm >= BUDDY1_OBSTACLE_MIN_MM && !a->no_echo) : !a->range_mm) &&
               a->servo_deg <= BUDDY1_SERVO_MAX_DEG && a->state <= BUDDY1_S5_STATE_MAX &&
               a->route <= BUDDY1_ROUTE_RIGHT && a->pass <= BUDDY1_S5_PASS_MAX;
    }
    case BUDDY1_TOPIC_SCAN: {
        BUDDY1_SCAN *q = &v->scan;
        q->valid = (uint8_t)get(p, bit, 1); q->scan_id = (uint8_t)get(p, bit, 8);
        q->closest_mm = (uint16_t)get(p, bit, 12); q->width_mm = (uint16_t)get(p, bit, 12);
        q->left_clear_mm = (uint16_t)get(p, bit, 12); q->right_clear_mm = (uint16_t)get(p, bit, 12);
        q->forward_mm = (uint16_t)get(p, bit, 12);
        q->left_edge_mm = (int16_t)((int32_t)get(p, bit, 13) - BUDDY1_SCAN_EDGE_MAX_MM);
        q->right_edge_mm = (int16_t)((int32_t)get(p, bit, 13) - BUDDY1_SCAN_EDGE_MAX_MM);
        v->scan_valid = 1;
        if (q->closest_mm > BUDDY1_OBSTACLE_MAX_MM || q->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            q->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM || q->forward_mm > BUDDY1_OBSTACLE_MAX_MM ||
            q->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM || q->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM) return 0;
        return q->valid ? (q->closest_mm >= BUDDY1_OBSTACLE_MIN_MM && q->left_edge_mm >= q->right_edge_mm)
                        : !(q->closest_mm || q->width_mm || q->left_clear_mm || q->right_clear_mm ||
                            q->forward_mm || q->left_edge_mm || q->right_edge_mm);
    }
    default: return 0;
    }
}

unsigned buddy1_encode(uint8_t topic, const BUDDY1_SNAPSHOT *v,
                       uint8_t *packet, unsigned capacity)
{
    unsigned bit = 0, i, flags, length = buddy1_payload_size(topic);
    if (!v || !packet || length == 0U || capacity < length ||
        topic == BUDDY1_TOPIC_MAP_REPLY) return 0;
    for (i = 0; i < length; ++i) packet[i] = 0;
    packet[0] = BUDDY1_WIRE_VERSION;
    if (topic == BUDDY1_TOPIC_SNAPSHOT) {
        flags = validity(v);
        put(packet + 1, &bit, flags, 7);
        /* Invalid sections have canonical zero bits, regardless of stale data
         * in the caller's temporary view. Zero height remains a valid value. */
        for (i = 0; i <= BUDDY1_TOPIC_FAULTS; ++i) {
            if ((flags & (1U << i)) != 0U) {
                if (!write_fields((uint8_t)i, v, packet + 1, &bit)) return 0;
            } else {
                put(packet + 1, &bit, 0, body_bits[i]);
            }
        }
    } else if (topic == BUDDY1_TOPIC_STATUS) {
        if (v->faults_valid && (v->faults & ~BUDDY1_FAULT_MASK) != 0U) return 0;
        put(packet + 1, &bit, 1, 1); /* heartbeat: sender is alive */
        put(packet + 1, &bit, v->faults_valid != 0U, 1);
        put(packet + 1, &bit, v->faults_valid ? v->faults : 0U, BUDDY1_FAULT_BITS);
    } else if (!write_fields(topic, v, packet + 1, &bit)) return 0;
    return length;
}

int buddy1_decode(uint8_t topic, const uint8_t *packet, unsigned length,
                  BUDDY1_SNAPSHOT *out)
{
    BUDDY1_SNAPSHOT v = {0};
    unsigned bit = 0, i, flags, remainder;
    if (!packet || !out || topic >= BUDDY1_TOPIC_COUNT ||
        topic == BUDDY1_TOPIC_MAP_REPLY || length != buddy1_payload_size(topic) ||
        packet[0] != BUDDY1_WIRE_VERSION) return 0;
    remainder = body_bits[topic] % 8U;
    if (remainder && (packet[length - 1U] >> remainder) != 0U) return 0;
    if (topic == BUDDY1_TOPIC_SNAPSHOT) {
        flags = (unsigned)get(packet + 1, &bit, 7);
        for (i = 0; i <= BUDDY1_TOPIC_FAULTS; ++i) {
            if ((flags & (1U << i)) != 0U) {
                if (!read_fields((uint8_t)i, packet + 1, &bit, &v)) return 0;
            } else {
                /* At most 34 bits, so consume one bit at a time. */
                unsigned n;
                for (n = 0; n < body_bits[i]; ++n)
                    if (get(packet + 1, &bit, 1) != 0U) return 0;
            }
        }
    } else if (topic == BUDDY1_TOPIC_STATUS) {
        if (get(packet + 1, &bit, 1) != 1U) return 0;
        v.faults_valid = (uint8_t)get(packet + 1, &bit, 1);
        v.faults = (uint8_t)get(packet + 1, &bit, BUDDY1_FAULT_BITS);
        if (!v.faults_valid && v.faults) return 0;
    } else if (!read_fields(topic, packet + 1, &bit, &v)) return 0;
    *out = v;
    return 1;
}

int buddy1_payload_valid(uint8_t topic, const uint8_t *packet, unsigned length)
{
    BUDDY1_SNAPSHOT decoded;
    if (topic == BUDDY1_TOPIC_MAP_REPLY)
        return packet && length == 2U && packet[0] == BUDDY1_WIRE_VERSION &&
               packet[1] <= BUDDY1_REPLY_INTERNAL;
    return buddy1_decode(topic, packet, length, &decoded);
}

int buddy1_map_pack(const BUDDY1_BARCODE_MAP *map, uint8_t packet[BUDDY1_MAP_BYTES])
{
    uint8_t letters[4];
    unsigned i, j, bit = 0;
    if (!map || !packet) return 0;
    letters[0] = map->forward; letters[1] = map->left;
    letters[2] = map->right; letters[3] = map->uturn;
    for (i = 0; i < 4; ++i) {
        if (letters[i] < 'A' || letters[i] > 'Z') return 0;
        for (j = 0; j < i; ++j) if (letters[i] == letters[j]) return 0;
    }
    packet[0] = BUDDY1_WIRE_VERSION;
    packet[1] = packet[2] = packet[3] = 0;
    for (i = 0; i < 4; ++i) put(packet + 1, &bit, (uint32_t)(letters[i] - 'A'), 5);
    return 1;
}

int buddy1_map_unpack(const uint8_t *packet, unsigned length, BUDDY1_BARCODE_MAP *out)
{
    BUDDY1_BARCODE_MAP map;
    uint8_t canonical[BUDDY1_MAP_BYTES];
    unsigned bit = 0;
    if (!packet || !out || length != BUDDY1_MAP_BYTES ||
        packet[0] != BUDDY1_WIRE_VERSION || (packet[3] & 0xf0U) != 0U) return 0;
    map.forward = (uint8_t)('A' + get(packet + 1, &bit, 5));
    map.left = (uint8_t)('A' + get(packet + 1, &bit, 5));
    map.right = (uint8_t)('A' + get(packet + 1, &bit, 5));
    map.uturn = (uint8_t)('A' + get(packet + 1, &bit, 5));
    if (!buddy1_map_pack(&map, canonical)) return 0;
    *out = map;
    return 1;
}
