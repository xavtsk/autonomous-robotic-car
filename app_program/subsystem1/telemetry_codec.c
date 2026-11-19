#include "telemetry_codec.h"

/* Field widths exclude the one-byte protocol version. MQTT topic selects the
 * layout; no redundant field names or message type are sent in the payload. */
_Static_assert(BUDDY1_FAULT_MASK == (1U << BUDDY1_FAULT_BITS) - 1U,
               "fault mask and declared width must agree");
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
    1U + 2U * BUDDY1_TILT_BITS + 3U + 1U + 9U + 1U + 9U + 1U +
        BUDDY1_TURN_BITS + 2U,
    1U + 1U + 12U + 8U + 3U + 3U + 2U + 4U + 3U + 1U,
    1U + 8U + 5U * 12U + 2U * 13U,
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
_Static_assert(BUDDY1_TOPIC_COUNT <= 16U, "outbox topic selector is four bits");

static void put (uint8_t * p_bytes, unsigned * p_bit, uint32_t value,
                 unsigned width)
{
    unsigned value_i;
    for (value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        if (value_i < 32U && (value & ((uint32_t)1U << value_i)) != 0U)
        {
            p_bytes[*p_bit / 8U] |= (uint8_t)(1U << (*p_bit % 8U));
        }
    }
}

static uint32_t get (const uint8_t * p_bytes, unsigned * p_bit, unsigned width)
{
    uint32_t value = 0;
    unsigned value_i;
    for (value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        if ((p_bytes[*p_bit / 8U] & (1U << (*p_bit % 8U))) != 0U)
        {
            value |= (uint32_t)1U << value_i;
        }
    }
    return value;
}

unsigned buddy1_payload_size (uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? 1U + (g_body_bits[topic] + 7U) / 8U
                                      : 0U;
}

unsigned buddy1_payload_bits (uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? g_body_bits[topic] : 0U;
}

const char * buddy1_topic_name (uint8_t topic)
{
    return topic < BUDDY1_TOPIC_COUNT ? g_topics[topic] : (const char *)0;
}

static unsigned validity (const buddy1_snapshot_t * p_v)
{
    return (unsigned)((p_v->motion_valid != 0U) |
                      ((p_v->barcode_valid != 0U) << 1) |
                      ((p_v->hump_valid != 0U) << 2) |
                      ((p_v->heading_valid != 0U) << 3) |
                      ((p_v->obstacle_valid != 0U) << 4) |
                      ((p_v->robot_state_valid != 0U) << 5) |
                      ((p_v->faults_valid != 0U) << 6));
}

static int write_fields (uint8_t topic, const buddy1_snapshot_t * p_v,
                         uint8_t * p_p, unsigned * p_bit)
{
    int32_t left, right;
    switch (topic)
    {
    case BUDDY1_TOPIC_MOTION:
        left = p_v->left_mms;
        right = p_v->right_mms;
        if (left < -BUDDY1_SPEED_MAX_MMS || left > BUDDY1_SPEED_MAX_MMS ||
            right < -BUDDY1_SPEED_MAX_MMS || right > BUDDY1_SPEED_MAX_MMS)
        {
            return 0;
        }
        /* Nearest 2 mm/s, half-way cases away from zero: error <= 1 mm/s.
         * Conversion affects telemetry only, never Buddy 2's control values. */
        put(p_p, p_bit, (uint32_t)((left < 0 ? -left : left) + 1) / 2U, 8);
        put(p_p, p_bit, (uint32_t)(left < 0), 1);
        put(p_p, p_bit, (uint32_t)((right < 0 ? -right : right) + 1) / 2U, 8);
        put(p_p, p_bit, (uint32_t)(right < 0), 1);
        return 1;
    case BUDDY1_TOPIC_BARCODE:
        if (p_v->barcode_letter < 'A' || p_v->barcode_letter > 'Z' ||
            p_v->barcode_action < BUDDY1_ACTION_UNKNOWN ||
            p_v->barcode_action > BUDDY1_ACTION_UTURN)
        {
            return 0;
        }
        put(p_p, p_bit, (uint32_t)(p_v->barcode_letter - 'A'), 5);
        put(p_p, p_bit, (uint32_t)p_v->barcode_action, 3);
        return 1;
    case BUDDY1_TOPIC_HUMP:
        if (p_v->hump_height_mm > BUDDY1_HUMP_HEIGHT_MAX_MM)
        {
            return 0;
        }
        put(p_p, p_bit, p_v->hump_count, 16);
        put(p_p, p_bit, p_v->hump_height_mm, 9);
        return 1;
    case BUDDY1_TOPIC_HEADING:
        if (p_v->heading_deg < 0 || p_v->heading_deg > 359)
        {
            return 0;
        }
        put(p_p, p_bit, (uint32_t)p_v->heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_OBSTACLE:
        if (p_v->obstacle_distance_mm < BUDDY1_OBSTACLE_MIN_MM ||
            p_v->obstacle_distance_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_v->obstacle_width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM ||
            p_v->obstacle_route < BUDDY1_ROUTE_UNKNOWN ||
            p_v->obstacle_route > BUDDY1_ROUTE_RIGHT)
        {
            return 0;
        }
        put(p_p, p_bit, p_v->obstacle_distance_mm, 12);
        put(p_p, p_bit, p_v->obstacle_width_mm, 12);
        put(p_p, p_bit, (uint32_t)p_v->obstacle_route, 2);
        return 1;
    case BUDDY1_TOPIC_STATE:
        if (p_v->robot_state < BUDDY1_STATE_STOPPED ||
            p_v->robot_state > BUDDY1_STATE_ERROR)
        {
            return 0;
        }
        put(p_p, p_bit, (uint32_t)(p_v->robot_state - BUDDY1_STATE_STOPPED), 3);
        return 1;
    case BUDDY1_TOPIC_FAULTS:
        if ((p_v->faults & ~BUDDY1_FAULT_MASK) != 0U)
        {
            return 0;
        }
        put(p_p, p_bit, p_v->faults, BUDDY1_FAULT_BITS);
        return 1;
    case BUDDY1_TOPIC_POSE:
        if (p_v->pose_distance_cm >= BUDDY1_POSE_DISTANCE_MOD ||
            p_v->pose_heading_deg < 0 || p_v->pose_heading_deg > 359)
        {
            return 0;
        }
        put(p_p, p_bit, p_v->pose_distance_cm, BUDDY1_POSE_DISTANCE_BITS);
        put(p_p, p_bit, (uint32_t)p_v->pose_heading_deg, 9);
        return 1;
    case BUDDY1_TOPIC_TERRAIN:
    {
        const buddy1_terrain_t * p_t = &p_v->terrain;
        if (p_t->state >= BUDDY1_TERRAIN_STATES ||
            p_t->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM ||
            p_t->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM)
        {
            return 0;
        }
        if (p_t->tilt_valid)
        {
            if (p_t->pitch_x10 < -BUDDY1_TILT_LIMIT_X10 ||
                p_t->pitch_x10 > BUDDY1_TILT_LIMIT_X10 ||
                p_t->roll_x10 < -BUDDY1_TILT_LIMIT_X10 ||
                p_t->roll_x10 > BUDDY1_TILT_LIMIT_X10)
            {
                return 0;
            }
        }
        else if (p_t->pitch_x10 != 0 || p_t->roll_x10 != 0)
        {
            return 0; /* canonical zero */
        }
        if ((!p_t->on_hump && p_t->climb_mm != 0U) ||
            (!p_t->peak_valid && p_t->peak_mm != 0U))
        {
            return 0;
        }
        if (p_t->turn_valid ? (p_t->turn_dps < -BUDDY1_TURN_LIMIT_DPS ||
                               p_t->turn_dps > BUDDY1_TURN_LIMIT_DPS)
                            : p_t->turn_dps != 0)
        {
            return 0;
        }
        if (p_t->safety > BUDDY1_SAFETY_OVERTURNED)
        {
            return 0;
        }
        put(p_p, p_bit, p_t->tilt_valid != 0U, 1);
        /* Two's complement in 11 bits. */
        put(p_p, p_bit, (uint32_t)(int32_t)p_t->pitch_x10 & 0x7FFU,
            BUDDY1_TILT_BITS);
        put(p_p, p_bit, (uint32_t)(int32_t)p_t->roll_x10 & 0x7FFU,
            BUDDY1_TILT_BITS);
        put(p_p, p_bit, p_t->state, 3);
        put(p_p, p_bit, p_t->on_hump != 0U, 1);
        put(p_p, p_bit, p_t->climb_mm, 9);
        put(p_p, p_bit, p_t->peak_valid != 0U, 1);
        put(p_p, p_bit, p_t->peak_mm, 9);
        put(p_p, p_bit, p_t->turn_valid != 0U, 1);
        put(p_p, p_bit, (uint32_t)(int32_t)p_t->turn_dps & 0x3FFU,
            BUDDY1_TURN_BITS);
        put(p_p, p_bit, p_t->safety, 2);
        return 1;
    }
    case BUDDY1_TOPIC_AVOIDANCE:
    {
        const buddy1_avoidance_t * p_a = &p_v->avoidance;
        if (p_a->range_mm > BUDDY1_OBSTACLE_MAX_MM ||
            (p_a->range_valid &&
             (p_a->range_mm < BUDDY1_OBSTACLE_MIN_MM || p_a->no_echo)) ||
            (!p_a->range_valid && p_a->range_mm) ||
            p_a->servo_deg > BUDDY1_SERVO_MAX_DEG ||
            p_a->state > BUDDY1_S5_STATE_MAX || p_a->fault > 7U ||
            p_a->route > BUDDY1_ROUTE_RIGHT || p_a->leg > 15U ||
            p_a->pass > BUDDY1_S5_PASS_MAX || p_a->swapped > 1U ||
            p_a->range_valid > 1U || p_a->no_echo > 1U)
        {
            return 0;
        }
        put(p_p, p_bit, p_a->range_valid, 1);
        put(p_p, p_bit, p_a->no_echo, 1);
        put(p_p, p_bit, p_a->range_mm, 12);
        put(p_p, p_bit, p_a->servo_deg, 8);
        put(p_p, p_bit, p_a->state, 3);
        put(p_p, p_bit, p_a->fault, 3);
        put(p_p, p_bit, p_a->route, 2);
        put(p_p, p_bit, p_a->leg, 4);
        put(p_p, p_bit, p_a->pass, 3);
        put(p_p, p_bit, p_a->swapped, 1);
        return 1;
    }
    case BUDDY1_TOPIC_SCAN:
    {
        const buddy1_scan_t * p_q = &p_v->scan;
        if (p_q->closest_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->width_mm > BUDDY1_OBSTACLE_WIDTH_MAX_MM ||
            p_q->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->forward_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->left_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM ||
            p_q->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM ||
            p_q->right_edge_mm < -BUDDY1_SCAN_EDGE_MAX_MM ||
            p_q->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM || p_q->valid > 1U)
        {
            return 0;
        }
        if (p_q->valid
                ? (p_q->closest_mm < BUDDY1_OBSTACLE_MIN_MM ||
                   p_q->left_edge_mm < p_q->right_edge_mm)
                : (p_q->closest_mm || p_q->width_mm || p_q->left_clear_mm ||
                   p_q->right_clear_mm || p_q->forward_mm ||
                   p_q->left_edge_mm || p_q->right_edge_mm))
        {
            return 0;
        }
        put(p_p, p_bit, p_q->valid, 1);
        put(p_p, p_bit, p_q->scan_id, 8);
        put(p_p, p_bit, p_q->closest_mm, 12);
        put(p_p, p_bit, p_q->width_mm, 12);
        put(p_p, p_bit, p_q->left_clear_mm, 12);
        put(p_p, p_bit, p_q->right_clear_mm, 12);
        put(p_p, p_bit, p_q->forward_mm, 12);
        put(p_p, p_bit, (uint32_t)(p_q->left_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM),
            13);
        put(p_p, p_bit,
            (uint32_t)(p_q->right_edge_mm + BUDDY1_SCAN_EDGE_MAX_MM), 13);
        return 1;
    }
    case BUDDY1_TOPIC_SCAN_SAMPLE:
    {
        const buddy1_scan_sample_t * p_s = &p_v->scan_sample;
        if (p_s->peek > 1U || p_s->valid > 1U || p_s->count < 1U ||
            p_s->count > BUDDY1_SCAN_SAMPLES_MAX || p_s->index >= p_s->count ||
            p_s->angle_deg < 30U || p_s->angle_deg > 150U ||
            (p_s->angle_deg - 30U) % 5U != 0U ||
            (p_s->valid ? (p_s->distance_mm < BUDDY1_OBSTACLE_MIN_MM ||
                           p_s->distance_mm > BUDDY1_OBSTACLE_MAX_MM)
                        : p_s->distance_mm != 0U))
        {
            return 0;
        }
        put(p_p, p_bit, p_s->scan_id, 8);
        put(p_p, p_bit, p_s->peek, 1);
        put(p_p, p_bit, p_s->index, 4);
        put(p_p, p_bit, (uint32_t)p_s->count - 1U, 4);
        put(p_p, p_bit, (uint32_t)(p_s->angle_deg - 30U) / 5U, 5);
        put(p_p, p_bit, p_s->valid, 1);
        put(p_p, p_bit, p_s->distance_mm, 12);
        return 1;
    }
    default:
        return 0;
    }
}

static int read_fields (uint8_t topic, const uint8_t * p_p, unsigned * p_bit,
                        buddy1_snapshot_t * p_v)
{
    uint32_t mag, negative;
    switch (topic)
    {
    case BUDDY1_TOPIC_MOTION:
        mag = get(p_p, p_bit, 8);
        negative = get(p_p, p_bit, 1);
        if (mag == 0U && negative != 0U)
        {
            return 0; /* no negative zero */
        }
        p_v->left_mms = (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));
        mag = get(p_p, p_bit, 8);
        negative = get(p_p, p_bit, 1);
        if (mag == 0U && negative != 0U)
        {
            return 0;
        }
        p_v->right_mms = (int16_t)((int32_t)(mag * 2U) * (negative ? -1 : 1));
        p_v->motion_valid = 1;
        return 1;
    case BUDDY1_TOPIC_BARCODE:
        p_v->barcode_letter = (uint8_t)('A' + get(p_p, p_bit, 5));
        p_v->barcode_action = (buddy1_barcode_action_t)get(p_p, p_bit, 3);
        p_v->barcode_valid = 1;
        return p_v->barcode_letter <= 'Z' &&
               p_v->barcode_action <= BUDDY1_ACTION_UTURN;
    case BUDDY1_TOPIC_HUMP:
        p_v->hump_count = (uint16_t)get(p_p, p_bit, 16);
        p_v->hump_height_mm = (uint16_t)get(p_p, p_bit, 9);
        p_v->hump_valid = 1;
        return p_v->hump_height_mm <= BUDDY1_HUMP_HEIGHT_MAX_MM;
    case BUDDY1_TOPIC_HEADING:
        p_v->heading_deg = (int16_t)get(p_p, p_bit, 9);
        p_v->heading_valid = 1;
        return p_v->heading_deg <= 359;
    case BUDDY1_TOPIC_OBSTACLE:
        p_v->obstacle_distance_mm = (uint16_t)get(p_p, p_bit, 12);
        p_v->obstacle_width_mm = (uint16_t)get(p_p, p_bit, 12);
        p_v->obstacle_route = (buddy1_obstacle_route_t)get(p_p, p_bit, 2);
        p_v->obstacle_valid = 1;
        return p_v->obstacle_distance_mm >= BUDDY1_OBSTACLE_MIN_MM &&
               p_v->obstacle_distance_mm <= BUDDY1_OBSTACLE_MAX_MM &&
               p_v->obstacle_route <= BUDDY1_ROUTE_RIGHT;
    case BUDDY1_TOPIC_STATE:
        p_v->robot_state =
            (buddy1_robot_state_t)(get(p_p, p_bit, 3) + BUDDY1_STATE_STOPPED);
        p_v->robot_state_valid = 1;
        return 1;
    case BUDDY1_TOPIC_FAULTS:
        p_v->faults = (uint8_t)get(p_p, p_bit, BUDDY1_FAULT_BITS);
        p_v->faults_valid = 1;
        return 1;
    case BUDDY1_TOPIC_POSE:
        p_v->pose_distance_cm =
            (uint16_t)get(p_p, p_bit, BUDDY1_POSE_DISTANCE_BITS);
        p_v->pose_heading_deg = (int16_t)get(p_p, p_bit, 9);
        p_v->pose_valid = 1;
        return p_v->pose_heading_deg <= 359;
    case BUDDY1_TOPIC_TERRAIN:
    {
        buddy1_terrain_t * p_t = &p_v->terrain;
        uint32_t pitch, roll, turn;
        p_t->tilt_valid = (uint8_t)get(p_p, p_bit, 1);
        pitch = get(p_p, p_bit, BUDDY1_TILT_BITS);
        roll = get(p_p, p_bit, BUDDY1_TILT_BITS);
        p_t->pitch_x10 = (int16_t)((pitch & 0x400U) ? (int32_t)pitch - 0x800
                                                    : (int32_t)pitch);
        p_t->roll_x10 =
            (int16_t)((roll & 0x400U) ? (int32_t)roll - 0x800 : (int32_t)roll);
        p_t->state = (uint8_t)get(p_p, p_bit, 3);
        p_t->on_hump = (uint8_t)get(p_p, p_bit, 1);
        p_t->climb_mm = (uint16_t)get(p_p, p_bit, 9);
        p_t->peak_valid = (uint8_t)get(p_p, p_bit, 1);
        p_t->peak_mm = (uint16_t)get(p_p, p_bit, 9);
        p_t->turn_valid = (uint8_t)get(p_p, p_bit, 1);
        turn = get(p_p, p_bit, BUDDY1_TURN_BITS);
        p_t->turn_dps =
            (int16_t)((turn & 0x200U) ? (int32_t)turn - 0x400 : (int32_t)turn);
        p_t->safety = (uint8_t)get(p_p, p_bit, 2);
        p_v->terrain_valid = 1;
        if (p_t->climb_mm > BUDDY1_HUMP_HEIGHT_MAX_MM ||
            p_t->peak_mm > BUDDY1_HUMP_HEIGHT_MAX_MM ||
            (!p_t->on_hump && p_t->climb_mm) ||
            (!p_t->peak_valid && p_t->peak_mm))
        {
            return 0;
        }
        if (p_t->safety > BUDDY1_SAFETY_OVERTURNED ||
            (p_t->turn_valid ? (p_t->turn_dps < -BUDDY1_TURN_LIMIT_DPS ||
                                p_t->turn_dps > BUDDY1_TURN_LIMIT_DPS)
                             : p_t->turn_dps != 0))
        {
            return 0;
        }
        if (!p_t->tilt_valid)
        {
            return p_t->pitch_x10 == 0 && p_t->roll_x10 == 0;
        }
        return p_t->pitch_x10 >= -BUDDY1_TILT_LIMIT_X10 &&
               p_t->pitch_x10 <= BUDDY1_TILT_LIMIT_X10 &&
               p_t->roll_x10 >= -BUDDY1_TILT_LIMIT_X10 &&
               p_t->roll_x10 <= BUDDY1_TILT_LIMIT_X10;
    }
    case BUDDY1_TOPIC_AVOIDANCE:
    {
        buddy1_avoidance_t * p_a = &p_v->avoidance;
        p_a->range_valid = (uint8_t)get(p_p, p_bit, 1);
        p_a->no_echo = (uint8_t)get(p_p, p_bit, 1);
        p_a->range_mm = (uint16_t)get(p_p, p_bit, 12);
        p_a->servo_deg = (uint8_t)get(p_p, p_bit, 8);
        p_a->state = (uint8_t)get(p_p, p_bit, 3);
        p_a->fault = (uint8_t)get(p_p, p_bit, 3);
        p_a->route = (uint8_t)get(p_p, p_bit, 2);
        p_a->leg = (uint8_t)get(p_p, p_bit, 4);
        p_a->pass = (uint8_t)get(p_p, p_bit, 3);
        p_a->swapped = (uint8_t)get(p_p, p_bit, 1);
        p_v->avoidance_valid = 1;
        return p_a->range_mm <= BUDDY1_OBSTACLE_MAX_MM &&
               (p_a->range_valid
                    ? (p_a->range_mm >= BUDDY1_OBSTACLE_MIN_MM && !p_a->no_echo)
                    : !p_a->range_mm) &&
               p_a->servo_deg <= BUDDY1_SERVO_MAX_DEG &&
               p_a->state <= BUDDY1_S5_STATE_MAX &&
               p_a->route <= BUDDY1_ROUTE_RIGHT &&
               p_a->pass <= BUDDY1_S5_PASS_MAX;
    }
    case BUDDY1_TOPIC_SCAN:
    {
        buddy1_scan_t * p_q = &p_v->scan;
        p_q->valid = (uint8_t)get(p_p, p_bit, 1);
        p_q->scan_id = (uint8_t)get(p_p, p_bit, 8);
        p_q->closest_mm = (uint16_t)get(p_p, p_bit, 12);
        p_q->width_mm = (uint16_t)get(p_p, p_bit, 12);
        p_q->left_clear_mm = (uint16_t)get(p_p, p_bit, 12);
        p_q->right_clear_mm = (uint16_t)get(p_p, p_bit, 12);
        p_q->forward_mm = (uint16_t)get(p_p, p_bit, 12);
        p_q->left_edge_mm =
            (int16_t)((int32_t)get(p_p, p_bit, 13) - BUDDY1_SCAN_EDGE_MAX_MM);
        p_q->right_edge_mm =
            (int16_t)((int32_t)get(p_p, p_bit, 13) - BUDDY1_SCAN_EDGE_MAX_MM);
        p_v->scan_valid = 1;
        if (p_q->closest_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->left_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->right_clear_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->forward_mm > BUDDY1_OBSTACLE_MAX_MM ||
            p_q->left_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM ||
            p_q->right_edge_mm > BUDDY1_SCAN_EDGE_MAX_MM)
        {
            return 0;
        }
        return p_q->valid
                   ? (p_q->closest_mm >= BUDDY1_OBSTACLE_MIN_MM &&
                      p_q->left_edge_mm >= p_q->right_edge_mm)
                   : !(p_q->closest_mm || p_q->width_mm || p_q->left_clear_mm ||
                       p_q->right_clear_mm || p_q->forward_mm ||
                       p_q->left_edge_mm || p_q->right_edge_mm);
    }
    case BUDDY1_TOPIC_SCAN_SAMPLE:
    {
        buddy1_scan_sample_t * p_s = &p_v->scan_sample;
        p_s->scan_id = (uint8_t)get(p_p, p_bit, 8);
        p_s->peek = (uint8_t)get(p_p, p_bit, 1);
        p_s->index = (uint8_t)get(p_p, p_bit, 4);
        p_s->count = (uint8_t)(get(p_p, p_bit, 4) + 1U);
        p_s->angle_deg = (uint8_t)(30U + 5U * get(p_p, p_bit, 5));
        p_s->valid = (uint8_t)get(p_p, p_bit, 1);
        p_s->distance_mm = (uint16_t)get(p_p, p_bit, 12);
        p_v->scan_sample_valid = 1;
        return p_s->index < p_s->count && p_s->angle_deg <= 150U &&
               (p_s->valid ? (p_s->distance_mm >= BUDDY1_OBSTACLE_MIN_MM &&
                              p_s->distance_mm <= BUDDY1_OBSTACLE_MAX_MM)
                           : p_s->distance_mm == 0U);
    }
    default:
        return 0;
    }
}

unsigned buddy1_encode (uint8_t topic, const buddy1_snapshot_t * p_v,
                        uint8_t * p_packet, unsigned capacity)
{
    unsigned bit = 0, value_i, flags, length = buddy1_payload_size(topic);
    if (!p_v || !p_packet || length == 0U || capacity < length ||
        topic == BUDDY1_TOPIC_MAP_REPLY)
    {
        return 0;
    }
    for (value_i = 0; value_i < length; ++value_i)
    {
        p_packet[value_i] = 0;
    }
    p_packet[0] = BUDDY1_WIRE_VERSION;
    if (topic == BUDDY1_TOPIC_SNAPSHOT)
    {
        flags = validity(p_v);
        put(p_packet + 1, &bit, flags, 7);
        /* Invalid sections have canonical zero bits, regardless of stale data
         * in the caller's temporary view. Zero height remains a valid value. */
        for (value_i = 0; value_i <= BUDDY1_TOPIC_FAULTS; ++value_i)
        {
            if ((flags & (1U << value_i)) != 0U)
            {
                if (!write_fields((uint8_t)value_i, p_v, p_packet + 1, &bit))
                {
                    return 0;
                }
            }
            else
            {
                put(p_packet + 1, &bit, 0, g_body_bits[value_i]);
            }
        }
        /* 19 Nov 2026: distance/heading since START appended after the
         * original seven sections (bits 102-123), with its own validity bit. */
        put(p_packet + 1, &bit, p_v->pose_valid != 0U, 1);
        if (p_v->pose_valid)
        {
            if (!write_fields(BUDDY1_TOPIC_POSE, p_v, p_packet + 1, &bit))
            {
                return 0;
            }
        }
        else
        {
            put(p_packet + 1, &bit, 0, g_body_bits[BUDDY1_TOPIC_POSE]);
        }
    }
    else if (topic == BUDDY1_TOPIC_STATUS)
    {
        if (p_v->faults_valid && (p_v->faults & ~BUDDY1_FAULT_MASK) != 0U)
        {
            return 0;
        }
        put(p_packet + 1, &bit, 1, 1); /* heartbeat: sender is alive */
        put(p_packet + 1, &bit, p_v->faults_valid != 0U, 1);
        put(p_packet + 1, &bit, p_v->faults_valid ? p_v->faults : 0U,
            BUDDY1_FAULT_BITS);
    }
    else if (!write_fields(topic, p_v, p_packet + 1, &bit))
    {
        return 0;
    }
    return length;
}

int buddy1_decode (uint8_t topic, const uint8_t * p_packet, unsigned length,
                   buddy1_snapshot_t * p_out)
{
    buddy1_snapshot_t value_v = {0};
    unsigned bit = 0, value_i, flags, remainder;
    if (!p_packet || !p_out || topic >= BUDDY1_TOPIC_COUNT ||
        topic == BUDDY1_TOPIC_MAP_REPLY ||
        length != buddy1_payload_size(topic) ||
        p_packet[0] != BUDDY1_WIRE_VERSION)
    {
        return 0;
    }
    remainder = g_body_bits[topic] % 8U;
    if (remainder && (p_packet[length - 1U] >> remainder) != 0U)
    {
        return 0;
    }
    if (topic == BUDDY1_TOPIC_SNAPSHOT)
    {
        flags = (unsigned)get(p_packet + 1, &bit, 7);
        for (value_i = 0; value_i <= BUDDY1_TOPIC_FAULTS; ++value_i)
        {
            if ((flags & (1U << value_i)) != 0U)
            {
                if (!read_fields((uint8_t)value_i, p_packet + 1, &bit,
                                 &value_v))
                {
                    return 0;
                }
            }
            else
            {
                /* At most 34 bits, so consume one bit at a time. */
                unsigned value_n;
                for (value_n = 0; value_n < g_body_bits[value_i]; ++value_n)
                {
                    if (get(p_packet + 1, &bit, 1) != 0U)
                    {
                        return 0;
                    }
                }
            }
        }
        if (get(p_packet + 1, &bit, 1) != 0U)
        {
            if (!read_fields(BUDDY1_TOPIC_POSE, p_packet + 1, &bit, &value_v))
            {
                return 0;
            }
        }
        else
        {
            unsigned value_n;
            for (value_n = 0; value_n < g_body_bits[BUDDY1_TOPIC_POSE];
                 ++value_n)
            {
                if (get(p_packet + 1, &bit, 1) != 0U)
                {
                    return 0;
                }
            }
        }
    }
    else if (topic == BUDDY1_TOPIC_STATUS)
    {
        if (get(p_packet + 1, &bit, 1) != 1U)
        {
            return 0;
        }
        value_v.faults_valid = (uint8_t)get(p_packet + 1, &bit, 1);
        value_v.faults = (uint8_t)get(p_packet + 1, &bit, BUDDY1_FAULT_BITS);
        if (!value_v.faults_valid && value_v.faults)
        {
            return 0;
        }
    }
    else if (!read_fields(topic, p_packet + 1, &bit, &value_v))
    {
        return 0;
    }
    *p_out = value_v;
    return 1;
}

int buddy1_payload_valid (uint8_t topic, const uint8_t * p_packet,
                          unsigned length)
{
    buddy1_snapshot_t decoded;
    if (topic == BUDDY1_TOPIC_MAP_REPLY)
    {
        return p_packet && length == 2U && p_packet[0] == BUDDY1_WIRE_VERSION &&
               p_packet[1] <= BUDDY1_REPLY_INTERNAL;
    }
    return buddy1_decode(topic, p_packet, length, &decoded);
}

int buddy1_map_pack (const buddy1_barcode_map_t * p_map,
                     uint8_t p_packet[BUDDY1_MAP_BYTES])
{
    uint8_t letters[4];
    unsigned value_i, value_j, bit = 0;
    if (!p_map || !p_packet)
    {
        return 0;
    }
    letters[0] = p_map->forward;
    letters[1] = p_map->left;
    letters[2] = p_map->right;
    letters[3] = p_map->uturn;
    for (value_i = 0; value_i < 4; ++value_i)
    {
        if (letters[value_i] < 'A' || letters[value_i] > 'Z')
        {
            return 0;
        }
        for (value_j = 0; value_j < value_i; ++value_j)
        {
            if (letters[value_i] == letters[value_j])
            {
                return 0;
            }
        }
    }
    p_packet[0] = BUDDY1_WIRE_VERSION;
    p_packet[1] = p_packet[2] = p_packet[3] = 0;
    for (value_i = 0; value_i < 4; ++value_i)
    {
        put(p_packet + 1, &bit, (uint32_t)(letters[value_i] - 'A'), 5);
    }
    return 1;
}

int buddy1_map_unpack (const uint8_t * p_packet, unsigned length,
                       buddy1_barcode_map_t * p_out)
{
    buddy1_barcode_map_t map;
    uint8_t canonical[BUDDY1_MAP_BYTES];
    unsigned bit = 0;
    if (!p_packet || !p_out || length != BUDDY1_MAP_BYTES ||
        p_packet[0] != BUDDY1_WIRE_VERSION || (p_packet[3] & 0xf0U) != 0U)
    {
        return 0;
    }
    map.forward = (uint8_t)('A' + get(p_packet + 1, &bit, 5));
    map.left = (uint8_t)('A' + get(p_packet + 1, &bit, 5));
    map.right = (uint8_t)('A' + get(p_packet + 1, &bit, 5));
    map.uturn = (uint8_t)('A' + get(p_packet + 1, &bit, 5));
    if (!buddy1_map_pack(&map, canonical))
    {
        return 0;
    }
    *p_out = map;
    return 1;
}
