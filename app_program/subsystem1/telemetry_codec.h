/* Tribunal 1 application wire format. No RTOS, pointers or C struct layouts
 * cross the wire. See MQTT_PROTOCOL.md for bit order and field ranges. */
#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>

#define BUDDY1_WIRE_VERSION       3U
#define BUDDY1_PAYLOAD_MAX       14U /* version + 102-bit snapshot */
#define BUDDY1_HUMP_HEIGHT_MAX_MM 500U
#define BUDDY1_OBSTACLE_MIN_MM    30U
#define BUDDY1_OBSTACLE_MAX_MM    4000U
#define BUDDY1_OBSTACLE_WIDTH_MAX_MM 4095U
#define BUDDY1_MAP_BYTES          4U
#define BUDDY1_SPEED_MAX_MMS    510
#define BUDDY1_FAULT_STALL       (1U << 0)
#define BUDDY1_FAULT_ENCODER     (1U << 1)
#define BUDDY1_FAULT_IMU         (1U << 2)
#define BUDDY1_FAULT_NO_ECHO     (1U << 3)
#define BUDDY1_FAULT_WIFI        (1U << 4)
#define BUDDY1_FAULT_IR          (1U << 5)
#define BUDDY1_FAULT_BITS        6U
#define BUDDY1_FAULT_MASK        0x3fU
/* Route pose (live only, not in the snapshot): distance since START in cm,
 * sent modulo 4096 and unwrapped by the laptop; heading since START. */
#define BUDDY1_POSE_DISTANCE_BITS 12U
#define BUDDY1_POSE_DISTANCE_MOD  (1U << BUDDY1_POSE_DISTANCE_BITS)

enum {
    BUDDY1_TOPIC_MOTION = 0, BUDDY1_TOPIC_BARCODE, BUDDY1_TOPIC_HUMP,
    BUDDY1_TOPIC_HEADING, BUDDY1_TOPIC_OBSTACLE, BUDDY1_TOPIC_STATE,
    BUDDY1_TOPIC_FAULTS, BUDDY1_TOPIC_SNAPSHOT, BUDDY1_TOPIC_MAP_REPLY,
    BUDDY1_TOPIC_STATUS, BUDDY1_TOPIC_POSE, BUDDY1_TOPIC_COUNT
};
typedef enum {
    BUDDY1_ACTION_UNKNOWN = 0, BUDDY1_ACTION_FORWARD, BUDDY1_ACTION_LEFT,
    BUDDY1_ACTION_RIGHT, BUDDY1_ACTION_UTURN
} BUDDY1_BARCODE_ACTION;
typedef enum {
    BUDDY1_ROUTE_UNKNOWN = 0, BUDDY1_ROUTE_LEFT, BUDDY1_ROUTE_RIGHT
} BUDDY1_OBSTACLE_ROUTE;
typedef enum {
    BUDDY1_STATE_UNKNOWN = 0, BUDDY1_STATE_STOPPED,
    BUDDY1_STATE_LINE_FOLLOWING, BUDDY1_STATE_TURNING_LEFT,
    BUDDY1_STATE_TURNING_RIGHT, BUDDY1_STATE_UTURN,
    BUDDY1_STATE_AVOIDING_OBSTACLE, BUDDY1_STATE_COMPLETE, BUDDY1_STATE_ERROR
} BUDDY1_ROBOT_STATE;
enum {
    BUDDY1_REPLY_OK = 0, BUDDY1_REPLY_INVALID, BUDDY1_REPLY_BUSY,
    BUDDY1_REPLY_INTERNAL
};
typedef struct {
    uint8_t forward, left, right, uturn; /* decoded A-Z; never sent raw */
} BUDDY1_BARCODE_MAP;

/* Temporary decoded view for callers/calculations. The shared cache is a
 * packed 14-byte packet, protected by the Buddy 1 mutex. */
typedef struct {
    uint8_t motion_valid;
    int16_t left_mms, right_mms;
    uint8_t barcode_valid, barcode_letter;
    BUDDY1_BARCODE_ACTION barcode_action;
    uint8_t hump_valid;
    uint16_t hump_count, hump_height_mm;
    uint8_t heading_valid;
    int16_t heading_deg;
    uint8_t obstacle_valid;
    uint16_t obstacle_distance_mm, obstacle_width_mm;
    BUDDY1_OBSTACLE_ROUTE obstacle_route;
    uint8_t robot_state_valid;
    BUDDY1_ROBOT_STATE robot_state;
    uint8_t faults_valid, faults;
    /* Live-only pose; never part of the snapshot packet. */
    uint8_t pose_valid;
    uint16_t pose_distance_cm;  /* 0..4095, distance since START modulo 4096 */
    int16_t pose_heading_deg;   /* 0..359, + = anticlockwise since START */
} BUDDY1_SNAPSHOT;

unsigned buddy1_payload_size(uint8_t topic);
unsigned buddy1_payload_bits(uint8_t topic);
const char *buddy1_topic_name(uint8_t topic);
/* Return packet length, or zero for invalid input/capacity. Byte 0 is version;
 * body bits are numbered from the least significant bit of byte 1. */
unsigned buddy1_encode(uint8_t topic, const BUDDY1_SNAPSHOT *value,
                       uint8_t *packet, unsigned capacity);
int buddy1_decode(uint8_t topic, const uint8_t *packet, unsigned length,
                  BUDDY1_SNAPSHOT *out);
int buddy1_payload_valid(uint8_t topic, const uint8_t *packet, unsigned length);
int buddy1_map_pack(const BUDDY1_BARCODE_MAP *map, uint8_t packet[BUDDY1_MAP_BYTES]);
int buddy1_map_unpack(const uint8_t *packet, unsigned length, BUDDY1_BARCODE_MAP *out);

#endif
